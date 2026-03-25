use std::{env, error::Error, fs};

use fluxkit_core::{
    ActuatorCompensationConfig, ActuatorEstimate, ActuatorLimits, ActuatorParams, ControlMode,
    CurrentLoopConfig, FastLoopInput, InverterParams, MotorController, MotorLimits, MotorParams,
    RotorEstimate, TickSchedule,
};
use fluxkit_math::{
    ContinuousMechanicalAngle,
    angle::mechanical_to_electrical,
    inverse_clarke, inverse_park,
    units::{Amps, Duty, Henries, Hertz, NewtonMeters, Ohms, RadPerSec, Volts, Webers},
};
use fluxkit_pmsm_sim::{ActuatorPlantParams, PmsmModel, PmsmParams};
use plotters::prelude::*;

const FAST_DT_SECONDS: f32 = 1.0 / 20_000.0;
const MEDIUM_DECIMATION: usize = 10;
const MEDIUM_DT_SECONDS: f32 = FAST_DT_SECONDS * MEDIUM_DECIMATION as f32;
const GEAR_RATIO: f32 = 2.0;

#[derive(Clone, Copy)]
struct Sample {
    time_seconds: f32,
    position_target: f32,
    position_unwrapped: f32,
    velocity: f32,
    iq_measured: f32,
}

fn main() -> Result<(), Box<dyn Error>> {
    let output_path = env::args()
        .nth(1)
        .unwrap_or_else(|| "target/plots/closed_loop_position.svg".to_owned());
    if let Some(parent) = std::path::Path::new(&output_path).parent() {
        fs::create_dir_all(parent)?;
    }

    let bus_voltage = Volts::new(24.0);
    let position_target = 1.5 * std::f32::consts::TAU;
    let mut controller = MotorController::new(
        motor_params(),
        inverter_params(),
        actuator_params(),
        config(),
    );
    let mut plant = PmsmModel::new_zeroed(plant_params())?;
    let mut samples = Vec::with_capacity(40_000);

    controller.set_mode(ControlMode::Position);
    controller.set_position_target(ContinuousMechanicalAngle::new(position_target));
    controller.enable();

    for step in 0..40_000 {
        let state = *plant.state();
        let mechanical_angle = state.mechanical_angle.wrapped();
        let output_angle =
            ContinuousMechanicalAngle::new(state.mechanical_angle.get() / GEAR_RATIO).wrapped();
        let electrical_angle =
            mechanical_to_electrical(mechanical_angle.into(), plant.params().pole_pairs as u32);
        let phase_currents = inverse_clarke(inverse_park(
            state.current_dq.map(|current| current.get()),
            electrical_angle.get(),
        ))
        .map(Amps::new);

        let schedule = if step % MEDIUM_DECIMATION == MEDIUM_DECIMATION - 1 {
            TickSchedule::with_medium(MEDIUM_DT_SECONDS)
        } else {
            TickSchedule::none()
        };

        let output = controller.tick(
            FastLoopInput {
                phase_currents,
                bus_voltage,
                rotor: RotorEstimate {
                    mechanical_angle: mechanical_angle.into(),
                    mechanical_velocity: state.mechanical_velocity,
                },
                actuator: ActuatorEstimate {
                    output_angle: output_angle.into(),
                    output_velocity: RadPerSec::new(state.mechanical_velocity.get() / GEAR_RATIO),
                },
                dt_seconds: FAST_DT_SECONDS,
            },
            schedule,
        );

        plant.step_phase_duty(
            output.phase_duty,
            bus_voltage,
            NewtonMeters::ZERO,
            FAST_DT_SECONDS,
        )?;

        let status = controller.status();
        samples.push(Sample {
            time_seconds: step as f32 * FAST_DT_SECONDS,
            position_target,
            position_unwrapped: status.last_unwrapped_output_mechanical_angle.get(),
            velocity: status.last_output_mechanical_velocity.get(),
            iq_measured: status.last_measured_idq.q.get(),
        });
    }

    draw_position_plot(&output_path, &samples)?;
    println!("wrote {output_path}");
    Ok(())
}

fn draw_position_plot(path: &str, samples: &[Sample]) -> Result<(), Box<dyn Error>> {
    let root = SVGBackend::new(path, (640, 960)).into_drawing_area();
    root.fill(&WHITE)?;

    let areas = root.split_evenly((3, 1));
    let end_time = samples
        .last()
        .map(|sample| sample.time_seconds)
        .unwrap_or(1.0);
    let position_max = samples.iter().fold(0.0_f32, |acc, sample| {
        acc.max(sample.position_target.max(sample.position_unwrapped))
    });
    let velocity_max = samples
        .iter()
        .fold(0.0_f32, |acc, sample| acc.max(sample.velocity.abs()));
    let iq_max = samples
        .iter()
        .fold(0.0_f32, |acc, sample| acc.max(sample.iq_measured.abs()));
    let velocity_span = (velocity_max * 1.1).max(1.0);
    let iq_span = (iq_max * 1.1).max(1.0);

    {
        let mut chart = ChartBuilder::on(&areas[0])
            .caption("Closed-Loop Position Response", ("sans-serif", 28))
            .margin(16)
            .x_label_area_size(40)
            .y_label_area_size(60)
            .build_cartesian_2d(0.0_f32..end_time, -0.2_f32..(position_max * 1.1).max(1.0))?;

        chart
            .configure_mesh()
            .disable_mesh()
            .x_desc("time (s)")
            .y_desc("mechanical angle (rad)")
            .draw()?;

        chart
            .draw_series(LineSeries::new(
                samples
                    .iter()
                    .map(|sample| (sample.time_seconds, sample.position_target)),
                &BLACK,
            ))?
            .label("target")
            .legend(|(x, y)| PathElement::new(vec![(x, y), (x + 20, y)], BLACK));

        chart
            .draw_series(LineSeries::new(
                samples
                    .iter()
                    .map(|sample| (sample.time_seconds, sample.position_unwrapped)),
                &BLUE,
            ))?
            .label("unwrapped position")
            .legend(|(x, y)| PathElement::new(vec![(x, y), (x + 20, y)], BLUE));

        chart
            .configure_series_labels()
            .background_style(WHITE)
            .border_style(BLACK)
            .draw()?;
    }

    {
        let mut chart = ChartBuilder::on(&areas[1])
            .caption("Mechanical Velocity", ("sans-serif", 24))
            .margin(16)
            .x_label_area_size(40)
            .y_label_area_size(60)
            .build_cartesian_2d(0.0_f32..end_time, -velocity_span..velocity_span)?;

        chart
            .configure_mesh()
            .disable_mesh()
            .x_desc("time (s)")
            .y_desc("omega_mech (rad/s)")
            .draw()?;

        chart.draw_series(LineSeries::new(
            samples
                .iter()
                .map(|sample| (sample.time_seconds, sample.velocity)),
            &RGBColor(0, 121, 140),
        ))?;
    }

    {
        let mut chart = ChartBuilder::on(&areas[2])
            .caption("Measured iq", ("sans-serif", 24))
            .margin(16)
            .x_label_area_size(40)
            .y_label_area_size(60)
            .build_cartesian_2d(0.0_f32..end_time, -iq_span..iq_span)?;

        chart
            .configure_mesh()
            .disable_mesh()
            .x_desc("time (s)")
            .y_desc("iq (A)")
            .draw()?;

        chart.draw_series(LineSeries::new(
            samples
                .iter()
                .map(|sample| (sample.time_seconds, sample.iq_measured)),
            &RED,
        ))?;
    }

    root.present()?;
    Ok(())
}

fn motor_params() -> MotorParams {
    MotorParams {
        pole_pairs: 7,
        phase_resistance_ohm: Ohms::new(0.12),
        d_inductance_h: Henries::new(0.000_03),
        q_inductance_h: Henries::new(0.000_03),
        flux_linkage_weber: Webers::new(0.005),
        electrical_angle_offset: fluxkit_math::ElectricalAngle::new(0.0),
        limits: MotorLimits {
            max_phase_current: Amps::new(10.0),
            max_mech_speed: Some(RadPerSec::new(150.0)),
        },
    }
}

fn inverter_params() -> InverterParams {
    InverterParams {
        pwm_frequency_hz: Hertz::new(20_000.0),
        min_duty: Duty::new(0.0),
        max_duty: Duty::new(1.0),
        min_bus_voltage: Volts::new(6.0),
        max_bus_voltage: Volts::new(60.0),
        max_voltage_command: Volts::new(24.0),
    }
}

fn config() -> CurrentLoopConfig {
    CurrentLoopConfig {
        kp_d: 0.2,
        ki_d: 400.0,
        kp_q: 0.2,
        ki_q: 400.0,
        velocity_kp: 0.2,
        velocity_ki: 8.0,
        position_kp: 12.0,
        position_ki: 0.0,
        max_voltage_mag: Volts::new(12.0),
        id_ref_default: Amps::ZERO,
        max_id_target: Amps::new(5.0),
        max_iq_target: Amps::new(8.0),
        max_velocity_target: RadPerSec::new(120.0),
        max_current_ref_derivative_amps_per_sec: 10_000.0,
        enable_current_feedforward: true,
    }
}

fn actuator_params() -> ActuatorParams {
    ActuatorParams {
        gear_ratio: GEAR_RATIO,
        compensation: ActuatorCompensationConfig::disabled(),
        limits: ActuatorLimits {
            max_output_velocity: Some(RadPerSec::new(30.0)),
            max_output_torque: Some(NewtonMeters::new(10.0)),
        },
    }
}

fn plant_params() -> PmsmParams {
    PmsmParams {
        pole_pairs: 7,
        phase_resistance_ohm: Ohms::new(0.12),
        d_inductance_h: Henries::new(0.000_03),
        q_inductance_h: Henries::new(0.000_03),
        flux_linkage_weber: Webers::new(0.005),
        actuator: ActuatorPlantParams {
            gear_ratio: GEAR_RATIO,
            output_inertia_kg_m2: 0.0008,
            positive_viscous_coefficient: 0.0002,
            negative_viscous_coefficient: 0.0002,
            ..ActuatorPlantParams::disabled()
        },
        max_voltage_mag: None,
    }
}
