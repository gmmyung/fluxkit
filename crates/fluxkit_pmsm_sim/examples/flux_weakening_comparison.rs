use std::{env, error::Error, fs};

use fluxkit_core::{
    ActuatorCompensationConfig, ActuatorEstimate, ActuatorLimits, ActuatorParams, ControlInput,
    CurrentLoopConfig, FluxWeakeningConfig, InverterParams, MotorController, MotorLimits,
    MotorParams, RotorEstimate, motor::ControllerCommand,
};
use fluxkit_math::{
    angle::mechanical_to_electrical,
    inverse_clarke, inverse_park,
    units::{Amps, Duty, Henries, Hertz, NewtonMeters, Ohms, RadPerSec, Volts, Webers},
};
use fluxkit_pmsm_sim::{ActuatorPlantParams, PmsmModel, PmsmParams, ThermalPlantParams};
use plotters::prelude::*;

const FAST_DT_SECONDS: f32 = 1.0 / 20_000.0;
const SIMULATION_STEPS: usize = 30_000;
const SAMPLE_STRIDE: usize = 10;
const TARGET_STEP_INDEX: usize = 1_000;
const TARGET_VELOCITY: f32 = 300.0;

#[derive(Clone, Copy)]
struct Sample {
    time_seconds: f32,
    target_velocity: f32,
    no_fw_velocity: f32,
    fw_velocity: f32,
    no_fw_voltage_utilization: f32,
    fw_voltage_utilization: f32,
    fw_id: f32,
}

fn main() -> Result<(), Box<dyn Error>> {
    let output_path = env::args()
        .nth(1)
        .unwrap_or_else(|| "target/plots/flux_weakening_comparison.svg".to_owned());
    if let Some(parent) = std::path::Path::new(&output_path).parent() {
        fs::create_dir_all(parent)?;
    }

    let bus_voltage = Volts::new(12.0);
    let mut no_fw_controller = MotorController::new(
        motor_params(),
        inverter_params(),
        actuator_params(),
        config(FluxWeakeningConfig::disabled()),
        fluxkit_math::Svpwm,
        fluxkit_core::PassThroughCurrentEstimator::new(),
    );
    let mut fw_controller = MotorController::new(
        motor_params(),
        inverter_params(),
        actuator_params(),
        config(FluxWeakeningConfig::enabled(
            0.86,
            RadPerSec::new(220.0),
            Amps::new(8.0),
            RadPerSec::new(900.0),
        )),
        fluxkit_math::Svpwm,
        fluxkit_core::PassThroughCurrentEstimator::new(),
    );
    let mut no_fw_plant = PmsmModel::new_zeroed(plant_params())?;
    let mut fw_plant = PmsmModel::new_zeroed(plant_params())?;
    let mut samples = Vec::with_capacity(SIMULATION_STEPS / SAMPLE_STRIDE + 1);

    for step in 0..SIMULATION_STEPS {
        let target_velocity = if step < TARGET_STEP_INDEX {
            0.0
        } else {
            TARGET_VELOCITY
        };
        let command = ControllerCommand::Velocity(RadPerSec::new(target_velocity));

        let no_fw_output = no_fw_controller.step(control_input(&no_fw_plant, bus_voltage, command));
        let fw_output = fw_controller.step(control_input(&fw_plant, bus_voltage, command));

        no_fw_plant.step_phase_duty(
            no_fw_output.phase_duty,
            bus_voltage,
            NewtonMeters::ZERO,
            FAST_DT_SECONDS,
        )?;
        fw_plant.step_phase_duty(
            fw_output.phase_duty,
            bus_voltage,
            NewtonMeters::ZERO,
            FAST_DT_SECONDS,
        )?;

        if step % SAMPLE_STRIDE == 0 || step + 1 == SIMULATION_STEPS {
            let no_fw_status = no_fw_controller.status();
            let fw_status = fw_controller.status();
            samples.push(Sample {
                time_seconds: step as f32 * FAST_DT_SECONDS,
                target_velocity,
                no_fw_velocity: no_fw_plant.state().mechanical_velocity.get(),
                fw_velocity: fw_plant.state().mechanical_velocity.get(),
                no_fw_voltage_utilization: no_fw_status.last_voltage_utilization,
                fw_voltage_utilization: fw_status.last_voltage_utilization,
                fw_id: fw_status.last_flux_weakening_id.get(),
            });
        }
    }

    draw_plot(&output_path, &samples)?;
    println!("wrote {output_path}");
    Ok(())
}

fn control_input(
    plant: &PmsmModel,
    bus_voltage: Volts,
    command: ControllerCommand,
) -> ControlInput {
    let state = *plant.state();
    let mechanical_angle = state.mechanical_angle.wrapped();
    let electrical_angle =
        mechanical_to_electrical(mechanical_angle.into(), plant.params().pole_pairs as u32);
    let phase_currents = inverse_clarke(inverse_park(
        state.current_dq.map(|current| current.get()),
        electrical_angle.get(),
    ))
    .map(Amps::new);

    ControlInput {
        command,
        armed: true,
        clear_fault_requested: false,
        phase_currents,
        bus_voltage,
        winding_temperature_c: state.winding_temperature_c,
        rotor: RotorEstimate {
            mechanical_angle: mechanical_angle.into(),
            mechanical_velocity: state.mechanical_velocity,
        },
        actuator: ActuatorEstimate {
            output_angle: mechanical_angle.into(),
            output_velocity: state.mechanical_velocity,
        },
        dt_seconds: FAST_DT_SECONDS,
    }
}

fn draw_plot(path: &str, samples: &[Sample]) -> Result<(), Box<dyn Error>> {
    let root = SVGBackend::new(path, (960, 720)).into_drawing_area();
    root.fill(&WHITE)?;

    let areas = root.split_evenly((3, 1));
    let end_time = samples
        .last()
        .map(|sample| sample.time_seconds)
        .unwrap_or(1.0);
    let velocity_max = samples.iter().fold(TARGET_VELOCITY, |acc, sample| {
        acc.max(sample.no_fw_velocity.abs())
            .max(sample.fw_velocity.abs())
            .max(sample.target_velocity.abs())
    });
    let utilization_max = samples.iter().fold(1.0_f32, |acc, sample| {
        acc.max(sample.no_fw_voltage_utilization)
            .max(sample.fw_voltage_utilization)
    });
    let id_min = samples
        .iter()
        .fold(0.0_f32, |acc, sample| acc.min(sample.fw_id));

    {
        let mut chart = ChartBuilder::on(&areas[0])
            .caption("Flux Weakening Speed Extension", ("sans-serif", 28))
            .margin(14)
            .x_label_area_size(36)
            .y_label_area_size(72)
            .build_cartesian_2d(0.0_f32..end_time, -10.0_f32..(velocity_max * 1.08))?;

        chart
            .configure_mesh()
            .disable_mesh()
            .x_desc("time (s)")
            .y_desc("mechanical velocity (rad/s)")
            .draw()?;

        chart
            .draw_series(LineSeries::new(
                samples
                    .iter()
                    .map(|sample| (sample.time_seconds, sample.target_velocity)),
                &BLACK,
            ))?
            .label("target")
            .legend(|(x, y)| PathElement::new(vec![(x, y), (x + 20, y)], BLACK));

        chart
            .draw_series(LineSeries::new(
                samples
                    .iter()
                    .map(|sample| (sample.time_seconds, sample.no_fw_velocity)),
                &RED,
            ))?
            .label("without flux weakening")
            .legend(|(x, y)| PathElement::new(vec![(x, y), (x + 20, y)], RED));

        chart
            .draw_series(LineSeries::new(
                samples
                    .iter()
                    .map(|sample| (sample.time_seconds, sample.fw_velocity)),
                &BLUE,
            ))?
            .label("with flux weakening")
            .legend(|(x, y)| PathElement::new(vec![(x, y), (x + 20, y)], BLUE));

        chart
            .configure_series_labels()
            .background_style(WHITE)
            .border_style(BLACK)
            .draw()?;
    }

    {
        let mut chart = ChartBuilder::on(&areas[1])
            .caption("Voltage Utilization", ("sans-serif", 22))
            .margin(14)
            .x_label_area_size(36)
            .y_label_area_size(72)
            .build_cartesian_2d(0.0_f32..end_time, 0.0_f32..(utilization_max * 1.08))?;

        chart
            .configure_mesh()
            .disable_mesh()
            .x_desc("time (s)")
            .y_desc("requested / limit")
            .draw()?;

        chart
            .draw_series(LineSeries::new(
                samples
                    .iter()
                    .map(|sample| (sample.time_seconds, sample.no_fw_voltage_utilization)),
                &RED,
            ))?
            .label("without flux weakening")
            .legend(|(x, y)| PathElement::new(vec![(x, y), (x + 20, y)], RED));

        chart
            .draw_series(LineSeries::new(
                samples
                    .iter()
                    .map(|sample| (sample.time_seconds, sample.fw_voltage_utilization)),
                &BLUE,
            ))?
            .label("with flux weakening")
            .legend(|(x, y)| PathElement::new(vec![(x, y), (x + 20, y)], BLUE));

        chart
            .draw_series(LineSeries::new(
                [(0.0, 0.86), (end_time, 0.86)],
                &RGBColor(80, 80, 80),
            ))?
            .label("FW target")
            .legend(|(x, y)| PathElement::new(vec![(x, y), (x + 20, y)], RGBColor(80, 80, 80)));

        chart
            .configure_series_labels()
            .background_style(WHITE)
            .border_style(BLACK)
            .draw()?;
    }

    {
        let mut chart = ChartBuilder::on(&areas[2])
            .caption("Applied Flux-Weakening Current", ("sans-serif", 22))
            .margin(14)
            .x_label_area_size(36)
            .y_label_area_size(72)
            .build_cartesian_2d(0.0_f32..end_time, (id_min * 1.15)..0.5_f32)?;

        chart
            .configure_mesh()
            .disable_mesh()
            .x_desc("time (s)")
            .y_desc("id contribution (A)")
            .draw()?;

        chart.draw_series(LineSeries::new(
            samples
                .iter()
                .map(|sample| (sample.time_seconds, sample.fw_id)),
            &RGBColor(0, 121, 140),
        ))?;
    }

    root.present()?;
    Ok(())
}

fn motor_params() -> MotorParams {
    MotorParams {
        pole_pairs: 7,
        phase_resistance_ohm_ref: Ohms::new(0.08),
        d_inductance_h: Henries::new(0.000_50),
        q_inductance_h: Henries::new(0.000_90),
        flux_linkage_weber: Webers::new(0.004),
        electrical_direction: fluxkit_math::ElectricalDirection::Positive,
        electrical_angle_offset: fluxkit_math::ElectricalAngle::new(0.0),
        limits: MotorLimits {
            max_phase_current: Amps::new(12.0),
            max_mech_speed: Some(RadPerSec::new(360.0)),
            max_winding_temperature_c: None,
        },
    }
}

fn inverter_params() -> InverterParams {
    InverterParams {
        pwm_frequency_hz: Hertz::new(20_000.0),
        min_duty: Duty::new(0.0),
        max_duty: Duty::new(1.0),
        min_bus_voltage: Volts::new(6.0),
        max_bus_voltage: Volts::new(24.0),
        max_voltage_command: Volts::new(8.0),
    }
}

fn config(flux_weakening: FluxWeakeningConfig) -> CurrentLoopConfig {
    CurrentLoopConfig {
        kp_d: 0.45,
        ki_d: 120.0,
        kp_q: 0.45,
        ki_q: 120.0,
        velocity_kp: 0.08,
        velocity_ki: 2.0,
        position_kp: 0.0,
        position_ki: 0.0,
        max_voltage_mag: Volts::new(8.0),
        id_ref_default: Amps::ZERO,
        max_id_target: Amps::new(10.0),
        max_iq_target: Amps::new(10.0),
        max_velocity_target: RadPerSec::new(330.0),
        max_current_ref_derivative_amps_per_sec: 20_000.0,
        enable_current_feedforward: true,
        flux_weakening,
    }
}

fn actuator_params() -> ActuatorParams {
    ActuatorParams {
        gear_ratio: 1.0,
        compensation: ActuatorCompensationConfig::disabled(),
        limits: ActuatorLimits {
            max_output_velocity: Some(RadPerSec::new(330.0)),
            max_output_torque: Some(NewtonMeters::new(8.0)),
        },
    }
}

fn plant_params() -> PmsmParams {
    PmsmParams {
        pole_pairs: 7,
        phase_resistance_ohm_ref: Ohms::new(0.08),
        d_inductance_h: Henries::new(0.000_50),
        q_inductance_h: Henries::new(0.000_90),
        flux_linkage_weber: Webers::new(0.004),
        electrical_direction: fluxkit_math::ElectricalDirection::Positive,
        thermal: ThermalPlantParams::default_for_ambient(25.0),
        actuator: ActuatorPlantParams {
            gear_ratio: 1.0,
            output_inertia_kg_m2: 0.000_40,
            positive_viscous_coefficient: 0.000_02,
            negative_viscous_coefficient: 0.000_02,
            ..ActuatorPlantParams::disabled()
        },
        max_voltage_mag: None,
    }
}
