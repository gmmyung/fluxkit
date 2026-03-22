use std::{env, error::Error, fs};

use fluxkit_core::{
    ActuatorCompensationConfig, ActuatorEstimate, ActuatorParams, ControlMode, CurrentLoopConfig,
    FastLoopInput, FrictionCompensation, InverterParams, MotorController, MotorParams,
    RotorEstimate,
};
use fluxkit_math::{
    MechanicalAngle,
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
const SIMULATION_STEPS: usize = 5_000;
const TARGET_STEP_INDEX: usize = 100;
const OUTPUT_TORQUE_STEP_NM: f32 = 0.06;

#[derive(Clone, Copy)]
struct Sample {
    time_seconds: f32,
    uncompensated_output_velocity: f32,
    compensated_output_velocity: f32,
    expected_output_velocity: f32,
    feedback_torque: f32,
    total_output_torque: f32,
    breakaway_compensation_torque: f32,
    coulomb_compensation_torque: f32,
    viscous_compensation_torque: f32,
}

fn main() -> Result<(), Box<dyn Error>> {
    let output_path = env::args()
        .nth(1)
        .unwrap_or_else(|| "target/plots/closed_loop_torque_command.svg".to_owned());
    if let Some(parent) = std::path::Path::new(&output_path).parent() {
        fs::create_dir_all(parent)?;
    }

    let bus_voltage = Volts::new(24.0);
    let mut uncompensated = MotorController::new(
        motor_params(),
        inverter_params(),
        actuator_params_disabled(),
        config(),
    );
    let mut compensated = MotorController::new(
        motor_params(),
        inverter_params(),
        actuator_params_compensated(),
        config(),
    );
    let mut uncompensated_plant = PmsmModel::new_zeroed(plant_params()).unwrap();
    let mut compensated_plant = PmsmModel::new_zeroed(plant_params()).unwrap();
    let mut samples = Vec::with_capacity(SIMULATION_STEPS);
    let expected_output_inertia = total_output_equivalent_inertia_kg_m2();
    let mut expected_output_velocity = 0.0_f32;

    uncompensated.set_mode(ControlMode::Torque);
    uncompensated.enable();

    compensated.set_mode(ControlMode::Torque);
    compensated.enable();

    for step in 0..SIMULATION_STEPS {
        let output_torque_target = output_torque_target_for_step(step);
        uncompensated.set_torque_target(NewtonMeters::new(output_torque_target));
        compensated.set_torque_target(NewtonMeters::new(output_torque_target));

        if step % MEDIUM_DECIMATION == 0 {
            uncompensated.medium_tick(MEDIUM_DT_SECONDS);
            compensated.medium_tick(MEDIUM_DT_SECONDS);
        }

        let uncompensated_output =
            uncompensated.fast_tick(fast_loop_input(&uncompensated_plant, bus_voltage));
        let compensated_output =
            compensated.fast_tick(fast_loop_input(&compensated_plant, bus_voltage));

        uncompensated_plant.step_phase_duty(
            uncompensated_output.phase_duty,
            bus_voltage,
            NewtonMeters::ZERO,
            FAST_DT_SECONDS,
        )?;
        compensated_plant.step_phase_duty(
            compensated_output.phase_duty,
            bus_voltage,
            NewtonMeters::ZERO,
            FAST_DT_SECONDS,
        )?;

        let compensation = compensated.status().last_actuator_compensation;
        let expected_output_acceleration = if expected_output_inertia > 0.0 {
            compensation.feedback_torque.get() / expected_output_inertia
        } else {
            0.0
        };
        expected_output_velocity += expected_output_acceleration * FAST_DT_SECONDS;

        samples.push(Sample {
            time_seconds: step as f32 * FAST_DT_SECONDS,
            uncompensated_output_velocity: uncompensated_plant.state().mechanical_velocity.get()
                / GEAR_RATIO,
            compensated_output_velocity: compensated_plant.state().mechanical_velocity.get()
                / GEAR_RATIO,
            expected_output_velocity,
            feedback_torque: compensation.feedback_torque.get(),
            total_output_torque: compensation.total_output_torque_command.get(),
            breakaway_compensation_torque: compensation.breakaway_torque.get(),
            coulomb_compensation_torque: compensation.coulomb_torque.get(),
            viscous_compensation_torque: compensation.viscous_torque.get(),
        });
    }

    draw_plot(&output_path, &samples)?;
    println!("wrote {output_path}");
    Ok(())
}

fn fast_loop_input(plant: &PmsmModel, bus_voltage: Volts) -> FastLoopInput {
    let state = *plant.state();
    let wrapped_mechanical_angle = state.mechanical_angle.wrapped_0_2pi();
    let wrapped_output_angle =
        MechanicalAngle::new(state.mechanical_angle.get() / plant.params().actuator.gear_ratio)
            .wrapped_0_2pi();
    let electrical_angle =
        mechanical_to_electrical(wrapped_mechanical_angle, plant.params().pole_pairs as u32)
            .wrapped_pm_pi();
    let phase_currents = inverse_clarke(inverse_park(
        state.current_dq.map(|current| current.get()),
        electrical_angle.get(),
    ))
    .map(Amps::new);

    FastLoopInput {
        phase_currents,
        bus_voltage,
        rotor: RotorEstimate {
            mechanical_angle: wrapped_mechanical_angle,
            mechanical_velocity: state.mechanical_velocity,
        },
        actuator: ActuatorEstimate {
            output_angle: wrapped_output_angle,
            output_velocity: RadPerSec::new(
                state.mechanical_velocity.get() / plant.params().actuator.gear_ratio,
            ),
        },
        dt_seconds: FAST_DT_SECONDS,
    }
}

fn draw_plot(path: &str, samples: &[Sample]) -> Result<(), Box<dyn Error>> {
    let root = SVGBackend::new(path, (960, 480)).into_drawing_area();
    root.fill(&WHITE)?;

    let areas = root.split_evenly((1, 2));
    let end_time = samples
        .last()
        .map(|sample| sample.time_seconds)
        .unwrap_or(1.0);
    let velocity_max = samples.iter().fold(0.0_f32, |acc, sample| {
        acc.max(
            sample
                .uncompensated_output_velocity
                .abs()
                .max(sample.compensated_output_velocity.abs())
                .max(sample.expected_output_velocity.abs()),
        )
    });
    let torque_max = samples.iter().fold(0.0_f32, |acc, sample| {
        acc.max(
            sample
                .feedback_torque
                .abs()
                .max(sample.total_output_torque.abs())
                .max(sample.breakaway_compensation_torque.abs())
                .max(sample.coulomb_compensation_torque.abs())
                .max(sample.viscous_compensation_torque.abs()),
        )
    });

    {
        let mut chart = ChartBuilder::on(&areas[0])
            .caption("Torque Command", ("sans-serif", 24))
            .margin(16)
            .x_label_area_size(40)
            .y_label_area_size(70)
            .build_cartesian_2d(0.0_f32..end_time, -0.2_f32..(velocity_max * 1.1).max(1.0))?;

        chart
            .configure_mesh()
            .disable_mesh()
            .x_desc("time (s)")
            .y_desc("output velocity (rad/s)")
            .draw()?;

        chart
            .draw_series(LineSeries::new(
                samples
                    .iter()
                    .map(|sample| (sample.time_seconds, sample.expected_output_velocity)),
                &BLACK,
            ))?
            .label("expected if friction is cancelled")
            .legend(|(x, y)| PathElement::new(vec![(x, y), (x + 20, y)], BLACK));

        chart
            .draw_series(LineSeries::new(
                samples
                    .iter()
                    .map(|sample| (sample.time_seconds, sample.uncompensated_output_velocity)),
                &RED,
            ))?
            .label("uncompensated real")
            .legend(|(x, y)| PathElement::new(vec![(x, y), (x + 20, y)], RED));

        chart
            .draw_series(LineSeries::new(
                samples
                    .iter()
                    .map(|sample| (sample.time_seconds, sample.compensated_output_velocity)),
                &BLUE,
            ))?
            .label("compensated real")
            .legend(|(x, y)| PathElement::new(vec![(x, y), (x + 20, y)], BLUE));

        chart
            .configure_series_labels()
            .background_style(WHITE)
            .border_style(BLACK)
            .draw()?;
    }

    {
        let mut chart = ChartBuilder::on(&areas[1])
            .caption("Output Torque Breakdown", ("sans-serif", 24))
            .margin(16)
            .x_label_area_size(40)
            .y_label_area_size(70)
            .build_cartesian_2d(0.0_f32..end_time, -0.05_f32..(torque_max * 1.2).max(0.5))?;

        chart
            .configure_mesh()
            .disable_mesh()
            .x_desc("time (s)")
            .y_desc("output torque (Nm)")
            .draw()?;

        chart
            .draw_series(LineSeries::new(
                samples
                    .iter()
                    .map(|sample| (sample.time_seconds, sample.feedback_torque)),
                &RED,
            ))?
            .label("feedback torque")
            .legend(|(x, y)| PathElement::new(vec![(x, y), (x + 20, y)], RED));

        chart
            .draw_series(LineSeries::new(
                samples
                    .iter()
                    .map(|sample| (sample.time_seconds, sample.breakaway_compensation_torque)),
                &RGBColor(196, 30, 58),
            ))?
            .label("breakaway compensation")
            .legend(|(x, y)| PathElement::new(vec![(x, y), (x + 20, y)], RGBColor(196, 30, 58)));

        chart
            .draw_series(LineSeries::new(
                samples
                    .iter()
                    .map(|sample| (sample.time_seconds, sample.coulomb_compensation_torque)),
                &RGBColor(0, 121, 140),
            ))?
            .label("coulomb compensation")
            .legend(|(x, y)| PathElement::new(vec![(x, y), (x + 20, y)], RGBColor(0, 121, 140)));

        chart
            .draw_series(LineSeries::new(
                samples
                    .iter()
                    .map(|sample| (sample.time_seconds, sample.viscous_compensation_torque)),
                &RGBColor(218, 165, 32),
            ))?
            .label("viscous compensation")
            .legend(|(x, y)| PathElement::new(vec![(x, y), (x + 20, y)], RGBColor(218, 165, 32)));

        chart
            .draw_series(LineSeries::new(
                samples
                    .iter()
                    .map(|sample| (sample.time_seconds, sample.total_output_torque)),
                &BLACK,
            ))?
            .label("total output torque")
            .legend(|(x, y)| PathElement::new(vec![(x, y), (x + 20, y)], BLACK));

        chart
            .configure_series_labels()
            .background_style(WHITE)
            .border_style(BLACK)
            .draw()?;
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
        flux_linkage_weber: Some(Webers::new(0.005)),
        electrical_angle_offset: fluxkit_math::ElectricalAngle::new(0.0),
        max_phase_current: Amps::new(10.0),
        max_mech_speed: Some(RadPerSec::new(150.0)),
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

fn actuator_params_disabled() -> ActuatorParams {
    ActuatorParams {
        gear_ratio: GEAR_RATIO,
        max_output_velocity: Some(RadPerSec::new(30.0)),
        max_output_torque: Some(NewtonMeters::new(10.0)),
        compensation: ActuatorCompensationConfig::disabled(),
    }
}

fn actuator_params_compensated() -> ActuatorParams {
    ActuatorParams {
        gear_ratio: GEAR_RATIO,
        max_output_velocity: Some(RadPerSec::new(30.0)),
        max_output_torque: Some(NewtonMeters::new(10.0)),
        compensation: ActuatorCompensationConfig {
            friction: FrictionCompensation {
                enabled: true,
                positive_breakaway_torque: NewtonMeters::new(0.09),
                negative_breakaway_torque: NewtonMeters::new(0.09),
                positive_coulomb_torque: NewtonMeters::new(0.04),
                negative_coulomb_torque: NewtonMeters::new(0.04),
                positive_viscous_coefficient: 0.02,
                negative_viscous_coefficient: 0.02,
                zero_velocity_blend_band: RadPerSec::new(0.05),
            },
            max_total_torque: NewtonMeters::new(0.16),
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
        inertia_kg_m2: 0.0002,
        viscous_friction_nm_per_rad_per_sec: 0.0002,
        static_friction_nm: NewtonMeters::new(0.0),
        actuator: ActuatorPlantParams {
            gear_ratio: GEAR_RATIO,
            actuator_inertia_kg_m2: 0.005,
            load_inertia_kg_m2: 0.015,
            positive_breakaway_torque: NewtonMeters::new(0.08),
            negative_breakaway_torque: NewtonMeters::new(0.08),
            positive_coulomb_torque: NewtonMeters::new(0.04),
            negative_coulomb_torque: NewtonMeters::new(0.04),
            positive_viscous_coefficient: 0.02,
            negative_viscous_coefficient: 0.02,
            zero_velocity_blend_band: RadPerSec::new(0.05),
        },
        max_voltage_mag: None,
    }
}

fn total_output_equivalent_inertia_kg_m2() -> f32 {
    let plant = plant_params();
    plant.actuator.total_output_inertia_kg_m2()
        + plant.inertia_kg_m2 * plant.actuator.gear_ratio * plant.actuator.gear_ratio
}

fn output_torque_target_for_step(step: usize) -> f32 {
    if step < TARGET_STEP_INDEX {
        0.0
    } else {
        OUTPUT_TORQUE_STEP_NM
    }
}
