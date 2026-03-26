use fluxkit_core::{
    ActuatorCompensationConfig, ActuatorEstimate, ActuatorLimits, ActuatorParams, ControlMode,
    CurrentLoopConfig, FastLoopInput, FrictionCompensation, InverterParams, MotorController,
    MotorLimits, MotorParams, RotorEstimate, TickSchedule,
};
use fluxkit_math::{
    ContinuousMechanicalAngle,
    angle::mechanical_to_electrical,
    inverse_clarke, inverse_park,
    units::{Amps, Duty, Henries, Hertz, NewtonMeters, Ohms, RadPerSec, Volts, Webers},
};
use fluxkit_pmsm_sim::{ActuatorPlantParams, PmsmModel, PmsmParams, ThermalPlantParams};

const FAST_DT_SECONDS: f32 = 1.0 / 20_000.0;
const MEDIUM_DECIMATION: usize = 10;
const MEDIUM_DT_SECONDS: f32 = FAST_DT_SECONDS * MEDIUM_DECIMATION as f32;
const GEAR_RATIO: f32 = 2.0;
const TARGET_STEP_INDEX: usize = 100;
const POSITION_TARGET_RADIANS: f32 = 2.0;

fn motor_params() -> MotorParams {
    MotorParams {
        pole_pairs: 7,
        phase_resistance_ohm_ref: Ohms::new(0.12),
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

fn current_loop_config() -> CurrentLoopConfig {
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
        phase_resistance_ohm_ref: Ohms::new(0.12),
        d_inductance_h: Henries::new(0.000_03),
        q_inductance_h: Henries::new(0.000_03),
        flux_linkage_weber: Webers::new(0.005),
        thermal: ThermalPlantParams::default_for_ambient(25.0),
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

fn plant_params_with_output_inertia() -> PmsmParams {
    let mut params = plant_params();
    params.actuator.output_inertia_kg_m2 = 0.0208;
    params.actuator.positive_breakaway_torque = NewtonMeters::new(0.08);
    params.actuator.negative_breakaway_torque = NewtonMeters::new(0.08);
    params.actuator.positive_coulomb_torque = NewtonMeters::new(0.04);
    params.actuator.negative_coulomb_torque = NewtonMeters::new(0.04);
    params.actuator.positive_viscous_coefficient = 0.02;
    params.actuator.negative_viscous_coefficient = 0.02;
    params.actuator.zero_velocity_blend_band = RadPerSec::new(0.5);
    params
}

fn actuator_params_with_friction_compensation() -> ActuatorParams {
    ActuatorParams {
        gear_ratio: GEAR_RATIO,
        compensation: ActuatorCompensationConfig {
            friction: FrictionCompensation {
                enabled: true,
                positive_breakaway_torque: NewtonMeters::new(0.08),
                negative_breakaway_torque: NewtonMeters::new(0.08),
                positive_coulomb_torque: NewtonMeters::new(0.04),
                negative_coulomb_torque: NewtonMeters::new(0.04),
                positive_viscous_coefficient: 0.02,
                negative_viscous_coefficient: 0.02,
                zero_velocity_blend_band: RadPerSec::new(0.5),
            },
            max_total_torque: NewtonMeters::new(0.4),
        },
        limits: ActuatorLimits {
            max_output_velocity: Some(RadPerSec::new(30.0)),
            max_output_torque: Some(NewtonMeters::new(10.0)),
        },
    }
}

fn fast_loop_input(plant: &PmsmModel, bus_voltage: Volts) -> FastLoopInput {
    let state = *plant.state();
    let wrapped_mechanical_angle = state.mechanical_angle.wrapped();
    let wrapped_output_angle = ContinuousMechanicalAngle::new(
        state.mechanical_angle.get() / plant.params().actuator.gear_ratio,
    )
    .wrapped();
    let electrical_angle = mechanical_to_electrical(
        wrapped_mechanical_angle.into(),
        plant.params().pole_pairs as u32,
    );
    let phase_currents = inverse_clarke(inverse_park(
        state.current_dq.map(|current| current.get()),
        electrical_angle.get(),
    ))
    .map(Amps::new);

    FastLoopInput {
        phase_currents,
        bus_voltage,
        winding_temperature_c: state.winding_temperature_c,
        rotor: RotorEstimate {
            mechanical_angle: wrapped_mechanical_angle.into(),
            mechanical_velocity: state.mechanical_velocity,
        },
        actuator: ActuatorEstimate {
            output_angle: wrapped_output_angle.into(),
            output_velocity: RadPerSec::new(
                state.mechanical_velocity.get() / plant.params().actuator.gear_ratio,
            ),
        },
        dt_seconds: FAST_DT_SECONDS,
    }
}

fn run_fast_step(
    controller: &mut MotorController,
    plant: &mut PmsmModel,
    bus_voltage: Volts,
    load_torque: NewtonMeters,
) {
    let output = controller.fast_tick(fast_loop_input(plant, bus_voltage));
    assert_eq!(controller.status().active_error, None);
    plant
        .step_phase_duty(output.phase_duty, bus_voltage, load_torque, FAST_DT_SECONDS)
        .unwrap();
}

fn assert_abs_diff_le(actual: f32, expected: f32, max_abs_diff: f32, what: &str) {
    let abs_diff = (actual - expected).abs();
    assert!(
        abs_diff <= max_abs_diff,
        "{what} was {actual}, expected {expected} +/- {max_abs_diff} (abs diff {abs_diff})",
    );
}

#[test]
fn current_mode_drives_positive_q_current_into_the_plant() {
    let bus_voltage = Volts::new(24.0);
    let mut controller = MotorController::new(
        motor_params(),
        inverter_params(),
        actuator_params(),
        current_loop_config(),
    );
    let mut plant = PmsmModel::new_zeroed(plant_params()).unwrap();

    controller.set_mode(ControlMode::Current);
    controller.set_iq_target(Amps::new(3.0));
    controller.enable();

    for _ in 0..4_000 {
        run_fast_step(&mut controller, &mut plant, bus_voltage, NewtonMeters::ZERO);
    }

    let status = controller.status();
    assert!(status.active_error.is_none());
    assert_abs_diff_le(
        status.last_measured_idq.q.get(),
        3.0,
        0.05,
        "measured q current",
    );
    assert_abs_diff_le(
        status.last_measured_idq.d.get(),
        0.0,
        0.05,
        "measured d current",
    );
    assert!(
        plant.state().mechanical_velocity.get() > 100.0,
        "mechanical velocity did not build strongly enough: {}",
        plant.state().mechanical_velocity.get()
    );
}

#[test]
fn position_mode_tracks_output_axis_feedback() {
    let bus_voltage = Volts::new(24.0);
    let mut controller = MotorController::new(
        motor_params(),
        inverter_params(),
        actuator_params(),
        current_loop_config(),
    );
    let mut plant = PmsmModel::new_zeroed(plant_params()).unwrap();

    controller.set_mode(ControlMode::Position);
    controller.set_position_target(ContinuousMechanicalAngle::new(POSITION_TARGET_RADIANS));
    controller.enable();

    for step in 0..200_000 {
        let schedule = if step % MEDIUM_DECIMATION == MEDIUM_DECIMATION - 1 {
            TickSchedule::with_medium(MEDIUM_DT_SECONDS)
        } else {
            TickSchedule::none()
        };
        let output = controller.tick(fast_loop_input(&plant, bus_voltage), schedule);
        assert_eq!(controller.status().active_error, None);
        plant
            .step_phase_duty(
                output.phase_duty,
                bus_voltage,
                NewtonMeters::ZERO,
                FAST_DT_SECONDS,
            )
            .unwrap();
    }

    let status = controller.status();
    assert!(status.active_error.is_none());
    assert_abs_diff_le(
        status.last_unwrapped_output_mechanical_angle.get(),
        POSITION_TARGET_RADIANS,
        0.01,
        "unwrapped output mechanical angle",
    );
    assert_abs_diff_le(
        status.last_output_mechanical_velocity.get(),
        0.0,
        0.01,
        "output mechanical velocity",
    );
}

#[test]
fn friction_compensation_improves_velocity_command_with_output_inertia() {
    let bus_voltage = Volts::new(24.0);
    let mut uncompensated = MotorController::new(
        motor_params(),
        inverter_params(),
        actuator_params(),
        current_loop_config(),
    );
    let mut compensated = MotorController::new(
        motor_params(),
        inverter_params(),
        actuator_params_with_friction_compensation(),
        current_loop_config(),
    );
    let mut uncompensated_plant =
        PmsmModel::new_zeroed(plant_params_with_output_inertia()).unwrap();
    let mut compensated_plant = PmsmModel::new_zeroed(plant_params_with_output_inertia()).unwrap();

    uncompensated.set_mode(ControlMode::Velocity);
    uncompensated.enable();

    compensated.set_mode(ControlMode::Velocity);
    compensated.enable();

    for step in 0..1_000 {
        let target_velocity = if step < TARGET_STEP_INDEX {
            0.0
        } else {
            let elapsed = (step - TARGET_STEP_INDEX) as f32 * FAST_DT_SECONDS;
            (30.0 * elapsed).min(3.0)
        };
        uncompensated.set_velocity_target(RadPerSec::new(target_velocity));
        compensated.set_velocity_target(RadPerSec::new(target_velocity));

        let schedule = if step % MEDIUM_DECIMATION == 0 {
            TickSchedule::with_medium(MEDIUM_DT_SECONDS)
        } else {
            TickSchedule::none()
        };

        let uncompensated_output =
            uncompensated.tick(fast_loop_input(&uncompensated_plant, bus_voltage), schedule);
        let compensated_output =
            compensated.tick(fast_loop_input(&compensated_plant, bus_voltage), schedule);

        assert_eq!(uncompensated.status().active_error, None);
        assert_eq!(compensated.status().active_error, None);

        uncompensated_plant
            .step_phase_duty(
                uncompensated_output.phase_duty,
                bus_voltage,
                NewtonMeters::ZERO,
                FAST_DT_SECONDS,
            )
            .unwrap();
        compensated_plant
            .step_phase_duty(
                compensated_output.phase_duty,
                bus_voltage,
                NewtonMeters::ZERO,
                FAST_DT_SECONDS,
            )
            .unwrap();
    }

    let uncompensated_output_velocity =
        uncompensated_plant.state().mechanical_velocity.get() / GEAR_RATIO;
    let compensated_output_velocity =
        compensated_plant.state().mechanical_velocity.get() / GEAR_RATIO;
    let velocity_delta = compensated_output_velocity - uncompensated_output_velocity;

    assert!(
        velocity_delta > 0.04,
        "compensated output velocity improvement {velocity_delta} was too small (compensated {compensated_output_velocity}, uncompensated {uncompensated_output_velocity})",
    );
    assert!(
        compensated
            .status()
            .last_actuator_compensation
            .friction_torque
            .get()
            > 0.03
    );
}
