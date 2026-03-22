use fluxkit_core::{
    ActuatorCompensationConfig, ActuatorEstimate, ActuatorParams, ControlMode, CurrentLoopConfig,
    FastLoopInput, FrictionCompensation, InertiaCompensation, InverterParams, LoadCompensation,
    MotorController, MotorParams, RotorEstimate, TickSchedule,
};
use fluxkit_math::{
    MechanicalAngle,
    angle::mechanical_to_electrical,
    inverse_clarke, inverse_park,
    units::{Amps, Duty, Henries, Hertz, NewtonMeters, Ohms, RadPerSec, Volts, Webers},
};
use fluxkit_pmsm_sim::{ActuatorPlantParams, PmsmModel, PmsmParams};

const FAST_DT_SECONDS: f32 = 1.0 / 20_000.0;
const MEDIUM_DECIMATION: usize = 10;
const MEDIUM_DT_SECONDS: f32 = FAST_DT_SECONDS * MEDIUM_DECIMATION as f32;
const GEAR_RATIO: f32 = 2.0;

fn motor_params() -> MotorParams {
    MotorParams {
        pole_pairs: 7,
        phase_resistance_ohm: Ohms::new(0.12),
        d_inductance_h: Henries::new(0.000_03),
        q_inductance_h: Henries::new(0.000_03),
        flux_linkage_weber: Some(Webers::new(0.005)),
        max_phase_current: Amps::new(10.0),
        max_mech_speed: Some(RadPerSec::new(150.0)),
    }
}

fn inverter_params() -> InverterParams {
    InverterParams {
        pwm_frequency_hz: Hertz::new(20_000.0),
        deadtime_ns: 500,
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
        max_output_velocity: Some(RadPerSec::new(30.0)),
        max_output_torque: Some(NewtonMeters::new(10.0)),
        compensation: ActuatorCompensationConfig::disabled(),
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
            ..ActuatorPlantParams::disabled()
        },
        max_voltage_mag: None,
    }
}

fn plant_params_with_output_bias() -> PmsmParams {
    let mut params = plant_params();
    params.actuator.constant_bias_torque = NewtonMeters::new(0.4);
    params
}

fn actuator_params_with_load_compensation() -> ActuatorParams {
    ActuatorParams {
        gear_ratio: GEAR_RATIO,
        max_output_velocity: Some(RadPerSec::new(30.0)),
        max_output_torque: Some(NewtonMeters::new(10.0)),
        compensation: ActuatorCompensationConfig {
            friction: FrictionCompensation::disabled(),
            inertia: InertiaCompensation::disabled(),
            load: LoadCompensation {
                enabled: true,
                constant_bias_torque: NewtonMeters::new(0.4),
            },
            max_total_torque: NewtonMeters::new(1.0),
        },
    }
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
            electrical_angle,
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

fn run_fast_step(
    controller: &mut MotorController,
    plant: &mut PmsmModel,
    bus_voltage: Volts,
    load_torque: NewtonMeters,
) {
    let output = controller.fast_tick(fast_loop_input(plant, bus_voltage));
    assert_eq!(output.error, None);
    plant
        .step_phase_duty(output.phase_duty, bus_voltage, load_torque, FAST_DT_SECONDS)
        .unwrap();
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
    assert!(status.last_measured_idq.q.get() > 1.5);
    assert!(plant.state().mechanical_velocity.get() > 0.0);
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
    controller.set_position_target(MechanicalAngle::new(1.5 * core::f32::consts::TAU));
    controller.enable();

    for step in 0..200_000 {
        let schedule = if step % MEDIUM_DECIMATION == MEDIUM_DECIMATION - 1 {
            TickSchedule::with_medium(MEDIUM_DT_SECONDS)
        } else {
            TickSchedule::none()
        };
        let output = controller.tick(fast_loop_input(&plant, bus_voltage), schedule);
        assert_eq!(output.error, None);
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
    assert!(
        status.last_unwrapped_output_mechanical_angle.get() > 2.0,
        "output-axis position did not advance meaningfully: {}",
        status.last_unwrapped_output_mechanical_angle.get()
    );
    assert!(status.last_output_mechanical_velocity.get().is_finite());
}

#[test]
fn actuator_load_compensation_improves_breakaway_against_output_bias() {
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
        actuator_params_with_load_compensation(),
        current_loop_config(),
    );
    let mut uncompensated_plant = PmsmModel::new_zeroed(plant_params_with_output_bias()).unwrap();
    let mut compensated_plant = PmsmModel::new_zeroed(plant_params_with_output_bias()).unwrap();

    uncompensated.set_mode(ControlMode::Torque);
    uncompensated.set_torque_target(NewtonMeters::new(0.2));
    uncompensated.enable();
    uncompensated.medium_tick(MEDIUM_DT_SECONDS);

    compensated.set_mode(ControlMode::Torque);
    compensated.set_torque_target(NewtonMeters::new(0.2));
    compensated.enable();
    compensated.medium_tick(MEDIUM_DT_SECONDS);

    for _ in 0..10_000 {
        run_fast_step(
            &mut uncompensated,
            &mut uncompensated_plant,
            bus_voltage,
            NewtonMeters::ZERO,
        );
        run_fast_step(
            &mut compensated,
            &mut compensated_plant,
            bus_voltage,
            NewtonMeters::ZERO,
        );
    }

    let uncompensated_output_velocity =
        uncompensated_plant.state().mechanical_velocity.get() / GEAR_RATIO;
    let compensated_output_velocity =
        compensated_plant.state().mechanical_velocity.get() / GEAR_RATIO;

    assert!(
        compensated_output_velocity > uncompensated_output_velocity + 0.5,
        "compensated output velocity {compensated_output_velocity} did not exceed uncompensated {uncompensated_output_velocity}",
    );
    assert!(
        compensated
            .status()
            .last_actuator_compensation
            .load_torque
            .get()
            > 0.0
    );
}
