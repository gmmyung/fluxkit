use super::{ControllerCommand, MotorController};
use crate::{
    actuator::{
        ActuatorCompensationConfig, ActuatorEstimate, ActuatorLimits, ActuatorParams,
        FrictionCompensation,
    },
    config::CurrentLoopConfig,
    control::current::CurrentEstimator,
    error::Error,
    io::{ControlInput, RotorEstimate},
    mode::ControlMode,
    params::{InverterParams, MotorLimits, MotorModel, MotorParams},
    state::MotorState,
};
use fluxkit_math::{
    ContinuousMechanicalAngle, ElectricalAngle, ElectricalDirection, SinePwm, Svpwm,
    frame::{Abc, Dq},
    units::{Amps, Duty, Henries, Hertz, NewtonMeters, Ohms, RadPerSec, Volts, Webers},
};

fn test_motor() -> MotorParams {
    MotorParams::from_model_and_limits(
        MotorModel {
            pole_pairs: 7,
            phase_resistance_ohm_ref: Ohms::new(0.12),
            d_inductance_h: Henries::new(0.000_03),
            q_inductance_h: Henries::new(0.000_03),
            flux_linkage_weber: Webers::new(0.005),
            electrical_direction: ElectricalDirection::Positive,
            electrical_angle_offset: ElectricalAngle::new(0.0),
        },
        MotorLimits {
            max_phase_current: Amps::new(20.0),
            max_mech_speed: Some(RadPerSec::new(100.0)),
            max_winding_temperature_c: None,
        },
    )
}

fn test_actuator() -> ActuatorParams {
    ActuatorParams::from_model_limits_and_compensation(
        crate::actuator::ActuatorModel { gear_ratio: 5.0 },
        ActuatorLimits {
            max_output_velocity: Some(RadPerSec::new(20.0)),
            max_output_torque: Some(NewtonMeters::new(20.0)),
        },
        ActuatorCompensationConfig::disabled(),
    )
}

fn test_inverter() -> InverterParams {
    InverterParams {
        pwm_frequency_hz: Hertz::new(20_000.0),
        min_duty: Duty::new(0.0),
        max_duty: Duty::new(1.0),
        min_bus_voltage: Volts::new(6.0),
        max_bus_voltage: Volts::new(60.0),
        max_voltage_command: Volts::new(24.0),
    }
}

fn test_config() -> CurrentLoopConfig {
    CurrentLoopConfig {
        kp_d: 2.0,
        ki_d: 50.0,
        kp_q: 2.0,
        ki_q: 50.0,
        velocity_kp: 0.5,
        velocity_ki: 10.0,
        position_kp: 4.0,
        position_ki: 0.0,
        max_voltage_mag: Volts::new(12.0),
        id_ref_default: Amps::ZERO,
        max_id_target: Amps::new(10.0),
        max_iq_target: Amps::new(10.0),
        max_velocity_target: RadPerSec::new(100.0),
        max_current_ref_derivative_amps_per_sec: 10_000.0,
        enable_current_feedforward: true,
    }
}

fn test_input() -> ControlInput {
    ControlInput {
        command: ControllerCommand::Disabled,
        armed: false,
        clear_fault_requested: false,
        phase_currents: Abc::new(Amps::ZERO, Amps::ZERO, Amps::ZERO),
        bus_voltage: Volts::new(24.0),
        winding_temperature_c: 25.0,
        rotor: RotorEstimate {
            mechanical_angle: ContinuousMechanicalAngle::new(0.0),
            mechanical_velocity: RadPerSec::ZERO,
        },
        actuator: ActuatorEstimate {
            output_angle: ContinuousMechanicalAngle::new(0.0),
            output_velocity: RadPerSec::ZERO,
        },
        dt_seconds: 1.0 / 20_000.0,
    }
}

#[derive(Clone, Copy, Debug)]
struct FixedCurrentEstimator {
    estimated_idq: Dq<Amps>,
}

impl CurrentEstimator for FixedCurrentEstimator {
    fn reset(&mut self) {}

    fn output(&self) -> Dq<Amps> {
        self.estimated_idq
    }

    fn update(&mut self, _measured_idq: Dq<Amps>, _dt_seconds: f32) -> Dq<Amps> {
        self.estimated_idq
    }
}

#[test]
fn zero_input_zero_target_returns_neutral_output() {
    let mut controller = MotorController::new(
        test_motor(),
        test_inverter(),
        test_actuator(),
        test_config(),
        Svpwm,
        crate::PassThroughCurrentEstimator::new(),
    );
    controller.set_mode(ControlMode::Current);
    controller.enable();

    let output = controller.fast_tick(test_input());

    assert_eq!(output.phase_duty.a.get(), 0.5);
    assert_eq!(output.phase_duty.b.get(), 0.5);
    assert_eq!(output.phase_duty.c.get(), 0.5);
    assert_eq!(output.measured_idq, Dq::new(Amps::ZERO, Amps::ZERO));
    assert_eq!(output.commanded_vdq, Dq::new(Volts::ZERO, Volts::ZERO));
    assert!(!output.saturated);
    assert_eq!(controller.status().state, MotorState::Running);
    assert_eq!(controller.status().active_error, None);
}

#[test]
fn invalid_bus_voltage_latches_fault_and_centers_output() {
    let mut controller = MotorController::new(
        test_motor(),
        test_inverter(),
        test_actuator(),
        test_config(),
        Svpwm,
        crate::PassThroughCurrentEstimator::new(),
    );
    controller.set_mode(ControlMode::Current);
    controller.enable();

    let mut input = test_input();
    input.bus_voltage = Volts::new(0.0);
    let output = controller.fast_tick(input);

    assert_eq!(output.phase_duty.a.get(), 0.5);
    assert_eq!(controller.status().state, MotorState::Faulted);
    assert_eq!(
        controller.status().active_error,
        Some(Error::InvalidBusVoltage)
    );
}

#[test]
fn invalid_angle_latches_fault() {
    let mut controller = MotorController::new(
        test_motor(),
        test_inverter(),
        test_actuator(),
        test_config(),
        Svpwm,
        crate::PassThroughCurrentEstimator::new(),
    );
    controller.set_mode(ControlMode::Current);
    controller.enable();

    let mut input = test_input();
    input.rotor.mechanical_angle = ContinuousMechanicalAngle::new(f32::NAN);
    let _output = controller.fast_tick(input);

    assert_eq!(controller.status().state, MotorState::Faulted);
    assert_eq!(
        controller.status().active_error,
        Some(Error::InvalidRotorAngle)
    );
}

#[test]
fn over_temperature_latches_fault_and_centers_output() {
    let mut controller = MotorController::new(
        test_motor(),
        test_inverter(),
        test_actuator(),
        test_config(),
        Svpwm,
        crate::PassThroughCurrentEstimator::new(),
    );
    controller.motor.limits.max_winding_temperature_c = Some(80.0);
    controller.set_mode(ControlMode::Current);
    controller.enable();

    let mut input = test_input();
    input.winding_temperature_c = 95.0;
    let output = controller.fast_tick(input);

    assert_eq!(output.phase_duty.a.get(), 0.5);
    assert_eq!(output.phase_duty.b.get(), 0.5);
    assert_eq!(output.phase_duty.c.get(), 0.5);
    assert_eq!(controller.status().state, MotorState::Faulted);
    assert_eq!(
        controller.status().active_error,
        Some(Error::OverTemperature)
    );
}

#[test]
fn positive_iq_target_produces_positive_vq() {
    let mut controller = MotorController::new(
        test_motor(),
        test_inverter(),
        test_actuator(),
        test_config(),
        Svpwm,
        crate::PassThroughCurrentEstimator::new(),
    );
    controller.set_mode(ControlMode::Current);
    controller.set_iq_target(Amps::new(3.0));
    controller.enable();

    let output = controller.fast_tick(test_input());

    assert!(output.commanded_vdq.q.get() > 0.0);
    assert_eq!(controller.status().active_error, None);
}

#[test]
fn current_estimator_output_drives_current_loop_error() {
    let mut config = test_config();
    config.kp_d = 0.0;
    config.ki_d = 0.0;
    config.kp_q = 2.0;
    config.ki_q = 0.0;
    config.enable_current_feedforward = false;

    let mut controller = MotorController::new(
        test_motor(),
        test_inverter(),
        test_actuator(),
        config,
        Svpwm,
        FixedCurrentEstimator {
            estimated_idq: Dq::new(Amps::ZERO, Amps::new(2.5)),
        },
    );
    controller.set_mode(ControlMode::Current);
    controller.set_iq_target(Amps::new(3.0));
    controller.enable();

    let output = controller.fast_tick(test_input());

    assert_eq!(output.measured_idq, Dq::new(Amps::ZERO, Amps::ZERO));
    assert!((output.commanded_vdq.q.get() - 1.0).abs() < 1.0e-6);
}

#[test]
fn saturation_is_reported_for_aggressive_current_request() {
    let mut config = test_config();
    config.kp_q = 50.0;
    config.max_voltage_mag = Volts::new(4.0);

    let mut controller = MotorController::new(
        test_motor(),
        test_inverter(),
        test_actuator(),
        config,
        Svpwm,
        crate::PassThroughCurrentEstimator::new(),
    );
    controller.set_mode(ControlMode::Current);
    controller.set_id_target(Amps::new(10.0));
    controller.set_iq_target(Amps::new(10.0));
    controller.enable();

    let output = controller.fast_tick(test_input());

    assert!(output.saturated);
    assert!(output.phase_duty.a.get() >= 0.0 && output.phase_duty.a.get() <= 1.0);
    assert!(output.phase_duty.b.get() >= 0.0 && output.phase_duty.b.get() <= 1.0);
    assert!(output.phase_duty.c.get() >= 0.0 && output.phase_duty.c.get() <= 1.0);
}

#[test]
fn duty_stays_within_configured_duty_window() {
    let mut inverter = test_inverter();
    inverter.min_duty = Duty::new(0.1);
    inverter.max_duty = Duty::new(0.9);

    let mut controller = MotorController::new(
        test_motor(),
        inverter,
        test_actuator(),
        test_config(),
        Svpwm,
        crate::PassThroughCurrentEstimator::new(),
    );
    controller.set_mode(ControlMode::Current);
    controller.set_iq_target(Amps::new(10.0));
    controller.enable();

    let output = controller.fast_tick(test_input());

    for duty in [
        output.phase_duty.a,
        output.phase_duty.b,
        output.phase_duty.c,
    ] {
        assert!(duty.get() >= 0.1 && duty.get() <= 0.9);
    }
}

#[test]
fn state_transitions_are_explicit() {
    let mut controller = MotorController::new(
        test_motor(),
        test_inverter(),
        test_actuator(),
        test_config(),
        Svpwm,
        crate::PassThroughCurrentEstimator::new(),
    );

    assert_eq!(controller.status().state, MotorState::Disabled);

    controller.enable();
    assert_eq!(controller.status().state, MotorState::Ready);

    controller.set_mode(ControlMode::Current);
    controller.fast_tick(test_input());
    assert_eq!(controller.status().state, MotorState::Running);

    let mut bad_input = test_input();
    bad_input.bus_voltage = Volts::new(100.0);
    controller.fast_tick(bad_input);
    assert_eq!(controller.status().state, MotorState::Faulted);

    controller.clear_error();
    assert_eq!(controller.status().state, MotorState::Disabled);

    controller.disable();
    assert_eq!(controller.status().state, MotorState::Disabled);
}

#[test]
fn controller_can_use_alternate_modulator() {
    let mut controller = MotorController::new(
        test_motor(),
        test_inverter(),
        test_actuator(),
        test_config(),
        SinePwm,
        crate::PassThroughCurrentEstimator::new(),
    );
    controller.set_mode(ControlMode::Current);
    controller.set_iq_target(Amps::new(3.0));
    controller.enable();

    let output = controller.fast_tick(test_input());

    assert_eq!(controller.status().active_error, None);
    for duty in [
        output.phase_duty.a,
        output.phase_duty.b,
        output.phase_duty.c,
    ] {
        assert!((0.0..=1.0).contains(&duty.get()));
    }
}

#[test]
fn torque_mode_updates_iq_target_in_supervisory_update() {
    let mut controller = MotorController::new(
        test_motor(),
        test_inverter(),
        test_actuator(),
        test_config(),
        Svpwm,
        crate::PassThroughCurrentEstimator::new(),
    );
    controller.set_mode(ControlMode::Torque);
    controller.set_torque_target(NewtonMeters::new(1.5));
    controller.enable();

    controller.update_supervisory_references(0.001);

    assert!((controller.iq_target.get() - 5.714_286).abs() < 1.0e-6);
}

#[test]
fn torque_mode_adds_bounded_friction_compensation() {
    let mut actuator = test_actuator();
    actuator.compensation = ActuatorCompensationConfig {
        friction: FrictionCompensation {
            enabled: true,
            positive_breakaway_torque: NewtonMeters::new(0.2),
            negative_breakaway_torque: NewtonMeters::new(0.3),
            positive_coulomb_torque: NewtonMeters::new(0.4),
            negative_coulomb_torque: NewtonMeters::new(0.5),
            positive_viscous_coefficient: 0.02,
            negative_viscous_coefficient: 0.03,
            zero_velocity_blend_band: RadPerSec::new(1.0),
        },
        max_total_torque: NewtonMeters::new(0.5),
    };

    let mut controller = MotorController::new(
        test_motor(),
        test_inverter(),
        actuator,
        test_config(),
        Svpwm,
        crate::PassThroughCurrentEstimator::new(),
    );
    controller.set_mode(ControlMode::Torque);
    controller.set_torque_target(NewtonMeters::new(0.2));
    controller.enable();

    controller.update_supervisory_references(0.001);

    assert!(
        controller
            .status()
            .last_actuator_compensation
            .friction_torque
            .get()
            > 0.0
    );
    assert!(
        controller
            .status()
            .last_actuator_compensation
            .total_compensation_torque
            .get()
            <= 0.5
    );
    assert!(
        controller
            .status()
            .last_actuator_compensation
            .total_output_torque_command
            .get()
            > 0.2
    );
}

#[test]
fn breakaway_compensation_only_fills_missing_margin() {
    let mut actuator = test_actuator();
    actuator.compensation = ActuatorCompensationConfig {
        friction: FrictionCompensation {
            enabled: true,
            positive_breakaway_torque: NewtonMeters::new(0.2),
            negative_breakaway_torque: NewtonMeters::new(0.2),
            positive_coulomb_torque: NewtonMeters::new(0.05),
            negative_coulomb_torque: NewtonMeters::new(0.05),
            positive_viscous_coefficient: 0.0,
            negative_viscous_coefficient: 0.0,
            zero_velocity_blend_band: RadPerSec::new(1.0),
        },
        max_total_torque: NewtonMeters::new(1.0),
    };

    let mut controller = MotorController::new(
        test_motor(),
        test_inverter(),
        actuator,
        test_config(),
        Svpwm,
        crate::PassThroughCurrentEstimator::new(),
    );
    controller.set_mode(ControlMode::Torque);
    controller.set_torque_target(NewtonMeters::new(0.25));
    controller.enable();

    controller.update_supervisory_references(0.001);

    let telemetry = controller.status().last_actuator_compensation;
    assert!(
        telemetry.breakaway_torque.get().abs() < 1.0e-6,
        "expected breakaway term to be capped away once command exceeds breakaway, got {}",
        telemetry.breakaway_torque.get()
    );
    assert!(
        telemetry.coulomb_torque.get() > 0.0,
        "expected coulomb term to remain active, got {}",
        telemetry.coulomb_torque.get()
    );
}

#[test]
fn torque_mode_friction_compensation_tracks_measured_velocity_for_viscous_drag() {
    let mut actuator = test_actuator();
    actuator.compensation = ActuatorCompensationConfig {
        friction: FrictionCompensation {
            enabled: true,
            positive_breakaway_torque: NewtonMeters::new(0.1),
            negative_breakaway_torque: NewtonMeters::new(0.1),
            positive_coulomb_torque: NewtonMeters::new(0.05),
            negative_coulomb_torque: NewtonMeters::new(0.05),
            positive_viscous_coefficient: 0.1,
            negative_viscous_coefficient: 0.1,
            zero_velocity_blend_band: RadPerSec::new(1.0),
        },
        max_total_torque: NewtonMeters::new(1.0),
    };

    let mut controller = MotorController::new(
        test_motor(),
        test_inverter(),
        actuator,
        test_config(),
        Svpwm,
        crate::PassThroughCurrentEstimator::new(),
    );
    controller.set_mode(ControlMode::Torque);
    controller.set_torque_target(NewtonMeters::new(0.2));
    controller.enable();

    controller.update_supervisory_references(0.001);
    let near_zero = controller
        .status()
        .last_actuator_compensation
        .friction_torque
        .get();

    controller.status.last_output_mechanical_velocity = RadPerSec::new(4.0);
    controller.update_supervisory_references(0.001);
    let moving = controller
        .status()
        .last_actuator_compensation
        .friction_torque
        .get();

    assert!(
        moving > near_zero,
        "expected viscous compensation to grow with measured velocity: near_zero={near_zero}, moving={moving}",
    );
}

#[test]
fn velocity_mode_generates_positive_q_current_target() {
    let mut controller = MotorController::new(
        test_motor(),
        test_inverter(),
        test_actuator(),
        test_config(),
        Svpwm,
        crate::PassThroughCurrentEstimator::new(),
    );
    controller.set_mode(ControlMode::Velocity);
    controller.set_velocity_target(RadPerSec::new(20.0));
    controller.enable();
    controller.fast_tick(test_input());

    controller.update_supervisory_references(0.001);

    assert!(controller.iq_target.get() > 0.0);
}

#[test]
fn mit_mode_generates_positive_q_current_target() {
    let mut controller = MotorController::new(
        test_motor(),
        test_inverter(),
        test_actuator(),
        test_config(),
        Svpwm,
        crate::PassThroughCurrentEstimator::new(),
    );
    controller.set_mode(ControlMode::Mit);
    controller.set_mit_command(
        ContinuousMechanicalAngle::new(1.0),
        RadPerSec::ZERO,
        5.0,
        0.2,
        NewtonMeters::ZERO,
    );
    controller.enable();
    controller.fast_tick(test_input());

    controller.update_supervisory_references(0.01);

    assert!(controller.output_torque_target.get() > 0.0);
    assert!(controller.iq_target.get() > 0.0);
}

#[test]
fn position_mode_runs_position_and_velocity_loops_in_supervisory_update() {
    let mut controller = MotorController::new(
        test_motor(),
        test_inverter(),
        test_actuator(),
        test_config(),
        Svpwm,
        crate::PassThroughCurrentEstimator::new(),
    );
    controller.set_mode(ControlMode::Position);
    controller.set_position_target(ContinuousMechanicalAngle::new(1.0));
    controller.enable();
    controller.fast_tick(test_input());

    controller.update_supervisory_references(0.01);

    assert!(controller.output_velocity_target.get() > 0.0);
    assert!(controller.iq_target.get() > 0.0);
}

#[test]
fn wrapped_encoder_angle_is_unwrapped_for_multi_turn_positioning() {
    let mut controller = MotorController::new(
        test_motor(),
        test_inverter(),
        test_actuator(),
        test_config(),
        Svpwm,
        crate::PassThroughCurrentEstimator::new(),
    );
    controller.set_mode(ControlMode::Position);
    controller.enable();

    let mut input = test_input();
    input.actuator.output_angle = ContinuousMechanicalAngle::new(6.0);
    controller.fast_tick(input);

    let mut wrapped_input = test_input();
    wrapped_input.actuator.output_angle = ContinuousMechanicalAngle::new(0.2);
    controller.fast_tick(wrapped_input);

    assert!(
        controller
            .status()
            .last_unwrapped_output_mechanical_angle
            .get()
            > 6.2
    );
}

#[test]
fn open_loop_voltage_mode_bypasses_current_pi() {
    let mut controller = MotorController::new(
        test_motor(),
        test_inverter(),
        test_actuator(),
        test_config(),
        Svpwm,
        crate::PassThroughCurrentEstimator::new(),
    );
    controller.set_mode(ControlMode::OpenLoopVoltage);
    controller.set_open_loop_voltage_target(Dq::new(Volts::new(0.0), Volts::new(3.0)));
    controller.enable();

    let output = controller.fast_tick(test_input());

    assert!(output.commanded_vdq.q.get() > 0.0);
    assert!(
        output.phase_duty.a.get() != 0.5
            || output.phase_duty.b.get() != 0.5
            || output.phase_duty.c.get() != 0.5
    );
}

#[test]
fn current_feedforward_adds_back_emf_compensation() {
    let mut controller = MotorController::new(
        test_motor(),
        test_inverter(),
        test_actuator(),
        test_config(),
        Svpwm,
        crate::PassThroughCurrentEstimator::new(),
    );
    controller.set_mode(ControlMode::Current);
    controller.enable();

    let mut input = test_input();
    input.rotor.mechanical_velocity = RadPerSec::new(25.0);
    let output = controller.fast_tick(input);

    assert!(output.commanded_vdq.q.get() > 0.0);
}

#[test]
fn current_feedforward_adds_reference_derivative_term_on_step() {
    let mut config = test_config();
    config.kp_d = 0.0;
    config.ki_d = 0.0;
    config.kp_q = 0.0;
    config.ki_q = 0.0;

    let mut controller = MotorController::new(
        test_motor(),
        test_inverter(),
        test_actuator(),
        config,
        Svpwm,
        crate::PassThroughCurrentEstimator::new(),
    );
    controller.set_mode(ControlMode::Current);
    controller.enable();

    let baseline = controller.fast_tick(test_input());
    assert_eq!(baseline.commanded_vdq.q, Volts::ZERO);

    controller.set_iq_target(Amps::new(3.0));
    let step_response = controller.fast_tick(test_input());
    let steady_response = controller.fast_tick(test_input());

    assert!(step_response.commanded_vdq.q.get() > steady_response.commanded_vdq.q.get());
}

#[test]
fn current_reference_derivative_feedforward_is_clamped() {
    let mut config = test_config();
    config.kp_d = 0.0;
    config.ki_d = 0.0;
    config.kp_q = 0.0;
    config.ki_q = 0.0;
    config.max_current_ref_derivative_amps_per_sec = 1_000.0;

    let motor = test_motor();
    let mut controller = MotorController::new(
        motor,
        test_inverter(),
        test_actuator(),
        config,
        Svpwm,
        crate::PassThroughCurrentEstimator::new(),
    );
    controller.set_mode(ControlMode::Current);
    controller.enable();
    controller.fast_tick(test_input());

    controller.set_iq_target(Amps::new(10.0));
    let output = controller.fast_tick(test_input());

    let expected_q = motor.phase_resistance_ohm_ref.get() * 10.0
        + motor.q_inductance_h.get() * config.max_current_ref_derivative_amps_per_sec;
    assert!((output.commanded_vdq.q.get() - expected_q).abs() < 1.0e-5);
}

#[test]
fn modulation_limit_tracks_selected_modulator() {
    let mut config = test_config();
    config.max_voltage_mag = Volts::new(20.0);

    let mut sine_controller = MotorController::new(
        test_motor(),
        test_inverter(),
        test_actuator(),
        config,
        SinePwm,
        crate::PassThroughCurrentEstimator::new(),
    );
    sine_controller.set_mode(ControlMode::OpenLoopVoltage);
    sine_controller.set_open_loop_voltage_target(Dq::new(Volts::ZERO, Volts::new(13.0)));
    sine_controller.enable();

    let mut svpwm_controller = MotorController::new(
        test_motor(),
        test_inverter(),
        test_actuator(),
        config,
        Svpwm,
        crate::PassThroughCurrentEstimator::new(),
    );
    svpwm_controller.set_mode(ControlMode::OpenLoopVoltage);
    svpwm_controller.set_open_loop_voltage_target(Dq::new(Volts::ZERO, Volts::new(13.0)));
    svpwm_controller.enable();

    let sine_output = sine_controller.fast_tick(test_input());
    let svpwm_output = svpwm_controller.fast_tick(test_input());

    assert!(sine_output.commanded_vdq.q.get() <= 12.0);
    assert!(svpwm_output.commanded_vdq.q.get() > sine_output.commanded_vdq.q.get());
}

#[test]
fn feedforward_can_be_disabled() {
    let mut config = test_config();
    config.enable_current_feedforward = false;

    let mut controller = MotorController::new(
        test_motor(),
        test_inverter(),
        test_actuator(),
        config,
        Svpwm,
        crate::PassThroughCurrentEstimator::new(),
    );
    controller.set_mode(ControlMode::Current);
    controller.enable();

    let mut input = test_input();
    input.rotor.mechanical_velocity = RadPerSec::new(25.0);
    let output = controller.fast_tick(input);

    assert_eq!(output.commanded_vdq.q, Volts::ZERO);
}
