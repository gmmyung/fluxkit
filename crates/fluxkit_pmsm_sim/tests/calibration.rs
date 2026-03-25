use fluxkit_core::{
    ActuatorBlendBandCalibrationConfig, ActuatorBlendBandCalibrationInput,
    ActuatorBlendBandCalibrator, ActuatorBreakawayCalibrationConfig,
    ActuatorBreakawayCalibrationInput, ActuatorBreakawayCalibrator, ActuatorCompensationConfig,
    ActuatorEstimate, ActuatorFrictionCalibrationConfig, ActuatorFrictionCalibrationInput,
    ActuatorFrictionCalibrator, ActuatorGearRatioCalibrationConfig,
    ActuatorGearRatioCalibrationInput, ActuatorGearRatioCalibrator, ActuatorLimits, ActuatorModel,
    ActuatorParams, ControlMode, CurrentLoopConfig, FastLoopInput, FluxLinkageCalibrationConfig,
    FluxLinkageCalibrationInput, FluxLinkageCalibrator, InverterParams, MotorController,
    MotorLimits, MotorModel, PhaseInductanceCalibrationConfig, PhaseInductanceCalibrationInput,
    PhaseInductanceCalibrator, PhaseResistanceCalibrationConfig, PhaseResistanceCalibrationInput,
    PhaseResistanceCalibrator, PolePairsAndOffsetCalibrationConfig,
    PolePairsAndOffsetCalibrationInput, PolePairsAndOffsetCalibrator, RotorEstimate, TickSchedule,
};
use fluxkit_math::{
    ContinuousMechanicalAngle, ElectricalAngle, MechanicalAngle,
    angle::{mechanical_to_electrical, wrap},
    inverse_clarke, inverse_park,
    units::{Henries, NewtonMeters, Ohms, RadPerSec, Volts, Webers},
};
use fluxkit_pmsm_sim::{ActuatorPlantParams, PmsmModel, PmsmParams, PmsmState};

const FAST_DT_SECONDS: f32 = 1.0 / 20_000.0;
const GEAR_RATIO: f32 = 2.0;

fn motor_params() -> PmsmParams {
    PmsmParams {
        pole_pairs: 7,
        phase_resistance_ohm: Ohms::new(0.12),
        d_inductance_h: Henries::new(0.000_03),
        q_inductance_h: Henries::new(0.000_03),
        flux_linkage_weber: Webers::new(0.005),
        actuator: ActuatorPlantParams {
            output_inertia_kg_m2: 0.0002,
            positive_coulomb_torque: NewtonMeters::new(0.02),
            negative_coulomb_torque: NewtonMeters::new(0.02),
            positive_viscous_coefficient: 0.002,
            negative_viscous_coefficient: 0.002,
            ..ActuatorPlantParams::disabled()
        },
        max_voltage_mag: None,
    }
}

fn controller_motor_params() -> fluxkit_core::MotorParams {
    fluxkit_core::MotorParams::from_model_and_limits(
        MotorModel {
            pole_pairs: 7,
            phase_resistance_ohm: Ohms::new(0.12),
            d_inductance_h: Henries::new(0.000_03),
            q_inductance_h: Henries::new(0.000_03),
            flux_linkage_weber: Webers::new(0.005),
            electrical_angle_offset: ElectricalAngle::new(0.0),
        },
        MotorLimits {
            max_phase_current: fluxkit_math::units::Amps::new(10.0),
            max_mech_speed: Some(RadPerSec::new(150.0)),
        },
    )
}

fn inverter_params() -> InverterParams {
    InverterParams {
        pwm_frequency_hz: fluxkit_math::units::Hertz::new(20_000.0),
        min_duty: fluxkit_math::units::Duty::new(0.0),
        max_duty: fluxkit_math::units::Duty::new(1.0),
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
        id_ref_default: fluxkit_math::units::Amps::ZERO,
        max_id_target: fluxkit_math::units::Amps::new(5.0),
        max_iq_target: fluxkit_math::units::Amps::new(8.0),
        max_velocity_target: RadPerSec::new(120.0),
        max_current_ref_derivative_amps_per_sec: 10_000.0,
        enable_current_feedforward: true,
    }
}

fn actuator_params_calibrating() -> ActuatorParams {
    ActuatorParams::from_model_limits_and_compensation(
        ActuatorModel {
            gear_ratio: GEAR_RATIO,
        },
        ActuatorLimits {
            max_output_velocity: Some(RadPerSec::new(30.0)),
            max_output_torque: Some(NewtonMeters::new(10.0)),
        },
        ActuatorCompensationConfig::disabled(),
    )
}

fn actuator_params_unknown_ratio() -> ActuatorParams {
    ActuatorParams::from_model_limits_and_compensation(
        ActuatorModel { gear_ratio: 1.0 },
        ActuatorLimits {
            max_output_velocity: Some(RadPerSec::new(30.0)),
            max_output_torque: Some(NewtonMeters::new(10.0)),
        },
        ActuatorCompensationConfig::disabled(),
    )
}

fn plant_params_with_asymmetric_friction() -> PmsmParams {
    PmsmParams {
        pole_pairs: 7,
        phase_resistance_ohm: Ohms::new(0.12),
        d_inductance_h: Henries::new(0.000_03),
        q_inductance_h: Henries::new(0.000_03),
        flux_linkage_weber: Webers::new(0.005),
        actuator: ActuatorPlantParams {
            gear_ratio: GEAR_RATIO,
            output_inertia_kg_m2: 0.0208,
            positive_breakaway_torque: NewtonMeters::ZERO,
            negative_breakaway_torque: NewtonMeters::ZERO,
            positive_coulomb_torque: NewtonMeters::new(0.04),
            negative_coulomb_torque: NewtonMeters::new(0.05),
            positive_viscous_coefficient: 0.02,
            negative_viscous_coefficient: 0.03,
            zero_velocity_blend_band: RadPerSec::new(0.05),
        },
        max_voltage_mag: None,
    }
}

fn plant_params_with_breakaway() -> PmsmParams {
    PmsmParams {
        pole_pairs: 7,
        phase_resistance_ohm: Ohms::new(0.12),
        d_inductance_h: Henries::new(0.000_03),
        q_inductance_h: Henries::new(0.000_03),
        flux_linkage_weber: Webers::new(0.005),
        actuator: ActuatorPlantParams {
            gear_ratio: GEAR_RATIO,
            output_inertia_kg_m2: 0.0208,
            positive_breakaway_torque: NewtonMeters::new(0.08),
            negative_breakaway_torque: NewtonMeters::new(0.09),
            positive_coulomb_torque: NewtonMeters::new(0.04),
            negative_coulomb_torque: NewtonMeters::new(0.05),
            positive_viscous_coefficient: 0.0,
            negative_viscous_coefficient: 0.0,
            zero_velocity_blend_band: RadPerSec::new(0.05),
        },
        max_voltage_mag: None,
    }
}

fn fast_loop_input(plant: &PmsmModel, bus_voltage: Volts) -> FastLoopInput {
    let state = *plant.state();
    let wrapped_mechanical_angle = state.mechanical_angle.wrapped();
    let electrical_angle = mechanical_to_electrical(
        wrapped_mechanical_angle.into(),
        plant.params().pole_pairs as u32,
    );
    let phase_currents = inverse_clarke(inverse_park(
        state.current_dq.map(|current| current.get()),
        electrical_angle.get(),
    ))
    .map(fluxkit_math::units::Amps::new);

    FastLoopInput {
        phase_currents,
        bus_voltage,
        rotor: RotorEstimate {
            mechanical_angle: wrapped_mechanical_angle.into(),
            mechanical_velocity: state.mechanical_velocity,
        },
        actuator: ActuatorEstimate {
            output_angle: ContinuousMechanicalAngle::new(state.mechanical_angle.get() / GEAR_RATIO)
                .wrapped()
                .into(),
            output_velocity: RadPerSec::new(state.mechanical_velocity.get() / GEAR_RATIO),
        },
        dt_seconds: FAST_DT_SECONDS,
    }
}

#[test]
fn magnetic_hold_recovers_phase_resistance() {
    let params = motor_params();
    let mut plant = PmsmModel::new(
        params,
        PmsmState {
            mechanical_angle: ContinuousMechanicalAngle::new(0.4),
            mechanical_velocity: RadPerSec::ZERO,
            current_dq: fluxkit_math::Dq::new(fluxkit_math::Amps::ZERO, fluxkit_math::Amps::ZERO),
        },
    )
    .unwrap();
    let mut calibrator = PhaseResistanceCalibrator::new(PhaseResistanceCalibrationConfig {
        align_voltage_mag: Volts::new(1.0),
        align_stator_angle: ElectricalAngle::new(0.0),
        settle_velocity_threshold: RadPerSec::new(0.03),
        settle_time_seconds: 0.03,
        sample_time_seconds: 0.05,
        min_projected_current: fluxkit_math::Amps::new(0.2),
        timeout_seconds: 1.0,
    })
    .unwrap();
    let mut phase_currents = fluxkit_math::frame::Abc::new(
        fluxkit_math::Amps::ZERO,
        fluxkit_math::Amps::ZERO,
        fluxkit_math::Amps::ZERO,
    );

    for _ in 0..40_000 {
        let command = calibrator.tick(PhaseResistanceCalibrationInput {
            phase_currents,
            mechanical_velocity: plant.state().mechanical_velocity,
            dt_seconds: FAST_DT_SECONDS,
        });

        let snapshot = plant
            .step_alpha_beta(command, NewtonMeters::ZERO, FAST_DT_SECONDS)
            .unwrap();
        phase_currents = snapshot.phase_currents;

        if calibrator.result().is_some() {
            break;
        }
    }

    assert_eq!(calibrator.error(), None);
    let result = calibrator.result().unwrap();
    assert!(
        (result.phase_resistance_ohm.get() - params.phase_resistance_ohm.get()).abs() < 0.01,
        "estimated R = {}, actual R = {}",
        result.phase_resistance_ohm.get(),
        params.phase_resistance_ohm.get()
    );
}

#[test]
fn magnetic_hold_recovers_phase_inductance() {
    let params = motor_params();
    let mut plant = PmsmModel::new(
        params,
        PmsmState {
            mechanical_angle: ContinuousMechanicalAngle::new(0.4),
            mechanical_velocity: RadPerSec::ZERO,
            current_dq: fluxkit_math::Dq::new(fluxkit_math::Amps::ZERO, fluxkit_math::Amps::ZERO),
        },
    )
    .unwrap();
    let mut calibrator = PhaseInductanceCalibrator::new(PhaseInductanceCalibrationConfig {
        phase_resistance_ohm: params.phase_resistance_ohm,
        hold_voltage_mag: Volts::new(1.0),
        step_voltage_mag: Volts::new(0.5),
        align_stator_angle: ElectricalAngle::new(0.0),
        settle_velocity_threshold: RadPerSec::new(0.03),
        settle_time_seconds: 0.03,
        sample_time_seconds: 200.0e-6,
        min_projected_current_step: fluxkit_math::Amps::new(0.05),
        timeout_seconds: 1.0,
    })
    .unwrap();
    let mut phase_currents = fluxkit_math::frame::Abc::new(
        fluxkit_math::Amps::ZERO,
        fluxkit_math::Amps::ZERO,
        fluxkit_math::Amps::ZERO,
    );

    for _ in 0..40_000 {
        let command = calibrator.tick(PhaseInductanceCalibrationInput {
            phase_currents,
            mechanical_velocity: plant.state().mechanical_velocity,
            dt_seconds: FAST_DT_SECONDS,
        });

        let snapshot = plant
            .step_alpha_beta(command, NewtonMeters::ZERO, FAST_DT_SECONDS)
            .unwrap();
        phase_currents = snapshot.phase_currents;

        if calibrator.result().is_some() {
            break;
        }
    }

    assert_eq!(calibrator.error(), None);
    let result = calibrator.result().unwrap();
    assert!(
        (result.phase_inductance_h.get() - params.d_inductance_h.get()).abs() < 3.0e-6,
        "estimated L = {}, actual L = {}",
        result.phase_inductance_h.get(),
        params.d_inductance_h.get()
    );
}

#[test]
fn slow_sweep_recovers_pole_pairs_and_offset() {
    let params = motor_params();
    let mut plant = PmsmModel::new(
        params,
        PmsmState {
            mechanical_angle: ContinuousMechanicalAngle::new(0.9),
            mechanical_velocity: RadPerSec::ZERO,
            current_dq: fluxkit_math::Dq::new(fluxkit_math::Amps::ZERO, fluxkit_math::Amps::ZERO),
        },
    )
    .unwrap();
    let align_stator_angle = ElectricalAngle::new(0.7);
    let mut calibrator = PolePairsAndOffsetCalibrator::new(PolePairsAndOffsetCalibrationConfig {
        align_voltage_mag: Volts::new(2.5),
        align_stator_angle,
        sweep_electrical_velocity: RadPerSec::new(10.0),
        sweep_electrical_cycles: 3.0,
        settle_velocity_threshold: RadPerSec::new(0.03),
        initial_settle_time_seconds: 0.03,
        final_settle_time_seconds: 0.03,
        pole_pair_rounding_tolerance: 0.05,
        max_pole_pairs: 32,
        timeout_seconds: 4.0,
    })
    .unwrap();
    let encoder_mechanical_bias = -0.11_f32;
    let mut last_measured_angle = MechanicalAngle::new(0.0);

    for _ in 0..80_000 {
        last_measured_angle = MechanicalAngle::new(wrap(
            plant.state().mechanical_angle.get() + encoder_mechanical_bias,
        ));
        let command = calibrator.tick(PolePairsAndOffsetCalibrationInput {
            mechanical_angle: last_measured_angle,
            mechanical_velocity: plant.state().mechanical_velocity,
            dt_seconds: FAST_DT_SECONDS,
        });

        plant
            .step_alpha_beta(command, NewtonMeters::ZERO, FAST_DT_SECONDS)
            .unwrap();

        if calibrator.result().is_some() {
            break;
        }
    }

    assert_eq!(calibrator.error(), None);
    let result = calibrator.result().unwrap();
    assert_eq!(result.pole_pairs, params.pole_pairs);
    let reconstructed = wrap(
        mechanical_to_electrical(last_measured_angle.into(), result.pole_pairs as u32).get()
            + result.electrical_angle_offset.get()
            - align_stator_angle.get(),
    );
    assert!(
        reconstructed.abs() < 0.03,
        "endpoint electrical alignment error = {reconstructed}"
    );
}

#[test]
fn controlled_spin_recovers_flux_linkage() {
    let params = motor_params();
    let mut plant = PmsmModel::new(
        params,
        PmsmState {
            mechanical_angle: ContinuousMechanicalAngle::new(0.4),
            mechanical_velocity: RadPerSec::ZERO,
            current_dq: fluxkit_math::Dq::new(fluxkit_math::Amps::ZERO, fluxkit_math::Amps::ZERO),
        },
    )
    .unwrap();
    let mut calibrator = FluxLinkageCalibrator::new(FluxLinkageCalibrationConfig {
        phase_resistance_ohm: params.phase_resistance_ohm,
        phase_inductance_h: params.d_inductance_h,
        pole_pairs: params.pole_pairs,
        electrical_angle_offset: ElectricalAngle::new(0.0),
        align_voltage_mag: Volts::new(2.0),
        spin_voltage_mag: Volts::new(2.0),
        align_stator_angle: ElectricalAngle::new(0.0),
        spin_electrical_velocity: RadPerSec::new(20.0),
        initial_settle_velocity_threshold: RadPerSec::new(0.03),
        initial_settle_time_seconds: 0.03,
        min_electrical_velocity: RadPerSec::new(8.0),
        sample_time_seconds: 0.05,
        timeout_seconds: 2.0,
    })
    .unwrap();
    let mut phase_currents = fluxkit_math::frame::Abc::new(
        fluxkit_math::Amps::ZERO,
        fluxkit_math::Amps::ZERO,
        fluxkit_math::Amps::ZERO,
    );

    for _ in 0..60_000 {
        let command = calibrator.tick(FluxLinkageCalibrationInput {
            phase_currents,
            mechanical_angle: plant.state().mechanical_angle.wrapped().into(),
            mechanical_velocity: plant.state().mechanical_velocity,
            dt_seconds: FAST_DT_SECONDS,
        });

        let snapshot = plant
            .step_alpha_beta(command, NewtonMeters::ZERO, FAST_DT_SECONDS)
            .unwrap();
        phase_currents = snapshot.phase_currents;

        if calibrator.result().is_some() {
            break;
        }
    }

    assert_eq!(calibrator.error(), None);
    let result = calibrator.result().unwrap();
    assert!(
        (result.flux_linkage_weber.get() - params.flux_linkage_weber.get()).abs() < 1.0e-4,
        "estimated psi = {}, actual psi = {}",
        result.flux_linkage_weber.get(),
        params.flux_linkage_weber.get()
    );
}

#[test]
fn steady_velocity_sweep_recovers_actuator_friction() {
    let bus_voltage = Volts::new(24.0);
    let mut controller = MotorController::new(
        controller_motor_params(),
        inverter_params(),
        actuator_params_calibrating(),
        current_loop_config(),
    );
    let mut plant = PmsmModel::new_zeroed(plant_params_with_asymmetric_friction()).unwrap();
    let mut calibrator = ActuatorFrictionCalibrator::new(
        ActuatorFrictionCalibrationConfig::default_for_velocity_sweep(),
    )
    .unwrap();

    controller.set_mode(ControlMode::Velocity);
    controller.enable();

    for step in 0..200_000 {
        let status = controller.status();
        let command = calibrator.tick(ActuatorFrictionCalibrationInput {
            output_velocity: status.last_output_mechanical_velocity,
            output_torque_command: status
                .last_actuator_compensation
                .total_output_torque_command,
            dt_seconds: FAST_DT_SECONDS,
        });
        controller.set_velocity_target(command.velocity_target);

        let _ = step;
        let schedule = TickSchedule::with_medium(FAST_DT_SECONDS);
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

        if calibrator.result().is_some() {
            break;
        }
    }

    assert_eq!(calibrator.error(), None);
    let result = calibrator.result().unwrap();
    assert!(
        (result.positive_coulomb_torque.get() - 0.04).abs() < 0.01,
        "estimated positive coulomb = {}",
        result.positive_coulomb_torque.get()
    );
    assert!(
        (result.negative_coulomb_torque.get() - 0.05).abs() < 0.01,
        "estimated negative coulomb = {}",
        result.negative_coulomb_torque.get()
    );
    assert!(
        (result.positive_viscous_coefficient - 0.02).abs() < 0.01,
        "estimated positive viscous = {}",
        result.positive_viscous_coefficient
    );
    assert!(
        (result.negative_viscous_coefficient - 0.03).abs() < 0.01,
        "estimated negative viscous = {}",
        result.negative_viscous_coefficient
    );
}

#[test]
fn steady_velocity_travel_recovers_actuator_gear_ratio() {
    let bus_voltage = Volts::new(24.0);
    let mut controller = MotorController::new(
        controller_motor_params(),
        inverter_params(),
        actuator_params_unknown_ratio(),
        current_loop_config(),
    );
    let mut plant = PmsmModel::new_zeroed(plant_params_with_asymmetric_friction()).unwrap();
    let mut calibrator = ActuatorGearRatioCalibrator::new(
        ActuatorGearRatioCalibrationConfig::default_for_travel_ratio(),
    )
    .unwrap();

    controller.set_mode(ControlMode::Velocity);
    controller.enable();

    for _ in 0..200_000 {
        let status = controller.status();
        let command = calibrator.tick(ActuatorGearRatioCalibrationInput {
            rotor_mechanical_angle: status.last_rotor_mechanical_angle,
            output_mechanical_angle: status.last_output_mechanical_angle,
            output_velocity: status.last_output_mechanical_velocity,
            dt_seconds: FAST_DT_SECONDS,
        });
        controller.set_velocity_target(command.velocity_target);

        let output = controller.tick(
            fast_loop_input(&plant, bus_voltage),
            TickSchedule::with_medium(FAST_DT_SECONDS),
        );
        assert_eq!(controller.status().active_error, None);
        plant
            .step_phase_duty(
                output.phase_duty,
                bus_voltage,
                NewtonMeters::ZERO,
                FAST_DT_SECONDS,
            )
            .unwrap();

        if calibrator.result().is_some() {
            break;
        }
    }

    assert_eq!(calibrator.error(), None);
    let result = calibrator.result().unwrap();
    assert!(
        (result.gear_ratio - GEAR_RATIO).abs() < 0.05,
        "estimated gear ratio = {}",
        result.gear_ratio
    );
}

#[test]
fn torque_ramp_recovers_actuator_breakaway() {
    let bus_voltage = Volts::new(24.0);
    let mut controller = MotorController::new(
        controller_motor_params(),
        inverter_params(),
        actuator_params_calibrating(),
        current_loop_config(),
    );
    let mut plant = PmsmModel::new_zeroed(plant_params_with_breakaway()).unwrap();
    let mut calibrator = ActuatorBreakawayCalibrator::new(ActuatorBreakawayCalibrationConfig {
        positive_coulomb_torque: NewtonMeters::new(0.04),
        negative_coulomb_torque: NewtonMeters::new(0.05),
        torque_ramp_rate_nm_per_sec: 1.0,
        max_torque: NewtonMeters::new(0.3),
        motion_velocity_threshold: RadPerSec::new(0.05),
        motion_confirm_time_seconds: 0.01,
        rest_velocity_threshold: RadPerSec::new(0.02),
        rest_time_seconds: 0.05,
        timeout_seconds: 4.0,
    })
    .unwrap();

    controller.set_mode(ControlMode::Torque);
    controller.enable();

    for _ in 0..200_000 {
        let status = controller.status();
        let command = calibrator.tick(ActuatorBreakawayCalibrationInput {
            output_velocity: status.last_output_mechanical_velocity,
            output_torque_command: status
                .last_actuator_compensation
                .total_output_torque_command,
            dt_seconds: FAST_DT_SECONDS,
        });
        controller.set_torque_target(command.torque_target);

        let output = controller.tick(
            fast_loop_input(&plant, bus_voltage),
            TickSchedule::with_medium(FAST_DT_SECONDS),
        );
        assert_eq!(controller.status().active_error, None);
        plant
            .step_phase_duty(
                output.phase_duty,
                bus_voltage,
                NewtonMeters::ZERO,
                FAST_DT_SECONDS,
            )
            .unwrap();

        if calibrator.result().is_some() {
            break;
        }
    }

    assert_eq!(calibrator.error(), None);
    let result = calibrator.result().unwrap();
    assert!(
        (result.positive_breakaway_torque.get() - 0.08).abs() < 0.02,
        "estimated positive breakaway = {}",
        result.positive_breakaway_torque.get()
    );
    assert!(
        (result.negative_breakaway_torque.get() - 0.09).abs() < 0.02,
        "estimated negative breakaway = {}",
        result.negative_breakaway_torque.get()
    );
}

#[test]
fn low_speed_sweep_recovers_zero_velocity_blend_band() {
    let bus_voltage = Volts::new(24.0);
    let mut plant = PmsmModel::new_zeroed(plant_params_with_breakaway()).unwrap();
    let mut controller = MotorController::new(
        controller_motor_params(),
        inverter_params(),
        actuator_params_calibrating(),
        current_loop_config(),
    );
    let mut calibrator = ActuatorBlendBandCalibrator::new(
        ActuatorBlendBandCalibrationConfig::default_for_release_ramp(),
    )
    .unwrap();

    controller.set_mode(ControlMode::Torque);
    controller.enable();

    for _ in 0..200_000 {
        let status = controller.status();
        let command = calibrator.tick(ActuatorBlendBandCalibrationInput {
            output_velocity: status.last_output_mechanical_velocity,
            output_torque_command: status
                .last_actuator_compensation
                .total_output_torque_command,
            dt_seconds: FAST_DT_SECONDS,
        });
        controller.set_torque_target(command.torque_target);

        let output = controller.tick(
            fast_loop_input(&plant, bus_voltage),
            TickSchedule::with_medium(FAST_DT_SECONDS),
        );
        assert_eq!(controller.status().active_error, None);
        plant
            .step_phase_duty(
                output.phase_duty,
                bus_voltage,
                NewtonMeters::ZERO,
                FAST_DT_SECONDS,
            )
            .unwrap();

        if calibrator.result().is_some() {
            break;
        }
    }

    assert_eq!(calibrator.error(), None);
    let result = calibrator.result().unwrap();
    assert!(
        (result.zero_velocity_blend_band.get() - 0.05).abs() < 0.01,
        "estimated blend band = {}",
        result.zero_velocity_blend_band.get()
    );
}
