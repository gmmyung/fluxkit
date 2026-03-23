use std::{cell::RefCell, rc::Rc};

use fluxkit::{
    ActuatorBlendBandCalibrationConfig, ActuatorBlendBandCalibrator,
    ActuatorBreakawayCalibrationConfig, ActuatorBreakawayCalibrator, ActuatorCalibration,
    ActuatorCalibrationSystem, ActuatorCompensationConfig, ActuatorFrictionCalibrationConfig,
    ActuatorFrictionCalibrator, ActuatorParams, CalibrationTickResult, CurrentLoopConfig,
    ElectricalAngle, FluxLinkageCalibrationConfig, FluxLinkageCalibrator, InverterParams,
    MechanicalAngle, MotorCalibration, MotorCalibrationHardware, MotorCalibrationSystem,
    MotorController, MotorHardware, MotorParams, MotorSystem, PhaseInductanceCalibrationConfig,
    PhaseInductanceCalibrator, PhaseResistanceCalibrationConfig, PhaseResistanceCalibrator,
    PolePairsAndOffsetCalibrationConfig, PolePairsAndOffsetCalibrator, TickSchedule,
    centered_phase_duty,
    hal::{
        BusVoltageSensor, CurrentSampleValidity, CurrentSampler, OutputReading, OutputSensor,
        PhaseCurrentSample, PhasePwm, RotorReading, RotorSensor,
    },
    math::{
        inverse_clarke, inverse_park,
        units::{Amps, Duty, Henries, Hertz, NewtonMeters, Ohms, RadPerSec, Volts, Webers},
    },
};
use fluxkit_pmsm_sim::{ActuatorPlantParams, PmsmModel, PmsmParams, PmsmState};

const FAST_DT_SECONDS: f32 = 1.0 / 20_000.0;
const GEAR_RATIO: f32 = 2.0;

fn plant_params() -> PmsmParams {
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

fn actuator_friction_plant_params() -> PmsmParams {
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

fn actuator_breakaway_plant_params() -> PmsmParams {
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

fn controller_motor_params() -> MotorParams {
    MotorParams {
        pole_pairs: 7,
        phase_resistance_ohm: Ohms::new(0.12),
        d_inductance_h: Henries::new(0.000_03),
        q_inductance_h: Henries::new(0.000_03),
        flux_linkage_weber: Some(Webers::new(0.005)),
        electrical_angle_offset: ElectricalAngle::new(0.0),
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

fn actuator_params_calibrating() -> ActuatorParams {
    ActuatorParams {
        gear_ratio: GEAR_RATIO,
        max_output_velocity: Some(RadPerSec::new(30.0)),
        max_output_torque: Some(NewtonMeters::new(10.0)),
        compensation: ActuatorCompensationConfig::disabled(),
    }
}

#[derive(Debug)]
struct SimHarness {
    plant: PmsmModel,
    bus_voltage: Volts,
    load_torque: NewtonMeters,
    last_duty: fluxkit::math::frame::Abc<Duty>,
    rotor_bias: f32,
}

type SharedHarness = Rc<RefCell<SimHarness>>;

fn phase_currents(shared: &SharedHarness) -> fluxkit::math::frame::Abc<Amps> {
    let harness = shared.borrow();
    let state = *harness.plant.state();
    let electrical_angle = fluxkit::math::angle::mechanical_to_electrical(
        state.mechanical_angle.wrapped_0_2pi(),
        harness.plant.params().pole_pairs as u32,
    )
    .wrapped_pm_pi();
    inverse_clarke(inverse_park(
        state.current_dq.map(|current| current.get()),
        electrical_angle.get(),
    ))
    .map(Amps::new)
}

#[derive(Clone, Debug)]
struct SimPwm {
    shared: SharedHarness,
}

impl PhasePwm for SimPwm {
    type Error = core::convert::Infallible;

    fn enable(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }
    fn disable(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }

    fn set_duty(&mut self, a: Duty, b: Duty, c: Duty) -> Result<(), Self::Error> {
        let duty = fluxkit::math::frame::Abc::new(a, b, c);
        let mut harness = self.shared.borrow_mut();
        harness.last_duty = duty;
        let bus_voltage = harness.bus_voltage;
        let load_torque = harness.load_torque;
        harness
            .plant
            .step_phase_duty(duty, bus_voltage, load_torque, FAST_DT_SECONDS)
            .unwrap();
        Ok(())
    }
}

#[derive(Clone, Debug)]
struct SimCurrent {
    shared: SharedHarness,
}

impl CurrentSampler for SimCurrent {
    type Error = core::convert::Infallible;

    fn sample_phase_currents(&mut self) -> Result<PhaseCurrentSample, Self::Error> {
        Ok(PhaseCurrentSample {
            currents: phase_currents(&self.shared),
            validity: CurrentSampleValidity::Valid,
        })
    }
}

#[derive(Clone, Debug)]
struct SimBus {
    shared: SharedHarness,
}

impl BusVoltageSensor for SimBus {
    type Error = core::convert::Infallible;

    fn sample_bus_voltage(&mut self) -> Result<Volts, Self::Error> {
        Ok(self.shared.borrow().bus_voltage)
    }
}

#[derive(Clone, Debug)]
struct SimRotor {
    shared: SharedHarness,
}

impl RotorSensor for SimRotor {
    type Error = core::convert::Infallible;

    fn read_rotor(&mut self) -> Result<RotorReading, Self::Error> {
        let harness = self.shared.borrow();
        let state = *harness.plant.state();
        Ok(RotorReading {
            mechanical_angle: MechanicalAngle::new(fluxkit::math::angle::wrap_0_2pi(
                state.mechanical_angle.get() + harness.rotor_bias,
            )),
            mechanical_velocity: state.mechanical_velocity,
        })
    }
}

#[derive(Clone, Debug)]
struct SimOutput {
    shared: SharedHarness,
}

impl OutputSensor for SimOutput {
    type Error = core::convert::Infallible;

    fn read_output(&mut self) -> Result<OutputReading, Self::Error> {
        let harness = self.shared.borrow();
        let state = *harness.plant.state();
        Ok(OutputReading {
            mechanical_angle: MechanicalAngle::new(state.mechanical_angle.get() / GEAR_RATIO)
                .wrapped_0_2pi(),
            mechanical_velocity: RadPerSec::new(state.mechanical_velocity.get() / GEAR_RATIO),
        })
    }
}

fn hardware(
    shared: &SharedHarness,
) -> MotorCalibrationHardware<SimPwm, SimCurrent, SimBus, SimRotor> {
    MotorCalibrationHardware {
        pwm: SimPwm {
            shared: Rc::clone(shared),
        },
        current: SimCurrent {
            shared: Rc::clone(shared),
        },
        bus: SimBus {
            shared: Rc::clone(shared),
        },
        rotor: SimRotor {
            shared: Rc::clone(shared),
        },
    }
}

fn motor_hardware(
    shared: &SharedHarness,
) -> MotorHardware<SimPwm, SimCurrent, SimBus, SimRotor, SimOutput> {
    MotorHardware {
        pwm: SimPwm {
            shared: Rc::clone(shared),
        },
        current: SimCurrent {
            shared: Rc::clone(shared),
        },
        bus: SimBus {
            shared: Rc::clone(shared),
        },
        rotor: SimRotor {
            shared: Rc::clone(shared),
        },
        output: SimOutput {
            shared: Rc::clone(shared),
        },
    }
}

#[test]
fn motor_calibration_system_recovers_phase_resistance_and_inductance() {
    let shared = Rc::new(RefCell::new(SimHarness {
        plant: PmsmModel::new(
            plant_params(),
            PmsmState {
                mechanical_angle: MechanicalAngle::new(0.4),
                mechanical_velocity: RadPerSec::ZERO,
                current_dq: fluxkit::Dq::new(Amps::ZERO, Amps::ZERO),
            },
        )
        .unwrap(),
        bus_voltage: Volts::new(24.0),
        load_torque: NewtonMeters::ZERO,
        last_duty: centered_phase_duty(),
        rotor_bias: 0.0,
    }));
    let mut system = MotorCalibrationSystem::new(hardware(&shared));

    let mut resistance = PhaseResistanceCalibrator::new(PhaseResistanceCalibrationConfig {
        align_voltage_mag: Volts::new(1.0),
        align_stator_angle: ElectricalAngle::new(0.0),
        settle_velocity_threshold: RadPerSec::new(0.03),
        settle_time_seconds: 0.03,
        sample_time_seconds: 0.05,
        min_projected_current: Amps::new(0.2),
        timeout_seconds: 1.0,
    })
    .unwrap();

    let resistance_result = loop {
        match system
            .tick_phase_resistance(&mut resistance, FAST_DT_SECONDS)
            .unwrap()
        {
            CalibrationTickResult::Running => {}
            CalibrationTickResult::Complete(result) => break result,
        }
    };

    let mut inductance = PhaseInductanceCalibrator::new(PhaseInductanceCalibrationConfig {
        phase_resistance_ohm: resistance_result.phase_resistance_ohm,
        hold_voltage_mag: Volts::new(1.0),
        step_voltage_mag: Volts::new(0.5),
        align_stator_angle: ElectricalAngle::new(0.0),
        settle_velocity_threshold: RadPerSec::new(0.03),
        settle_time_seconds: 0.03,
        sample_time_seconds: 200.0e-6,
        min_projected_current_step: Amps::new(0.05),
        timeout_seconds: 1.0,
    })
    .unwrap();

    let inductance_result = loop {
        match system
            .tick_phase_inductance(&mut inductance, FAST_DT_SECONDS)
            .unwrap()
        {
            CalibrationTickResult::Running => {}
            CalibrationTickResult::Complete(result) => break result,
        }
    };

    let params = plant_params();
    assert!(
        (resistance_result.phase_resistance_ohm.get() - params.phase_resistance_ohm.get()).abs()
            < 0.01
    );
    assert!(
        (inductance_result.phase_inductance_h.get() - params.d_inductance_h.get()).abs() < 3.0e-6
    );
}

#[test]
fn motor_calibration_results_apply_through_public_record() {
    let sweep_result = fluxkit::PolePairsAndOffsetCalibrationResult {
        pole_pairs: 7,
        electrical_angle_offset: ElectricalAngle::new(0.15),
    };
    let resistance_result = fluxkit::PhaseResistanceCalibrationResult {
        phase_resistance_ohm: Ohms::new(0.12),
    };
    let inductance_result = fluxkit::PhaseInductanceCalibrationResult {
        phase_inductance_h: Henries::new(30.0e-6),
    };
    let flux_linkage_result = fluxkit::FluxLinkageCalibrationResult {
        flux_linkage_weber: Webers::new(0.005),
    };

    let calibration = MotorCalibration::empty()
        .merge(sweep_result.into())
        .merge(resistance_result.into())
        .merge(inductance_result.into())
        .merge(flux_linkage_result.into());

    let mut motor = MotorParams {
        pole_pairs: 1,
        phase_resistance_ohm: Ohms::new(0.5),
        d_inductance_h: Henries::new(1.0e-3),
        q_inductance_h: Henries::new(1.0e-3),
        flux_linkage_weber: None,
        electrical_angle_offset: ElectricalAngle::new(0.0),
        max_phase_current: Amps::new(10.0),
        max_mech_speed: None,
    };
    calibration.apply_to_motor_params(&mut motor);

    assert_eq!(motor.pole_pairs, 7);
    assert_eq!(motor.electrical_angle_offset, ElectricalAngle::new(0.15));
    assert_eq!(motor.phase_resistance_ohm, Ohms::new(0.12));
    assert_eq!(motor.d_inductance_h, Henries::new(30.0e-6));
    assert_eq!(motor.q_inductance_h, Henries::new(30.0e-6));
    assert_eq!(motor.flux_linkage_weber, Some(Webers::new(0.005)));
}

#[test]
fn motor_calibration_system_recovers_pole_pairs_and_offset_through_public_wrapper() {
    let params = plant_params();
    let shared = Rc::new(RefCell::new(SimHarness {
        plant: PmsmModel::new(
            params,
            PmsmState {
                mechanical_angle: MechanicalAngle::new(0.9),
                mechanical_velocity: RadPerSec::ZERO,
                current_dq: fluxkit::Dq::new(Amps::ZERO, Amps::ZERO),
            },
        )
        .unwrap(),
        bus_voltage: Volts::new(24.0),
        load_torque: NewtonMeters::ZERO,
        last_duty: centered_phase_duty(),
        rotor_bias: -0.11,
    }));
    let mut system = MotorCalibrationSystem::new(hardware(&shared));
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

    let result = loop {
        match system
            .tick_pole_pairs_and_offset(&mut calibrator, FAST_DT_SECONDS)
            .unwrap()
        {
            CalibrationTickResult::Running => {}
            CalibrationTickResult::Complete(result) => break result,
        }
    };

    assert_eq!(result.pole_pairs, params.pole_pairs);
    let harness = shared.borrow();
    let last_mechanical_angle = MechanicalAngle::new(fluxkit::math::angle::wrap_0_2pi(
        harness.plant.state().mechanical_angle.get() + harness.rotor_bias,
    ));
    let reconstructed = fluxkit::math::angle::wrap_pm_pi(
        fluxkit::math::angle::mechanical_to_electrical(
            last_mechanical_angle,
            result.pole_pairs as u32,
        )
        .get()
            + result.electrical_angle_offset.get()
            - align_stator_angle.get(),
    );
    assert!(reconstructed.abs() < 0.03);
}

#[test]
fn motor_calibration_system_recovers_flux_linkage_through_public_wrapper() {
    let params = plant_params();
    let shared = Rc::new(RefCell::new(SimHarness {
        plant: PmsmModel::new(
            params,
            PmsmState {
                mechanical_angle: MechanicalAngle::new(0.4),
                mechanical_velocity: RadPerSec::ZERO,
                current_dq: fluxkit::Dq::new(Amps::ZERO, Amps::ZERO),
            },
        )
        .unwrap(),
        bus_voltage: Volts::new(24.0),
        load_torque: NewtonMeters::ZERO,
        last_duty: centered_phase_duty(),
        rotor_bias: 0.0,
    }));
    let mut system = MotorCalibrationSystem::new(hardware(&shared));

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

    let result = loop {
        match system
            .tick_flux_linkage(&mut calibrator, FAST_DT_SECONDS)
            .unwrap()
        {
            CalibrationTickResult::Running => {}
            CalibrationTickResult::Complete(result) => break result,
        }
    };

    assert!((result.flux_linkage_weber.get() - params.flux_linkage_weber.get()).abs() < 1.0e-4);
}

#[test]
fn actuator_calibration_system_recovers_friction_through_public_wrapper() {
    let shared = Rc::new(RefCell::new(SimHarness {
        plant: PmsmModel::new_zeroed(actuator_friction_plant_params()).unwrap(),
        bus_voltage: Volts::new(24.0),
        load_torque: NewtonMeters::ZERO,
        last_duty: centered_phase_duty(),
        rotor_bias: 0.0,
    }));

    let controller = MotorController::new(
        controller_motor_params(),
        inverter_params(),
        actuator_params_calibrating(),
        current_loop_config(),
    );
    let motor_system = MotorSystem::new(motor_hardware(&shared), controller);
    let mut system = ActuatorCalibrationSystem::new(motor_system);
    let mut calibrator = ActuatorFrictionCalibrator::new(
        ActuatorFrictionCalibrationConfig::default_for_velocity_sweep(),
    )
    .unwrap();

    let result = loop {
        let schedule = TickSchedule::with_medium(FAST_DT_SECONDS);

        match system
            .tick_friction(&mut calibrator, FAST_DT_SECONDS, schedule)
            .unwrap()
        {
            CalibrationTickResult::Running => {}
            CalibrationTickResult::Complete(result) => break result,
        }
    };
    assert!((result.positive_coulomb_torque.get() - 0.04).abs() < 0.01);
    assert!((result.negative_coulomb_torque.get() - 0.05).abs() < 0.01);
    assert!((result.positive_viscous_coefficient - 0.02).abs() < 0.01);
    assert!((result.negative_viscous_coefficient - 0.03).abs() < 0.01);

    let mut actuator = actuator_params_calibrating();
    let calibration: ActuatorCalibration = result.into();
    calibration.apply_to_actuator_params(&mut actuator);
    assert!((actuator.compensation.friction.positive_coulomb_torque.get() - 0.04).abs() < 0.01);
    assert!((actuator.compensation.friction.negative_viscous_coefficient - 0.03).abs() < 0.01);
}

#[test]
fn actuator_calibration_system_recovers_breakaway_through_public_wrapper() {
    let shared = Rc::new(RefCell::new(SimHarness {
        plant: PmsmModel::new_zeroed(actuator_breakaway_plant_params()).unwrap(),
        bus_voltage: Volts::new(24.0),
        load_torque: NewtonMeters::ZERO,
        last_duty: centered_phase_duty(),
        rotor_bias: 0.0,
    }));

    let controller = MotorController::new(
        controller_motor_params(),
        inverter_params(),
        actuator_params_calibrating(),
        current_loop_config(),
    );
    let motor_system = MotorSystem::new(motor_hardware(&shared), controller);
    let mut system = ActuatorCalibrationSystem::new(motor_system);
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

    let result = loop {
        let schedule = TickSchedule::with_medium(FAST_DT_SECONDS);

        match system
            .tick_breakaway(&mut calibrator, FAST_DT_SECONDS, schedule)
            .unwrap()
        {
            CalibrationTickResult::Running => {}
            CalibrationTickResult::Complete(result) => break result,
        }
    };
    assert!((result.positive_breakaway_torque.get() - 0.08).abs() < 0.02);
    assert!((result.negative_breakaway_torque.get() - 0.09).abs() < 0.02);

    let mut actuator = actuator_params_calibrating();
    let calibration: ActuatorCalibration = result.into();
    calibration.apply_to_actuator_params(&mut actuator);
    assert!(
        (actuator
            .compensation
            .friction
            .positive_breakaway_torque
            .get()
            - 0.08)
            .abs()
            < 0.02
    );
    assert!(
        (actuator
            .compensation
            .friction
            .negative_breakaway_torque
            .get()
            - 0.09)
            .abs()
            < 0.02
    );
}

#[test]
fn actuator_calibration_system_recovers_zero_velocity_blend_band_through_public_wrapper() {
    let shared = Rc::new(RefCell::new(SimHarness {
        plant: PmsmModel::new_zeroed(actuator_breakaway_plant_params()).unwrap(),
        bus_voltage: Volts::new(24.0),
        load_torque: NewtonMeters::ZERO,
        last_duty: centered_phase_duty(),
        rotor_bias: 0.0,
    }));

    let controller = MotorController::new(
        controller_motor_params(),
        inverter_params(),
        actuator_params_calibrating(),
        current_loop_config(),
    );
    let motor_system = MotorSystem::new(motor_hardware(&shared), controller);
    let mut system = ActuatorCalibrationSystem::new(motor_system);
    let mut calibrator = ActuatorBlendBandCalibrator::new(
        ActuatorBlendBandCalibrationConfig::default_for_release_ramp(),
    )
    .unwrap();

    let result = loop {
        let schedule = TickSchedule::with_medium(FAST_DT_SECONDS);

        match system
            .tick_blend_band(&mut calibrator, FAST_DT_SECONDS, schedule)
            .unwrap()
        {
            CalibrationTickResult::Running => {}
            CalibrationTickResult::Complete(result) => break result,
        }
    };
    assert!((result.zero_velocity_blend_band.get() - 0.05).abs() < 0.01);

    let mut actuator = actuator_params_calibrating();
    let calibration: ActuatorCalibration = result.into();
    calibration.apply_to_actuator_params(&mut actuator);
    assert!(
        (actuator
            .compensation
            .friction
            .zero_velocity_blend_band
            .get()
            - 0.05)
            .abs()
            < 0.01
    );
}
