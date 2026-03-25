use std::{cell::RefCell, rc::Rc};

use fluxkit::core::MotorCalibration;
use fluxkit::{
    ActuatorCalibrationLimits, ActuatorCalibrationRequest, ActuatorCalibrationSystem,
    ActuatorLimits, ContinuousMechanicalAngle, CurrentLoopConfig, ElectricalAngle, InverterParams,
    MotorCalibrationLimits, MotorCalibrationRequest, MotorCalibrationSystem, MotorHardware,
    MotorLimits, MotorModel, MotorParams, TickSchedule, centered_phase_duty,
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
    MotorParams::from_model_and_limits(
        MotorModel {
            pole_pairs: 7,
            phase_resistance_ohm: Ohms::new(0.12),
            d_inductance_h: Henries::new(0.000_03),
            q_inductance_h: Henries::new(0.000_03),
            flux_linkage_weber: Webers::new(0.005),
            electrical_angle_offset: ElectricalAngle::new(0.0),
        },
        MotorLimits {
            max_phase_current: Amps::new(10.0),
            max_mech_speed: Some(RadPerSec::new(150.0)),
        },
    )
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
        state.mechanical_angle.wrapped().into(),
        harness.plant.params().pole_pairs as u32,
    );
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
            mechanical_angle: fluxkit::math::MechanicalAngle::new(
                state.mechanical_angle.get() + harness.rotor_bias,
            ),
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
            mechanical_angle: ContinuousMechanicalAngle::new(
                state.mechanical_angle.get() / GEAR_RATIO,
            )
            .wrapped(),
            mechanical_velocity: RadPerSec::new(state.mechanical_velocity.get() / GEAR_RATIO),
        })
    }
}

fn calibration_hardware(shared: &SharedHarness) -> (SimPwm, SimCurrent, SimBus, SimRotor) {
    (
        SimPwm {
            shared: Rc::clone(shared),
        },
        SimCurrent {
            shared: Rc::clone(shared),
        },
        SimBus {
            shared: Rc::clone(shared),
        },
        SimRotor {
            shared: Rc::clone(shared),
        },
    )
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
                mechanical_angle: ContinuousMechanicalAngle::new(0.4),
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
    let (pwm, current, bus, rotor) = calibration_hardware(&shared);
    let params = plant_params();
    let mut system = MotorCalibrationSystem::new(
        pwm,
        current,
        bus,
        rotor,
        fluxkit::math::Svpwm,
        MotorCalibrationRequest {
            pole_pairs: Some(params.pole_pairs),
            electrical_angle_offset: Some(ElectricalAngle::new(0.0)),
            phase_resistance_ohm: None,
            phase_inductance_h: None,
            flux_linkage_weber: Some(params.flux_linkage_weber),
        },
        MotorCalibrationLimits {
            max_align_voltage_mag: Volts::new(1.0),
            max_spin_voltage_mag: Volts::new(3.0),
            max_electrical_velocity: RadPerSec::new(60.0),
            timeout_seconds: 2.0,
        },
    )
    .unwrap();

    let result = loop {
        if let Some(result) = system.tick(FAST_DT_SECONDS).unwrap() {
            break result;
        }
    };

    assert!((result.phase_resistance_ohm.get() - params.phase_resistance_ohm.get()).abs() < 0.01);
    assert!((result.phase_inductance_h.get() - params.d_inductance_h.get()).abs() < 3.0e-6);
}

#[test]
fn motor_calibration_results_apply_through_public_record() {
    let sweep_result = fluxkit::core::PolePairsAndOffsetCalibrationResult {
        pole_pairs: 7,
        electrical_angle_offset: ElectricalAngle::new(0.15),
    };
    let resistance_result = fluxkit::core::PhaseResistanceCalibrationResult {
        phase_resistance_ohm: Ohms::new(0.12),
    };
    let inductance_result = fluxkit::core::PhaseInductanceCalibrationResult {
        phase_inductance_h: Henries::new(30.0e-6),
    };
    let flux_linkage_result = fluxkit::core::FluxLinkageCalibrationResult {
        flux_linkage_weber: Webers::new(0.005),
    };

    let calibration = MotorCalibration::empty()
        .merge(sweep_result.into())
        .merge(resistance_result.into())
        .merge(inductance_result.into())
        .merge(flux_linkage_result.into());

    let mut motor = MotorParams::from_model_and_limits(
        MotorModel {
            pole_pairs: 1,
            phase_resistance_ohm: Ohms::new(0.5),
            d_inductance_h: Henries::new(1.0e-3),
            q_inductance_h: Henries::new(1.0e-3),
            flux_linkage_weber: Webers::new(0.001),
            electrical_angle_offset: ElectricalAngle::new(0.0),
        },
        MotorLimits {
            max_phase_current: Amps::new(10.0),
            max_mech_speed: None,
        },
    );
    calibration.apply_to_motor_params(&mut motor);

    assert_eq!(motor.pole_pairs, 7);
    assert_eq!(motor.electrical_angle_offset, ElectricalAngle::new(0.15));
    assert_eq!(motor.phase_resistance_ohm, Ohms::new(0.12));
    assert_eq!(motor.d_inductance_h, Henries::new(30.0e-6));
    assert_eq!(motor.q_inductance_h, Henries::new(30.0e-6));
    assert_eq!(motor.flux_linkage_weber, Webers::new(0.005));
}

#[test]
fn motor_calibration_builds_params_from_limits() {
    let calibration = fluxkit::MotorCalibrationResult {
        pole_pairs: 7,
        electrical_angle_offset: ElectricalAngle::new(0.15),
        phase_resistance_ohm: Ohms::new(0.12),
        phase_inductance_h: Henries::new(30.0e-6),
        flux_linkage_weber: Webers::new(0.005),
    };

    let motor = calibration.into_motor_params(MotorLimits {
        max_phase_current: Amps::new(10.0),
        max_mech_speed: Some(RadPerSec::new(150.0)),
    });

    assert_eq!(motor.pole_pairs, 7);
    assert_eq!(motor.electrical_angle_offset, ElectricalAngle::new(0.15));
    assert_eq!(motor.phase_resistance_ohm, Ohms::new(0.12));
    assert_eq!(motor.d_inductance_h, Henries::new(30.0e-6));
    assert_eq!(motor.q_inductance_h, Henries::new(30.0e-6));
    assert_eq!(motor.flux_linkage_weber, Webers::new(0.005));
    assert_eq!(motor.limits.max_phase_current, Amps::new(10.0));
}

#[test]
fn motor_calibration_system_recovers_pole_pairs_and_offset_through_public_wrapper() {
    let params = plant_params();
    let shared = Rc::new(RefCell::new(SimHarness {
        plant: PmsmModel::new(
            params,
            PmsmState {
                mechanical_angle: ContinuousMechanicalAngle::new(0.9),
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
    let (pwm, current, bus, rotor) = calibration_hardware(&shared);
    let mut system = MotorCalibrationSystem::new(
        pwm,
        current,
        bus,
        rotor,
        fluxkit::math::Svpwm,
        MotorCalibrationRequest {
            pole_pairs: None,
            electrical_angle_offset: None,
            phase_resistance_ohm: Some(params.phase_resistance_ohm),
            phase_inductance_h: Some(params.d_inductance_h),
            flux_linkage_weber: Some(params.flux_linkage_weber),
        },
        MotorCalibrationLimits {
            max_align_voltage_mag: Volts::new(2.5),
            max_spin_voltage_mag: Volts::new(3.0),
            max_electrical_velocity: RadPerSec::new(10.0),
            timeout_seconds: 4.0,
        },
    )
    .unwrap();
    let align_stator_angle =
        fluxkit::core::PolePairsAndOffsetCalibrationConfig::default_for_sweep().align_stator_angle;

    let result = loop {
        if let Some(result) = system.tick(FAST_DT_SECONDS).unwrap() {
            break result;
        }
    };

    assert_eq!(result.pole_pairs, params.pole_pairs);
    let harness = shared.borrow();
    let last_mechanical_angle = ContinuousMechanicalAngle::new(fluxkit::math::angle::wrap(
        harness.plant.state().mechanical_angle.get() + harness.rotor_bias,
    ));
    let reconstructed = fluxkit::math::angle::wrap(
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
                mechanical_angle: ContinuousMechanicalAngle::new(0.4),
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
    let (pwm, current, bus, rotor) = calibration_hardware(&shared);
    let mut system = MotorCalibrationSystem::new(
        pwm,
        current,
        bus,
        rotor,
        fluxkit::math::Svpwm,
        MotorCalibrationRequest {
            pole_pairs: Some(params.pole_pairs),
            electrical_angle_offset: Some(ElectricalAngle::new(0.0)),
            phase_resistance_ohm: Some(params.phase_resistance_ohm),
            phase_inductance_h: Some(params.d_inductance_h),
            flux_linkage_weber: None,
        },
        MotorCalibrationLimits {
            max_align_voltage_mag: Volts::new(2.0),
            max_spin_voltage_mag: Volts::new(2.0),
            max_electrical_velocity: RadPerSec::new(20.0),
            timeout_seconds: 2.0,
        },
    )
    .unwrap();

    let result = loop {
        if let Some(result) = system.tick(FAST_DT_SECONDS).unwrap() {
            break result;
        }
    };

    assert!((result.flux_linkage_weber.get() - params.flux_linkage_weber.get()).abs() < 1.0e-4);
}

#[test]
fn actuator_calibration_system_recovers_gear_ratio_through_public_wrapper() {
    let shared = Rc::new(RefCell::new(SimHarness {
        plant: PmsmModel::new_zeroed(actuator_friction_plant_params()).unwrap(),
        bus_voltage: Volts::new(24.0),
        load_torque: NewtonMeters::ZERO,
        last_duty: centered_phase_duty(),
        rotor_bias: 0.0,
    }));

    let mut system = ActuatorCalibrationSystem::new(
        motor_hardware(&shared),
        controller_motor_params(),
        inverter_params(),
        current_loop_config(),
        fluxkit::math::Svpwm,
        fluxkit::PassThroughEstimator::new(),
        fluxkit::PassThroughEstimator::new(),
        ActuatorCalibrationRequest {
            gear_ratio: None,
            positive_coulomb_torque: Some(NewtonMeters::new(0.04)),
            negative_coulomb_torque: Some(NewtonMeters::new(0.05)),
            positive_viscous_coefficient: Some(0.02),
            negative_viscous_coefficient: Some(0.03),
            positive_breakaway_torque: Some(NewtonMeters::ZERO),
            negative_breakaway_torque: Some(NewtonMeters::ZERO),
            zero_velocity_blend_band: Some(RadPerSec::new(0.05)),
        },
        ActuatorCalibrationLimits {
            max_velocity_target: RadPerSec::new(10.0),
            max_torque_target: NewtonMeters::new(0.3),
            timeout_seconds: 5.0,
        },
    )
    .unwrap();

    let result = loop {
        if let Some(result) = system
            .tick(FAST_DT_SECONDS, TickSchedule::with_medium(FAST_DT_SECONDS))
            .unwrap()
        {
            break result;
        }
    };

    assert!((result.gear_ratio - GEAR_RATIO).abs() < 0.05);
    assert!(
        (system
            .motor_system()
            .controller()
            .actuator_params()
            .gear_ratio
            - GEAR_RATIO)
            .abs()
            < 0.05
    );
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

    let mut system = ActuatorCalibrationSystem::new(
        motor_hardware(&shared),
        controller_motor_params(),
        inverter_params(),
        current_loop_config(),
        fluxkit::math::Svpwm,
        fluxkit::PassThroughEstimator::new(),
        fluxkit::PassThroughEstimator::new(),
        ActuatorCalibrationRequest {
            gear_ratio: Some(GEAR_RATIO),
            positive_coulomb_torque: Some(NewtonMeters::new(0.04)),
            negative_coulomb_torque: Some(NewtonMeters::new(0.05)),
            positive_viscous_coefficient: Some(0.0),
            negative_viscous_coefficient: Some(0.0),
            positive_breakaway_torque: None,
            negative_breakaway_torque: None,
            zero_velocity_blend_band: Some(RadPerSec::new(0.05)),
        },
        ActuatorCalibrationLimits {
            max_velocity_target: RadPerSec::new(10.0),
            max_torque_target: NewtonMeters::new(0.3),
            timeout_seconds: 5.0,
        },
    )
    .unwrap();

    let result = loop {
        if let Some(result) = system
            .tick(FAST_DT_SECONDS, TickSchedule::with_medium(FAST_DT_SECONDS))
            .unwrap()
        {
            break result;
        }
    };
    assert!((result.positive_breakaway_torque.get() - 0.08).abs() < 0.02);
    assert!((result.negative_breakaway_torque.get() - 0.09).abs() < 0.02);
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

    let mut system = ActuatorCalibrationSystem::new(
        motor_hardware(&shared),
        controller_motor_params(),
        inverter_params(),
        current_loop_config(),
        fluxkit::math::Svpwm,
        fluxkit::PassThroughEstimator::new(),
        fluxkit::PassThroughEstimator::new(),
        ActuatorCalibrationRequest {
            gear_ratio: Some(GEAR_RATIO),
            positive_coulomb_torque: Some(NewtonMeters::new(0.04)),
            negative_coulomb_torque: Some(NewtonMeters::new(0.05)),
            positive_viscous_coefficient: Some(0.0),
            negative_viscous_coefficient: Some(0.0),
            positive_breakaway_torque: Some(NewtonMeters::new(0.08)),
            negative_breakaway_torque: Some(NewtonMeters::new(0.09)),
            zero_velocity_blend_band: None,
        },
        ActuatorCalibrationLimits {
            max_velocity_target: RadPerSec::new(10.0),
            max_torque_target: NewtonMeters::new(0.3),
            timeout_seconds: 5.0,
        },
    )
    .unwrap();

    let result = loop {
        if let Some(result) = system
            .tick(FAST_DT_SECONDS, TickSchedule::with_medium(FAST_DT_SECONDS))
            .unwrap()
        {
            break result;
        }
    };
    assert!((result.zero_velocity_blend_band.get() - 0.05).abs() < 0.01);
}

#[test]
fn motor_calibration_system_runs_request_driven_campaign() {
    let params = plant_params();
    let shared = Rc::new(RefCell::new(SimHarness {
        plant: PmsmModel::new(
            params,
            PmsmState {
                mechanical_angle: ContinuousMechanicalAngle::new(0.4),
                mechanical_velocity: RadPerSec::ZERO,
                current_dq: fluxkit::Dq::new(Amps::ZERO, Amps::ZERO),
            },
        )
        .unwrap(),
        bus_voltage: Volts::new(24.0),
        load_torque: NewtonMeters::ZERO,
        last_duty: centered_phase_duty(),
        rotor_bias: 0.21,
    }));
    let (pwm, current, bus, rotor) = calibration_hardware(&shared);
    let mut system = MotorCalibrationSystem::new(
        pwm,
        current,
        bus,
        rotor,
        fluxkit::math::Svpwm,
        MotorCalibrationRequest::default(),
        MotorCalibrationLimits {
            max_align_voltage_mag: Volts::new(2.0),
            max_spin_voltage_mag: Volts::new(3.0),
            max_electrical_velocity: RadPerSec::new(60.0),
            timeout_seconds: 6.0,
        },
    )
    .unwrap();

    let result = loop {
        if let Some(result) = system.tick(FAST_DT_SECONDS).unwrap() {
            break result;
        }
    };

    assert_eq!(result.pole_pairs, params.pole_pairs);
    assert!((result.phase_resistance_ohm.get() - params.phase_resistance_ohm.get()).abs() < 0.01);
    assert!((result.phase_inductance_h.get() - params.d_inductance_h.get()).abs() < 3.0e-6);
    assert!((result.flux_linkage_weber.get() - params.flux_linkage_weber.get()).abs() < 1.0e-4);
    assert!(result.electrical_angle_offset.get().is_finite());
}

#[test]
fn actuator_calibration_system_applies_provided_and_measured_values_through_live_controller() {
    let shared = Rc::new(RefCell::new(SimHarness {
        plant: PmsmModel::new_zeroed(actuator_breakaway_plant_params()).unwrap(),
        bus_voltage: Volts::new(24.0),
        load_torque: NewtonMeters::ZERO,
        last_duty: centered_phase_duty(),
        rotor_bias: 0.0,
    }));

    let mut system = ActuatorCalibrationSystem::new(
        motor_hardware(&shared),
        controller_motor_params(),
        inverter_params(),
        current_loop_config(),
        fluxkit::math::Svpwm,
        fluxkit::PassThroughEstimator::new(),
        fluxkit::PassThroughEstimator::new(),
        ActuatorCalibrationRequest {
            gear_ratio: Some(GEAR_RATIO),
            positive_coulomb_torque: Some(NewtonMeters::new(0.04)),
            negative_coulomb_torque: Some(NewtonMeters::new(0.05)),
            positive_viscous_coefficient: Some(0.0),
            negative_viscous_coefficient: Some(0.0),
            positive_breakaway_torque: None,
            negative_breakaway_torque: None,
            zero_velocity_blend_band: None,
        },
        ActuatorCalibrationLimits {
            max_velocity_target: RadPerSec::new(10.0),
            max_torque_target: NewtonMeters::new(0.3),
            timeout_seconds: 5.0,
        },
    )
    .unwrap();

    let result = loop {
        if let Some(result) = system
            .tick(FAST_DT_SECONDS, TickSchedule::with_medium(FAST_DT_SECONDS))
            .unwrap()
        {
            break result;
        }
    };

    assert!((result.gear_ratio - GEAR_RATIO).abs() < 1.0e-6);
    assert!((result.positive_coulomb_torque.get() - 0.04).abs() < 1.0e-6);
    assert!((result.negative_coulomb_torque.get() - 0.05).abs() < 1.0e-6);
    assert!(result.positive_breakaway_torque.get().is_finite());
    assert!(result.zero_velocity_blend_band.get().is_finite());
    assert!(
        (system
            .motor_system()
            .controller()
            .actuator_params()
            .gear_ratio
            - GEAR_RATIO)
            .abs()
            < 1.0e-6
    );
    assert!(
        (system
            .motor_system()
            .controller()
            .actuator_params()
            .compensation
            .friction
            .positive_coulomb_torque
            .get()
            - 0.04)
            .abs()
            < 1.0e-6
    );
}

#[test]
fn actuator_calibration_builds_compensated_params_from_limits() {
    let calibration = fluxkit::ActuatorCalibrationResult {
        gear_ratio: GEAR_RATIO,
        positive_breakaway_torque: NewtonMeters::new(0.08),
        negative_breakaway_torque: NewtonMeters::new(0.09),
        positive_coulomb_torque: NewtonMeters::new(0.04),
        negative_coulomb_torque: NewtonMeters::new(0.05),
        positive_viscous_coefficient: 0.02,
        negative_viscous_coefficient: 0.03,
        zero_velocity_blend_band: RadPerSec::new(0.05),
    };

    let actuator = calibration.into_friction_compensated_actuator_params(
        ActuatorLimits {
            max_output_velocity: Some(RadPerSec::new(30.0)),
            max_output_torque: Some(NewtonMeters::new(10.0)),
        },
        NewtonMeters::new(0.3),
    );

    assert!((actuator.gear_ratio - GEAR_RATIO).abs() < 1.0e-6);
    assert!(actuator.compensation.friction.enabled);
    assert_eq!(
        actuator.compensation.max_total_torque,
        NewtonMeters::new(0.3)
    );
    assert_eq!(
        actuator.compensation.friction.positive_breakaway_torque,
        NewtonMeters::new(0.08)
    );
    assert_eq!(
        actuator.compensation.friction.negative_coulomb_torque,
        NewtonMeters::new(0.05)
    );
    assert_eq!(
        actuator.compensation.friction.positive_viscous_coefficient,
        0.02
    );
    assert_eq!(
        actuator.compensation.friction.zero_velocity_blend_band,
        RadPerSec::new(0.05)
    );
    assert_eq!(
        actuator.limits.max_output_velocity,
        Some(RadPerSec::new(30.0))
    );
}
