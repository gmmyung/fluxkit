use std::{
    cell::RefCell,
    rc::Rc,
    sync::{Arc, Mutex, mpsc},
    thread,
};

use fluxkit::{
    Abc, ActuatorCalibrationLimits, ActuatorCalibrationPhase, ActuatorCalibrationRequest,
    ActuatorCalibrationRuntime, ActuatorLimits, BusVoltageSensor, ContinuousMechanicalAngle,
    CurrentLoopConfig, CurrentSampleValidity, CurrentSampler, ElectricalAngle, ElectricalDirection,
    InverterParams, MechanicalAngle, MotorCalibrationLimits, MotorCalibrationPhase,
    MotorCalibrationRequest, MotorCalibrationRuntime, MotorCommand, MotorLimits, MotorModel,
    MotorParams, MotorRuntime, OutputReading, OutputSensor, PassThroughEstimator,
    PhaseCurrentSample, PhasePwm, RadPerSec, RotorReading, RotorSensor, Svpwm, TemperatureSensor,
    units::{Amps, Duty, Henries, Hertz, NewtonMeters, Ohms, Volts, Webers},
};
use fluxkit_core::{
    FluxLinkageCalibrationResult, MotorCalibration, PhaseInductanceCalibrationResult,
    PhaseResistanceCalibrationResult, PolePairsAndOffsetCalibrationConfig,
    PolePairsAndOffsetCalibrationResult,
};
use fluxkit_hal::centered_phase_duty;
use fluxkit_math::{inverse_clarke, inverse_park};
use fluxkit_pmsm_sim::{ActuatorPlantParams, PmsmModel, PmsmParams, PmsmState, ThermalPlantParams};

const FAST_DT_SECONDS: f32 = 1.0 / 20_000.0;
const GEAR_RATIO: f32 = 2.0;
const WINDING_TEMP_C: f32 = 25.0;

fn plant_params() -> PmsmParams {
    PmsmParams {
        pole_pairs: 7,
        phase_resistance_ohm_ref: Ohms::new(0.12),
        d_inductance_h: Henries::new(0.000_03),
        q_inductance_h: Henries::new(0.000_03),
        flux_linkage_weber: Webers::new(0.005),
        electrical_direction: ElectricalDirection::Positive,
        thermal: ThermalPlantParams::default_for_ambient(WINDING_TEMP_C),
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
        phase_resistance_ohm_ref: Ohms::new(0.12),
        d_inductance_h: Henries::new(0.000_03),
        q_inductance_h: Henries::new(0.000_03),
        flux_linkage_weber: Webers::new(0.005),
        electrical_direction: ElectricalDirection::Positive,
        thermal: ThermalPlantParams::default_for_ambient(WINDING_TEMP_C),
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
        phase_resistance_ohm_ref: Ohms::new(0.12),
        d_inductance_h: Henries::new(0.000_03),
        q_inductance_h: Henries::new(0.000_03),
        flux_linkage_weber: Webers::new(0.005),
        electrical_direction: fluxkit_math::ElectricalDirection::Positive,
        thermal: ThermalPlantParams::default_for_ambient(WINDING_TEMP_C),
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
            phase_resistance_ohm_ref: Ohms::new(0.12),
            d_inductance_h: Henries::new(0.000_03),
            q_inductance_h: Henries::new(0.000_03),
            flux_linkage_weber: Webers::new(0.005),
            electrical_direction: ElectricalDirection::Positive,
            electrical_angle_offset: ElectricalAngle::new(0.0),
        },
        MotorLimits {
            max_phase_current: Amps::new(10.0),
            max_mech_speed: Some(RadPerSec::new(150.0)),
            max_winding_temperature_c: None,
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
    last_duty: Abc<Duty>,
    rotor_bias: f32,
}

type SharedHarness = Rc<RefCell<SimHarness>>;
type SharedThreadHarness = Arc<Mutex<SimHarness>>;

fn phase_currents(shared: &SharedHarness) -> Abc<Amps> {
    let harness = shared.borrow();
    let state = *harness.plant.state();
    let electrical_angle = fluxkit::angle::mechanical_to_electrical_with_direction(
        state.mechanical_angle.wrapped().into(),
        harness.plant.params().pole_pairs as u32,
        harness.plant.params().electrical_direction,
    );
    inverse_clarke(inverse_park(
        state.current_dq.map(|current| current.get()),
        electrical_angle.get(),
    ))
    .map(Amps::new)
}

fn phase_currents_thread(shared: &SharedThreadHarness) -> Abc<Amps> {
    let harness = shared.lock().unwrap();
    let state = *harness.plant.state();
    let electrical_angle = fluxkit::angle::mechanical_to_electrical_with_direction(
        state.mechanical_angle.wrapped().into(),
        harness.plant.params().pole_pairs as u32,
        harness.plant.params().electrical_direction,
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
        let duty = Abc::new(a, b, c);
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
            mechanical_angle: MechanicalAngle::new(
                state.mechanical_angle.get() + harness.rotor_bias,
            ),
            mechanical_velocity: state.mechanical_velocity,
        })
    }
}

#[derive(Clone, Debug)]
struct SimTemp {
    shared: SharedHarness,
}

impl TemperatureSensor for SimTemp {
    type Error = core::convert::Infallible;

    fn sample_temperature_c(&mut self) -> Result<f32, Self::Error> {
        Ok(self.shared.borrow().plant.winding_temperature_c())
    }
}

#[derive(Clone, Debug)]
struct ThreadedSimPwm {
    shared: SharedThreadHarness,
}

impl PhasePwm for ThreadedSimPwm {
    type Error = core::convert::Infallible;

    fn enable(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }

    fn disable(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }

    fn set_duty(&mut self, a: Duty, b: Duty, c: Duty) -> Result<(), Self::Error> {
        let duty = Abc::new(a, b, c);
        let mut harness = self.shared.lock().unwrap();
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
struct ThreadedSimCurrent {
    shared: SharedThreadHarness,
}

impl CurrentSampler for ThreadedSimCurrent {
    type Error = core::convert::Infallible;

    fn sample_phase_currents(&mut self) -> Result<PhaseCurrentSample, Self::Error> {
        Ok(PhaseCurrentSample {
            currents: phase_currents_thread(&self.shared),
            validity: CurrentSampleValidity::Valid,
        })
    }
}

#[derive(Clone, Debug)]
struct ThreadedSimBus {
    shared: SharedThreadHarness,
}

impl BusVoltageSensor for ThreadedSimBus {
    type Error = core::convert::Infallible;

    fn sample_bus_voltage(&mut self) -> Result<Volts, Self::Error> {
        Ok(self.shared.lock().unwrap().bus_voltage)
    }
}

#[derive(Clone, Debug)]
struct ThreadedSimRotor {
    shared: SharedThreadHarness,
}

impl RotorSensor for ThreadedSimRotor {
    type Error = core::convert::Infallible;

    fn read_rotor(&mut self) -> Result<RotorReading, Self::Error> {
        let harness = self.shared.lock().unwrap();
        let state = *harness.plant.state();
        Ok(RotorReading {
            mechanical_angle: ContinuousMechanicalAngle::new(fluxkit::angle::wrap(
                state.mechanical_angle.get() + harness.rotor_bias,
            ))
            .wrapped(),
            mechanical_velocity: state.mechanical_velocity,
        })
    }
}

#[derive(Clone, Debug)]
struct ThreadedSimTemp {
    shared: SharedThreadHarness,
}

impl TemperatureSensor for ThreadedSimTemp {
    type Error = core::convert::Infallible;

    fn sample_temperature_c(&mut self) -> Result<f32, Self::Error> {
        Ok(self.shared.lock().unwrap().plant.winding_temperature_c())
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

#[derive(Clone, Debug)]
struct SimOutputInverted {
    shared: SharedHarness,
}

impl OutputSensor for SimOutputInverted {
    type Error = core::convert::Infallible;

    fn read_output(&mut self) -> Result<OutputReading, Self::Error> {
        let harness = self.shared.borrow();
        let state = *harness.plant.state();
        Ok(OutputReading {
            mechanical_angle: ContinuousMechanicalAngle::new(
                -(state.mechanical_angle.get() / GEAR_RATIO),
            )
            .wrapped(),
            mechanical_velocity: RadPerSec::new(-(state.mechanical_velocity.get() / GEAR_RATIO)),
        })
    }
}

fn calibration_hardware(shared: &SharedHarness) -> (SimPwm, SimCurrent, SimBus, SimRotor, SimTemp) {
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
        SimTemp {
            shared: Rc::clone(shared),
        },
    )
}

fn threaded_calibration_hardware(
    shared: &SharedThreadHarness,
) -> (
    ThreadedSimPwm,
    ThreadedSimCurrent,
    ThreadedSimBus,
    ThreadedSimRotor,
    ThreadedSimTemp,
) {
    (
        ThreadedSimPwm {
            shared: Arc::clone(shared),
        },
        ThreadedSimCurrent {
            shared: Arc::clone(shared),
        },
        ThreadedSimBus {
            shared: Arc::clone(shared),
        },
        ThreadedSimRotor {
            shared: Arc::clone(shared),
        },
        ThreadedSimTemp {
            shared: Arc::clone(shared),
        },
    )
}

fn runtime_handles(
    shared: &SharedHarness,
) -> (SimPwm, SimCurrent, SimBus, SimRotor, SimOutput, SimTemp) {
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
        SimOutput {
            shared: Rc::clone(shared),
        },
        SimTemp {
            shared: Rc::clone(shared),
        },
    )
}

#[test]
fn motor_calibration_runtime_recovers_phase_resistance_and_inductance() {
    let shared = Rc::new(RefCell::new(SimHarness {
        plant: PmsmModel::new(
            plant_params(),
            PmsmState {
                mechanical_angle: ContinuousMechanicalAngle::new(0.4),
                mechanical_velocity: RadPerSec::ZERO,
                current_dq: fluxkit::Dq::new(Amps::ZERO, Amps::ZERO),
                winding_temperature_c: WINDING_TEMP_C,
            },
        )
        .unwrap(),
        bus_voltage: Volts::new(24.0),
        load_torque: NewtonMeters::ZERO,
        last_duty: centered_phase_duty(),
        rotor_bias: 0.0,
    }));
    let (pwm, current, bus, rotor, temp) = calibration_hardware(&shared);
    let params = plant_params();
    let system = MotorCalibrationRuntime::new(
        pwm,
        current,
        bus,
        rotor,
        temp,
        Svpwm,
        PassThroughEstimator::new(),
        MotorCalibrationRequest {
            pole_pairs: Some(params.pole_pairs),
            electrical_direction: Some(params.electrical_direction),
            electrical_angle_offset: Some(ElectricalAngle::new(0.0)),
            phase_resistance_ohm_ref: None,
            phase_inductance_h: None,
            flux_linkage_weber: Some(params.flux_linkage_weber),
        },
        MotorCalibrationLimits {
            max_align_voltage_mag: Volts::new(1.0),
            max_spin_voltage_mag: Volts::new(3.0),
            max_electrical_velocity: RadPerSec::new(60.0),
            timeout_seconds: 2.0,
        },
        FAST_DT_SECONDS,
    )
    .unwrap();

    let (handle, ticker) = system.split().expect("calibration should split once");
    let result = loop {
        ticker.tick().unwrap();
        if let Some(result) = handle.status().result {
            break result;
        }
    };

    assert!(
        (result.phase_resistance_ohm_ref.get() - params.phase_resistance_ohm_ref.get()).abs()
            < 0.01
    );
    assert!((result.phase_inductance_h.get() - params.d_inductance_h.get()).abs() < 3.0e-6);
}

#[test]
fn motor_calibration_results_apply_through_public_record() {
    let sweep_result = PolePairsAndOffsetCalibrationResult {
        pole_pairs: 7,
        electrical_direction: ElectricalDirection::Positive,
        electrical_angle_offset: ElectricalAngle::new(0.15),
    };
    let resistance_result = PhaseResistanceCalibrationResult {
        phase_resistance_ohm_ref: Ohms::new(0.12),
    };
    let inductance_result = PhaseInductanceCalibrationResult {
        phase_inductance_h: Henries::new(30.0e-6),
    };
    let flux_linkage_result = FluxLinkageCalibrationResult {
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
            phase_resistance_ohm_ref: Ohms::new(0.5),
            d_inductance_h: Henries::new(1.0e-3),
            q_inductance_h: Henries::new(1.0e-3),
            flux_linkage_weber: Webers::new(0.001),
            electrical_direction: ElectricalDirection::Positive,
            electrical_angle_offset: ElectricalAngle::new(0.0),
        },
        MotorLimits {
            max_phase_current: Amps::new(10.0),
            max_mech_speed: None,
            max_winding_temperature_c: None,
        },
    );
    calibration.apply_to_motor_params(&mut motor);

    assert_eq!(motor.pole_pairs, 7);
    assert_eq!(motor.electrical_direction, ElectricalDirection::Positive);
    assert_eq!(motor.electrical_angle_offset, ElectricalAngle::new(0.15));
    assert_eq!(motor.phase_resistance_ohm_ref, Ohms::new(0.12));
    assert_eq!(motor.d_inductance_h, Henries::new(30.0e-6));
    assert_eq!(motor.q_inductance_h, Henries::new(30.0e-6));
    assert_eq!(motor.flux_linkage_weber, Webers::new(0.005));
}

#[test]
fn motor_calibration_builds_params_from_limits() {
    let calibration = fluxkit::MotorCalibrationResult {
        pole_pairs: 7,
        electrical_direction: ElectricalDirection::Positive,
        electrical_angle_offset: ElectricalAngle::new(0.15),
        phase_resistance_ohm_ref: Ohms::new(0.12),
        phase_inductance_h: Henries::new(30.0e-6),
        flux_linkage_weber: Webers::new(0.005),
    };

    let motor = calibration.into_motor_params(MotorLimits {
        max_phase_current: Amps::new(10.0),
        max_mech_speed: Some(RadPerSec::new(150.0)),
        max_winding_temperature_c: None,
    });

    assert_eq!(motor.pole_pairs, 7);
    assert_eq!(motor.electrical_direction, ElectricalDirection::Positive);
    assert_eq!(motor.electrical_angle_offset, ElectricalAngle::new(0.15));
    assert_eq!(motor.phase_resistance_ohm_ref, Ohms::new(0.12));
    assert_eq!(motor.d_inductance_h, Henries::new(30.0e-6));
    assert_eq!(motor.q_inductance_h, Henries::new(30.0e-6));
    assert_eq!(motor.flux_linkage_weber, Webers::new(0.005));
    assert_eq!(motor.limits.max_phase_current, Amps::new(10.0));
}

#[test]
fn motor_calibration_runtime_recovers_pole_pairs_and_offset_through_public_wrapper() {
    let params = plant_params();
    let shared = Rc::new(RefCell::new(SimHarness {
        plant: PmsmModel::new(
            params,
            PmsmState {
                mechanical_angle: ContinuousMechanicalAngle::new(0.9),
                mechanical_velocity: RadPerSec::ZERO,
                current_dq: fluxkit::Dq::new(Amps::ZERO, Amps::ZERO),
                winding_temperature_c: WINDING_TEMP_C,
            },
        )
        .unwrap(),
        bus_voltage: Volts::new(24.0),
        load_torque: NewtonMeters::ZERO,
        last_duty: centered_phase_duty(),
        rotor_bias: -0.11,
    }));
    let (pwm, current, bus, rotor, temp) = calibration_hardware(&shared);
    let system = MotorCalibrationRuntime::new(
        pwm,
        current,
        bus,
        rotor,
        temp,
        Svpwm,
        PassThroughEstimator::new(),
        MotorCalibrationRequest {
            pole_pairs: None,
            electrical_direction: None,
            electrical_angle_offset: None,
            phase_resistance_ohm_ref: Some(params.phase_resistance_ohm_ref),
            phase_inductance_h: Some(params.d_inductance_h),
            flux_linkage_weber: Some(params.flux_linkage_weber),
        },
        MotorCalibrationLimits {
            max_align_voltage_mag: Volts::new(2.5),
            max_spin_voltage_mag: Volts::new(3.0),
            max_electrical_velocity: RadPerSec::new(10.0),
            timeout_seconds: 4.0,
        },
        FAST_DT_SECONDS,
    )
    .unwrap();
    let align_stator_angle =
        PolePairsAndOffsetCalibrationConfig::default_for_sweep().align_stator_angle;

    let (handle, ticker) = system.split().expect("calibration should split once");
    let result = loop {
        ticker.tick().unwrap();
        if let Some(result) = handle.status().result {
            break result;
        }
    };

    assert_eq!(result.pole_pairs, params.pole_pairs);
    assert_eq!(result.electrical_direction, params.electrical_direction);
    let harness = shared.borrow();
    let last_mechanical_angle = ContinuousMechanicalAngle::new(fluxkit::angle::wrap(
        harness.plant.state().mechanical_angle.get() + harness.rotor_bias,
    ));
    let reconstructed = fluxkit::angle::wrap(
        fluxkit::angle::mechanical_to_electrical_with_direction(
            last_mechanical_angle,
            result.pole_pairs as u32,
            result.electrical_direction,
        )
        .get()
            + result.electrical_angle_offset.get()
            - align_stator_angle.get(),
    );
    assert!(reconstructed.abs() < 0.03);
}

#[test]
fn motor_calibration_runtime_recovers_negative_electrical_direction() {
    let mut params = plant_params();
    params.electrical_direction = ElectricalDirection::Negative;
    let shared = Rc::new(RefCell::new(SimHarness {
        plant: PmsmModel::new_zeroed(params).unwrap(),
        bus_voltage: Volts::new(24.0),
        load_torque: NewtonMeters::ZERO,
        last_duty: centered_phase_duty(),
        rotor_bias: 0.0,
    }));

    let (pwm, current, bus, rotor, temp) = calibration_hardware(&shared);
    let system = MotorCalibrationRuntime::new(
        pwm,
        current,
        bus,
        rotor,
        temp,
        Svpwm,
        PassThroughEstimator::new(),
        MotorCalibrationRequest {
            pole_pairs: None,
            electrical_direction: None,
            electrical_angle_offset: None,
            phase_resistance_ohm_ref: Some(params.phase_resistance_ohm_ref),
            phase_inductance_h: Some(params.d_inductance_h),
            flux_linkage_weber: Some(params.flux_linkage_weber),
        },
        MotorCalibrationLimits {
            max_align_voltage_mag: Volts::new(2.5),
            max_spin_voltage_mag: Volts::new(3.0),
            max_electrical_velocity: RadPerSec::new(10.0),
            timeout_seconds: 4.0,
        },
        FAST_DT_SECONDS,
    )
    .unwrap();

    let (handle, ticker) = system.split().expect("calibration should split once");
    let result = loop {
        ticker.tick().unwrap();
        if let Some(result) = handle.status().result {
            break result;
        }
    };

    assert_eq!(result.electrical_direction, ElectricalDirection::Negative);
}

#[test]
fn calibration_runtimes_report_current_or_next_phase() {
    let shared = Rc::new(RefCell::new(SimHarness {
        plant: PmsmModel::new_zeroed(plant_params()).unwrap(),
        bus_voltage: Volts::new(24.0),
        load_torque: NewtonMeters::ZERO,
        last_duty: centered_phase_duty(),
        rotor_bias: 0.0,
    }));

    let (pwm, current, bus, rotor, temp) = calibration_hardware(&shared);
    let motor = MotorCalibrationRuntime::new(
        pwm,
        current,
        bus,
        rotor,
        temp,
        Svpwm,
        PassThroughEstimator::new(),
        MotorCalibrationRequest {
            pole_pairs: None,
            electrical_direction: None,
            electrical_angle_offset: None,
            phase_resistance_ohm_ref: Some(Ohms::new(0.12)),
            phase_inductance_h: Some(Henries::new(0.000_03)),
            flux_linkage_weber: Some(Webers::new(0.005)),
        },
        MotorCalibrationLimits {
            max_align_voltage_mag: Volts::new(2.0),
            max_spin_voltage_mag: Volts::new(3.0),
            max_electrical_velocity: RadPerSec::new(60.0),
            timeout_seconds: 2.0,
        },
        FAST_DT_SECONDS,
    )
    .unwrap();
    let (motor_handle, _motor_ticker) = motor.split().expect("calibration should split once");
    assert_eq!(
        motor_handle.status().phase,
        Some(MotorCalibrationPhase::PolePairsAndOffset)
    );
    assert_eq!(motor_handle.status().result, None);

    let (pwm, current, bus, rotor, output, temp) = runtime_handles(&shared);
    let actuator = ActuatorCalibrationRuntime::new(
        pwm,
        current,
        bus,
        rotor,
        output,
        temp,
        controller_motor_params(),
        inverter_params(),
        current_loop_config(),
        Svpwm,
        PassThroughEstimator::new(),
        PassThroughEstimator::new(),
        ActuatorCalibrationRequest {
            gear_ratio: Some(GEAR_RATIO),
            positive_coulomb_torque: None,
            negative_coulomb_torque: None,
            positive_viscous_coefficient: None,
            negative_viscous_coefficient: None,
            positive_breakaway_torque: Some(NewtonMeters::new(0.08)),
            negative_breakaway_torque: Some(NewtonMeters::new(0.09)),
            zero_velocity_blend_band: Some(RadPerSec::new(0.05)),
        },
        ActuatorCalibrationLimits {
            max_velocity_target: RadPerSec::new(10.0),
            max_torque_target: NewtonMeters::new(0.3),
            timeout_seconds: 5.0,
        },
        FAST_DT_SECONDS,
    )
    .unwrap();
    let (actuator_handle, _actuator_ticker) =
        actuator.split().expect("calibration should split once");
    assert_eq!(
        actuator_handle.status().phase,
        Some(ActuatorCalibrationPhase::Friction)
    );
    assert_eq!(actuator_handle.status().result, None);
}

#[test]
fn calibration_handles_publish_completion() {
    let shared = Rc::new(RefCell::new(SimHarness {
        plant: PmsmModel::new_zeroed(plant_params()).unwrap(),
        bus_voltage: Volts::new(24.0),
        load_torque: NewtonMeters::ZERO,
        last_duty: centered_phase_duty(),
        rotor_bias: 0.0,
    }));

    let (pwm, current, bus, rotor, temp) = calibration_hardware(&shared);
    let motor = MotorCalibrationRuntime::new(
        pwm,
        current,
        bus,
        rotor,
        temp,
        Svpwm,
        PassThroughEstimator::new(),
        MotorCalibrationRequest::all(),
        MotorCalibrationLimits {
            max_align_voltage_mag: Volts::new(2.0),
            max_spin_voltage_mag: Volts::new(3.0),
            max_electrical_velocity: RadPerSec::new(60.0),
            timeout_seconds: 6.0,
        },
        FAST_DT_SECONDS,
    )
    .unwrap();

    let (motor_handle, motor_ticker) = motor.split().expect("calibration should split once");
    loop {
        motor_ticker.tick().unwrap();
        if motor_handle.status().result.is_some() {
            break;
        }
    }

    let motor_status = motor_handle.status();
    assert_eq!(motor_status.fault_latched, false);
    assert_eq!(motor_status.phase, None);
    let motor_result = motor_status
        .result
        .expect("motor result should be published");

    let motor_params = motor_result.into_motor_params(MotorLimits {
        max_phase_current: Amps::new(10.0),
        max_mech_speed: Some(RadPerSec::new(150.0)),
        max_winding_temperature_c: None,
    });

    let (pwm, current, bus, rotor, output, temp) = runtime_handles(&shared);
    let actuator = ActuatorCalibrationRuntime::new(
        pwm,
        current,
        bus,
        rotor,
        output,
        temp,
        motor_params,
        inverter_params(),
        current_loop_config(),
        Svpwm,
        PassThroughEstimator::new(),
        PassThroughEstimator::new(),
        ActuatorCalibrationRequest::all(),
        ActuatorCalibrationLimits {
            max_velocity_target: RadPerSec::new(10.0),
            max_torque_target: NewtonMeters::new(0.3),
            timeout_seconds: 20.0,
        },
        FAST_DT_SECONDS,
    )
    .unwrap();

    let (actuator_handle, actuator_ticker) =
        actuator.split().expect("calibration should split once");
    loop {
        actuator_ticker.tick().unwrap();
        if actuator_handle.status().result.is_some() {
            break;
        }
    }

    let actuator_status = actuator_handle.status();
    assert_eq!(actuator_status.fault_latched, false);
    assert_eq!(actuator_status.phase, None);
    assert!(actuator_status.result.is_some());
}

#[test]
fn motor_calibration_runtime_recovers_flux_linkage_through_public_wrapper() {
    let params = plant_params();
    let shared = Rc::new(RefCell::new(SimHarness {
        plant: PmsmModel::new(
            params,
            PmsmState {
                mechanical_angle: ContinuousMechanicalAngle::new(0.4),
                mechanical_velocity: RadPerSec::ZERO,
                current_dq: fluxkit::Dq::new(Amps::ZERO, Amps::ZERO),
                winding_temperature_c: WINDING_TEMP_C,
            },
        )
        .unwrap(),
        bus_voltage: Volts::new(24.0),
        load_torque: NewtonMeters::ZERO,
        last_duty: centered_phase_duty(),
        rotor_bias: 0.0,
    }));
    let (pwm, current, bus, rotor, temp) = calibration_hardware(&shared);
    let system = MotorCalibrationRuntime::new(
        pwm,
        current,
        bus,
        rotor,
        temp,
        Svpwm,
        PassThroughEstimator::new(),
        MotorCalibrationRequest {
            pole_pairs: Some(params.pole_pairs),
            electrical_direction: Some(params.electrical_direction),
            electrical_angle_offset: Some(ElectricalAngle::new(0.0)),
            phase_resistance_ohm_ref: Some(params.phase_resistance_ohm_ref),
            phase_inductance_h: Some(params.d_inductance_h),
            flux_linkage_weber: None,
        },
        MotorCalibrationLimits {
            max_align_voltage_mag: Volts::new(2.0),
            max_spin_voltage_mag: Volts::new(2.0),
            max_electrical_velocity: RadPerSec::new(20.0),
            timeout_seconds: 2.0,
        },
        FAST_DT_SECONDS,
    )
    .unwrap();

    let (handle, ticker) = system.split().expect("calibration should split once");
    let result = loop {
        ticker.tick().unwrap();
        if let Some(result) = handle.status().result {
            break result;
        }
    };

    assert!((result.flux_linkage_weber.get() - params.flux_linkage_weber.get()).abs() < 1.0e-4);
}

#[test]
fn actuator_calibration_runtime_recovers_gear_ratio_through_public_wrapper() {
    let shared = Rc::new(RefCell::new(SimHarness {
        plant: PmsmModel::new_zeroed(actuator_friction_plant_params()).unwrap(),
        bus_voltage: Volts::new(24.0),
        load_torque: NewtonMeters::ZERO,
        last_duty: centered_phase_duty(),
        rotor_bias: 0.0,
    }));

    let (pwm, current, bus, rotor, output, temp) = runtime_handles(&shared);
    let system = ActuatorCalibrationRuntime::new(
        pwm,
        current,
        bus,
        rotor,
        output,
        temp,
        controller_motor_params(),
        inverter_params(),
        current_loop_config(),
        Svpwm,
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
        FAST_DT_SECONDS,
    )
    .unwrap();

    let (handle, ticker) = system.split().expect("calibration should split once");
    let result = loop {
        ticker.tick().unwrap();
        if let Some(result) = handle.status().result {
            break result;
        }
    };

    assert!((result.gear_ratio - GEAR_RATIO).abs() < 0.05);
}

#[test]
fn actuator_calibration_runtime_faults_on_opposite_output_sensor_direction() {
    let shared = Rc::new(RefCell::new(SimHarness {
        plant: PmsmModel::new_zeroed(actuator_friction_plant_params()).unwrap(),
        bus_voltage: Volts::new(24.0),
        load_torque: NewtonMeters::ZERO,
        last_duty: centered_phase_duty(),
        rotor_bias: 0.0,
    }));

    let (pwm, current, bus, rotor, temp) = calibration_hardware(&shared);
    let output = SimOutputInverted {
        shared: Rc::clone(&shared),
    };
    let system = ActuatorCalibrationRuntime::new(
        pwm,
        current,
        bus,
        rotor,
        output,
        temp,
        controller_motor_params(),
        inverter_params(),
        current_loop_config(),
        Svpwm,
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
        FAST_DT_SECONDS,
    )
    .unwrap();

    let (handle, ticker) = system.split().expect("calibration should split once");
    let error = loop {
        match ticker.tick() {
            Ok(()) => {}
            Err(error) => break error,
        }
    };

    assert!(matches!(
        error,
        fluxkit::ActuatorCalibrationRuntimeError::Calibration(
            fluxkit::CalibrationError::OppositeDirection
        )
    ));
    assert!(handle.status().fault_latched);
}

#[test]
fn extracted_actuator_calibration_marks_handles_and_tickers_inactive() {
    let shared = Rc::new(RefCell::new(SimHarness {
        plant: PmsmModel::new_zeroed(actuator_breakaway_plant_params()).unwrap(),
        bus_voltage: Volts::new(24.0),
        load_torque: NewtonMeters::ZERO,
        last_duty: centered_phase_duty(),
        rotor_bias: 0.0,
    }));
    let (pwm, current, bus, rotor, temp) = calibration_hardware(&shared);
    let output = SimOutput {
        shared: Rc::clone(&shared),
    };

    let system = ActuatorCalibrationRuntime::new(
        pwm,
        current,
        bus,
        rotor,
        output,
        temp,
        controller_motor_params(),
        inverter_params(),
        current_loop_config(),
        Svpwm,
        PassThroughEstimator::new(),
        PassThroughEstimator::new(),
        ActuatorCalibrationRequest::all(),
        ActuatorCalibrationLimits {
            max_velocity_target: RadPerSec::new(10.0),
            max_torque_target: NewtonMeters::new(0.3),
            timeout_seconds: 5.0,
        },
        FAST_DT_SECONDS,
    )
    .unwrap();

    let (handle, ticker) = system.split().expect("calibration should split once");

    let _parts = system
        .try_into_parts()
        .expect("runtime parts should be available");

    assert!(!handle.status().active);
    assert!(matches!(
        ticker.tick(),
        Err(fluxkit::ActuatorCalibrationRuntimeError::Inactive)
    ));
}

#[test]
fn actuator_calibration_runtime_recovers_breakaway_through_public_wrapper() {
    let shared = Rc::new(RefCell::new(SimHarness {
        plant: PmsmModel::new_zeroed(actuator_breakaway_plant_params()).unwrap(),
        bus_voltage: Volts::new(24.0),
        load_torque: NewtonMeters::ZERO,
        last_duty: centered_phase_duty(),
        rotor_bias: 0.0,
    }));

    let (pwm, current, bus, rotor, output, temp) = runtime_handles(&shared);
    let system = ActuatorCalibrationRuntime::new(
        pwm,
        current,
        bus,
        rotor,
        output,
        temp,
        controller_motor_params(),
        inverter_params(),
        current_loop_config(),
        Svpwm,
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
        FAST_DT_SECONDS,
    )
    .unwrap();

    let (handle, ticker) = system.split().expect("calibration should split once");
    let result = loop {
        ticker.tick().unwrap();
        if let Some(result) = handle.status().result {
            break result;
        }
    };
    assert!((result.positive_breakaway_torque.get() - 0.08).abs() < 0.02);
    assert!((result.negative_breakaway_torque.get() - 0.09).abs() < 0.02);
}

#[test]
fn actuator_calibration_runtime_recovers_zero_velocity_blend_band_through_public_wrapper() {
    let shared = Rc::new(RefCell::new(SimHarness {
        plant: PmsmModel::new_zeroed(actuator_breakaway_plant_params()).unwrap(),
        bus_voltage: Volts::new(24.0),
        load_torque: NewtonMeters::ZERO,
        last_duty: centered_phase_duty(),
        rotor_bias: 0.0,
    }));

    let (pwm, current, bus, rotor, output, temp) = runtime_handles(&shared);
    let system = ActuatorCalibrationRuntime::new(
        pwm,
        current,
        bus,
        rotor,
        output,
        temp,
        controller_motor_params(),
        inverter_params(),
        current_loop_config(),
        Svpwm,
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
        FAST_DT_SECONDS,
    )
    .unwrap();

    let (handle, ticker) = system.split().expect("calibration should split once");
    let result = loop {
        ticker.tick().unwrap();
        if let Some(result) = handle.status().result {
            break result;
        }
    };
    assert!((result.zero_velocity_blend_band.get() - 0.05).abs() < 0.01);
}

#[test]
fn motor_calibration_runtime_runs_request_driven_campaign() {
    let params = plant_params();
    let shared = Rc::new(RefCell::new(SimHarness {
        plant: PmsmModel::new(
            params,
            PmsmState {
                mechanical_angle: ContinuousMechanicalAngle::new(0.4),
                mechanical_velocity: RadPerSec::ZERO,
                current_dq: fluxkit::Dq::new(Amps::ZERO, Amps::ZERO),
                winding_temperature_c: WINDING_TEMP_C,
            },
        )
        .unwrap(),
        bus_voltage: Volts::new(24.0),
        load_torque: NewtonMeters::ZERO,
        last_duty: centered_phase_duty(),
        rotor_bias: 0.21,
    }));
    let (pwm, current, bus, rotor, temp) = calibration_hardware(&shared);
    let system = MotorCalibrationRuntime::new(
        pwm,
        current,
        bus,
        rotor,
        temp,
        Svpwm,
        PassThroughEstimator::new(),
        MotorCalibrationRequest::all(),
        MotorCalibrationLimits {
            max_align_voltage_mag: Volts::new(2.0),
            max_spin_voltage_mag: Volts::new(3.0),
            max_electrical_velocity: RadPerSec::new(60.0),
            timeout_seconds: 6.0,
        },
        FAST_DT_SECONDS,
    )
    .unwrap();

    let (handle, ticker) = system.split().expect("calibration should split once");
    let result = loop {
        ticker.tick().unwrap();
        if let Some(result) = handle.status().result {
            break result;
        }
    };

    assert_eq!(result.pole_pairs, params.pole_pairs);
    assert!(
        (result.phase_resistance_ohm_ref.get() - params.phase_resistance_ohm_ref.get()).abs()
            < 0.01
    );
    assert!((result.phase_inductance_h.get() - params.d_inductance_h.get()).abs() < 3.0e-6);
    assert!((result.flux_linkage_weber.get() - params.flux_linkage_weber.get()).abs() < 1.0e-4);
    assert!(result.electrical_angle_offset.get().is_finite());
}

#[test]
fn motor_calibration_result_can_be_sent_from_irq_thread_to_main_context() {
    let params = plant_params();
    let shared = Arc::new(Mutex::new(SimHarness {
        plant: PmsmModel::new(
            params,
            PmsmState {
                mechanical_angle: ContinuousMechanicalAngle::new(0.4),
                mechanical_velocity: RadPerSec::ZERO,
                current_dq: fluxkit::Dq::new(Amps::ZERO, Amps::ZERO),
                winding_temperature_c: WINDING_TEMP_C,
            },
        )
        .unwrap(),
        bus_voltage: Volts::new(24.0),
        load_torque: NewtonMeters::ZERO,
        last_duty: centered_phase_duty(),
        rotor_bias: 0.12,
    }));

    let (result_tx, result_rx) = mpsc::sync_channel(1);
    let shared_for_irq = Arc::clone(&shared);

    thread::scope(|scope| {
        scope.spawn(move || {
            let (pwm, current, bus, rotor, temp) = threaded_calibration_hardware(&shared_for_irq);
            let system = MotorCalibrationRuntime::new(
                pwm,
                current,
                bus,
                rotor,
                temp,
                Svpwm,
                PassThroughEstimator::new(),
                MotorCalibrationRequest::all(),
                MotorCalibrationLimits {
                    max_align_voltage_mag: Volts::new(2.0),
                    max_spin_voltage_mag: Volts::new(3.0),
                    max_electrical_velocity: RadPerSec::new(60.0),
                    timeout_seconds: 6.0,
                },
                FAST_DT_SECONDS,
            )
            .unwrap();

            let (handle, ticker) = system.split().expect("calibration should split once");
            let result = loop {
                ticker.tick().unwrap();
                if let Some(result) = handle.status().result {
                    break result;
                }
            };

            result_tx.send(result).unwrap();
        });

        let result = result_rx.recv().unwrap();
        let motor_params = result.into_motor_params(MotorLimits {
            max_phase_current: Amps::new(10.0),
            max_mech_speed: Some(RadPerSec::new(150.0)),
            max_winding_temperature_c: None,
        });

        assert_eq!(motor_params.pole_pairs, params.pole_pairs);
        assert!(
            (motor_params.phase_resistance_ohm_ref.get() - params.phase_resistance_ohm_ref.get())
                .abs()
                < 0.01
        );
        assert!((motor_params.d_inductance_h.get() - params.d_inductance_h.get()).abs() < 3.0e-6);
        assert!(
            (motor_params.flux_linkage_weber.get() - params.flux_linkage_weber.get()).abs()
                < 1.0e-4
        );
        assert!(motor_params.electrical_angle_offset.get().is_finite());
    });
}

#[test]
fn actuator_calibration_runtime_applies_provided_and_measured_values_through_live_controller() {
    let shared = Rc::new(RefCell::new(SimHarness {
        plant: PmsmModel::new_zeroed(actuator_breakaway_plant_params()).unwrap(),
        bus_voltage: Volts::new(24.0),
        load_torque: NewtonMeters::ZERO,
        last_duty: centered_phase_duty(),
        rotor_bias: 0.0,
    }));

    let (pwm, current, bus, rotor, output, temp) = runtime_handles(&shared);
    let system = ActuatorCalibrationRuntime::new(
        pwm,
        current,
        bus,
        rotor,
        output,
        temp,
        controller_motor_params(),
        inverter_params(),
        current_loop_config(),
        Svpwm,
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
        FAST_DT_SECONDS,
    )
    .unwrap();

    let (handle, ticker) = system.split().expect("calibration should split once");
    let result = loop {
        ticker.tick().unwrap();
        if let Some(result) = handle.status().result {
            break result;
        }
    };

    assert!((result.gear_ratio - GEAR_RATIO).abs() < 1.0e-6);
    assert!((result.positive_coulomb_torque.get() - 0.04).abs() < 1.0e-6);
    assert!((result.negative_coulomb_torque.get() - 0.05).abs() < 1.0e-6);
    assert!(result.positive_breakaway_torque.get().is_finite());
    assert!(result.zero_velocity_blend_band.get().is_finite());
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

#[test]
fn full_request_driven_bringup_recovers_calibration_and_reaches_runtime_velocity_target() {
    let params = PmsmParams {
        pole_pairs: 7,
        phase_resistance_ohm_ref: Ohms::new(0.12),
        d_inductance_h: Henries::new(0.000_03),
        q_inductance_h: Henries::new(0.000_03),
        flux_linkage_weber: Webers::new(0.005),
        electrical_direction: ElectricalDirection::Positive,
        thermal: ThermalPlantParams::default_for_ambient(WINDING_TEMP_C),
        actuator: ActuatorPlantParams {
            gear_ratio: GEAR_RATIO,
            output_inertia_kg_m2: 0.0208,
            positive_breakaway_torque: NewtonMeters::new(0.08),
            negative_breakaway_torque: NewtonMeters::new(0.09),
            positive_coulomb_torque: NewtonMeters::new(0.04),
            negative_coulomb_torque: NewtonMeters::new(0.05),
            positive_viscous_coefficient: 0.02,
            negative_viscous_coefficient: 0.03,
            zero_velocity_blend_band: RadPerSec::new(0.05),
        },
        max_voltage_mag: None,
    };
    let shared = Rc::new(RefCell::new(SimHarness {
        plant: PmsmModel::new(
            params,
            PmsmState {
                mechanical_angle: ContinuousMechanicalAngle::new(0.4),
                mechanical_velocity: RadPerSec::ZERO,
                current_dq: fluxkit::Dq::new(Amps::ZERO, Amps::ZERO),
                winding_temperature_c: WINDING_TEMP_C,
            },
        )
        .unwrap(),
        bus_voltage: Volts::new(24.0),
        load_torque: NewtonMeters::ZERO,
        last_duty: centered_phase_duty(),
        rotor_bias: 0.18,
    }));

    let (pwm, current, bus, rotor, temp) = calibration_hardware(&shared);
    let motor_calibration = MotorCalibrationRuntime::new(
        pwm,
        current,
        bus,
        rotor,
        temp,
        Svpwm,
        PassThroughEstimator::new(),
        MotorCalibrationRequest::all(),
        MotorCalibrationLimits {
            max_align_voltage_mag: Volts::new(2.0),
            max_spin_voltage_mag: Volts::new(3.0),
            max_electrical_velocity: RadPerSec::new(60.0),
            timeout_seconds: 6.0,
        },
        FAST_DT_SECONDS,
    )
    .unwrap();

    let (motor_handle, motor_ticker) = motor_calibration
        .split()
        .expect("calibration should split once");
    let motor_result = loop {
        motor_ticker.tick().unwrap();
        if let Some(result) = motor_handle.status().result {
            break result;
        }
    };

    assert_eq!(motor_result.pole_pairs, params.pole_pairs);
    assert!(
        (motor_result.phase_resistance_ohm_ref.get() - params.phase_resistance_ohm_ref.get()).abs()
            < 0.01
    );
    assert!((motor_result.phase_inductance_h.get() - params.d_inductance_h.get()).abs() < 4.0e-6);
    assert!(
        (motor_result.flux_linkage_weber.get() - params.flux_linkage_weber.get()).abs() < 2.0e-4
    );
    assert!(motor_result.electrical_angle_offset.get().is_finite());

    let motor_params = motor_result.into_motor_params(MotorLimits {
        max_phase_current: Amps::new(10.0),
        max_mech_speed: Some(RadPerSec::new(150.0)),
        max_winding_temperature_c: None,
    });

    let (pwm, current, bus, rotor, output, temp) = runtime_handles(&shared);
    let actuator_calibration = ActuatorCalibrationRuntime::new(
        pwm,
        current,
        bus,
        rotor,
        output,
        temp,
        motor_params,
        inverter_params(),
        current_loop_config(),
        Svpwm,
        PassThroughEstimator::new(),
        PassThroughEstimator::new(),
        ActuatorCalibrationRequest::all(),
        ActuatorCalibrationLimits {
            max_velocity_target: RadPerSec::new(10.0),
            max_torque_target: NewtonMeters::new(0.3),
            timeout_seconds: 20.0,
        },
        FAST_DT_SECONDS,
    )
    .unwrap();

    let (actuator_handle, actuator_ticker) = actuator_calibration
        .split()
        .expect("calibration should split once");
    let actuator_result = loop {
        actuator_ticker.tick().unwrap();
        if let Some(result) = actuator_handle.status().result {
            break result;
        }
    };

    assert!((actuator_result.gear_ratio - GEAR_RATIO).abs() < 0.05);
    assert!(actuator_result.positive_breakaway_torque.get().is_finite());
    assert!(actuator_result.negative_breakaway_torque.get().is_finite());
    assert!(actuator_result.positive_coulomb_torque.get().is_finite());
    assert!(actuator_result.negative_coulomb_torque.get().is_finite());
    assert!(actuator_result.positive_viscous_coefficient.is_finite());
    assert!(actuator_result.negative_viscous_coefficient.is_finite());
    assert!((actuator_result.zero_velocity_blend_band.get() - 0.05).abs() < 0.01);

    let actuator_params = actuator_result.into_friction_compensated_actuator_params(
        ActuatorLimits {
            max_output_velocity: Some(RadPerSec::new(30.0)),
            max_output_torque: Some(NewtonMeters::new(10.0)),
        },
        NewtonMeters::new(0.3),
    );

    let (pwm, current, bus, rotor, output, temp) = runtime_handles(&shared);
    let runtime = MotorRuntime::new(
        pwm,
        current,
        bus,
        rotor,
        output,
        temp,
        fluxkit::MotorRuntimeParams::new(
            motor_params,
            inverter_params(),
            actuator_params,
            current_loop_config(),
            FAST_DT_SECONDS,
        ),
        Svpwm,
        PassThroughEstimator::new(),
        PassThroughEstimator::new(),
    );

    let (handle, runtime_ticker) = runtime.split().expect("runtime should split once");
    handle.set_command(MotorCommand::Velocity(RadPerSec::new(2.0)));
    handle.arm();

    for _ in 0..100_000 {
        runtime_ticker.tick().unwrap();
    }

    let status = handle.status().controller;
    assert_eq!(handle.status().fault_latched, false);
    assert_eq!(status.active_error, None);
    assert!((status.last_output_mechanical_velocity.get() - 2.0).abs() < 0.25);
}
