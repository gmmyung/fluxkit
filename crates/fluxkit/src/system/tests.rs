use core::convert::Infallible;

use fluxkit_core::{
    ActuatorLimits, ActuatorModel, ActuatorParams, ControlMode, CurrentLoopConfig, InverterParams,
    MotorLimits, MotorModel, MotorParams, MotorState,
};
use fluxkit_hal::{
    BusVoltageSensor, CurrentSampleValidity, CurrentSampler, OutputReading, OutputSensor,
    PhaseCurrentSample, PhasePwm, RotorReading, RotorSensor, TemperatureSensor,
    centered_phase_duty,
};
use fluxkit_math::{
    ContinuousMechanicalAngle, Dq, MechanicalAngle, WrappedEstimator,
    estimation::{
        AngularEstimate, EstimatorSeed, MechanicalMotionEstimate, MechanicalMotionSample,
        MechanicalMotionSeed,
    },
    frame::Abc,
    units::{Amps, Duty, Henries, Hertz, NewtonMeters, Ohms, RadPerSec, Volts},
};

use super::{Hardware, MotorRuntime, MotorRuntimeBuildError, MotorRuntimeError};

#[derive(Debug)]
struct FakePwm {
    enabled: bool,
    duty: Abc<Duty>,
}

impl Default for FakePwm {
    fn default() -> Self {
        Self {
            enabled: false,
            duty: centered_phase_duty(),
        }
    }
}

impl PhasePwm for FakePwm {
    type Error = Infallible;

    fn enable(&mut self) -> Result<(), Self::Error> {
        self.enabled = true;
        Ok(())
    }

    fn disable(&mut self) -> Result<(), Self::Error> {
        self.enabled = false;
        Ok(())
    }

    fn set_duty(&mut self, a: Duty, b: Duty, c: Duty) -> Result<(), Self::Error> {
        self.duty = Abc::new(a, b, c);
        Ok(())
    }
}

#[derive(Debug)]
struct FakeCurrentSensor {
    sample: PhaseCurrentSample,
}

impl CurrentSampler for FakeCurrentSensor {
    type Error = Infallible;

    fn sample_phase_currents(&mut self) -> Result<PhaseCurrentSample, Self::Error> {
        Ok(self.sample)
    }
}

#[derive(Debug)]
struct FakeBusSensor {
    voltage: Volts,
}

impl BusVoltageSensor for FakeBusSensor {
    type Error = Infallible;

    fn sample_bus_voltage(&mut self) -> Result<Volts, Self::Error> {
        Ok(self.voltage)
    }
}

#[derive(Debug)]
struct FakeTempSensor {
    winding_temperature_c: f32,
}

impl TemperatureSensor for FakeTempSensor {
    type Error = Infallible;

    fn sample_temperature_c(&mut self) -> Result<f32, Self::Error> {
        Ok(self.winding_temperature_c)
    }
}

#[derive(Debug)]
struct FakeRotor {
    reading: RotorReading,
}

impl RotorSensor for FakeRotor {
    type Error = Infallible;

    fn read_rotor(&mut self) -> Result<RotorReading, Self::Error> {
        Ok(self.reading)
    }
}

#[derive(Debug)]
struct FakeOutput {
    reading: OutputReading,
}

impl OutputSensor for FakeOutput {
    type Error = Infallible;

    fn read_output(&mut self) -> Result<OutputReading, Self::Error> {
        Ok(self.reading)
    }
}

#[derive(Clone, Copy, Debug)]
struct FixedEstimator {
    output: MechanicalMotionEstimate,
}

impl WrappedEstimator for FixedEstimator {
    type Input = MechanicalMotionSample;
    type Output = MechanicalMotionEstimate;
    type Seed = MechanicalMotionSeed;

    fn initialize(&mut self, seed: Self::Seed) {
        match seed {
            EstimatorSeed::Uninitialized => {
                self.output = AngularEstimate::new(
                    MechanicalAngle::new(0.0),
                    ContinuousMechanicalAngle::new(0.0),
                    RadPerSec::ZERO,
                );
            }
            EstimatorSeed::Value(wrapped_value) => {
                self.output =
                    AngularEstimate::new(wrapped_value.wrapped(), wrapped_value, RadPerSec::ZERO);
            }
            EstimatorSeed::ValueRate {
                value: wrapped_value,
                rate: measured_rate,
            } => {
                self.output =
                    AngularEstimate::new(wrapped_value.wrapped(), wrapped_value, measured_rate);
            }
            EstimatorSeed::Estimate(estimate) => {
                self.output = estimate;
            }
        }
    }

    fn output(&self) -> Self::Output {
        self.output
    }

    fn update(&mut self, _sample: Self::Input, _dt: f32) -> Self::Output {
        self.output
    }
}

fn motor_params() -> MotorParams {
    MotorParams::from_model_and_limits(
        MotorModel {
            pole_pairs: 7,
            phase_resistance_ohm_ref: Ohms::new(0.08),
            d_inductance_h: Henries::new(0.00012),
            q_inductance_h: Henries::new(0.00012),
            flux_linkage_weber: fluxkit_math::units::Webers::new(0.05),
            electrical_direction: fluxkit_math::ElectricalDirection::Positive,
            electrical_angle_offset: fluxkit_math::ElectricalAngle::new(0.0),
        },
        MotorLimits {
            max_phase_current: Amps::new(20.0),
            max_mech_speed: None,
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

fn actuator_params() -> ActuatorParams {
    ActuatorParams::from_model_limits_and_compensation(
        ActuatorModel { gear_ratio: 5.0 },
        ActuatorLimits {
            max_output_velocity: Some(RadPerSec::new(10.0)),
            max_output_torque: Some(NewtonMeters::new(10.0)),
        },
        fluxkit_core::ActuatorCompensationConfig::disabled(),
    )
}

fn current_loop_config() -> CurrentLoopConfig {
    CurrentLoopConfig {
        kp_d: 0.2,
        ki_d: 25.0,
        kp_q: 0.3,
        ki_q: 30.0,
        velocity_kp: 0.5,
        velocity_ki: 10.0,
        position_kp: 4.0,
        position_ki: 0.0,
        max_voltage_mag: Volts::new(12.0),
        id_ref_default: Amps::ZERO,
        max_id_target: Amps::new(5.0),
        max_iq_target: Amps::new(10.0),
        max_velocity_target: RadPerSec::new(50.0),
        max_current_ref_derivative_amps_per_sec: 10_000.0,
        enable_current_feedforward: true,
    }
}

fn hardware(
    validity: CurrentSampleValidity,
) -> Hardware<FakePwm, FakeCurrentSensor, FakeBusSensor, FakeRotor, FakeOutput, FakeTempSensor> {
    Hardware {
        pwm: FakePwm::default(),
        current: FakeCurrentSensor {
            sample: PhaseCurrentSample {
                currents: Abc::new(Amps::new(0.0), Amps::new(0.0), Amps::new(0.0)),
                validity,
            },
        },
        bus: FakeBusSensor {
            voltage: Volts::new(24.0),
        },
        rotor: FakeRotor {
            reading: RotorReading {
                mechanical_angle: fluxkit_math::MechanicalAngle::new(0.0),
                mechanical_velocity: RadPerSec::new(0.0),
            },
        },
        output: FakeOutput {
            reading: OutputReading {
                mechanical_angle: fluxkit_math::MechanicalAngle::new(0.0),
                mechanical_velocity: RadPerSec::new(0.0),
            },
        },
        temp: FakeTempSensor {
            winding_temperature_c: 25.0,
        },
    }
}

#[test]
fn fast_tick_reads_hal_and_applies_phase_duty() {
    let hardware = hardware(CurrentSampleValidity::Valid);
    let system = MotorRuntime::new(
        hardware.pwm,
        hardware.current,
        hardware.bus,
        hardware.rotor,
        hardware.output,
        hardware.temp,
        super::MotorRuntimeParams::new(
            motor_params(),
            inverter_params(),
            actuator_params(),
            current_loop_config(),
            0.000_05,
        ),
        fluxkit_math::Svpwm,
        crate::PassThroughCurrentEstimator::new(),
        fluxkit_math::PassThroughEstimator::new(),
        fluxkit_math::PassThroughEstimator::new(),
    )
    .expect("valid runtime config");
    let (handle, ticker) = system.split().expect("runtime should split once");
    handle.set_command(super::MotorCommand::Current(Dq::new(
        Amps::ZERO,
        Amps::new(2.0),
    )));
    handle.arm();
    ticker.tick().unwrap();

    let parts = system
        .try_into_parts()
        .expect("runtime parts should be available");
    assert_eq!(handle.status().controller.active_error, None);
    assert!(parts.pwm.enabled);
    assert_eq!(handle.status().controller.state, MotorState::Running);
    assert!(parts.pwm.duty.a.get() >= 0.0);
    assert!(parts.pwm.duty.a.get() <= 1.0);
    assert_ne!(parts.pwm.duty, centered_phase_duty());
}

#[test]
fn invalid_current_sample_returns_error_and_forces_neutral_pwm() {
    let mut hardware = hardware(CurrentSampleValidity::Invalid);
    hardware.pwm.duty = Abc::new(Duty::new(0.2), Duty::new(0.7), Duty::new(0.6));
    let system = MotorRuntime::new(
        hardware.pwm,
        hardware.current,
        hardware.bus,
        hardware.rotor,
        hardware.output,
        hardware.temp,
        super::MotorRuntimeParams::new(
            motor_params(),
            inverter_params(),
            actuator_params(),
            current_loop_config(),
            0.000_05,
        ),
        fluxkit_math::Svpwm,
        crate::PassThroughCurrentEstimator::new(),
        fluxkit_math::PassThroughEstimator::new(),
        fluxkit_math::PassThroughEstimator::new(),
    )
    .expect("valid runtime config");
    let (handle, ticker) = system.split().expect("runtime should split once");
    handle.arm();
    let error = ticker.tick().unwrap_err();
    let parts = system
        .try_into_parts()
        .expect("runtime parts should be available");

    assert!(matches!(error, MotorRuntimeError::InvalidCurrentSample));
    assert_eq!(parts.pwm.duty, centered_phase_duty());
}

#[test]
fn supervisory_work_runs_inside_fast_cycle() {
    let hardware = hardware(CurrentSampleValidity::Valid);
    let system = MotorRuntime::new(
        hardware.pwm,
        hardware.current,
        hardware.bus,
        hardware.rotor,
        hardware.output,
        hardware.temp,
        super::MotorRuntimeParams::new(
            motor_params(),
            inverter_params(),
            actuator_params(),
            current_loop_config(),
            0.000_05,
        ),
        fluxkit_math::Svpwm,
        crate::PassThroughCurrentEstimator::new(),
        fluxkit_math::PassThroughEstimator::new(),
        fluxkit_math::PassThroughEstimator::new(),
    )
    .expect("valid runtime config");
    let (handle, ticker) = system.split().expect("runtime should split once");
    handle.set_command(super::MotorCommand::Position(
        ContinuousMechanicalAngle::new(1.0),
    ));
    handle.arm();
    ticker.tick().unwrap();
    let first = handle
        .status()
        .last_fast_output
        .expect("first runtime output should be published");
    ticker.tick().unwrap();
    let second = handle
        .status()
        .last_fast_output
        .expect("second runtime output should be published");

    assert_eq!(first.phase_duty, centered_phase_duty());
    assert_ne!(second.phase_duty, centered_phase_duty());
    assert!(
        handle
            .status()
            .controller
            .last_output_mechanical_angle
            .get()
            .abs()
            < 1.0e-6
    );
}

#[test]
fn explicit_estimators_drive_controller_side_motion_estimates() {
    let hardware = hardware(CurrentSampleValidity::Valid);
    let system = MotorRuntime::new(
        hardware.pwm,
        hardware.current,
        hardware.bus,
        hardware.rotor,
        hardware.output,
        hardware.temp,
        super::MotorRuntimeParams::new(
            motor_params(),
            inverter_params(),
            actuator_params(),
            current_loop_config(),
            0.000_05,
        ),
        fluxkit_math::Svpwm,
        crate::PassThroughCurrentEstimator::new(),
        FixedEstimator {
            output: AngularEstimate::new(
                MechanicalAngle::new(0.3),
                ContinuousMechanicalAngle::new(1.3),
                RadPerSec::new(4.0),
            ),
        },
        FixedEstimator {
            output: AngularEstimate::new(
                MechanicalAngle::new(0.6),
                ContinuousMechanicalAngle::new(2.6),
                RadPerSec::new(1.5),
            ),
        },
    )
    .expect("valid runtime config");
    let (handle, ticker) = system.split().expect("runtime should split once");
    handle.set_command(super::MotorCommand::Current(Dq::new(
        Amps::ZERO,
        Amps::ZERO,
    )));
    handle.arm();
    ticker.tick().unwrap();

    let status = handle.status().controller;
    assert_eq!(
        status.last_rotor_mechanical_angle,
        ContinuousMechanicalAngle::new(1.3)
    );
    assert_eq!(
        status.last_output_mechanical_angle,
        ContinuousMechanicalAngle::new(2.6)
    );
    assert_eq!(status.last_output_mechanical_velocity, RadPerSec::new(1.5));
    assert_eq!(handle.status().output_velocity, RadPerSec::new(1.5));
}

#[test]
fn runtime_handle_updates_command_and_receives_status() {
    let hardware = hardware(CurrentSampleValidity::Valid);
    let system = MotorRuntime::new(
        hardware.pwm,
        hardware.current,
        hardware.bus,
        hardware.rotor,
        hardware.output,
        hardware.temp,
        super::MotorRuntimeParams::new(
            motor_params(),
            inverter_params(),
            actuator_params(),
            current_loop_config(),
            0.000_05,
        ),
        fluxkit_math::Svpwm,
        crate::PassThroughCurrentEstimator::new(),
        fluxkit_math::PassThroughEstimator::new(),
        fluxkit_math::PassThroughEstimator::new(),
    )
    .expect("valid runtime config");
    let (handle, ticker) = system.split().expect("runtime should split once");
    handle.set_command(super::MotorCommand::Current(Dq::new(
        Amps::ZERO,
        Amps::new(2.0),
    )));
    handle.arm();
    ticker.tick().unwrap();
    let status = handle.status();

    assert_eq!(status.controller.mode, ControlMode::Current);
    assert!(status.last_fast_output.is_some());
    assert_eq!(status.output_velocity, RadPerSec::ZERO);
    assert_eq!(
        handle.command(),
        super::MotorCommand::Current(Dq::new(Amps::ZERO, Amps::new(2.0)))
    );
    assert!(!status.fault_latched);
}

#[test]
fn over_temperature_latches_runtime_fault_and_centers_output() {
    let mut motor = motor_params();
    motor.limits.max_winding_temperature_c = Some(80.0);
    let mut hardware = hardware(CurrentSampleValidity::Valid);
    hardware.temp.winding_temperature_c = 95.0;
    let system = MotorRuntime::new(
        hardware.pwm,
        hardware.current,
        hardware.bus,
        hardware.rotor,
        hardware.output,
        hardware.temp,
        super::MotorRuntimeParams::new(
            motor,
            inverter_params(),
            actuator_params(),
            current_loop_config(),
            0.000_05,
        ),
        fluxkit_math::Svpwm,
        crate::PassThroughCurrentEstimator::new(),
        fluxkit_math::PassThroughEstimator::new(),
        fluxkit_math::PassThroughEstimator::new(),
    )
    .expect("valid runtime config");
    let (handle, ticker) = system.split().expect("runtime should split once");
    handle.arm();
    ticker.tick().unwrap();
    let status = handle.status();

    assert_eq!(
        status
            .last_fast_output
            .expect("faulted output should be published")
            .phase_duty,
        centered_phase_duty()
    );
    assert!(status.fault_latched);
    assert_eq!(
        status.controller.active_error,
        Some(fluxkit_core::Error::OverTemperature)
    );
    assert_eq!(status.controller.state, MotorState::Faulted);
}

#[test]
fn extracted_runtime_marks_handles_and_tickers_inactive() {
    let hardware = hardware(CurrentSampleValidity::Valid);
    let system = MotorRuntime::new(
        hardware.pwm,
        hardware.current,
        hardware.bus,
        hardware.rotor,
        hardware.output,
        hardware.temp,
        super::MotorRuntimeParams::new(
            motor_params(),
            inverter_params(),
            actuator_params(),
            current_loop_config(),
            0.000_05,
        ),
        fluxkit_math::Svpwm,
        crate::PassThroughCurrentEstimator::new(),
        fluxkit_math::PassThroughEstimator::new(),
        fluxkit_math::PassThroughEstimator::new(),
    )
    .expect("valid runtime config");
    let (handle, ticker) = system.split().expect("runtime should split once");

    let _parts = system
        .try_into_parts()
        .expect("runtime parts should be available");

    assert!(!handle.status().active);
    handle.arm();
    assert!(!handle.status().armed);
    assert!(matches!(ticker.tick(), Err(MotorRuntimeError::Inactive)));
}

#[test]
fn runtime_builder_rejects_non_positive_dt_seconds() {
    let hardware = hardware(CurrentSampleValidity::Valid);
    let error = MotorRuntime::new(
        hardware.pwm,
        hardware.current,
        hardware.bus,
        hardware.rotor,
        hardware.output,
        hardware.temp,
        super::MotorRuntimeParams::new(
            motor_params(),
            inverter_params(),
            actuator_params(),
            current_loop_config(),
            0.0,
        ),
        fluxkit_math::Svpwm,
        crate::PassThroughCurrentEstimator::new(),
        fluxkit_math::PassThroughEstimator::new(),
        fluxkit_math::PassThroughEstimator::new(),
    )
    .expect_err("non-positive dt should be rejected");

    assert_eq!(error, MotorRuntimeBuildError::InvalidDtSeconds);
}
