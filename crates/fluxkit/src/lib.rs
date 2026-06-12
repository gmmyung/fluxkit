#![no_std]
#![forbid(unsafe_code)]
#![deny(rust_2018_idioms)]
#![warn(missing_docs, missing_debug_implementations)]
#![doc = include_str!("../README.md")]

pub mod calibration;
mod capability;
pub mod system;

pub use calibration::{
    ActuatorCalibrationHandle, ActuatorCalibrationLimits, ActuatorCalibrationPhase,
    ActuatorCalibrationRequest, ActuatorCalibrationResult, ActuatorCalibrationRuntime,
    ActuatorCalibrationRuntimeError, ActuatorCalibrationStatus, ActuatorCalibrationTicker,
    FluxLinkageRoutineConfig, MotorCalibrationConfig, MotorCalibrationHandle,
    MotorCalibrationLimits, MotorCalibrationParts, MotorCalibrationPhase, MotorCalibrationRequest,
    MotorCalibrationResult, MotorCalibrationRuntime, MotorCalibrationRuntimeError,
    MotorCalibrationStatus, MotorCalibrationTicker, PhaseInductanceRoutineConfig,
    PhaseResistanceRoutineConfig, PolePairsAndOffsetRoutineConfig,
};
pub use capability::CapabilitySplitError;
pub use fluxkit_core::{
    ActuatorCompensationConfig, ActuatorCompensationTelemetry, ActuatorEstimate, ActuatorLimits,
    ActuatorModel, ActuatorParams, CalibrationError, ControlInput, ControlMode, ControlOutput,
    CurrentEstimator, CurrentLoopConfig, CurrentLoopConfigBuilder, Error, FrictionCompensation,
    InverterParams, LpfCurrentEstimator, LpfCurrentEstimatorConfig, MotorLimits, MotorModel,
    MotorParams, MotorState, MotorStatus, PassThroughCurrentEstimator, RotorEstimate,
};
pub use fluxkit_hal::{
    BusVoltageSensor, CurrentSampleValidity, CurrentSampler, OutputReading, OutputSensor,
    PhaseCurrentSample, PhasePwm, RotorReading, RotorSensor, TemperatureSensor,
};
pub use fluxkit_math::{
    Abc, Amps, ContinuousMechanicalAngle, Dq, Duty, ElectricalAngle, ElectricalDirection, Henries,
    Hertz, MechanicalAngle, MechanicalMotionEstimate, MechanicalMotionSample, MechanicalMotionSeed,
    Modulator, NewtonMeters, Ohms, PassThroughEstimator, PhaseDuty, PllEstimator,
    PllEstimatorConfig, RadPerSec, SinePwm, Svpwm, Volts, Webers, WrappedEstimator, angle, units,
};
pub use system::MotorRuntimeError;
pub use system::{
    MechanicalMotionEstimator, MotorCommand, MotorHandle, MotorHardware, MotorRuntime,
    MotorRuntimeBundle, MotorRuntimeOutput, MotorRuntimeParams, MotorRuntimeStatus, MotorTicker,
    RuntimeAlgorithms,
};
