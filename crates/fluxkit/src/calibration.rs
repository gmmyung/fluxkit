//! Project-facing calibration runtimes for motor and actuator bring-up.
//!
//! `fluxkit_core` owns the pure calibration procedures. This module wraps them
//! in HAL-facing runtimes that fit the same fixed-period execution model as the
//! runtime.
//!
//! Use these runtimes when your project needs to:
//!
//! 1. identify motor electrical parameters
//! 2. identify actuator/output-side parameters
//! 3. convert the results into runtime params
//! 4. hand off into [`crate::MotorRuntime`]
//!
//! The calibration runtimes are intentionally separate from the runtime surface:
//!
//! - calibration returns final resolved records
//! - runtime consumes fully built params
//! - projects can decide when to calibrate, persist, skip, or re-run bring-up
//!
//! Recommended bring-up order:
//!
//! 1. `MotorCalibrationRuntime`
//!    - pole pairs + electrical offset
//!    - phase resistance normalized to `25°C`
//!    - phase inductance
//!    - flux linkage
//! 2. `ActuatorCalibrationRuntime`
//!    - gear ratio
//!    - Coulomb + viscous friction
//!    - breakaway torque
//!    - zero-velocity blend band
//!
//! The primary wrapper surface is request-driven:
//!
//! - construct `MotorCalibrationRuntime` or `ActuatorCalibrationRuntime`
//! - provide a calibration request plus operating limits
//! - create the unique handle/ticker pair with `split()`
//! - call ticker `tick()` until `handle().status().result` becomes `Some(...)`
//! - use `try_into_parts()` when handing owned hardware/runtime state into the
//!   next bring-up phase
//!
//! Lower-level routine driving remains in `fluxkit_core`.
//!
//! # End-to-end example
//!
//! ```ignore
//! use fluxkit::{
//!     ActuatorCalibrationLimits, ActuatorCalibrationRequest, ActuatorCalibrationRuntime,
//!     MotorCalibrationLimits, MotorCalibrationRequest, MotorCalibrationRuntime, MotorLimits,
//!     MotorRuntime, MotorRuntimeParams, PassThroughEstimator, Svpwm,
//!     units::{Amps, NewtonMeters, RadPerSec, Volts},
//! };
//!
//! # fn take_motor_calibration_handles() -> (
//! #     impl fluxkit::PhasePwm,
//! #     impl fluxkit::CurrentSampler,
//! #     impl fluxkit::BusVoltageSensor,
//! #     impl fluxkit::RotorSensor,
//! #     impl fluxkit::TemperatureSensor,
//! # ) { todo!() }
//! # fn take_runtime_handles() -> (
//! #     impl fluxkit::PhasePwm,
//! #     impl fluxkit::CurrentSampler,
//! #     impl fluxkit::BusVoltageSensor,
//! #     impl fluxkit::RotorSensor,
//! #     impl fluxkit::OutputSensor,
//! #     impl fluxkit::TemperatureSensor,
//! # ) { todo!() }
//! # fn inverter_params() -> fluxkit::InverterParams { todo!() }
//! # fn current_loop_config() -> fluxkit::CurrentLoopConfig { todo!() }
//! const DT: f32 = 1.0 / 20_000.0;
//!
//! let (pwm, current, bus, rotor, temp) = take_motor_calibration_handles();
//! let motor_calibration = MotorCalibrationRuntime::new(
//!     pwm,
//!     current,
//!     bus,
//!     rotor,
//!     temp,
//!     Svpwm,
//!     PassThroughEstimator::new(),
//!     MotorCalibrationRequest::all(),
//!     MotorCalibrationLimits {
//!         max_align_voltage_mag: Volts::new(2.0),
//!         max_spin_voltage_mag: Volts::new(3.0),
//!         max_electrical_velocity: RadPerSec::new(60.0),
//!         timeout_seconds: 6.0,
//!     },
//!     DT,
//! )?;
//!
//! let (motor_handle, motor_ticker) = motor_calibration.split()?;
//! let motor_result = loop {
//!     motor_ticker.tick()?;
//!     if let Some(result) = motor_handle.status().result {
//!         break result;
//!     }
//! };
//!
//! let motor_params = motor_result.into_motor_params(MotorLimits {
//!     max_phase_current: Amps::new(10.0),
//!     max_mech_speed: Some(RadPerSec::new(150.0)),
//!     max_winding_temperature_c: None,
//! });
//!
//! let (pwm, current, bus, rotor, output, temp) = take_runtime_handles();
//! let actuator_calibration = ActuatorCalibrationRuntime::new(
//!     pwm,
//!     current,
//!     bus,
//!     rotor,
//!     output,
//!     temp,
//!     motor_params,
//!     inverter_params(),
//!     current_loop_config(),
//!     Svpwm,
//!     PassThroughEstimator::new(),
//!     PassThroughEstimator::new(),
//!     ActuatorCalibrationRequest::all(),
//!     ActuatorCalibrationLimits {
//!         max_velocity_target: RadPerSec::new(10.0),
//!         max_torque_target: NewtonMeters::new(0.3),
//!         timeout_seconds: 5.0,
//!     },
//!     DT,
//! )?;
//!
//! let (actuator_handle, actuator_ticker) = actuator_calibration.split()?;
//! let actuator_result = loop {
//!     actuator_ticker.tick()?;
//!     if let Some(result) = actuator_handle.status().result {
//!         break result;
//!     }
//! };
//!
//! let actuator_params = actuator_result.into_friction_compensated_actuator_params(
//!     fluxkit::ActuatorLimits {
//!         max_output_velocity: Some(RadPerSec::new(30.0)),
//!         max_output_torque: Some(NewtonMeters::new(10.0)),
//!     },
//!     NewtonMeters::new(0.3),
//! );
//!
//! let (pwm, current, bus, rotor, output, temp) = take_runtime_handles();
//! let runtime = MotorRuntime::new(
//!     pwm,
//!     current,
//!     bus,
//!     rotor,
//!     output,
//!     temp,
//!     MotorRuntimeParams::new(
//!         motor_params,
//!         inverter_params(),
//!         actuator_params,
//!         current_loop_config(),
//!         DT,
//!     ),
//!     Svpwm,
//!     PassThroughEstimator::new(),
//!     PassThroughEstimator::new(),
//! );
//! let (handle, ticker) = runtime.split()?;
//! handle.set_command(fluxkit::MotorCommand::Velocity(RadPerSec::new(2.0)));
//! handle.arm();
//!
//! loop {
//!     ticker.tick()?;
//! }
//! # Ok::<(), Box<dyn std::error::Error>>(())
//! ```

mod actuator;
mod motor;
mod shared;

pub use actuator::{
    ActuatorCalibrationHandle, ActuatorCalibrationLimits, ActuatorCalibrationPhase,
    ActuatorCalibrationRequest, ActuatorCalibrationResult, ActuatorCalibrationRuntime,
    ActuatorCalibrationRuntimeError, ActuatorCalibrationStatus, ActuatorCalibrationTicker,
};
pub use motor::{
    FluxLinkageRoutineConfig, MotorCalibrationConfig, MotorCalibrationHandle,
    MotorCalibrationLimits, MotorCalibrationParts, MotorCalibrationPhase, MotorCalibrationRequest,
    MotorCalibrationResult, MotorCalibrationRuntime, MotorCalibrationRuntimeError,
    MotorCalibrationStatus, MotorCalibrationTicker, PhaseInductanceRoutineConfig,
    PhaseResistanceRoutineConfig, PolePairsAndOffsetRoutineConfig,
};
