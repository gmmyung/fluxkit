//! Request-driven calibration systems kept intentionally separate from the
//! IRQ-driven runtime control surface.
//!
//! `fluxkit_core` owns the pure calibration state machines. This module owns
//! the generic glue needed to:
//!
//! - sample HAL sensors
//! - feed those samples into the pure calibrators
//! - apply the returned drive requests through PWM or `MotorSystem`
//! - neutral or disable the drive path on completion and failure
//!
//! Calibration remains synchronous on purpose. Runtime control in `fluxkit`
//! is IRQ-driven through [`crate::MotorSystem`], while these systems own the
//! simpler step-by-step bring-up flow.
//!
//! Recommended bring-up order:
//!
//! 1. `MotorCalibrationSystem`
//!    - pole pairs + electrical offset
//!    - phase resistance
//!    - phase inductance
//!    - flux linkage
//! 2. `ActuatorCalibrationSystem`
//!    - gear ratio
//!    - Coulomb + viscous friction
//!    - breakaway torque
//!    - zero-velocity blend band
//!
//! The primary wrapper surface is request-driven:
//!
//! - construct `MotorCalibrationSystem` or `ActuatorCalibrationSystem`
//! - provide a calibration request plus operating limits
//! - call `tick(...)` until it returns `Some(final_calibration_result)`
//!
//! Lower-level routine driving remains in `fluxkit_core`.
//!
//! # End-to-end example
//!
//! ```ignore
//! use fluxkit::{
//!     ActuatorCalibrationLimits, ActuatorCalibrationRequest, ActuatorCalibrationSystem,
//!     MotorCalibrationLimits, MotorCalibrationRequest, MotorCalibrationSystem,
//!     MotorController, MotorHardware, MotorLimits, MotorSystem, PassThroughEstimator,
//!     TickSchedule,
//!     math::{
//!         Svpwm,
//!         units::{Amps, NewtonMeters, RadPerSec, Volts},
//!     },
//! };
//!
//! # fn take_motor_calibration_handles() -> (
//! #     impl fluxkit::PhasePwm,
//! #     impl fluxkit::CurrentSampler,
//! #     impl fluxkit::BusVoltageSensor,
//! #     impl fluxkit::RotorSensor,
//! # ) { todo!() }
//! # fn take_runtime_hardware() -> MotorHardware<
//! #     impl fluxkit::PhasePwm,
//! #     impl fluxkit::CurrentSampler,
//! #     impl fluxkit::BusVoltageSensor,
//! #     impl fluxkit::RotorSensor,
//! #     impl fluxkit::OutputSensor,
//! # > { todo!() }
//! # fn inverter_params() -> fluxkit::InverterParams { todo!() }
//! # fn current_loop_config() -> fluxkit::CurrentLoopConfig { todo!() }
//! const DT: f32 = 1.0 / 20_000.0;
//!
//! let (pwm, current, bus, rotor) = take_motor_calibration_handles();
//! let mut motor_calibration_system = MotorCalibrationSystem::new(
//!     pwm,
//!     current,
//!     bus,
//!     rotor,
//!     Svpwm,
//!     MotorCalibrationRequest {
//!         pole_pairs: None,
//!         electrical_angle_offset: None,
//!         phase_resistance_ohm: None,
//!         phase_inductance_h: None,
//!         flux_linkage_weber: None,
//!     },
//!     MotorCalibrationLimits {
//!         max_align_voltage_mag: Volts::new(2.0),
//!         max_spin_voltage_mag: Volts::new(3.0),
//!         max_electrical_velocity: RadPerSec::new(60.0),
//!         timeout_seconds: 6.0,
//!     },
//! )?;
//!
//! let motor_calibration = loop {
//!     if let Some(result) = motor_calibration_system.tick(DT)? {
//!         break result;
//!     }
//! };
//!
//! let motor_params = motor_calibration.into_motor_params(MotorLimits {
//!     max_phase_current: Amps::new(10.0),
//!     max_mech_speed: Some(RadPerSec::new(150.0)),
//! });
//!
//! let mut actuator_calibration_system = ActuatorCalibrationSystem::new(
//!     take_runtime_hardware(),
//!     motor_params,
//!     inverter_params(),
//!     current_loop_config(),
//!     Svpwm,
//!     PassThroughEstimator::new(),
//!     PassThroughEstimator::new(),
//!     ActuatorCalibrationRequest {
//!         gear_ratio: None,
//!         positive_coulomb_torque: None,
//!         negative_coulomb_torque: None,
//!         positive_viscous_coefficient: None,
//!         negative_viscous_coefficient: None,
//!         positive_breakaway_torque: None,
//!         negative_breakaway_torque: None,
//!         zero_velocity_blend_band: None,
//!     },
//!     ActuatorCalibrationLimits {
//!         max_velocity_target: RadPerSec::new(10.0),
//!         max_torque_target: NewtonMeters::new(0.3),
//!         timeout_seconds: 5.0,
//!     },
//! )?;
//!
//! let actuator_calibration = loop {
//!     if let Some(result) =
//!         actuator_calibration_system.tick(DT, TickSchedule::with_medium(DT))?
//!     {
//!         break result;
//!     }
//! };
//!
//! let actuator_params = actuator_calibration.into_friction_compensated_actuator_params(
//!     fluxkit::ActuatorLimits {
//!         max_output_velocity: Some(RadPerSec::new(30.0)),
//!         max_output_torque: Some(NewtonMeters::new(10.0)),
//!     },
//!     NewtonMeters::new(0.3),
//! );
//!
//! let controller = MotorController::new(
//!     motor_params,
//!     inverter_params(),
//!     actuator_params,
//!     current_loop_config(),
//! );
//!
//! let mut runtime = MotorSystem::new(
//!     take_runtime_hardware(),
//!     controller,
//!     PassThroughEstimator::new(),
//!     PassThroughEstimator::new(),
//!     fluxkit::MotorRuntimeConfig {
//!         fast_dt_seconds: DT,
//!         medium_divider: 20,
//!         slow_divider: 0,
//!     },
//! );
//! let handle = runtime.handle();
//! handle.set_command(fluxkit::MotorCommand {
//!     mode: fluxkit::ControlMode::Velocity,
//!     output_velocity_target: RadPerSec::new(2.0),
//!     ..fluxkit::MotorCommand::default()
//! });
//! handle.arm();
//!
//! loop {
//!     let _output = runtime.on_pwm_interrupt()?;
//!     runtime.run_deferred()?;
//! }
//! # Ok::<(), Box<dyn std::error::Error>>(())
//! ```

mod actuator;
mod motor;
mod shared;

pub use actuator::{
    ActuatorCalibrationLimits, ActuatorCalibrationRequest, ActuatorCalibrationResult,
    ActuatorCalibrationSystem, ActuatorCalibrationSystemError,
};
pub use motor::{
    MotorCalibrationLimits, MotorCalibrationRequest, MotorCalibrationResult,
    MotorCalibrationSystem, MotorCalibrationSystemError,
};
