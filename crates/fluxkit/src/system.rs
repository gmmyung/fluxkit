//! Project-facing runtime wrapper over the pure controller and HAL traits.
//!
//! This module is for the common embedded ownership model:
//!
//! - one context owns the active [`MotorRuntime`] and provides a [`MotorTicker`]
//! - another context holds a [`MotorHandle`] for commands and status
//! - phase transitions consume the active runtime through
//!   [`MotorRuntime::try_into_parts`]
//!
//! In other words, [`MotorRuntime`] is the loop object you put behind your
//! fixed-period control interrupt.
//!
//! Each call to [`MotorTicker::tick`] performs one full control step:
//!
//! - sample hardware
//! - sample winding temperature
//! - update motion estimators
//! - run the controller
//! - apply PWM duty
//! - publish status for non-IRQ code
//!
//! `fluxkit` deliberately presents a simpler project-oriented runtime surface
//! here over the pure controller engine.
//!
//! ```ignore
//! # let pwm = todo!();
//! # let current = todo!();
//! # let bus = todo!();
//! # let rotor = todo!();
//! # let output = todo!();
//! # let temp = todo!();
//! # let motor_params = todo!();
//! # let inverter_params = todo!();
//! # let actuator_params = todo!();
//! # let current_loop_config = todo!();
//! let runtime = fluxkit::MotorRuntime::new(
//!     fluxkit::MotorHardware {
//!         pwm,
//!         current,
//!         bus,
//!         rotor,
//!         output,
//!         temp,
//!     },
//!     fluxkit::MotorRuntimeParams::new(
//!         motor_params,
//!         inverter_params,
//!         actuator_params,
//!         current_loop_config,
//!         1.0 / 20_000.0,
//!     ),
//!     fluxkit::RuntimeAlgorithms {
//!         modulator: fluxkit::Svpwm,
//!         current_estimator: fluxkit::PassThroughCurrentEstimator::new(),
//!         rotor_estimator: fluxkit::PassThroughEstimator::new(),
//!         output_estimator: fluxkit::PassThroughEstimator::new(),
//!     },
//! )?;
//! let (handle, ticker) = runtime.split()?;
//! handle.set_command(fluxkit::MotorCommand::Velocity(
//!     fluxkit::RadPerSec::new(2.0),
//! ));
//! handle.arm();
//!
//! loop {
//!     ticker.tick()?;
//! }
//! # Ok::<(), Box<dyn std::error::Error>>(())
//! ```

use core::cell::{Cell, RefCell};
use core::fmt;

use critical_section::Mutex;
use fluxkit_core::motor::{ControllerCommand, MotorControllerParts};
use fluxkit_core::{
    ActuatorCalibration, ActuatorEstimate, ActuatorParams, ControlInput, ControlOutput,
    CurrentEstimator, CurrentLoopConfig, InverterParams, MotorController, MotorParams, MotorStatus,
    RotorEstimate,
};
use fluxkit_hal::{
    BusVoltageSensor, CurrentSampleValidity, CurrentSampler, OutputSensor, PhasePwm, RotorSensor,
    TemperatureSensor,
};
use fluxkit_math::{
    ContinuousMechanicalAngle, MechanicalMotionEstimate, MechanicalMotionSample,
    MechanicalMotionSeed, Modulator, WrappedEstimator,
    frame::Dq,
    units::{Amps, NewtonMeters, RadPerSec, Volts},
};

use crate::CapabilitySplitError;
use crate::capability::{split_once, take_active_inner};

/// Hardware handles owned by one motor-control runtime.
#[derive(Debug)]
pub struct MotorHardware<PWM, CURRENT, BUS, ROTOR, OUTPUT, TEMP> {
    /// PWM output handle.
    pub pwm: PWM,
    /// Phase-current sampler.
    pub current: CURRENT,
    /// DC bus-voltage sensor.
    pub bus: BUS,
    /// Rotor sensor.
    pub rotor: ROTOR,
    /// Output/load-side sensor.
    pub output: OUTPUT,
    /// Winding temperature sensor.
    pub temp: TEMP,
}

/// Runtime algorithms owned by one motor-control runtime.
#[derive(Debug)]
pub struct RuntimeAlgorithms<MOD, CurrentEst, RotorEst, OutputEst> {
    /// Modulation strategy.
    pub modulator: MOD,
    /// Current-estimator state used by the controller.
    pub current_estimator: CurrentEst,
    /// Rotor motion estimator state.
    pub rotor_estimator: RotorEst,
    /// Output motion estimator state.
    pub output_estimator: OutputEst,
}

/// HAL and integration failures that can occur outside the pure controller.
#[derive(Debug)]
pub enum MotorRuntimeError<PwmE, CurrentE, BusE, RotorE, OutputE, TempE> {
    /// The runtime was temporarily unavailable because another context currently
    /// holds the active inner state.
    Busy,
    /// The runtime owner no longer contains an active inner runtime.
    Inactive,
    /// PWM output operation failed.
    Pwm(PwmE),
    /// Phase-current acquisition failed.
    Current(CurrentE),
    /// DC bus-voltage acquisition failed.
    Bus(BusE),
    /// Rotor-sensor acquisition failed.
    Rotor(RotorE),
    /// Output-sensor acquisition failed.
    Output(OutputE),
    /// Temperature-sensor acquisition failed.
    Temp(TempE),
    /// The current sample was explicitly marked invalid for control use.
    InvalidCurrentSample,
}

impl<PwmE, CurrentE, BusE, RotorE, OutputE, TempE> fmt::Display
    for MotorRuntimeError<PwmE, CurrentE, BusE, RotorE, OutputE, TempE>
where
    PwmE: fmt::Display,
    CurrentE: fmt::Display,
    BusE: fmt::Display,
    RotorE: fmt::Display,
    OutputE: fmt::Display,
    TempE: fmt::Display,
{
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Busy => f.write_str("runtime busy"),
            Self::Inactive => f.write_str("runtime inactive"),
            Self::Pwm(error) => write!(f, "pwm error: {error}"),
            Self::Current(error) => write!(f, "current-sensor error: {error}"),
            Self::Bus(error) => write!(f, "bus-voltage error: {error}"),
            Self::Rotor(error) => write!(f, "rotor-sensor error: {error}"),
            Self::Output(error) => write!(f, "output-sensor error: {error}"),
            Self::Temp(error) => write!(f, "temperature-sensor error: {error}"),
            Self::InvalidCurrentSample => f.write_str("invalid current sample"),
        }
    }
}

/// Static runtime-construction failures detected before the control loop starts.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum MotorRuntimeBuildError {
    /// The fixed runtime period must be finite and greater than zero.
    InvalidDtSeconds,
}

impl fmt::Display for MotorRuntimeBuildError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidDtSeconds => f.write_str("invalid runtime dt_seconds"),
        }
    }
}

impl core::error::Error for MotorRuntimeBuildError {}

impl<PwmE, CurrentE, BusE, RotorE, OutputE, TempE> core::error::Error
    for MotorRuntimeError<PwmE, CurrentE, BusE, RotorE, OutputE, TempE>
where
    PwmE: core::error::Error + 'static,
    CurrentE: core::error::Error + 'static,
    BusE: core::error::Error + 'static,
    RotorE: core::error::Error + 'static,
    OutputE: core::error::Error + 'static,
    TempE: core::error::Error + 'static,
{
    fn source(&self) -> Option<&(dyn core::error::Error + 'static)> {
        match self {
            Self::Busy => None,
            Self::Inactive => None,
            Self::Pwm(error) => Some(error),
            Self::Current(error) => Some(error),
            Self::Bus(error) => Some(error),
            Self::Rotor(error) => Some(error),
            Self::Output(error) => Some(error),
            Self::Temp(error) => Some(error),
            Self::InvalidCurrentSample => None,
        }
    }
}

/// Runtime command snapshot consumed by the wrapper-owned controller.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum MotorCommand {
    /// Disabled command with all targets cleared.
    Disabled,
    /// Direct current-mode command with explicit `d/q` current targets.
    Current(Dq<Amps>),
    /// Output-torque command.
    Torque(NewtonMeters),
    /// Output-side MIT impedance command.
    Mit {
        /// Output-axis position target.
        position: ContinuousMechanicalAngle,
        /// Output-axis velocity target.
        velocity: RadPerSec,
        /// Position stiffness gain in `Nm / rad`.
        kp: f32,
        /// Velocity damping gain in `Nm / (rad/s)`.
        kd: f32,
        /// Output-axis feedforward torque.
        torque_ff: NewtonMeters,
    },
    /// Output-velocity command.
    Velocity(RadPerSec),
    /// Output-position command.
    Position(ContinuousMechanicalAngle),
    /// Open-loop `d/q` voltage command.
    OpenLoopVoltage(Dq<Volts>),
}

impl Default for MotorCommand {
    fn default() -> Self {
        Self::Disabled
    }
}

#[inline]
fn core_command_from_runtime(command: MotorCommand) -> ControllerCommand {
    match command {
        MotorCommand::Disabled => ControllerCommand::Disabled,
        MotorCommand::Current(idq_target) => ControllerCommand::Current(idq_target),
        MotorCommand::Torque(output_torque_target) => {
            ControllerCommand::Torque(output_torque_target)
        }
        MotorCommand::Mit {
            position,
            velocity,
            kp,
            kd,
            torque_ff,
        } => ControllerCommand::Mit {
            position,
            velocity,
            kp,
            kd,
            torque_ff,
        },
        MotorCommand::Velocity(output_velocity_target) => {
            ControllerCommand::Velocity(output_velocity_target)
        }
        MotorCommand::Position(output_position_target) => {
            ControllerCommand::Position(output_position_target)
        }
        MotorCommand::OpenLoopVoltage(open_loop_voltage_target) => {
            ControllerCommand::OpenLoopVoltage(open_loop_voltage_target)
        }
    }
}

/// Output of one owned [`MotorTicker::tick`] cycle.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct MotorRuntimeOutput {
    /// Final phase-duty command applied to PWM.
    pub phase_duty: fluxkit_math::PhaseDuty,
    /// Measured `d/q` current for this cycle.
    pub measured_idq: Dq<Amps>,
    /// Commanded `d/q` voltage for this cycle.
    pub commanded_vdq: Dq<Volts>,
    /// `true` if voltage limiting saturated the command.
    pub saturated: bool,
}

impl MotorRuntimeOutput {
    #[inline]
    fn from_fast_loop(output: ControlOutput) -> Self {
        Self {
            phase_duty: output.phase_duty,
            measured_idq: output.measured_idq,
            commanded_vdq: output.commanded_vdq,
            saturated: output.saturated,
        }
    }
}

/// Runtime-facing status snapshot shared with non-ISR code.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct MotorRuntimeStatus {
    /// `true` while the runtime owner still contains an active inner runtime.
    pub active: bool,
    /// Latest controller status snapshot.
    pub controller: MotorStatus,
    /// Latest output-axis mechanical velocity shared with non-IRQ code.
    pub output_velocity: RadPerSec,
    /// Latest runtime output, if the runtime has run at least one cycle.
    pub last_fast_output: Option<MotorRuntimeOutput>,
    /// `true` when runtime arming is requested.
    pub armed: bool,
    /// `true` when the wrapper latched a runtime fault.
    pub fault_latched: bool,
}

/// Fully specified runtime configuration and controller parameters.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct MotorRuntimeParams {
    /// Motor electrical model and operating limits.
    pub motor: MotorParams,
    /// Inverter limits and PWM configuration.
    pub inverter: InverterParams,
    /// Actuator/output model, limits, and compensation configuration.
    pub actuator: ActuatorParams,
    /// Current-loop and supervisory controller tuning.
    pub current_loop: CurrentLoopConfig,
    /// Fixed control-loop period in seconds.
    pub dt_seconds: f32,
}

/// Owned runtime parts that can be moved between project phases.
#[derive(Debug)]
pub struct MotorRuntimeParts<
    PWM,
    CURRENT,
    BUS,
    ROTOR,
    OUTPUT,
    TEMP,
    MOD,
    CurrentEst,
    RotorEst,
    OutputEst,
> {
    /// Runtime hardware handles.
    pub hardware: MotorHardware<PWM, CURRENT, BUS, ROTOR, OUTPUT, TEMP>,
    /// Runtime controller parameters.
    pub params: MotorRuntimeParams,
    /// Runtime algorithms and estimator state.
    pub algorithms: RuntimeAlgorithms<MOD, CurrentEst, RotorEst, OutputEst>,
}

impl MotorRuntimeParams {
    /// Bundles the fixed runtime parameters needed to construct a [`MotorRuntime`].
    #[inline]
    pub const fn new(
        motor: MotorParams,
        inverter: InverterParams,
        actuator: ActuatorParams,
        current_loop: CurrentLoopConfig,
        dt_seconds: f32,
    ) -> Self {
        Self {
            motor,
            inverter,
            actuator,
            current_loop,
            dt_seconds,
        }
    }
}

#[inline]
fn validate_dt_seconds(dt_seconds: f32) -> bool {
    dt_seconds.is_finite() && dt_seconds > 0.0
}

#[derive(Clone, Copy, Debug)]
struct SharedRuntimeState {
    command: MotorCommand,
    status: MotorRuntimeStatus,
    clear_fault_requested: bool,
}

#[derive(Clone, Copy, Debug)]
struct RuntimeRequest {
    command: MotorCommand,
    armed: bool,
    clear_fault_requested: bool,
}

/// Non-ISR command and status access for a motor runtime.
#[derive(Debug)]
pub struct MotorHandle<'a> {
    shared: &'a Mutex<RefCell<SharedRuntimeState>>,
}

impl<'a> MotorHandle<'a> {
    /// Returns the latest shared runtime command snapshot.
    pub fn command(&self) -> MotorCommand {
        critical_section::with(|cs| self.shared.borrow(cs).borrow().command)
    }

    /// Replaces the shared runtime command snapshot.
    pub fn set_command(&self, command: MotorCommand) {
        critical_section::with(|cs| {
            let mut shared = self.shared.borrow(cs).borrow_mut();
            if !shared.status.active {
                return;
            }
            shared.command = command;
        });
    }

    /// Returns the latest shared runtime status snapshot.
    pub fn status(&self) -> MotorRuntimeStatus {
        critical_section::with(|cs| self.shared.borrow(cs).borrow().status)
    }

    /// Returns `true` when the runtime has latched a fault.
    #[inline]
    pub fn is_faulted(&self) -> bool {
        self.status().fault_latched
    }

    /// Returns `true` while the owning runtime is still active.
    #[inline]
    pub fn is_active(&self) -> bool {
        self.status().active
    }

    /// Requests that the runtime arm the motor path.
    pub fn arm(&self) {
        critical_section::with(|cs| {
            let mut shared = self.shared.borrow(cs).borrow_mut();
            if shared.status.active {
                shared.status.armed = true;
            }
        });
    }

    /// Requests that the runtime disarm the motor path.
    pub fn disarm(&self) {
        critical_section::with(|cs| {
            let mut shared = self.shared.borrow(cs).borrow_mut();
            if shared.status.active {
                shared.status.armed = false;
            }
        });
    }

    /// Returns `true` when the runtime is currently requested to be armed.
    pub fn is_armed(&self) -> bool {
        self.status().armed
    }

    /// Requests that the runtime clear latched faults on the next owned cycle.
    pub fn clear_fault(&self) {
        critical_section::with(|cs| {
            let mut shared = self.shared.borrow(cs).borrow_mut();
            if shared.status.active {
                shared.clear_fault_requested = true;
                shared.status.fault_latched = false;
            }
        });
    }
}

#[derive(Debug)]
struct InnerMotorRuntime<
    PWM,
    CURRENT,
    BUS,
    ROTOR,
    OUTPUT,
    TEMP,
    MOD,
    CurrentEst,
    RotorEst,
    OutputEst,
> {
    hardware: MotorHardware<PWM, CURRENT, BUS, ROTOR, OUTPUT, TEMP>,
    controller: MotorController<MOD, CurrentEst>,
    rotor_estimator: RotorEst,
    output_estimator: OutputEst,
    dt_seconds: f32,
    pwm_armed: bool,
}

#[derive(Debug)]
struct RuntimeLoop<'a, PWM, CURRENT, BUS, ROTOR, OUTPUT, TEMP, MOD, CurrentEst, RotorEst, OutputEst>
{
    runtime: &'a mut InnerMotorRuntime<
        PWM,
        CURRENT,
        BUS,
        ROTOR,
        OUTPUT,
        TEMP,
        MOD,
        CurrentEst,
        RotorEst,
        OutputEst,
    >,
    shared: &'a Mutex<RefCell<SharedRuntimeState>>,
}

/// Main-context owner of one active motor runtime.
#[derive(Debug)]
pub struct MotorRuntime<
    PWM,
    CURRENT,
    BUS,
    ROTOR,
    OUTPUT,
    TEMP,
    MOD,
    CurrentEst,
    RotorEst,
    OutputEst,
> {
    inner: Mutex<
        RefCell<
            Option<
                InnerMotorRuntime<
                    PWM,
                    CURRENT,
                    BUS,
                    ROTOR,
                    OUTPUT,
                    TEMP,
                    MOD,
                    CurrentEst,
                    RotorEst,
                    OutputEst,
                >,
            >,
        >,
    >,
    shared: Mutex<RefCell<SharedRuntimeState>>,
    split_taken: Cell<bool>,
}

/// IRQ-side execution capability for one active motor runtime.
#[derive(Debug)]
pub struct MotorTicker<
    'a,
    PWM,
    CURRENT,
    BUS,
    ROTOR,
    OUTPUT,
    TEMP,
    MOD,
    CurrentEst,
    RotorEst,
    OutputEst,
> {
    inner: &'a Mutex<
        RefCell<
            Option<
                InnerMotorRuntime<
                    PWM,
                    CURRENT,
                    BUS,
                    ROTOR,
                    OUTPUT,
                    TEMP,
                    MOD,
                    CurrentEst,
                    RotorEst,
                    OutputEst,
                >,
            >,
        >,
    >,
    shared: &'a Mutex<RefCell<SharedRuntimeState>>,
}

/// Helper trait for estimators that consume wrapped mechanical motion samples
/// and produce continuous mechanical motion estimates.
pub trait MechanicalMotionEstimator:
    WrappedEstimator<
        Input = MechanicalMotionSample,
        Output = MechanicalMotionEstimate,
        Seed = MechanicalMotionSeed,
    >
{
}

impl<T> MechanicalMotionEstimator for T where
    T: WrappedEstimator<
            Input = MechanicalMotionSample,
            Output = MechanicalMotionEstimate,
            Seed = MechanicalMotionSeed,
        >
{
}

mod runtime;
mod runtime_loop;

#[cfg(test)]
mod tests;
