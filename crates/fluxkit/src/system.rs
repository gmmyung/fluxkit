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
//!     pwm,
//!     current,
//!     bus,
//!     rotor,
//!     output,
//!     temp,
//!     fluxkit::MotorRuntimeParams::new(
//!         motor_params,
//!         inverter_params,
//!         actuator_params,
//!         current_loop_config,
//!         1.0 / 20_000.0,
//!     ),
//!     fluxkit::Svpwm,
//!     fluxkit::PassThroughEstimator::new(),
//!     fluxkit::PassThroughEstimator::new(),
//! );
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
    ActuatorCalibration, ActuatorEstimate, ActuatorParams, CurrentLoopConfig, FastLoopInput,
    FastLoopOutput, InverterParams, MotorController, MotorParams, MotorStatus, RotorEstimate,
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

/// Private hardware handles owned by one motor-control runtime.
#[derive(Debug)]
pub(crate) struct Hardware<PWM, CURRENT, BUS, ROTOR, OUTPUT, TEMP> {
    pwm: PWM,
    current: CURRENT,
    bus: BUS,
    rotor: ROTOR,
    output: OUTPUT,
    temp: TEMP,
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

impl From<FastLoopOutput> for MotorRuntimeOutput {
    fn from(output: FastLoopOutput) -> Self {
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
pub struct MotorRuntimeParts<PWM, CURRENT, BUS, ROTOR, OUTPUT, TEMP, MOD, RotorEst, OutputEst> {
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
    /// Motor electrical model and operating limits.
    pub motor: MotorParams,
    /// Inverter limits and PWM configuration.
    pub inverter: InverterParams,
    /// Actuator/output model, limits, and compensation configuration.
    pub actuator: ActuatorParams,
    /// Current-loop and supervisory controller tuning.
    pub current_loop: CurrentLoopConfig,
    /// Modulation strategy.
    pub modulator: MOD,
    /// Rotor motion estimator state.
    pub rotor_estimator: RotorEst,
    /// Output motion estimator state.
    pub output_estimator: OutputEst,
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

#[derive(Clone, Copy, Debug)]
struct SharedRuntimeState {
    command: MotorCommand,
    command_dirty: bool,
    status: MotorRuntimeStatus,
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
            shared.command_dirty = true;
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
struct InnerMotorRuntime<PWM, CURRENT, BUS, ROTOR, OUTPUT, TEMP, MOD, RotorEst, OutputEst> {
    hardware: Hardware<PWM, CURRENT, BUS, ROTOR, OUTPUT, TEMP>,
    controller: MotorController<MOD>,
    rotor_estimator: RotorEst,
    output_estimator: OutputEst,
    dt_seconds: f32,
    pwm_armed: bool,
}

#[derive(Debug)]
struct RuntimeLoop<'a, PWM, CURRENT, BUS, ROTOR, OUTPUT, TEMP, MOD, RotorEst, OutputEst> {
    runtime:
        &'a mut InnerMotorRuntime<PWM, CURRENT, BUS, ROTOR, OUTPUT, TEMP, MOD, RotorEst, OutputEst>,
    shared: &'a Mutex<RefCell<SharedRuntimeState>>,
}

/// Main-context owner of one active motor runtime.
#[derive(Debug)]
pub struct MotorRuntime<PWM, CURRENT, BUS, ROTOR, OUTPUT, TEMP, MOD, RotorEst, OutputEst> {
    inner: Mutex<
        RefCell<
            Option<
                InnerMotorRuntime<PWM, CURRENT, BUS, ROTOR, OUTPUT, TEMP, MOD, RotorEst, OutputEst>,
            >,
        >,
    >,
    shared: Mutex<RefCell<SharedRuntimeState>>,
    split_taken: Cell<bool>,
}

/// IRQ-side execution capability for one active motor runtime.
#[derive(Debug)]
pub struct MotorTicker<'a, PWM, CURRENT, BUS, ROTOR, OUTPUT, TEMP, MOD, RotorEst, OutputEst> {
    inner: &'a Mutex<
        RefCell<
            Option<
                InnerMotorRuntime<PWM, CURRENT, BUS, ROTOR, OUTPUT, TEMP, MOD, RotorEst, OutputEst>,
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

impl<PWM, CURRENT, BUS, ROTOR, OUTPUT, TEMP, MOD, RotorEst, OutputEst>
    MotorRuntime<PWM, CURRENT, BUS, ROTOR, OUTPUT, TEMP, MOD, RotorEst, OutputEst>
where
    MOD: Modulator,
    RotorEst: MechanicalMotionEstimator,
    OutputEst: MechanicalMotionEstimator,
{
    /// Creates a new runtime with an explicit loop period, controller params,
    /// modulator, and rotor/output estimators.
    pub fn new(
        pwm: PWM,
        current: CURRENT,
        bus: BUS,
        rotor: ROTOR,
        output: OUTPUT,
        temp: TEMP,
        params: MotorRuntimeParams,
        modulator: MOD,
        rotor_estimator: RotorEst,
        output_estimator: OutputEst,
    ) -> Self {
        Self::from_parts(
            MotorRuntimeParts {
                pwm,
                current,
                bus,
                rotor,
                output,
                temp,
                motor: params.motor,
                inverter: params.inverter,
                actuator: params.actuator,
                current_loop: params.current_loop,
                modulator,
                rotor_estimator,
                output_estimator,
            },
            params.dt_seconds,
        )
    }

    /// Creates a runtime from previously owned runtime parts and a loop period.
    pub fn from_parts(
        parts: MotorRuntimeParts<PWM, CURRENT, BUS, ROTOR, OUTPUT, TEMP, MOD, RotorEst, OutputEst>,
        dt_seconds: f32,
    ) -> Self {
        let controller = MotorController::new(
            parts.motor,
            parts.inverter,
            parts.actuator,
            parts.current_loop,
            parts.modulator,
        );
        Self::from_controller_parts(
            Hardware {
                pwm: parts.pwm,
                current: parts.current,
                bus: parts.bus,
                rotor: parts.rotor,
                output: parts.output,
                temp: parts.temp,
            },
            controller,
            parts.rotor_estimator,
            parts.output_estimator,
            dt_seconds,
        )
    }

    fn from_controller_parts(
        hardware: Hardware<PWM, CURRENT, BUS, ROTOR, OUTPUT, TEMP>,
        controller: MotorController<MOD>,
        rotor_estimator: RotorEst,
        output_estimator: OutputEst,
        dt_seconds: f32,
    ) -> Self {
        Self {
            shared: Mutex::new(RefCell::new(SharedRuntimeState {
                command: MotorCommand::default(),
                command_dirty: false,
                status: MotorRuntimeStatus {
                    active: true,
                    controller: controller.status(),
                    last_fast_output: None,
                    armed: false,
                    fault_latched: false,
                },
                clear_fault_requested: false,
            })),
            inner: Mutex::new(RefCell::new(Some(InnerMotorRuntime {
                hardware,
                controller,
                rotor_estimator,
                output_estimator,
                dt_seconds,
                pwm_armed: false,
            }))),
            split_taken: Cell::new(false),
        }
    }

    /// Splits this runtime into its unique main-context handle and IRQ-side ticker.
    ///
    /// This can be called at most once for a given runtime owner.
    #[inline]
    pub fn split(
        &self,
    ) -> Result<
        (
            MotorHandle<'_>,
            MotorTicker<'_, PWM, CURRENT, BUS, ROTOR, OUTPUT, TEMP, MOD, RotorEst, OutputEst>,
        ),
        CapabilitySplitError,
    > {
        if !critical_section::with(|cs| self.shared.borrow(cs).borrow().status.active) {
            return Err(CapabilitySplitError::Inactive);
        }
        if self.split_taken.replace(true) {
            return Err(CapabilitySplitError::AlreadySplit);
        }
        Ok((
            MotorHandle {
                shared: &self.shared,
            },
            self.ticker_internal(),
        ))
    }

    #[inline]
    pub(crate) fn ticker_internal(
        &self,
    ) -> MotorTicker<'_, PWM, CURRENT, BUS, ROTOR, OUTPUT, TEMP, MOD, RotorEst, OutputEst> {
        MotorTicker {
            inner: &self.inner,
            shared: &self.shared,
        }
    }

    /// Attempts to take ownership of the active runtime parts for reuse in another phase.
    pub fn try_into_parts(
        &self,
    ) -> Option<MotorRuntimeParts<PWM, CURRENT, BUS, ROTOR, OUTPUT, TEMP, MOD, RotorEst, OutputEst>>
    {
        let InnerMotorRuntime {
            hardware,
            controller,
            rotor_estimator,
            output_estimator,
            ..
        } = critical_section::with(|cs| self.inner.borrow(cs).borrow_mut().take())?;
        critical_section::with(|cs| {
            let mut shared = self.shared.borrow(cs).borrow_mut();
            shared.status.active = false;
            shared.status.armed = false;
            shared.clear_fault_requested = false;
        });
        let MotorControllerParts {
            motor,
            inverter,
            actuator,
            config,
            modulator,
        } = controller.into_parts();
        Some(MotorRuntimeParts {
            pwm: hardware.pwm,
            current: hardware.current,
            bus: hardware.bus,
            rotor: hardware.rotor,
            output: hardware.output,
            temp: hardware.temp,
            motor,
            inverter,
            actuator,
            current_loop: config,
            modulator,
            rotor_estimator,
            output_estimator,
        })
    }
}

impl<PWM, CURRENT, BUS, ROTOR, OUTPUT, TEMP, MOD, RotorEst, OutputEst>
    MotorRuntime<PWM, CURRENT, BUS, ROTOR, OUTPUT, TEMP, MOD, RotorEst, OutputEst>
where
    PWM: PhasePwm,
    CURRENT: CurrentSampler,
    BUS: BusVoltageSensor,
    ROTOR: RotorSensor,
    OUTPUT: OutputSensor,
    TEMP: TemperatureSensor,
    MOD: Modulator,
    RotorEst: MechanicalMotionEstimator,
    OutputEst: MechanicalMotionEstimator,
{
    /// Sets the armed state immediately on the owned runtime.
    #[inline]
    pub(crate) fn set_armed_immediate(
        &self,
        armed: bool,
    ) -> Result<
        (),
        MotorRuntimeError<
            PWM::Error,
            CURRENT::Error,
            BUS::Error,
            ROTOR::Error,
            OUTPUT::Error,
            TEMP::Error,
        >,
    > {
        critical_section::with(|cs| {
            let mut shared = self.shared.borrow(cs).borrow_mut();
            if shared.status.active {
                shared.status.armed = armed;
            }
        });
        let mut inner = critical_section::with(|cs| self.inner.borrow(cs).borrow_mut().take())
            .ok_or_else(|| {
                let active =
                    critical_section::with(|cs| self.shared.borrow(cs).borrow().status.active);
                if active {
                    MotorRuntimeError::Busy
                } else {
                    MotorRuntimeError::Inactive
                }
            })?;
        {
            let mut runtime = RuntimeLoop {
                runtime: &mut inner,
                shared: &self.shared,
            };
            runtime.sync_runtime_requests()?;
            runtime.publish_runtime_status(None);
        }
        critical_section::with(|cs| {
            *self.inner.borrow(cs).borrow_mut() = Some(inner);
        });
        Ok(())
    }

    #[inline]
    pub(crate) fn apply_command_immediate(&self, command: MotorCommand) {
        critical_section::with(|cs| {
            let mut shared = self.shared.borrow(cs).borrow_mut();
            if shared.status.active {
                shared.command = command;
                shared.command_dirty = true;
            }
        });
    }

    #[inline]
    pub(crate) fn controller_status(&self) -> MotorStatus {
        critical_section::with(|cs| self.shared.borrow(cs).borrow().status.controller)
    }

    #[inline]
    pub(crate) fn friction_compensation_enabled(&self) -> bool {
        critical_section::with(|cs| {
            self.inner
                .borrow(cs)
                .borrow()
                .as_ref()
                .is_some_and(|inner| inner.controller.friction_compensation_enabled())
        })
    }

    #[inline]
    pub(crate) fn apply_actuator_calibration(&self, calibration: &ActuatorCalibration) {
        critical_section::with(|cs| {
            if let Some(inner) = self.inner.borrow(cs).borrow_mut().as_mut() {
                inner.controller.apply_actuator_calibration(calibration);
            }
        });
    }
}

impl<'a, PWM, CURRENT, BUS, ROTOR, OUTPUT, TEMP, MOD, RotorEst, OutputEst>
    MotorTicker<'a, PWM, CURRENT, BUS, ROTOR, OUTPUT, TEMP, MOD, RotorEst, OutputEst>
where
    PWM: PhasePwm,
    CURRENT: CurrentSampler,
    BUS: BusVoltageSensor,
    ROTOR: RotorSensor,
    OUTPUT: OutputSensor,
    TEMP: TemperatureSensor,
    MOD: Modulator,
    RotorEst: MechanicalMotionEstimator,
    OutputEst: MechanicalMotionEstimator,
{
    /// Runs one full control cycle using the configured runtime period.
    pub fn tick(
        &self,
    ) -> Result<
        (),
        MotorRuntimeError<
            PWM::Error,
            CURRENT::Error,
            BUS::Error,
            ROTOR::Error,
            OUTPUT::Error,
            TEMP::Error,
        >,
    > {
        let mut inner = critical_section::with(|cs| self.inner.borrow(cs).borrow_mut().take())
            .ok_or_else(|| {
                let active =
                    critical_section::with(|cs| self.shared.borrow(cs).borrow().status.active);
                if active {
                    MotorRuntimeError::Busy
                } else {
                    MotorRuntimeError::Inactive
                }
            })?;
        let result = {
            let mut runtime = RuntimeLoop {
                runtime: &mut inner,
                shared: self.shared,
            };
            runtime.tick()
        };
        critical_section::with(|cs| {
            *self.inner.borrow(cs).borrow_mut() = Some(inner);
        });
        result
    }
}

impl<'a, PWM, CURRENT, BUS, ROTOR, OUTPUT, TEMP, MOD, RotorEst, OutputEst>
    RuntimeLoop<'a, PWM, CURRENT, BUS, ROTOR, OUTPUT, TEMP, MOD, RotorEst, OutputEst>
where
    PWM: PhasePwm,
    CURRENT: CurrentSampler,
    BUS: BusVoltageSensor,
    ROTOR: RotorSensor,
    OUTPUT: OutputSensor,
    TEMP: TemperatureSensor,
    MOD: Modulator,
    RotorEst: MechanicalMotionEstimator,
    OutputEst: MechanicalMotionEstimator,
{
    fn sync_runtime_requests(
        &mut self,
    ) -> Result<
        bool,
        MotorRuntimeError<
            PWM::Error,
            CURRENT::Error,
            BUS::Error,
            ROTOR::Error,
            OUTPUT::Error,
            TEMP::Error,
        >,
    > {
        let (command, command_dirty, clear_fault_requested, armed_requested) =
            critical_section::with(|cs| {
                let mut shared = self.shared.borrow(cs).borrow_mut();
                let command = shared.command;
                let command_dirty = shared.command_dirty;
                let clear_fault_requested = shared.clear_fault_requested;
                let armed_requested = shared.status.armed;
                shared.command_dirty = false;
                shared.clear_fault_requested = false;
                (
                    command,
                    command_dirty,
                    clear_fault_requested,
                    armed_requested,
                )
            });

        if clear_fault_requested {
            self.runtime.controller.clear_error();
            critical_section::with(|cs| {
                self.shared.borrow(cs).borrow_mut().status.fault_latched =
                    self.runtime.controller.status().active_error.is_some();
            });
        }

        if armed_requested != self.runtime.pwm_armed {
            if armed_requested {
                self.runtime
                    .hardware
                    .pwm
                    .enable()
                    .map_err(MotorRuntimeError::Pwm)?;
                self.runtime.controller.set_armed(true);
            } else {
                self.runtime.controller.set_armed(false);
                self.runtime
                    .hardware
                    .pwm
                    .set_neutral()
                    .map_err(MotorRuntimeError::Pwm)?;
                self.runtime
                    .hardware
                    .pwm
                    .disable()
                    .map_err(MotorRuntimeError::Pwm)?;
            }
            self.runtime.pwm_armed = armed_requested;
        }

        if command_dirty {
            self.runtime
                .controller
                .apply_command(core_command_from_runtime(command));
        }

        Ok(self.runtime.pwm_armed)
    }

    fn publish_runtime_status(&self, last_fast_output: Option<MotorRuntimeOutput>) {
        critical_section::with(|cs| {
            let mut shared = self.shared.borrow(cs).borrow_mut();
            if let Some(output) = last_fast_output {
                shared.status.last_fast_output = Some(output);
            }
            shared.status.controller = self.runtime.controller.status();
            shared.status.fault_latched |= self.runtime.controller.status().active_error.is_some();
        });
    }

    fn mark_runtime_fault(&self) {
        critical_section::with(|cs| {
            self.shared.borrow(cs).borrow_mut().status.fault_latched = true;
        });
    }

    fn fault_and_neutral(
        &mut self,
    ) -> Result<
        (),
        MotorRuntimeError<
            PWM::Error,
            CURRENT::Error,
            BUS::Error,
            ROTOR::Error,
            OUTPUT::Error,
            TEMP::Error,
        >,
    > {
        self.runtime
            .hardware
            .pwm
            .set_neutral()
            .map_err(MotorRuntimeError::Pwm)?;
        self.mark_runtime_fault();
        self.publish_runtime_status(None);
        Ok(())
    }

    /// Runs one full control cycle using the configured runtime period.
    ///
    /// This executes one full owned controller cycle using the configured `dt`.
    ///
    /// Internally that means:
    ///
    /// 1. sample hardware and run the electrical current loop
    /// 2. update the supervisory references for the next cycle
    ///
    /// so supervisory updates affect the next electrical step, not the current one.
    pub fn tick(
        &mut self,
    ) -> Result<
        (),
        MotorRuntimeError<
            PWM::Error,
            CURRENT::Error,
            BUS::Error,
            ROTOR::Error,
            OUTPUT::Error,
            TEMP::Error,
        >,
    > {
        match self.execute_fast_cycle(self.runtime.dt_seconds) {
            Ok(_) => Ok(()),
            Err(error) => {
                if !matches!(error, MotorRuntimeError::InvalidCurrentSample) {
                    let _ = self.fault_and_neutral();
                }
                Err(error)
            }
        }
    }

    /// Samples hardware, runs one owned controller cycle, and applies duty.
    pub(crate) fn run_cycle(
        &mut self,
        dt_seconds: f32,
    ) -> Result<
        MotorRuntimeOutput,
        MotorRuntimeError<
            PWM::Error,
            CURRENT::Error,
            BUS::Error,
            ROTOR::Error,
            OUTPUT::Error,
            TEMP::Error,
        >,
    > {
        let armed = self.sync_runtime_requests()?;
        if !armed {
            let output = MotorRuntimeOutput::from(FastLoopOutput {
                phase_duty: fluxkit_hal::centered_phase_duty(),
                measured_idq: fluxkit_math::frame::Dq::new(Amps::ZERO, Amps::ZERO),
                commanded_vdq: fluxkit_math::frame::Dq::new(Volts::ZERO, Volts::ZERO),
                saturated: false,
            });
            self.publish_runtime_status(Some(output));
            return Ok(output);
        }

        let current = self
            .runtime
            .hardware
            .current
            .sample_phase_currents()
            .map_err(MotorRuntimeError::Current)?;

        if current.validity == CurrentSampleValidity::Invalid {
            self.fault_and_neutral()?;
            return Err(MotorRuntimeError::InvalidCurrentSample);
        }

        let bus_voltage = self
            .runtime
            .hardware
            .bus
            .sample_bus_voltage()
            .map_err(MotorRuntimeError::Bus)?;
        let winding_temperature_c = self
            .runtime
            .hardware
            .temp
            .sample_temperature_c()
            .map_err(MotorRuntimeError::Temp)?;

        let rotor = self
            .runtime
            .hardware
            .rotor
            .read_rotor()
            .map_err(MotorRuntimeError::Rotor)?;
        let output_axis = self
            .runtime
            .hardware
            .output
            .read_output()
            .map_err(MotorRuntimeError::Output)?;
        let rotor_motion = self.runtime.rotor_estimator.update(
            MechanicalMotionSample {
                wrapped_value: rotor.mechanical_angle,
                measured_rate: rotor.mechanical_velocity,
            },
            dt_seconds,
        );
        let output_motion = self.runtime.output_estimator.update(
            MechanicalMotionSample {
                wrapped_value: output_axis.mechanical_angle,
                measured_rate: output_axis.mechanical_velocity,
            },
            dt_seconds,
        );

        let output = self.runtime.controller.step(
            FastLoopInput {
                phase_currents: current.currents,
                bus_voltage,
                winding_temperature_c,
                rotor: RotorEstimate {
                    mechanical_angle: rotor_motion.unwrapped(),
                    mechanical_velocity: rotor_motion.velocity(),
                },
                actuator: ActuatorEstimate {
                    output_angle: output_motion.unwrapped(),
                    output_velocity: output_motion.velocity(),
                },
                dt_seconds,
            },
            dt_seconds,
        );

        self.runtime
            .hardware
            .pwm
            .set_phase_duty(output.phase_duty)
            .map_err(MotorRuntimeError::Pwm)?;

        let output = MotorRuntimeOutput::from(output);
        self.publish_runtime_status(Some(output));
        Ok(output)
    }

    #[inline]
    pub(crate) fn execute_fast_cycle(
        &mut self,
        dt_seconds: f32,
    ) -> Result<
        MotorRuntimeOutput,
        MotorRuntimeError<
            PWM::Error,
            CURRENT::Error,
            BUS::Error,
            ROTOR::Error,
            OUTPUT::Error,
            TEMP::Error,
        >,
    > {
        self.run_cycle(dt_seconds)
    }
}

#[cfg(test)]
mod tests {
    use core::convert::Infallible;

    use fluxkit_core::{
        ActuatorLimits, ActuatorModel, ActuatorParams, ControlMode, CurrentLoopConfig,
        InverterParams, MotorLimits, MotorModel, MotorParams, MotorState,
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

    use super::{Hardware, MotorRuntime, MotorRuntimeError};

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
                    self.output = AngularEstimate::new(
                        wrapped_value.wrapped(),
                        wrapped_value,
                        RadPerSec::ZERO,
                    );
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
    ) -> Hardware<FakePwm, FakeCurrentSensor, FakeBusSensor, FakeRotor, FakeOutput, FakeTempSensor>
    {
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
            fluxkit_math::PassThroughEstimator::new(),
            fluxkit_math::PassThroughEstimator::new(),
        );
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
            fluxkit_math::PassThroughEstimator::new(),
            fluxkit_math::PassThroughEstimator::new(),
        );
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
            fluxkit_math::PassThroughEstimator::new(),
            fluxkit_math::PassThroughEstimator::new(),
        );
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
        );
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
            fluxkit_math::PassThroughEstimator::new(),
            fluxkit_math::PassThroughEstimator::new(),
        );
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
            fluxkit_math::PassThroughEstimator::new(),
            fluxkit_math::PassThroughEstimator::new(),
        );
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
            fluxkit_math::PassThroughEstimator::new(),
            fluxkit_math::PassThroughEstimator::new(),
        );
        let (handle, ticker) = system.split().expect("runtime should split once");

        let _parts = system
            .try_into_parts()
            .expect("runtime parts should be available");

        assert!(!handle.status().active);
        handle.arm();
        assert!(!handle.status().armed);
        assert!(matches!(ticker.tick(), Err(MotorRuntimeError::Inactive)));
    }
}
