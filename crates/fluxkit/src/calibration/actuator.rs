use core::cell::{Cell, RefCell};
use core::fmt;

use critical_section::Mutex;
use fluxkit_core::{
    ActuatorBlendBandCalibrationInput, ActuatorBlendBandCalibrator,
    ActuatorBreakawayCalibrationInput, ActuatorBreakawayCalibrator,
    ActuatorCalibration as PartialActuatorCalibration, ActuatorCalibrationRoutine,
    ActuatorCompensationConfig, ActuatorFrictionCalibrationInput, ActuatorFrictionCalibrator,
    ActuatorGearRatioCalibrationInput, ActuatorGearRatioCalibrator, ActuatorLimits, ActuatorModel,
    ActuatorParams, CalibrationError, CurrentLoopConfig, InverterParams, MotorParams, MotorStatus,
};
use fluxkit_hal::{
    BusVoltageSensor, CurrentSampler, OutputSensor, PhasePwm, RotorSensor, TemperatureSensor,
};
use fluxkit_math::{
    Modulator,
    units::{NewtonMeters, RadPerSec},
};

use super::shared::{RoutineState, SharedStatus, read_status, write_status};
use crate::{
    CapabilitySplitError, MotorRuntime, MotorRuntimeError,
    capability::{split_once, take_active_inner},
    system::{MechanicalMotionEstimator, MotorRuntimeParts},
};

/// HAL and integration failures that can occur while running actuator-side
/// calibration through the public motor-runtime wrapper.
#[derive(Debug)]
pub enum ActuatorCalibrationRuntimeError<PwmE, CurrentE, BusE, RotorE, OutputE, TempE> {
    /// The runtime was temporarily unavailable because another context holds the active inner state.
    Busy,
    /// The runtime owner no longer contains an active inner runtime.
    Inactive,
    /// Underlying motor-runtime operation failed.
    Motor(MotorRuntimeError<PwmE, CurrentE, BusE, RotorE, OutputE, TempE>),
    /// The pure core calibration procedure failed.
    Calibration(CalibrationError),
}

impl<PwmE, CurrentE, BusE, RotorE, OutputE, TempE> fmt::Display
    for ActuatorCalibrationRuntimeError<PwmE, CurrentE, BusE, RotorE, OutputE, TempE>
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
            Self::Motor(error) => write!(f, "motor-runtime error: {error}"),
            Self::Calibration(error) => write!(f, "calibration error: {error}"),
        }
    }
}

impl<PwmE, CurrentE, BusE, RotorE, OutputE, TempE> core::error::Error
    for ActuatorCalibrationRuntimeError<PwmE, CurrentE, BusE, RotorE, OutputE, TempE>
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
            Self::Motor(error) => Some(error),
            Self::Calibration(error) => Some(error),
        }
    }
}

/// User intent for the actuator-side calibration campaign.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct ActuatorCalibrationRequest {
    /// Provided gear ratio. When absent, gear ratio is calibrated.
    ///
    /// Gear-ratio calibration assumes the output sensor uses the same positive
    /// motion convention as the reduced rotor/output axis. If output velocity
    /// appears opposite to the commanded direction, the runtime faults with
    /// [`CalibrationError::OppositeDirection`].
    pub gear_ratio: Option<f32>,
    /// Provided positive-direction Coulomb friction torque.
    pub positive_coulomb_torque: Option<NewtonMeters>,
    /// Provided negative-direction Coulomb friction torque.
    pub negative_coulomb_torque: Option<NewtonMeters>,
    /// Provided positive-direction viscous coefficient.
    pub positive_viscous_coefficient: Option<f32>,
    /// Provided negative-direction viscous coefficient.
    pub negative_viscous_coefficient: Option<f32>,
    /// Provided positive-direction breakaway torque.
    pub positive_breakaway_torque: Option<NewtonMeters>,
    /// Provided negative-direction breakaway torque.
    pub negative_breakaway_torque: Option<NewtonMeters>,
    /// Provided zero-velocity blend band.
    pub zero_velocity_blend_band: Option<RadPerSec>,
}

impl ActuatorCalibrationRequest {
    /// Calibrate every actuator-side quantity.
    #[inline]
    pub fn all() -> Self {
        Self::default()
    }
}

/// User-facing operating limits for the actuator-side calibration campaign.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct ActuatorCalibrationLimits {
    /// Maximum commanded output-axis velocity target.
    pub max_velocity_target: RadPerSec,
    /// Maximum commanded output-axis torque target.
    pub max_torque_target: NewtonMeters,
    /// Absolute timeout cap applied to each actuator-side routine.
    pub timeout_seconds: f32,
}

/// Current or next request-driven actuator-calibration phase.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum ActuatorCalibrationPhase {
    /// Gear-ratio identification.
    GearRatio,
    /// Coulomb and viscous friction identification.
    Friction,
    /// Breakaway-torque identification.
    Breakaway,
    /// Zero-velocity blend-band identification.
    BlendBand,
}

/// Fully resolved actuator-side calibration result returned by the request-driven
/// campaign API.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct ActuatorCalibrationResult {
    /// Mechanical reduction ratio from motor shaft to output axis.
    pub gear_ratio: f32,
    /// Additional startup torque near zero speed in the positive direction.
    pub positive_breakaway_torque: NewtonMeters,
    /// Additional startup torque near zero speed in the negative direction.
    pub negative_breakaway_torque: NewtonMeters,
    /// Constant friction torque while moving in the positive direction.
    pub positive_coulomb_torque: NewtonMeters,
    /// Constant friction torque while moving in the negative direction.
    pub negative_coulomb_torque: NewtonMeters,
    /// Positive-direction viscous coefficient in `Nm / (rad/s)`.
    pub positive_viscous_coefficient: f32,
    /// Negative-direction viscous coefficient in `Nm / (rad/s)`.
    pub negative_viscous_coefficient: f32,
    /// Smoothing band around zero speed for friction blending.
    pub zero_velocity_blend_band: RadPerSec,
}

impl ActuatorCalibrationResult {
    /// Builds actuator parameters directly from this resolved calibration,
    /// output-axis limits, and an explicit compensation policy.
    #[inline]
    pub fn into_actuator_params(
        self,
        limits: ActuatorLimits,
        compensation: ActuatorCompensationConfig,
    ) -> ActuatorParams {
        let mut actuator = ActuatorParams::from_model_limits_and_compensation(
            ActuatorModel {
                gear_ratio: self.gear_ratio,
            },
            limits,
            compensation,
        );
        self.apply_to_actuator_params(&mut actuator);
        actuator
    }

    /// Builds actuator parameters from this resolved calibration and
    /// output-axis limits with compensation disabled by default.
    #[inline]
    pub fn into_uncompensated_actuator_params(self, limits: ActuatorLimits) -> ActuatorParams {
        self.into_actuator_params(limits, ActuatorCompensationConfig::disabled())
    }

    /// Builds actuator parameters with friction compensation enabled from this
    /// resolved calibration.
    ///
    /// This is the common runtime path once actuator friction has been
    /// calibrated and you want the controller to use it.
    #[inline]
    pub fn into_friction_compensated_actuator_params(
        self,
        limits: ActuatorLimits,
        max_total_torque: NewtonMeters,
    ) -> ActuatorParams {
        let mut compensation = ActuatorCompensationConfig::disabled();
        compensation.friction.enabled = true;
        compensation.max_total_torque = max_total_torque;
        self.into_actuator_params(limits, compensation)
    }

    /// Applies this resolved record onto an existing actuator-parameter record.
    #[inline]
    pub fn apply_to_actuator_params(&self, actuator: &mut ActuatorParams) {
        actuator.gear_ratio = self.gear_ratio;
        actuator.compensation.friction.positive_breakaway_torque = self.positive_breakaway_torque;
        actuator.compensation.friction.negative_breakaway_torque = self.negative_breakaway_torque;
        actuator.compensation.friction.positive_coulomb_torque = self.positive_coulomb_torque;
        actuator.compensation.friction.negative_coulomb_torque = self.negative_coulomb_torque;
        actuator.compensation.friction.positive_viscous_coefficient =
            self.positive_viscous_coefficient;
        actuator.compensation.friction.negative_viscous_coefficient =
            self.negative_viscous_coefficient;
        actuator.compensation.friction.zero_velocity_blend_band = self.zero_velocity_blend_band;
    }
}

/// Shared calibration status snapshot for non-owning contexts.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct ActuatorCalibrationStatus {
    /// `true` while the calibration owner still contains an active inner runtime.
    pub active: bool,
    /// Current or next phase while calibration is in progress.
    pub phase: Option<ActuatorCalibrationPhase>,
    /// Final resolved result once calibration completes.
    pub result: Option<ActuatorCalibrationResult>,
    /// `true` when the calibration runtime has latched a terminal fault.
    pub fault_latched: bool,
}

/// Non-owning access to calibration progress and final result.
#[derive(Debug)]
pub struct ActuatorCalibrationHandle<'a> {
    shared: &'a Mutex<RefCell<SharedStatus<ActuatorCalibrationStatus>>>,
}

impl<'a> ActuatorCalibrationHandle<'a> {
    /// Returns the latest shared calibration status.
    pub fn status(&self) -> ActuatorCalibrationStatus {
        read_status(self.shared)
    }

    /// Returns `true` while the owning calibration runtime is still active.
    #[inline]
    pub fn is_active(&self) -> bool {
        self.status().active
    }

    /// Returns `true` once calibration has produced a final result.
    #[inline]
    pub fn is_complete(&self) -> bool {
        self.status().result.is_some()
    }

    /// Returns `true` when calibration has latched a terminal fault.
    #[inline]
    pub fn is_faulted(&self) -> bool {
        self.status().fault_latched
    }
}

/// Active inner actuator-calibration runtime owned by the public wrapper.
#[derive(Debug)]
struct InnerActuatorCalibrationRuntime<
    PWM,
    CURRENT,
    BUS,
    ROTOR,
    OUTPUT,
    TEMP,
    MOD,
    RotorEst,
    OutputEst,
> {
    motor_system: MotorRuntime<PWM, CURRENT, BUS, ROTOR, OUTPUT, TEMP, MOD, RotorEst, OutputEst>,
    limits: ActuatorCalibrationLimits,
    dt_seconds: f32,
    gear_ratio: Option<f32>,
    positive_breakaway_torque: Option<NewtonMeters>,
    negative_breakaway_torque: Option<NewtonMeters>,
    positive_coulomb_torque: Option<NewtonMeters>,
    negative_coulomb_torque: Option<NewtonMeters>,
    positive_viscous_coefficient: Option<f32>,
    negative_viscous_coefficient: Option<f32>,
    zero_velocity_blend_band: Option<RadPerSec>,
    current_phase: Option<ActuatorCalibrationPhase>,
    resolved_result: Option<ActuatorCalibrationResult>,
    active_routine: Option<ActuatorCalibrationRoutine>,
}

/// Main-context owner of one active actuator-calibration runtime.
#[derive(Debug)]
pub struct ActuatorCalibrationRuntime<
    PWM,
    CURRENT,
    BUS,
    ROTOR,
    OUTPUT,
    TEMP,
    MOD,
    RotorEst,
    OutputEst,
> {
    inner: Mutex<
        RefCell<
            Option<
                InnerActuatorCalibrationRuntime<
                    PWM,
                    CURRENT,
                    BUS,
                    ROTOR,
                    OUTPUT,
                    TEMP,
                    MOD,
                    RotorEst,
                    OutputEst,
                >,
            >,
        >,
    >,
    shared: Mutex<RefCell<SharedStatus<ActuatorCalibrationStatus>>>,
    split_taken: Cell<bool>,
}

/// IRQ-side execution capability for one active actuator-calibration runtime.
#[derive(Debug)]
pub struct ActuatorCalibrationTicker<
    'a,
    PWM,
    CURRENT,
    BUS,
    ROTOR,
    OUTPUT,
    TEMP,
    MOD,
    RotorEst,
    OutputEst,
> {
    inner: &'a Mutex<
        RefCell<
            Option<
                InnerActuatorCalibrationRuntime<
                    PWM,
                    CURRENT,
                    BUS,
                    ROTOR,
                    OUTPUT,
                    TEMP,
                    MOD,
                    RotorEst,
                    OutputEst,
                >,
            >,
        >,
    >,
    shared: &'a Mutex<RefCell<SharedStatus<ActuatorCalibrationStatus>>>,
}

fn next_phase_for_request(request: ActuatorCalibrationRequest) -> Option<ActuatorCalibrationPhase> {
    if request.gear_ratio.is_none() {
        return Some(ActuatorCalibrationPhase::GearRatio);
    }
    if request.positive_coulomb_torque.is_none()
        || request.negative_coulomb_torque.is_none()
        || request.positive_viscous_coefficient.is_none()
        || request.negative_viscous_coefficient.is_none()
    {
        return Some(ActuatorCalibrationPhase::Friction);
    }
    if request.positive_breakaway_torque.is_none() || request.negative_breakaway_torque.is_none() {
        return Some(ActuatorCalibrationPhase::Breakaway);
    }
    if request.zero_velocity_blend_band.is_none() {
        return Some(ActuatorCalibrationPhase::BlendBand);
    }
    None
}

fn resolved_result_for_request(
    request: ActuatorCalibrationRequest,
) -> Option<ActuatorCalibrationResult> {
    Some(ActuatorCalibrationResult {
        gear_ratio: request.gear_ratio?,
        positive_breakaway_torque: request.positive_breakaway_torque?,
        negative_breakaway_torque: request.negative_breakaway_torque?,
        positive_coulomb_torque: request.positive_coulomb_torque?,
        negative_coulomb_torque: request.negative_coulomb_torque?,
        positive_viscous_coefficient: request.positive_viscous_coefficient?,
        negative_viscous_coefficient: request.negative_viscous_coefficient?,
        zero_velocity_blend_band: request.zero_velocity_blend_band?,
    })
}

#[inline]
fn placeholder_actuator_params(limits: ActuatorCalibrationLimits) -> ActuatorParams {
    ActuatorParams::from_model_limits_and_compensation(
        ActuatorModel { gear_ratio: 1.0 },
        ActuatorLimits {
            max_output_velocity: Some(limits.max_velocity_target),
            max_output_torque: None,
        },
        ActuatorCompensationConfig::disabled(),
    )
}

#[inline]
fn min_torque(a: NewtonMeters, b: NewtonMeters) -> NewtonMeters {
    NewtonMeters::new(a.get().min(b.get()))
}

#[inline]
fn clamp_abs_rad_per_sec(value: RadPerSec, limit: RadPerSec) -> RadPerSec {
    let capped = value.get().abs().min(limit.get().abs());
    RadPerSec::new(value.get().signum() * capped)
}

#[inline]
fn validate_limits(limits: ActuatorCalibrationLimits) -> bool {
    limits.max_velocity_target.get().is_finite()
        && limits.max_velocity_target.get() > 0.0
        && limits.max_torque_target.get().is_finite()
        && limits.max_torque_target.get() > 0.0
        && limits.timeout_seconds.is_finite()
        && limits.timeout_seconds > 0.0
}

#[inline]
fn validate_dt_seconds(dt_seconds: f32) -> bool {
    dt_seconds.is_finite() && dt_seconds > 0.0
}

impl From<&ActuatorCalibrationRoutine> for ActuatorCalibrationPhase {
    fn from(value: &ActuatorCalibrationRoutine) -> Self {
        match value {
            ActuatorCalibrationRoutine::GearRatio(_) => Self::GearRatio,
            ActuatorCalibrationRoutine::Friction(_) => Self::Friction,
            ActuatorCalibrationRoutine::Breakaway(_) => Self::Breakaway,
            ActuatorCalibrationRoutine::BlendBand(_) => Self::BlendBand,
        }
    }
}

mod execution;
mod runtime;
