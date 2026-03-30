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

impl<PWM, CURRENT, BUS, ROTOR, OUTPUT, TEMP, MOD, RotorEst, OutputEst>
    ActuatorCalibrationRuntime<PWM, CURRENT, BUS, ROTOR, OUTPUT, TEMP, MOD, RotorEst, OutputEst>
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
    /// Creates a new actuator-calibration runtime without requiring predefined
    /// actuator parameters.
    ///
    /// Internally this builds a `MotorController` with a neutral actuator
    /// placeholder:
    ///
    /// - `gear_ratio = 1.0`
    /// - output-axis velocity limit copied from calibration limits
    /// - output torque left unconstrained by placeholder actuator limits so the
    ///   calibration routines can still establish motion before gear ratio and
    ///   friction are known
    /// - friction compensation disabled
    ///
    /// The intended high-level flow is to run gear-ratio calibration first and
    /// then let subsequent completed actuator-calibration deltas patch the live
    /// controller parameters automatically.
    pub fn new(
        pwm: PWM,
        current: CURRENT,
        bus: BUS,
        rotor: ROTOR,
        output: OUTPUT,
        temp: TEMP,
        motor: MotorParams,
        inverter: InverterParams,
        config: CurrentLoopConfig,
        modulator: MOD,
        rotor_estimator: RotorEst,
        output_estimator: OutputEst,
        request: ActuatorCalibrationRequest,
        limits: ActuatorCalibrationLimits,
        dt_seconds: f32,
    ) -> Result<Self, CalibrationError> {
        Self::from_parts(
            MotorRuntimeParts {
                pwm,
                current,
                bus,
                rotor,
                output,
                temp,
                motor,
                inverter,
                actuator: placeholder_actuator_params(limits),
                current_loop: config,
                modulator,
                rotor_estimator,
                output_estimator,
            },
            request,
            limits,
            dt_seconds,
        )
    }

    /// Creates a new actuator-calibration runtime from owned runtime parts.
    pub fn from_parts(
        parts: MotorRuntimeParts<PWM, CURRENT, BUS, ROTOR, OUTPUT, TEMP, MOD, RotorEst, OutputEst>,
        request: ActuatorCalibrationRequest,
        limits: ActuatorCalibrationLimits,
        dt_seconds: f32,
    ) -> Result<Self, CalibrationError> {
        if !validate_limits(limits) || !validate_dt_seconds(dt_seconds) {
            return Err(CalibrationError::InvalidConfiguration);
        }

        let motor_system = MotorRuntime::from_parts(parts, dt_seconds);

        let mut inner = InnerActuatorCalibrationRuntime {
            motor_system,
            limits,
            dt_seconds,
            gear_ratio: request.gear_ratio,
            positive_breakaway_torque: request.positive_breakaway_torque,
            negative_breakaway_torque: request.negative_breakaway_torque,
            positive_coulomb_torque: request.positive_coulomb_torque,
            negative_coulomb_torque: request.negative_coulomb_torque,
            positive_viscous_coefficient: request.positive_viscous_coefficient,
            negative_viscous_coefficient: request.negative_viscous_coefficient,
            zero_velocity_blend_band: request.zero_velocity_blend_band,
            active_routine: None,
        };
        let partial = inner.partial_calibration();
        inner.apply_live_calibration(&partial);
        Ok(Self {
            inner: Mutex::new(RefCell::new(Some(inner))),
            shared: Mutex::new(RefCell::new(SharedStatus {
                status: ActuatorCalibrationStatus {
                    active: true,
                    phase: next_phase_for_request(request),
                    result: None,
                    fault_latched: false,
                },
            })),
            split_taken: Cell::new(false),
        })
    }

    /// Splits this calibration runtime into its unique handle and ticker.
    ///
    /// This can be called at most once for a given calibration owner.
    #[inline]
    pub fn split(
        &self,
    ) -> Result<
        (
            ActuatorCalibrationHandle<'_>,
            ActuatorCalibrationTicker<
                '_,
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
        ),
        CapabilitySplitError,
    > {
        split_once(read_status(&self.shared).active, &self.split_taken)?;
        Ok((
            ActuatorCalibrationHandle {
                shared: &self.shared,
            },
            ActuatorCalibrationTicker {
                inner: &self.inner,
                shared: &self.shared,
            },
        ))
    }

    /// Attempts to take ownership of the active runtime parts for reuse in another phase.
    #[inline]
    pub fn try_into_parts(
        &self,
    ) -> Option<MotorRuntimeParts<PWM, CURRENT, BUS, ROTOR, OUTPUT, TEMP, MOD, RotorEst, OutputEst>>
    {
        let inner = critical_section::with(|cs| self.inner.borrow(cs).borrow_mut().take())?;
        write_status(&self.shared, |status| status.active = false);
        inner.motor_system.try_into_parts()
    }
}

impl<PWM, CURRENT, BUS, ROTOR, OUTPUT, TEMP, MOD, RotorEst, OutputEst>
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
    >
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
    fn phase(&self) -> Option<ActuatorCalibrationPhase> {
        self.active_routine
            .as_ref()
            .map(ActuatorCalibrationPhase::from)
            .or_else(|| self.next_phase())
    }

    pub fn tick(
        &mut self,
        shared: &Mutex<RefCell<SharedStatus<ActuatorCalibrationStatus>>>,
    ) -> Result<
        (),
        ActuatorCalibrationRuntimeError<
            PWM::Error,
            CURRENT::Error,
            BUS::Error,
            ROTOR::Error,
            OUTPUT::Error,
            TEMP::Error,
        >,
    > {
        let result = self.tick_inner();
        match result {
            Ok(_) => {
                self.publish_status(shared, false);
                Ok(())
            }
            Err(error) => {
                self.publish_status(shared, true);
                Err(error)
            }
        }
    }

    fn tick_inner(
        &mut self,
    ) -> Result<
        Option<()>,
        ActuatorCalibrationRuntimeError<
            PWM::Error,
            CURRENT::Error,
            BUS::Error,
            ROTOR::Error,
            OUTPUT::Error,
            TEMP::Error,
        >,
    > {
        let partial = self.partial_calibration();
        self.apply_live_calibration(&partial);

        if self.active_routine.is_none() {
            self.active_routine = self
                .build_next_routine()
                .map_err(ActuatorCalibrationRuntimeError::Calibration)?;
            if self.active_routine.is_none() {
                self.resolve_calibration()
                    .map_err(ActuatorCalibrationRuntimeError::Calibration)?;
                return Ok(Some(()));
            }
        }

        let mut routine = self
            .active_routine
            .take()
            .expect("active routine must exist");
        if let Some(delta) = self.tick_active_routine(&mut routine)? {
            self.merge_partial(delta);
            if self
                .build_next_routine()
                .map_err(ActuatorCalibrationRuntimeError::Calibration)?
                .is_none()
            {
                self.resolve_calibration()
                    .map_err(ActuatorCalibrationRuntimeError::Calibration)?;
                return Ok(Some(()));
            }
        } else {
            self.active_routine = Some(routine);
        }

        Ok(None)
    }

    fn publish_status(
        &self,
        shared: &Mutex<RefCell<SharedStatus<ActuatorCalibrationStatus>>>,
        fault_latched: bool,
    ) {
        write_status(shared, |status| {
            *status = ActuatorCalibrationStatus {
                active: true,
                phase: self.phase(),
                result: self.resolve_calibration().ok(),
                fault_latched,
            };
        });
    }

    fn next_phase(&self) -> Option<ActuatorCalibrationPhase> {
        if self.gear_ratio.is_none() {
            return Some(ActuatorCalibrationPhase::GearRatio);
        }
        if self.positive_coulomb_torque.is_none()
            || self.negative_coulomb_torque.is_none()
            || self.positive_viscous_coefficient.is_none()
            || self.negative_viscous_coefficient.is_none()
        {
            return Some(ActuatorCalibrationPhase::Friction);
        }
        if self.positive_breakaway_torque.is_none() || self.negative_breakaway_torque.is_none() {
            return Some(ActuatorCalibrationPhase::Breakaway);
        }
        if self.zero_velocity_blend_band.is_none() {
            return Some(ActuatorCalibrationPhase::BlendBand);
        }
        None
    }

    fn tick_active_routine(
        &mut self,
        routine: &mut ActuatorCalibrationRoutine,
    ) -> Result<
        Option<PartialActuatorCalibration>,
        ActuatorCalibrationRuntimeError<
            PWM::Error,
            CURRENT::Error,
            BUS::Error,
            ROTOR::Error,
            OUTPUT::Error,
            TEMP::Error,
        >,
    > {
        match routine {
            ActuatorCalibrationRoutine::GearRatio(calibrator) => self.tick_gear_ratio(calibrator),
            ActuatorCalibrationRoutine::Friction(calibrator) => self.tick_friction(calibrator),
            ActuatorCalibrationRoutine::Breakaway(calibrator) => self.tick_breakaway(calibrator),
            ActuatorCalibrationRoutine::BlendBand(calibrator) => self.tick_blend_band(calibrator),
        }
    }

    fn build_next_routine(&self) -> Result<Option<ActuatorCalibrationRoutine>, CalibrationError> {
        let limits = self.limits;

        if self.gear_ratio.is_none() {
            let mut cfg =
                fluxkit_core::ActuatorGearRatioCalibrationConfig::default_for_travel_ratio();
            cfg.velocity_target =
                clamp_abs_rad_per_sec(cfg.velocity_target, limits.max_velocity_target);
            cfg.timeout_seconds = cfg.timeout_seconds.min(limits.timeout_seconds);
            return ActuatorGearRatioCalibrator::new(cfg)
                .map(ActuatorCalibrationRoutine::GearRatio)
                .map(Some);
        }

        if self.positive_coulomb_torque.is_none()
            || self.negative_coulomb_torque.is_none()
            || self.positive_viscous_coefficient.is_none()
            || self.negative_viscous_coefficient.is_none()
        {
            let mut cfg =
                fluxkit_core::ActuatorFrictionCalibrationConfig::default_for_velocity_sweep();
            cfg.velocity_points = [
                clamp_abs_rad_per_sec(cfg.velocity_points[0], limits.max_velocity_target),
                clamp_abs_rad_per_sec(cfg.velocity_points[1], limits.max_velocity_target),
                clamp_abs_rad_per_sec(cfg.velocity_points[2], limits.max_velocity_target),
            ];
            cfg.timeout_seconds = cfg.timeout_seconds.min(limits.timeout_seconds);
            return ActuatorFrictionCalibrator::new(cfg)
                .map(ActuatorCalibrationRoutine::Friction)
                .map(Some);
        }

        if self.positive_breakaway_torque.is_none() || self.negative_breakaway_torque.is_none() {
            let mut cfg =
                fluxkit_core::ActuatorBreakawayCalibrationConfig::default_for_torque_ramp();
            cfg.positive_coulomb_torque = self
                .positive_coulomb_torque
                .expect("friction resolved before breakaway");
            cfg.negative_coulomb_torque = self
                .negative_coulomb_torque
                .expect("friction resolved before breakaway");
            cfg.max_torque = min_torque(cfg.max_torque, limits.max_torque_target);
            cfg.timeout_seconds = cfg.timeout_seconds.min(limits.timeout_seconds);
            return fluxkit_core::ActuatorBreakawayCalibrator::new(cfg)
                .map(ActuatorCalibrationRoutine::Breakaway)
                .map(Some);
        }

        if self.zero_velocity_blend_band.is_none() {
            let mut cfg =
                fluxkit_core::ActuatorBlendBandCalibrationConfig::default_for_release_ramp();
            cfg.max_torque = min_torque(cfg.max_torque, limits.max_torque_target);
            cfg.timeout_seconds = cfg.timeout_seconds.min(limits.timeout_seconds);
            return fluxkit_core::ActuatorBlendBandCalibrator::new(cfg)
                .map(ActuatorCalibrationRoutine::BlendBand)
                .map(Some);
        }

        Ok(None)
    }

    fn merge_partial(&mut self, delta: PartialActuatorCalibration) {
        if self.gear_ratio.is_none() {
            if let Some(value) = delta.gear_ratio {
                self.gear_ratio = Some(value);
            }
        }
        if self.positive_breakaway_torque.is_none() {
            if let Some(value) = delta.friction.positive_breakaway_torque {
                self.positive_breakaway_torque = Some(value);
            }
        }
        if self.negative_breakaway_torque.is_none() {
            if let Some(value) = delta.friction.negative_breakaway_torque {
                self.negative_breakaway_torque = Some(value);
            }
        }
        if self.positive_coulomb_torque.is_none() {
            if let Some(value) = delta.friction.positive_coulomb_torque {
                self.positive_coulomb_torque = Some(value);
            }
        }
        if self.negative_coulomb_torque.is_none() {
            if let Some(value) = delta.friction.negative_coulomb_torque {
                self.negative_coulomb_torque = Some(value);
            }
        }
        if self.positive_viscous_coefficient.is_none() {
            if let Some(value) = delta.friction.positive_viscous_coefficient {
                self.positive_viscous_coefficient = Some(value);
            }
        }
        if self.negative_viscous_coefficient.is_none() {
            if let Some(value) = delta.friction.negative_viscous_coefficient {
                self.negative_viscous_coefficient = Some(value);
            }
        }
        if self.zero_velocity_blend_band.is_none() {
            if let Some(value) = delta.friction.zero_velocity_blend_band {
                self.zero_velocity_blend_band = Some(value);
            }
        }
    }

    fn partial_calibration(&self) -> PartialActuatorCalibration {
        PartialActuatorCalibration {
            gear_ratio: self.gear_ratio,
            friction: fluxkit_core::ActuatorFrictionCalibration {
                positive_breakaway_torque: self.positive_breakaway_torque,
                negative_breakaway_torque: self.negative_breakaway_torque,
                positive_coulomb_torque: self.positive_coulomb_torque,
                negative_coulomb_torque: self.negative_coulomb_torque,
                positive_viscous_coefficient: self.positive_viscous_coefficient,
                negative_viscous_coefficient: self.negative_viscous_coefficient,
                zero_velocity_blend_band: self.zero_velocity_blend_band,
            },
        }
    }

    fn resolve_calibration(&self) -> Result<ActuatorCalibrationResult, CalibrationError> {
        Ok(ActuatorCalibrationResult {
            gear_ratio: self
                .gear_ratio
                .ok_or(CalibrationError::InvalidConfiguration)?,
            positive_breakaway_torque: self
                .positive_breakaway_torque
                .ok_or(CalibrationError::InvalidConfiguration)?,
            negative_breakaway_torque: self
                .negative_breakaway_torque
                .ok_or(CalibrationError::InvalidConfiguration)?,
            positive_coulomb_torque: self
                .positive_coulomb_torque
                .ok_or(CalibrationError::InvalidConfiguration)?,
            negative_coulomb_torque: self
                .negative_coulomb_torque
                .ok_or(CalibrationError::InvalidConfiguration)?,
            positive_viscous_coefficient: self
                .positive_viscous_coefficient
                .ok_or(CalibrationError::InvalidConfiguration)?,
            negative_viscous_coefficient: self
                .negative_viscous_coefficient
                .ok_or(CalibrationError::InvalidConfiguration)?,
            zero_velocity_blend_band: self
                .zero_velocity_blend_band
                .ok_or(CalibrationError::InvalidConfiguration)?,
        })
    }

    fn tick_gear_ratio(
        &mut self,
        calibrator: &mut ActuatorGearRatioCalibrator,
    ) -> Result<
        Option<PartialActuatorCalibration>,
        ActuatorCalibrationRuntimeError<
            PWM::Error,
            CURRENT::Error,
            BUS::Error,
            ROTOR::Error,
            OUTPUT::Error,
            TEMP::Error,
        >,
    > {
        self.tick_calibrator(
            calibrator,
            false,
            |calibrator, status, dt| {
                calibrator.tick(ActuatorGearRatioCalibrationInput {
                    rotor_mechanical_angle: status.last_rotor_mechanical_angle,
                    output_mechanical_angle: status.last_output_mechanical_angle,
                    output_velocity: status.last_output_mechanical_velocity,
                    dt_seconds: dt,
                })
            },
            |motor_system, command| {
                motor_system.apply_command_immediate(crate::MotorCommand::Velocity(
                    command.velocity_target,
                ));
            },
        )
    }

    fn tick_friction(
        &mut self,
        calibrator: &mut ActuatorFrictionCalibrator,
    ) -> Result<
        Option<PartialActuatorCalibration>,
        ActuatorCalibrationRuntimeError<
            PWM::Error,
            CURRENT::Error,
            BUS::Error,
            ROTOR::Error,
            OUTPUT::Error,
            TEMP::Error,
        >,
    > {
        self.tick_calibrator(
            calibrator,
            true,
            |calibrator, status, dt| {
                calibrator.tick(ActuatorFrictionCalibrationInput {
                    output_velocity: status.last_output_mechanical_velocity,
                    output_torque_command: status
                        .last_actuator_compensation
                        .total_output_torque_command,
                    dt_seconds: dt,
                })
            },
            |motor_system, command| {
                motor_system.apply_command_immediate(crate::MotorCommand::Velocity(
                    command.velocity_target,
                ));
            },
        )
    }

    fn tick_breakaway(
        &mut self,
        calibrator: &mut ActuatorBreakawayCalibrator,
    ) -> Result<
        Option<PartialActuatorCalibration>,
        ActuatorCalibrationRuntimeError<
            PWM::Error,
            CURRENT::Error,
            BUS::Error,
            ROTOR::Error,
            OUTPUT::Error,
            TEMP::Error,
        >,
    > {
        self.tick_calibrator(
            calibrator,
            true,
            |calibrator, status, dt| {
                calibrator.tick(ActuatorBreakawayCalibrationInput {
                    output_velocity: status.last_output_mechanical_velocity,
                    output_torque_command: status
                        .last_actuator_compensation
                        .total_output_torque_command,
                    dt_seconds: dt,
                })
            },
            |motor_system, command| {
                motor_system
                    .apply_command_immediate(crate::MotorCommand::Torque(command.torque_target));
            },
        )
    }

    fn tick_blend_band(
        &mut self,
        calibrator: &mut ActuatorBlendBandCalibrator,
    ) -> Result<
        Option<PartialActuatorCalibration>,
        ActuatorCalibrationRuntimeError<
            PWM::Error,
            CURRENT::Error,
            BUS::Error,
            ROTOR::Error,
            OUTPUT::Error,
            TEMP::Error,
        >,
    > {
        self.tick_calibrator(
            calibrator,
            true,
            |calibrator, status, dt| {
                calibrator.tick(ActuatorBlendBandCalibrationInput {
                    output_velocity: status.last_output_mechanical_velocity,
                    output_torque_command: status
                        .last_actuator_compensation
                        .total_output_torque_command,
                    dt_seconds: dt,
                })
            },
            |motor_system, command| {
                motor_system
                    .apply_command_immediate(crate::MotorCommand::Torque(command.torque_target));
            },
        )
    }

    fn tick_calibrator<R, Cal, Command, Build, Apply>(
        &mut self,
        calibrator: &mut Cal,
        require_friction_disabled: bool,
        build_command: Build,
        apply_command: Apply,
    ) -> Result<
        Option<PartialActuatorCalibration>,
        ActuatorCalibrationRuntimeError<
            PWM::Error,
            CURRENT::Error,
            BUS::Error,
            ROTOR::Error,
            OUTPUT::Error,
            TEMP::Error,
        >,
    >
    where
        Cal: RoutineState<R>,
        R: Into<PartialActuatorCalibration>,
        Build: FnOnce(&mut Cal, MotorStatus, f32) -> Command,
        Apply: FnOnce(
            &mut MotorRuntime<PWM, CURRENT, BUS, ROTOR, OUTPUT, TEMP, MOD, RotorEst, OutputEst>,
            Command,
        ),
    {
        if let Some(result) = self.preflight(calibrator)? {
            return Ok(result);
        }

        self.prepare_motor(require_friction_disabled)?;
        let status = self.motor_system.controller_status();
        let command = build_command(calibrator, status, self.dt_seconds);
        apply_command(&mut self.motor_system, command);
        let _ = self
            .motor_system
            .ticker_internal()
            .tick()
            .map_err(ActuatorCalibrationRuntimeError::Motor)?;

        self.postflight(calibrator)
    }

    fn prepare_motor(
        &mut self,
        require_friction_disabled: bool,
    ) -> Result<
        (),
        ActuatorCalibrationRuntimeError<
            PWM::Error,
            CURRENT::Error,
            BUS::Error,
            ROTOR::Error,
            OUTPUT::Error,
            TEMP::Error,
        >,
    > {
        if require_friction_disabled && self.motor_system.friction_compensation_enabled() {
            self.motor_system
                .set_armed_immediate(false)
                .map_err(ActuatorCalibrationRuntimeError::Motor)?;
            return Err(ActuatorCalibrationRuntimeError::Calibration(
                CalibrationError::InvalidConfiguration,
            ));
        }

        self.motor_system
            .set_armed_immediate(true)
            .map_err(ActuatorCalibrationRuntimeError::Motor)?;
        Ok(())
    }

    fn preflight<R, Cal>(
        &mut self,
        calibrator: &Cal,
    ) -> Result<
        Option<Option<PartialActuatorCalibration>>,
        ActuatorCalibrationRuntimeError<
            PWM::Error,
            CURRENT::Error,
            BUS::Error,
            ROTOR::Error,
            OUTPUT::Error,
            TEMP::Error,
        >,
    >
    where
        Cal: RoutineState<R>,
        R: Into<PartialActuatorCalibration>,
    {
        if let Some(result) = calibrator.result() {
            self.motor_system
                .set_armed_immediate(false)
                .map_err(ActuatorCalibrationRuntimeError::Motor)?;
            let delta = result.into();
            self.apply_live_calibration(&delta);
            return Ok(Some(Some(delta)));
        }
        if let Some(error) = calibrator.error() {
            self.motor_system
                .set_armed_immediate(false)
                .map_err(ActuatorCalibrationRuntimeError::Motor)?;
            return Err(ActuatorCalibrationRuntimeError::Calibration(error));
        }
        Ok(None)
    }

    fn postflight<R, Cal>(
        &mut self,
        calibrator: &Cal,
    ) -> Result<
        Option<PartialActuatorCalibration>,
        ActuatorCalibrationRuntimeError<
            PWM::Error,
            CURRENT::Error,
            BUS::Error,
            ROTOR::Error,
            OUTPUT::Error,
            TEMP::Error,
        >,
    >
    where
        Cal: RoutineState<R>,
        R: Into<PartialActuatorCalibration>,
    {
        if let Some(result) = calibrator.result() {
            self.motor_system
                .set_armed_immediate(false)
                .map_err(ActuatorCalibrationRuntimeError::Motor)?;
            let delta = result.into();
            self.apply_live_calibration(&delta);
            Ok(Some(delta))
        } else if let Some(error) = calibrator.error() {
            self.motor_system
                .set_armed_immediate(false)
                .map_err(ActuatorCalibrationRuntimeError::Motor)?;
            Err(ActuatorCalibrationRuntimeError::Calibration(error))
        } else {
            Ok(None)
        }
    }

    fn apply_live_calibration(&mut self, calibration: &PartialActuatorCalibration) {
        self.motor_system.apply_actuator_calibration(calibration);
    }
}

impl<'a, PWM, CURRENT, BUS, ROTOR, OUTPUT, TEMP, MOD, RotorEst, OutputEst>
    ActuatorCalibrationTicker<'a, PWM, CURRENT, BUS, ROTOR, OUTPUT, TEMP, MOD, RotorEst, OutputEst>
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
    /// Advances the request-driven actuator calibration campaign by one fixed-period step.
    pub fn tick(
        &self,
    ) -> Result<
        (),
        ActuatorCalibrationRuntimeError<
            PWM::Error,
            CURRENT::Error,
            BUS::Error,
            ROTOR::Error,
            OUTPUT::Error,
            TEMP::Error,
        >,
    > {
        let mut inner = take_active_inner(
            &self.inner,
            || read_status(&self.shared).active,
            |active| {
                if active {
                    ActuatorCalibrationRuntimeError::Busy
                } else {
                    ActuatorCalibrationRuntimeError::Inactive
                }
            },
        )?;
        let result = inner.tick(self.shared);
        critical_section::with(|cs| {
            *self.inner.borrow(cs).borrow_mut() = Some(inner);
        });
        result
    }
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
