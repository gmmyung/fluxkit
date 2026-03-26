use core::fmt;

use fluxkit_core::{
    ActuatorBlendBandCalibrationInput, ActuatorBlendBandCalibrator,
    ActuatorBreakawayCalibrationInput, ActuatorBreakawayCalibrator,
    ActuatorCalibration as PartialActuatorCalibration, ActuatorCalibrationRoutine,
    ActuatorCompensationConfig, ActuatorFrictionCalibrationInput, ActuatorFrictionCalibrator,
    ActuatorGearRatioCalibrationInput, ActuatorGearRatioCalibrator, ActuatorLimits, ActuatorModel,
    ActuatorParams, CalibrationError, ControlMode, CurrentLoopConfig, InverterParams, MotorParams,
    MotorState, MotorStatus,
};
use fluxkit_hal::{
    BusVoltageSensor, CurrentSampler, OutputSensor, PhasePwm, RotorSensor, TemperatureSensor,
};
use fluxkit_math::{
    Modulator,
    units::{NewtonMeters, RadPerSec},
};

use super::shared::RoutineState;
use crate::{MotorHardware, MotorSystem, MotorSystemError, system::MechanicalMotionEstimator};

/// HAL and integration failures that can occur while running actuator-side
/// calibration through the full public motor-system wrapper.
#[derive(Debug)]
pub enum ActuatorCalibrationSystemError<PwmE, CurrentE, BusE, RotorE, OutputE, TempE> {
    /// Underlying motor-system operation failed.
    Motor(MotorSystemError<PwmE, CurrentE, BusE, RotorE, OutputE, TempE>),
    /// The pure core calibration procedure failed.
    Calibration(CalibrationError),
}

impl<PwmE, CurrentE, BusE, RotorE, OutputE, TempE> fmt::Display
    for ActuatorCalibrationSystemError<PwmE, CurrentE, BusE, RotorE, OutputE, TempE>
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
            Self::Motor(error) => write!(f, "motor-system error: {error}"),
            Self::Calibration(error) => write!(f, "calibration error: {error}"),
        }
    }
}

impl<PwmE, CurrentE, BusE, RotorE, OutputE, TempE> core::error::Error
    for ActuatorCalibrationSystemError<PwmE, CurrentE, BusE, RotorE, OutputE, TempE>
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
        limits: fluxkit_core::ActuatorLimits,
        compensation: fluxkit_core::ActuatorCompensationConfig,
    ) -> fluxkit_core::ActuatorParams {
        let mut actuator = fluxkit_core::ActuatorParams::from_model_limits_and_compensation(
            fluxkit_core::ActuatorModel {
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
    pub fn into_uncompensated_actuator_params(
        self,
        limits: fluxkit_core::ActuatorLimits,
    ) -> fluxkit_core::ActuatorParams {
        self.into_actuator_params(limits, fluxkit_core::ActuatorCompensationConfig::disabled())
    }

    /// Builds actuator parameters with friction compensation enabled from this
    /// resolved calibration.
    ///
    /// This is the common runtime path once actuator friction has been
    /// calibrated and you want the controller to use it.
    #[inline]
    pub fn into_friction_compensated_actuator_params(
        self,
        limits: fluxkit_core::ActuatorLimits,
        max_total_torque: NewtonMeters,
    ) -> fluxkit_core::ActuatorParams {
        let mut compensation = fluxkit_core::ActuatorCompensationConfig::disabled();
        compensation.friction.enabled = true;
        compensation.max_total_torque = max_total_torque;
        self.into_actuator_params(limits, compensation)
    }

    /// Applies this resolved record onto an existing actuator-parameter record.
    #[inline]
    pub fn apply_to_actuator_params(&self, actuator: &mut fluxkit_core::ActuatorParams) {
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

/// Encapsulated synchronous actuator-calibration stack built on the public
/// `MotorSystem` wrapper.
#[derive(Debug)]
pub struct ActuatorCalibrationSystem<
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
    motor_system: MotorSystem<PWM, CURRENT, BUS, ROTOR, OUTPUT, TEMP, MOD, RotorEst, OutputEst>,
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

impl<PWM, CURRENT, BUS, ROTOR, OUTPUT, TEMP, MOD, RotorEst, OutputEst>
    ActuatorCalibrationSystem<PWM, CURRENT, BUS, ROTOR, OUTPUT, TEMP, MOD, RotorEst, OutputEst>
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
    /// Creates a new actuator-calibration system without requiring predefined
    /// actuator parameters.
    ///
    /// Internally this builds a `MotorController` with a neutral actuator
    /// placeholder:
    ///
    /// - `gear_ratio = 1.0`
    /// - output-axis limits copied from calibration limits
    /// - friction compensation disabled
    ///
    /// The intended high-level flow is to run gear-ratio calibration first and
    /// then let subsequent completed actuator-calibration deltas patch the live
    /// controller parameters automatically.
    pub fn new(
        hardware: MotorHardware<PWM, CURRENT, BUS, ROTOR, OUTPUT, TEMP>,
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
        if !validate_limits(limits) || !validate_dt_seconds(dt_seconds) {
            return Err(CalibrationError::InvalidConfiguration);
        }

        let controller = fluxkit_core::MotorController::new_with_modulator(
            motor,
            inverter,
            placeholder_actuator_params(limits),
            config,
            modulator,
        );
        let motor_system = MotorSystem::new(
            hardware,
            controller,
            rotor_estimator,
            output_estimator,
            dt_seconds,
        );

        let mut system = Self {
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
        let partial = system.partial_calibration();
        system.apply_live_calibration(&partial);
        Ok(system)
    }

    /// Returns shared access to the owned motor system.
    #[inline]
    pub const fn motor_system(
        &self,
    ) -> &MotorSystem<PWM, CURRENT, BUS, ROTOR, OUTPUT, TEMP, MOD, RotorEst, OutputEst> {
        &self.motor_system
    }

    /// Returns mutable access to the owned motor system.
    #[inline]
    pub fn motor_system_mut(
        &mut self,
    ) -> &mut MotorSystem<PWM, CURRENT, BUS, ROTOR, OUTPUT, TEMP, MOD, RotorEst, OutputEst> {
        &mut self.motor_system
    }

    /// Splits the actuator-calibration system back into the owned motor system.
    #[inline]
    pub fn into_motor_system(
        self,
    ) -> MotorSystem<PWM, CURRENT, BUS, ROTOR, OUTPUT, TEMP, MOD, RotorEst, OutputEst> {
        self.motor_system
    }

    /// Returns the current or next calibration phase, if the campaign is not complete.
    #[inline]
    pub fn phase(&self) -> Option<ActuatorCalibrationPhase> {
        self.active_routine
            .as_ref()
            .map(ActuatorCalibrationPhase::from)
            .or_else(|| self.next_phase())
    }

    /// Advances the request-driven actuator calibration campaign by one fixed-period step.
    pub fn tick(
        &mut self,
    ) -> Result<
        Option<ActuatorCalibrationResult>,
        ActuatorCalibrationSystemError<
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
                .map_err(ActuatorCalibrationSystemError::Calibration)?;
            if self.active_routine.is_none() {
                return Ok(Some(
                    self.resolve_calibration()
                        .map_err(ActuatorCalibrationSystemError::Calibration)?,
                ));
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
                .map_err(ActuatorCalibrationSystemError::Calibration)?
                .is_none()
            {
                return Ok(Some(
                    self.resolve_calibration()
                        .map_err(ActuatorCalibrationSystemError::Calibration)?,
                ));
            }
        } else {
            self.active_routine = Some(routine);
        }

        Ok(None)
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
        ActuatorCalibrationSystemError<
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
        ActuatorCalibrationSystemError<
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
            ControlMode::Velocity,
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
                motor_system
                    .controller_mut()
                    .set_velocity_target(command.velocity_target);
            },
        )
    }

    fn tick_friction(
        &mut self,
        calibrator: &mut ActuatorFrictionCalibrator,
    ) -> Result<
        Option<PartialActuatorCalibration>,
        ActuatorCalibrationSystemError<
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
            ControlMode::Velocity,
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
                motor_system
                    .controller_mut()
                    .set_velocity_target(command.velocity_target);
            },
        )
    }

    fn tick_breakaway(
        &mut self,
        calibrator: &mut ActuatorBreakawayCalibrator,
    ) -> Result<
        Option<PartialActuatorCalibration>,
        ActuatorCalibrationSystemError<
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
            ControlMode::Torque,
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
                    .controller_mut()
                    .set_torque_target(command.torque_target);
            },
        )
    }

    fn tick_blend_band(
        &mut self,
        calibrator: &mut ActuatorBlendBandCalibrator,
    ) -> Result<
        Option<PartialActuatorCalibration>,
        ActuatorCalibrationSystemError<
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
            ControlMode::Torque,
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
                    .controller_mut()
                    .set_torque_target(command.torque_target);
            },
        )
    }

    fn tick_calibrator<R, Cal, Command, Build, Apply>(
        &mut self,
        calibrator: &mut Cal,
        mode: ControlMode,
        require_friction_disabled: bool,
        build_command: Build,
        apply_command: Apply,
    ) -> Result<
        Option<PartialActuatorCalibration>,
        ActuatorCalibrationSystemError<
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
            &mut MotorSystem<PWM, CURRENT, BUS, ROTOR, OUTPUT, TEMP, MOD, RotorEst, OutputEst>,
            Command,
        ),
    {
        if let Some(result) = self.preflight(calibrator)? {
            return Ok(result);
        }

        self.prepare_motor(mode, require_friction_disabled)?;
        let status = self.motor_system.controller().status();
        let command = build_command(calibrator, status, self.dt_seconds);
        apply_command(&mut self.motor_system, command);
        let _ = self
            .motor_system
            .tick()
            .map_err(ActuatorCalibrationSystemError::Motor)?;

        self.postflight(calibrator)
    }

    fn prepare_motor(
        &mut self,
        mode: ControlMode,
        require_friction_disabled: bool,
    ) -> Result<
        (),
        ActuatorCalibrationSystemError<
            PWM::Error,
            CURRENT::Error,
            BUS::Error,
            ROTOR::Error,
            OUTPUT::Error,
            TEMP::Error,
        >,
    > {
        if require_friction_disabled
            && self
                .motor_system
                .controller()
                .actuator_params()
                .compensation
                .friction
                .enabled
        {
            self.disable_motor()?;
            return Err(ActuatorCalibrationSystemError::Calibration(
                CalibrationError::InvalidConfiguration,
            ));
        }

        self.motor_system.controller_mut().set_mode(mode);
        if self.motor_system.controller().status().state == MotorState::Disabled {
            self.motor_system
                .enable()
                .map_err(ActuatorCalibrationSystemError::Motor)?;
        }
        Ok(())
    }

    fn preflight<R, Cal>(
        &mut self,
        calibrator: &Cal,
    ) -> Result<
        Option<Option<PartialActuatorCalibration>>,
        ActuatorCalibrationSystemError<
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
            self.disable_motor()?;
            let delta = result.into();
            self.apply_live_calibration(&delta);
            return Ok(Some(Some(delta)));
        }
        if let Some(error) = calibrator.error() {
            self.disable_motor()?;
            return Err(ActuatorCalibrationSystemError::Calibration(error));
        }
        Ok(None)
    }

    fn postflight<R, Cal>(
        &mut self,
        calibrator: &Cal,
    ) -> Result<
        Option<PartialActuatorCalibration>,
        ActuatorCalibrationSystemError<
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
            self.disable_motor()?;
            let delta = result.into();
            self.apply_live_calibration(&delta);
            Ok(Some(delta))
        } else if let Some(error) = calibrator.error() {
            self.disable_motor()?;
            Err(ActuatorCalibrationSystemError::Calibration(error))
        } else {
            Ok(None)
        }
    }

    fn disable_motor(
        &mut self,
    ) -> Result<
        (),
        ActuatorCalibrationSystemError<
            PWM::Error,
            CURRENT::Error,
            BUS::Error,
            ROTOR::Error,
            OUTPUT::Error,
            TEMP::Error,
        >,
    > {
        self.motor_system
            .disable()
            .map_err(ActuatorCalibrationSystemError::Motor)
    }

    fn apply_live_calibration(&mut self, calibration: &PartialActuatorCalibration) {
        calibration
            .apply_to_actuator_params(self.motor_system.controller_mut().actuator_params_mut());
    }
}

#[inline]
fn placeholder_actuator_params(limits: ActuatorCalibrationLimits) -> ActuatorParams {
    ActuatorParams::from_model_limits_and_compensation(
        ActuatorModel { gear_ratio: 1.0 },
        ActuatorLimits {
            max_output_velocity: Some(limits.max_velocity_target),
            max_output_torque: Some(limits.max_torque_target),
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
