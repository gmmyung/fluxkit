//! Actuator breakaway calibration from slow torque ramps.
//!
//! This procedure commands a slow output-torque ramp in each direction until
//! sustained output motion is observed. The observed release torque is reduced
//! by the previously calibrated Coulomb term so the result stores only the
//! additional breakaway component near zero speed.

use fluxkit_math::units::{NewtonMeters, RadPerSec};

use crate::calibration::shared::{
    CalibrationError,
    release_ramp::{BidirectionalRampPhase, BidirectionalRampTracker},
};

/// Static configuration for actuator breakaway calibration.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct ActuatorBreakawayCalibrationConfig {
    /// Previously calibrated positive-direction Coulomb term.
    pub positive_coulomb_torque: NewtonMeters,
    /// Previously calibrated negative-direction Coulomb term.
    pub negative_coulomb_torque: NewtonMeters,
    /// Ramp rate applied in both directions, in `Nm/s`.
    pub torque_ramp_rate_nm_per_sec: f32,
    /// Maximum absolute output torque command used during the search.
    pub max_torque: NewtonMeters,
    /// Sustained output-speed threshold that counts as breakaway.
    pub motion_velocity_threshold: RadPerSec,
    /// Continuous motion time required to declare breakaway.
    pub motion_confirm_time_seconds: f32,
    /// Near-zero speed threshold required before reversing direction.
    pub rest_velocity_threshold: RadPerSec,
    /// Continuous rest time required before reversing direction.
    pub rest_time_seconds: f32,
    /// Absolute timeout for the whole procedure.
    pub timeout_seconds: f32,
}

impl ActuatorBreakawayCalibrationConfig {
    /// Returns a conservative default suitable for simulator-backed tests.
    pub const fn default_for_torque_ramp() -> Self {
        Self {
            positive_coulomb_torque: NewtonMeters::ZERO,
            negative_coulomb_torque: NewtonMeters::ZERO,
            torque_ramp_rate_nm_per_sec: 1.0,
            max_torque: NewtonMeters::new(1.0),
            motion_velocity_threshold: RadPerSec::new(0.05),
            motion_confirm_time_seconds: 0.01,
            rest_velocity_threshold: RadPerSec::new(0.02),
            rest_time_seconds: 0.05,
            timeout_seconds: 4.0,
        }
    }
}

/// One synchronous sample frame for actuator breakaway calibration.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct ActuatorBreakawayCalibrationInput {
    /// Measured output-axis velocity.
    pub output_velocity: RadPerSec,
    /// Actual bounded output torque command applied by the controller.
    pub output_torque_command: NewtonMeters,
    /// Time since the previous calibration tick.
    pub dt_seconds: f32,
}

/// Torque-target command emitted by the breakaway calibrator.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct ActuatorBreakawayCalibrationCommand {
    /// Output-axis torque target for the controller.
    pub torque_target: NewtonMeters,
}

/// Result of a completed actuator breakaway calibration.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct ActuatorBreakawayCalibrationResult {
    /// Additional startup torque near zero speed in the positive direction.
    pub positive_breakaway_torque: NewtonMeters,
    /// Additional startup torque near zero speed in the negative direction.
    pub negative_breakaway_torque: NewtonMeters,
}

/// Pure state machine for actuator breakaway calibration.
#[derive(Clone, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct ActuatorBreakawayCalibrator {
    config: ActuatorBreakawayCalibrationConfig,
    elapsed_seconds: f32,
    ramp: BidirectionalRampTracker,
    positive_result: Option<NewtonMeters>,
    negative_result: Option<NewtonMeters>,
    result: Option<ActuatorBreakawayCalibrationResult>,
    error: Option<CalibrationError>,
}

impl ActuatorBreakawayCalibrator {
    /// Creates a new actuator breakaway calibrator.
    pub fn new(config: ActuatorBreakawayCalibrationConfig) -> Result<Self, CalibrationError> {
        if !validate_config(config) {
            return Err(CalibrationError::InvalidConfiguration);
        }

        Ok(Self {
            config,
            elapsed_seconds: 0.0,
            ramp: BidirectionalRampTracker::new(),
            positive_result: None,
            negative_result: None,
            result: None,
            error: None,
        })
    }

    /// Returns the finished result when calibration has succeeded.
    #[inline]
    pub const fn result(&self) -> Option<ActuatorBreakawayCalibrationResult> {
        self.result
    }

    /// Returns the failure cause when calibration has failed.
    #[inline]
    pub const fn error(&self) -> Option<CalibrationError> {
        self.error
    }

    /// Returns the currently commanded output-axis torque target.
    #[inline]
    pub fn commanded_torque_target(&self) -> NewtonMeters {
        if self.result.is_some() || self.error.is_some() {
            return NewtonMeters::ZERO;
        }

        self.ramp.commanded_torque_target(
            self.config.torque_ramp_rate_nm_per_sec,
            self.config.max_torque,
        )
    }

    /// Advances the calibration procedure by one sample.
    pub fn tick(
        &mut self,
        input: ActuatorBreakawayCalibrationInput,
    ) -> ActuatorBreakawayCalibrationCommand {
        if self.result.is_some() || self.error.is_some() {
            return ActuatorBreakawayCalibrationCommand {
                torque_target: NewtonMeters::ZERO,
            };
        }

        if !validate_input(input) {
            self.error = Some(CalibrationError::InvalidInput);
            return ActuatorBreakawayCalibrationCommand {
                torque_target: NewtonMeters::ZERO,
            };
        }

        self.elapsed_seconds += input.dt_seconds;
        if self.elapsed_seconds >= self.config.timeout_seconds {
            self.error = Some(CalibrationError::Timeout);
            return ActuatorBreakawayCalibrationCommand {
                torque_target: NewtonMeters::ZERO,
            };
        }

        match self.ramp.phase {
            BidirectionalRampPhase::PositiveRamp => self.tick_positive_ramp(input),
            BidirectionalRampPhase::NeutralSettle => self.tick_neutral_settle(input),
            BidirectionalRampPhase::NegativeRamp => self.tick_negative_ramp(input),
        }

        ActuatorBreakawayCalibrationCommand {
            torque_target: self.commanded_torque_target(),
        }
    }

    fn tick_positive_ramp(&mut self, input: ActuatorBreakawayCalibrationInput) {
        let ramp_torque = self.ramp.advance_ramp(
            input.dt_seconds,
            self.config.torque_ramp_rate_nm_per_sec,
            self.config.max_torque,
        );
        if let Some(release_torque) = self.ramp.observe_release(
            input.output_velocity.get() >= self.config.motion_velocity_threshold.get(),
            input.dt_seconds,
            input.output_torque_command.get().abs(),
            self.config.motion_confirm_time_seconds,
        ) {
            let breakaway = (release_torque - self.config.positive_coulomb_torque.get()).max(0.0);
            self.positive_result = Some(NewtonMeters::new(breakaway));
            self.ramp.begin_neutral_settle();
            return;
        }

        if ramp_torque >= self.config.max_torque.get() {
            self.error = Some(CalibrationError::IndeterminateEstimate);
        }
    }

    fn tick_neutral_settle(&mut self, input: ActuatorBreakawayCalibrationInput) {
        if self.ramp.observe_rest(
            input.output_velocity.get().abs() <= self.config.rest_velocity_threshold.get(),
            input.dt_seconds,
            self.config.rest_time_seconds,
        ) {
            self.ramp.begin_negative_ramp();
        }
    }

    fn tick_negative_ramp(&mut self, input: ActuatorBreakawayCalibrationInput) {
        let ramp_torque = self.ramp.advance_ramp(
            input.dt_seconds,
            self.config.torque_ramp_rate_nm_per_sec,
            self.config.max_torque,
        );
        if let Some(release_torque) = self.ramp.observe_release(
            input.output_velocity.get() <= -self.config.motion_velocity_threshold.get(),
            input.dt_seconds,
            input.output_torque_command.get().abs(),
            self.config.motion_confirm_time_seconds,
        ) {
            let breakaway = (release_torque - self.config.negative_coulomb_torque.get()).max(0.0);
            self.negative_result = Some(NewtonMeters::new(breakaway));
            self.result = Some(ActuatorBreakawayCalibrationResult {
                positive_breakaway_torque: self.positive_result.unwrap_or(NewtonMeters::ZERO),
                negative_breakaway_torque: NewtonMeters::new(breakaway),
            });
            return;
        }

        if ramp_torque >= self.config.max_torque.get() {
            self.error = Some(CalibrationError::IndeterminateEstimate);
        }
    }
}

fn validate_config(config: ActuatorBreakawayCalibrationConfig) -> bool {
    config.positive_coulomb_torque.get().is_finite()
        && config.positive_coulomb_torque.get() >= 0.0
        && config.negative_coulomb_torque.get().is_finite()
        && config.negative_coulomb_torque.get() >= 0.0
        && config.torque_ramp_rate_nm_per_sec.is_finite()
        && config.torque_ramp_rate_nm_per_sec > 0.0
        && config.max_torque.get().is_finite()
        && config.max_torque.get() > 0.0
        && config.motion_velocity_threshold.get().is_finite()
        && config.motion_velocity_threshold.get() > 0.0
        && config.motion_confirm_time_seconds.is_finite()
        && config.motion_confirm_time_seconds > 0.0
        && config.rest_velocity_threshold.get().is_finite()
        && config.rest_velocity_threshold.get() >= 0.0
        && config.rest_time_seconds.is_finite()
        && config.rest_time_seconds > 0.0
        && config.timeout_seconds.is_finite()
        && config.timeout_seconds
            > 2.0 * config.max_torque.get() / config.torque_ramp_rate_nm_per_sec
}

fn validate_input(input: ActuatorBreakawayCalibrationInput) -> bool {
    input.output_velocity.get().is_finite()
        && input.output_torque_command.get().is_finite()
        && input.dt_seconds.is_finite()
        && input.dt_seconds > 0.0
}

#[cfg(test)]
mod tests {
    use crate::CalibrationError;

    use super::{
        ActuatorBreakawayCalibrationConfig, ActuatorBreakawayCalibrationInput,
        ActuatorBreakawayCalibrator,
    };
    use fluxkit_math::units::{NewtonMeters, RadPerSec};

    #[test]
    fn synthetic_torque_ramp_recovers_breakaway() {
        let mut calibrator = ActuatorBreakawayCalibrator::new(ActuatorBreakawayCalibrationConfig {
            positive_coulomb_torque: NewtonMeters::new(0.04),
            negative_coulomb_torque: NewtonMeters::new(0.05),
            torque_ramp_rate_nm_per_sec: 1.0,
            max_torque: NewtonMeters::new(0.3),
            motion_velocity_threshold: RadPerSec::new(0.05),
            motion_confirm_time_seconds: 0.01,
            rest_velocity_threshold: RadPerSec::new(0.02),
            rest_time_seconds: 0.02,
            timeout_seconds: 2.0,
        })
        .unwrap();

        loop {
            let command = calibrator.commanded_torque_target();
            let velocity = if command.get() >= 0.13 {
                RadPerSec::new(0.1)
            } else if command.get() <= -0.16 {
                RadPerSec::new(-0.1)
            } else {
                RadPerSec::ZERO
            };
            let _ = calibrator.tick(ActuatorBreakawayCalibrationInput {
                output_velocity: velocity,
                output_torque_command: command,
                dt_seconds: 0.01,
            });

            if calibrator.result().is_some() {
                let result = calibrator.result().unwrap();
                assert!((result.positive_breakaway_torque.get() - 0.09).abs() < 0.02);
                assert!((result.negative_breakaway_torque.get() - 0.11).abs() < 0.02);
                break;
            }

            if calibrator.error().is_some() {
                panic!("synthetic breakaway calibration unexpectedly failed");
            }
        }
    }

    #[test]
    fn times_out_when_motion_never_occurs() {
        let mut calibrator = ActuatorBreakawayCalibrator::new(
            ActuatorBreakawayCalibrationConfig::default_for_torque_ramp(),
        )
        .unwrap();

        for _ in 0..10_000 {
            let command = calibrator.commanded_torque_target();
            let _ = calibrator.tick(ActuatorBreakawayCalibrationInput {
                output_velocity: RadPerSec::ZERO,
                output_torque_command: command,
                dt_seconds: 0.001,
            });
            if calibrator.error().is_some() {
                break;
            }
        }

        assert!(matches!(
            calibrator.error(),
            Some(CalibrationError::Timeout | CalibrationError::IndeterminateEstimate)
        ));
    }
}
