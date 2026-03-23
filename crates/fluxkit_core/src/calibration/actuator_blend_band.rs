//! Zero-velocity blend-band calibration from low-speed release ramps.
//!
//! This procedure ramps output torque slowly in each direction, then measures
//! the output velocity during the first sustained release from stiction.
//! The resulting transition speed is used as a practical estimate of the
//! friction model's zero-velocity blend band.

use fluxkit_math::units::{NewtonMeters, RadPerSec};

use super::error::CalibrationError;

/// Static configuration for zero-velocity blend-band calibration.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct ActuatorBlendBandCalibrationConfig {
    /// Ramp rate applied in both directions, in `Nm/s`.
    pub torque_ramp_rate_nm_per_sec: f32,
    /// Maximum absolute output torque command used during the search.
    pub max_torque: NewtonMeters,
    /// Sustained output-speed threshold that counts as release.
    pub motion_velocity_threshold: RadPerSec,
    /// Continuous motion time required to declare release.
    pub motion_confirm_time_seconds: f32,
    /// Near-zero speed threshold required before reversing direction.
    pub rest_velocity_threshold: RadPerSec,
    /// Continuous rest time required before reversing direction.
    pub rest_time_seconds: f32,
    /// Absolute timeout for the whole procedure.
    pub timeout_seconds: f32,
}

impl ActuatorBlendBandCalibrationConfig {
    /// Returns a conservative default suitable for simulator-backed tests.
    pub const fn default_for_release_ramp() -> Self {
        Self {
            torque_ramp_rate_nm_per_sec: 1.0,
            max_torque: NewtonMeters::new(0.3),
            motion_velocity_threshold: RadPerSec::new(0.05),
            motion_confirm_time_seconds: 0.01,
            rest_velocity_threshold: RadPerSec::new(0.02),
            rest_time_seconds: 0.05,
            timeout_seconds: 4.0,
        }
    }
}

/// One synchronous sample frame for zero-velocity blend-band calibration.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct ActuatorBlendBandCalibrationInput {
    /// Measured output-axis velocity.
    pub output_velocity: RadPerSec,
    /// Actual bounded output torque command applied by the controller.
    pub output_torque_command: NewtonMeters,
    /// Time since the previous calibration tick.
    pub dt_seconds: f32,
}

/// Torque-target command emitted by the blend-band calibrator.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct ActuatorBlendBandCalibrationCommand {
    /// Output-axis torque target for the controller.
    pub torque_target: NewtonMeters,
}

/// Result of a completed zero-velocity blend-band calibration.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct ActuatorBlendBandCalibrationResult {
    /// Smoothing band around zero output velocity for friction blending.
    pub zero_velocity_blend_band: RadPerSec,
}

/// Compact state of the zero-velocity blend-band calibration procedure.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum ActuatorBlendBandCalibrationState {
    /// The calibrator is ramping positive torque from zero.
    PositiveRamp,
    /// The calibrator is waiting at zero torque for the actuator to stop.
    NeutralSettle,
    /// The calibrator is ramping negative torque from zero.
    NegativeRamp,
    /// The procedure completed successfully.
    Complete,
    /// The procedure failed.
    Failed(CalibrationError),
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum BlendBandPhase {
    PositiveRamp,
    NeutralSettle,
    NegativeRamp,
}

/// Pure state machine for zero-velocity blend-band calibration.
#[derive(Clone, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct ActuatorBlendBandCalibrator {
    config: ActuatorBlendBandCalibrationConfig,
    elapsed_seconds: f32,
    phase: BlendBandPhase,
    ramp_seconds: f32,
    motion_seconds: f32,
    release_velocity_candidate: Option<RadPerSec>,
    rest_seconds: f32,
    positive_band: Option<RadPerSec>,
    result: Option<ActuatorBlendBandCalibrationResult>,
    error: Option<CalibrationError>,
}

impl ActuatorBlendBandCalibrator {
    /// Creates a new zero-velocity blend-band calibrator.
    pub fn new(config: ActuatorBlendBandCalibrationConfig) -> Result<Self, CalibrationError> {
        if !validate_config(config) {
            return Err(CalibrationError::InvalidConfiguration);
        }

        Ok(Self {
            config,
            elapsed_seconds: 0.0,
            phase: BlendBandPhase::PositiveRamp,
            ramp_seconds: 0.0,
            motion_seconds: 0.0,
            release_velocity_candidate: None,
            rest_seconds: 0.0,
            positive_band: None,
            result: None,
            error: None,
        })
    }

    /// Returns the current calibration state.
    #[inline]
    pub const fn state(&self) -> ActuatorBlendBandCalibrationState {
        if let Some(error) = self.error {
            ActuatorBlendBandCalibrationState::Failed(error)
        } else if self.result.is_some() {
            ActuatorBlendBandCalibrationState::Complete
        } else {
            match self.phase {
                BlendBandPhase::PositiveRamp => ActuatorBlendBandCalibrationState::PositiveRamp,
                BlendBandPhase::NeutralSettle => ActuatorBlendBandCalibrationState::NeutralSettle,
                BlendBandPhase::NegativeRamp => ActuatorBlendBandCalibrationState::NegativeRamp,
            }
        }
    }

    /// Returns the finished result when calibration has succeeded.
    #[inline]
    pub const fn result(&self) -> Option<ActuatorBlendBandCalibrationResult> {
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

        match self.phase {
            BlendBandPhase::PositiveRamp => NewtonMeters::new(self.current_ramp_torque()),
            BlendBandPhase::NeutralSettle => NewtonMeters::ZERO,
            BlendBandPhase::NegativeRamp => NewtonMeters::new(-self.current_ramp_torque()),
        }
    }

    /// Advances the calibration procedure by one sample.
    pub fn tick(
        &mut self,
        input: ActuatorBlendBandCalibrationInput,
    ) -> ActuatorBlendBandCalibrationCommand {
        if self.result.is_some() || self.error.is_some() {
            return ActuatorBlendBandCalibrationCommand {
                torque_target: NewtonMeters::ZERO,
            };
        }

        if !validate_input(input) {
            self.error = Some(CalibrationError::InvalidInput);
            return ActuatorBlendBandCalibrationCommand {
                torque_target: NewtonMeters::ZERO,
            };
        }

        self.elapsed_seconds += input.dt_seconds;
        if self.elapsed_seconds >= self.config.timeout_seconds {
            self.error = Some(CalibrationError::Timeout);
            return ActuatorBlendBandCalibrationCommand {
                torque_target: NewtonMeters::ZERO,
            };
        }

        match self.phase {
            BlendBandPhase::PositiveRamp => self.tick_positive_ramp(input),
            BlendBandPhase::NeutralSettle => self.tick_neutral_settle(input),
            BlendBandPhase::NegativeRamp => self.tick_negative_ramp(input),
        }

        ActuatorBlendBandCalibrationCommand {
            torque_target: self.commanded_torque_target(),
        }
    }

    fn tick_positive_ramp(&mut self, input: ActuatorBlendBandCalibrationInput) {
        self.ramp_seconds += input.dt_seconds;
        if input.output_velocity.get() >= self.config.motion_velocity_threshold.get() {
            if self.motion_seconds <= 0.0 {
                self.release_velocity_candidate = Some(input.output_velocity);
            }
            self.motion_seconds += input.dt_seconds;
        } else {
            self.reset_motion_window();
        }

        if self.motion_seconds >= self.config.motion_confirm_time_seconds {
            let band = self
                .release_velocity_candidate
                .unwrap_or(self.config.motion_velocity_threshold);
            self.positive_band = Some(band);
            self.phase = BlendBandPhase::NeutralSettle;
            self.ramp_seconds = 0.0;
            self.rest_seconds = 0.0;
            self.reset_motion_window();
            return;
        }

        if self.current_ramp_torque() >= self.config.max_torque.get() {
            self.error = Some(CalibrationError::IndeterminateEstimate);
        }
    }

    fn tick_neutral_settle(&mut self, input: ActuatorBlendBandCalibrationInput) {
        if input.output_velocity.get().abs() <= self.config.rest_velocity_threshold.get() {
            self.rest_seconds += input.dt_seconds;
        } else {
            self.rest_seconds = 0.0;
        }

        if self.rest_seconds >= self.config.rest_time_seconds {
            self.phase = BlendBandPhase::NegativeRamp;
            self.ramp_seconds = 0.0;
            self.rest_seconds = 0.0;
            self.reset_motion_window();
        }
    }

    fn tick_negative_ramp(&mut self, input: ActuatorBlendBandCalibrationInput) {
        self.ramp_seconds += input.dt_seconds;
        if input.output_velocity.get() <= -self.config.motion_velocity_threshold.get() {
            if self.motion_seconds <= 0.0 {
                self.release_velocity_candidate =
                    Some(RadPerSec::new(input.output_velocity.get().abs()));
            }
            self.motion_seconds += input.dt_seconds;
        } else {
            self.reset_motion_window();
        }

        if self.motion_seconds >= self.config.motion_confirm_time_seconds {
            let negative_band = self
                .release_velocity_candidate
                .unwrap_or(self.config.motion_velocity_threshold);
            let positive_band = self
                .positive_band
                .unwrap_or(self.config.motion_velocity_threshold);
            self.result = Some(ActuatorBlendBandCalibrationResult {
                zero_velocity_blend_band: RadPerSec::new(
                    0.5 * (positive_band.get() + negative_band.get()),
                ),
            });
            return;
        }

        if self.current_ramp_torque() >= self.config.max_torque.get() {
            self.error = Some(CalibrationError::IndeterminateEstimate);
        }
    }

    #[inline]
    fn current_ramp_torque(&self) -> f32 {
        (self.ramp_seconds * self.config.torque_ramp_rate_nm_per_sec)
            .min(self.config.max_torque.get())
    }

    #[inline]
    fn reset_motion_window(&mut self) {
        self.motion_seconds = 0.0;
        self.release_velocity_candidate = None;
    }
}

fn validate_config(config: ActuatorBlendBandCalibrationConfig) -> bool {
    config.torque_ramp_rate_nm_per_sec.is_finite()
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

fn validate_input(input: ActuatorBlendBandCalibrationInput) -> bool {
    input.output_velocity.get().is_finite()
        && input.output_torque_command.get().is_finite()
        && input.dt_seconds.is_finite()
        && input.dt_seconds > 0.0
}

#[cfg(test)]
mod tests {
    use super::{
        ActuatorBlendBandCalibrationConfig, ActuatorBlendBandCalibrationInput,
        ActuatorBlendBandCalibrationState, ActuatorBlendBandCalibrator,
    };
    use fluxkit_math::units::RadPerSec;

    #[test]
    fn synthetic_release_ramp_recovers_blend_band() {
        let mut calibrator = ActuatorBlendBandCalibrator::new(
            ActuatorBlendBandCalibrationConfig::default_for_release_ramp(),
        )
        .unwrap();

        loop {
            let command = calibrator.commanded_torque_target();
            let velocity = if command.get() >= 0.13 {
                RadPerSec::new(0.052)
            } else if command.get() <= -0.16 {
                RadPerSec::new(-0.052)
            } else {
                RadPerSec::ZERO
            };
            let _ = calibrator.tick(ActuatorBlendBandCalibrationInput {
                output_velocity: velocity,
                output_torque_command: command,
                dt_seconds: 0.01,
            });

            if calibrator.state() == ActuatorBlendBandCalibrationState::Complete {
                let result = calibrator.result().unwrap();
                assert!((result.zero_velocity_blend_band.get() - 0.05).abs() < 0.005);
                break;
            }

            if calibrator.error().is_some() {
                panic!("synthetic blend-band calibration unexpectedly failed");
            }
        }
    }
}
