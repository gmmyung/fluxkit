//! Actuator-friction calibration from steady velocity sweeps.
//!
//! This procedure commands a sequence of positive and negative output-axis
//! velocity targets. Once the actuator settles around each target, it averages
//! the commanded output torque required to sustain that motion and fits
//! separate Coulomb-plus-viscous models for forward and reverse travel:
//!
//! `tau ~= tau_coulomb + b * omega`
//!
//! Breakaway terms are intentionally left for a later low-speed refinement
//! procedure.

use fluxkit_math::units::{NewtonMeters, RadPerSec};

use super::error::CalibrationError;

const NUM_VELOCITY_POINTS: usize = 3;

/// Static configuration for steady-state actuator-friction calibration.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct ActuatorFrictionCalibrationConfig {
    /// Positive output-speed magnitudes to visit in both directions.
    pub velocity_points: [RadPerSec; NUM_VELOCITY_POINTS],
    /// Maximum absolute output-speed error considered settled.
    pub settle_velocity_error: RadPerSec,
    /// Continuous settle time required before sampling torque.
    pub settle_time_seconds: f32,
    /// Averaging window for each steady-state torque sample.
    pub sample_time_seconds: f32,
    /// Absolute timeout for the whole procedure.
    pub timeout_seconds: f32,
}

impl ActuatorFrictionCalibrationConfig {
    /// Returns a conservative default suitable for simulator-backed tests.
    pub const fn default_for_velocity_sweep() -> Self {
        Self {
            velocity_points: [
                RadPerSec::new(0.5),
                RadPerSec::new(1.0),
                RadPerSec::new(1.5),
            ],
            settle_velocity_error: RadPerSec::new(0.01),
            settle_time_seconds: 0.1,
            sample_time_seconds: 0.1,
            timeout_seconds: 8.0,
        }
    }
}

/// One synchronous sample frame for actuator-friction calibration.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct ActuatorFrictionCalibrationInput {
    /// Measured output-axis velocity.
    pub output_velocity: RadPerSec,
    /// Commanded output torque currently required to sustain motion.
    pub output_torque_command: NewtonMeters,
    /// Time since the previous calibration tick.
    pub dt_seconds: f32,
}

/// Velocity-target command emitted by the friction calibrator.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct ActuatorFrictionCalibrationCommand {
    /// Output-axis velocity target for the controller.
    pub velocity_target: RadPerSec,
}

/// Result of a completed actuator-friction calibration.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct ActuatorFrictionCalibrationResult {
    /// Constant friction torque while moving in the positive direction.
    pub positive_coulomb_torque: NewtonMeters,
    /// Constant friction torque while moving in the negative direction.
    pub negative_coulomb_torque: NewtonMeters,
    /// Positive-direction viscous coefficient in `Nm / (rad/s)`.
    pub positive_viscous_coefficient: f32,
    /// Negative-direction viscous coefficient in `Nm / (rad/s)`.
    pub negative_viscous_coefficient: f32,
}

/// Compact state of the actuator-friction calibration procedure.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum ActuatorFrictionCalibrationState {
    /// The calibrator is waiting for steady-state motion at the current target.
    Settling,
    /// The calibrator is averaging the steady-state torque sample.
    Sampling,
    /// The procedure completed successfully.
    Complete,
    /// The procedure failed.
    Failed(CalibrationError),
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum SweepDirection {
    Positive,
    Negative,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct SweepPhase {
    direction: SweepDirection,
    index: usize,
}

#[derive(Clone, Copy, Debug, PartialEq)]
struct FrictionSample {
    velocity: f32,
    torque: f32,
}

/// Pure state machine for actuator-friction calibration.
#[derive(Clone, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct ActuatorFrictionCalibrator {
    config: ActuatorFrictionCalibrationConfig,
    elapsed_seconds: f32,
    phase: SweepPhase,
    settled_seconds: f32,
    sample_seconds: f32,
    sample_count: u32,
    velocity_sum: f32,
    torque_sum: f32,
    positive_samples: [Option<FrictionSample>; NUM_VELOCITY_POINTS],
    negative_samples: [Option<FrictionSample>; NUM_VELOCITY_POINTS],
    result: Option<ActuatorFrictionCalibrationResult>,
    error: Option<CalibrationError>,
}

impl ActuatorFrictionCalibrator {
    /// Creates a new actuator-friction calibrator.
    pub fn new(config: ActuatorFrictionCalibrationConfig) -> Result<Self, CalibrationError> {
        if !validate_config(config) {
            return Err(CalibrationError::InvalidConfiguration);
        }

        Ok(Self {
            config,
            elapsed_seconds: 0.0,
            phase: SweepPhase {
                direction: SweepDirection::Positive,
                index: 0,
            },
            settled_seconds: 0.0,
            sample_seconds: 0.0,
            sample_count: 0,
            velocity_sum: 0.0,
            torque_sum: 0.0,
            positive_samples: [None; NUM_VELOCITY_POINTS],
            negative_samples: [None; NUM_VELOCITY_POINTS],
            result: None,
            error: None,
        })
    }

    /// Returns the current calibration state.
    #[inline]
    pub const fn state(&self) -> ActuatorFrictionCalibrationState {
        if let Some(error) = self.error {
            ActuatorFrictionCalibrationState::Failed(error)
        } else if self.result.is_some() {
            ActuatorFrictionCalibrationState::Complete
        } else if self.sample_seconds > 0.0 {
            ActuatorFrictionCalibrationState::Sampling
        } else {
            ActuatorFrictionCalibrationState::Settling
        }
    }

    /// Returns the finished result when calibration has succeeded.
    #[inline]
    pub const fn result(&self) -> Option<ActuatorFrictionCalibrationResult> {
        self.result
    }

    /// Returns the failure cause when calibration has failed.
    #[inline]
    pub const fn error(&self) -> Option<CalibrationError> {
        self.error
    }

    /// Returns the currently commanded output-axis velocity target.
    #[inline]
    pub fn commanded_velocity_target(&self) -> RadPerSec {
        if self.result.is_some() || self.error.is_some() {
            return RadPerSec::ZERO;
        }

        let magnitude = self.config.velocity_points[self.phase.index].get();
        let sign = match self.phase.direction {
            SweepDirection::Positive => 1.0,
            SweepDirection::Negative => -1.0,
        };
        RadPerSec::new(sign * magnitude)
    }

    /// Advances the calibration procedure by one sample.
    pub fn tick(
        &mut self,
        input: ActuatorFrictionCalibrationInput,
    ) -> ActuatorFrictionCalibrationCommand {
        if self.result.is_some() || self.error.is_some() {
            return ActuatorFrictionCalibrationCommand {
                velocity_target: RadPerSec::ZERO,
            };
        }

        if !validate_input(input) {
            self.error = Some(CalibrationError::InvalidInput);
            return ActuatorFrictionCalibrationCommand {
                velocity_target: RadPerSec::ZERO,
            };
        }

        self.elapsed_seconds += input.dt_seconds;
        if self.elapsed_seconds >= self.config.timeout_seconds {
            self.error = Some(CalibrationError::Timeout);
            return ActuatorFrictionCalibrationCommand {
                velocity_target: RadPerSec::ZERO,
            };
        }

        let target_velocity = self.commanded_velocity_target();
        if (input.output_velocity.get() - target_velocity.get()).abs()
            > self.config.settle_velocity_error.get()
        {
            self.reset_phase_accumulators();
            return ActuatorFrictionCalibrationCommand {
                velocity_target: target_velocity,
            };
        }

        if self.sample_seconds == 0.0 {
            self.settled_seconds += input.dt_seconds;
            if self.settled_seconds < self.config.settle_time_seconds {
                return ActuatorFrictionCalibrationCommand {
                    velocity_target: target_velocity,
                };
            }
        }

        let sign = match self.phase.direction {
            SweepDirection::Positive => 1.0,
            SweepDirection::Negative => -1.0,
        };
        let signed_velocity = sign * input.output_velocity.get();
        let signed_torque = sign * input.output_torque_command.get();
        if !signed_velocity.is_finite() || !signed_torque.is_finite() {
            self.error = Some(CalibrationError::InvalidInput);
            return ActuatorFrictionCalibrationCommand {
                velocity_target: RadPerSec::ZERO,
            };
        }

        self.sample_seconds += input.dt_seconds;
        self.sample_count += 1;
        self.velocity_sum += signed_velocity;
        self.torque_sum += signed_torque;

        if self.sample_seconds >= self.config.sample_time_seconds {
            let mean_velocity = self.velocity_sum / self.sample_count as f32;
            let mean_torque = self.torque_sum / self.sample_count as f32;
            if !mean_velocity.is_finite()
                || !mean_torque.is_finite()
                || mean_velocity <= 0.0
                || mean_torque < 0.0
            {
                self.error = Some(CalibrationError::IndeterminateEstimate);
                return ActuatorFrictionCalibrationCommand {
                    velocity_target: RadPerSec::ZERO,
                };
            }

            let sample = FrictionSample {
                velocity: mean_velocity,
                torque: mean_torque,
            };
            match self.phase.direction {
                SweepDirection::Positive => self.positive_samples[self.phase.index] = Some(sample),
                SweepDirection::Negative => self.negative_samples[self.phase.index] = Some(sample),
            }

            self.reset_phase_accumulators();
            if !self.advance_phase() {
                self.finish_fit();
                return ActuatorFrictionCalibrationCommand {
                    velocity_target: RadPerSec::ZERO,
                };
            }
        }

        ActuatorFrictionCalibrationCommand {
            velocity_target: self.commanded_velocity_target(),
        }
    }

    fn reset_phase_accumulators(&mut self) {
        self.settled_seconds = 0.0;
        self.sample_seconds = 0.0;
        self.sample_count = 0;
        self.velocity_sum = 0.0;
        self.torque_sum = 0.0;
    }

    fn advance_phase(&mut self) -> bool {
        match self.phase.direction {
            SweepDirection::Positive if self.phase.index + 1 < NUM_VELOCITY_POINTS => {
                self.phase.index += 1;
                true
            }
            SweepDirection::Positive => {
                self.phase = SweepPhase {
                    direction: SweepDirection::Negative,
                    index: 0,
                };
                true
            }
            SweepDirection::Negative if self.phase.index + 1 < NUM_VELOCITY_POINTS => {
                self.phase.index += 1;
                true
            }
            SweepDirection::Negative => false,
        }
    }

    fn finish_fit(&mut self) {
        let positive = match fit_friction(self.positive_samples) {
            Some(value) => value,
            None => {
                self.error = Some(CalibrationError::IndeterminateEstimate);
                return;
            }
        };
        let negative = match fit_friction(self.negative_samples) {
            Some(value) => value,
            None => {
                self.error = Some(CalibrationError::IndeterminateEstimate);
                return;
            }
        };

        self.result = Some(ActuatorFrictionCalibrationResult {
            positive_coulomb_torque: NewtonMeters::new(positive.0),
            negative_coulomb_torque: NewtonMeters::new(negative.0),
            positive_viscous_coefficient: positive.1,
            negative_viscous_coefficient: negative.1,
        });
    }
}

fn fit_friction(samples: [Option<FrictionSample>; NUM_VELOCITY_POINTS]) -> Option<(f32, f32)> {
    let [a, b, c] = samples;
    let samples = [a?, b?, c?];
    let n = NUM_VELOCITY_POINTS as f32;
    let mut sum_x = 0.0;
    let mut sum_y = 0.0;
    let mut sum_xx = 0.0;
    let mut sum_xy = 0.0;

    for sample in samples {
        sum_x += sample.velocity;
        sum_y += sample.torque;
        sum_xx += sample.velocity * sample.velocity;
        sum_xy += sample.velocity * sample.torque;
    }

    let denom = n * sum_xx - sum_x * sum_x;
    if !denom.is_finite() || denom.abs() <= 1.0e-9 {
        return None;
    }

    let slope = (n * sum_xy - sum_x * sum_y) / denom;
    let intercept = (sum_y - slope * sum_x) / n;
    if !slope.is_finite() || !intercept.is_finite() || slope < 0.0 || intercept < 0.0 {
        return None;
    }

    Some((intercept, slope))
}

fn validate_config(config: ActuatorFrictionCalibrationConfig) -> bool {
    let mut previous = 0.0;
    for velocity in config.velocity_points {
        let value = velocity.get();
        if !value.is_finite() || value <= previous {
            return false;
        }
        previous = value;
    }

    config.settle_velocity_error.get().is_finite()
        && config.settle_velocity_error.get() > 0.0
        && config.settle_time_seconds.is_finite()
        && config.settle_time_seconds > 0.0
        && config.sample_time_seconds.is_finite()
        && config.sample_time_seconds > 0.0
        && config.timeout_seconds.is_finite()
        && config.timeout_seconds
            > 2.0
                * NUM_VELOCITY_POINTS as f32
                * (config.settle_time_seconds + config.sample_time_seconds)
}

fn validate_input(input: ActuatorFrictionCalibrationInput) -> bool {
    input.output_velocity.get().is_finite()
        && input.output_torque_command.get().is_finite()
        && input.dt_seconds.is_finite()
        && input.dt_seconds > 0.0
}

#[cfg(test)]
mod tests {
    use super::{
        ActuatorFrictionCalibrationConfig, ActuatorFrictionCalibrationInput,
        ActuatorFrictionCalibrationState, ActuatorFrictionCalibrator,
    };
    use crate::CalibrationError;
    use fluxkit_math::units::{NewtonMeters, RadPerSec};

    #[test]
    fn synthetic_velocity_sweep_recovers_friction_fit() {
        let mut calibrator = ActuatorFrictionCalibrator::new(
            ActuatorFrictionCalibrationConfig::default_for_velocity_sweep(),
        )
        .unwrap();

        loop {
            let command = calibrator.tick(ActuatorFrictionCalibrationInput {
                output_velocity: command_velocity_for_state(calibrator.commanded_velocity_target()),
                output_torque_command: torque_for_velocity(calibrator.commanded_velocity_target()),
                dt_seconds: 0.01,
            });

            if calibrator.state() == ActuatorFrictionCalibrationState::Complete {
                let result = calibrator.result().unwrap();
                assert!((result.positive_coulomb_torque.get() - 0.04).abs() < 1.0e-4);
                assert!((result.negative_coulomb_torque.get() - 0.05).abs() < 1.0e-4);
                assert!((result.positive_viscous_coefficient - 0.02).abs() < 1.0e-4);
                assert!((result.negative_viscous_coefficient - 0.03).abs() < 1.0e-4);
                break;
            }

            if calibrator.error().is_some() {
                panic!("synthetic friction calibration unexpectedly failed");
            }

            let _ = command;
        }
    }

    #[test]
    fn times_out_when_velocity_never_settles() {
        let mut calibrator = ActuatorFrictionCalibrator::new(
            ActuatorFrictionCalibrationConfig::default_for_velocity_sweep(),
        )
        .unwrap();

        for _ in 0..1_000 {
            let _ = calibrator.tick(ActuatorFrictionCalibrationInput {
                output_velocity: RadPerSec::new(0.0),
                output_torque_command: NewtonMeters::ZERO,
                dt_seconds: 0.01,
            });
            if calibrator.error().is_some() {
                break;
            }
        }

        assert!(matches!(
            calibrator.state(),
            ActuatorFrictionCalibrationState::Failed(CalibrationError::Timeout)
        ));
    }

    fn command_velocity_for_state(target: RadPerSec) -> RadPerSec {
        target
    }

    fn torque_for_velocity(target: RadPerSec) -> NewtonMeters {
        let sign_positive = target.get() >= 0.0;
        let speed = target.get().abs();
        if sign_positive {
            NewtonMeters::new(0.04 + 0.02 * speed)
        } else {
            NewtonMeters::new(-(0.05 + 0.03 * speed))
        }
    }
}
