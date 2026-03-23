//! Actuator gear-ratio calibration from simultaneous rotor/output travel.
//!
//! This procedure commands a steady positive output-axis velocity target. Once
//! the output velocity settles, it unwraps both the rotor and output encoder
//! angles over a short sample window and estimates:
//!
//! `gear_ratio ~= |rotor_travel / output_travel|`

use fluxkit_math::{MechanicalAngle, angle::shortest_angle_delta, units::RadPerSec};

use super::error::CalibrationError;

/// Static configuration for actuator gear-ratio calibration.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct ActuatorGearRatioCalibrationConfig {
    /// Output-axis velocity target used to create measurable travel.
    pub velocity_target: RadPerSec,
    /// Maximum absolute output-velocity error considered settled.
    pub settle_velocity_error: RadPerSec,
    /// Continuous settle time required before sampling travel.
    pub settle_time_seconds: f32,
    /// Travel-sampling window after settling.
    pub sample_time_seconds: f32,
    /// Minimum required output travel magnitude over the sample window.
    pub min_output_travel: MechanicalAngle,
    /// Absolute timeout for the whole procedure.
    pub timeout_seconds: f32,
}

impl ActuatorGearRatioCalibrationConfig {
    /// Returns a conservative default suitable for simulator-backed tests.
    pub const fn default_for_travel_ratio() -> Self {
        Self {
            velocity_target: RadPerSec::new(1.0),
            settle_velocity_error: RadPerSec::new(0.05),
            settle_time_seconds: 0.05,
            sample_time_seconds: 0.15,
            min_output_travel: MechanicalAngle::new(0.05),
            timeout_seconds: 4.0,
        }
    }
}

/// One synchronous sample frame for gear-ratio calibration.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct ActuatorGearRatioCalibrationInput {
    /// Wrapped rotor mechanical angle from the motor encoder.
    pub rotor_mechanical_angle: MechanicalAngle,
    /// Wrapped output-axis mechanical angle from the actuator encoder.
    pub output_mechanical_angle: MechanicalAngle,
    /// Measured output-axis mechanical velocity.
    pub output_velocity: RadPerSec,
    /// Time since the previous calibration tick.
    pub dt_seconds: f32,
}

/// Velocity-target command emitted by the gear-ratio calibrator.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct ActuatorGearRatioCalibrationCommand {
    /// Output-axis velocity target for the controller.
    pub velocity_target: RadPerSec,
}

/// Result of a completed actuator gear-ratio calibration.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct ActuatorGearRatioCalibrationResult {
    /// Mechanical reduction ratio from motor shaft to output axis.
    pub gear_ratio: f32,
}

/// Compact state of the actuator gear-ratio calibration procedure.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum ActuatorGearRatioCalibrationState {
    /// Waiting for the output velocity to settle near the commanded target.
    Settling,
    /// Sampling simultaneous rotor/output travel.
    Sampling,
    /// The procedure completed successfully.
    Complete,
    /// The procedure failed.
    Failed(CalibrationError),
}

/// Pure state machine for actuator gear-ratio calibration.
#[derive(Clone, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct ActuatorGearRatioCalibrator {
    config: ActuatorGearRatioCalibrationConfig,
    elapsed_seconds: f32,
    settled_seconds: f32,
    sample_seconds: f32,
    start_unwrapped_rotor_angle: Option<f32>,
    start_unwrapped_output_angle: Option<f32>,
    current_unwrapped_rotor_angle: f32,
    current_unwrapped_output_angle: f32,
    last_wrapped_rotor_angle: Option<MechanicalAngle>,
    last_wrapped_output_angle: Option<MechanicalAngle>,
    result: Option<ActuatorGearRatioCalibrationResult>,
    error: Option<CalibrationError>,
}

impl ActuatorGearRatioCalibrator {
    /// Creates a new actuator gear-ratio calibrator.
    pub fn new(config: ActuatorGearRatioCalibrationConfig) -> Result<Self, CalibrationError> {
        if !validate_config(config) {
            return Err(CalibrationError::InvalidConfiguration);
        }

        Ok(Self {
            config,
            elapsed_seconds: 0.0,
            settled_seconds: 0.0,
            sample_seconds: 0.0,
            start_unwrapped_rotor_angle: None,
            start_unwrapped_output_angle: None,
            current_unwrapped_rotor_angle: 0.0,
            current_unwrapped_output_angle: 0.0,
            last_wrapped_rotor_angle: None,
            last_wrapped_output_angle: None,
            result: None,
            error: None,
        })
    }

    /// Returns the current calibration state.
    #[inline]
    pub const fn state(&self) -> ActuatorGearRatioCalibrationState {
        if let Some(error) = self.error {
            ActuatorGearRatioCalibrationState::Failed(error)
        } else if self.result.is_some() {
            ActuatorGearRatioCalibrationState::Complete
        } else if self.sample_seconds > 0.0 {
            ActuatorGearRatioCalibrationState::Sampling
        } else {
            ActuatorGearRatioCalibrationState::Settling
        }
    }

    /// Returns the finished result when calibration has succeeded.
    #[inline]
    pub const fn result(&self) -> Option<ActuatorGearRatioCalibrationResult> {
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
            RadPerSec::ZERO
        } else {
            self.config.velocity_target
        }
    }

    /// Advances the calibration procedure by one sample.
    pub fn tick(
        &mut self,
        input: ActuatorGearRatioCalibrationInput,
    ) -> ActuatorGearRatioCalibrationCommand {
        if self.result.is_some() || self.error.is_some() {
            return ActuatorGearRatioCalibrationCommand {
                velocity_target: RadPerSec::ZERO,
            };
        }

        if !validate_input(input) {
            self.error = Some(CalibrationError::InvalidInput);
            return ActuatorGearRatioCalibrationCommand {
                velocity_target: RadPerSec::ZERO,
            };
        }

        self.elapsed_seconds += input.dt_seconds;
        if self.elapsed_seconds >= self.config.timeout_seconds {
            self.error = Some(CalibrationError::Timeout);
            return ActuatorGearRatioCalibrationCommand {
                velocity_target: RadPerSec::ZERO,
            };
        }

        self.update_unwrapped_angles(input.rotor_mechanical_angle, input.output_mechanical_angle);

        if (input.output_velocity.get() - self.config.velocity_target.get()).abs()
            > self.config.settle_velocity_error.get()
        {
            self.reset_sampling();
            return ActuatorGearRatioCalibrationCommand {
                velocity_target: self.config.velocity_target,
            };
        }

        if self.sample_seconds == 0.0 {
            self.settled_seconds += input.dt_seconds;
            if self.settled_seconds < self.config.settle_time_seconds {
                return ActuatorGearRatioCalibrationCommand {
                    velocity_target: self.config.velocity_target,
                };
            }

            self.start_unwrapped_rotor_angle = Some(self.current_unwrapped_rotor_angle);
            self.start_unwrapped_output_angle = Some(self.current_unwrapped_output_angle);
        }

        self.sample_seconds += input.dt_seconds;
        if self.sample_seconds < self.config.sample_time_seconds {
            return ActuatorGearRatioCalibrationCommand {
                velocity_target: self.config.velocity_target,
            };
        }

        let Some(start_rotor_angle) = self.start_unwrapped_rotor_angle else {
            self.error = Some(CalibrationError::IndeterminateEstimate);
            return ActuatorGearRatioCalibrationCommand {
                velocity_target: RadPerSec::ZERO,
            };
        };
        let Some(start_output_angle) = self.start_unwrapped_output_angle else {
            self.error = Some(CalibrationError::IndeterminateEstimate);
            return ActuatorGearRatioCalibrationCommand {
                velocity_target: RadPerSec::ZERO,
            };
        };

        let rotor_travel = self.current_unwrapped_rotor_angle - start_rotor_angle;
        let output_travel = self.current_unwrapped_output_angle - start_output_angle;
        if output_travel.abs() < self.config.min_output_travel.get() || rotor_travel.abs() < 1.0e-6
        {
            self.error = Some(CalibrationError::IndeterminateEstimate);
            return ActuatorGearRatioCalibrationCommand {
                velocity_target: RadPerSec::ZERO,
            };
        }

        let gear_ratio = (rotor_travel / output_travel).abs();
        if !gear_ratio.is_finite() || gear_ratio <= 0.0 {
            self.error = Some(CalibrationError::IndeterminateEstimate);
            return ActuatorGearRatioCalibrationCommand {
                velocity_target: RadPerSec::ZERO,
            };
        }

        self.result = Some(ActuatorGearRatioCalibrationResult { gear_ratio });
        ActuatorGearRatioCalibrationCommand {
            velocity_target: RadPerSec::ZERO,
        }
    }

    fn update_unwrapped_angles(
        &mut self,
        rotor_wrapped_angle: MechanicalAngle,
        output_wrapped_angle: MechanicalAngle,
    ) {
        match self.last_wrapped_rotor_angle {
            Some(last) => {
                self.current_unwrapped_rotor_angle +=
                    shortest_angle_delta(last.get(), rotor_wrapped_angle.get());
                self.last_wrapped_rotor_angle = Some(rotor_wrapped_angle);
            }
            None => {
                self.current_unwrapped_rotor_angle = rotor_wrapped_angle.get();
                self.last_wrapped_rotor_angle = Some(rotor_wrapped_angle);
            }
        }

        match self.last_wrapped_output_angle {
            Some(last) => {
                self.current_unwrapped_output_angle +=
                    shortest_angle_delta(last.get(), output_wrapped_angle.get());
                self.last_wrapped_output_angle = Some(output_wrapped_angle);
            }
            None => {
                self.current_unwrapped_output_angle = output_wrapped_angle.get();
                self.last_wrapped_output_angle = Some(output_wrapped_angle);
            }
        }
    }

    fn reset_sampling(&mut self) {
        self.settled_seconds = 0.0;
        self.sample_seconds = 0.0;
        self.start_unwrapped_rotor_angle = None;
        self.start_unwrapped_output_angle = None;
    }
}

fn validate_config(config: ActuatorGearRatioCalibrationConfig) -> bool {
    config.velocity_target.get().is_finite()
        && config.velocity_target.get() > 0.0
        && config.settle_velocity_error.get().is_finite()
        && config.settle_velocity_error.get() > 0.0
        && config.settle_time_seconds.is_finite()
        && config.settle_time_seconds > 0.0
        && config.sample_time_seconds.is_finite()
        && config.sample_time_seconds > 0.0
        && config.min_output_travel.get().is_finite()
        && config.min_output_travel.get() > 0.0
        && config.timeout_seconds.is_finite()
        && config.timeout_seconds > 0.0
}

fn validate_input(input: ActuatorGearRatioCalibrationInput) -> bool {
    input.rotor_mechanical_angle.get().is_finite()
        && input.output_mechanical_angle.get().is_finite()
        && input.output_velocity.get().is_finite()
        && input.dt_seconds.is_finite()
        && input.dt_seconds > 0.0
}

#[cfg(test)]
mod tests {
    use fluxkit_math::{MechanicalAngle, angle::wrap_0_2pi, units::RadPerSec};

    use super::{
        ActuatorGearRatioCalibrationConfig, ActuatorGearRatioCalibrationInput,
        ActuatorGearRatioCalibrationState, ActuatorGearRatioCalibrator,
    };
    use crate::CalibrationError;

    #[test]
    fn synthetic_travel_recovers_gear_ratio() {
        let mut calibrator = ActuatorGearRatioCalibrator::new(ActuatorGearRatioCalibrationConfig {
            velocity_target: RadPerSec::new(1.0),
            settle_velocity_error: RadPerSec::new(0.01),
            settle_time_seconds: 0.01,
            sample_time_seconds: 0.05,
            min_output_travel: MechanicalAngle::new(0.02),
            timeout_seconds: 1.0,
        })
        .unwrap();

        let dt = 0.001;
        let mut output_angle = 0.0;
        let mut rotor_angle = 0.0;

        for _ in 0..200 {
            output_angle += 1.0 * dt;
            rotor_angle += 3.0 * dt;

            let _ = calibrator.tick(ActuatorGearRatioCalibrationInput {
                rotor_mechanical_angle: MechanicalAngle::new(wrap_0_2pi(rotor_angle)),
                output_mechanical_angle: MechanicalAngle::new(wrap_0_2pi(output_angle)),
                output_velocity: RadPerSec::new(1.0),
                dt_seconds: dt,
            });

            if calibrator.state() == ActuatorGearRatioCalibrationState::Complete {
                break;
            }
        }

        assert_eq!(
            calibrator.state(),
            ActuatorGearRatioCalibrationState::Complete
        );
        let result = calibrator.result().unwrap();
        assert!((result.gear_ratio - 3.0).abs() < 1.0e-3);
    }

    #[test]
    fn times_out_when_output_velocity_never_settles() {
        let mut calibrator = ActuatorGearRatioCalibrator::new(ActuatorGearRatioCalibrationConfig {
            timeout_seconds: 0.01,
            ..ActuatorGearRatioCalibrationConfig::default_for_travel_ratio()
        })
        .unwrap();

        for _ in 0..100 {
            let _ = calibrator.tick(ActuatorGearRatioCalibrationInput {
                rotor_mechanical_angle: MechanicalAngle::new(0.0),
                output_mechanical_angle: MechanicalAngle::new(0.0),
                output_velocity: RadPerSec::ZERO,
                dt_seconds: 0.001,
            });

            if matches!(
                calibrator.state(),
                ActuatorGearRatioCalibrationState::Failed(CalibrationError::Timeout)
            ) {
                break;
            }
        }

        assert_eq!(
            calibrator.state(),
            ActuatorGearRatioCalibrationState::Failed(CalibrationError::Timeout)
        );
    }
}
