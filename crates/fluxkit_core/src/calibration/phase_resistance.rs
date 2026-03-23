//! Phase-resistance calibration via magnetic hold and steady current.
//!
//! The procedure applies a fixed stator-frame voltage vector, waits for the
//! rotor to settle against that field, then averages the projected steady-state
//! current along the commanded vector. Under near-zero mechanical speed,
//! `R ~= V / I` for that stationary excitation.

use fluxkit_math::{
    AlphaBeta, ElectricalAngle,
    frame::Abc,
    transforms::clarke,
    trig::sin_cos,
    units::{Amps, Ohms, RadPerSec, Volts},
};

use super::error::CalibrationError;

/// Static configuration for phase-resistance calibration.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct PhaseResistanceCalibrationConfig {
    /// Magnitude of the fixed stator-frame voltage vector.
    pub align_voltage_mag: Volts,
    /// Stator-frame angle of the excitation vector.
    pub align_stator_angle: ElectricalAngle,
    /// Maximum mechanical speed considered "settled".
    pub settle_velocity_threshold: RadPerSec,
    /// Continuous settle time required before sampling current.
    pub settle_time_seconds: f32,
    /// Averaging window for the projected steady-state current.
    pub sample_time_seconds: f32,
    /// Minimum usable projected current magnitude.
    pub min_projected_current: Amps,
    /// Absolute timeout for the whole procedure.
    pub timeout_seconds: f32,
}

impl PhaseResistanceCalibrationConfig {
    /// Returns a conservative default suitable for host-side bring-up and
    /// simulator-backed tests.
    pub const fn default_for_hold() -> Self {
        Self {
            align_voltage_mag: Volts::new(1.0),
            align_stator_angle: ElectricalAngle::new(0.0),
            settle_velocity_threshold: RadPerSec::new(0.05),
            settle_time_seconds: 0.05,
            sample_time_seconds: 0.05,
            min_projected_current: Amps::new(0.1),
            timeout_seconds: 2.0,
        }
    }
}

/// One synchronous sample frame for phase-resistance calibration.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct PhaseResistanceCalibrationInput {
    /// Measured three-phase currents.
    pub phase_currents: Abc<Amps>,
    /// Mechanical rotor velocity reported by the encoder path.
    pub mechanical_velocity: RadPerSec,
    /// Time since the previous calibration tick.
    pub dt_seconds: f32,
}

/// Result of a completed phase-resistance calibration.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct PhaseResistanceCalibrationResult {
    /// Calibrated phase resistance to store in `MotorParams`.
    pub phase_resistance_ohm: Ohms,
}

/// Compact state of the phase-resistance calibration procedure.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum PhaseResistanceCalibrationState {
    /// The calibrator is holding the field and waiting for settle.
    Aligning,
    /// The calibrator is averaging steady-state current.
    Sampling,
    /// The procedure completed successfully.
    Complete,
    /// The procedure failed.
    Failed(CalibrationError),
}

/// Pure state machine for phase-resistance calibration.
#[derive(Clone, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct PhaseResistanceCalibrator {
    config: PhaseResistanceCalibrationConfig,
    elapsed_seconds: f32,
    settled_seconds: f32,
    sample_seconds: f32,
    projected_current_integral: f32,
    result: Option<PhaseResistanceCalibrationResult>,
    error: Option<CalibrationError>,
}

impl PhaseResistanceCalibrator {
    /// Creates a new phase-resistance calibrator.
    pub fn new(config: PhaseResistanceCalibrationConfig) -> Result<Self, CalibrationError> {
        if !validate_config(config) {
            return Err(CalibrationError::InvalidConfiguration);
        }

        Ok(Self {
            config,
            elapsed_seconds: 0.0,
            settled_seconds: 0.0,
            sample_seconds: 0.0,
            projected_current_integral: 0.0,
            result: None,
            error: None,
        })
    }

    /// Returns the current calibration state.
    #[inline]
    pub const fn state(&self) -> PhaseResistanceCalibrationState {
        if let Some(error) = self.error {
            PhaseResistanceCalibrationState::Failed(error)
        } else if self.result.is_some() {
            PhaseResistanceCalibrationState::Complete
        } else if self.sample_seconds > 0.0 {
            PhaseResistanceCalibrationState::Sampling
        } else {
            PhaseResistanceCalibrationState::Aligning
        }
    }

    /// Returns the finished result when calibration has succeeded.
    #[inline]
    pub const fn result(&self) -> Option<PhaseResistanceCalibrationResult> {
        self.result
    }

    /// Returns the failure cause when calibration has failed.
    #[inline]
    pub const fn error(&self) -> Option<CalibrationError> {
        self.error
    }

    /// Returns the fixed `alpha/beta` excitation vector requested by this procedure.
    #[inline]
    pub fn commanded_voltage_alpha_beta(&self) -> AlphaBeta<Volts> {
        if self.result.is_some() || self.error.is_some() {
            return AlphaBeta::new(Volts::ZERO, Volts::ZERO);
        }

        let (s, c) = sin_cos(self.config.align_stator_angle.get());
        let mag = self.config.align_voltage_mag.get();
        AlphaBeta::new(Volts::new(mag * c), Volts::new(mag * s))
    }

    /// Advances the calibration procedure by one sample.
    pub fn tick(&mut self, input: PhaseResistanceCalibrationInput) -> AlphaBeta<Volts> {
        if self.result.is_some() || self.error.is_some() {
            return AlphaBeta::new(Volts::ZERO, Volts::ZERO);
        }

        if !validate_input(input) {
            self.error = Some(CalibrationError::InvalidInput);
            return AlphaBeta::new(Volts::ZERO, Volts::ZERO);
        }

        self.elapsed_seconds += input.dt_seconds;
        if self.elapsed_seconds >= self.config.timeout_seconds {
            self.error = Some(CalibrationError::Timeout);
            return AlphaBeta::new(Volts::ZERO, Volts::ZERO);
        }

        if input.mechanical_velocity.get().abs() > self.config.settle_velocity_threshold.get() {
            self.settled_seconds = 0.0;
            self.sample_seconds = 0.0;
            self.projected_current_integral = 0.0;
            return self.commanded_voltage_alpha_beta();
        }

        if self.sample_seconds == 0.0 {
            self.settled_seconds += input.dt_seconds;
            if self.settled_seconds < self.config.settle_time_seconds {
                return self.commanded_voltage_alpha_beta();
            }
        }

        let (s, c) = sin_cos(self.config.align_stator_angle.get());
        let unit = AlphaBeta::new(c, s);
        let projected_current = project_alpha_beta_current(input.phase_currents, unit);
        self.projected_current_integral += projected_current * input.dt_seconds;
        self.sample_seconds += input.dt_seconds;

        if self.sample_seconds >= self.config.sample_time_seconds {
            let mean_current = self.projected_current_integral / self.sample_seconds;
            if !mean_current.is_finite()
                || mean_current.abs() < self.config.min_projected_current.get()
            {
                self.error = Some(CalibrationError::IndeterminateEstimate);
                return AlphaBeta::new(Volts::ZERO, Volts::ZERO);
            }

            self.result = Some(PhaseResistanceCalibrationResult {
                phase_resistance_ohm: Ohms::new(
                    self.config.align_voltage_mag.get().abs() / mean_current.abs(),
                ),
            });
            return AlphaBeta::new(Volts::ZERO, Volts::ZERO);
        }

        self.commanded_voltage_alpha_beta()
    }
}

fn project_alpha_beta_current(phase_currents: Abc<Amps>, unit: AlphaBeta<f32>) -> f32 {
    let current_ab = clarke(phase_currents.map(|current| current.get()));
    current_ab.alpha * unit.alpha + current_ab.beta * unit.beta
}

fn validate_config(config: PhaseResistanceCalibrationConfig) -> bool {
    config.align_voltage_mag.get().is_finite()
        && config.align_voltage_mag.get() > 0.0
        && config.align_stator_angle.get().is_finite()
        && config.settle_velocity_threshold.get().is_finite()
        && config.settle_velocity_threshold.get() >= 0.0
        && config.settle_time_seconds.is_finite()
        && config.settle_time_seconds > 0.0
        && config.sample_time_seconds.is_finite()
        && config.sample_time_seconds > 0.0
        && config.min_projected_current.get().is_finite()
        && config.min_projected_current.get() > 0.0
        && config.timeout_seconds.is_finite()
        && config.timeout_seconds > (config.settle_time_seconds + config.sample_time_seconds)
}

fn validate_input(input: PhaseResistanceCalibrationInput) -> bool {
    input.phase_currents.a.get().is_finite()
        && input.phase_currents.b.get().is_finite()
        && input.phase_currents.c.get().is_finite()
        && input.mechanical_velocity.get().is_finite()
        && input.dt_seconds.is_finite()
        && input.dt_seconds > 0.0
}

#[cfg(test)]
mod tests {
    use fluxkit_math::{frame::Abc, units::Amps};

    use super::{
        CalibrationError, PhaseResistanceCalibrationConfig, PhaseResistanceCalibrationInput,
        PhaseResistanceCalibrationState, PhaseResistanceCalibrator,
    };

    #[test]
    fn completes_from_steady_projected_current() {
        let mut calibrator = PhaseResistanceCalibrator::new(PhaseResistanceCalibrationConfig {
            settle_time_seconds: 0.01,
            sample_time_seconds: 0.02,
            timeout_seconds: 1.0,
            ..PhaseResistanceCalibrationConfig::default_for_hold()
        })
        .unwrap();

        for _ in 0..8 {
            let _ = calibrator.tick(PhaseResistanceCalibrationInput {
                phase_currents: Abc::new(Amps::new(2.0), Amps::new(-1.0), Amps::new(-1.0)),
                mechanical_velocity: fluxkit_math::units::RadPerSec::ZERO,
                dt_seconds: 0.005,
            });
            if calibrator.state() == PhaseResistanceCalibrationState::Complete {
                break;
            }
        }

        assert_eq!(
            calibrator.state(),
            PhaseResistanceCalibrationState::Complete
        );
        let result = calibrator.result().unwrap();
        assert!((result.phase_resistance_ohm.get() - 0.5).abs() < 1.0e-6);
    }

    #[test]
    fn times_out_when_rotor_never_settles() {
        let mut calibrator = PhaseResistanceCalibrator::new(PhaseResistanceCalibrationConfig {
            settle_velocity_threshold: fluxkit_math::units::RadPerSec::new(0.01),
            settle_time_seconds: 0.02,
            sample_time_seconds: 0.02,
            timeout_seconds: 0.05,
            ..PhaseResistanceCalibrationConfig::default_for_hold()
        })
        .unwrap();

        for _ in 0..20 {
            let _ = calibrator.tick(PhaseResistanceCalibrationInput {
                phase_currents: Abc::new(Amps::ZERO, Amps::ZERO, Amps::ZERO),
                mechanical_velocity: fluxkit_math::units::RadPerSec::new(1.0),
                dt_seconds: 0.005,
            });
        }

        assert_eq!(
            calibrator.state(),
            PhaseResistanceCalibrationState::Failed(CalibrationError::Timeout)
        );
    }
}
