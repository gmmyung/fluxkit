//! Phase-inductance calibration via magnetic hold and a small voltage step.
//!
//! The procedure first applies a fixed hold vector so the rotor settles against
//! a known stator field. Once the rotor is stationary enough, it applies an
//! additional voltage step along the same stator direction and estimates the
//! common phase inductance from multiple projected-current samples and the
//! previously calibrated phase resistance:
//!
//! `L ~= (dV - R * mean(di)) / (di / dt)`
//!
//! For now this procedure identifies one common inductance and the persisted
//! calibration surface applies it as `Ld = Lq`.

use fluxkit_math::{
    AlphaBeta, ElectricalAngle,
    frame::Abc,
    transforms::clarke,
    trig::sin_cos,
    units::{Amps, Henries, Ohms, RadPerSec, Volts},
};

use super::error::CalibrationError;

/// Static configuration for phase-inductance calibration.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct PhaseInductanceCalibrationConfig {
    /// Previously calibrated phase resistance used to correct the voltage step.
    pub phase_resistance_ohm: Ohms,
    /// Baseline hold voltage magnitude used to align and hold the rotor.
    pub hold_voltage_mag: Volts,
    /// Additional voltage step magnitude applied during the slope measurement.
    pub step_voltage_mag: Volts,
    /// Stator-frame angle of the excitation vector.
    pub align_stator_angle: ElectricalAngle,
    /// Maximum mechanical speed considered "settled".
    pub settle_velocity_threshold: RadPerSec,
    /// Continuous settle time required before applying the step.
    pub settle_time_seconds: f32,
    /// Short measurement window used for the current-slope estimate.
    pub sample_time_seconds: f32,
    /// Minimum usable projected-current step.
    pub min_projected_current_step: Amps,
    /// Absolute timeout for the whole procedure.
    pub timeout_seconds: f32,
}

impl PhaseInductanceCalibrationConfig {
    /// Returns a conservative default suitable for simulator-backed tests.
    pub const fn default_for_hold() -> Self {
        Self {
            phase_resistance_ohm: Ohms::new(0.1),
            hold_voltage_mag: Volts::new(1.0),
            step_voltage_mag: Volts::new(0.5),
            align_stator_angle: ElectricalAngle::new(0.0),
            settle_velocity_threshold: RadPerSec::new(0.05),
            settle_time_seconds: 0.05,
            sample_time_seconds: 200.0e-6,
            min_projected_current_step: Amps::new(0.05),
            timeout_seconds: 2.0,
        }
    }
}

/// One synchronous sample frame for phase-inductance calibration.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct PhaseInductanceCalibrationInput {
    /// Measured three-phase currents.
    pub phase_currents: Abc<Amps>,
    /// Mechanical rotor velocity reported by the encoder path.
    pub mechanical_velocity: RadPerSec,
    /// Time since the previous calibration tick.
    pub dt_seconds: f32,
}

/// Result of a completed phase-inductance calibration.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct PhaseInductanceCalibrationResult {
    /// Calibrated common phase inductance to store as `Ld = Lq`.
    pub phase_inductance_h: Henries,
}

/// Compact state of the phase-inductance calibration procedure.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum PhaseInductanceCalibrationState {
    /// The calibrator is holding the field and waiting for settle.
    Aligning,
    /// The calibrator is measuring the early current step response.
    Sampling,
    /// The procedure completed successfully.
    Complete,
    /// The procedure failed.
    Failed(CalibrationError),
}

/// Pure state machine for common phase-inductance calibration.
#[derive(Clone, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct PhaseInductanceCalibrator {
    config: PhaseInductanceCalibrationConfig,
    elapsed_seconds: f32,
    settled_seconds: f32,
    sample_seconds: f32,
    baseline_projected_current: Option<f32>,
    sample_count: u32,
    sum_t: f32,
    sum_delta_i: f32,
    sum_t_sq: f32,
    sum_t_delta_i: f32,
    result: Option<PhaseInductanceCalibrationResult>,
    error: Option<CalibrationError>,
}

impl PhaseInductanceCalibrator {
    /// Creates a new phase-inductance calibrator.
    pub fn new(config: PhaseInductanceCalibrationConfig) -> Result<Self, CalibrationError> {
        if !validate_config(config) {
            return Err(CalibrationError::InvalidConfiguration);
        }

        Ok(Self {
            config,
            elapsed_seconds: 0.0,
            settled_seconds: 0.0,
            sample_seconds: 0.0,
            baseline_projected_current: None,
            sample_count: 0,
            sum_t: 0.0,
            sum_delta_i: 0.0,
            sum_t_sq: 0.0,
            sum_t_delta_i: 0.0,
            result: None,
            error: None,
        })
    }

    /// Returns the current calibration state.
    #[inline]
    pub const fn state(&self) -> PhaseInductanceCalibrationState {
        if let Some(error) = self.error {
            PhaseInductanceCalibrationState::Failed(error)
        } else if self.result.is_some() {
            PhaseInductanceCalibrationState::Complete
        } else if self.baseline_projected_current.is_some() {
            PhaseInductanceCalibrationState::Sampling
        } else {
            PhaseInductanceCalibrationState::Aligning
        }
    }

    /// Returns the finished result when calibration has succeeded.
    #[inline]
    pub const fn result(&self) -> Option<PhaseInductanceCalibrationResult> {
        self.result
    }

    /// Returns the failure cause when calibration has failed.
    #[inline]
    pub const fn error(&self) -> Option<CalibrationError> {
        self.error
    }

    /// Returns the currently commanded `alpha/beta` excitation vector.
    #[inline]
    pub fn commanded_voltage_alpha_beta(&self) -> AlphaBeta<Volts> {
        if self.result.is_some() || self.error.is_some() {
            return AlphaBeta::new(Volts::ZERO, Volts::ZERO);
        }

        let magnitude = if self.baseline_projected_current.is_some() {
            self.config.hold_voltage_mag.get() + self.config.step_voltage_mag.get()
        } else {
            self.config.hold_voltage_mag.get()
        };
        voltage_vector(self.config.align_stator_angle, magnitude)
    }

    /// Advances the calibration procedure by one sample.
    pub fn tick(&mut self, input: PhaseInductanceCalibrationInput) -> AlphaBeta<Volts> {
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

        let projected_current =
            projected_current(input.phase_currents, self.config.align_stator_angle);

        if input.mechanical_velocity.get().abs() > self.config.settle_velocity_threshold.get() {
            self.reset_measurement_state();
            return self.commanded_voltage_alpha_beta();
        }

        if let Some(baseline_current) = self.baseline_projected_current {
            self.sample_seconds += input.dt_seconds;
            let sample_time = self.sample_seconds;
            let delta_i = projected_current - baseline_current;
            self.sample_count += 1;
            self.sum_t += sample_time;
            self.sum_delta_i += delta_i;
            self.sum_t_sq += sample_time * sample_time;
            self.sum_t_delta_i += sample_time * delta_i;

            if self.sample_seconds >= self.config.sample_time_seconds {
                return self.finish_inductance_estimate();
            }

            return self.commanded_voltage_alpha_beta();
        }

        self.settled_seconds += input.dt_seconds;
        if self.settled_seconds >= self.config.settle_time_seconds {
            self.baseline_projected_current = Some(projected_current);
            self.sample_seconds = 0.0;
            self.sample_count = 0;
            self.sum_t = 0.0;
            self.sum_delta_i = 0.0;
            self.sum_t_sq = 0.0;
            self.sum_t_delta_i = 0.0;
        }

        self.commanded_voltage_alpha_beta()
    }

    fn finish_inductance_estimate(&mut self) -> AlphaBeta<Volts> {
        let n = self.sample_count as f32;
        let denom = n * self.sum_t_sq - self.sum_t * self.sum_t;
        if self.sample_count < 2 || !denom.is_finite() || denom.abs() <= 1.0e-15 {
            self.error = Some(CalibrationError::IndeterminateEstimate);
            return AlphaBeta::new(Volts::ZERO, Volts::ZERO);
        }

        let mean_delta_i = self.sum_delta_i / n;
        let slope = (n * self.sum_t_delta_i - self.sum_t * self.sum_delta_i) / denom;
        if !mean_delta_i.is_finite()
            || !slope.is_finite()
            || mean_delta_i.abs() < self.config.min_projected_current_step.get()
            || slope.abs() <= 1.0e-9
        {
            self.error = Some(CalibrationError::IndeterminateEstimate);
            return AlphaBeta::new(Volts::ZERO, Volts::ZERO);
        }

        let effective_step_voltage = self.config.step_voltage_mag.get()
            - self.config.phase_resistance_ohm.get() * mean_delta_i;
        let inductance = effective_step_voltage / slope;
        if !inductance.is_finite() || inductance <= 0.0 {
            self.error = Some(CalibrationError::IndeterminateEstimate);
            return AlphaBeta::new(Volts::ZERO, Volts::ZERO);
        }

        self.result = Some(PhaseInductanceCalibrationResult {
            phase_inductance_h: Henries::new(inductance),
        });
        AlphaBeta::new(Volts::ZERO, Volts::ZERO)
    }

    fn reset_measurement_state(&mut self) {
        self.settled_seconds = 0.0;
        self.sample_seconds = 0.0;
        self.baseline_projected_current = None;
        self.sample_count = 0;
        self.sum_t = 0.0;
        self.sum_delta_i = 0.0;
        self.sum_t_sq = 0.0;
        self.sum_t_delta_i = 0.0;
    }
}

fn projected_current(phase_currents: Abc<Amps>, angle: ElectricalAngle) -> f32 {
    let current_ab = clarke(phase_currents.map(|current| current.get()));
    let (s, c) = sin_cos(angle.get());
    current_ab.alpha * c + current_ab.beta * s
}

fn voltage_vector(angle: ElectricalAngle, magnitude: f32) -> AlphaBeta<Volts> {
    let (s, c) = sin_cos(angle.get());
    AlphaBeta::new(Volts::new(magnitude * c), Volts::new(magnitude * s))
}

fn validate_config(config: PhaseInductanceCalibrationConfig) -> bool {
    config.phase_resistance_ohm.get().is_finite()
        && config.phase_resistance_ohm.get() > 0.0
        && config.hold_voltage_mag.get().is_finite()
        && config.hold_voltage_mag.get() > 0.0
        && config.step_voltage_mag.get().is_finite()
        && config.step_voltage_mag.get() > 0.0
        && config.align_stator_angle.get().is_finite()
        && config.settle_velocity_threshold.get().is_finite()
        && config.settle_velocity_threshold.get() >= 0.0
        && config.settle_time_seconds.is_finite()
        && config.settle_time_seconds > 0.0
        && config.sample_time_seconds.is_finite()
        && config.sample_time_seconds > 0.0
        && config.min_projected_current_step.get().is_finite()
        && config.min_projected_current_step.get() > 0.0
        && config.timeout_seconds.is_finite()
        && config.timeout_seconds > (config.settle_time_seconds + config.sample_time_seconds)
}

fn validate_input(input: PhaseInductanceCalibrationInput) -> bool {
    input.phase_currents.a.get().is_finite()
        && input.phase_currents.b.get().is_finite()
        && input.phase_currents.c.get().is_finite()
        && input.mechanical_velocity.get().is_finite()
        && input.dt_seconds.is_finite()
        && input.dt_seconds > 0.0
}

#[cfg(test)]
mod tests {
    use fluxkit_math::{
        frame::Abc,
        units::{Amps, Ohms, Volts},
    };

    use super::{
        CalibrationError, PhaseInductanceCalibrationConfig, PhaseInductanceCalibrationInput,
        PhaseInductanceCalibrationState, PhaseInductanceCalibrator,
    };

    #[test]
    fn completes_from_projected_current_step() {
        let mut calibrator = PhaseInductanceCalibrator::new(PhaseInductanceCalibrationConfig {
            phase_resistance_ohm: Ohms::new(0.5),
            hold_voltage_mag: Volts::new(1.0),
            step_voltage_mag: Volts::new(0.5),
            settle_time_seconds: 0.01,
            sample_time_seconds: 0.01,
            timeout_seconds: 1.0,
            min_projected_current_step: Amps::new(0.1),
            ..PhaseInductanceCalibrationConfig::default_for_hold()
        })
        .unwrap();

        let baseline = Abc::new(Amps::new(2.0), Amps::new(-1.0), Amps::new(-1.0));
        let stepped_1 = Abc::new(Amps::new(2.5), Amps::new(-1.25), Amps::new(-1.25));
        let stepped_2 = Abc::new(Amps::new(3.0), Amps::new(-1.5), Amps::new(-1.5));

        for step in 0..8 {
            let currents = if step < 2 {
                baseline
            } else if step == 2 {
                stepped_1
            } else {
                stepped_2
            };
            let _ = calibrator.tick(PhaseInductanceCalibrationInput {
                phase_currents: currents,
                mechanical_velocity: fluxkit_math::units::RadPerSec::ZERO,
                dt_seconds: 0.005,
            });
            if calibrator.state() == PhaseInductanceCalibrationState::Complete {
                break;
            }
        }

        assert_eq!(
            calibrator.state(),
            PhaseInductanceCalibrationState::Complete
        );
        let result = calibrator.result().unwrap();
        assert!((result.phase_inductance_h.get() - 0.00125).abs() < 1.0e-6);
    }

    #[test]
    fn times_out_when_rotor_never_settles() {
        let mut calibrator = PhaseInductanceCalibrator::new(PhaseInductanceCalibrationConfig {
            phase_resistance_ohm: Ohms::new(0.5),
            settle_velocity_threshold: fluxkit_math::units::RadPerSec::new(0.01),
            settle_time_seconds: 0.02,
            sample_time_seconds: 0.01,
            timeout_seconds: 0.05,
            ..PhaseInductanceCalibrationConfig::default_for_hold()
        })
        .unwrap();

        for _ in 0..20 {
            let _ = calibrator.tick(PhaseInductanceCalibrationInput {
                phase_currents: Abc::new(Amps::ZERO, Amps::ZERO, Amps::ZERO),
                mechanical_velocity: fluxkit_math::units::RadPerSec::new(1.0),
                dt_seconds: 0.005,
            });
        }

        assert_eq!(
            calibrator.state(),
            PhaseInductanceCalibrationState::Failed(CalibrationError::Timeout)
        );
    }
}
