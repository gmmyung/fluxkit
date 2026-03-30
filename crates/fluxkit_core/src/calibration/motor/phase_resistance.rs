//! Phase-resistance calibration via repeated magnetic hold and steady current.
//!
//! The procedure applies a stator-frame voltage vector, waits for the rotor to
//! settle against that field, then averages the projected steady-state current
//! along the commanded vector. It repeats that measurement for several voltage
//! levels separated by a fixed increment, then averages the resulting ohmic
//! estimates. Under near-zero mechanical speed, `R ~= V / I` for each
//! stationary excitation.
//!
//! The estimate is intentionally sign-sensitive: the projected current must
//! align with the commanded stator vector. A clearly opposite-signed steady
//! current is rejected as a calibration fault instead of being folded back into
//! a positive resistance magnitude.

use fluxkit_math::{
    AlphaBeta, ElectricalAngle,
    frame::Abc,
    transforms::clarke,
    trig::sin_cos,
    units::{Amps, Ohms, RadPerSec, Volts},
};

use crate::{
    calibration::shared::{CalibrationError, timing::HoldTiming},
    params::{PHASE_RESISTANCE_REFERENCE_TEMP_C, PHASE_RESISTANCE_TEMP_COEFF_PER_C},
};

/// Static configuration for phase-resistance calibration.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct PhaseResistanceCalibrationConfig {
    /// Magnitude of the fixed stator-frame voltage vector.
    pub align_voltage_mag: Volts,
    /// Increment applied to the excitation magnitude between repeated
    /// resistance measurements.
    pub voltage_increment_mag: Volts,
    /// Stator-frame angle of the excitation vector.
    pub align_stator_angle: ElectricalAngle,
    /// Maximum mechanical speed considered "settled".
    pub settle_velocity_threshold: RadPerSec,
    /// Continuous settle time required before sampling current.
    pub settle_time_seconds: f32,
    /// Averaging window for the projected steady-state current.
    pub sample_time_seconds: f32,
    /// Number of repeated resistance measurements to average.
    pub measurement_count: u16,
    /// Minimum usable projected current magnitude.
    pub min_projected_current: Amps,
    /// Absolute timeout for the whole procedure.
    pub timeout_seconds: f32,
}

impl PhaseResistanceCalibrationConfig {
    /// Returns a conservative default suitable for host-side bring-up and
    /// simulator-backed tests.
    pub fn default_for_hold() -> Self {
        Self {
            align_voltage_mag: Volts::new(2.0),
            voltage_increment_mag: Volts::new(0.25),
            align_stator_angle: ElectricalAngle::new(0.0),
            settle_velocity_threshold: RadPerSec::new(1.0),
            settle_time_seconds: 0.05,
            sample_time_seconds: 0.05,
            measurement_count: 3,
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
    /// Measured winding temperature in degrees Celsius.
    pub winding_temperature_c: f32,
    /// Time since the previous calibration tick.
    pub dt_seconds: f32,
}

/// Result of a completed phase-resistance calibration.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct PhaseResistanceCalibrationResult {
    /// Calibrated phase resistance normalized to the fixed `25°C` reference
    /// temperature.
    pub phase_resistance_ohm_ref: Ohms,
}

/// Pure state machine for phase-resistance calibration.
#[derive(Clone, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct PhaseResistanceCalibrator {
    config: PhaseResistanceCalibrationConfig,
    timing: HoldTiming,
    measurement_index: u16,
    sample_seconds: f32,
    projected_current_integral: f32,
    normalized_resistance_sum: f32,
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
            timing: HoldTiming::new(),
            measurement_index: 0,
            sample_seconds: 0.0,
            projected_current_integral: 0.0,
            normalized_resistance_sum: 0.0,
            result: None,
            error: None,
        })
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
        let mag = self.current_align_voltage_mag();
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

        if let Some(error) = self
            .timing
            .advance_elapsed(input.dt_seconds, self.config.timeout_seconds)
        {
            self.error = Some(error);
            return AlphaBeta::new(Volts::ZERO, Volts::ZERO);
        }

        if input.mechanical_velocity.get().abs() > self.config.settle_velocity_threshold.get() {
            self.timing.reset_settle();
            self.sample_seconds = 0.0;
            self.projected_current_integral = 0.0;
            return self.commanded_voltage_alpha_beta();
        }

        if self.sample_seconds == 0.0
            && !self
                .timing
                .settle_ready(true, input.dt_seconds, self.config.settle_time_seconds)
        {
            return self.commanded_voltage_alpha_beta();
        }

        let (s, c) = sin_cos(self.config.align_stator_angle.get());
        let unit = AlphaBeta::new(c, s);
        let projected_current = project_alpha_beta_current(input.phase_currents, unit);
        self.projected_current_integral += projected_current * input.dt_seconds;
        self.sample_seconds += input.dt_seconds;

        if self.sample_seconds >= self.config.sample_time_seconds {
            let mean_current = self.projected_current_integral / self.sample_seconds;
            if !mean_current.is_finite() {
                fluxkit_warn!(
                    "phase resistance calibration indeterminate non-finite mean_current={} sample_s={} measurement_index={}",
                    mean_current,
                    self.sample_seconds,
                    self.measurement_index
                );
                self.error = Some(CalibrationError::IndeterminateEstimate);
                return AlphaBeta::new(Volts::ZERO, Volts::ZERO);
            }

            if mean_current <= -self.config.min_projected_current.get() {
                self.error = Some(CalibrationError::OppositeDirection);
                return AlphaBeta::new(Volts::ZERO, Volts::ZERO);
            }

            if mean_current < self.config.min_projected_current.get() {
                fluxkit_warn!(
                    "phase resistance calibration indeterminate insufficient mean_current={} min_projected_current={} sample_s={} measurement_index={}",
                    mean_current,
                    self.config.min_projected_current.get(),
                    self.sample_seconds,
                    self.measurement_index
                );
                self.error = Some(CalibrationError::IndeterminateEstimate);
                return AlphaBeta::new(Volts::ZERO, Volts::ZERO);
            }

            let measured_resistance = self.current_align_voltage_mag() / mean_current;
            let reference_scale = 1.0
                + PHASE_RESISTANCE_TEMP_COEFF_PER_C
                    * (input.winding_temperature_c - PHASE_RESISTANCE_REFERENCE_TEMP_C);
            if !reference_scale.is_finite() || reference_scale <= 0.0 {
                fluxkit_warn!(
                    "phase resistance calibration indeterminate invalid reference_scale={} winding_temp_c={}",
                    reference_scale,
                    input.winding_temperature_c
                );
                self.error = Some(CalibrationError::IndeterminateEstimate);
                return AlphaBeta::new(Volts::ZERO, Volts::ZERO);
            }

            self.normalized_resistance_sum += measured_resistance / reference_scale;
            self.measurement_index += 1;

            if self.measurement_index >= self.config.measurement_count {
                let mean_resistance =
                    self.normalized_resistance_sum / self.measurement_index as f32;
                if !mean_resistance.is_finite() || mean_resistance <= 0.0 {
                    fluxkit_warn!(
                        "phase resistance calibration indeterminate invalid mean_resistance={} measurement_count={} normalized_sum={}",
                        mean_resistance,
                        self.measurement_index,
                        self.normalized_resistance_sum
                    );
                    self.error = Some(CalibrationError::IndeterminateEstimate);
                    return AlphaBeta::new(Volts::ZERO, Volts::ZERO);
                }

                self.result = Some(PhaseResistanceCalibrationResult {
                    phase_resistance_ohm_ref: Ohms::new(mean_resistance),
                });
                return AlphaBeta::new(Volts::ZERO, Volts::ZERO);
            }

            self.timing.reset_settle();
            self.sample_seconds = 0.0;
            self.projected_current_integral = 0.0;
            return self.commanded_voltage_alpha_beta();
        }

        self.commanded_voltage_alpha_beta()
    }

    #[inline]
    fn current_align_voltage_mag(&self) -> f32 {
        self.config.align_voltage_mag.get()
            + self.config.voltage_increment_mag.get() * self.measurement_index as f32
    }
}

fn project_alpha_beta_current(phase_currents: Abc<Amps>, unit: AlphaBeta<f32>) -> f32 {
    let current_ab = clarke(phase_currents.map(|current| current.get()));
    current_ab.alpha * unit.alpha + current_ab.beta * unit.beta
}

fn validate_config(config: PhaseResistanceCalibrationConfig) -> bool {
    config.align_voltage_mag.get().is_finite()
        && config.align_voltage_mag.get() > 0.0
        && config.voltage_increment_mag.get().is_finite()
        && config.voltage_increment_mag.get() >= 0.0
        && config.align_stator_angle.get().is_finite()
        && config.settle_velocity_threshold.get().is_finite()
        && config.settle_velocity_threshold.get() >= 0.0
        && config.settle_time_seconds.is_finite()
        && config.settle_time_seconds > 0.0
        && config.sample_time_seconds.is_finite()
        && config.sample_time_seconds > 0.0
        && config.measurement_count > 0
        && config.min_projected_current.get().is_finite()
        && config.min_projected_current.get() > 0.0
        && config.timeout_seconds.is_finite()
        && config.timeout_seconds
            > config.measurement_count as f32
                * (config.settle_time_seconds + config.sample_time_seconds)
}

fn validate_input(input: PhaseResistanceCalibrationInput) -> bool {
    input.phase_currents.a.get().is_finite()
        && input.phase_currents.b.get().is_finite()
        && input.phase_currents.c.get().is_finite()
        && input.mechanical_velocity.get().is_finite()
        && input.winding_temperature_c.is_finite()
        && input.dt_seconds.is_finite()
        && input.dt_seconds > 0.0
}

#[cfg(test)]
mod tests {
    use fluxkit_math::{frame::Abc, units::Amps};

    use super::{
        CalibrationError, PhaseResistanceCalibrationConfig, PhaseResistanceCalibrationInput,
        PhaseResistanceCalibrator,
    };

    #[test]
    fn completes_from_steady_projected_current() {
        let mut calibrator = PhaseResistanceCalibrator::new(PhaseResistanceCalibrationConfig {
            align_voltage_mag: fluxkit_math::units::Volts::new(1.0),
            voltage_increment_mag: fluxkit_math::units::Volts::ZERO,
            settle_time_seconds: 0.01,
            sample_time_seconds: 0.02,
            measurement_count: 1,
            timeout_seconds: 1.0,
            ..PhaseResistanceCalibrationConfig::default_for_hold()
        })
        .unwrap();

        for _ in 0..8 {
            let _ = calibrator.tick(PhaseResistanceCalibrationInput {
                phase_currents: Abc::new(Amps::new(2.0), Amps::new(-1.0), Amps::new(-1.0)),
                mechanical_velocity: fluxkit_math::units::RadPerSec::ZERO,
                winding_temperature_c: 25.0,
                dt_seconds: 0.005,
            });
            if calibrator.result().is_some() {
                break;
            }
        }

        assert_eq!(calibrator.error(), None);
        let result = calibrator.result().unwrap();
        assert!((result.phase_resistance_ohm_ref.get() - 0.5).abs() < 1.0e-6);
    }

    #[test]
    fn averages_multiple_steady_state_measurements() {
        let mut calibrator = PhaseResistanceCalibrator::new(PhaseResistanceCalibrationConfig {
            align_voltage_mag: fluxkit_math::units::Volts::new(1.0),
            voltage_increment_mag: fluxkit_math::units::Volts::new(0.25),
            settle_time_seconds: 0.01,
            sample_time_seconds: 0.02,
            measurement_count: 3,
            timeout_seconds: 1.0,
            ..PhaseResistanceCalibrationConfig::default_for_hold()
        })
        .unwrap();

        let projected_currents = [2.0, 2.5, 3.0];
        let mut stage_index = 0usize;

        for tick in 0..24 {
            let projected_current = projected_currents[stage_index];
            let _ = calibrator.tick(PhaseResistanceCalibrationInput {
                phase_currents: Abc::new(
                    Amps::new(projected_current),
                    Amps::new(-0.5 * projected_current),
                    Amps::new(-0.5 * projected_current),
                ),
                mechanical_velocity: fluxkit_math::units::RadPerSec::ZERO,
                winding_temperature_c: 25.0,
                dt_seconds: 0.005,
            });
            if calibrator.result().is_some() {
                break;
            }
            if stage_index < projected_currents.len() - 1 && (tick + 1) % 5 == 0 {
                stage_index += 1;
            }
        }

        assert_eq!(calibrator.error(), None);
        let result = calibrator.result().unwrap();
        assert!((result.phase_resistance_ohm_ref.get() - 0.5).abs() < 1.0e-6);
    }

    #[test]
    fn times_out_when_rotor_never_settles() {
        let mut calibrator = PhaseResistanceCalibrator::new(PhaseResistanceCalibrationConfig {
            settle_velocity_threshold: fluxkit_math::units::RadPerSec::new(0.01),
            settle_time_seconds: 0.02,
            sample_time_seconds: 0.02,
            measurement_count: 1,
            timeout_seconds: 0.05,
            ..PhaseResistanceCalibrationConfig::default_for_hold()
        })
        .unwrap();

        for _ in 0..20 {
            let _ = calibrator.tick(PhaseResistanceCalibrationInput {
                phase_currents: Abc::new(Amps::ZERO, Amps::ZERO, Amps::ZERO),
                mechanical_velocity: fluxkit_math::units::RadPerSec::new(1.0),
                winding_temperature_c: 25.0,
                dt_seconds: 0.005,
            });
        }

        assert_eq!(calibrator.error(), Some(CalibrationError::Timeout));
    }

    #[test]
    fn rejects_opposite_signed_steady_current() {
        let mut calibrator = PhaseResistanceCalibrator::new(PhaseResistanceCalibrationConfig {
            align_voltage_mag: fluxkit_math::units::Volts::new(1.0),
            voltage_increment_mag: fluxkit_math::units::Volts::ZERO,
            settle_time_seconds: 0.01,
            sample_time_seconds: 0.02,
            measurement_count: 1,
            timeout_seconds: 1.0,
            min_projected_current: Amps::new(0.1),
            ..PhaseResistanceCalibrationConfig::default_for_hold()
        })
        .unwrap();

        for _ in 0..8 {
            let _ = calibrator.tick(PhaseResistanceCalibrationInput {
                phase_currents: Abc::new(Amps::new(-2.0), Amps::new(1.0), Amps::new(1.0)),
                mechanical_velocity: fluxkit_math::units::RadPerSec::ZERO,
                winding_temperature_c: 25.0,
                dt_seconds: 0.005,
            });
            if calibrator.error().is_some() {
                break;
            }
        }

        assert_eq!(calibrator.result(), None);
        assert_eq!(
            calibrator.error(),
            Some(CalibrationError::OppositeDirection)
        );
    }
}
