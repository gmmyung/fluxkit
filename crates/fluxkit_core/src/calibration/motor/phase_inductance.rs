//! Phase-inductance calibration via repeated exact-RL `0 -> +V` d-axis pulses.
//!
//! The procedure first applies a fixed hold vector so the rotor settles against
//! a known stator field. Once the rotor is stationary enough, it switches to
//! repeated `0 -> +V -> 0` excitation on the rotor `d` axis and estimates the
//! common phase inductance from every valid current sample within those pulse
//! segments.
//!
//! With known phase resistance `R`, fixed pulse voltage `V`, and constant
//! voltage over one sample interval, the exact first-order RL response is:
//!
//! `i(t) - i_inf = exp(-R t / L) * (i0 - i_inf)`
//!
//! where `i0` is the measured current at the segment edge, `i_inf = V / R` for
//! the on window, and `i_inf = 0` for the off window. The calibrator
//! accumulates `ln(|residual / residual0|)` versus time-since-edge over many
//! valid samples, then solves for `L` from the fitted decay rate.
//!
//! For now this procedure identifies one common inductance and the persisted
//! calibration surface applies it as `Ld = Lq`.

use fluxkit_math::{
    AlphaBeta, ElectricalAngle,
    frame::{Abc, Dq},
    scalar::ln,
    transforms::{clarke, inverse_park, park},
    trig::sin_cos,
    units::{Amps, Henries, Ohms, RadPerSec, Volts},
};

use crate::calibration::shared::{CalibrationError, timing::HoldTiming};

const MIN_RESIDUAL_MAGNITUDE: f32 = 1.0e-6;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
enum MeasurementPhase {
    PreconditioningOff,
    OnPulse,
    OffPulse,
}

/// Static configuration for phase-inductance calibration.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct PhaseInductanceCalibrationConfig {
    /// Previously calibrated phase resistance used by the exact RL model.
    pub phase_resistance_ohm: Ohms,
    /// Baseline hold voltage magnitude used to align and hold the rotor before
    /// pulse measurement starts.
    pub hold_voltage_mag: Volts,
    /// Magnitude of the repeated `d`-axis pulse voltage.
    pub step_voltage_mag: Volts,
    /// Stator-frame angle of the initial hold vector.
    pub align_stator_angle: ElectricalAngle,
    /// Maximum mechanical speed considered "settled" before pulse measurement
    /// begins.
    pub settle_velocity_threshold: RadPerSec,
    /// Continuous settle time required before starting pulses.
    pub settle_time_seconds: f32,
    /// Duration of each on or off pulse window.
    pub sample_time_seconds: f32,
    /// Number of `0 -> +V -> 0` pulse cycles to accumulate.
    pub repeat_count: u16,
    /// Minimum usable `d`-axis current rise during any positive pulse window.
    pub min_projected_current_step: Amps,
    /// Absolute timeout for the whole procedure.
    pub timeout_seconds: f32,
}

impl PhaseInductanceCalibrationConfig {
    /// Returns a conservative default suitable for simulator-backed tests.
    pub fn default_for_hold() -> Self {
        Self {
            phase_resistance_ohm: Ohms::new(0.1),
            hold_voltage_mag: Volts::new(1.0),
            step_voltage_mag: Volts::new(0.5),
            align_stator_angle: ElectricalAngle::new(0.0),
            settle_velocity_threshold: RadPerSec::new(1.0),
            settle_time_seconds: 0.05,
            sample_time_seconds: 1.0e-3,
            repeat_count: 500,
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
    /// Electrical rotor angle used to express the pulse on the `d` axis.
    pub electrical_angle: ElectricalAngle,
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

/// Pure state machine for common phase-inductance calibration.
#[derive(Clone, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct PhaseInductanceCalibrator {
    config: PhaseInductanceCalibrationConfig,
    timing: HoldTiming,
    last_electrical_angle: Option<ElectricalAngle>,
    measurement_phase: Option<MeasurementPhase>,
    completed_cycles: u16,
    window_seconds: f32,
    window_start_current_d: Option<f32>,
    max_positive_window_delta_i: f32,
    fit_sample_count: u32,
    sum_t_sq: f32,
    sum_t_log_ratio: f32,
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
            timing: HoldTiming::new(),
            last_electrical_angle: None,
            measurement_phase: None,
            completed_cycles: 0,
            window_seconds: 0.0,
            window_start_current_d: None,
            max_positive_window_delta_i: 0.0,
            fit_sample_count: 0,
            sum_t_sq: 0.0,
            sum_t_log_ratio: 0.0,
            result: None,
            error: None,
        })
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

        match self.measurement_phase {
            None => hold_voltage_vector(
                self.config.align_stator_angle,
                self.config.hold_voltage_mag.get(),
            ),
            Some(MeasurementPhase::PreconditioningOff | MeasurementPhase::OffPulse) => {
                AlphaBeta::new(Volts::ZERO, Volts::ZERO)
            }
            Some(MeasurementPhase::OnPulse) => {
                let electrical_angle = self
                    .last_electrical_angle
                    .unwrap_or(ElectricalAngle::new(0.0));
                d_axis_voltage_vector(electrical_angle, self.config.step_voltage_mag.get())
            }
        }
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

        self.last_electrical_angle = Some(input.electrical_angle);

        if let Some(error) = self
            .timing
            .advance_elapsed(input.dt_seconds, self.config.timeout_seconds)
        {
            self.error = Some(error);
            return AlphaBeta::new(Volts::ZERO, Volts::ZERO);
        }

        let current_d = d_axis_current(input.phase_currents, input.electrical_angle);

        if let Some(phase) = self.measurement_phase {
            self.accumulate_segment_sample(phase, current_d, input.dt_seconds);
            self.window_seconds += input.dt_seconds;

            if self.window_seconds >= self.config.sample_time_seconds {
                self.complete_window(phase, current_d);
                if self.result.is_some() || self.error.is_some() {
                    return AlphaBeta::new(Volts::ZERO, Volts::ZERO);
                }
            }

            return self.commanded_voltage_alpha_beta();
        }

        if input.mechanical_velocity.get().abs() > self.config.settle_velocity_threshold.get() {
            self.reset_measurement_state();
            return self.commanded_voltage_alpha_beta();
        }

        if !self
            .timing
            .settle_ready(true, input.dt_seconds, self.config.settle_time_seconds)
        {
            return self.commanded_voltage_alpha_beta();
        }

        self.start_measurement(current_d);
        self.commanded_voltage_alpha_beta()
    }

    fn start_measurement(&mut self, current_d: f32) {
        self.measurement_phase = Some(MeasurementPhase::PreconditioningOff);
        self.completed_cycles = 0;
        self.window_seconds = 0.0;
        self.window_start_current_d = Some(current_d);
        self.max_positive_window_delta_i = 0.0;
        self.fit_sample_count = 0;
        self.sum_t_sq = 0.0;
        self.sum_t_log_ratio = 0.0;
    }

    fn accumulate_segment_sample(
        &mut self,
        phase: MeasurementPhase,
        current_d: f32,
        dt_seconds: f32,
    ) {
        let Some(start_current_d) = self.window_start_current_d else {
            return;
        };

        let target_current = match phase {
            MeasurementPhase::PreconditioningOff | MeasurementPhase::OffPulse => 0.0,
            MeasurementPhase::OnPulse => {
                self.config.step_voltage_mag.get() / self.config.phase_resistance_ohm.get()
            }
        };
        let start_residual = target_current - start_current_d;
        let current_residual = target_current - current_d;
        let sample_time = self.window_seconds + dt_seconds;
        if start_residual.is_finite()
            && current_residual.is_finite()
            && sample_time.is_finite()
            && sample_time > 0.0
            && start_residual.abs() > MIN_RESIDUAL_MAGNITUDE
            && current_residual.abs() > MIN_RESIDUAL_MAGNITUDE
            && start_residual * current_residual > 0.0
        {
            let log_ratio = ln(current_residual.abs() / start_residual.abs());
            if log_ratio.is_finite() && log_ratio < 0.0 {
                self.fit_sample_count += 1;
                self.sum_t_sq += sample_time * sample_time;
                self.sum_t_log_ratio += sample_time * log_ratio;
            }
        }
    }

    fn complete_window(&mut self, phase: MeasurementPhase, current_d: f32) {
        match phase {
            MeasurementPhase::PreconditioningOff => {
                self.measurement_phase = Some(MeasurementPhase::OnPulse);
            }
            MeasurementPhase::OnPulse => {
                if let Some(start_current_d) = self.window_start_current_d {
                    self.max_positive_window_delta_i = self
                        .max_positive_window_delta_i
                        .max(current_d - start_current_d);
                }
                self.measurement_phase = Some(MeasurementPhase::OffPulse);
            }
            MeasurementPhase::OffPulse => {
                self.completed_cycles += 1;
                if self.completed_cycles >= self.config.repeat_count {
                    self.finish_inductance_estimate();
                    return;
                }
                self.measurement_phase = Some(MeasurementPhase::OnPulse);
            }
        }

        self.window_seconds = 0.0;
        self.window_start_current_d = Some(current_d);
    }

    fn finish_inductance_estimate(&mut self) {
        if self.fit_sample_count < 2
            || !self.max_positive_window_delta_i.is_finite()
            || self.max_positive_window_delta_i < self.config.min_projected_current_step.get()
            || !self.sum_t_sq.is_finite()
            || self.sum_t_sq <= 1.0e-15
            || !self.sum_t_log_ratio.is_finite()
        {
            fluxkit_warn!(
                "phase inductance calibration indeterminate insufficient fit fit_sample_count={} max_positive_delta_i={} min_projected_current_step={} sum_t_sq={} sum_t_log_ratio={} completed_cycles={}",
                self.fit_sample_count,
                self.max_positive_window_delta_i,
                self.config.min_projected_current_step.get(),
                self.sum_t_sq,
                self.sum_t_log_ratio,
                self.completed_cycles
            );
            self.error = Some(CalibrationError::IndeterminateEstimate);
            return;
        }

        let decay_rate = -self.sum_t_log_ratio / self.sum_t_sq;
        if !decay_rate.is_finite() || decay_rate <= 1.0e-9 {
            fluxkit_warn!(
                "phase inductance calibration indeterminate invalid decay_rate={} sum_t_sq={} sum_t_log_ratio={}",
                decay_rate,
                self.sum_t_sq,
                self.sum_t_log_ratio
            );
            self.error = Some(CalibrationError::IndeterminateEstimate);
            return;
        }

        let inductance = self.config.phase_resistance_ohm.get() / decay_rate;
        if !inductance.is_finite() || inductance <= 0.0 {
            fluxkit_warn!(
                "phase inductance calibration indeterminate invalid inductance={} resistance={} decay_rate={}",
                inductance,
                self.config.phase_resistance_ohm.get(),
                decay_rate
            );
            self.error = Some(CalibrationError::IndeterminateEstimate);
            return;
        }

        self.result = Some(PhaseInductanceCalibrationResult {
            phase_inductance_h: Henries::new(inductance),
        });
    }

    fn reset_measurement_state(&mut self) {
        self.timing.reset_settle();
        self.measurement_phase = None;
        self.completed_cycles = 0;
        self.window_seconds = 0.0;
        self.window_start_current_d = None;
        self.max_positive_window_delta_i = 0.0;
        self.fit_sample_count = 0;
        self.sum_t_sq = 0.0;
        self.sum_t_log_ratio = 0.0;
    }
}

fn d_axis_current(phase_currents: Abc<Amps>, electrical_angle: ElectricalAngle) -> f32 {
    let current_ab = clarke(phase_currents.map(|current| current.get()));
    park(current_ab, electrical_angle.get()).d
}

fn hold_voltage_vector(angle: ElectricalAngle, magnitude: f32) -> AlphaBeta<Volts> {
    let (s, c) = sin_cos(angle.get());
    AlphaBeta::new(Volts::new(magnitude * c), Volts::new(magnitude * s))
}

fn d_axis_voltage_vector(electrical_angle: ElectricalAngle, vd: f32) -> AlphaBeta<Volts> {
    let voltage_ab = inverse_park(Dq::new(vd, 0.0), electrical_angle.get());
    AlphaBeta::new(Volts::new(voltage_ab.alpha), Volts::new(voltage_ab.beta))
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
        && config.repeat_count > 0
        && config.min_projected_current_step.get().is_finite()
        && config.min_projected_current_step.get() > 0.0
        && config.timeout_seconds.is_finite()
        && config.timeout_seconds
            > (config.settle_time_seconds
                + config.sample_time_seconds * (2.0 * config.repeat_count as f32 + 1.0))
}

fn validate_input(input: PhaseInductanceCalibrationInput) -> bool {
    input.phase_currents.a.get().is_finite()
        && input.phase_currents.b.get().is_finite()
        && input.phase_currents.c.get().is_finite()
        && input.electrical_angle.get().is_finite()
        && input.mechanical_velocity.get().is_finite()
        && input.dt_seconds.is_finite()
        && input.dt_seconds > 0.0
}

#[cfg(test)]
mod tests {
    use fluxkit_math::{
        ElectricalAngle,
        frame::Abc,
        transforms::park,
        units::{Amps, Ohms, Volts},
    };

    use super::{
        CalibrationError, PhaseInductanceCalibrationConfig, PhaseInductanceCalibrationInput,
        PhaseInductanceCalibrator,
    };

    fn phase_currents_for_d_current(current_d: f32) -> Abc<Amps> {
        Abc::new(
            Amps::new(current_d),
            Amps::new(-0.5 * current_d),
            Amps::new(-0.5 * current_d),
        )
    }

    fn advance_rl(current: f32, voltage: f32, resistance: f32, inductance: f32, dt: f32) -> f32 {
        let steady_state = voltage / resistance;
        let decay = (-resistance * dt / inductance).exp();
        steady_state + (current - steady_state) * decay
    }

    #[test]
    fn completes_from_repeated_zero_to_positive_pulses() {
        let resistance_ohm = 0.5;
        let inductance_h = 0.00125;
        let dt_seconds = 0.0001;
        let electrical_angle = ElectricalAngle::new(0.0);
        let mut calibrator = PhaseInductanceCalibrator::new(PhaseInductanceCalibrationConfig {
            phase_resistance_ohm: Ohms::new(resistance_ohm),
            hold_voltage_mag: Volts::new(1.0),
            step_voltage_mag: Volts::new(0.5),
            settle_time_seconds: 0.002,
            sample_time_seconds: 0.001,
            repeat_count: 40,
            timeout_seconds: 1.0,
            min_projected_current_step: Amps::new(0.05),
            ..PhaseInductanceCalibrationConfig::default_for_hold()
        })
        .unwrap();
        let mut current_d = 0.0;

        for _ in 0..1200 {
            let phase_currents = phase_currents_for_d_current(current_d);
            let command = calibrator.tick(PhaseInductanceCalibrationInput {
                phase_currents,
                electrical_angle,
                mechanical_velocity: fluxkit_math::units::RadPerSec::ZERO,
                dt_seconds,
            });
            let applied_d_voltage =
                park(command.map(|voltage| voltage.get()), electrical_angle.get()).d;
            current_d = advance_rl(
                current_d,
                applied_d_voltage,
                resistance_ohm,
                inductance_h,
                dt_seconds,
            );
            if calibrator.result().is_some() || calibrator.error().is_some() {
                break;
            }
        }

        assert_eq!(calibrator.error(), None);
        let result = calibrator.result().unwrap();
        assert!((result.phase_inductance_h.get() - inductance_h).abs() < 2.0e-5);
    }

    #[test]
    fn times_out_when_rotor_never_settles() {
        let mut calibrator = PhaseInductanceCalibrator::new(PhaseInductanceCalibrationConfig {
            phase_resistance_ohm: Ohms::new(0.5),
            settle_velocity_threshold: fluxkit_math::units::RadPerSec::new(0.01),
            settle_time_seconds: 0.02,
            sample_time_seconds: 0.001,
            repeat_count: 5,
            timeout_seconds: 0.1,
            ..PhaseInductanceCalibrationConfig::default_for_hold()
        })
        .unwrap();

        for _ in 0..20 {
            let _ = calibrator.tick(PhaseInductanceCalibrationInput {
                phase_currents: Abc::new(Amps::ZERO, Amps::ZERO, Amps::ZERO),
                electrical_angle: ElectricalAngle::new(0.0),
                mechanical_velocity: fluxkit_math::units::RadPerSec::new(1.0),
                dt_seconds: 0.005,
            });
        }

        assert_eq!(calibrator.error(), Some(CalibrationError::Timeout));
    }

    #[test]
    fn rejects_when_positive_pulse_excursion_is_too_small() {
        let mut calibrator = PhaseInductanceCalibrator::new(PhaseInductanceCalibrationConfig {
            phase_resistance_ohm: Ohms::new(0.5),
            hold_voltage_mag: Volts::new(1.0),
            step_voltage_mag: Volts::new(0.5),
            settle_time_seconds: 0.002,
            sample_time_seconds: 0.001,
            repeat_count: 8,
            timeout_seconds: 1.0,
            min_projected_current_step: Amps::new(0.2),
            ..PhaseInductanceCalibrationConfig::default_for_hold()
        })
        .unwrap();

        for _ in 0..200 {
            let _ = calibrator.tick(PhaseInductanceCalibrationInput {
                phase_currents: phase_currents_for_d_current(0.02),
                electrical_angle: ElectricalAngle::new(0.0),
                mechanical_velocity: fluxkit_math::units::RadPerSec::ZERO,
                dt_seconds: 0.001,
            });
            if calibrator.result().is_some() || calibrator.error().is_some() {
                break;
            }
        }

        assert_eq!(
            calibrator.error(),
            Some(CalibrationError::IndeterminateEstimate)
        );
        assert_eq!(calibrator.result(), None);
    }
}
