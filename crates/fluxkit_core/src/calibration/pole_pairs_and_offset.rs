//! Pole-pair and electrical-offset calibration via slow stator-field sweep.
//!
//! The procedure first magnetically aligns the rotor to a known stator-frame
//! vector, then rotates that field slowly through a configured number of
//! electrical cycles while observing wrapped mechanical angle travel. The
//! integer pole-pair estimate is derived from the ratio of commanded
//! electrical travel to measured unwrapped mechanical travel. Finally, the
//! endpoint is allowed to settle and the electrical zero offset is derived
//! from the estimated pole-pair count and final measured mechanical angle.

use fluxkit_math::{
    AlphaBeta, ElectricalAngle, MechanicalAngle,
    angle::{mechanical_to_electrical, shortest_angle_delta},
    scalar::TAU,
    trig::sin_cos,
    units::{RadPerSec, Volts},
};

use super::error::CalibrationError;

/// Static configuration for slow-sweep pole-pair and offset calibration.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct PolePairsAndOffsetCalibrationConfig {
    /// Magnitude of the rotating stator-frame voltage vector.
    pub align_voltage_mag: Volts,
    /// Initial stator-frame angle used for the first settle phase.
    pub align_stator_angle: ElectricalAngle,
    /// Signed electrical angular sweep speed in `rad/s`.
    pub sweep_electrical_velocity: RadPerSec,
    /// Number of electrical cycles to sweep through.
    pub sweep_electrical_cycles: f32,
    /// Maximum acceptable rotor speed for the settle phases.
    pub settle_velocity_threshold: RadPerSec,
    /// Required continuous settle time before sweep start.
    pub initial_settle_time_seconds: f32,
    /// Required continuous settle time at sweep end before computing the result.
    pub final_settle_time_seconds: f32,
    /// Maximum allowed deviation from an integer pole-pair estimate.
    pub pole_pair_rounding_tolerance: f32,
    /// Upper bound for the estimated pole-pair count.
    pub max_pole_pairs: u8,
    /// Absolute timeout for the whole procedure.
    pub timeout_seconds: f32,
}

impl PolePairsAndOffsetCalibrationConfig {
    /// Returns a conservative default suitable for slow host-side testing.
    pub const fn default_for_sweep() -> Self {
        Self {
            align_voltage_mag: Volts::new(1.0),
            align_stator_angle: ElectricalAngle::new(0.0),
            sweep_electrical_velocity: RadPerSec::new(8.0),
            sweep_electrical_cycles: 3.0,
            settle_velocity_threshold: RadPerSec::new(0.05),
            initial_settle_time_seconds: 0.05,
            final_settle_time_seconds: 0.05,
            pole_pair_rounding_tolerance: 0.2,
            max_pole_pairs: 64,
            timeout_seconds: 5.0,
        }
    }
}

/// One synchronous sample frame for pole-pair and offset calibration.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct PolePairsAndOffsetCalibrationInput {
    /// Wrapped mechanical rotor angle from the encoder path.
    pub mechanical_angle: MechanicalAngle,
    /// Mechanical rotor velocity from the encoder path.
    pub mechanical_velocity: RadPerSec,
    /// Time since the previous calibration tick.
    pub dt_seconds: f32,
}

/// Result of a completed pole-pair and offset calibration.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct PolePairsAndOffsetCalibrationResult {
    /// Estimated electrical pole-pair count.
    pub pole_pairs: u8,
    /// Calibrated electrical zero offset.
    pub electrical_angle_offset: ElectricalAngle,
}

/// Compact state of the sweep procedure.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum PolePairsAndOffsetCalibrationState {
    /// Waiting for the rotor to settle to the initial hold vector.
    InitialAlign,
    /// Sweeping the commanded stator vector.
    Sweeping,
    /// Waiting for the rotor to settle at the sweep endpoint.
    FinalAlign,
    /// The procedure completed successfully.
    Complete,
    /// The procedure failed.
    Failed(CalibrationError),
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum InternalState {
    InitialAlign,
    Sweeping,
    FinalAlign,
}

/// Pure state machine for pole-pair and offset estimation.
#[derive(Clone, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct PolePairsAndOffsetCalibrator {
    config: PolePairsAndOffsetCalibrationConfig,
    state: InternalState,
    elapsed_seconds: f32,
    settled_seconds: f32,
    commanded_electrical_angle: ElectricalAngle,
    start_unwrapped_mechanical_angle: Option<f32>,
    current_unwrapped_mechanical_angle: f32,
    last_wrapped_mechanical_angle: Option<MechanicalAngle>,
    result: Option<PolePairsAndOffsetCalibrationResult>,
    error: Option<CalibrationError>,
}

impl PolePairsAndOffsetCalibrator {
    /// Creates a new slow-sweep calibrator.
    pub fn new(config: PolePairsAndOffsetCalibrationConfig) -> Result<Self, CalibrationError> {
        if !validate_config(config) {
            return Err(CalibrationError::InvalidConfiguration);
        }

        Ok(Self {
            commanded_electrical_angle: config.align_stator_angle,
            config,
            state: InternalState::InitialAlign,
            elapsed_seconds: 0.0,
            settled_seconds: 0.0,
            start_unwrapped_mechanical_angle: None,
            current_unwrapped_mechanical_angle: 0.0,
            last_wrapped_mechanical_angle: None,
            result: None,
            error: None,
        })
    }

    /// Returns the current procedure state.
    #[inline]
    pub const fn state(&self) -> PolePairsAndOffsetCalibrationState {
        if let Some(error) = self.error {
            PolePairsAndOffsetCalibrationState::Failed(error)
        } else if self.result.is_some() {
            PolePairsAndOffsetCalibrationState::Complete
        } else {
            match self.state {
                InternalState::InitialAlign => PolePairsAndOffsetCalibrationState::InitialAlign,
                InternalState::Sweeping => PolePairsAndOffsetCalibrationState::Sweeping,
                InternalState::FinalAlign => PolePairsAndOffsetCalibrationState::FinalAlign,
            }
        }
    }

    /// Returns the finished result when calibration has succeeded.
    #[inline]
    pub const fn result(&self) -> Option<PolePairsAndOffsetCalibrationResult> {
        self.result
    }

    /// Returns the failure cause when calibration has failed.
    #[inline]
    pub const fn error(&self) -> Option<CalibrationError> {
        self.error
    }

    /// Returns the currently commanded stator-frame hold vector.
    pub fn commanded_voltage_alpha_beta(&self) -> AlphaBeta<Volts> {
        if self.result.is_some() || self.error.is_some() {
            return AlphaBeta::new(Volts::ZERO, Volts::ZERO);
        }

        let (s, c) = sin_cos(self.commanded_electrical_angle.get());
        let mag = self.config.align_voltage_mag.get();
        AlphaBeta::new(Volts::new(mag * c), Volts::new(mag * s))
    }

    /// Advances the sweep procedure by one sample.
    pub fn tick(&mut self, input: PolePairsAndOffsetCalibrationInput) -> AlphaBeta<Volts> {
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

        self.update_unwrapped_mechanical_angle(input.mechanical_angle);

        match self.state {
            InternalState::InitialAlign => {
                self.update_settle_timer(input.mechanical_velocity, input.dt_seconds);
                if self.settled_seconds >= self.config.initial_settle_time_seconds {
                    self.start_unwrapped_mechanical_angle =
                        Some(self.current_unwrapped_mechanical_angle);
                    self.settled_seconds = 0.0;
                    self.state = InternalState::Sweeping;
                }
            }
            InternalState::Sweeping => {
                let advanced = self.config.sweep_electrical_velocity.get() * input.dt_seconds;
                self.commanded_electrical_angle =
                    ElectricalAngle::new(self.commanded_electrical_angle.get() + advanced);

                let signed_target_travel =
                    self.config.sweep_electrical_cycles * self.config.sweep_direction() * TAU;
                let swept_travel =
                    self.commanded_electrical_angle.get() - self.config.align_stator_angle.get();
                if swept_travel.abs() >= signed_target_travel.abs() {
                    self.commanded_electrical_angle = ElectricalAngle::new(
                        self.config.align_stator_angle.get() + signed_target_travel,
                    );
                    self.settled_seconds = 0.0;
                    self.state = InternalState::FinalAlign;
                }
            }
            InternalState::FinalAlign => {
                self.update_settle_timer(input.mechanical_velocity, input.dt_seconds);
                if self.settled_seconds >= self.config.final_settle_time_seconds {
                    let Some(start_mechanical_angle) = self.start_unwrapped_mechanical_angle else {
                        self.error = Some(CalibrationError::IndeterminateEstimate);
                        return AlphaBeta::new(Volts::ZERO, Volts::ZERO);
                    };

                    let mechanical_travel =
                        self.current_unwrapped_mechanical_angle - start_mechanical_angle;
                    let signed_target_travel =
                        self.config.sweep_electrical_cycles * self.config.sweep_direction() * TAU;
                    if mechanical_travel.abs() < f32::EPSILON {
                        self.error = Some(CalibrationError::IndeterminateEstimate);
                        return AlphaBeta::new(Volts::ZERO, Volts::ZERO);
                    }

                    let ratio = signed_target_travel / mechanical_travel;
                    let rounded = (ratio.abs() + 0.5) as u32 as f32;
                    if !ratio.is_finite()
                        || mechanical_travel.abs() < 0.25 * TAU
                        || rounded < 1.0
                        || rounded > self.config.max_pole_pairs as f32
                        || (ratio.abs() - rounded).abs() > self.config.pole_pair_rounding_tolerance
                    {
                        self.error = Some(CalibrationError::IndeterminateEstimate);
                        return AlphaBeta::new(Volts::ZERO, Volts::ZERO);
                    }

                    let pole_pairs = rounded as u8;
                    let measured_electrical =
                        mechanical_to_electrical(input.mechanical_angle, pole_pairs as u32);
                    let offset = ElectricalAngle::new(
                        self.commanded_electrical_angle.get() - measured_electrical.get(),
                    )
                    .wrapped_pm_pi();
                    self.result = Some(PolePairsAndOffsetCalibrationResult {
                        pole_pairs,
                        electrical_angle_offset: offset,
                    });
                    return AlphaBeta::new(Volts::ZERO, Volts::ZERO);
                }
            }
        }

        self.commanded_voltage_alpha_beta()
    }

    fn update_settle_timer(&mut self, mechanical_velocity: RadPerSec, dt_seconds: f32) {
        if mechanical_velocity.get().abs() <= self.config.settle_velocity_threshold.get() {
            self.settled_seconds += dt_seconds;
        } else {
            self.settled_seconds = 0.0;
        }
    }

    fn update_unwrapped_mechanical_angle(&mut self, wrapped_angle: MechanicalAngle) {
        match self.last_wrapped_mechanical_angle {
            Some(last) => {
                self.current_unwrapped_mechanical_angle +=
                    shortest_angle_delta(last.get(), wrapped_angle.get());
                self.last_wrapped_mechanical_angle = Some(wrapped_angle);
            }
            None => {
                self.current_unwrapped_mechanical_angle = wrapped_angle.get();
                self.last_wrapped_mechanical_angle = Some(wrapped_angle);
            }
        }
    }
}

fn validate_config(config: PolePairsAndOffsetCalibrationConfig) -> bool {
    config.align_voltage_mag.get().is_finite()
        && config.align_voltage_mag.get() > 0.0
        && config.align_stator_angle.get().is_finite()
        && config.sweep_electrical_velocity.get().is_finite()
        && config.sweep_electrical_velocity.get().abs() > 0.0
        && config.sweep_electrical_cycles.is_finite()
        && config.sweep_electrical_cycles > 0.0
        && config.settle_velocity_threshold.get().is_finite()
        && config.settle_velocity_threshold.get() >= 0.0
        && config.initial_settle_time_seconds.is_finite()
        && config.initial_settle_time_seconds > 0.0
        && config.final_settle_time_seconds.is_finite()
        && config.final_settle_time_seconds > 0.0
        && config.pole_pair_rounding_tolerance.is_finite()
        && config.pole_pair_rounding_tolerance >= 0.0
        && config.max_pole_pairs > 0
        && config.timeout_seconds.is_finite()
        && config.timeout_seconds
            > config.initial_settle_time_seconds + config.final_settle_time_seconds
}

fn validate_input(input: PolePairsAndOffsetCalibrationInput) -> bool {
    input.mechanical_angle.get().is_finite()
        && input.mechanical_velocity.get().is_finite()
        && input.dt_seconds.is_finite()
        && input.dt_seconds > 0.0
}

impl PolePairsAndOffsetCalibrationConfig {
    fn sweep_direction(&self) -> f32 {
        self.sweep_electrical_velocity.get().signum()
    }
}

#[cfg(test)]
mod tests {
    use super::{
        PolePairsAndOffsetCalibrationConfig, PolePairsAndOffsetCalibrationInput,
        PolePairsAndOffsetCalibrationState, PolePairsAndOffsetCalibrator,
    };
    use fluxkit_math::{
        ElectricalAngle, MechanicalAngle, angle::wrap_0_2pi, scalar::TAU, units::RadPerSec,
    };

    #[test]
    fn synthetic_following_motion_recovers_pole_pairs_and_offset() {
        let config = PolePairsAndOffsetCalibrationConfig {
            align_voltage_mag: fluxkit_math::Volts::new(1.0),
            align_stator_angle: ElectricalAngle::new(0.0),
            sweep_electrical_velocity: RadPerSec::new(10.0),
            sweep_electrical_cycles: 2.0,
            settle_velocity_threshold: RadPerSec::new(0.01),
            initial_settle_time_seconds: 0.01,
            final_settle_time_seconds: 0.01,
            pole_pair_rounding_tolerance: 0.2,
            max_pole_pairs: 32,
            timeout_seconds: 2.0,
        };
        let mut calibrator = PolePairsAndOffsetCalibrator::new(config).unwrap();
        let pole_pairs = 7.0_f32;
        let encoder_bias = 0.2_f32;
        let dt = 0.001;
        let mut commanded_electrical_angle = 0.0_f32;

        for step in 0..3_000 {
            let electrical_velocity = if step < 20 {
                0.0
            } else if step < 20 + ((config.sweep_electrical_cycles * TAU) / 10.0 / dt) as usize {
                10.0
            } else {
                0.0
            };
            commanded_electrical_angle += electrical_velocity * dt;
            let mechanical_angle = MechanicalAngle::new(wrap_0_2pi(
                commanded_electrical_angle / pole_pairs + encoder_bias,
            ));
            let velocity = RadPerSec::new(electrical_velocity / pole_pairs);
            let _ = calibrator.tick(PolePairsAndOffsetCalibrationInput {
                mechanical_angle,
                mechanical_velocity: velocity,
                dt_seconds: dt,
            });

            if calibrator.state() == PolePairsAndOffsetCalibrationState::Complete {
                break;
            }
        }

        let result = calibrator.result().unwrap();
        assert_eq!(result.pole_pairs, 7);
    }
}
