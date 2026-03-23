//! Flux-linkage calibration via controlled open-loop electrical spin.
//!
//! The procedure first applies a fixed alignment vector, then commands a
//! constant-magnitude rotating `alpha/beta` voltage vector. Once the rotor is
//! spinning fast enough, it transforms measured currents into the rotor `dq`
//! frame using the already calibrated pole-pair count and electrical offset,
//! and estimates the permanent-magnet flux linkage from the `q`-axis voltage
//! equation:
//!
//! `psi ~= (v_q - R i_q - L di_q/dt) / omega_e - L i_d`

use fluxkit_math::{
    AlphaBeta, ElectricalAngle, MechanicalAngle,
    angle::{mechanical_to_electrical, wrap_pm_pi},
    frame::Abc,
    transforms::{clarke, park},
    trig::sin_cos,
    units::{Amps, Ohms, RadPerSec, Volts, Webers},
};

use super::error::CalibrationError;

/// Static configuration for flux-linkage calibration.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct FluxLinkageCalibrationConfig {
    /// Previously calibrated phase resistance.
    pub phase_resistance_ohm: Ohms,
    /// Previously calibrated common phase inductance.
    pub phase_inductance_h: fluxkit_math::units::Henries,
    /// Previously calibrated pole-pair count.
    pub pole_pairs: u8,
    /// Previously calibrated electrical zero offset.
    pub electrical_angle_offset: ElectricalAngle,
    /// Fixed alignment voltage before the spin begins.
    pub align_voltage_mag: Volts,
    /// Voltage magnitude of the rotating stator vector during the spin.
    pub spin_voltage_mag: Volts,
    /// Initial stator-frame alignment angle.
    pub align_stator_angle: ElectricalAngle,
    /// Commanded electrical angular velocity during the spin.
    pub spin_electrical_velocity: RadPerSec,
    /// Maximum mechanical speed considered "settled" during the initial align.
    pub initial_settle_velocity_threshold: RadPerSec,
    /// Required initial settle time before switching into the spin phase.
    pub initial_settle_time_seconds: f32,
    /// Minimum electrical speed required to accept flux-linkage samples.
    pub min_electrical_velocity: RadPerSec,
    /// Sampling window for averaging the flux-linkage estimate.
    pub sample_time_seconds: f32,
    /// Absolute timeout for the whole procedure.
    pub timeout_seconds: f32,
}

impl FluxLinkageCalibrationConfig {
    /// Returns conservative defaults suitable for simulator-backed tests.
    pub const fn default_for_spin() -> Self {
        Self {
            phase_resistance_ohm: Ohms::new(0.1),
            phase_inductance_h: fluxkit_math::units::Henries::new(30.0e-6),
            pole_pairs: 7,
            electrical_angle_offset: ElectricalAngle::new(0.0),
            align_voltage_mag: Volts::new(2.0),
            spin_voltage_mag: Volts::new(2.0),
            align_stator_angle: ElectricalAngle::new(0.0),
            spin_electrical_velocity: RadPerSec::new(20.0),
            initial_settle_velocity_threshold: RadPerSec::new(0.05),
            initial_settle_time_seconds: 0.05,
            min_electrical_velocity: RadPerSec::new(8.0),
            sample_time_seconds: 0.05,
            timeout_seconds: 3.0,
        }
    }
}

/// One synchronous sample frame for flux-linkage calibration.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct FluxLinkageCalibrationInput {
    /// Measured three-phase currents.
    pub phase_currents: Abc<Amps>,
    /// Measured wrapped mechanical angle from the encoder path.
    pub mechanical_angle: MechanicalAngle,
    /// Measured mechanical rotor velocity.
    pub mechanical_velocity: RadPerSec,
    /// Time since the previous calibration tick.
    pub dt_seconds: f32,
}

/// Result of a completed flux-linkage calibration.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct FluxLinkageCalibrationResult {
    /// Calibrated permanent-magnet flux linkage.
    pub flux_linkage_weber: Webers,
}

/// Compact state of the flux-linkage calibration procedure.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum FluxLinkageCalibrationState {
    /// The rotor is being magnetically aligned before the spin starts.
    Aligning,
    /// The procedure is spinning the stator field but not yet sampling.
    Spinning,
    /// The procedure is averaging flux-linkage estimates.
    Sampling,
    /// The procedure completed successfully.
    Complete,
    /// The procedure failed.
    Failed(CalibrationError),
}

/// Pure state machine for flux-linkage calibration.
#[derive(Clone, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct FluxLinkageCalibrator {
    config: FluxLinkageCalibrationConfig,
    elapsed_seconds: f32,
    settled_seconds: f32,
    spinning: bool,
    command_angle: ElectricalAngle,
    last_command_angle: Option<ElectricalAngle>,
    sample_seconds: f32,
    sample_count: u32,
    flux_integral: f32,
    last_sample_iq: Option<f32>,
    result: Option<FluxLinkageCalibrationResult>,
    error: Option<CalibrationError>,
}

impl FluxLinkageCalibrator {
    /// Creates a new flux-linkage calibrator.
    pub fn new(config: FluxLinkageCalibrationConfig) -> Result<Self, CalibrationError> {
        if !validate_config(config) {
            return Err(CalibrationError::InvalidConfiguration);
        }

        Ok(Self {
            config,
            elapsed_seconds: 0.0,
            settled_seconds: 0.0,
            spinning: false,
            command_angle: config.align_stator_angle,
            last_command_angle: None,
            sample_seconds: 0.0,
            sample_count: 0,
            flux_integral: 0.0,
            last_sample_iq: None,
            result: None,
            error: None,
        })
    }

    /// Returns the current calibration state.
    #[inline]
    pub const fn state(&self) -> FluxLinkageCalibrationState {
        if let Some(error) = self.error {
            FluxLinkageCalibrationState::Failed(error)
        } else if self.result.is_some() {
            FluxLinkageCalibrationState::Complete
        } else if self.sample_seconds > 0.0 {
            FluxLinkageCalibrationState::Sampling
        } else if self.spinning {
            FluxLinkageCalibrationState::Spinning
        } else {
            FluxLinkageCalibrationState::Aligning
        }
    }

    /// Returns the finished result when calibration has succeeded.
    #[inline]
    pub const fn result(&self) -> Option<FluxLinkageCalibrationResult> {
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

        if self.spinning {
            voltage_vector(self.command_angle, self.config.spin_voltage_mag.get())
        } else {
            voltage_vector(
                self.config.align_stator_angle,
                self.config.align_voltage_mag.get(),
            )
        }
    }

    /// Advances the calibration procedure by one sample.
    pub fn tick(&mut self, input: FluxLinkageCalibrationInput) -> AlphaBeta<Volts> {
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

        if !self.spinning {
            if input.mechanical_velocity.get().abs()
                > self.config.initial_settle_velocity_threshold.get()
            {
                self.settled_seconds = 0.0;
            } else {
                self.settled_seconds += input.dt_seconds;
            }

            if self.settled_seconds < self.config.initial_settle_time_seconds {
                return self.commanded_voltage_alpha_beta();
            }

            self.spinning = true;
            self.command_angle = self.config.align_stator_angle;
            self.last_command_angle = None;
        } else {
            self.process_spin_sample(input);
        }

        self.next_spin_command(input.dt_seconds)
    }

    fn process_spin_sample(&mut self, input: FluxLinkageCalibrationInput) {
        let Some(last_command_angle) = self.last_command_angle else {
            return;
        };

        let electrical_angle = rotor_electrical_angle(
            input.mechanical_angle,
            self.config.pole_pairs as u32,
            self.config.electrical_angle_offset,
        );
        let current_ab = clarke(input.phase_currents.map(|current| current.get()));
        let current_dq = park(current_ab, electrical_angle.get());
        let applied_ab = voltage_vector(last_command_angle, self.config.spin_voltage_mag.get());
        let applied_dq = park(
            applied_ab.map(|voltage| voltage.get()),
            electrical_angle.get(),
        );
        let omega_e = input.mechanical_velocity.get() * self.config.pole_pairs as f32;

        if !omega_e.is_finite() || omega_e.abs() < self.config.min_electrical_velocity.get() {
            self.reset_sampling_state();
            return;
        }

        let iq = current_dq.q;
        let Some(last_iq) = self.last_sample_iq else {
            self.last_sample_iq = Some(iq);
            return;
        };
        self.last_sample_iq = Some(iq);

        let diq_dt = (iq - last_iq) / input.dt_seconds;
        let inductance = self.config.phase_inductance_h.get();
        let estimate = (applied_dq.q
            - self.config.phase_resistance_ohm.get() * current_dq.q
            - inductance * diq_dt)
            / omega_e
            - inductance * current_dq.d;

        if !estimate.is_finite() || estimate <= 0.0 {
            self.error = Some(CalibrationError::IndeterminateEstimate);
            return;
        }

        self.flux_integral += estimate * input.dt_seconds;
        self.sample_seconds += input.dt_seconds;
        self.sample_count += 1;

        if self.sample_seconds >= self.config.sample_time_seconds {
            let mean_flux = self.flux_integral / self.sample_seconds;
            if !mean_flux.is_finite() || mean_flux <= 0.0 || self.sample_count == 0 {
                self.error = Some(CalibrationError::IndeterminateEstimate);
                return;
            }

            self.result = Some(FluxLinkageCalibrationResult {
                flux_linkage_weber: Webers::new(mean_flux),
            });
        }
    }

    fn next_spin_command(&mut self, dt_seconds: f32) -> AlphaBeta<Volts> {
        let angle = self.command_angle;
        self.last_command_angle = Some(angle);
        self.command_angle = ElectricalAngle::new(wrap_pm_pi(
            angle.get() + self.config.spin_electrical_velocity.get() * dt_seconds,
        ));
        voltage_vector(angle, self.config.spin_voltage_mag.get())
    }

    fn reset_sampling_state(&mut self) {
        self.sample_seconds = 0.0;
        self.sample_count = 0;
        self.flux_integral = 0.0;
        self.last_sample_iq = None;
    }
}

fn rotor_electrical_angle(
    mechanical_angle: MechanicalAngle,
    pole_pairs: u32,
    electrical_angle_offset: ElectricalAngle,
) -> ElectricalAngle {
    ElectricalAngle::new(
        mechanical_to_electrical(mechanical_angle, pole_pairs).get()
            + electrical_angle_offset.get(),
    )
    .wrapped_pm_pi()
}

fn voltage_vector(angle: ElectricalAngle, magnitude: f32) -> AlphaBeta<Volts> {
    let (s, c) = sin_cos(angle.get());
    AlphaBeta::new(Volts::new(magnitude * c), Volts::new(magnitude * s))
}

fn validate_config(config: FluxLinkageCalibrationConfig) -> bool {
    config.phase_resistance_ohm.get().is_finite()
        && config.phase_resistance_ohm.get() > 0.0
        && config.phase_inductance_h.get().is_finite()
        && config.phase_inductance_h.get() > 0.0
        && config.pole_pairs > 0
        && config.electrical_angle_offset.get().is_finite()
        && config.align_voltage_mag.get().is_finite()
        && config.align_voltage_mag.get() > 0.0
        && config.spin_voltage_mag.get().is_finite()
        && config.spin_voltage_mag.get() > 0.0
        && config.align_stator_angle.get().is_finite()
        && config.spin_electrical_velocity.get().is_finite()
        && config.spin_electrical_velocity.get().abs() > 0.0
        && config.initial_settle_velocity_threshold.get().is_finite()
        && config.initial_settle_velocity_threshold.get() >= 0.0
        && config.initial_settle_time_seconds.is_finite()
        && config.initial_settle_time_seconds > 0.0
        && config.min_electrical_velocity.get().is_finite()
        && config.min_electrical_velocity.get() > 0.0
        && config.sample_time_seconds.is_finite()
        && config.sample_time_seconds > 0.0
        && config.timeout_seconds.is_finite()
        && config.timeout_seconds
            > (config.initial_settle_time_seconds + config.sample_time_seconds)
}

fn validate_input(input: FluxLinkageCalibrationInput) -> bool {
    input.phase_currents.a.get().is_finite()
        && input.phase_currents.b.get().is_finite()
        && input.phase_currents.c.get().is_finite()
        && input.mechanical_angle.get().is_finite()
        && input.mechanical_velocity.get().is_finite()
        && input.dt_seconds.is_finite()
        && input.dt_seconds > 0.0
}

#[cfg(test)]
mod tests {
    use fluxkit_math::{
        ElectricalAngle,
        angle::MechanicalAngle,
        frame::Abc,
        units::{Amps, Henries, Ohms, RadPerSec},
    };

    use super::{
        CalibrationError, FluxLinkageCalibrationConfig, FluxLinkageCalibrationInput,
        FluxLinkageCalibrationState, FluxLinkageCalibrator,
    };

    #[test]
    fn times_out_when_rotor_never_reaches_spin_speed() {
        let mut calibrator = FluxLinkageCalibrator::new(FluxLinkageCalibrationConfig {
            phase_resistance_ohm: Ohms::new(0.12),
            phase_inductance_h: Henries::new(30.0e-6),
            pole_pairs: 7,
            electrical_angle_offset: ElectricalAngle::new(0.0),
            initial_settle_velocity_threshold: RadPerSec::new(0.01),
            initial_settle_time_seconds: 0.01,
            min_electrical_velocity: RadPerSec::new(10.0),
            sample_time_seconds: 0.02,
            timeout_seconds: 0.05,
            ..FluxLinkageCalibrationConfig::default_for_spin()
        })
        .unwrap();

        for _ in 0..20 {
            let _ = calibrator.tick(FluxLinkageCalibrationInput {
                phase_currents: Abc::new(Amps::ZERO, Amps::ZERO, Amps::ZERO),
                mechanical_angle: MechanicalAngle::new(0.0),
                mechanical_velocity: RadPerSec::ZERO,
                dt_seconds: 0.005,
            });
        }

        assert_eq!(
            calibrator.state(),
            FluxLinkageCalibrationState::Failed(CalibrationError::Timeout)
        );
    }
}
