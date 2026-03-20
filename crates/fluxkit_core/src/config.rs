//! Runtime-tunable controller configuration.

use fluxkit_math::units::{Amps, Volts};

/// Runtime tuning for the current loop.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct CurrentLoopConfig {
    /// Proportional gain for the `d`-axis PI controller.
    pub kp_d: f32,
    /// Integral gain for the `d`-axis PI controller.
    pub ki_d: f32,
    /// Proportional gain for the `q`-axis PI controller.
    pub kp_q: f32,
    /// Integral gain for the `q`-axis PI controller.
    pub ki_q: f32,
    /// Circular voltage magnitude clamp for the current loop.
    pub max_voltage_mag: Volts,
    /// Default `d`-axis current target.
    pub id_ref_default: Amps,
    /// Symmetric `d`-axis current-command limit.
    pub max_id_target: Amps,
    /// Symmetric `q`-axis current-command limit.
    pub max_iq_target: Amps,
}
