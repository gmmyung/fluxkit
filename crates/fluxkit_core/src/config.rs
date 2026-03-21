//! Runtime-tunable controller configuration.

use fluxkit_math::units::{Amps, RadPerSec, Volts};

/// Runtime tuning for the controller stack.
///
/// Loop ownership is split as follows:
///
/// - `fast_tick()`
///   - `d/q` current PI
///   - optional model-based current feedforward
/// - `medium_tick()`
///   - torque-to-current mapping
///   - velocity PI
///   - position PI followed immediately by velocity PI when `Position` mode is active
///
/// `slow_tick()` is currently only a reserved hook and does not own a control loop.
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
    /// Proportional gain for the medium-rate velocity loop.
    pub velocity_kp: f32,
    /// Integral gain for the medium-rate velocity loop.
    pub velocity_ki: f32,
    /// Proportional gain for the medium-rate position loop.
    pub position_kp: f32,
    /// Integral gain for the medium-rate position loop.
    pub position_ki: f32,
    /// Circular voltage magnitude clamp for the current loop.
    pub max_voltage_mag: Volts,
    /// Default `d`-axis current target.
    pub id_ref_default: Amps,
    /// Symmetric `d`-axis current-command limit.
    pub max_id_target: Amps,
    /// Symmetric `q`-axis current-command limit.
    pub max_iq_target: Amps,
    /// Symmetric mechanical velocity-command limit used by the outer loops.
    pub max_velocity_target: RadPerSec,
    /// Enables model-based current-loop feedforward when `true`.
    ///
    /// The feedforward term uses motor resistance, inductances, pole pairs,
    /// current-reference derivative, mechanical velocity, and optional flux linkage.
    pub enable_current_feedforward: bool,
}
