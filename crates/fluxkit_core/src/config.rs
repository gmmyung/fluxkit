//! Runtime-tunable controller configuration.

use fluxkit_math::units::{Amps, Henries, Ohms, RadPerSec, Volts};

/// Runtime tuning for the controller stack.
///
/// Each controller cycle runs:
///
/// - `d/q` current PI
/// - optional model-based current feedforward
/// - torque-to-current mapping
/// - velocity PI
/// - position PI followed immediately by velocity PI when `Position` mode is active
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
    /// Symmetric clamp for the current-reference derivative used by feedforward, in `A/s`.
    ///
    /// This limits only the `L * d(i_ref)/dt` feedforward term so that large
    /// current-command steps do not inject an excessive one-tick voltage kick.
    pub max_current_ref_derivative_amps_per_sec: f32,
    /// Enables model-based current-loop feedforward when `true`.
    ///
    /// The feedforward term uses motor resistance, inductances, pole pairs,
    /// current-reference derivative, mechanical velocity, and optional flux linkage.
    pub enable_current_feedforward: bool,
    /// Optional flux-weakening policy for generated current targets.
    ///
    /// Flux weakening is applied to `Torque`, `Mit`, `Velocity`, and
    /// `Position` modes. Direct `Current` mode remains explicit and bypasses
    /// this policy.
    pub flux_weakening: FluxWeakeningConfig,
}

/// Flux-weakening policy for high-speed voltage headroom management.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct FluxWeakeningConfig {
    /// Enables flux weakening when `true`.
    pub enabled: bool,
    /// Requested voltage-vector utilization as a fraction of the active voltage limit.
    pub voltage_utilization_target: f32,
    /// Integral bandwidth used to adapt negative `d`-axis current.
    pub bandwidth: RadPerSec,
    /// Maximum negative `d`-axis current that flux weakening may add.
    pub max_negative_id: Amps,
}

impl FluxWeakeningConfig {
    /// Returns a disabled flux-weakening policy.
    #[inline]
    pub const fn disabled() -> Self {
        Self {
            enabled: false,
            voltage_utilization_target: 0.95,
            bandwidth: RadPerSec::ZERO,
            max_negative_id: Amps::ZERO,
        }
    }

    /// Returns an enabled flux-weakening policy.
    #[inline]
    pub const fn enabled(
        voltage_utilization_target: f32,
        bandwidth: RadPerSec,
        max_negative_id: Amps,
    ) -> Self {
        Self {
            enabled: true,
            voltage_utilization_target,
            bandwidth,
            max_negative_id,
        }
    }
}

impl CurrentLoopConfig {
    /// Starts building a current-loop config from a chosen current-loop
    /// bandwidth and measured motor electrical parameters.
    ///
    /// The builder computes symmetric `d/q` current PI gains as:
    ///
    /// - `Kp = omega_bw * L`
    /// - `Ki = omega_bw * R`
    ///
    /// where:
    ///
    /// - `omega_bw` is the chosen current-loop bandwidth
    /// - `L` is the measured phase inductance
    /// - `R` is the measured phase resistance
    #[inline]
    pub fn builder(
        current_loop_bandwidth: RadPerSec,
        phase_inductance_h: Henries,
        phase_resistance_ohm_ref: Ohms,
    ) -> CurrentLoopConfigBuilder {
        CurrentLoopConfigBuilder::new(
            current_loop_bandwidth,
            phase_inductance_h,
            phase_resistance_ohm_ref,
        )
    }
}

/// Builder for [`CurrentLoopConfig`] that sizes the symmetric `d/q` current PI
/// gains from a chosen bandwidth and measured electrical parameters.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct CurrentLoopConfigBuilder {
    config: CurrentLoopConfig,
}

impl CurrentLoopConfigBuilder {
    /// Creates a builder with `d/q` current PI gains computed from:
    ///
    /// - `Kp = omega_bw * L`
    /// - `Ki = omega_bw * R`
    #[inline]
    pub fn new(
        current_loop_bandwidth: RadPerSec,
        phase_inductance_h: Henries,
        phase_resistance_ohm_ref: Ohms,
    ) -> Self {
        let bandwidth = current_loop_bandwidth.get();
        let kp = bandwidth * phase_inductance_h.get();
        let ki = bandwidth * phase_resistance_ohm_ref.get();
        Self {
            config: CurrentLoopConfig {
                kp_d: kp,
                ki_d: ki,
                kp_q: kp,
                ki_q: ki,
                velocity_kp: 0.0,
                velocity_ki: 0.0,
                position_kp: 0.0,
                position_ki: 0.0,
                max_voltage_mag: Volts::new(0.0),
                id_ref_default: Amps::ZERO,
                max_id_target: Amps::ZERO,
                max_iq_target: Amps::ZERO,
                max_velocity_target: RadPerSec::ZERO,
                max_current_ref_derivative_amps_per_sec: 0.0,
                enable_current_feedforward: false,
                flux_weakening: FluxWeakeningConfig::disabled(),
            },
        }
    }

    /// Sets the medium-rate velocity-loop PI gains.
    #[inline]
    pub fn velocity_gains(mut self, kp: f32, ki: f32) -> Self {
        self.config.velocity_kp = kp;
        self.config.velocity_ki = ki;
        self
    }

    /// Sets the medium-rate position-loop PI gains.
    #[inline]
    pub fn position_gains(mut self, kp: f32, ki: f32) -> Self {
        self.config.position_kp = kp;
        self.config.position_ki = ki;
        self
    }

    /// Sets the circular voltage magnitude clamp for the current loop.
    #[inline]
    pub fn max_voltage_mag(mut self, max_voltage_mag: Volts) -> Self {
        self.config.max_voltage_mag = max_voltage_mag;
        self
    }

    /// Sets the default `d`-axis current target.
    #[inline]
    pub fn id_ref_default(mut self, id_ref_default: Amps) -> Self {
        self.config.id_ref_default = id_ref_default;
        self
    }

    /// Sets the symmetric `d`-axis current-command limit.
    #[inline]
    pub fn max_id_target(mut self, max_id_target: Amps) -> Self {
        self.config.max_id_target = max_id_target;
        self
    }

    /// Sets the symmetric `q`-axis current-command limit.
    #[inline]
    pub fn max_iq_target(mut self, max_iq_target: Amps) -> Self {
        self.config.max_iq_target = max_iq_target;
        self
    }

    /// Sets the symmetric output mechanical velocity target limit.
    #[inline]
    pub fn max_velocity_target(mut self, max_velocity_target: RadPerSec) -> Self {
        self.config.max_velocity_target = max_velocity_target;
        self
    }

    /// Sets the clamp for the current-reference derivative feedforward term.
    #[inline]
    pub fn max_current_ref_derivative_amps_per_sec(
        mut self,
        max_current_ref_derivative_amps_per_sec: f32,
    ) -> Self {
        self.config.max_current_ref_derivative_amps_per_sec =
            max_current_ref_derivative_amps_per_sec;
        self
    }

    /// Enables or disables model-based current-loop feedforward.
    #[inline]
    pub fn current_feedforward(mut self, enable_current_feedforward: bool) -> Self {
        self.config.enable_current_feedforward = enable_current_feedforward;
        self
    }

    /// Sets the flux-weakening policy.
    #[inline]
    pub fn flux_weakening(mut self, flux_weakening: FluxWeakeningConfig) -> Self {
        self.config.flux_weakening = flux_weakening;
        self
    }

    /// Returns the fully built config.
    #[inline]
    pub const fn build(self) -> CurrentLoopConfig {
        self.config
    }
}

#[cfg(test)]
mod tests {
    use super::{CurrentLoopConfig, CurrentLoopConfigBuilder, FluxWeakeningConfig};
    use fluxkit_math::units::{Amps, Henries, Ohms, RadPerSec, Volts};

    #[test]
    fn builder_sizes_symmetric_current_pi_from_bandwidth() {
        let config = CurrentLoopConfig::builder(
            RadPerSec::new(2_000.0),
            Henries::new(30.0e-6),
            Ohms::new(0.12),
        )
        .velocity_gains(0.2, 8.0)
        .position_gains(12.0, 0.0)
        .max_voltage_mag(Volts::new(12.0))
        .id_ref_default(Amps::ZERO)
        .max_id_target(Amps::new(5.0))
        .max_iq_target(Amps::new(8.0))
        .max_velocity_target(RadPerSec::new(120.0))
        .max_current_ref_derivative_amps_per_sec(10_000.0)
        .current_feedforward(true)
        .flux_weakening(FluxWeakeningConfig::enabled(
            0.9,
            RadPerSec::new(500.0),
            Amps::new(3.0),
        ))
        .build();

        assert!((config.kp_d - 0.06).abs() < 1.0e-6);
        assert!((config.ki_d - 240.0).abs() < 1.0e-6);
        assert_eq!(config.kp_q, config.kp_d);
        assert_eq!(config.ki_q, config.ki_d);
        assert_eq!(config.max_iq_target, Amps::new(8.0));
        assert!(config.enable_current_feedforward);
        assert_eq!(config.flux_weakening.max_negative_id, Amps::new(3.0));
    }

    #[test]
    fn builder_new_matches_current_loop_config_builder_constructor() {
        let via_config = CurrentLoopConfig::builder(
            RadPerSec::new(2_000.0),
            Henries::new(30.0e-6),
            Ohms::new(0.12),
        )
        .build();
        let via_builder = CurrentLoopConfigBuilder::new(
            RadPerSec::new(2_000.0),
            Henries::new(30.0e-6),
            Ohms::new(0.12),
        )
        .build();

        assert_eq!(via_config, via_builder);
    }
}
