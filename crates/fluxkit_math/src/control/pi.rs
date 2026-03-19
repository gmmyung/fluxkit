//! Deterministic PI controller with explicit state.

use crate::scalar::clamp;

/// PI controller configuration.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct PiConfig {
    /// Proportional gain.
    pub kp: f32,
    /// Integral gain.
    pub ki: f32,
    /// Minimum output.
    pub out_min: f32,
    /// Maximum output.
    pub out_max: f32,
}

/// PI controller dynamic state.
#[derive(Clone, Copy, Debug, PartialEq, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct PiState {
    /// Stored integrator contribution.
    pub integrator: f32,
}

/// PI controller with explicit configuration and state.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct PiController {
    /// Static configuration.
    pub cfg: PiConfig,
    /// Internal controller state.
    pub state: PiState,
}

impl PiController {
    /// Creates a PI controller with zeroed state.
    #[inline]
    pub const fn new(cfg: PiConfig) -> Self {
        Self {
            cfg,
            state: PiState { integrator: 0.0 },
        }
    }

    /// Resets the integrator to zero.
    #[inline]
    pub fn reset(&mut self) {
        self.state.integrator = 0.0;
    }

    /// Sets the integrator explicitly.
    #[inline]
    pub fn set_integrator(&mut self, integrator: f32) {
        self.state.integrator = clamp(integrator, self.cfg.out_min, self.cfg.out_max);
    }

    /// Updates the controller without feedforward.
    #[inline]
    pub fn update(&mut self, error: f32, dt: f32) -> f32 {
        self.update_with_feedforward(error, 0.0, dt)
    }

    /// Updates the controller with an additive feedforward term.
    ///
    /// Anti-windup policy:
    /// the integrator is advanced explicitly and then clamped against the
    /// output headroom left after the proportional and feedforward terms.
    #[inline]
    pub fn update_with_feedforward(&mut self, error: f32, ff: f32, dt: f32) -> f32 {
        debug_assert!(
            self.cfg.out_min <= self.cfg.out_max,
            "invalid PI output limits"
        );

        let p = self.cfg.kp * error;
        let base = ff + p;
        let integrated = if dt > 0.0 && dt.is_finite() {
            self.state.integrator + self.cfg.ki * error * dt
        } else {
            self.state.integrator
        };

        let integrator_min = self.cfg.out_min - base;
        let integrator_max = self.cfg.out_max - base;
        self.state.integrator = clamp(integrated, integrator_min, integrator_max);

        clamp(
            base + self.state.integrator,
            self.cfg.out_min,
            self.cfg.out_max,
        )
    }
}

#[cfg(test)]
mod tests {
    use super::{PiConfig, PiController};

    fn approx_eq(a: f32, b: f32) {
        assert!((a - b).abs() < 1.0e-6, "{a} != {b}");
    }

    #[test]
    fn pi_accumulates_integral_state() {
        let mut pi = PiController::new(PiConfig {
            kp: 2.0,
            ki: 10.0,
            out_min: -5.0,
            out_max: 5.0,
        });

        approx_eq(pi.update(0.5, 0.1), 1.5);
        approx_eq(pi.state.integrator, 0.5);
        approx_eq(pi.update(0.5, 0.1), 2.0);
        approx_eq(pi.state.integrator, 1.0);
    }

    #[test]
    fn pi_saturates_without_winding_up_past_limits() {
        let mut pi = PiController::new(PiConfig {
            kp: 4.0,
            ki: 20.0,
            out_min: -2.0,
            out_max: 2.0,
        });

        let y = pi.update(1.0, 0.1);
        approx_eq(y, 2.0);
        approx_eq(pi.state.integrator, -2.0);
    }

    #[test]
    fn feedforward_uses_remaining_headroom() {
        let mut pi = PiController::new(PiConfig {
            kp: 1.0,
            ki: 10.0,
            out_min: -3.0,
            out_max: 3.0,
        });

        let y = pi.update_with_feedforward(0.5, 2.5, 0.1);
        approx_eq(y, 3.0);
        approx_eq(pi.state.integrator, 0.0);
    }

    #[test]
    fn reset_and_set_integrator_are_explicit() {
        let mut pi = PiController::new(PiConfig {
            kp: 0.0,
            ki: 0.0,
            out_min: -1.0,
            out_max: 1.0,
        });

        pi.set_integrator(2.0);
        approx_eq(pi.state.integrator, 1.0);
        pi.reset();
        approx_eq(pi.state.integrator, 0.0);
    }
}
