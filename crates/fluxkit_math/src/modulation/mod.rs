//! PWM modulation math.

pub mod sine_pwm;
pub mod svpwm;

use crate::frame::{Abc, AlphaBeta};
use crate::scalar::FRAC_1_SQRT_3;
use crate::units::{Duty, Volts};

/// Three normalized phase duties in `[0.0, 1.0]`, represented in the generic
/// three-phase frame container.
pub type PhaseDuty = Abc<Duty>;

/// Common modulation output shared across modulation strategies.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct ModulationOutput {
    /// Normalized phase duties.
    pub duty: PhaseDuty,
    /// `true` when the requested voltage could not be represented without
    /// clipping under the selected modulation strategy.
    pub saturated: bool,
}

/// Common interface for upper layers that want to swap modulation strategies.
pub trait Modulator {
    /// Returns the ideal no-clipping circular voltage limit for this modulator.
    fn linear_limit(&self, vbus: Volts) -> Volts;

    /// Converts a stationary-frame voltage request into phase duties.
    fn modulate(&self, v_ab: AlphaBeta<f32>, vbus: Volts) -> ModulationOutput;
}

/// Sinusoidal PWM modulator adapter.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct SinePwm;

/// Linear space-vector PWM modulator adapter.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Svpwm;

/// Center-aligned 50% duty on every phase.
#[inline]
const fn centered_duty() -> PhaseDuty {
    Abc::new(Duty::new(0.5), Duty::new(0.5), Duty::new(0.5))
}

/// Returns the ideal peak `alpha-beta` or `dq` voltage magnitude available with
/// sinusoidal PWM before clipping.
///
/// Non-positive or non-finite inputs return zero.
#[inline]
pub fn spwm_linear_limit(vbus: Volts) -> Volts {
    let vbus = vbus.get();
    if !vbus.is_finite() || vbus <= 0.0 {
        Volts::ZERO
    } else {
        Volts::new(0.5 * vbus)
    }
}

/// Returns the ideal peak `alpha-beta` or `dq` voltage magnitude available with
/// linear SVPWM before clipping.
///
/// This matches the current `svpwm()` implementation, whose linear circular
/// limit is `vbus / sqrt(3)`. Non-positive or non-finite inputs return zero.
#[inline]
pub fn svpwm_linear_limit(vbus: Volts) -> Volts {
    let vbus = vbus.get();
    if !vbus.is_finite() || vbus <= 0.0 {
        Volts::ZERO
    } else {
        Volts::new(vbus * FRAC_1_SQRT_3)
    }
}

/// Returns the remaining symmetric `q`-axis voltage headroom for a requested
/// `d`-axis voltage under a circular voltage limit.
///
/// This computes `sqrt(v_limit^2 - vd^2)` when `|vd| < v_limit`, otherwise
/// returns zero. Non-finite or non-positive `v_limit` inputs return zero.
#[inline]
pub fn dq_q_limit(vd: Volts, v_limit: Volts) -> Volts {
    let vd = vd.get().abs();
    let v_limit = v_limit.get();

    if !vd.is_finite() || !v_limit.is_finite() || v_limit <= 0.0 || vd >= v_limit {
        return Volts::ZERO;
    }

    Volts::new(libm::sqrtf(v_limit * v_limit - vd * vd))
}

impl Modulator for SinePwm {
    #[inline]
    fn linear_limit(&self, vbus: Volts) -> Volts {
        spwm_linear_limit(vbus)
    }

    #[inline]
    fn modulate(&self, v_ab: AlphaBeta<f32>, vbus: Volts) -> ModulationOutput {
        sine_pwm::sine_pwm_result(v_ab, vbus.get())
    }
}

impl Modulator for Svpwm {
    #[inline]
    fn linear_limit(&self, vbus: Volts) -> Volts {
        svpwm_linear_limit(vbus)
    }

    #[inline]
    fn modulate(&self, v_ab: AlphaBeta<f32>, vbus: Volts) -> ModulationOutput {
        let result = svpwm::svpwm(v_ab, vbus.get());
        ModulationOutput {
            duty: result.duty,
            saturated: result.saturated,
        }
    }
}

pub use sine_pwm::sine_pwm;
pub use svpwm::{SvpwmResult, svpwm};

#[cfg(test)]
mod tests {
    use super::{Modulator, SinePwm, Svpwm, dq_q_limit, spwm_linear_limit, svpwm_linear_limit};
    use crate::frame::AlphaBeta;
    use crate::units::Volts;

    fn approx_eq(a: f32, b: f32) {
        assert!((a - b).abs() < 1.0e-5, "{a} != {b}");
    }

    #[test]
    fn modulation_limits_match_expected_ideal_values() {
        approx_eq(spwm_linear_limit(Volts::new(24.0)).get(), 12.0);
        approx_eq(svpwm_linear_limit(Volts::new(24.0)).get(), 13.856_406);
    }

    #[test]
    fn dq_q_limit_respects_circular_headroom() {
        let q_limit = dq_q_limit(Volts::new(6.0), Volts::new(10.0));
        approx_eq(q_limit.get(), 8.0);
        approx_eq(dq_q_limit(Volts::new(10.0), Volts::new(10.0)).get(), 0.0);
    }

    #[test]
    fn invalid_limits_return_zero() {
        assert_eq!(spwm_linear_limit(Volts::new(0.0)), Volts::ZERO);
        assert_eq!(svpwm_linear_limit(Volts::new(-1.0)), Volts::ZERO);
        assert_eq!(
            dq_q_limit(Volts::new(f32::NAN), Volts::new(10.0)),
            Volts::ZERO
        );
    }

    #[test]
    fn trait_adapters_expose_strategy_specific_limits() {
        approx_eq(SinePwm.linear_limit(Volts::new(24.0)).get(), 12.0);
        approx_eq(Svpwm.linear_limit(Volts::new(24.0)).get(), 13.856_406);
    }

    #[test]
    fn trait_adapters_make_modulators_swappable() {
        let v_ab = AlphaBeta::new(12.5, 0.0);

        assert!(SinePwm.modulate(v_ab, Volts::new(24.0)).saturated);
        assert!(!Svpwm.modulate(v_ab, Volts::new(24.0)).saturated);
    }
}
