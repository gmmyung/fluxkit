//! Linear space-vector PWM.
//!
//! The implementation uses common-mode injection by subtracting the midpoint
//! of the instantaneous phase-voltage extrema. Resulting duties are normalized
//! into `[0.0, 1.0]`. Requests outside the linear region are scaled back so the
//! commanded phase span fits within the available bus voltage.

use super::{PhaseDuty, centered_duty};
use crate::frame::{Abc, AlphaBeta};
use crate::scalar::{SQRT_3_OVER_2, clamp};
use crate::units::Duty;
use crate::util::is_finite2;

/// SVPWM result with status information.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct SvpwmResult {
    /// Normalized phase duties.
    pub duty: PhaseDuty,
    /// `true` when the requested vector was clipped or the bus input was invalid.
    pub saturated: bool,
    /// Requested phase-span modulation index, equal to `(vmax - vmin) / vbus`
    /// before clipping. Values above `1.0` indicate an over-range request.
    pub modulation_index: f32,
}

/// Computes linear SVPWM duties from a stationary-frame voltage request.
#[inline]
pub fn svpwm(v_ab: AlphaBeta<f32>, vbus: f32) -> SvpwmResult {
    if !is_finite2(v_ab.alpha, v_ab.beta) || !vbus.is_finite() || vbus <= 0.0 {
        return SvpwmResult {
            duty: centered_duty(),
            saturated: true,
            modulation_index: 0.0,
        };
    }

    let va = v_ab.alpha;
    let vb = -0.5 * v_ab.alpha + SQRT_3_OVER_2 * v_ab.beta;
    let vc = -0.5 * v_ab.alpha - SQRT_3_OVER_2 * v_ab.beta;

    let vmax = va.max(vb.max(vc));
    let vmin = va.min(vb.min(vc));
    let requested_span = vmax - vmin;
    let modulation_index = requested_span / vbus;

    let scale = if requested_span > vbus {
        vbus / requested_span
    } else {
        1.0
    };
    let saturated = scale < 1.0;

    let va = va * scale;
    let vb = vb * scale;
    let vc = vc * scale;

    let vmax = va.max(vb.max(vc));
    let vmin = va.min(vb.min(vc));
    let vcom = 0.5 * (vmax + vmin);
    let inv_vbus = 1.0 / vbus;

    SvpwmResult {
        duty: Abc::new(
            Duty::new(clamp(0.5 + (va - vcom) * inv_vbus, 0.0, 1.0)),
            Duty::new(clamp(0.5 + (vb - vcom) * inv_vbus, 0.0, 1.0)),
            Duty::new(clamp(0.5 + (vc - vcom) * inv_vbus, 0.0, 1.0)),
        ),
        saturated,
        modulation_index,
    }
}

#[cfg(test)]
mod tests {
    use super::svpwm;
    use crate::frame::AlphaBeta;
    use crate::modulation::PhaseDuty;

    fn approx_eq(a: f32, b: f32) {
        assert!((a - b).abs() < 1.0e-5, "{a} != {b}");
    }

    fn assert_duty_bounds(duty: PhaseDuty) {
        assert!((0.0..=1.0).contains(&duty.a.get()));
        assert!((0.0..=1.0).contains(&duty.b.get()));
        assert!((0.0..=1.0).contains(&duty.c.get()));
    }

    #[test]
    fn zero_vector_centers_all_phases() {
        let result = svpwm(AlphaBeta::new(0.0, 0.0), 24.0);
        assert_eq!(result.duty, super::super::centered_duty());
        assert!(!result.saturated);
    }

    #[test]
    fn positive_alpha_vector_has_balanced_symmetry() {
        let result = svpwm(AlphaBeta::new(6.0, 0.0), 24.0);
        assert_duty_bounds(result.duty);
        approx_eq(result.duty.a.get(), 0.6875);
        approx_eq(result.duty.b.get(), 0.3125);
        approx_eq(result.duty.c.get(), 0.3125);
    }

    #[test]
    fn overmodulation_request_is_scaled_back() {
        let result = svpwm(AlphaBeta::new(30.0, 0.0), 24.0);
        assert!(result.saturated);
        assert!(result.modulation_index > 1.0);
        assert_duty_bounds(result.duty);
        approx_eq(result.duty.a.get(), 1.0);
        approx_eq(result.duty.b.get(), 0.0);
        approx_eq(result.duty.c.get(), 0.0);
    }

    #[test]
    fn bus_voltage_must_be_positive() {
        let result = svpwm(AlphaBeta::new(1.0, 2.0), 0.0);
        assert_eq!(result.duty, super::super::centered_duty());
        assert!(result.saturated);
    }

    #[test]
    fn modulation_index_scales_with_bus_voltage() {
        let low_bus = svpwm(AlphaBeta::new(3.0, 1.0), 12.0);
        let high_bus = svpwm(AlphaBeta::new(3.0, 1.0), 24.0);

        assert!(low_bus.modulation_index > high_bus.modulation_index);
    }
}
