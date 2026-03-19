//! Sinusoidal PWM reference implementation.

use crate::frame::Abc;
use crate::frame::AlphaBeta;
use crate::modulation::{ModulationOutput, PhaseDuty, centered_duty};
use crate::scalar::SQRT_3_OVER_2;
use crate::units::Duty;

/// Computes clamped sinusoidal phase duties in `[0.0, 1.0]`.
#[inline]
pub fn sine_pwm(v_ab: AlphaBeta<f32>, vbus: f32) -> PhaseDuty {
    sine_pwm_result(v_ab, vbus).duty
}

#[inline]
pub(crate) fn sine_pwm_result(v_ab: AlphaBeta<f32>, vbus: f32) -> ModulationOutput {
    if !vbus.is_finite() || vbus <= 0.0 {
        return ModulationOutput {
            duty: centered_duty(),
            saturated: true,
        };
    }

    let inv_vbus = 1.0 / vbus;
    let va = v_ab.alpha;
    let vb = -0.5 * v_ab.alpha + SQRT_3_OVER_2 * v_ab.beta;
    let vc = -0.5 * v_ab.alpha - SQRT_3_OVER_2 * v_ab.beta;
    let da = 0.5 + va * inv_vbus;
    let db = 0.5 + vb * inv_vbus;
    let dc = 0.5 + vc * inv_vbus;

    let duty = Abc::new(
        Duty::new(crate::scalar::clamp(da, 0.0, 1.0)),
        Duty::new(crate::scalar::clamp(db, 0.0, 1.0)),
        Duty::new(crate::scalar::clamp(dc, 0.0, 1.0)),
    );

    let saturated = da < 0.0 || da > 1.0 || db < 0.0 || db > 1.0 || dc < 0.0 || dc > 1.0;

    ModulationOutput { duty, saturated }
}
