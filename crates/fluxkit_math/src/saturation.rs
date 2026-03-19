//! Saturation and norm limiting helpers.

use crate::frame::{AlphaBeta, Dq};
use crate::scalar::clamp as scalar_clamp;

/// Clamps `x` into `[min, max]`.
#[inline]
pub fn clamp(x: f32, min: f32, max: f32) -> f32 {
    scalar_clamp(x, min, max)
}

/// Clamps `x` into `[-max_abs, max_abs]`.
#[inline]
pub fn clamp_abs(x: f32, max_abs: f32) -> f32 {
    scalar_clamp(x, -max_abs, max_abs)
}

/// Circularly limits an `alpha-beta` vector magnitude.
#[inline]
pub fn limit_norm_ab(v: AlphaBeta<f32>, max_mag: f32) -> AlphaBeta<f32> {
    let max_mag = max_mag.max(0.0);
    let mag2 = v.alpha * v.alpha + v.beta * v.beta;
    let max2 = max_mag * max_mag;

    if mag2 == 0.0 || mag2 <= max2 {
        return v;
    }

    if max_mag == 0.0 {
        return AlphaBeta::zero();
    }

    let scale = max_mag / libm::sqrtf(mag2);
    AlphaBeta {
        alpha: v.alpha * scale,
        beta: v.beta * scale,
    }
}

/// Circularly limits a `dq` vector magnitude.
#[inline]
pub fn limit_norm_dq(v: Dq<f32>, max_mag: f32) -> Dq<f32> {
    let max_mag = max_mag.max(0.0);
    let mag2 = v.d * v.d + v.q * v.q;
    let max2 = max_mag * max_mag;

    if mag2 == 0.0 || mag2 <= max2 {
        return v;
    }

    if max_mag == 0.0 {
        return Dq::zero();
    }

    let scale = max_mag / libm::sqrtf(mag2);
    Dq {
        d: v.d * scale,
        q: v.q * scale,
    }
}
#[cfg(test)]
mod tests {
    use super::{clamp, clamp_abs, limit_norm_ab, limit_norm_dq};
    use crate::frame::{AlphaBeta, Dq};

    fn approx_eq(a: f32, b: f32) {
        assert!((a - b).abs() < 1.0e-5, "{a} != {b}");
    }

    #[test]
    fn clamp_helpers_bound_scalars() {
        approx_eq(clamp(2.0, -1.0, 1.0), 1.0);
        approx_eq(clamp_abs(-3.0, 2.0), -2.0);
    }

    #[test]
    fn norm_limiters_preserve_direction() {
        let ab = limit_norm_ab(AlphaBeta::new(3.0, 4.0), 2.0);
        let dq = limit_norm_dq(Dq::new(6.0, 8.0), 1.0);

        approx_eq(ab.alpha, 1.2);
        approx_eq(ab.beta, 1.6);
        approx_eq(dq.d, 0.6);
        approx_eq(dq.q, 0.8);
    }
}
