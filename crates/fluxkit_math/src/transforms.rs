//! Clarke and Park transforms for FOC.
//!
//! # Convention
//!
//! - Clarke: amplitude-invariant balanced form.
//! - Alpha axis: aligned with phase A.
//! - Park: positive electrical angle rotates the stationary `alpha-beta` frame
//!   into the rotating `dq` frame.
//! - `d` axis: aligned with rotor electrical angle.
//! - `q` axis: positive under the standard PMSM torque-producing convention.

use crate::frame::{Abc, AlphaBeta, Dq};
use crate::scalar::{FRAC_1_SQRT_3, SQRT_3_OVER_2};
use crate::trig::sin_cos;

/// Applies the amplitude-invariant Clarke transform.
#[inline]
pub fn clarke(abc: Abc<f32>) -> AlphaBeta<f32> {
    AlphaBeta {
        alpha: abc.a,
        beta: (abc.a + 2.0 * abc.b) * FRAC_1_SQRT_3,
    }
}

/// Applies the inverse Clarke transform that is the exact inverse of
/// [`clarke`] for balanced three-phase inputs.
#[inline]
pub fn inverse_clarke(ab: AlphaBeta<f32>) -> Abc<f32> {
    Abc {
        a: ab.alpha,
        b: -0.5 * ab.alpha + SQRT_3_OVER_2 * ab.beta,
        c: -0.5 * ab.alpha - SQRT_3_OVER_2 * ab.beta,
    }
}

/// Applies the Park transform using electrical angle `theta_elec`.
#[inline]
pub fn park(ab: AlphaBeta<f32>, theta_elec: f32) -> Dq<f32> {
    let (s, c) = sin_cos(theta_elec);
    Dq {
        d: ab.alpha * c + ab.beta * s,
        q: -ab.alpha * s + ab.beta * c,
    }
}

/// Applies the inverse Park transform using electrical angle `theta_elec`.
#[inline]
pub fn inverse_park(dq: Dq<f32>, theta_elec: f32) -> AlphaBeta<f32> {
    let (s, c) = sin_cos(theta_elec);
    AlphaBeta {
        alpha: dq.d * c - dq.q * s,
        beta: dq.d * s + dq.q * c,
    }
}

#[cfg(test)]
mod tests {
    use super::{clarke, inverse_clarke, inverse_park, park};
    use crate::frame::{Abc, AlphaBeta, Dq};
    use crate::scalar::{FRAC_1_SQRT_3, PI};

    fn approx_eq(a: f32, b: f32) {
        assert!((a - b).abs() < 1.0e-5, "{a} != {b}");
    }

    fn approx_eq_ab(a: AlphaBeta<f32>, b: AlphaBeta<f32>) {
        approx_eq(a.alpha, b.alpha);
        approx_eq(a.beta, b.beta);
    }

    fn approx_eq_dq(a: Dq<f32>, b: Dq<f32>) {
        approx_eq(a.d, b.d);
        approx_eq(a.q, b.q);
    }

    fn approx_eq_abc(a: Abc<f32>, b: Abc<f32>) {
        approx_eq(a.a, b.a);
        approx_eq(a.b, b.b);
        approx_eq(a.c, b.c);
    }

    #[test]
    fn clarke_matches_known_balanced_vector() {
        let abc = Abc::new(1.0, -0.5, -0.5);
        let ab = clarke(abc);

        approx_eq(ab.alpha, 1.0);
        approx_eq(ab.beta, 0.0);
    }

    #[test]
    fn clarke_matches_general_known_vector() {
        let abc = Abc::new(1.2, -0.4, -0.8);
        let ab = clarke(abc);

        approx_eq(ab.alpha, 1.2);
        approx_eq(ab.beta, (1.2 - 0.8) * FRAC_1_SQRT_3);
    }

    #[test]
    fn inverse_clarke_round_trip_restores_balanced_input() {
        let abc = Abc::new(0.9, -0.2, -0.7);
        approx_eq_abc(inverse_clarke(clarke(abc)), abc);
    }

    #[test]
    fn park_and_inverse_park_round_trip() {
        let ab = AlphaBeta::new(0.75, -0.2);
        let theta = 1.3;
        approx_eq_ab(inverse_park(park(ab, theta), theta), ab);
    }

    #[test]
    fn zero_angle_park_is_identity() {
        let ab = AlphaBeta::new(2.0, -3.0);
        approx_eq_dq(park(ab, 0.0), Dq::new(2.0, -3.0));
    }

    #[test]
    fn quarter_turn_rotates_axes() {
        let ab = AlphaBeta::new(1.0, 0.0);
        let dq = park(ab, 0.5 * PI);

        approx_eq(dq.d, 0.0);
        approx_eq(dq.q, -1.0);
    }

    #[test]
    fn transform_sweeps_remain_consistent() {
        let angles = [-7.0, -1.0, -0.25, 0.0, 0.9, 2.5, 8.0];
        let vectors = [
            AlphaBeta::new(0.0, 0.0),
            AlphaBeta::new(1.0, 0.0),
            AlphaBeta::new(-0.3, 0.7),
            AlphaBeta::new(2.0, -1.0),
        ];

        for theta in angles {
            for vector in vectors {
                approx_eq_ab(inverse_park(park(vector, theta), theta), vector);
            }
        }
    }
}
