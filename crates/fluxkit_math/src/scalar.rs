//! Scalar aliases, constants, and helpers.

/// Internal real scalar type for the MVP implementation.
pub type Real = f32;

/// Pi.
pub const PI: Real = core::f32::consts::PI;
/// Tau, equal to `2 * pi`.
pub const TAU: Real = core::f32::consts::TAU;
/// Square root of three.
pub const SQRT_3: Real = 1.732_050_8;
/// One over square root of three.
pub const FRAC_1_SQRT_3: Real = 0.577_350_26;
/// Square root of three divided by two.
pub const SQRT_3_OVER_2: Real = 0.866_025_4;

/// Clamps `x` into the inclusive range `[lo, hi]`.
#[inline]
pub fn clamp(x: Real, lo: Real, hi: Real) -> Real {
    if x < lo {
        lo
    } else if x > hi {
        hi
    } else {
        x
    }
}

/// Clamps `x` into the symmetric inclusive range `[-limit, limit]`.
#[inline]
pub fn clamp_abs(x: Real, limit: Real) -> Real {
    clamp(x, -limit, limit)
}

/// Square root helper backed by `micromath` and refined with two Newton steps.
#[inline]
pub fn sqrt(x: Real) -> Real {
    if !x.is_finite() || x < 0.0 {
        return Real::NAN;
    }
    if x == 0.0 {
        return 0.0;
    }

    let estimate = micromath::F32Ext::sqrt(x);
    if !estimate.is_finite() || estimate <= 0.0 {
        return estimate;
    }

    let refined = 0.5 * (estimate + x / estimate);
    0.5 * (refined + x / refined)
}

/// Natural logarithm helper backed by `micromath`.
#[inline]
pub fn ln(x: Real) -> Real {
    if !x.is_finite() || x < 0.0 {
        return Real::NAN;
    }
    if x == 0.0 {
        return Real::NEG_INFINITY;
    }

    let estimate = micromath::F32Ext::ln(x);
    if !estimate.is_finite() {
        return estimate;
    }

    let exp0 = micromath::F32Ext::exp(estimate);
    if !exp0.is_finite() || exp0 <= 0.0 {
        return estimate;
    }

    let refined = estimate - 1.0 + x / exp0;
    let exp1 = micromath::F32Ext::exp(refined);
    if !exp1.is_finite() || exp1 <= 0.0 {
        return refined;
    }

    refined - 1.0 + x / exp1
}
