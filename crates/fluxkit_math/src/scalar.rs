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
