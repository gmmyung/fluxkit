//! Trigonometric helpers isolated behind a small API.

use crate::angle::wrap;
use crate::scalar::sqrt;

/// Returns `sin(theta)`.
#[inline]
pub fn sin(theta: f32) -> f32 {
    sin_cos(theta).0
}

/// Returns `cos(theta)`.
#[inline]
pub fn cos(theta: f32) -> f32 {
    sin_cos(theta).1
}

/// Returns `(sin(theta), cos(theta))`.
#[inline]
pub fn sin_cos(theta: f32) -> (f32, f32) {
    let theta = wrap(theta);
    let (s, c) = micromath::F32Ext::sin_cos(theta);
    let norm2 = s * s + c * c;
    if !norm2.is_finite() || norm2 <= 0.0 {
        return (s, c);
    }

    let inv_norm = 1.0 / sqrt(norm2);
    (s * inv_norm, c * inv_norm)
}
