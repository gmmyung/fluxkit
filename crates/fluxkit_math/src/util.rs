//! Small shared helpers.

/// Returns `true` when both values are finite.
#[inline]
pub fn is_finite2(a: f32, b: f32) -> bool {
    a.is_finite() && b.is_finite()
}

/// Returns `true` when all three values are finite.
#[inline]
pub fn is_finite3(a: f32, b: f32, c: f32) -> bool {
    a.is_finite() && b.is_finite() && c.is_finite()
}
