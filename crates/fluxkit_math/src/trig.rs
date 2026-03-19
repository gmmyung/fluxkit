//! Trigonometric helpers isolated behind a small API.

/// Returns `sin(theta)`.
#[inline]
pub fn sin(theta: f32) -> f32 {
    libm::sinf(theta)
}

/// Returns `cos(theta)`.
#[inline]
pub fn cos(theta: f32) -> f32 {
    libm::cosf(theta)
}

/// Returns `(sin(theta), cos(theta))`.
#[inline]
pub fn sin_cos(theta: f32) -> (f32, f32) {
    (sin(theta), cos(theta))
}
