//! First-order low-pass filter.

use crate::scalar::clamp;

/// Deterministic first-order low-pass filter.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct LowPassFilter {
    alpha: f32,
    y: f32,
    initialized: bool,
}

impl LowPassFilter {
    /// Creates a filter with the provided smoothing factor.
    ///
    /// `alpha` is clamped into `[0.0, 1.0]`.
    #[inline]
    pub fn new(alpha: f32) -> Self {
        Self {
            alpha: clamp(alpha, 0.0, 1.0),
            y: 0.0,
            initialized: false,
        }
    }

    /// Creates a filter from a sampling interval `dt` and time constant `tau`.
    ///
    /// Uses `alpha = dt / (tau + dt)`. A non-positive `tau` becomes a direct
    /// pass-through with `alpha = 1.0`.
    #[inline]
    pub fn from_tau(dt: f32, tau: f32) -> Self {
        let alpha = if tau <= 0.0 {
            1.0
        } else if dt <= 0.0 {
            0.0
        } else {
            dt / (tau + dt)
        };
        Self::new(alpha)
    }

    /// Resets the internal state to `value`.
    #[inline]
    pub fn reset(&mut self, value: f32) {
        self.y = value;
        self.initialized = true;
    }

    /// Returns the configured smoothing factor.
    #[inline]
    pub const fn alpha(&self) -> f32 {
        self.alpha
    }

    /// Returns the current output value.
    #[inline]
    pub const fn output(&self) -> f32 {
        self.y
    }

    /// Updates the filter state with a new sample.
    #[inline]
    pub fn update(&mut self, x: f32) -> f32 {
        if !self.initialized {
            self.y = x;
            self.initialized = true;
        } else {
            self.y += self.alpha * (x - self.y);
        }
        self.y
    }
}

#[cfg(test)]
mod tests {
    use super::LowPassFilter;

    fn approx_eq(a: f32, b: f32) {
        assert!((a - b).abs() < 1.0e-6, "{a} != {b}");
    }

    #[test]
    fn low_pass_initializes_from_first_sample() {
        let mut lpf = LowPassFilter::new(0.25);
        approx_eq(lpf.update(4.0), 4.0);
        approx_eq(lpf.update(8.0), 5.0);
    }

    #[test]
    fn low_pass_from_tau_is_reasonable() {
        let lpf = LowPassFilter::from_tau(0.001, 0.009);
        approx_eq(lpf.alpha(), 0.1);
    }

    #[test]
    fn low_pass_reset_is_explicit() {
        let mut lpf = LowPassFilter::new(0.5);
        lpf.reset(-2.0);
        approx_eq(lpf.output(), -2.0);
        approx_eq(lpf.update(2.0), 0.0);
    }
}
