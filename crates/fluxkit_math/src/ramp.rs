//! Slew-rate limiting.

/// Symmetric or asymmetric slew-rate limiter with explicit state.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct SlewRateLimiter {
    /// Maximum rising slope in units per second.
    pub rise_per_sec: f32,
    /// Maximum falling slope magnitude in units per second.
    pub fall_per_sec: f32,
    /// Current output value.
    pub y: f32,
}

impl SlewRateLimiter {
    /// Creates a limiter with zero initial output.
    #[inline]
    pub const fn new(rise_per_sec: f32, fall_per_sec: f32) -> Self {
        Self {
            rise_per_sec,
            fall_per_sec,
            y: 0.0,
        }
    }

    /// Resets the output state directly.
    #[inline]
    pub fn reset(&mut self, value: f32) {
        self.y = value;
    }

    /// Updates the limited output for a new target and sample interval.
    #[inline]
    pub fn update(&mut self, target: f32, dt: f32) -> f32 {
        if dt <= 0.0 || !dt.is_finite() {
            return self.y;
        }

        let delta = target - self.y;
        let up = self.rise_per_sec.max(0.0) * dt;
        let down = self.fall_per_sec.max(0.0) * dt;

        let step = if delta > up {
            up
        } else if delta < -down {
            -down
        } else {
            delta
        };

        self.y += step;
        self.y
    }
}

#[cfg(test)]
mod tests {
    use super::SlewRateLimiter;

    fn approx_eq(a: f32, b: f32) {
        assert!((a - b).abs() < 1.0e-6, "{a} != {b}");
    }

    #[test]
    fn slew_rate_limits_rise_and_fall_independently() {
        let mut limiter = SlewRateLimiter::new(10.0, 5.0);

        approx_eq(limiter.update(2.0, 0.1), 1.0);
        approx_eq(limiter.update(-2.0, 0.1), 0.5);
    }

    #[test]
    fn slew_rate_reset_is_explicit() {
        let mut limiter = SlewRateLimiter::new(1.0, 1.0);
        limiter.reset(3.0);
        approx_eq(limiter.update(0.0, 1.0), 2.0);
    }
}
