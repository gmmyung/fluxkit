//! Small timing helpers shared by calibration state machines.

use crate::calibration::shared::CalibrationError;

/// Shared timeout and settle tracking for magnetic-hold style procedures.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub(crate) struct HoldTiming {
    /// Elapsed procedure time in seconds.
    pub elapsed_seconds: f32,
    /// Continuous settled time in seconds.
    pub settled_seconds: f32,
}

impl HoldTiming {
    /// Zeroed timing state.
    #[inline]
    pub const fn new() -> Self {
        Self {
            elapsed_seconds: 0.0,
            settled_seconds: 0.0,
        }
    }

    /// Advances elapsed time and returns a timeout error when the configured
    /// absolute timeout has been reached.
    #[inline]
    pub fn advance_elapsed(
        &mut self,
        dt_seconds: f32,
        timeout_seconds: f32,
    ) -> Option<CalibrationError> {
        self.elapsed_seconds += dt_seconds;
        if self.elapsed_seconds >= timeout_seconds {
            Some(CalibrationError::Timeout)
        } else {
            None
        }
    }

    /// Updates the settle timer and reports whether the required settle window
    /// has been satisfied continuously.
    #[inline]
    pub fn settle_ready(
        &mut self,
        is_settled: bool,
        dt_seconds: f32,
        settle_time_seconds: f32,
    ) -> bool {
        if is_settled {
            self.settled_seconds += dt_seconds;
        } else {
            self.settled_seconds = 0.0;
        }
        self.settled_seconds >= settle_time_seconds
    }

    /// Clears the settle timer.
    #[inline]
    pub fn reset_settle(&mut self) {
        self.settled_seconds = 0.0;
    }
}
