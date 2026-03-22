//! Explicit multi-rate scheduling contract for one controller cycle.

/// Optional lower-rate work to run around a fast current-loop step.
///
/// The intended ownership model is:
///
/// - one execution context owns the [`crate::MotorController`]
/// - every fast-cycle invokes one controller entrypoint
/// - slower supervisory work is represented as due flags inside this struct
///
/// This avoids sharing the controller across preempting timer interrupts.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct TickSchedule {
    /// If present, run the medium-rate supervisory loop after the fast loop.
    ///
    /// The updated references take effect on the next fast cycle.
    pub medium_dt_seconds: Option<f32>,
    /// If present, run the slow-rate supervisory loop after the medium loop.
    pub slow_dt_seconds: Option<f32>,
}

impl TickSchedule {
    /// Schedule no additional supervisory work.
    pub const NONE: Self = Self {
        medium_dt_seconds: None,
        slow_dt_seconds: None,
    };

    /// Creates a schedule that runs only the fast loop.
    #[inline]
    pub const fn none() -> Self {
        Self::NONE
    }

    /// Creates a schedule with medium-rate work due.
    #[inline]
    pub const fn with_medium(dt_seconds: f32) -> Self {
        Self {
            medium_dt_seconds: Some(dt_seconds),
            slow_dt_seconds: None,
        }
    }

    /// Creates a schedule with slow-rate work due.
    #[inline]
    pub const fn with_slow(dt_seconds: f32) -> Self {
        Self {
            medium_dt_seconds: None,
            slow_dt_seconds: Some(dt_seconds),
        }
    }

    /// Creates a schedule with both medium- and slow-rate work due.
    #[inline]
    pub const fn with_medium_and_slow(medium_dt_seconds: f32, slow_dt_seconds: f32) -> Self {
        Self {
            medium_dt_seconds: Some(medium_dt_seconds),
            slow_dt_seconds: Some(slow_dt_seconds),
        }
    }
}
