//! Monotonic timing traits for platform code.

/// Monotonic microsecond timestamp source.
pub trait MonotonicMicros {
    /// Returns a monotonically increasing microsecond timestamp.
    fn now_micros(&self) -> u64;
}
