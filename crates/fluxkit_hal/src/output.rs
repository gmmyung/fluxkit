//! Output-axis sensing traits.

use fluxkit_math::{MechanicalAngle, units::RadPerSec};

/// Output-axis reading returned by the actuator encoder path.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct OutputReading {
    /// Wrapped output-axis mechanical angle from the output encoder in `[-pi, pi)`.
    pub mechanical_angle: MechanicalAngle,
    /// Output-axis mechanical speed derived from the output encoder path.
    pub mechanical_velocity: RadPerSec,
}

/// Narrow synchronous trait for output-axis sensing.
pub trait OutputSensor {
    /// Platform-specific error type.
    type Error: core::error::Error;

    /// Returns the current output-axis estimate.
    fn read_output(&mut self) -> Result<OutputReading, Self::Error>;
}
