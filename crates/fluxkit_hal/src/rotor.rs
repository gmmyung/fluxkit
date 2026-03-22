//! Rotor-sensing traits.

use fluxkit_math::{MechanicalAngle, units::RadPerSec};

/// Rotor reading returned by the absolute encoder path.
///
/// The platform provides mechanical angle and speed only. `fluxkit_core`
/// derives electrical angle internally from the configured pole-pair count and
/// electrical zero offset.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct RotorReading {
    /// Mechanical rotor angle from the absolute encoder.
    pub mechanical_angle: MechanicalAngle,
    /// Mechanical rotor speed derived from the encoder path.
    pub mechanical_velocity: RadPerSec,
}

/// Narrow synchronous trait for absolute-encoder rotor sensing.
pub trait RotorSensor {
    /// Platform-specific error type.
    type Error: core::error::Error;

    /// Returns the current encoder-backed rotor estimate.
    fn read_rotor(&mut self) -> Result<RotorReading, Self::Error>;
}
