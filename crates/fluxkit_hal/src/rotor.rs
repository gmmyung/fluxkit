//! Rotor-sensing traits.

use fluxkit_math::{ElectricalAngle, units::RadPerSec};

/// Hardware or estimator source behind a rotor reading.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RotorSensorKind {
    /// Encoder-backed reading.
    Encoder,
    /// Hall-sensor-backed reading.
    Hall,
    /// Sensorless estimate.
    Sensorless,
}

/// Rotor reading returned by a sensor or estimator.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RotorReading {
    /// Electrical rotor angle.
    pub electrical_angle: ElectricalAngle,
    /// Estimated mechanical rotor speed.
    pub mechanical_velocity: RadPerSec,
    /// Source kind for the reading.
    pub kind: RotorSensorKind,
}

/// Narrow synchronous trait for rotor sensing.
pub trait RotorSensor {
    /// Platform-specific error type.
    type Error;

    /// Returns the current rotor estimate.
    fn read_rotor(&mut self) -> Result<RotorReading, Self::Error>;
}
