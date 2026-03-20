//! DC bus-voltage sensing traits.

use fluxkit_math::units::Volts;

/// Narrow synchronous trait for DC bus-voltage acquisition.
pub trait BusVoltageSensor {
    /// Platform-specific error type.
    type Error;

    /// Returns the measured DC bus voltage.
    fn sample_bus_voltage(&mut self) -> Result<Volts, Self::Error>;
}
