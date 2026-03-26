//! Synchronous winding-temperature sensing traits.

/// Synchronous temperature sensor.
pub trait TemperatureSensor {
    /// Platform-specific error type.
    type Error: core::error::Error;

    /// Returns the measured temperature in degrees Celsius.
    fn sample_temperature_c(&mut self) -> Result<f32, Self::Error>;
}
