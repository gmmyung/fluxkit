//! Gate-driver control traits.

/// Gate-driver-specific fault classification.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum GateDriverFault {
    /// Gate-driver undervoltage lockout.
    UnderVoltage,
    /// Gate-driver overtemperature condition.
    OverTemperature,
    /// Desaturation or overcurrent shutdown.
    Desaturation,
    /// External hardware fault input.
    ExternalFault,
    /// Unknown driver-specific fault.
    Unknown,
}

/// Narrow synchronous trait for gate-driver supervision.
pub trait GateDriver {
    /// Platform-specific error type.
    type Error;

    /// Enables the gate driver.
    fn enable_gate(&mut self) -> Result<(), Self::Error>;

    /// Disables the gate driver.
    fn disable_gate(&mut self) -> Result<(), Self::Error>;

    /// Clears any latched gate-driver faults.
    fn clear_faults(&mut self) -> Result<(), Self::Error>;

    /// Returns the current gate-driver fault, if any.
    fn fault_status(&mut self) -> Result<Option<GateDriverFault>, Self::Error>;
}
