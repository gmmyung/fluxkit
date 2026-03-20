//! Optional external fault-input traits.

/// Optional trait for a discrete fault or trip input.
pub trait FaultInput {
    /// Platform-specific error type.
    type Error: core::error::Error;

    /// Returns `true` when the hardware fault input is active.
    fn is_fault_active(&mut self) -> Result<bool, Self::Error>;
}
