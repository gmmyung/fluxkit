//! Fault definitions for the pure control engine.

/// Latched controller fault kind.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FaultKind {
    /// The measured DC bus voltage was invalid for control.
    InvalidBusVoltage,
    /// The phase-current measurement was not finite.
    InvalidPhaseCurrent,
    /// The rotor angle estimate was not finite.
    InvalidRotorAngle,
    /// The controller produced a non-finite intermediate value.
    NonFiniteComputation,
    /// The control-step timing was invalid or missed its expected cadence.
    TimingOverrun,
    /// Static configuration was invalid.
    ConfigurationInvalid,
}
