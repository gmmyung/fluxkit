//! Error definitions for the pure control engine.

use core::fmt;

/// Core error kind used by the control engine.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum Error {
    /// The measured DC bus voltage was invalid for control.
    InvalidBusVoltage,
    /// The measured winding temperature was invalid for control.
    InvalidTemperature,
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

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let message = match self {
            Self::InvalidBusVoltage => "invalid bus voltage",
            Self::InvalidTemperature => "invalid winding temperature",
            Self::InvalidPhaseCurrent => "invalid phase current",
            Self::InvalidRotorAngle => "invalid rotor angle",
            Self::NonFiniteComputation => "non-finite computation",
            Self::TimingOverrun => "timing overrun",
            Self::ConfigurationInvalid => "invalid controller configuration",
        };

        f.write_str(message)
    }
}

impl core::error::Error for Error {}
