//! Shared calibration error type.

use core::fmt;

/// Calibration failures unrelated to the main control-loop fault model.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum CalibrationError {
    /// Static configuration is invalid.
    InvalidConfiguration,
    /// A sampled input frame is invalid.
    InvalidInput,
    /// The observed motion did not support a reliable estimate.
    IndeterminateEstimate,
    /// The rotor never settled within the configured timeout.
    Timeout,
}

impl fmt::Display for CalibrationError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidConfiguration => f.write_str("invalid calibration configuration"),
            Self::InvalidInput => f.write_str("invalid calibration input"),
            Self::IndeterminateEstimate => {
                f.write_str("calibration data did not support a reliable estimate")
            }
            Self::Timeout => f.write_str("calibration timed out before the rotor settled"),
        }
    }
}

impl core::error::Error for CalibrationError {}
