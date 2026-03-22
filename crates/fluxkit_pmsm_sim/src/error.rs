//! Error definitions for the PMSM simulator.

use core::fmt;

/// Errors returned by the PMSM simulator.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum Error {
    /// Static motor or plant parameters are invalid.
    InvalidParameters,
    /// A step input contained non-finite or otherwise invalid values.
    InvalidStepInput,
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidParameters => f.write_str("invalid PMSM simulator parameters"),
            Self::InvalidStepInput => f.write_str("invalid PMSM simulator step input"),
        }
    }
}

impl core::error::Error for Error {}
