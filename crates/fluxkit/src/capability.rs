use core::fmt;

/// Failure returned when a runtime or calibration owner cannot be split into
/// its unique handle and ticker capabilities.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum CapabilitySplitError {
    /// A handle/ticker pair was already created from this owner.
    AlreadySplit,
    /// The owner no longer contains an active inner runtime.
    Inactive,
}

impl fmt::Display for CapabilitySplitError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::AlreadySplit => f.write_str("capabilities already split"),
            Self::Inactive => f.write_str("runtime inactive"),
        }
    }
}

impl core::error::Error for CapabilitySplitError {}
