//! Explicit command types for higher-level runtime glue.

use fluxkit_math::units::Amps;

use crate::mode::ControlMode;

/// Command enum reserved for future runtime-facing adapters.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum MotorCommand {
    /// Enable the controller.
    Enable,
    /// Disable the controller.
    Disable,
    /// Clear a latched controller error if possible.
    ClearError,
    /// Select the active control mode.
    SetMode(ControlMode),
    /// Update the `d`-axis current target.
    SetIdTarget(Amps),
    /// Update the `q`-axis current target.
    SetIqTarget(Amps),
}
