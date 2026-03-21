//! Control-mode definitions.

/// Active control mode for the motor controller.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum ControlMode {
    /// Force a neutral output and keep the controller inactive.
    Disabled,
    /// Closed-loop `d/q` current control.
    Current,
}
