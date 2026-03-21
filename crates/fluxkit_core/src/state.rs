//! Explicit motor runtime states.

/// Coarse runtime state of the motor controller.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum MotorState {
    /// Output is inhibited and the controller is idle.
    Disabled,
    /// Enabled and ready to enter a runnable control mode.
    Ready,
    /// Closed-loop operation is active.
    Running,
    /// A latched fault blocks further operation.
    Faulted,
}
