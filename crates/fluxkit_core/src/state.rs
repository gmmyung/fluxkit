//! Explicit motor runtime states.

/// Coarse runtime state of the motor controller.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MotorState {
    /// Pre-initialization placeholder retained for future boot sequencing.
    Uninitialized,
    /// Output is inhibited and the controller is idle.
    Disabled,
    /// Enabled and ready to enter a runnable control mode.
    Ready,
    /// Placeholder state for current-sensor offset calibration.
    CalibratingCurrentOffset,
    /// Placeholder state for rotor-alignment procedures.
    AligningRotor,
    /// Closed-loop operation is active.
    Running,
    /// Placeholder state for controlled shutdown.
    Stopping,
    /// A latched fault blocks further operation.
    Faulted,
}
