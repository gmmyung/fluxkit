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
    /// Torque-oriented request mode, currently aliased to current control.
    Torque,
    /// Placeholder for future closed-loop velocity control.
    Velocity,
    /// Placeholder for future closed-loop position control.
    Position,
    /// Placeholder for future open-loop voltage injection.
    OpenLoopVoltage,
}
