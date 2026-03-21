//! Control-mode definitions.

/// Active control mode for the motor controller.
///
/// Scheduling contract:
///
/// - `fast_tick()` always owns the electrical current loop and modulation
/// - `medium_tick()` owns torque, velocity, and position supervisory updates
/// - `slow_tick()` is currently a reserved hook only
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum ControlMode {
    /// Force a neutral output and keep the controller inactive.
    Disabled,
    /// Closed-loop `d/q` current control.
    Current,
    /// Closed-loop torque request mapped into the `q`-axis current target during `medium_tick()`.
    Torque,
    /// Closed-loop velocity control generating a `q`-axis current target during `medium_tick()`.
    Velocity,
    /// Closed-loop position control generating a velocity target and then a `q`-axis current
    /// target in the same `medium_tick()`.
    Position,
    /// Direct open-loop `d/q` voltage command.
    OpenLoopVoltage,
}
