//! Control-mode definitions.

/// Active control mode for the motor controller.
///
/// Each controller cycle runs:
///
/// - the electrical current loop and modulation
/// - the supervisory torque, velocity, or position update for the next cycle
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum ControlMode {
    /// Force a neutral output and keep the controller inactive.
    Disabled,
    /// Closed-loop `d/q` current control.
    Current,
    /// Closed-loop torque request mapped into the `q`-axis current target.
    Torque,
    /// Closed-loop MIT impedance command with explicit output-side position,
    /// velocity, stiffness, damping, and torque feedforward.
    Mit,
    /// Closed-loop velocity control generating a `q`-axis current target.
    Velocity,
    /// Closed-loop position control generating a velocity target and then a `q`-axis current
    /// target in the same controller cycle.
    Position,
    /// Direct open-loop `d/q` voltage command.
    OpenLoopVoltage,
}
