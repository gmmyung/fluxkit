//! Compact status snapshots for telemetry and inspection.

use fluxkit_math::{
    ContinuousMechanicalAngle,
    frame::Dq,
    units::{Amps, RadPerSec, Volts},
};

use crate::{
    actuator::ActuatorCompensationTelemetry, error::Error, mode::ControlMode, state::MotorState,
};

/// Cheap-to-copy controller status snapshot.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct MotorStatus {
    /// Current runtime state.
    pub state: MotorState,
    /// Requested control mode.
    pub mode: ControlMode,
    /// Latched active error, if any.
    pub active_error: Option<Error>,
    /// Most recent measured DC bus voltage.
    pub last_bus_voltage: Volts,
    /// Most recent measured motor winding temperature in degrees Celsius.
    pub last_winding_temperature_c: f32,
    /// Most recent measured `d/q` current vector.
    pub last_measured_idq: Dq<Amps>,
    /// Most recent limited `d/q` voltage command.
    pub last_commanded_vdq: Dq<Volts>,
    /// Most recent wrapped mechanical rotor angle from the motor encoder in `[-pi, pi)`.
    pub last_rotor_mechanical_angle: ContinuousMechanicalAngle,
    /// Most recent unwrapped mechanical rotor angle accumulated across encoder wraps.
    pub last_unwrapped_rotor_mechanical_angle: ContinuousMechanicalAngle,
    /// Most recent measured mechanical rotor velocity.
    pub last_rotor_mechanical_velocity: RadPerSec,
    /// Most recent wrapped output-axis angle from the actuator encoder in `[-pi, pi)`.
    pub last_output_mechanical_angle: ContinuousMechanicalAngle,
    /// Most recent unwrapped output-axis angle accumulated across encoder wraps.
    pub last_unwrapped_output_mechanical_angle: ContinuousMechanicalAngle,
    /// Most recent measured output-axis mechanical velocity.
    pub last_output_mechanical_velocity: RadPerSec,
    /// Most recent actuator compensation breakdown.
    pub last_actuator_compensation: ActuatorCompensationTelemetry,
    /// Most recent saturation flag.
    pub last_saturated: bool,
}
