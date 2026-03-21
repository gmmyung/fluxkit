//! Compact status snapshots for telemetry and inspection.

use fluxkit_math::{
    MechanicalAngle,
    frame::Dq,
    units::{Amps, RadPerSec, Volts},
};

use crate::{error::Error, mode::ControlMode, state::MotorState};

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
    /// Most recent measured `d/q` current vector.
    pub last_measured_idq: Dq<Amps>,
    /// Most recent limited `d/q` voltage command.
    pub last_commanded_vdq: Dq<Volts>,
    /// Most recent wrapped mechanical rotor angle from the absolute encoder.
    pub last_mechanical_angle: MechanicalAngle,
    /// Most recent unwrapped mechanical rotor angle accumulated across encoder wraps.
    pub last_unwrapped_mechanical_angle: MechanicalAngle,
    /// Most recent measured mechanical rotor velocity.
    pub last_mechanical_velocity: RadPerSec,
    /// Most recent saturation flag.
    pub last_saturated: bool,
}
