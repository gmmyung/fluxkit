//! Compact status snapshots for telemetry and inspection.

use fluxkit_math::{
    frame::Dq,
    units::{Amps, Volts},
};

use crate::{fault::FaultKind, io::AngleSource, mode::ControlMode, state::MotorState};

/// Cheap-to-copy controller status snapshot.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MotorStatus {
    /// Current runtime state.
    pub state: MotorState,
    /// Requested control mode.
    pub mode: ControlMode,
    /// Latched active fault, if any.
    pub active_fault: Option<FaultKind>,
    /// Most recent measured DC bus voltage.
    pub last_bus_voltage: Volts,
    /// Most recent measured `d/q` current vector.
    pub last_measured_idq: Dq<Amps>,
    /// Most recent limited `d/q` voltage command.
    pub last_commanded_vdq: Dq<Volts>,
    /// Most recent saturation flag.
    pub last_saturated: bool,
    /// Source of the most recent rotor estimate, if available.
    pub last_rotor_source: Option<AngleSource>,
}
