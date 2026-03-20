//! Compact status snapshots for telemetry and inspection.

use fluxkit_math::{
    frame::Dq,
    units::{Amps, Volts},
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
    /// Most recent saturation flag.
    pub last_saturated: bool,
}
