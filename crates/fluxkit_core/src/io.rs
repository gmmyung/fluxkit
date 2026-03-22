//! Fast-loop input and output contracts.

use fluxkit_math::{
    MechanicalAngle,
    frame::{Abc, Dq},
    modulation::PhaseDuty,
    units::{Amps, RadPerSec, Volts},
};

use crate::actuator::ActuatorEstimate;
use crate::error::Error;

/// Rotor angle and speed estimate supplied by platform code.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct RotorEstimate {
    /// Mechanical rotor angle from the absolute encoder.
    ///
    /// The controller derives electrical angle internally from this value,
    /// `MotorParams::pole_pairs`, and `MotorParams::electrical_angle_offset`.
    ///
    /// This is required by `Position` mode.
    pub mechanical_angle: MechanicalAngle,
    /// Mechanical rotor velocity estimate derived from the encoder path.
    ///
    /// This is used by current-loop feedforward and the velocity loop.
    pub mechanical_velocity: RadPerSec,
}

/// Synchronous data required by the high-rate current-control loop.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct FastLoopInput {
    /// Measured three-phase currents.
    pub phase_currents: Abc<Amps>,
    /// Measured DC bus voltage.
    pub bus_voltage: Volts,
    /// Rotor estimate used for the Park transforms.
    pub rotor: RotorEstimate,
    /// Output-axis estimate used by actuator-side supervisory loops.
    pub actuator: ActuatorEstimate,
    /// Time since the previous fast-loop invocation.
    pub dt_seconds: f32,
}

/// Result of one synchronous fast-loop execution.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct FastLoopOutput {
    /// Commanded normalized phase duties.
    pub phase_duty: PhaseDuty,
    /// Measured current vector in the rotating frame.
    pub measured_idq: Dq<Amps>,
    /// Voltage command after limiting.
    pub commanded_vdq: Dq<Volts>,
    /// `true` when the controller or modulator clipped the request.
    pub saturated: bool,
    /// Error observed during this tick, if any.
    pub error: Option<Error>,
}
