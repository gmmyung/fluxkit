//! Control-loop input and output contracts.

use fluxkit_math::{
    ContinuousMechanicalAngle,
    frame::{Abc, Dq},
    modulation::PhaseDuty,
    units::{Amps, RadPerSec, Volts},
};

use crate::{actuator::ActuatorEstimate, motor::ControllerCommand};

/// Rotor angle and speed estimate supplied by platform code.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct RotorEstimate {
    /// Continuous mechanical rotor angle.
    ///
    /// The controller derives electrical angle internally from this value,
    /// `MotorParams::pole_pairs`, and `MotorParams::electrical_angle_offset`.
    ///
    /// This is required by `Position` mode.
    pub mechanical_angle: ContinuousMechanicalAngle,
    /// Mechanical rotor velocity estimate derived from the encoder path.
    ///
    /// This is used by current-loop feedforward and the velocity loop.
    pub mechanical_velocity: RadPerSec,
}

/// Synchronous data required by one controller step.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct ControlInput {
    /// Command target to apply for this step.
    pub command: ControllerCommand,
    /// `true` when the controller should run, `false` when it should remain disabled.
    pub armed: bool,
    /// Requests that a latched controller fault be cleared before this step.
    pub clear_fault_requested: bool,
    /// Measured three-phase currents.
    pub phase_currents: Abc<Amps>,
    /// Measured DC bus voltage.
    pub bus_voltage: Volts,
    /// Measured motor winding temperature in degrees Celsius.
    pub winding_temperature_c: f32,
    /// Rotor estimate used for the Park transforms.
    pub rotor: RotorEstimate,
    /// Output-axis estimate used by actuator-side supervisory loops.
    pub actuator: ActuatorEstimate,
    /// Time since the previous control-loop invocation.
    pub dt_seconds: f32,
}

/// Result of one synchronous controller step.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct ControlOutput {
    /// Commanded normalized phase duties.
    pub phase_duty: PhaseDuty,
    /// Measured current vector in the rotating frame.
    pub measured_idq: Dq<Amps>,
    /// Voltage command after limiting.
    pub commanded_vdq: Dq<Volts>,
    /// `true` when the controller or modulator clipped the request.
    pub saturated: bool,
}
