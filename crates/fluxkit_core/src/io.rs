//! Fast-loop input and output contracts.

use fluxkit_math::{
    ElectricalAngle,
    frame::{Abc, Dq},
    modulation::PhaseDuty,
    units::{Amps, RadPerSec, Volts},
};

use crate::fault::FaultKind;

/// Provenance of the rotor-angle estimate.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum AngleSource {
    /// Incremental or absolute encoder.
    Encoder,
    /// Hall sensors.
    Hall,
    /// Sensorless observer.
    SensorlessObserver,
    /// Open-loop estimator or startup trajectory.
    OpenLoopEstimator,
}

/// Rotor angle and speed estimate supplied by platform code.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RotorEstimate {
    /// Electrical rotor angle.
    pub electrical_angle: ElectricalAngle,
    /// Mechanical rotor velocity estimate.
    pub mechanical_velocity: RadPerSec,
    /// Source that produced the estimate.
    pub source: AngleSource,
}

/// Synchronous data required by the high-rate current-control loop.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FastLoopInput {
    /// Measured three-phase currents.
    pub phase_currents: Abc<Amps>,
    /// Measured DC bus voltage.
    pub bus_voltage: Volts,
    /// Rotor estimate used for the Park transforms.
    pub rotor: RotorEstimate,
    /// Time since the previous fast-loop invocation.
    pub dt_seconds: f32,
}

/// Result of one synchronous fast-loop execution.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FastLoopOutput {
    /// Commanded normalized phase duties.
    pub phase_duty: PhaseDuty,
    /// Measured current vector in the rotating frame.
    pub measured_idq: Dq<Amps>,
    /// Voltage command after limiting.
    pub commanded_vdq: Dq<Volts>,
    /// `true` when the controller or modulator clipped the request.
    pub saturated: bool,
    /// Latched fault observed during this tick, if any.
    pub fault: Option<FaultKind>,
}
