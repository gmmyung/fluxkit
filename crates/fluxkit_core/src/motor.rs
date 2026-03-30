//! Deterministic motor-control engine used by the higher-level `fluxkit` wrapper.

use fluxkit_math::angle::{
    mechanical_to_electrical_with_direction, mechanical_velocity_to_electrical,
    shortest_angle_delta,
};
use fluxkit_math::{
    ContinuousMechanicalAngle, Modulator, PiConfig, PiController, Svpwm, clamp, clarke,
    inverse_park, limit_norm_dq, park,
    units::{Amps, NewtonMeters, RadPerSec, Volts},
};

use crate::{
    actuator::{ActuatorCompensationTelemetry, ActuatorParams},
    calibration::ActuatorCalibration,
    config::CurrentLoopConfig,
    control::current::CurrentReference,
    error::Error,
    io::{FastLoopInput, FastLoopOutput, RotorEstimate},
    mode::ControlMode,
    params::{
        InverterParams, MotorParams, PHASE_RESISTANCE_REFERENCE_TEMP_C,
        PHASE_RESISTANCE_TEMP_COEFF_PER_C,
    },
    state::MotorState,
    status::MotorStatus,
    util::{neutral_phase_duty, zero_current_dq, zero_voltage_dq},
    validation::{validate_controller_config, validate_fast_loop_input},
};

mod command;
mod fast_loop;
mod supervisory;
mod support;
#[cfg(test)]
mod tests;

#[cfg_attr(doc, aquamarine::aquamarine)]
/// Pure control-engine state for a single motor.
///
/// This type is the lower-level deterministic controller that `fluxkit`
/// runtime wrappers drive. Most application code should prefer
/// `fluxkit::MotorRuntime`; this type exists for engine-level integration and
/// testing.
///
/// Public interaction is intentionally narrow:
///
/// - `new(...)` builds the controller with explicit model and tuning data
/// - `apply_command(...)` replaces the active command target
/// - `set_armed(...)` transitions between disabled and running states
/// - `step(...)` executes one deterministic control cycle
/// - `status()` returns the latest compact snapshot
///
/// Internally each `step(...)` runs the electrical current loop first and then
/// updates supervisory references for the next cycle.
///
/// ```mermaid
/// flowchart LR
///     subgraph S[supervisory update]
///         PT[Position target] --> PP[Position PI]
///         RA[Unwrapped mechanical angle] --> PP
///         PP --> VT[Velocity target]
///         TT[Torque target] --> TM[Torque to iq mapping]
///         VT --> VP[Velocity PI]
///         RV[Mechanical velocity] --> VP
///         VP --> IQ[iq target]
///         TM --> IQ
///     end
///     A[Phase currents abc] --> B[Input validation]
///     V[Bus voltage] --> B
///     R[Rotor estimate] --> B
///     T[dt_seconds] --> B
///     B --> C[Clarke transform]
///     C --> D[Park transform]
///     D --> E[Measure id iq]
///     ID[id target] --> F[Current reference]
///     IQ --> F
///     OL[Open-loop vdq target] --> VL[Voltage select]
///     F --> FD[Model feedforward]
///     F --> ER[Current error]
///     E --> ER
///     ER --> G[d-axis PI]
///     ER --> H[q-axis PI]
///     FD --> G
///     FD --> H
///     G --> CV[Closed-loop vdq]
///     H --> CV
///     CV --> VL
///     VL --> L[Vector limit in dq]
///     L --> M[Inverse Park]
///     M --> N[Configured modulator]
///     N --> O[Duty clamp]
///     O --> P[Phase duty output]
///     L --> Q[Status snapshot]
///     E --> Q
///     R --> Q
/// ```
#[derive(Clone, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct MotorController<M = Svpwm> {
    motor: MotorParams,
    inverter: InverterParams,
    actuator: ActuatorParams,
    config: CurrentLoopConfig,
    modulator: M,
    state: MotorState,
    mode: ControlMode,
    id_target: Amps,
    iq_target: Amps,
    output_torque_target: NewtonMeters,
    output_velocity_target: RadPerSec,
    output_position_target: ContinuousMechanicalAngle,
    mit_kp: f32,
    mit_kd: f32,
    open_loop_voltage_target: fluxkit_math::frame::Dq<Volts>,
    d_pi: PiController,
    q_pi: PiController,
    velocity_pi: PiController,
    position_pi: PiController,
    active_error: Option<Error>,
    last_rotor: Option<RotorEstimate>,
    last_wrapped_mechanical_angle: Option<ContinuousMechanicalAngle>,
    last_wrapped_output_angle: Option<ContinuousMechanicalAngle>,
    last_current_ref: Option<fluxkit_math::frame::Dq<Amps>>,
    status: MotorStatus,
}

/// Lower-level controller command consumed by [`MotorController`].
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum ControllerCommand {
    /// Disabled command with all targets cleared.
    Disabled,
    /// Direct current-mode command with explicit `d/q` current targets.
    Current(fluxkit_math::frame::Dq<Amps>),
    /// Output-torque command.
    Torque(NewtonMeters),
    /// Output-side MIT impedance command.
    Mit {
        /// Output-axis position target.
        position: ContinuousMechanicalAngle,
        /// Output-axis velocity target.
        velocity: RadPerSec,
        /// Position stiffness gain in `Nm / rad`.
        kp: f32,
        /// Velocity damping gain in `Nm / (rad/s)`.
        kd: f32,
        /// Output-axis feedforward torque.
        torque_ff: NewtonMeters,
    },
    /// Output-velocity command.
    Velocity(RadPerSec),
    /// Output-position command.
    Position(ContinuousMechanicalAngle),
    /// Open-loop `d/q` voltage command.
    OpenLoopVoltage(fluxkit_math::frame::Dq<Volts>),
}

/// Owned controller-construction parts recovered from a [`MotorController`].
#[derive(Clone, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct MotorControllerParts<M = Svpwm> {
    /// Motor electrical model and limits.
    pub motor: MotorParams,
    /// Inverter configuration.
    pub inverter: InverterParams,
    /// Actuator model, limits, and compensation policy.
    pub actuator: ActuatorParams,
    /// Current-loop and supervisory tuning.
    pub config: CurrentLoopConfig,
    /// Configured modulation strategy.
    pub modulator: M,
}
