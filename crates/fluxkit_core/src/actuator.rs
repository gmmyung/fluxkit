//! Actuator-side parameters, compensation, and output-axis estimates.
//!
//! The compensation layer sits above the inner electrical current loop. It
//! turns output-axis feedback torque into a bounded output torque command by
//! adding calibrated friction feedforward terms.
//!
//! Friction compensation uses a hybrid signal policy:
//!
//! - near zero speed, breakaway direction follows the commanded motion hint so
//!   the controller can push through the deadzone without waiting for noisy
//!   measured motion
//! - that breakaway term is capped to only the missing static-friction margin,
//!   so it does not keep adding startup torque when the commanded effort is
//!   already sufficient
//! - once motion is established, Coulomb and viscous drag compensation follow
//!   the measured output velocity
//!
//! That split keeps startup and reversal behavior decisive while making viscous
//! drag compensation track the real actuator speed instead of a fixed
//! pseudo-velocity.
//!
//! # Reference plot
//!
//! Torque-command example with output-side friction parasitics. The
//! compensated controller applies bounded friction feedforward and exposes the
//! torque breakdown for tuning.
#![doc = "\n"]
#![doc = include_str!("../../../docs/plots/closed_loop_torque_command.svg")]
#![doc = r#"
Velocity-command example with output-side friction and attached inertia present
in the actuator model.
"#]
#![doc = "\n"]
#![doc = include_str!("../../../docs/plots/closed_loop_velocity_command.svg")]

use fluxkit_math::{
    ContinuousMechanicalAngle,
    units::{NewtonMeters, RadPerSec},
};

/// Static actuator model that maps motor-shaft behavior to the output axis.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct ActuatorModel {
    /// Mechanical reduction ratio from motor shaft to output axis.
    ///
    /// A value of `5.0` means the motor rotates five turns for one output-axis turn.
    pub gear_ratio: f32,
}

/// Static actuator model, optional compensation, and independent limits.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct ActuatorParams {
    /// Mechanical reduction ratio from motor shaft to output axis.
    ///
    /// A value of `5.0` means the motor rotates five turns for one output-axis turn.
    pub gear_ratio: f32,
    /// Optional calibrated mechanical and load-side compensation model.
    pub compensation: ActuatorCompensationConfig,
    /// Operating limits that are not part of the calibrated actuator model.
    pub limits: ActuatorLimits,
}

/// Safety and operating limits for the actuator output axis.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct ActuatorLimits {
    /// Optional output-axis mechanical speed limit.
    pub max_output_velocity: Option<RadPerSec>,
    /// Optional output-axis continuous torque limit.
    pub max_output_torque: Option<NewtonMeters>,
}

/// Output-axis estimate supplied by platform code.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct ActuatorEstimate {
    /// Wrapped output-axis angle in `[-pi, pi)`.
    pub output_angle: ContinuousMechanicalAngle,
    /// Output-axis mechanical velocity estimate.
    pub output_velocity: RadPerSec,
}

/// Optional mechanical and load-side compensation parameters.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct ActuatorCompensationConfig {
    /// Friction and drag compensation.
    pub friction: FrictionCompensation,
    /// Absolute bound on the summed compensation torque.
    pub max_total_torque: NewtonMeters,
}

impl ActuatorCompensationConfig {
    /// Returns a fully disabled compensation configuration.
    pub const fn disabled() -> Self {
        Self {
            friction: FrictionCompensation::disabled(),
            max_total_torque: NewtonMeters::ZERO,
        }
    }
}

impl ActuatorParams {
    /// Builds actuator parameters from a kinematic model, compensation, and
    /// independent limits.
    #[inline]
    pub const fn from_model_limits_and_compensation(
        model: ActuatorModel,
        limits: ActuatorLimits,
        compensation: ActuatorCompensationConfig,
    ) -> Self {
        Self {
            gear_ratio: model.gear_ratio,
            compensation,
            limits,
        }
    }

    /// Returns the calibrated actuator model portion of these parameters.
    #[inline]
    pub const fn model(&self) -> ActuatorModel {
        ActuatorModel {
            gear_ratio: self.gear_ratio,
        }
    }
}

/// Direction-dependent friction and drag compensation.
///
/// The controller uses these parameters in two regimes:
///
/// - breakaway compensation is guided by commanded direction near zero speed
///   and capped to only fill the missing static-friction margin
/// - Coulomb and viscous compensation transition onto measured output velocity
///   as the actuator starts moving
///
/// `zero_velocity_blend_band` defines the width of that transition.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct FrictionCompensation {
    /// Enables friction and drag compensation.
    pub enabled: bool,
    /// Additional startup torque near zero speed in the positive direction.
    pub positive_breakaway_torque: NewtonMeters,
    /// Additional startup torque near zero speed in the negative direction.
    pub negative_breakaway_torque: NewtonMeters,
    /// Constant friction torque while moving in the positive direction.
    pub positive_coulomb_torque: NewtonMeters,
    /// Constant friction torque while moving in the negative direction.
    pub negative_coulomb_torque: NewtonMeters,
    /// Positive-direction viscous coefficient in `Nm / (rad/s)`.
    pub positive_viscous_coefficient: f32,
    /// Negative-direction viscous coefficient in `Nm / (rad/s)`.
    pub negative_viscous_coefficient: f32,
    /// Smoothing band around zero speed used to avoid discontinuous switching
    /// and to blend from command-guided breakaway behavior into
    /// measurement-driven drag compensation.
    pub zero_velocity_blend_band: RadPerSec,
}

impl FrictionCompensation {
    /// Returns friction compensation disabled with zeroed coefficients.
    pub const fn disabled() -> Self {
        Self {
            enabled: false,
            positive_breakaway_torque: NewtonMeters::ZERO,
            negative_breakaway_torque: NewtonMeters::ZERO,
            positive_coulomb_torque: NewtonMeters::ZERO,
            negative_coulomb_torque: NewtonMeters::ZERO,
            positive_viscous_coefficient: 0.0,
            negative_viscous_coefficient: 0.0,
            zero_velocity_blend_band: RadPerSec::ZERO,
        }
    }
}

/// Per-cycle actuator compensation telemetry.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct ActuatorCompensationTelemetry {
    /// Feedback-produced output torque request before compensation.
    pub feedback_torque: NewtonMeters,
    /// Breakaway-friction contribution after near-zero blending.
    pub breakaway_torque: NewtonMeters,
    /// Coulomb-friction contribution after directional blending.
    pub coulomb_torque: NewtonMeters,
    /// Viscous-drag contribution from measured output velocity.
    pub viscous_torque: NewtonMeters,
    /// Total friction and drag compensation contribution.
    ///
    /// This is the sum of `breakaway_torque`, `coulomb_torque`, and
    /// `viscous_torque`.
    pub friction_torque: NewtonMeters,
    /// Total bounded additive compensation contribution.
    pub total_compensation_torque: NewtonMeters,
    /// Final bounded output torque command after adding compensation.
    pub total_output_torque_command: NewtonMeters,
}

impl ActuatorCompensationTelemetry {
    /// Returns a zeroed telemetry snapshot.
    pub const fn zero() -> Self {
        Self {
            feedback_torque: NewtonMeters::ZERO,
            breakaway_torque: NewtonMeters::ZERO,
            coulomb_torque: NewtonMeters::ZERO,
            viscous_torque: NewtonMeters::ZERO,
            friction_torque: NewtonMeters::ZERO,
            total_compensation_torque: NewtonMeters::ZERO,
            total_output_torque_command: NewtonMeters::ZERO,
        }
    }
}
