#![no_std]
#![forbid(unsafe_code)]
#![deny(rust_2018_idioms)]
#![warn(missing_docs, missing_debug_implementations)]
//! Deterministic `no_std` math primitives for field-oriented control (FOC).
//!
//! `fluxkit_math` is the convention-locked mathematical foundation for the larger
//! motor-control stack. It is designed for interrupt-driven embedded control
//! loops and host-side simulation using the same code paths.
//!
//! # Conventions
//!
//! - Clarke transform: amplitude-invariant, balanced three-phase form.
//! - Park transform: `d` axis aligned with the positive electrical angle.
//! - Positive `q`: standard PMSM torque-producing direction under the chosen
//!   Park sign convention.
//! - Angle wrapping helpers:
//!   - [`angle::wrap_pm_pi`] returns angles in `[-pi, pi)`.
//!   - [`angle::wrap_0_2pi`] returns angles in `[0, 2pi)`.
//! - SVPWM duties are normalized to `[0.0, 1.0]`.
//! - PI anti-windup uses bounded integrator clamping/back-calculation so the
//!   stored integrator remains consistent with the saturated output.
//!
//! # Numerical assumptions
//!
//! Public APIs are concrete over `f32` in the MVP. Trigonometric functions are
//! isolated in [`trig`] and currently use `libm`, which keeps the crate usable
//! in `no_std` builds without tying it to a particular MCU or DSP backend.

pub mod angle;
pub mod control;
pub mod filter;
pub mod frame;
pub mod modulation;
pub mod ramp;
pub mod saturation;
pub mod scalar;
pub mod transforms;
pub mod trig;
pub mod units;
pub mod util;

pub use angle::{ElectricalAngle, MechanicalAngle};
pub use control::pi::{PiConfig, PiController, PiState};
pub use filter::lpf::LowPassFilter;
pub use frame::{Abc, AlphaBeta, Dq};
pub use modulation::{
    ModulationOutput, Modulator, PhaseDuty, SinePwm, Svpwm, SvpwmResult, dq_q_limit, sine_pwm,
    spwm_linear_limit, svpwm, svpwm_linear_limit,
};
pub use ramp::SlewRateLimiter;
pub use saturation::{clamp, clamp_abs, limit_norm_ab, limit_norm_dq};
pub use transforms::{clarke, inverse_clarke, inverse_park, park};
pub use units::{
    Amps, Celsius, Duty, Henries, Hertz, NewtonMeters, Ohms, RadPerSec, Radians, Seconds, Volts,
    Webers,
};
