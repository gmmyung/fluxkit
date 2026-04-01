#![no_std]
#![forbid(unsafe_code)]
#![deny(rust_2018_idioms)]
#![warn(missing_docs, missing_debug_implementations)]
#![doc = include_str!("../README.md")]
pub mod angle;
pub mod control;
pub mod estimation;
pub mod frame;
pub mod modulation;
pub mod ramp;
pub mod saturation;
pub mod scalar;
pub mod transforms;
pub mod trig;
pub mod units;
pub mod util;

pub use angle::{ContinuousMechanicalAngle, ElectricalAngle, ElectricalDirection, MechanicalAngle};
pub use control::pi::{PiConfig, PiController, PiState};
pub use estimation::{
    AngularEstimate, AngularEstimatorSeed, AngularSample, ContinuousAngleValue,
    ContinuousEstimator, EstimatorSeed, LpfEstimator, LpfEstimatorConfig, LpfSignalEstimator,
    LpfWrappedEstimator, MechanicalMotionEstimate, MechanicalMotionSample, MechanicalMotionSeed,
    PassThroughEstimator, PassThroughSignalEstimator, PassThroughWrappedEstimator, PllEstimator,
    PllEstimatorConfig, PllWrappedEstimator, SignalEstimate, SignalSample, WrappedAngleValue,
    WrappedEstimator,
};
pub use frame::{Abc, AlphaBeta, Dq};
pub use modulation::{
    ModulationOutput, Modulator, PhaseDuty, SinePwm, Svpwm, SvpwmResult, dq_q_limit, sine_pwm,
    svpwm,
};
pub use ramp::SlewRateLimiter;
pub use saturation::{clamp, clamp_abs, limit_norm_ab, limit_norm_dq};
pub use transforms::{clarke, inverse_clarke, inverse_park, park};
pub use units::{
    Amps, Celsius, Duty, Henries, Hertz, NewtonMeters, Ohms, RadPerSec, Radians, Seconds, Volts,
    Webers,
};
