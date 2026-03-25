//! Generic estimator primitives and signal-specific estimator families.

mod angular;
mod internal;
mod signal;

pub use angular::{
    AngularEstimate, AngularEstimatorSeed, AngularSample, ContinuousAngleValue, LpfEstimator,
    LpfEstimatorConfig, LpfWrappedEstimator, MechanicalMotionEstimate, MechanicalMotionSample,
    MechanicalMotionSeed, PassThroughEstimator, PassThroughWrappedEstimator, PllEstimator,
    PllEstimatorConfig, PllWrappedEstimator, WrappedAngleValue, WrappedEstimator,
};
pub use signal::{
    ContinuousEstimator, EstimatorSeed, LpfSignalEstimator, PassThroughSignalEstimator,
    SignalEstimate, SignalSample,
};
