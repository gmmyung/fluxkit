//! Actuator-side calibration procedures and persisted records.

pub mod blend_band;
pub mod breakaway;
pub mod friction;
pub mod gear_ratio;
pub mod result;

pub use blend_band::{
    ActuatorBlendBandCalibrationCommand, ActuatorBlendBandCalibrationConfig,
    ActuatorBlendBandCalibrationInput, ActuatorBlendBandCalibrationResult,
    ActuatorBlendBandCalibrator,
};
pub use breakaway::{
    ActuatorBreakawayCalibrationCommand, ActuatorBreakawayCalibrationConfig,
    ActuatorBreakawayCalibrationInput, ActuatorBreakawayCalibrationResult,
    ActuatorBreakawayCalibrator,
};
pub use friction::{
    ActuatorFrictionCalibrationCommand, ActuatorFrictionCalibrationConfig,
    ActuatorFrictionCalibrationInput, ActuatorFrictionCalibrationResult,
    ActuatorFrictionCalibrator,
};
pub use gear_ratio::{
    ActuatorGearRatioCalibrationCommand, ActuatorGearRatioCalibrationConfig,
    ActuatorGearRatioCalibrationInput, ActuatorGearRatioCalibrationResult,
    ActuatorGearRatioCalibrator,
};
pub use result::{ActuatorCalibration, ActuatorFrictionCalibration};
