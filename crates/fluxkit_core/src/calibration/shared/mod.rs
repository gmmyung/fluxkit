//! Shared calibration glue types.

pub mod error;
pub mod release_ramp;
pub mod routine;
pub mod timing;

pub use error::CalibrationError;
pub use routine::{ActuatorCalibrationRoutine, MotorCalibrationRoutine};
