//! Motor-side calibration procedures and persisted records.

pub mod flux_linkage;
pub mod phase_inductance;
pub mod phase_resistance;
pub mod pole_pairs_and_offset;
pub mod result;

pub use flux_linkage::{
    FluxLinkageCalibrationConfig, FluxLinkageCalibrationInput, FluxLinkageCalibrationResult,
    FluxLinkageCalibrator,
};
pub use phase_inductance::{
    PhaseInductanceCalibrationConfig, PhaseInductanceCalibrationInput,
    PhaseInductanceCalibrationResult, PhaseInductanceCalibrator,
};
pub use phase_resistance::{
    PhaseResistanceCalibrationConfig, PhaseResistanceCalibrationInput,
    PhaseResistanceCalibrationResult, PhaseResistanceCalibrator,
};
pub use pole_pairs_and_offset::{
    PolePairsAndOffsetCalibrationConfig, PolePairsAndOffsetCalibrationInput,
    PolePairsAndOffsetCalibrationResult, PolePairsAndOffsetCalibrator,
};
pub use result::MotorCalibration;
