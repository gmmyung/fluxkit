//! Unified calibration routine enums.

use super::{
    ActuatorBlendBandCalibrationResult, ActuatorBlendBandCalibrator,
    ActuatorBreakawayCalibrationResult, ActuatorBreakawayCalibrator, ActuatorCalibration,
    ActuatorFrictionCalibrationResult, ActuatorFrictionCalibrator,
    ActuatorGearRatioCalibrationResult, ActuatorGearRatioCalibrator, CalibrationError,
    FluxLinkageCalibrationResult, FluxLinkageCalibrator, MotorCalibration,
    PhaseInductanceCalibrationResult, PhaseInductanceCalibrator, PhaseResistanceCalibrationResult,
    PhaseResistanceCalibrator, PolePairsAndOffsetCalibrationResult, PolePairsAndOffsetCalibrator,
};

/// One of the supported motor-side calibration routines.
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum MotorCalibrationRoutine {
    /// Pole-pair and electrical-angle-offset identification.
    PolePairsAndOffset(PolePairsAndOffsetCalibrator),
    /// Phase-resistance identification.
    PhaseResistance(PhaseResistanceCalibrator),
    /// Phase-inductance identification.
    PhaseInductance(PhaseInductanceCalibrator),
    /// Flux-linkage identification.
    FluxLinkage(FluxLinkageCalibrator),
}

/// Completed result of a motor-side calibration routine.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum MotorCalibrationRoutineResult {
    /// Pole-pair and electrical-angle-offset result.
    PolePairsAndOffset(PolePairsAndOffsetCalibrationResult),
    /// Phase-resistance result.
    PhaseResistance(PhaseResistanceCalibrationResult),
    /// Phase-inductance result.
    PhaseInductance(PhaseInductanceCalibrationResult),
    /// Flux-linkage result.
    FluxLinkage(FluxLinkageCalibrationResult),
}

impl MotorCalibrationRoutine {
    /// Returns the finished result when the active routine has succeeded.
    pub fn result(&self) -> Option<MotorCalibrationRoutineResult> {
        match self {
            Self::PolePairsAndOffset(calibrator) => calibrator
                .result()
                .map(MotorCalibrationRoutineResult::PolePairsAndOffset),
            Self::PhaseResistance(calibrator) => calibrator
                .result()
                .map(MotorCalibrationRoutineResult::PhaseResistance),
            Self::PhaseInductance(calibrator) => calibrator
                .result()
                .map(MotorCalibrationRoutineResult::PhaseInductance),
            Self::FluxLinkage(calibrator) => calibrator
                .result()
                .map(MotorCalibrationRoutineResult::FluxLinkage),
        }
    }

    /// Returns the failure cause when the active routine has failed.
    pub fn error(&self) -> Option<CalibrationError> {
        match self {
            Self::PolePairsAndOffset(calibrator) => calibrator.error(),
            Self::PhaseResistance(calibrator) => calibrator.error(),
            Self::PhaseInductance(calibrator) => calibrator.error(),
            Self::FluxLinkage(calibrator) => calibrator.error(),
        }
    }
}

impl From<MotorCalibrationRoutineResult> for MotorCalibration {
    fn from(result: MotorCalibrationRoutineResult) -> Self {
        match result {
            MotorCalibrationRoutineResult::PolePairsAndOffset(result) => result.into(),
            MotorCalibrationRoutineResult::PhaseResistance(result) => result.into(),
            MotorCalibrationRoutineResult::PhaseInductance(result) => result.into(),
            MotorCalibrationRoutineResult::FluxLinkage(result) => result.into(),
        }
    }
}

/// One of the supported actuator-side calibration routines.
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ActuatorCalibrationRoutine {
    /// Gear-ratio identification.
    GearRatio(ActuatorGearRatioCalibrator),
    /// Coulomb and viscous friction identification.
    Friction(ActuatorFrictionCalibrator),
    /// Breakaway-torque identification.
    Breakaway(ActuatorBreakawayCalibrator),
    /// Zero-velocity blend-band identification.
    BlendBand(ActuatorBlendBandCalibrator),
}

/// Completed result of an actuator-side calibration routine.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum ActuatorCalibrationRoutineResult {
    /// Gear-ratio result.
    GearRatio(ActuatorGearRatioCalibrationResult),
    /// Coulomb and viscous friction result.
    Friction(ActuatorFrictionCalibrationResult),
    /// Breakaway result.
    Breakaway(ActuatorBreakawayCalibrationResult),
    /// Zero-velocity blend-band result.
    BlendBand(ActuatorBlendBandCalibrationResult),
}

impl ActuatorCalibrationRoutine {
    /// Returns the finished result when the active routine has succeeded.
    pub fn result(&self) -> Option<ActuatorCalibrationRoutineResult> {
        match self {
            Self::GearRatio(calibrator) => calibrator
                .result()
                .map(ActuatorCalibrationRoutineResult::GearRatio),
            Self::Friction(calibrator) => calibrator
                .result()
                .map(ActuatorCalibrationRoutineResult::Friction),
            Self::Breakaway(calibrator) => calibrator
                .result()
                .map(ActuatorCalibrationRoutineResult::Breakaway),
            Self::BlendBand(calibrator) => calibrator
                .result()
                .map(ActuatorCalibrationRoutineResult::BlendBand),
        }
    }

    /// Returns the failure cause when the active routine has failed.
    pub fn error(&self) -> Option<CalibrationError> {
        match self {
            Self::GearRatio(calibrator) => calibrator.error(),
            Self::Friction(calibrator) => calibrator.error(),
            Self::Breakaway(calibrator) => calibrator.error(),
            Self::BlendBand(calibrator) => calibrator.error(),
        }
    }
}

impl From<ActuatorCalibrationRoutineResult> for ActuatorCalibration {
    fn from(result: ActuatorCalibrationRoutineResult) -> Self {
        match result {
            ActuatorCalibrationRoutineResult::GearRatio(result) => result.into(),
            ActuatorCalibrationRoutineResult::Friction(result) => result.into(),
            ActuatorCalibrationRoutineResult::Breakaway(result) => result.into(),
            ActuatorCalibrationRoutineResult::BlendBand(result) => result.into(),
        }
    }
}
