//! Unified calibration routine enums.

use crate::calibration::{
    actuator::{
        ActuatorBlendBandCalibrator, ActuatorBreakawayCalibrator, ActuatorCalibration,
        ActuatorFrictionCalibrator, ActuatorGearRatioCalibrator,
    },
    motor::{
        FluxLinkageCalibrator, MotorCalibration, PhaseInductanceCalibrator,
        PhaseResistanceCalibrator, PolePairsAndOffsetCalibrator,
    },
    shared::error::CalibrationError,
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

impl MotorCalibrationRoutine {
    /// Returns the finished calibration delta when the active routine has succeeded.
    pub fn result(&self) -> Option<MotorCalibration> {
        match self {
            Self::PolePairsAndOffset(calibrator) => calibrator.result().map(Into::into),
            Self::PhaseResistance(calibrator) => calibrator.result().map(Into::into),
            Self::PhaseInductance(calibrator) => calibrator.result().map(Into::into),
            Self::FluxLinkage(calibrator) => calibrator.result().map(Into::into),
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

impl ActuatorCalibrationRoutine {
    /// Returns the finished calibration delta when the active routine has succeeded.
    pub fn result(&self) -> Option<ActuatorCalibration> {
        match self {
            Self::GearRatio(calibrator) => calibrator.result().map(Into::into),
            Self::Friction(calibrator) => calibrator.result().map(Into::into),
            Self::Breakaway(calibrator) => calibrator.result().map(Into::into),
            Self::BlendBand(calibrator) => calibrator.result().map(Into::into),
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
