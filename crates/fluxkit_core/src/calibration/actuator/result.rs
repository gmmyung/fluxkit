//! Persistable actuator-side calibration record.

use fluxkit_math::units::{NewtonMeters, RadPerSec};

use crate::{
    actuator::{ActuatorParams, FrictionCompensation},
    calibration::actuator,
};

/// Persistable collection of actuator-side calibration values.
///
/// This record is intentionally incremental so later procedures can refine the
/// same actuator parameter surface without overwriting unrelated fields.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct ActuatorFrictionCalibration {
    /// Additional startup torque near zero speed in the positive direction.
    pub positive_breakaway_torque: Option<NewtonMeters>,
    /// Additional startup torque near zero speed in the negative direction.
    pub negative_breakaway_torque: Option<NewtonMeters>,
    /// Constant friction torque while moving in the positive direction.
    pub positive_coulomb_torque: Option<NewtonMeters>,
    /// Constant friction torque while moving in the negative direction.
    pub negative_coulomb_torque: Option<NewtonMeters>,
    /// Positive-direction viscous coefficient in `Nm / (rad/s)`.
    pub positive_viscous_coefficient: Option<f32>,
    /// Negative-direction viscous coefficient in `Nm / (rad/s)`.
    pub negative_viscous_coefficient: Option<f32>,
    /// Smoothing band around zero speed for friction blending.
    pub zero_velocity_blend_band: Option<RadPerSec>,
}

impl ActuatorFrictionCalibration {
    /// Empty friction calibration record with no identified values.
    #[inline]
    pub const fn empty() -> Self {
        Self {
            positive_breakaway_torque: None,
            negative_breakaway_torque: None,
            positive_coulomb_torque: None,
            negative_coulomb_torque: None,
            positive_viscous_coefficient: None,
            negative_viscous_coefficient: None,
            zero_velocity_blend_band: None,
        }
    }

    /// Merges two friction calibration records, preferring values from `newer`.
    #[inline]
    pub const fn merge(self, newer: Self) -> Self {
        Self {
            positive_breakaway_torque: if newer.positive_breakaway_torque.is_some() {
                newer.positive_breakaway_torque
            } else {
                self.positive_breakaway_torque
            },
            negative_breakaway_torque: if newer.negative_breakaway_torque.is_some() {
                newer.negative_breakaway_torque
            } else {
                self.negative_breakaway_torque
            },
            positive_coulomb_torque: if newer.positive_coulomb_torque.is_some() {
                newer.positive_coulomb_torque
            } else {
                self.positive_coulomb_torque
            },
            negative_coulomb_torque: if newer.negative_coulomb_torque.is_some() {
                newer.negative_coulomb_torque
            } else {
                self.negative_coulomb_torque
            },
            positive_viscous_coefficient: if newer.positive_viscous_coefficient.is_some() {
                newer.positive_viscous_coefficient
            } else {
                self.positive_viscous_coefficient
            },
            negative_viscous_coefficient: if newer.negative_viscous_coefficient.is_some() {
                newer.negative_viscous_coefficient
            } else {
                self.negative_viscous_coefficient
            },
            zero_velocity_blend_band: if newer.zero_velocity_blend_band.is_some() {
                newer.zero_velocity_blend_band
            } else {
                self.zero_velocity_blend_band
            },
        }
    }

    /// Applies any populated friction fields onto an existing compensation
    /// struct, preserving unspecified values.
    pub fn apply_to_friction_compensation(&self, friction: &mut FrictionCompensation) {
        if self == &Self::empty() {
            return;
        }

        if let Some(value) = self.positive_breakaway_torque {
            friction.positive_breakaway_torque = value;
        }
        if let Some(value) = self.negative_breakaway_torque {
            friction.negative_breakaway_torque = value;
        }
        if let Some(value) = self.positive_coulomb_torque {
            friction.positive_coulomb_torque = value;
        }
        if let Some(value) = self.negative_coulomb_torque {
            friction.negative_coulomb_torque = value;
        }
        if let Some(value) = self.positive_viscous_coefficient {
            friction.positive_viscous_coefficient = value;
        }
        if let Some(value) = self.negative_viscous_coefficient {
            friction.negative_viscous_coefficient = value;
        }
        if let Some(value) = self.zero_velocity_blend_band {
            friction.zero_velocity_blend_band = value;
        }
    }
}

/// Persistable collection of actuator-side calibration values.
///
/// This record is intentionally incremental so later procedures can refine the
/// same actuator parameter surface without overwriting unrelated fields.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct ActuatorCalibration {
    /// Mechanical reduction ratio from motor shaft to output axis.
    pub gear_ratio: Option<f32>,
    /// Output-side friction calibration surface.
    pub friction: ActuatorFrictionCalibration,
}

impl ActuatorCalibration {
    /// Empty calibration record with no identified values.
    #[inline]
    pub const fn empty() -> Self {
        Self {
            gear_ratio: None,
            friction: ActuatorFrictionCalibration::empty(),
        }
    }

    /// Merges two calibration records, preferring values from `newer`.
    #[inline]
    pub const fn merge(self, newer: Self) -> Self {
        Self {
            gear_ratio: if newer.gear_ratio.is_some() {
                newer.gear_ratio
            } else {
                self.gear_ratio
            },
            friction: self.friction.merge(newer.friction),
        }
    }

    /// Applies any populated fields onto an existing actuator-parameter record.
    ///
    /// This updates the gear ratio and friction coefficients only. It does not
    /// implicitly enable compensation or change the total-torque bound.
    pub fn apply_to_actuator_params(&self, actuator: &mut ActuatorParams) {
        if let Some(value) = self.gear_ratio {
            actuator.gear_ratio = value;
        }
        self.friction
            .apply_to_friction_compensation(&mut actuator.compensation.friction);
    }
}

impl ActuatorParams {
    /// Returns a copy of these parameters with a calibration record applied.
    #[inline]
    pub fn with_calibration(mut self, calibration: &ActuatorCalibration) -> Self {
        calibration.apply_to_actuator_params(&mut self);
        self
    }
}

impl From<actuator::ActuatorGearRatioCalibrationResult> for ActuatorCalibration {
    #[inline]
    fn from(result: actuator::ActuatorGearRatioCalibrationResult) -> Self {
        Self {
            gear_ratio: Some(result.gear_ratio),
            ..Self::empty()
        }
    }
}

impl From<actuator::ActuatorFrictionCalibrationResult> for ActuatorCalibration {
    #[inline]
    fn from(result: actuator::ActuatorFrictionCalibrationResult) -> Self {
        Self {
            friction: ActuatorFrictionCalibration {
                positive_coulomb_torque: Some(result.positive_coulomb_torque),
                negative_coulomb_torque: Some(result.negative_coulomb_torque),
                positive_viscous_coefficient: Some(result.positive_viscous_coefficient),
                negative_viscous_coefficient: Some(result.negative_viscous_coefficient),
                ..ActuatorFrictionCalibration::empty()
            },
            ..Self::empty()
        }
    }
}

impl From<actuator::ActuatorBreakawayCalibrationResult> for ActuatorCalibration {
    #[inline]
    fn from(result: actuator::ActuatorBreakawayCalibrationResult) -> Self {
        Self {
            friction: ActuatorFrictionCalibration {
                positive_breakaway_torque: Some(result.positive_breakaway_torque),
                negative_breakaway_torque: Some(result.negative_breakaway_torque),
                ..ActuatorFrictionCalibration::empty()
            },
            ..Self::empty()
        }
    }
}

impl From<actuator::ActuatorBlendBandCalibrationResult> for ActuatorCalibration {
    #[inline]
    fn from(result: actuator::ActuatorBlendBandCalibrationResult) -> Self {
        Self {
            friction: ActuatorFrictionCalibration {
                zero_velocity_blend_band: Some(result.zero_velocity_blend_band),
                ..ActuatorFrictionCalibration::empty()
            },
            ..Self::empty()
        }
    }
}

#[cfg(test)]
mod tests {
    use fluxkit_math::units::{NewtonMeters, RadPerSec};

    use super::{ActuatorCalibration, ActuatorFrictionCalibration};
    use crate::{
        actuator::{
            ActuatorCompensationConfig, ActuatorLimits, ActuatorParams, FrictionCompensation,
        },
        calibration::{
            ActuatorBlendBandCalibrationResult, ActuatorBreakawayCalibrationResult,
            ActuatorFrictionCalibrationResult, ActuatorGearRatioCalibrationResult,
        },
    };

    #[test]
    fn merge_prefers_newer_populated_fields() {
        let older = ActuatorCalibration {
            gear_ratio: Some(1.0),
            friction: ActuatorFrictionCalibration {
                positive_coulomb_torque: Some(NewtonMeters::new(0.1)),
                ..ActuatorFrictionCalibration::empty()
            },
            ..ActuatorCalibration::empty()
        };
        let newer = ActuatorCalibration {
            gear_ratio: Some(2.0),
            friction: ActuatorFrictionCalibration {
                positive_coulomb_torque: Some(NewtonMeters::new(0.04)),
                positive_viscous_coefficient: Some(0.02),
                ..ActuatorFrictionCalibration::empty()
            },
            ..ActuatorCalibration::empty()
        };

        let merged = older.merge(newer);
        assert_eq!(merged.gear_ratio, Some(2.0));
        assert_eq!(
            merged.friction.positive_coulomb_torque,
            Some(NewtonMeters::new(0.04))
        );
        assert_eq!(merged.friction.positive_viscous_coefficient, Some(0.02));
    }

    #[test]
    fn apply_to_actuator_params_overwrites_only_populated_fields() {
        let mut actuator = ActuatorParams {
            gear_ratio: 2.0,
            compensation: ActuatorCompensationConfig {
                friction: FrictionCompensation {
                    enabled: false,
                    positive_breakaway_torque: NewtonMeters::new(0.2),
                    negative_breakaway_torque: NewtonMeters::new(0.2),
                    positive_coulomb_torque: NewtonMeters::new(0.1),
                    negative_coulomb_torque: NewtonMeters::new(0.1),
                    positive_viscous_coefficient: 0.05,
                    negative_viscous_coefficient: 0.05,
                    zero_velocity_blend_band: RadPerSec::new(0.5),
                },
                max_total_torque: NewtonMeters::new(1.0),
            },
            limits: ActuatorLimits {
                max_output_velocity: None,
                max_output_torque: None,
            },
        };

        ActuatorCalibration {
            gear_ratio: Some(3.0),
            friction: ActuatorFrictionCalibration {
                positive_coulomb_torque: Some(NewtonMeters::new(0.04)),
                negative_viscous_coefficient: Some(0.02),
                ..ActuatorFrictionCalibration::empty()
            },
            ..ActuatorCalibration::empty()
        }
        .apply_to_actuator_params(&mut actuator);

        assert_eq!(actuator.gear_ratio, 3.0);
        assert_eq!(
            actuator.compensation.friction.positive_coulomb_torque,
            NewtonMeters::new(0.04)
        );
        assert_eq!(
            actuator.compensation.friction.negative_viscous_coefficient,
            0.02
        );
        assert_eq!(
            actuator.compensation.friction.positive_breakaway_torque,
            NewtonMeters::new(0.2)
        );
        assert!(!actuator.compensation.friction.enabled);
    }

    #[test]
    fn friction_result_maps_to_actuator_calibration_surface() {
        let calibration: ActuatorCalibration = ActuatorFrictionCalibrationResult {
            positive_coulomb_torque: NewtonMeters::new(0.04),
            negative_coulomb_torque: NewtonMeters::new(0.05),
            positive_viscous_coefficient: 0.02,
            negative_viscous_coefficient: 0.03,
        }
        .into();

        assert_eq!(
            calibration.friction.positive_coulomb_torque,
            Some(NewtonMeters::new(0.04))
        );
        assert_eq!(
            calibration.friction.negative_coulomb_torque,
            Some(NewtonMeters::new(0.05))
        );
        assert_eq!(
            calibration.friction.positive_viscous_coefficient,
            Some(0.02)
        );
        assert_eq!(
            calibration.friction.negative_viscous_coefficient,
            Some(0.03)
        );
    }

    #[test]
    fn gear_ratio_result_maps_to_actuator_calibration_surface() {
        let calibration: ActuatorCalibration =
            ActuatorGearRatioCalibrationResult { gear_ratio: 2.0 }.into();

        assert_eq!(calibration.gear_ratio, Some(2.0));
    }

    #[test]
    fn breakaway_result_maps_to_actuator_calibration_surface() {
        let calibration: ActuatorCalibration = ActuatorBreakawayCalibrationResult {
            positive_breakaway_torque: NewtonMeters::new(0.08),
            negative_breakaway_torque: NewtonMeters::new(0.09),
        }
        .into();

        assert_eq!(
            calibration.friction.positive_breakaway_torque,
            Some(NewtonMeters::new(0.08))
        );
        assert_eq!(
            calibration.friction.negative_breakaway_torque,
            Some(NewtonMeters::new(0.09))
        );
    }

    #[test]
    fn blend_band_result_maps_to_actuator_calibration_surface() {
        let calibration: ActuatorCalibration = ActuatorBlendBandCalibrationResult {
            zero_velocity_blend_band: RadPerSec::new(0.05),
        }
        .into();

        assert_eq!(
            calibration.friction.zero_velocity_blend_band,
            Some(RadPerSec::new(0.05))
        );
    }
}
