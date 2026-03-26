//! Persistable motor-side calibration record.

use fluxkit_math::{
    ElectricalAngle,
    units::{Henries, Ohms, Webers},
};

use crate::{calibration::motor, params::MotorParams};

/// Persistable collection of motor-side calibration values.
///
/// This record is intentionally incremental: early procedures populate only a
/// subset of fields, and later procedures merge additional estimates in.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct MotorCalibration {
    /// Estimated electrical pole-pair count.
    pub pole_pairs: Option<u8>,
    /// Electrical zero offset after mechanical-to-electrical conversion.
    pub electrical_angle_offset: Option<ElectricalAngle>,
    /// Estimated phase resistance normalized to `25°C`.
    pub phase_resistance_ohm_ref: Option<Ohms>,
    /// Estimated phase inductance applied to both `d` and `q` axes.
    pub phase_inductance_h: Option<Henries>,
    /// Estimated flux linkage.
    pub flux_linkage_weber: Option<Webers>,
}

impl MotorCalibration {
    /// Empty calibration record with no identified values.
    #[inline]
    pub const fn empty() -> Self {
        Self {
            pole_pairs: None,
            electrical_angle_offset: None,
            phase_resistance_ohm_ref: None,
            phase_inductance_h: None,
            flux_linkage_weber: None,
        }
    }

    /// Merges two calibration records, preferring values from `newer`.
    #[inline]
    pub const fn merge(self, newer: Self) -> Self {
        Self {
            pole_pairs: if newer.pole_pairs.is_some() {
                newer.pole_pairs
            } else {
                self.pole_pairs
            },
            electrical_angle_offset: if newer.electrical_angle_offset.is_some() {
                newer.electrical_angle_offset
            } else {
                self.electrical_angle_offset
            },
            phase_resistance_ohm_ref: if newer.phase_resistance_ohm_ref.is_some() {
                newer.phase_resistance_ohm_ref
            } else {
                self.phase_resistance_ohm_ref
            },
            phase_inductance_h: if newer.phase_inductance_h.is_some() {
                newer.phase_inductance_h
            } else {
                self.phase_inductance_h
            },
            flux_linkage_weber: if newer.flux_linkage_weber.is_some() {
                newer.flux_linkage_weber
            } else {
                self.flux_linkage_weber
            },
        }
    }

    /// Applies any populated fields onto an existing motor-parameter record.
    pub fn apply_to_motor_params(&self, motor: &mut MotorParams) {
        if let Some(pole_pairs) = self.pole_pairs {
            motor.pole_pairs = pole_pairs;
        }
        if let Some(offset) = self.electrical_angle_offset {
            motor.electrical_angle_offset = offset;
        }
        if let Some(resistance) = self.phase_resistance_ohm_ref {
            motor.phase_resistance_ohm_ref = resistance;
        }
        if let Some(inductance) = self.phase_inductance_h {
            motor.d_inductance_h = inductance;
            motor.q_inductance_h = inductance;
        }
        if let Some(flux_linkage) = self.flux_linkage_weber {
            motor.flux_linkage_weber = flux_linkage;
        }
    }
}

impl MotorParams {
    /// Returns a copy of these parameters with a calibration record applied.
    #[inline]
    pub fn with_calibration(mut self, calibration: &MotorCalibration) -> Self {
        calibration.apply_to_motor_params(&mut self);
        self
    }
}

impl From<motor::PolePairsAndOffsetCalibrationResult> for MotorCalibration {
    #[inline]
    fn from(result: motor::PolePairsAndOffsetCalibrationResult) -> Self {
        Self {
            pole_pairs: Some(result.pole_pairs),
            electrical_angle_offset: Some(result.electrical_angle_offset),
            ..Self::empty()
        }
    }
}

impl From<motor::PhaseResistanceCalibrationResult> for MotorCalibration {
    #[inline]
    fn from(result: motor::PhaseResistanceCalibrationResult) -> Self {
        Self {
            phase_resistance_ohm_ref: Some(result.phase_resistance_ohm_ref),
            ..Self::empty()
        }
    }
}

impl From<motor::PhaseInductanceCalibrationResult> for MotorCalibration {
    #[inline]
    fn from(result: motor::PhaseInductanceCalibrationResult) -> Self {
        Self {
            phase_inductance_h: Some(result.phase_inductance_h),
            ..Self::empty()
        }
    }
}

impl From<motor::FluxLinkageCalibrationResult> for MotorCalibration {
    #[inline]
    fn from(result: motor::FluxLinkageCalibrationResult) -> Self {
        Self {
            flux_linkage_weber: Some(result.flux_linkage_weber),
            ..Self::empty()
        }
    }
}

#[cfg(test)]
mod tests {
    use fluxkit_math::{
        ElectricalAngle,
        units::{Amps, Henries, Ohms, Webers},
    };

    use super::MotorCalibration;
    use crate::{
        FluxLinkageCalibrationResult, MotorLimits, PhaseInductanceCalibrationResult,
        params::MotorParams,
    };

    #[test]
    fn merge_prefers_newer_populated_fields() {
        let older = MotorCalibration {
            pole_pairs: Some(7),
            phase_resistance_ohm_ref: Some(Ohms::new(0.2)),
            ..MotorCalibration::empty()
        };
        let newer = MotorCalibration {
            phase_resistance_ohm_ref: Some(Ohms::new(0.12)),
            electrical_angle_offset: Some(ElectricalAngle::new(0.4)),
            ..MotorCalibration::empty()
        };

        let merged = older.merge(newer);
        assert_eq!(merged.pole_pairs, Some(7));
        assert_eq!(merged.phase_resistance_ohm_ref, Some(Ohms::new(0.12)));
        assert_eq!(
            merged.electrical_angle_offset,
            Some(ElectricalAngle::new(0.4))
        );
    }

    #[test]
    fn apply_to_motor_params_overwrites_only_populated_fields() {
        let mut params = MotorParams::from_model_and_limits(
            crate::params::MotorModel {
                pole_pairs: 4,
                phase_resistance_ohm_ref: Ohms::new(0.2),
                d_inductance_h: fluxkit_math::units::Henries::new(0.001),
                q_inductance_h: fluxkit_math::units::Henries::new(0.001),
                flux_linkage_weber: Webers::new(0.001),
                electrical_angle_offset: ElectricalAngle::new(0.0),
            },
            MotorLimits {
                max_phase_current: Amps::new(10.0),
                max_mech_speed: None,
            },
        );

        MotorCalibration {
            pole_pairs: Some(7),
            electrical_angle_offset: Some(ElectricalAngle::new(0.3)),
            phase_resistance_ohm_ref: Some(Ohms::new(0.12)),
            ..MotorCalibration::empty()
        }
        .apply_to_motor_params(&mut params);

        assert_eq!(params.pole_pairs, 7);
        assert_eq!(params.electrical_angle_offset, ElectricalAngle::new(0.3));
        assert_eq!(params.phase_resistance_ohm_ref, Ohms::new(0.12));
    }

    #[test]
    fn phase_inductance_result_maps_to_equal_d_and_q_inductance() {
        let calibration: MotorCalibration = PhaseInductanceCalibrationResult {
            phase_inductance_h: Henries::new(30.0e-6),
        }
        .into();

        assert_eq!(calibration.phase_inductance_h, Some(Henries::new(30.0e-6)));
    }

    #[test]
    fn flux_linkage_result_maps_to_flux_linkage_field() {
        let calibration: MotorCalibration = FluxLinkageCalibrationResult {
            flux_linkage_weber: Webers::new(0.005),
        }
        .into();

        assert_eq!(calibration.flux_linkage_weber, Some(Webers::new(0.005)));
    }
}
