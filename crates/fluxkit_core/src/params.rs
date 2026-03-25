//! Static motor and inverter parameter types.

use fluxkit_math::{
    ElectricalAngle,
    units::{Amps, Duty, Henries, Hertz, Ohms, RadPerSec, Volts, Webers},
};

/// Electrical motor model used by the controller.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct MotorModel {
    /// Number of electrical pole pairs.
    pub pole_pairs: u8,
    /// Phase resistance.
    pub phase_resistance_ohm: Ohms,
    /// `d`-axis inductance.
    pub d_inductance_h: Henries,
    /// `q`-axis inductance.
    pub q_inductance_h: Henries,
    /// Flux-linkage estimate.
    pub flux_linkage_weber: Webers,
    /// Electrical zero offset applied after converting mechanical angle using
    /// `pole_pairs`.
    pub electrical_angle_offset: ElectricalAngle,
}

/// Electrical motor parameters plus operating limits.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct MotorParams {
    /// Number of electrical pole pairs.
    pub pole_pairs: u8,
    /// Phase resistance.
    pub phase_resistance_ohm: Ohms,
    /// `d`-axis inductance.
    pub d_inductance_h: Henries,
    /// `q`-axis inductance.
    pub q_inductance_h: Henries,
    /// Flux-linkage estimate.
    pub flux_linkage_weber: Webers,
    /// Electrical zero offset applied after converting mechanical angle using
    /// `pole_pairs`.
    pub electrical_angle_offset: ElectricalAngle,
    /// Operating limits that are not part of the calibrated motor model.
    pub limits: MotorLimits,
}

/// Safety and operating limits for the controlled motor.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct MotorLimits {
    /// Absolute phase-current operating limit.
    pub max_phase_current: Amps,
    /// Optional mechanical speed limit.
    pub max_mech_speed: Option<RadPerSec>,
}

/// Electrical limits and modulation constraints of the inverter.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct InverterParams {
    /// PWM carrier frequency.
    pub pwm_frequency_hz: Hertz,
    /// Minimum allowed normalized duty.
    pub min_duty: Duty,
    /// Maximum allowed normalized duty.
    pub max_duty: Duty,
    /// Minimum valid DC bus voltage.
    pub min_bus_voltage: Volts,
    /// Maximum valid DC bus voltage.
    pub max_bus_voltage: Volts,
    /// Absolute voltage magnitude limit accepted from the controller.
    pub max_voltage_command: Volts,
}

impl MotorParams {
    /// Builds motor parameters from an electrical model plus independent limits.
    #[inline]
    pub const fn from_model_and_limits(model: MotorModel, limits: MotorLimits) -> Self {
        Self {
            pole_pairs: model.pole_pairs,
            phase_resistance_ohm: model.phase_resistance_ohm,
            d_inductance_h: model.d_inductance_h,
            q_inductance_h: model.q_inductance_h,
            flux_linkage_weber: model.flux_linkage_weber,
            electrical_angle_offset: model.electrical_angle_offset,
            limits,
        }
    }

    /// Returns the electrical motor model portion of these parameters.
    #[inline]
    pub const fn model(&self) -> MotorModel {
        MotorModel {
            pole_pairs: self.pole_pairs,
            phase_resistance_ohm: self.phase_resistance_ohm,
            d_inductance_h: self.d_inductance_h,
            q_inductance_h: self.q_inductance_h,
            flux_linkage_weber: self.flux_linkage_weber,
            electrical_angle_offset: self.electrical_angle_offset,
        }
    }
}
