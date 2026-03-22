//! Static motor and inverter parameter types.

use fluxkit_math::{
    ElectricalAngle,
    units::{Amps, Duty, Henries, Hertz, Ohms, RadPerSec, Volts, Webers},
};

/// Electrical and mechanical parameters of the controlled motor.
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
    /// Optional flux-linkage estimate.
    pub flux_linkage_weber: Option<Webers>,
    /// Electrical zero offset applied after converting mechanical angle using
    /// `pole_pairs`.
    pub electrical_angle_offset: ElectricalAngle,
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
