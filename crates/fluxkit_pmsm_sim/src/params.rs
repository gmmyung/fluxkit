//! Static PMSM plant parameters.

use fluxkit_math::units::{Henries, NewtonMeters, Ohms, RadPerSec, Volts, Webers};

/// Static parameters for an ideal PMSM plant model.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct PmsmParams {
    /// Number of electrical pole pairs.
    pub pole_pairs: u8,
    /// Phase resistance normalized to the winding reference temperature of
    /// `25°C`.
    pub phase_resistance_ohm_ref: Ohms,
    /// `d`-axis inductance.
    pub d_inductance_h: Henries,
    /// `q`-axis inductance.
    pub q_inductance_h: Henries,
    /// Permanent-magnet flux linkage.
    pub flux_linkage_weber: Webers,
    /// Lumped winding thermal model.
    pub thermal: ThermalPlantParams,
    /// Combined mechanical drivetrain model, including rotor-side and
    /// output-side dynamics.
    pub actuator: ActuatorPlantParams,
    /// Optional magnitude clamp for the applied stator-voltage vector.
    ///
    /// This limit is applied as a circular norm limit in the rotating `d/q`
    /// frame, regardless of whether the caller steps the model with `d/q`,
    /// `alpha/beta`, phase voltage, or duty input.
    pub max_voltage_mag: Option<Volts>,
}

/// Lumped winding thermal model.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct ThermalPlantParams {
    /// Ambient temperature in `°C`.
    pub ambient_temperature_c: f32,
    /// Lumped winding heat capacity in `J / °C`.
    pub winding_thermal_capacity_j_per_c: f32,
    /// Lumped thermal conductance from winding to ambient in `W / °C`.
    pub winding_thermal_conductance_w_per_c: f32,
}

impl ThermalPlantParams {
    /// Returns a simple passive winding thermal model initialized at the given
    /// ambient temperature.
    pub const fn default_for_ambient(ambient_temperature_c: f32) -> Self {
        Self {
            ambient_temperature_c,
            winding_thermal_capacity_j_per_c: 100.0,
            winding_thermal_conductance_w_per_c: 0.1,
        }
    }
}

/// Output-side actuator reduction, inertia, and friction model.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct ActuatorPlantParams {
    /// Mechanical reduction ratio from motor shaft to output axis.
    pub gear_ratio: f32,
    /// Total equivalent inertia expressed on the output side in `kg·m²`.
    pub output_inertia_kg_m2: f32,
    /// Additional startup torque near zero speed in the positive direction.
    pub positive_breakaway_torque: NewtonMeters,
    /// Additional startup torque near zero speed in the negative direction.
    pub negative_breakaway_torque: NewtonMeters,
    /// Constant friction torque while moving in the positive direction.
    pub positive_coulomb_torque: NewtonMeters,
    /// Constant friction torque while moving in the negative direction.
    pub negative_coulomb_torque: NewtonMeters,
    /// Positive-direction viscous coefficient in `Nm / (rad/s)`.
    pub positive_viscous_coefficient: f32,
    /// Negative-direction viscous coefficient in `Nm / (rad/s)`.
    pub negative_viscous_coefficient: f32,
    /// Smoothing band around zero output speed.
    pub zero_velocity_blend_band: RadPerSec,
}

impl ActuatorPlantParams {
    /// Returns a zero-friction actuator model with unity reduction.
    pub const fn disabled() -> Self {
        Self {
            gear_ratio: 1.0,
            output_inertia_kg_m2: 0.0,
            positive_breakaway_torque: NewtonMeters::ZERO,
            negative_breakaway_torque: NewtonMeters::ZERO,
            positive_coulomb_torque: NewtonMeters::ZERO,
            negative_coulomb_torque: NewtonMeters::ZERO,
            positive_viscous_coefficient: 0.0,
            negative_viscous_coefficient: 0.0,
            zero_velocity_blend_band: RadPerSec::ZERO,
        }
    }

    /// Returns the total equivalent output-side inertia in `kg·m²`.
    #[inline]
    pub const fn total_output_inertia_kg_m2(&self) -> f32 {
        self.output_inertia_kg_m2
    }

    /// Returns the total output-side inertia reflected to the motor shaft.
    #[inline]
    pub fn reflected_inertia_kg_m2(&self) -> f32 {
        let gear_ratio = self.gear_ratio.max(f32::EPSILON);
        self.output_inertia_kg_m2 / (gear_ratio * gear_ratio)
    }
}
