//! Angle wrappers and wrapping helpers.

use crate::scalar::{PI, TAU};
use crate::units::Radians;

/// Electrical angle in radians.
#[derive(Clone, Copy, Debug, PartialEq, PartialOrd, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct ElectricalAngle(pub Radians);

impl ElectricalAngle {
    /// Creates a new electrical angle.
    #[inline]
    pub const fn new(value: f32) -> Self {
        Self(Radians::new(value))
    }

    /// Creates a new electrical angle from a `Radians` wrapper.
    #[inline]
    pub const fn from_radians(value: Radians) -> Self {
        Self(value)
    }

    /// Returns the wrapped scalar value.
    #[inline]
    pub const fn get(self) -> f32 {
        self.0.get()
    }

    /// Returns the underlying `Radians` wrapper.
    #[inline]
    pub const fn radians(self) -> Radians {
        self.0
    }

    /// Returns the angle wrapped into `[-pi, pi)`.
    #[inline]
    pub fn wrapped_pm_pi(self) -> Self {
        Self::new(wrap_pm_pi(self.get()))
    }

    /// Returns the angle wrapped into `[0, 2pi)`.
    #[inline]
    pub fn wrapped_0_2pi(self) -> Self {
        Self::new(wrap_0_2pi(self.get()))
    }
}

/// Mechanical angle in radians.
#[derive(Clone, Copy, Debug, PartialEq, PartialOrd, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct MechanicalAngle(pub Radians);

impl MechanicalAngle {
    /// Creates a new mechanical angle.
    #[inline]
    pub const fn new(value: f32) -> Self {
        Self(Radians::new(value))
    }

    /// Creates a new mechanical angle from a `Radians` wrapper.
    #[inline]
    pub const fn from_radians(value: Radians) -> Self {
        Self(value)
    }

    /// Returns the wrapped scalar value.
    #[inline]
    pub const fn get(self) -> f32 {
        self.0.get()
    }

    /// Returns the underlying `Radians` wrapper.
    #[inline]
    pub const fn radians(self) -> Radians {
        self.0
    }

    /// Returns the angle wrapped into `[-pi, pi)`.
    #[inline]
    pub fn wrapped_pm_pi(self) -> Self {
        Self::new(wrap_pm_pi(self.get()))
    }

    /// Returns the angle wrapped into `[0, 2pi)`.
    #[inline]
    pub fn wrapped_0_2pi(self) -> Self {
        Self::new(wrap_0_2pi(self.get()))
    }
}

impl From<Radians> for ElectricalAngle {
    #[inline]
    fn from(value: Radians) -> Self {
        Self::from_radians(value)
    }
}

impl From<ElectricalAngle> for Radians {
    #[inline]
    fn from(value: ElectricalAngle) -> Self {
        value.radians()
    }
}

impl From<Radians> for MechanicalAngle {
    #[inline]
    fn from(value: Radians) -> Self {
        Self::from_radians(value)
    }
}

impl From<MechanicalAngle> for Radians {
    #[inline]
    fn from(value: MechanicalAngle) -> Self {
        value.radians()
    }
}

/// Wraps an angle into the canonical interval `[-pi, pi)`.
#[inline]
pub fn wrap_pm_pi(x: f32) -> f32 {
    let mut y = x % TAU;
    if y >= PI {
        y -= TAU;
    }
    if y < -PI {
        y += TAU;
    }
    y
}

/// Wraps an angle into the canonical interval `[0, 2pi)`.
#[inline]
pub fn wrap_0_2pi(x: f32) -> f32 {
    let mut y = x % TAU;
    if y < 0.0 {
        y += TAU;
    }
    y
}

/// Returns the shortest signed delta from `from` to `to` in `[-pi, pi)`.
#[inline]
pub fn shortest_angle_delta(from: f32, to: f32) -> f32 {
    wrap_pm_pi(to - from)
}

/// Converts a mechanical angle to an electrical angle using `pole_pairs`.
///
/// When `pole_pairs == 0`, returns zero and triggers a debug assertion in
/// debug builds because the conversion domain is invalid.
#[inline]
pub fn mechanical_to_electrical(theta_mech: MechanicalAngle, pole_pairs: u32) -> ElectricalAngle {
    debug_assert!(pole_pairs > 0, "pole_pairs must be non-zero");
    if pole_pairs == 0 {
        return ElectricalAngle::new(0.0);
    }
    ElectricalAngle::from_radians(theta_mech.radians() * pole_pairs as f32)
}

/// Converts an electrical angle to a mechanical angle using `pole_pairs`.
///
/// When `pole_pairs == 0`, returns zero and triggers a debug assertion in
/// debug builds because the conversion domain is invalid.
#[inline]
pub fn electrical_to_mechanical(theta_elec: ElectricalAngle, pole_pairs: u32) -> MechanicalAngle {
    debug_assert!(pole_pairs > 0, "pole_pairs must be non-zero");
    if pole_pairs == 0 {
        return MechanicalAngle::new(0.0);
    }
    MechanicalAngle::from_radians(theta_elec.radians() / pole_pairs as f32)
}

#[cfg(test)]
mod tests {
    use super::{
        ElectricalAngle, MechanicalAngle, electrical_to_mechanical, mechanical_to_electrical,
        shortest_angle_delta, wrap_0_2pi, wrap_pm_pi,
    };
    use crate::scalar::{PI, TAU};
    use crate::units::Radians;

    fn approx_eq(a: f32, b: f32) {
        assert!((a - b).abs() < 1.0e-6, "{a} != {b}");
    }

    #[test]
    fn wrap_helpers_match_expected_intervals() {
        approx_eq(wrap_pm_pi(3.5 * PI), -0.5 * PI);
        approx_eq(wrap_pm_pi(-3.5 * PI), 0.5 * PI);
        approx_eq(wrap_0_2pi(-0.25 * TAU), 0.75 * TAU);
        approx_eq(wrap_0_2pi(2.25 * TAU), 0.25 * TAU);
    }

    #[test]
    fn shortest_delta_uses_wrapped_difference() {
        approx_eq(shortest_angle_delta(0.9 * PI, -0.9 * PI), -1.8 * PI + TAU);
    }

    #[test]
    fn mechanical_and_electrical_conversion_is_explicit() {
        let mech = MechanicalAngle::new(1.25);
        let elec = mechanical_to_electrical(mech, 7);

        approx_eq(elec.get(), 8.75);
        approx_eq(electrical_to_mechanical(elec, 7).get(), mech.get());
        approx_eq(ElectricalAngle::new(4.0 * PI).wrapped_pm_pi().get(), 0.0);
        assert_eq!(
            ElectricalAngle::from_radians(Radians::new(1.0)).radians(),
            Radians::new(1.0)
        );
    }
}
