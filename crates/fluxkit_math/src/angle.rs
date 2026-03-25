//! Angle wrappers and wrapping helpers.

use crate::scalar::{PI, TAU};
use crate::units::Radians;

/// Electrical angle in radians, wrapped into `[-pi, pi)` by construction.
#[derive(Clone, Copy, Debug, PartialEq, PartialOrd, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct ElectricalAngle(pub Radians);

impl ElectricalAngle {
    /// Creates a new electrical angle.
    #[inline]
    pub fn new(value: f32) -> Self {
        Self(Radians::new(wrap(value)))
    }

    /// Creates a new electrical angle from a `Radians` wrapper.
    #[inline]
    pub fn from_radians(value: Radians) -> Self {
        Self::new(value.get())
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
}

/// Wrapped mechanical angle in radians, normalized to `[-pi, pi)` by construction.
#[derive(Clone, Copy, Debug, PartialEq, PartialOrd, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct MechanicalAngle(pub Radians);

impl MechanicalAngle {
    /// Creates a new wrapped mechanical angle.
    #[inline]
    pub fn new(value: f32) -> Self {
        Self(Radians::new(wrap(value)))
    }

    /// Creates a new wrapped mechanical angle from a `Radians` wrapper.
    #[inline]
    pub fn from_radians(value: Radians) -> Self {
        Self::new(value.get())
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
}

/// Continuous mechanical angle in radians.
#[derive(Clone, Copy, Debug, PartialEq, PartialOrd, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct ContinuousMechanicalAngle(pub Radians);

impl ContinuousMechanicalAngle {
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

    /// Returns the scalar value.
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
    pub fn wrapped(self) -> MechanicalAngle {
        MechanicalAngle::new(self.get())
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

impl From<Radians> for ContinuousMechanicalAngle {
    #[inline]
    fn from(value: Radians) -> Self {
        Self::from_radians(value)
    }
}

impl From<ContinuousMechanicalAngle> for Radians {
    #[inline]
    fn from(value: ContinuousMechanicalAngle) -> Self {
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

impl From<MechanicalAngle> for ContinuousMechanicalAngle {
    #[inline]
    fn from(value: MechanicalAngle) -> Self {
        Self::from_radians(value.radians())
    }
}

impl From<ContinuousMechanicalAngle> for MechanicalAngle {
    #[inline]
    fn from(value: ContinuousMechanicalAngle) -> Self {
        value.wrapped()
    }
}

/// Wraps an angle into the canonical interval `[-pi, pi)`.
#[inline]
pub fn wrap(x: f32) -> f32 {
    let mut y = x % TAU;
    if y >= PI {
        y -= TAU;
    }
    if y < -PI {
        y += TAU;
    }
    y
}

/// Returns the shortest signed delta from `from` to `to` in `[-pi, pi)`.
#[inline]
pub fn shortest_angle_delta(from: f32, to: f32) -> f32 {
    wrap(to - from)
}

/// Converts a mechanical angle to an electrical angle using `pole_pairs`.
///
/// When `pole_pairs == 0`, returns zero and triggers a debug assertion in
/// debug builds because the conversion domain is invalid.
#[inline]
pub fn mechanical_to_electrical(
    theta_mech: ContinuousMechanicalAngle,
    pole_pairs: u32,
) -> ElectricalAngle {
    debug_assert!(pole_pairs > 0, "pole_pairs must be non-zero");
    if pole_pairs == 0 {
        return ElectricalAngle::new(0.0);
    }
    ElectricalAngle::from_radians(theta_mech.radians() * pole_pairs as f32)
}

/// Converts a wrapped electrical angle to a wrapped mechanical angle using `pole_pairs`.
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
        ContinuousMechanicalAngle, ElectricalAngle, MechanicalAngle, electrical_to_mechanical,
        mechanical_to_electrical, shortest_angle_delta, wrap,
    };
    use crate::scalar::{PI, TAU};
    use crate::units::Radians;

    fn approx_eq(a: f32, b: f32) {
        assert!((a - b).abs() < 1.0e-6, "{a} != {b}");
    }

    #[test]
    fn wrap_helpers_match_expected_intervals() {
        approx_eq(wrap(3.5 * PI), -0.5 * PI);
        approx_eq(wrap(-3.5 * PI), 0.5 * PI);
        approx_eq(wrap(2.25 * TAU), 0.25 * TAU);
    }

    #[test]
    fn shortest_delta_uses_wrapped_difference() {
        approx_eq(shortest_angle_delta(0.9 * PI, -0.9 * PI), -1.8 * PI + TAU);
    }

    #[test]
    fn mechanical_and_electrical_conversion_is_explicit() {
        let mech = ContinuousMechanicalAngle::new(1.25);
        let elec = mechanical_to_electrical(mech, 7);

        approx_eq(elec.get(), wrap(8.75));
        approx_eq(
            electrical_to_mechanical(elec, 7).get(),
            MechanicalAngle::new(wrap(8.75) / 7.0).get(),
        );
        approx_eq(ElectricalAngle::new(4.0 * PI).get(), 0.0);
        assert_eq!(
            ElectricalAngle::from_radians(Radians::new(1.0)).radians(),
            Radians::new(1.0)
        );
        approx_eq(MechanicalAngle::new(4.0 * PI).get(), 0.0);
        approx_eq(ContinuousMechanicalAngle::new(4.0 * PI).get(), 4.0 * PI);
        approx_eq(
            ContinuousMechanicalAngle::new(4.0 * PI).wrapped().get(),
            0.0,
        );
    }
}
