//! Lightweight wrappers for physical quantities.

macro_rules! impl_unit {
    ($name:ident, $doc:literal) => {
        #[doc = $doc]
        #[derive(Clone, Copy, Debug, PartialEq, PartialOrd, Default)]
        #[cfg_attr(feature = "defmt", derive(defmt::Format))]
        #[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
        pub struct $name(pub f32);

        impl $name {
            /// Zero-valued quantity.
            pub const ZERO: Self = Self(0.0);

            /// Creates a new wrapped value.
            #[inline]
            pub const fn new(value: f32) -> Self {
                Self(value)
            }

            /// Returns the underlying scalar value.
            #[inline]
            pub const fn get(self) -> f32 {
                self.0
            }
        }

        impl From<f32> for $name {
            #[inline]
            fn from(value: f32) -> Self {
                Self(value)
            }
        }

        impl From<$name> for f32 {
            #[inline]
            fn from(value: $name) -> Self {
                value.0
            }
        }

        impl core::ops::Add for $name {
            type Output = Self;

            #[inline]
            fn add(self, rhs: Self) -> Self::Output {
                Self(self.0 + rhs.0)
            }
        }

        impl core::ops::AddAssign for $name {
            #[inline]
            fn add_assign(&mut self, rhs: Self) {
                self.0 += rhs.0;
            }
        }

        impl core::ops::Sub for $name {
            type Output = Self;

            #[inline]
            fn sub(self, rhs: Self) -> Self::Output {
                Self(self.0 - rhs.0)
            }
        }

        impl core::ops::SubAssign for $name {
            #[inline]
            fn sub_assign(&mut self, rhs: Self) {
                self.0 -= rhs.0;
            }
        }

        impl core::ops::Mul<f32> for $name {
            type Output = Self;

            #[inline]
            fn mul(self, rhs: f32) -> Self::Output {
                Self(self.0 * rhs)
            }
        }

        impl core::ops::MulAssign<f32> for $name {
            #[inline]
            fn mul_assign(&mut self, rhs: f32) {
                self.0 *= rhs;
            }
        }

        impl core::ops::Div<f32> for $name {
            type Output = Self;

            #[inline]
            fn div(self, rhs: f32) -> Self::Output {
                Self(self.0 / rhs)
            }
        }

        impl core::ops::DivAssign<f32> for $name {
            #[inline]
            fn div_assign(&mut self, rhs: f32) {
                self.0 /= rhs;
            }
        }

        impl core::ops::Neg for $name {
            type Output = Self;

            #[inline]
            fn neg(self) -> Self::Output {
                Self(-self.0)
            }
        }
    };
}

impl_unit!(Amps, "Electrical current in amperes.");
impl_unit!(Volts, "Electrical potential in volts.");
impl_unit!(Radians, "Generic angular position in radians.");
impl_unit!(RadPerSec, "Angular velocity in radians per second.");
impl_unit!(Seconds, "Time interval in seconds.");
impl_unit!(Duty, "Normalized PWM duty ratio.");
impl_unit!(Hertz, "Frequency in hertz.");
impl_unit!(Ohms, "Electrical resistance in ohms.");
impl_unit!(Henries, "Electrical inductance in henries.");
impl_unit!(Webers, "Magnetic flux linkage in webers.");
impl_unit!(Celsius, "Temperature in degrees Celsius.");
impl_unit!(NewtonMeters, "Torque in newton-meters.");

#[cfg(test)]
mod tests {
    use super::{Amps, Duty, Henries, NewtonMeters, Ohms, Webers};

    #[test]
    fn unit_arithmetic_is_lightweight_and_explicit() {
        let mut current = Amps::new(1.5);
        current += Amps::new(0.25);
        current *= 2.0;

        assert_eq!(current, Amps::new(3.5));
        assert_eq!(Duty::new(0.25).get(), 0.25);
        assert_eq!(Ohms::new(0.12).get(), 0.12);
        assert_eq!(Henries::new(0.000_035).get(), 0.000_035);
        assert_eq!(Webers::new(0.004_8).get(), 0.004_8);
        assert_eq!(NewtonMeters::new(0.6).get(), 0.6);
        assert_eq!(f32::from(current), 3.5);
    }
}
