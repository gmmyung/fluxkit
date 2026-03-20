//! Three-phase PWM output traits.

use fluxkit_math::{modulation::PhaseDuty, units::Duty};

use crate::util::centered_phase_duty;

/// Narrow contract for a three-phase PWM output stage.
pub trait PhasePwm {
    /// Platform-specific error type.
    type Error;

    /// Enables the phase outputs.
    fn enable(&mut self) -> Result<(), Self::Error>;

    /// Disables the phase outputs.
    fn disable(&mut self) -> Result<(), Self::Error>;

    /// Sets normalized duty for each phase.
    fn set_duty(&mut self, a: Duty, b: Duty, c: Duty) -> Result<(), Self::Error>;

    /// Sets all three duties from a typed duty vector.
    #[inline]
    fn set_phase_duty(&mut self, duty: PhaseDuty) -> Result<(), Self::Error> {
        self.set_duty(duty.a, duty.b, duty.c)
    }

    /// Drives a centered neutral duty on all phases.
    #[inline]
    fn set_neutral(&mut self) -> Result<(), Self::Error> {
        self.set_phase_duty(centered_phase_duty())
    }
}
