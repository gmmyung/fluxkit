//! Small helper utilities shared by HAL traits.

use fluxkit_math::{frame::Abc, modulation::PhaseDuty, units::Duty};

/// Returns a centered three-phase duty command.
#[inline]
pub const fn centered_phase_duty() -> PhaseDuty {
    Abc::new(Duty::new(0.5), Duty::new(0.5), Duty::new(0.5))
}
