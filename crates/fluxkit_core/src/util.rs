//! Internal utility helpers shared across controller modules.

use fluxkit_math::{
    frame::{Abc, Dq},
    modulation::PhaseDuty,
    units::{Amps, Duty, Volts},
};

/// Returns the neutral three-phase duty command.
#[inline]
pub const fn neutral_phase_duty() -> PhaseDuty {
    Abc::new(Duty::new(0.5), Duty::new(0.5), Duty::new(0.5))
}

/// Returns a zero-valued measured current vector.
#[inline]
pub const fn zero_current_dq() -> Dq<Amps> {
    Dq::new(Amps::new(0.0), Amps::new(0.0))
}

/// Returns a zero-valued voltage command vector.
#[inline]
pub const fn zero_voltage_dq() -> Dq<Volts> {
    Dq::new(Volts::new(0.0), Volts::new(0.0))
}
