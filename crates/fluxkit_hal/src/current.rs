//! Current-sampling traits.

use fluxkit_math::{frame::Abc, units::Amps};

/// Quality indicator for a sampled phase-current vector.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum CurrentSampleValidity {
    /// Hardware measurement is valid.
    Valid,
    /// Returned current was estimated rather than directly sampled.
    Estimated,
    /// The sensing chain clipped but still produced a bounded value.
    Saturated,
    /// The sample is not valid for control use.
    Invalid,
}

/// Sampled phase currents plus validity metadata.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct PhaseCurrentSample {
    /// Three-phase current vector.
    pub currents: Abc<Amps>,
    /// Sample quality.
    pub validity: CurrentSampleValidity,
}

/// Narrow synchronous trait for phase-current acquisition.
pub trait CurrentSampler {
    /// Platform-specific error type.
    type Error: core::error::Error;

    /// Returns the most recent phase-current sample.
    fn sample_phase_currents(&mut self) -> Result<PhaseCurrentSample, Self::Error>;
}
