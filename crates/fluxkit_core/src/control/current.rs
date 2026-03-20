//! Current-control helper types.

use fluxkit_math::units::Amps;

/// Requested `d/q` current references.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct CurrentReference {
    /// Direct-axis current reference.
    pub id: Amps,
    /// Quadrature-axis current reference.
    pub iq: Amps,
}
