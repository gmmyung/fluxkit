//! Current-control helper types.

use fluxkit_math::estimation::{
    ContinuousEstimator, EstimatorSeed, LpfSignalEstimator, SignalSample,
};
use fluxkit_math::frame::Dq;
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

/// Estimator contract used by the controller current loop.
pub trait CurrentEstimator {
    /// Resets estimator state to its explicit zero/initial state.
    fn reset(&mut self);

    /// Returns the latest estimated `d/q` current vector.
    fn output(&self) -> Dq<Amps>;

    /// Advances the estimator from a new measured `d/q` current sample.
    fn update(&mut self, measured_idq: Dq<Amps>, dt_seconds: f32) -> Dq<Amps>;
}

/// LPF estimator configuration for `d/q` current estimation.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct LpfCurrentEstimatorConfig {
    /// Time constant for the direct-axis current estimate.
    pub d_tau_seconds: f32,
    /// Time constant for the quadrature-axis current estimate.
    pub q_tau_seconds: f32,
}

impl LpfCurrentEstimatorConfig {
    /// Creates an explicit LPF current-estimator configuration.
    #[inline]
    pub const fn new(d_tau_seconds: f32, q_tau_seconds: f32) -> Self {
        Self {
            d_tau_seconds,
            q_tau_seconds,
        }
    }

    /// Creates a symmetric LPF current-estimator configuration.
    #[inline]
    pub const fn symmetric(tau_seconds: f32) -> Self {
        Self::new(tau_seconds, tau_seconds)
    }
}

/// Pass-through current estimator that forwards the latest measured `d/q`
/// current directly into the control loop.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct PassThroughCurrentEstimator {
    estimate: Dq<Amps>,
}

impl PassThroughCurrentEstimator {
    /// Creates a zeroed pass-through current estimator.
    #[inline]
    pub fn new() -> Self {
        Self {
            estimate: Dq::new(Amps::ZERO, Amps::ZERO),
        }
    }
}

impl Default for PassThroughCurrentEstimator {
    #[inline]
    fn default() -> Self {
        Self::new()
    }
}

impl CurrentEstimator for PassThroughCurrentEstimator {
    #[inline]
    fn reset(&mut self) {
        self.estimate = Dq::new(Amps::ZERO, Amps::ZERO);
    }

    #[inline]
    fn output(&self) -> Dq<Amps> {
        self.estimate
    }

    #[inline]
    fn update(&mut self, measured_idq: Dq<Amps>, _dt_seconds: f32) -> Dq<Amps> {
        self.estimate = measured_idq;
        self.estimate
    }
}

/// LPF-backed current estimator for `d/q` current samples.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct LpfCurrentEstimator {
    d_estimator: LpfSignalEstimator<Amps, Amps>,
    q_estimator: LpfSignalEstimator<Amps, Amps>,
}

impl LpfCurrentEstimator {
    /// Creates an LPF current estimator.
    #[inline]
    pub fn new(cfg: LpfCurrentEstimatorConfig) -> Self {
        Self {
            d_estimator: LpfSignalEstimator::new(cfg.d_tau_seconds, cfg.d_tau_seconds),
            q_estimator: LpfSignalEstimator::new(cfg.q_tau_seconds, cfg.q_tau_seconds),
        }
    }
}

impl CurrentEstimator for LpfCurrentEstimator {
    #[inline]
    fn reset(&mut self) {
        self.d_estimator.initialize(EstimatorSeed::Uninitialized);
        self.q_estimator.initialize(EstimatorSeed::Uninitialized);
    }

    #[inline]
    fn output(&self) -> Dq<Amps> {
        Dq::new(
            self.d_estimator.output().value(),
            self.q_estimator.output().value(),
        )
    }

    #[inline]
    fn update(&mut self, measured_idq: Dq<Amps>, dt_seconds: f32) -> Dq<Amps> {
        let d = self
            .d_estimator
            .update(
                SignalSample {
                    value: measured_idq.d,
                    measured_rate: Amps::ZERO,
                },
                dt_seconds,
            )
            .value();
        let q = self
            .q_estimator
            .update(
                SignalSample {
                    value: measured_idq.q,
                    measured_rate: Amps::ZERO,
                },
                dt_seconds,
            )
            .value();

        Dq::new(d, q)
    }
}

#[cfg(test)]
mod tests {
    use super::{
        CurrentEstimator, LpfCurrentEstimator, LpfCurrentEstimatorConfig,
        PassThroughCurrentEstimator,
    };
    use fluxkit_math::{Amps, Dq};

    #[test]
    fn pass_through_current_estimator_tracks_latest_sample() {
        let mut estimator = PassThroughCurrentEstimator::new();
        let estimate = estimator.update(Dq::new(Amps::new(2.0), Amps::new(-1.5)), 0.001);

        assert_eq!(estimate, Dq::new(Amps::new(2.0), Amps::new(-1.5)));
        assert_eq!(estimator.output(), estimate);
    }

    #[test]
    fn lpf_current_estimator_smooths_current_components() {
        let mut estimator = LpfCurrentEstimator::new(LpfCurrentEstimatorConfig::symmetric(0.01));

        let seeded = estimator.update(Dq::new(Amps::ZERO, Amps::ZERO), 0.001);
        assert_eq!(seeded, Dq::new(Amps::ZERO, Amps::ZERO));

        for _ in 0..50 {
            estimator.update(Dq::new(Amps::new(10.0), Amps::new(-8.0)), 0.001);
        }

        let estimate = estimator.output();
        assert!(estimate.d > Amps::ZERO);
        assert!(estimate.d < Amps::new(10.0));
        assert!(estimate.q < Amps::ZERO);
        assert!(estimate.q > Amps::new(-8.0));
    }

    #[test]
    fn lpf_current_estimator_reset_clears_state() {
        let mut estimator = LpfCurrentEstimator::new(LpfCurrentEstimatorConfig::new(0.01, 0.02));
        estimator.update(Dq::new(Amps::new(3.0), Amps::new(-4.0)), 0.001);

        estimator.reset();

        assert_eq!(estimator.output(), Dq::new(Amps::ZERO, Amps::ZERO));
        let reseeded = estimator.update(Dq::new(Amps::new(1.0), Amps::new(2.0)), 0.001);
        assert_eq!(reseeded, Dq::new(Amps::new(1.0), Amps::new(2.0)));
    }
}
