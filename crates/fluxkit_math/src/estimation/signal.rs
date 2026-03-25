use super::internal::LowPassState;

/// Generic estimator for continuous, non-wrapping signals.
pub trait ContinuousEstimator {
    /// Input sample consumed by the estimator.
    type Input;
    /// Output estimate produced by the estimator.
    type Output;
    /// Explicit state seeding payload.
    type Seed;

    /// Explicitly seeds or clears estimator state.
    fn initialize(&mut self, seed: Self::Seed);

    /// Returns the current estimate.
    fn output(&self) -> Self::Output;

    /// Advances the estimator with a new sample.
    fn update(&mut self, input: Self::Input, dt: f32) -> Self::Output;
}

/// Generic continuous-signal sample.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct SignalSample<V, R> {
    /// Measured value sample.
    pub value: V,
    /// Measured rate associated with the sample.
    pub measured_rate: R,
}

/// Generic continuous-signal estimate.
#[derive(Clone, Copy, Debug, PartialEq, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct SignalEstimate<V, R> {
    value: V,
    rate: R,
}

impl<V, R> SignalEstimate<V, R> {
    /// Creates an explicit signal estimate.
    #[inline]
    pub const fn new(value: V, rate: R) -> Self {
        Self { value, rate }
    }
}

impl<V: Copy, R: Copy> SignalEstimate<V, R> {
    /// Returns the estimated value.
    #[inline]
    pub const fn value(&self) -> V {
        self.value
    }

    /// Returns the estimated rate.
    #[inline]
    pub const fn rate(&self) -> R {
        self.rate
    }
}

/// Generic estimator seeding modes.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum EstimatorSeed<V, R, E = SignalEstimate<V, R>> {
    /// Clears initialization so the next update seeds directly from a sample.
    Uninitialized,
    /// Seeds only the value; the rate becomes zero/default.
    Value(V),
    /// Seeds value and rate.
    ValueRate {
        /// Seeded value.
        value: V,
        /// Seeded rate.
        rate: R,
    },
    /// Seeds the full estimate explicitly.
    Estimate(E),
}

/// Generic pass-through estimator for continuous scalar-like signals.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct PassThroughSignalEstimator<V, R> {
    estimate: SignalEstimate<V, R>,
    initialized: bool,
}

impl<V, R> PassThroughSignalEstimator<V, R>
where
    V: Copy + Default,
    R: Copy + Default,
{
    /// Creates an uninitialized pass-through estimator.
    #[inline]
    pub fn new() -> Self {
        Self {
            estimate: SignalEstimate::new(V::default(), R::default()),
            initialized: false,
        }
    }
}

impl<V, R> Default for PassThroughSignalEstimator<V, R>
where
    V: Copy + Default,
    R: Copy + Default,
{
    #[inline]
    fn default() -> Self {
        Self::new()
    }
}

impl<V, R> ContinuousEstimator for PassThroughSignalEstimator<V, R>
where
    V: Copy + Default,
    R: Copy + Default,
{
    type Input = SignalSample<V, R>;
    type Output = SignalEstimate<V, R>;
    type Seed = EstimatorSeed<V, R>;

    #[inline]
    fn initialize(&mut self, seed: Self::Seed) {
        match seed {
            EstimatorSeed::Uninitialized => {
                self.estimate = SignalEstimate::new(V::default(), R::default());
                self.initialized = false;
            }
            EstimatorSeed::Value(value) => {
                self.estimate = SignalEstimate::new(value, R::default());
                self.initialized = true;
            }
            EstimatorSeed::ValueRate { value, rate } => {
                self.estimate = SignalEstimate::new(value, rate);
                self.initialized = true;
            }
            EstimatorSeed::Estimate(estimate) => {
                self.estimate = estimate;
                self.initialized = true;
            }
        }
    }

    #[inline]
    fn output(&self) -> Self::Output {
        self.estimate
    }

    #[inline]
    fn update(&mut self, input: Self::Input, _dt: f32) -> Self::Output {
        if !self.initialized {
            self.initialize(EstimatorSeed::ValueRate {
                value: input.value,
                rate: input.measured_rate,
            });
            return self.estimate;
        }

        self.estimate = SignalEstimate::new(input.value, input.measured_rate);
        self.estimate
    }
}

/// LPF-backed estimator for continuous scalar-like signals.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct LpfSignalEstimator<V, R> {
    value_tau_seconds: f32,
    rate_tau_seconds: f32,
    value_filter: LowPassState,
    rate_filter: LowPassState,
    estimate: SignalEstimate<V, R>,
    initialized: bool,
}

impl<V, R> LpfSignalEstimator<V, R>
where
    V: Copy + Default + From<f32> + Into<f32>,
    R: Copy + Default + From<f32> + Into<f32>,
{
    /// Creates an uninitialized LPF signal estimator.
    #[inline]
    pub fn new(value_tau_seconds: f32, rate_tau_seconds: f32) -> Self {
        Self {
            value_tau_seconds,
            rate_tau_seconds,
            value_filter: LowPassState::new(0.0),
            rate_filter: LowPassState::new(0.0),
            estimate: SignalEstimate::new(V::default(), R::default()),
            initialized: false,
        }
    }
}

impl<V, R> ContinuousEstimator for LpfSignalEstimator<V, R>
where
    V: Copy + Default + From<f32> + Into<f32>,
    R: Copy + Default + From<f32> + Into<f32>,
{
    type Input = SignalSample<V, R>;
    type Output = SignalEstimate<V, R>;
    type Seed = EstimatorSeed<V, R>;

    #[inline]
    fn initialize(&mut self, seed: Self::Seed) {
        match seed {
            EstimatorSeed::Uninitialized => {
                self.estimate = SignalEstimate::new(V::default(), R::default());
                self.initialized = false;
            }
            EstimatorSeed::Value(value) => {
                self.value_filter.reset(value.into());
                self.rate_filter.reset(R::default().into());
                self.estimate = SignalEstimate::new(value, R::default());
                self.initialized = true;
            }
            EstimatorSeed::ValueRate { value, rate } => {
                self.value_filter.reset(value.into());
                self.rate_filter.reset(rate.into());
                self.estimate = SignalEstimate::new(value, rate);
                self.initialized = true;
            }
            EstimatorSeed::Estimate(estimate) => {
                self.value_filter.reset(estimate.value().into());
                self.rate_filter.reset(estimate.rate().into());
                self.estimate = estimate;
                self.initialized = true;
            }
        }
    }

    #[inline]
    fn output(&self) -> Self::Output {
        self.estimate
    }

    #[inline]
    fn update(&mut self, input: Self::Input, dt: f32) -> Self::Output {
        if !self.initialized {
            self.initialize(EstimatorSeed::ValueRate {
                value: input.value,
                rate: input.measured_rate,
            });
            return self.estimate;
        }

        if dt > 0.0 && dt.is_finite() {
            self.value_filter.set_tau(dt, self.value_tau_seconds);
            self.rate_filter.set_tau(dt, self.rate_tau_seconds);
        }

        self.estimate = SignalEstimate::new(
            V::from(self.value_filter.update(input.value.into())),
            R::from(self.rate_filter.update(input.measured_rate.into())),
        );
        self.estimate
    }
}

#[cfg(test)]
mod tests {
    use super::{
        ContinuousEstimator, EstimatorSeed, LpfSignalEstimator, PassThroughSignalEstimator,
        SignalEstimate, SignalSample,
    };

    fn approx_eq(a: f32, b: f32, tol: f32) {
        assert!((a - b).abs() < tol, "{a} != {b}");
    }

    #[test]
    fn pass_through_signal_estimator_tracks_value_and_rate() {
        let mut estimator = PassThroughSignalEstimator::<f32, f32>::new();
        let estimate = estimator.update(
            SignalSample {
                value: 3.0,
                measured_rate: -2.0,
            },
            0.001,
        );

        approx_eq(estimate.value(), 3.0, 1.0e-6);
        approx_eq(estimate.rate(), -2.0, 1.0e-6);
    }

    #[test]
    fn lpf_signal_estimator_smooths_scalar_signal() {
        let mut estimator = LpfSignalEstimator::<f32, f32>::new(0.01, 0.02);
        estimator.initialize(EstimatorSeed::ValueRate {
            value: 0.0,
            rate: 0.0,
        });

        for _ in 0..50 {
            estimator.update(
                SignalSample {
                    value: 10.0,
                    measured_rate: 20.0,
                },
                0.001,
            );
        }

        assert!(estimator.output().value() > 0.0);
        assert!(estimator.output().value() < 10.0);
    }

    #[test]
    fn signal_estimate_accessors_are_explicit() {
        let estimate = SignalEstimate::new(1.5_f32, -0.5_f32);
        approx_eq(estimate.value(), 1.5, 1.0e-6);
        approx_eq(estimate.rate(), -0.5, 1.0e-6);
    }
}
