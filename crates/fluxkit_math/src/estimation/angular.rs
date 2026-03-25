use crate::angle::shortest_angle_delta;
use crate::estimation::internal::{LowPassState, PhaseLockedLoop};
use crate::estimation::signal::EstimatorSeed;
use crate::{ContinuousMechanicalAngle, MechanicalAngle, units::RadPerSec};

/// Generic estimator for wrapped/modulo-domain signals.
pub trait WrappedEstimator {
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

/// Wrapped angular value usable by modulo-domain estimators.
pub trait WrappedAngleValue: Copy + Default {
    /// Associated continuous angle representation.
    type Continuous: ContinuousAngleValue<Wrapped = Self>;

    /// Constructs a wrapped angle from a scalar.
    fn from_scalar(value: f32) -> Self;

    /// Returns the wrapped scalar value.
    fn get(self) -> f32;
}

/// Continuous angular value usable by modulo-domain estimators.
pub trait ContinuousAngleValue: Copy + Default {
    /// Associated wrapped angle representation.
    type Wrapped: WrappedAngleValue<Continuous = Self>;

    /// Constructs a continuous angle from a scalar.
    fn from_scalar(value: f32) -> Self;

    /// Constructs a continuous angle from its wrapped counterpart.
    fn from_wrapped(value: Self::Wrapped) -> Self;

    /// Returns the continuous scalar value.
    fn get(self) -> f32;

    /// Wraps this value into its modulo-domain representation.
    fn wrapped(self) -> Self::Wrapped;
}

impl WrappedAngleValue for MechanicalAngle {
    type Continuous = ContinuousMechanicalAngle;

    #[inline]
    fn from_scalar(value: f32) -> Self {
        Self::new(value)
    }

    #[inline]
    fn get(self) -> f32 {
        self.get()
    }
}

impl ContinuousAngleValue for ContinuousMechanicalAngle {
    type Wrapped = MechanicalAngle;

    #[inline]
    fn from_scalar(value: f32) -> Self {
        Self::new(value)
    }

    #[inline]
    fn from_wrapped(value: Self::Wrapped) -> Self {
        value.into()
    }

    #[inline]
    fn get(self) -> f32 {
        self.get()
    }

    #[inline]
    fn wrapped(self) -> Self::Wrapped {
        self.wrapped()
    }
}

/// Angular sample with wrapped value and measured rate.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct AngularSample<A, R> {
    /// Wrapped angle sample.
    pub wrapped_value: A,
    /// Measured angular rate associated with the sample.
    pub measured_rate: R,
}

/// Angular estimate with both wrapped and continuous values.
#[derive(Clone, Copy, Debug, PartialEq, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct AngularEstimate<W, U, R> {
    wrapped: W,
    unwrapped: U,
    velocity: R,
}

impl<W, U, R> AngularEstimate<W, U, R> {
    /// Creates an explicit angular estimate.
    #[inline]
    pub const fn new(wrapped: W, unwrapped: U, velocity: R) -> Self {
        Self {
            wrapped,
            unwrapped,
            velocity,
        }
    }
}

impl<W: Copy, U: Copy, R: Copy> AngularEstimate<W, U, R> {
    /// Returns the wrapped angle estimate.
    #[inline]
    pub const fn wrapped(&self) -> W {
        self.wrapped
    }

    /// Returns the continuous/unwrapped angle estimate.
    #[inline]
    pub const fn unwrapped(&self) -> U {
        self.unwrapped
    }

    /// Returns the estimated angular velocity.
    #[inline]
    pub const fn velocity(&self) -> R {
        self.velocity
    }
}

/// Angle-specialized estimator seeding modes.
pub type AngularEstimatorSeed<W, R> = EstimatorSeed<
    <W as WrappedAngleValue>::Continuous,
    R,
    AngularEstimate<W, <W as WrappedAngleValue>::Continuous, R>,
>;

/// Common mechanical-motion sample alias.
pub type MechanicalMotionSample = AngularSample<MechanicalAngle, RadPerSec>;
/// Common mechanical-motion estimate alias.
pub type MechanicalMotionEstimate =
    AngularEstimate<MechanicalAngle, ContinuousMechanicalAngle, RadPerSec>;
/// Common mechanical-motion seed alias.
pub type MechanicalMotionSeed = AngularEstimatorSeed<MechanicalAngle, RadPerSec>;

/// Wrapped-angle pass-through estimator.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct PassThroughWrappedEstimator<W, R>
where
    W: WrappedAngleValue,
{
    estimate: AngularEstimate<W, W::Continuous, R>,
    initialized: bool,
}

impl<W, R> PassThroughWrappedEstimator<W, R>
where
    W: WrappedAngleValue,
    W::Continuous: ContinuousAngleValue<Wrapped = W>,
    R: Copy + Default,
{
    /// Creates an uninitialized pass-through angle estimator.
    #[inline]
    pub fn new() -> Self {
        Self {
            estimate: AngularEstimate::new(
                W::from_scalar(0.0),
                <W::Continuous as ContinuousAngleValue>::from_scalar(0.0),
                R::default(),
            ),
            initialized: false,
        }
    }
}

impl<W, R> Default for PassThroughWrappedEstimator<W, R>
where
    W: WrappedAngleValue,
    W::Continuous: ContinuousAngleValue<Wrapped = W>,
    R: Copy + Default,
{
    #[inline]
    fn default() -> Self {
        Self::new()
    }
}

impl<W, R> WrappedEstimator for PassThroughWrappedEstimator<W, R>
where
    W: WrappedAngleValue,
    W::Continuous: ContinuousAngleValue<Wrapped = W>,
    R: Copy + Default,
{
    type Input = AngularSample<W, R>;
    type Output = AngularEstimate<W, W::Continuous, R>;
    type Seed = AngularEstimatorSeed<W, R>;

    #[inline]
    fn initialize(&mut self, seed: Self::Seed) {
        match seed {
            EstimatorSeed::Uninitialized => {
                self.estimate = AngularEstimate::new(
                    W::from_scalar(0.0),
                    <W::Continuous as ContinuousAngleValue>::from_scalar(0.0),
                    R::default(),
                );
                self.initialized = false;
            }
            EstimatorSeed::Value(wrapped_value) => {
                let wrapped = wrapped_value.wrapped();
                self.estimate = AngularEstimate::new(
                    wrapped,
                    <W::Continuous as ContinuousAngleValue>::from_wrapped(wrapped),
                    R::default(),
                );
                self.initialized = true;
            }
            EstimatorSeed::ValueRate {
                value: wrapped_value,
                rate,
            } => {
                let wrapped = wrapped_value.wrapped();
                self.estimate = AngularEstimate::new(
                    wrapped,
                    <W::Continuous as ContinuousAngleValue>::from_wrapped(wrapped),
                    rate,
                );
                self.initialized = true;
            }
            EstimatorSeed::Estimate(estimate) => {
                self.estimate = AngularEstimate::new(
                    estimate.wrapped(),
                    estimate.unwrapped(),
                    estimate.velocity(),
                );
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
        let wrapped = input.wrapped_value;
        if !self.initialized {
            self.initialize(EstimatorSeed::ValueRate {
                value: <W::Continuous as ContinuousAngleValue>::from_wrapped(wrapped),
                rate: input.measured_rate,
            });
            return self.estimate;
        }

        let delta = shortest_angle_delta(self.estimate.wrapped().get(), wrapped.get());
        let velocity = if dt > 0.0 && dt.is_finite() {
            input.measured_rate
        } else {
            self.estimate.velocity()
        };

        self.estimate = AngularEstimate::new(
            wrapped,
            <W::Continuous as ContinuousAngleValue>::from_scalar(
                self.estimate.unwrapped().get() + delta,
            ),
            velocity,
        );
        self.estimate
    }
}

/// LPF estimator configuration for angular motion.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct LpfEstimatorConfig {
    /// Time constant for the continuous angle filter.
    pub value_tau_seconds: f32,
    /// Time constant for the angular-rate filter.
    pub rate_tau_seconds: f32,
}

/// LPF-backed angle estimator.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct LpfWrappedEstimator<W, R>
where
    W: WrappedAngleValue,
{
    cfg: LpfEstimatorConfig,
    passthrough: PassThroughWrappedEstimator<W, R>,
    value_filter: LowPassState,
    rate_filter: LowPassState,
    estimate: AngularEstimate<W, W::Continuous, R>,
    initialized: bool,
}

impl<W, R> LpfWrappedEstimator<W, R>
where
    W: WrappedAngleValue,
    W::Continuous: ContinuousAngleValue<Wrapped = W>,
    R: Copy + Default + From<f32> + Into<f32>,
{
    /// Creates an uninitialized LPF angle estimator.
    #[inline]
    pub fn new(cfg: LpfEstimatorConfig) -> Self {
        Self {
            cfg,
            passthrough: PassThroughWrappedEstimator::new(),
            value_filter: LowPassState::new(0.0),
            rate_filter: LowPassState::new(0.0),
            estimate: AngularEstimate::new(
                W::from_scalar(0.0),
                <W::Continuous as ContinuousAngleValue>::from_scalar(0.0),
                R::default(),
            ),
            initialized: false,
        }
    }
}

impl<W, R> WrappedEstimator for LpfWrappedEstimator<W, R>
where
    W: WrappedAngleValue,
    W::Continuous: ContinuousAngleValue<Wrapped = W>,
    R: Copy + Default + From<f32> + Into<f32>,
{
    type Input = AngularSample<W, R>;
    type Output = AngularEstimate<W, W::Continuous, R>;
    type Seed = AngularEstimatorSeed<W, R>;

    #[inline]
    fn initialize(&mut self, seed: Self::Seed) {
        self.passthrough.initialize(seed);
        let base = self.passthrough.output();
        self.value_filter.reset(base.unwrapped().get());
        self.rate_filter.reset(base.velocity().into());
        self.estimate = base;
        self.initialized = !matches!(seed, EstimatorSeed::Uninitialized);
    }

    #[inline]
    fn output(&self) -> Self::Output {
        self.estimate
    }

    #[inline]
    fn update(&mut self, input: Self::Input, dt: f32) -> Self::Output {
        let raw = self.passthrough.update(input, dt);
        if !self.initialized {
            self.initialize(EstimatorSeed::Estimate(raw));
            return self.estimate;
        }

        if dt > 0.0 && dt.is_finite() {
            self.value_filter.set_tau(dt, self.cfg.value_tau_seconds);
            self.rate_filter.set_tau(dt, self.cfg.rate_tau_seconds);
        }

        let unwrapped = <W::Continuous as ContinuousAngleValue>::from_scalar(
            self.value_filter.update(raw.unwrapped().get()),
        );
        let velocity = R::from(self.rate_filter.update(raw.velocity().into()));
        self.estimate = AngularEstimate::new(unwrapped.wrapped(), unwrapped, velocity);
        self.estimate
    }
}

/// PLL-backed estimator configuration for angular motion.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct PllEstimatorConfig {
    /// Proportional gain applied to wrapped phase error.
    pub kp: f32,
    /// Integral gain applied to wrapped phase error.
    pub ki: f32,
}

/// PLL-backed angle estimator.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct PllWrappedEstimator<W, R>
where
    W: WrappedAngleValue,
{
    pll: PhaseLockedLoop,
    estimate: AngularEstimate<W, W::Continuous, R>,
    initialized: bool,
}

impl<W, R> PllWrappedEstimator<W, R>
where
    W: WrappedAngleValue,
    W::Continuous: ContinuousAngleValue<Wrapped = W>,
    R: Copy + Default + From<f32> + Into<f32>,
{
    /// Creates an uninitialized PLL angle estimator.
    #[inline]
    pub fn new(cfg: PllEstimatorConfig) -> Self {
        Self {
            pll: PhaseLockedLoop::new(cfg.kp, cfg.ki),
            estimate: AngularEstimate::new(
                W::from_scalar(0.0),
                <W::Continuous as ContinuousAngleValue>::from_scalar(0.0),
                R::default(),
            ),
            initialized: false,
        }
    }
}

impl<W, R> WrappedEstimator for PllWrappedEstimator<W, R>
where
    W: WrappedAngleValue,
    W::Continuous: ContinuousAngleValue<Wrapped = W>,
    R: Copy + Default + From<f32> + Into<f32>,
{
    type Input = AngularSample<W, R>;
    type Output = AngularEstimate<W, W::Continuous, R>;
    type Seed = AngularEstimatorSeed<W, R>;

    #[inline]
    fn initialize(&mut self, seed: Self::Seed) {
        match seed {
            EstimatorSeed::Uninitialized => {
                self.estimate = AngularEstimate::new(
                    W::from_scalar(0.0),
                    <W::Continuous as ContinuousAngleValue>::from_scalar(0.0),
                    R::default(),
                );
                self.initialized = false;
            }
            EstimatorSeed::Value(wrapped_value) => {
                let wrapped = wrapped_value.wrapped();
                self.pll.reset(wrapped.get());
                self.estimate = AngularEstimate::new(
                    wrapped,
                    <W::Continuous as ContinuousAngleValue>::from_wrapped(wrapped),
                    R::default(),
                );
                self.initialized = true;
            }
            EstimatorSeed::ValueRate {
                value: wrapped_value,
                rate,
            } => {
                let wrapped = wrapped_value.wrapped();
                self.pll.reset_with_velocity(wrapped.get(), rate.into());
                self.estimate = AngularEstimate::new(
                    wrapped,
                    <W::Continuous as ContinuousAngleValue>::from_wrapped(wrapped),
                    rate,
                );
                self.initialized = true;
            }
            EstimatorSeed::Estimate(estimate) => {
                self.pll
                    .reset_with_velocity(estimate.wrapped().get(), estimate.velocity().into());
                self.estimate = AngularEstimate::new(
                    estimate.wrapped(),
                    estimate.unwrapped(),
                    estimate.velocity(),
                );
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
        let wrapped_input = input.wrapped_value;
        if !self.initialized {
            self.initialize(EstimatorSeed::ValueRate {
                value: <W::Continuous as ContinuousAngleValue>::from_wrapped(wrapped_input),
                rate: input.measured_rate,
            });
            return self.estimate;
        }

        let filtered_wrapped = <W::Continuous as ContinuousAngleValue>::from_scalar(
            self.pll.update(wrapped_input.get(), dt),
        )
        .wrapped();
        let delta = shortest_angle_delta(self.estimate.wrapped().get(), filtered_wrapped.get());
        self.estimate = AngularEstimate::new(
            filtered_wrapped,
            <W::Continuous as ContinuousAngleValue>::from_scalar(
                self.estimate.unwrapped().get() + delta,
            ),
            R::from(self.pll.velocity()),
        );
        self.estimate
    }
}

/// Mechanical-angle pass-through estimator.
pub type PassThroughEstimator = PassThroughWrappedEstimator<MechanicalAngle, RadPerSec>;
/// Mechanical-angle LPF estimator.
pub type LpfEstimator = LpfWrappedEstimator<MechanicalAngle, RadPerSec>;
/// Mechanical-angle PLL estimator.
pub type PllEstimator = PllWrappedEstimator<MechanicalAngle, RadPerSec>;

#[cfg(test)]
mod tests {
    use super::{
        AngularEstimate, AngularSample, EstimatorSeed, LpfEstimator, LpfEstimatorConfig,
        PassThroughEstimator, PllEstimator, PllEstimatorConfig, WrappedEstimator,
    };
    use crate::{
        ContinuousMechanicalAngle, MechanicalAngle, angle::shortest_angle_delta, units::RadPerSec,
    };

    fn approx_eq(a: f32, b: f32, tol: f32) {
        assert!((a - b).abs() < tol, "{a} != {b}");
    }

    #[test]
    fn pass_through_angle_estimator_unwraps_wrap_crossings() {
        let mut estimator = PassThroughEstimator::new();
        estimator.update(
            AngularSample {
                wrapped_value: MechanicalAngle::new(3.12),
                measured_rate: RadPerSec::new(0.5),
            },
            0.001,
        );
        let estimate = estimator.update(
            AngularSample {
                wrapped_value: MechanicalAngle::new(-3.09),
                measured_rate: RadPerSec::new(0.5),
            },
            0.001,
        );

        assert!(estimate.unwrapped().get() > 3.12);
        approx_eq(estimate.velocity().get(), 0.5, 1.0e-6);
    }

    #[test]
    fn angular_estimator_seed_can_seed_full_state() {
        let mut estimator = PassThroughEstimator::new();
        estimator.initialize(EstimatorSeed::Estimate(AngularEstimate::new(
            MechanicalAngle::new(1.0),
            ContinuousMechanicalAngle::new(7.283_185_5),
            RadPerSec::new(2.0),
        )));

        let estimate = estimator.output();
        approx_eq(estimate.unwrapped().get(), 7.283_185_5, 1.0e-6);
        approx_eq(estimate.velocity().get(), 2.0, 1.0e-6);
    }

    #[test]
    fn lpf_angle_estimator_smooths_velocity() {
        let mut estimator = LpfEstimator::new(LpfEstimatorConfig {
            value_tau_seconds: 0.01,
            rate_tau_seconds: 0.02,
        });
        estimator.initialize(EstimatorSeed::Value(ContinuousMechanicalAngle::new(0.0)));

        for _ in 0..50 {
            estimator.update(
                AngularSample {
                    wrapped_value: MechanicalAngle::new(0.5),
                    measured_rate: RadPerSec::new(10.0),
                },
                0.001,
            );
        }

        let estimate = estimator.output();
        assert!(estimate.velocity().get() > 0.0);
        assert!(estimate.velocity().get() < 10.0);
    }

    #[test]
    fn pll_estimator_tracks_wrapped_ramp() {
        let mut estimator = PllEstimator::new(PllEstimatorConfig {
            kp: 40.0,
            ki: 400.0,
        });
        let dt = 0.001;
        let omega = 4.0;
        let mut wrapped = MechanicalAngle::new(0.0);

        for _ in 0..1500 {
            wrapped = ContinuousMechanicalAngle::new(wrapped.get() + omega * dt).wrapped();
            estimator.update(
                AngularSample {
                    wrapped_value: wrapped,
                    measured_rate: RadPerSec::new(omega),
                },
                dt,
            );
        }

        let estimate = estimator.output();
        approx_eq(
            shortest_angle_delta(estimate.wrapped().get(), wrapped.get()),
            0.0,
            3.0e-2,
        );
        approx_eq(estimate.velocity().get(), omega, 2.5e-1);
    }
}
