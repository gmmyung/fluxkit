//! Shared bidirectional torque-ramp tracking for release-style procedures.

use fluxkit_math::units::NewtonMeters;

/// Shared phase machine used by bidirectional release-ramp procedures.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub(crate) enum BidirectionalRampPhase {
    /// Ramp positive torque from zero.
    PositiveRamp,
    /// Hold zero torque until the mechanism rests.
    NeutralSettle,
    /// Ramp negative torque from zero.
    NegativeRamp,
}

/// Shared progress and latch state for bidirectional release-ramp procedures.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub(crate) struct BidirectionalRampTracker {
    /// Current ramp phase.
    pub phase: BidirectionalRampPhase,
    /// Seconds spent in the current ramp phase.
    pub ramp_seconds: f32,
    /// Continuous motion time for the current release check.
    pub motion_seconds: f32,
    /// Continuous rest time during neutral settle.
    pub rest_seconds: f32,
    /// First latched release value for the current direction.
    pub release_candidate: Option<f32>,
}

impl BidirectionalRampTracker {
    /// Zeroed tracker starting in the positive ramp phase.
    #[inline]
    pub const fn new() -> Self {
        Self {
            phase: BidirectionalRampPhase::PositiveRamp,
            ramp_seconds: 0.0,
            motion_seconds: 0.0,
            rest_seconds: 0.0,
            release_candidate: None,
        }
    }

    /// Returns the currently commanded torque target for a symmetric
    /// bidirectional ramp.
    #[inline]
    pub fn commanded_torque_target(
        &self,
        torque_ramp_rate_nm_per_sec: f32,
        max_torque: NewtonMeters,
    ) -> NewtonMeters {
        match self.phase {
            BidirectionalRampPhase::PositiveRamp => {
                NewtonMeters::new(self.current_ramp_torque(torque_ramp_rate_nm_per_sec, max_torque))
            }
            BidirectionalRampPhase::NeutralSettle => NewtonMeters::ZERO,
            BidirectionalRampPhase::NegativeRamp => NewtonMeters::new(
                -self.current_ramp_torque(torque_ramp_rate_nm_per_sec, max_torque),
            ),
        }
    }

    /// Advances the ramp timer and returns the current absolute ramp torque.
    #[inline]
    pub fn advance_ramp(
        &mut self,
        dt_seconds: f32,
        torque_ramp_rate_nm_per_sec: f32,
        max_torque: NewtonMeters,
    ) -> f32 {
        self.ramp_seconds += dt_seconds;
        self.current_ramp_torque(torque_ramp_rate_nm_per_sec, max_torque)
    }

    /// Updates the release latch for the current ramp direction and returns the
    /// latched release value once sustained motion is confirmed.
    #[inline]
    pub fn observe_release(
        &mut self,
        moving: bool,
        dt_seconds: f32,
        release_value: f32,
        motion_confirm_time_seconds: f32,
    ) -> Option<f32> {
        if moving {
            if self.motion_seconds <= 0.0 {
                self.release_candidate = Some(release_value);
            }
            self.motion_seconds += dt_seconds;
        } else {
            self.reset_motion_window();
        }

        if self.motion_seconds >= motion_confirm_time_seconds {
            Some(self.release_candidate.unwrap_or(release_value))
        } else {
            None
        }
    }

    /// Updates the neutral settle timer and reports whether the mechanism has
    /// remained at rest long enough to reverse direction.
    #[inline]
    pub fn observe_rest(&mut self, resting: bool, dt_seconds: f32, rest_time_seconds: f32) -> bool {
        if resting {
            self.rest_seconds += dt_seconds;
        } else {
            self.rest_seconds = 0.0;
        }
        self.rest_seconds >= rest_time_seconds
    }

    /// Switches into the neutral settle phase and clears direction-specific
    /// tracking.
    #[inline]
    pub fn begin_neutral_settle(&mut self) {
        self.phase = BidirectionalRampPhase::NeutralSettle;
        self.ramp_seconds = 0.0;
        self.motion_seconds = 0.0;
        self.rest_seconds = 0.0;
        self.release_candidate = None;
    }

    /// Switches into the negative ramp phase and clears direction-specific
    /// tracking.
    #[inline]
    pub fn begin_negative_ramp(&mut self) {
        self.phase = BidirectionalRampPhase::NegativeRamp;
        self.ramp_seconds = 0.0;
        self.motion_seconds = 0.0;
        self.rest_seconds = 0.0;
        self.release_candidate = None;
    }

    /// Clears the release latch and motion timer.
    #[inline]
    pub fn reset_motion_window(&mut self) {
        self.motion_seconds = 0.0;
        self.release_candidate = None;
    }

    #[inline]
    fn current_ramp_torque(
        &self,
        torque_ramp_rate_nm_per_sec: f32,
        max_torque: NewtonMeters,
    ) -> f32 {
        (self.ramp_seconds * torque_ramp_rate_nm_per_sec).min(max_torque.get())
    }
}
