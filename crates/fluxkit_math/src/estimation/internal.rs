use crate::{
    angle::{shortest_angle_delta, wrap},
    scalar::clamp,
};

#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct LowPassState {
    alpha: f32,
    value: f32,
    initialized: bool,
}

impl LowPassState {
    #[inline]
    pub(crate) fn new(alpha: f32) -> Self {
        Self {
            alpha: clamp(alpha, 0.0, 1.0),
            value: 0.0,
            initialized: false,
        }
    }

    #[inline]
    pub(crate) fn reset(&mut self, value: f32) {
        self.value = value;
        self.initialized = true;
    }

    #[inline]
    pub(crate) fn set_tau(&mut self, dt: f32, tau: f32) {
        let alpha = if tau <= 0.0 {
            1.0
        } else if dt <= 0.0 {
            0.0
        } else {
            dt / (tau + dt)
        };
        self.alpha = clamp(alpha, 0.0, 1.0);
    }

    #[inline]
    pub(crate) fn update(&mut self, sample: f32) -> f32 {
        if !self.initialized {
            self.value = sample;
            self.initialized = true;
        } else {
            self.value += self.alpha * (sample - self.value);
        }
        self.value
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct PhaseLockedLoop {
    kp: f32,
    ki: f32,
    angle: f32,
    velocity: f32,
    initialized: bool,
}

impl PhaseLockedLoop {
    #[inline]
    pub(crate) const fn new(kp: f32, ki: f32) -> Self {
        Self {
            kp,
            ki,
            angle: 0.0,
            velocity: 0.0,
            initialized: false,
        }
    }

    #[inline]
    pub(crate) fn reset(&mut self, angle: f32) {
        self.reset_with_velocity(angle, 0.0);
    }

    #[inline]
    pub(crate) fn reset_with_velocity(&mut self, angle: f32, velocity: f32) {
        self.angle = wrap(angle);
        self.velocity = velocity;
        self.initialized = true;
    }

    #[inline]
    pub(crate) fn velocity(&self) -> f32 {
        self.velocity
    }

    #[inline]
    pub(crate) fn update(&mut self, angle: f32, dt: f32) -> f32 {
        if !self.initialized {
            self.reset(angle);
            return self.angle;
        }

        if dt <= 0.0 || !dt.is_finite() {
            return self.angle;
        }

        let error = shortest_angle_delta(self.angle, angle);
        self.velocity += self.ki * error * dt;
        let corrected_velocity = self.velocity + self.kp * error;
        self.angle = wrap(self.angle + corrected_velocity * dt);
        self.angle
    }
}
