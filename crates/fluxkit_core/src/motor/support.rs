use super::*;

#[inline]
pub(super) fn current_limit(configured: Amps, motor_limit: Amps) -> f32 {
    configured.get().min(motor_limit.get())
}

#[inline]
pub(super) fn output_velocity_limit(configured: RadPerSec, actuator: ActuatorParams) -> f32 {
    actuator
        .limits
        .max_output_velocity
        .map(|limit| configured.get().min(limit.get()))
        .unwrap_or(configured.get())
}

#[derive(Clone, Copy)]
pub(super) struct FrictionCompensationBreakdown {
    pub(super) breakaway: NewtonMeters,
    pub(super) coulomb: NewtonMeters,
    pub(super) viscous: NewtonMeters,
    pub(super) total: NewtonMeters,
}

impl FrictionCompensationBreakdown {
    pub(super) const fn zero() -> Self {
        Self {
            breakaway: NewtonMeters::ZERO,
            coulomb: NewtonMeters::ZERO,
            viscous: NewtonMeters::ZERO,
            total: NewtonMeters::ZERO,
        }
    }
}

#[inline]
pub(super) fn output_torque_limit(
    max_iq_target: f32,
    motor: MotorParams,
    actuator: ActuatorParams,
) -> f32 {
    let configured_limit = max_iq_target * motor_torque_constant(motor) * actuator.gear_ratio;

    actuator
        .limits
        .max_output_torque
        .map(|limit| configured_limit.min(limit.get()))
        .unwrap_or(configured_limit)
}

#[inline]
pub(super) fn motor_torque_constant(motor: MotorParams) -> f32 {
    1.5 * motor.pole_pairs as f32 * motor.flux_linkage_weber.get()
}

#[inline]
pub(super) fn friction_direction_and_motion_weight(
    direction_hint_velocity: f32,
    measured_velocity: f32,
    blend_band: f32,
) -> (f32, f32) {
    let direction_hint = clamp(direction_hint_velocity / blend_band, -1.0, 1.0);
    let measured_direction = clamp(measured_velocity / blend_band, -1.0, 1.0);
    let motion_weight = clamp(measured_velocity.abs() / blend_band, 0.0, 1.0);
    let direction = clamp(
        direction_hint * (1.0 - motion_weight) + measured_direction * motion_weight,
        -1.0,
        1.0,
    );
    (direction, motion_weight)
}

#[inline]
pub(super) fn dq_is_finite(d: f32, q: f32) -> bool {
    d.is_finite() && q.is_finite()
}

#[inline]
pub(super) fn duty_is_finite(duty: fluxkit_math::modulation::PhaseDuty) -> bool {
    duty.a.get().is_finite() && duty.b.get().is_finite() && duty.c.get().is_finite()
}
