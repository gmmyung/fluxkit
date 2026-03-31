use super::support::{
    FrictionCompensationBreakdown, friction_direction_and_motion_weight, motor_torque_constant,
};
use super::*;

impl<M, CurrentEst> MotorController<M, CurrentEst>
where
    M: Modulator,
    CurrentEst: CurrentEstimator,
{
    pub(super) fn update_supervisory_references(&mut self, dt_seconds: f32) {
        if self.active_error.is_some() || self.state == MotorState::Disabled {
            self.refresh_status();
            return;
        }

        match self.mode {
            ControlMode::Disabled | ControlMode::Current | ControlMode::OpenLoopVoltage => {}
            ControlMode::Torque => {
                let total_output_torque =
                    self.total_output_torque_command(self.output_torque_target);
                let iq_target = self.output_torque_to_iq(total_output_torque);
                self.set_iq_target(iq_target);
            }
            ControlMode::Mit => {
                if self.last_rotor.is_some() {
                    let position_error = self.output_position_target.get()
                        - self.status.last_unwrapped_output_mechanical_angle.get();
                    let velocity_error = self.output_velocity_target.get()
                        - self.status.last_output_mechanical_velocity.get();
                    let output_torque = self.output_torque_target.get()
                        + self.mit_kp * position_error
                        + self.mit_kd * velocity_error;
                    self.output_torque_target =
                        self.clamp_output_torque(NewtonMeters::new(output_torque));
                    let total_output_torque =
                        self.total_output_torque_command(self.output_torque_target);
                    let iq_target = self.output_torque_to_iq(total_output_torque);
                    self.set_iq_target(iq_target);
                }
            }
            ControlMode::Velocity | ControlMode::Position => {
                if self.last_rotor.is_some() {
                    if self.mode == ControlMode::Position {
                        let velocity_command = self.position_pi.update(
                            self.output_position_target.get()
                                - self.status.last_unwrapped_output_mechanical_angle.get(),
                            dt_seconds,
                        );
                        self.set_velocity_target(RadPerSec::new(velocity_command));
                    }

                    let output_torque = self.velocity_pi.update(
                        self.output_velocity_target.get()
                            - self.status.last_output_mechanical_velocity.get(),
                        dt_seconds,
                    );
                    self.output_torque_target =
                        self.clamp_output_torque(NewtonMeters::new(output_torque));
                    let total_output_torque =
                        self.total_output_torque_command(self.output_torque_target);

                    let iq_target = self.output_torque_to_iq(total_output_torque);
                    self.set_iq_target(iq_target);
                }
            }
        }

        self.refresh_status();
    }

    pub(super) fn clamp_output_torque(&self, torque: NewtonMeters) -> NewtonMeters {
        let limit = self
            .actuator
            .limits
            .max_output_torque
            .map(|value| value.get())
            .unwrap_or(f32::INFINITY);
        NewtonMeters::new(clamp(torque.get(), -limit, limit))
    }

    fn total_output_torque_command(&mut self, feedback_torque: NewtonMeters) -> NewtonMeters {
        let compensation = self.compute_actuator_compensation(feedback_torque);
        self.status.last_actuator_compensation = compensation;
        compensation.total_output_torque_command
    }

    fn compute_actuator_compensation(
        &mut self,
        feedback_torque: NewtonMeters,
    ) -> ActuatorCompensationTelemetry {
        let config = self.actuator.compensation;
        let direction_hint_velocity = self.friction_direction_hint_velocity(feedback_torque);
        let measured_velocity = self.status.last_output_mechanical_velocity.get();
        let friction = self.compute_friction_compensation(
            feedback_torque,
            direction_hint_velocity,
            measured_velocity,
            config,
        );
        let compensation_unbounded = friction.total.get();
        let max_compensation = config.max_total_torque.get().max(0.0);
        let total_compensation_torque = NewtonMeters::new(clamp(
            compensation_unbounded,
            -max_compensation,
            max_compensation,
        ));
        let total_output_torque_command = self.clamp_output_torque(NewtonMeters::new(
            feedback_torque.get() + total_compensation_torque.get(),
        ));

        ActuatorCompensationTelemetry {
            feedback_torque,
            breakaway_torque: friction.breakaway,
            coulomb_torque: friction.coulomb,
            viscous_torque: friction.viscous,
            friction_torque: friction.total,
            total_compensation_torque,
            total_output_torque_command,
        }
    }

    fn friction_direction_hint_velocity(&self, feedback_torque: NewtonMeters) -> f32 {
        let measured_velocity = self.status.last_output_mechanical_velocity.get();
        let blend_band = self
            .actuator
            .compensation
            .friction
            .zero_velocity_blend_band
            .get()
            .max(1.0e-3);

        match self.mode {
            ControlMode::Mit | ControlMode::Velocity | ControlMode::Position => {
                if self.output_velocity_target.get().abs() > 1.0e-6 {
                    self.output_velocity_target.get()
                } else {
                    measured_velocity
                }
            }
            ControlMode::Torque => {
                if feedback_torque.get().abs() > 1.0e-6 {
                    feedback_torque.get().signum() * blend_band
                } else {
                    measured_velocity
                }
            }
            ControlMode::Disabled | ControlMode::Current | ControlMode::OpenLoopVoltage => {
                measured_velocity
            }
        }
    }

    fn compute_friction_compensation(
        &self,
        feedback_torque: NewtonMeters,
        direction_hint_velocity: f32,
        measured_velocity: f32,
        config: crate::actuator::ActuatorCompensationConfig,
    ) -> FrictionCompensationBreakdown {
        let friction = config.friction;
        if !friction.enabled {
            return FrictionCompensationBreakdown::zero();
        }

        let blend_band = friction.zero_velocity_blend_band.get().max(1.0e-6);
        let (direction, motion_weight) = friction_direction_and_motion_weight(
            direction_hint_velocity,
            measured_velocity,
            blend_band,
        );
        let positive_weight = 0.5 * (direction + 1.0);
        let negative_weight = 1.0 - positive_weight;
        let coulomb = positive_weight * friction.positive_coulomb_torque.get()
            + negative_weight * friction.negative_coulomb_torque.get();
        let breakaway = positive_weight * friction.positive_breakaway_torque.get()
            + negative_weight * friction.negative_breakaway_torque.get();
        let viscous_coefficient = positive_weight * friction.positive_viscous_coefficient
            + negative_weight * friction.negative_viscous_coefficient;
        let breakaway_weight = 1.0 - motion_weight;
        let command_along_direction = (feedback_torque.get() * direction).max(0.0);
        let remaining_static_margin = (breakaway - command_along_direction).max(0.0);
        let breakaway_magnitude = (breakaway_weight * breakaway).min(remaining_static_margin);
        let breakaway_torque = NewtonMeters::new(direction * breakaway_magnitude);
        let coulomb_torque = NewtonMeters::new(direction * coulomb);
        let viscous_torque = NewtonMeters::new(viscous_coefficient * measured_velocity);
        FrictionCompensationBreakdown {
            breakaway: breakaway_torque,
            coulomb: coulomb_torque,
            viscous: viscous_torque,
            total: NewtonMeters::new(
                breakaway_torque.get() + coulomb_torque.get() + viscous_torque.get(),
            ),
        }
    }

    fn output_torque_to_iq(&self, output_torque: NewtonMeters) -> Amps {
        let torque_constant = motor_torque_constant(self.motor);
        let motor_torque = output_torque.get() / self.actuator.gear_ratio;
        Amps::new(motor_torque / torque_constant * self.motor.electrical_direction.signum())
    }
}
