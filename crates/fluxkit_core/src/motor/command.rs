use super::support::{current_limit, output_torque_limit, output_velocity_limit};
use super::*;

impl<M, CurrentEst> MotorController<M, CurrentEst>
where
    M: Modulator,
    CurrentEst: CurrentEstimator,
{
    /// Creates a new motor controller with an explicit modulation strategy.
    pub fn new(
        motor: MotorParams,
        inverter: InverterParams,
        actuator: ActuatorParams,
        config: CurrentLoopConfig,
        modulator: M,
        current_estimator: CurrentEst,
    ) -> Self {
        let static_voltage_limit = config
            .max_voltage_mag
            .get()
            .min(inverter.max_voltage_command.get())
            .max(0.0);

        let id_target = clamp(
            config.id_ref_default.get(),
            -current_limit(config.max_id_target, motor.limits.max_phase_current),
            current_limit(config.max_id_target, motor.limits.max_phase_current),
        );
        let velocity_limit = output_velocity_limit(config.max_velocity_target, actuator);
        let current_limit = current_limit(config.max_iq_target, motor.limits.max_phase_current);
        let output_torque_limit = output_torque_limit(current_limit, motor, actuator);

        let mut controller = Self {
            motor,
            inverter,
            actuator,
            config,
            modulator,
            current_estimator,
            command: ControllerCommand::Disabled,
            state: MotorState::Disabled,
            mode: ControlMode::Disabled,
            id_target: Amps::new(id_target),
            iq_target: Amps::ZERO,
            output_torque_target: NewtonMeters::ZERO,
            output_velocity_target: RadPerSec::ZERO,
            output_position_target: ContinuousMechanicalAngle::new(0.0),
            mit_kp: 0.0,
            mit_kd: 0.0,
            open_loop_voltage_target: zero_voltage_dq(),
            d_pi: PiController::new(PiConfig {
                kp: config.kp_d,
                ki: config.ki_d,
                out_min: -static_voltage_limit,
                out_max: static_voltage_limit,
            }),
            q_pi: PiController::new(PiConfig {
                kp: config.kp_q,
                ki: config.ki_q,
                out_min: -static_voltage_limit,
                out_max: static_voltage_limit,
            }),
            velocity_pi: PiController::new(PiConfig {
                kp: config.velocity_kp,
                ki: config.velocity_ki,
                out_min: -output_torque_limit,
                out_max: output_torque_limit,
            }),
            position_pi: PiController::new(PiConfig {
                kp: config.position_kp,
                ki: config.position_ki,
                out_min: -velocity_limit,
                out_max: velocity_limit,
            }),
            active_error: None,
            last_rotor: None,
            last_wrapped_mechanical_angle: None,
            last_wrapped_output_angle: None,
            last_current_ref: None,
            status: MotorStatus {
                state: MotorState::Disabled,
                mode: ControlMode::Disabled,
                active_error: None,
                last_bus_voltage: Volts::ZERO,
                last_winding_temperature_c: PHASE_RESISTANCE_REFERENCE_TEMP_C,
                last_measured_idq: zero_current_dq(),
                last_commanded_vdq: zero_voltage_dq(),
                last_voltage_utilization: 0.0,
                last_flux_weakening_id: Amps::ZERO,
                last_flux_weakening_active: false,
                last_rotor_mechanical_angle: ContinuousMechanicalAngle::new(0.0),
                last_unwrapped_rotor_mechanical_angle: ContinuousMechanicalAngle::new(0.0),
                last_rotor_mechanical_velocity: RadPerSec::ZERO,
                last_output_mechanical_angle: ContinuousMechanicalAngle::new(0.0),
                last_unwrapped_output_mechanical_angle: ContinuousMechanicalAngle::new(0.0),
                last_output_mechanical_velocity: RadPerSec::ZERO,
                last_actuator_compensation: ActuatorCompensationTelemetry::zero(),
                last_saturated: false,
            },
        };

        if !validate_controller_config(
            &controller.motor,
            &controller.inverter,
            &controller.actuator,
            &controller.config,
        ) {
            controller.latch_error(Error::ConfigurationInvalid);
        }

        controller.refresh_status();
        controller
    }

    /// Consumes the controller back into its owned construction parts.
    pub fn into_parts(self) -> MotorControllerParts<M, CurrentEst> {
        MotorControllerParts {
            motor: self.motor,
            inverter: self.inverter,
            actuator: self.actuator,
            config: self.config,
            modulator: self.modulator,
            current_estimator: self.current_estimator,
        }
    }

    /// Applies one controller command, replacing any previously active target.
    pub(super) fn apply_command(&mut self, command: ControllerCommand) {
        self.command = command;
        match command {
            ControllerCommand::Disabled => {
                self.set_id_target(Amps::ZERO);
                self.set_iq_target(Amps::ZERO);
                self.set_torque_target(NewtonMeters::ZERO);
                self.set_velocity_target(RadPerSec::ZERO);
                self.set_position_target(ContinuousMechanicalAngle::new(0.0));
                self.set_open_loop_voltage_target(fluxkit_math::frame::Dq::new(
                    Volts::ZERO,
                    Volts::ZERO,
                ));
                self.set_mode(ControlMode::Disabled);
            }
            ControllerCommand::Current(idq_target) => {
                self.set_mode(ControlMode::Current);
                self.set_id_target(idq_target.d);
                self.set_iq_target(idq_target.q);
            }
            ControllerCommand::Torque(output_torque_target) => {
                self.set_mode(ControlMode::Torque);
                self.set_torque_target(output_torque_target);
            }
            ControllerCommand::Mit {
                position,
                velocity,
                kp,
                kd,
                torque_ff,
            } => {
                self.set_mode(ControlMode::Mit);
                self.set_mit_command(position, velocity, kp, kd, torque_ff);
            }
            ControllerCommand::Velocity(output_velocity_target) => {
                self.set_mode(ControlMode::Velocity);
                self.set_velocity_target(output_velocity_target);
            }
            ControllerCommand::Position(output_position_target) => {
                self.set_mode(ControlMode::Position);
                self.set_position_target(output_position_target);
            }
            ControllerCommand::OpenLoopVoltage(open_loop_voltage_target) => {
                self.set_mode(ControlMode::OpenLoopVoltage);
                self.set_open_loop_voltage_target(open_loop_voltage_target);
            }
        }

        self.refresh_status();
    }

    /// Applies actuator-side calibration data onto the live controller model.
    pub fn apply_actuator_calibration(&mut self, calibration: &ActuatorCalibration) {
        calibration.apply_to_actuator_params(&mut self.actuator);
        self.refresh_actuator_dependent_limits();
        self.refresh_status();
    }

    /// Returns whether friction compensation is enabled in the live actuator model.
    pub fn friction_compensation_enabled(&self) -> bool {
        self.actuator.compensation.friction.enabled
    }

    pub(super) fn set_mode(&mut self, mode: ControlMode) {
        if self.mode != mode {
            self.reset_control_state();
        }
        self.mode = mode;
        self.refresh_status();
    }

    pub(super) fn set_iq_target(&mut self, iq: Amps) {
        let limit = current_limit(
            self.config.max_iq_target,
            self.motor.limits.max_phase_current,
        );
        self.iq_target = Amps::new(clamp(iq.get(), -limit, limit));
    }

    pub(super) fn set_id_target(&mut self, id: Amps) {
        let limit = current_limit(
            self.config.max_id_target,
            self.motor.limits.max_phase_current,
        );
        self.id_target = Amps::new(clamp(id.get(), -limit, limit));
    }

    pub(super) fn set_torque_target(&mut self, torque: NewtonMeters) {
        self.output_torque_target = self.clamp_output_torque(torque);
    }

    pub(super) fn set_velocity_target(&mut self, velocity: RadPerSec) {
        let limit = output_velocity_limit(self.config.max_velocity_target, self.actuator);
        self.output_velocity_target = RadPerSec::new(clamp(velocity.get(), -limit, limit));
    }

    pub(super) fn set_position_target(&mut self, position: ContinuousMechanicalAngle) {
        self.output_position_target = position;
    }

    pub(super) fn set_mit_command(
        &mut self,
        position: ContinuousMechanicalAngle,
        velocity: RadPerSec,
        kp: f32,
        kd: f32,
        torque_ff: NewtonMeters,
    ) {
        self.output_position_target = position;
        self.set_velocity_target(velocity);
        self.output_torque_target = self.clamp_output_torque(torque_ff);
        self.mit_kp = if kp.is_finite() { kp.max(0.0) } else { 0.0 };
        self.mit_kd = if kd.is_finite() { kd.max(0.0) } else { 0.0 };
    }

    pub(super) fn set_open_loop_voltage_target(&mut self, voltage: fluxkit_math::frame::Dq<Volts>) {
        self.open_loop_voltage_target = voltage;
    }

    pub(super) fn enable(&mut self) {
        if self.active_error.is_none() && self.state == MotorState::Disabled {
            self.state = MotorState::Ready;
        }
        self.refresh_status();
    }

    /// Disarms the controller state machine without requiring a sampled control input.
    pub fn disable(&mut self) {
        self.reset_control_state();
        self.state = MotorState::Disabled;
        self.refresh_status();
    }

    /// Clears a latched error when the static configuration is valid.
    pub fn clear_error(&mut self) {
        if validate_controller_config(&self.motor, &self.inverter, &self.actuator, &self.config) {
            self.active_error = None;
            self.reset_control_state();
            self.state = MotorState::Disabled;
        }
        self.refresh_status();
    }

    fn refresh_actuator_dependent_limits(&mut self) {
        let velocity_limit = output_velocity_limit(self.config.max_velocity_target, self.actuator);
        let current_limit = current_limit(
            self.config.max_iq_target,
            self.motor.limits.max_phase_current,
        );
        let output_torque_limit = output_torque_limit(current_limit, self.motor, self.actuator);
        self.velocity_pi.cfg.out_min = -output_torque_limit;
        self.velocity_pi.cfg.out_max = output_torque_limit;
        self.position_pi.cfg.out_min = -velocity_limit;
        self.position_pi.cfg.out_max = velocity_limit;
    }

    pub(super) fn latch_error(&mut self, error: Error) {
        self.active_error = Some(error);
        self.state = MotorState::Faulted;
        self.reset_control_state();
    }

    pub(super) fn reset_control_state(&mut self) {
        self.d_pi.reset();
        self.q_pi.reset();
        self.velocity_pi.reset();
        self.position_pi.reset();
        self.current_estimator.reset();
        self.last_current_ref = None;
        self.output_torque_target = NewtonMeters::ZERO;
        self.mit_kp = 0.0;
        self.mit_kd = 0.0;
        self.status.last_actuator_compensation = ActuatorCompensationTelemetry::zero();
        self.status.last_flux_weakening_id = Amps::ZERO;
        self.status.last_flux_weakening_active = false;
        self.status.last_voltage_utilization = 0.0;
    }

    pub(super) fn neutral_output(&self, _error: Error) -> ControlOutput {
        ControlOutput {
            phase_duty: neutral_phase_duty(),
            measured_idq: self.status.last_measured_idq,
            commanded_vdq: zero_voltage_dq(),
            saturated: false,
        }
    }

    pub(super) fn refresh_status(&mut self) {
        self.status.state = self.state;
        self.status.mode = self.mode;
        self.status.active_error = self.active_error;
    }

    /// Returns the latest compact status snapshot.
    #[inline]
    pub const fn status(&self) -> MotorStatus {
        self.status
    }
}
