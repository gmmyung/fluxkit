//! Main motor controller implementation.

use fluxkit_math::{
    Modulator, PiConfig, PiController, Svpwm, clamp, clarke, inverse_park, limit_norm_dq, park,
    units::{Amps, Volts},
};

use crate::{
    config::CurrentLoopConfig,
    control::current::CurrentReference,
    fault::FaultKind,
    io::{FastLoopInput, FastLoopOutput},
    mode::ControlMode,
    params::{InverterParams, MotorParams},
    state::MotorState,
    status::MotorStatus,
    util::{neutral_phase_duty, zero_current_dq, zero_voltage_dq},
    validation::{validate_controller_config, validate_fast_loop_input},
};

/// Pure control-engine state for a single motor.
#[derive(Clone, Debug)]
pub struct MotorController {
    motor: MotorParams,
    inverter: InverterParams,
    config: CurrentLoopConfig,
    state: MotorState,
    mode: ControlMode,
    id_target: Amps,
    iq_target: Amps,
    d_pi: PiController,
    q_pi: PiController,
    active_fault: Option<FaultKind>,
    status: MotorStatus,
}

impl MotorController {
    /// Creates a new motor controller with explicit params and tuning.
    pub fn new(motor: MotorParams, inverter: InverterParams, config: CurrentLoopConfig) -> Self {
        let static_voltage_limit = config
            .max_voltage_mag
            .get()
            .min(inverter.max_voltage_command.get())
            .max(0.0);

        let id_target = clamp(
            config.id_ref_default.get(),
            -current_limit(config.max_id_target, motor.max_phase_current),
            current_limit(config.max_id_target, motor.max_phase_current),
        );

        let mut controller = Self {
            motor,
            inverter,
            config,
            state: MotorState::Disabled,
            mode: ControlMode::Disabled,
            id_target: Amps::new(id_target),
            iq_target: Amps::ZERO,
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
            active_fault: None,
            status: MotorStatus {
                state: MotorState::Disabled,
                mode: ControlMode::Disabled,
                active_fault: None,
                last_bus_voltage: Volts::ZERO,
                last_measured_idq: zero_current_dq(),
                last_commanded_vdq: zero_voltage_dq(),
                last_saturated: false,
                last_rotor_source: None,
            },
        };

        if !validate_controller_config(&controller.motor, &controller.inverter, &controller.config)
        {
            controller.latch_fault(FaultKind::ConfigurationInvalid);
        }

        controller.refresh_status();
        controller
    }

    /// Returns the static motor parameters.
    #[inline]
    pub const fn motor_params(&self) -> &MotorParams {
        &self.motor
    }

    /// Returns the static inverter parameters.
    #[inline]
    pub const fn inverter_params(&self) -> &InverterParams {
        &self.inverter
    }

    /// Returns the runtime current-loop configuration.
    #[inline]
    pub const fn config(&self) -> &CurrentLoopConfig {
        &self.config
    }

    /// Selects the active control mode.
    pub fn set_mode(&mut self, mode: ControlMode) {
        self.mode = mode;
        self.refresh_status();
    }

    /// Updates the `q`-axis current target, clamped to configured limits.
    pub fn set_iq_target(&mut self, iq: Amps) {
        let limit = current_limit(self.config.max_iq_target, self.motor.max_phase_current);
        self.iq_target = Amps::new(clamp(iq.get(), -limit, limit));
    }

    /// Updates the `d`-axis current target, clamped to configured limits.
    pub fn set_id_target(&mut self, id: Amps) {
        let limit = current_limit(self.config.max_id_target, self.motor.max_phase_current);
        self.id_target = Amps::new(clamp(id.get(), -limit, limit));
    }

    /// Enables the controller if no fault is latched.
    pub fn enable(&mut self) {
        if self.active_fault.is_none() && self.state == MotorState::Disabled {
            self.state = MotorState::Ready;
        }
        self.refresh_status();
    }

    /// Disables the controller and clears dynamic controller state.
    pub fn disable(&mut self) {
        self.d_pi.reset();
        self.q_pi.reset();
        self.state = MotorState::Disabled;
        self.refresh_status();
    }

    /// Clears a latched fault when the static configuration is valid.
    pub fn clear_fault(&mut self) {
        if validate_controller_config(&self.motor, &self.inverter, &self.config) {
            self.active_fault = None;
            self.d_pi.reset();
            self.q_pi.reset();
            self.state = MotorState::Disabled;
        }
        self.refresh_status();
    }

    /// Executes the synchronous high-rate current loop.
    pub fn fast_tick(&mut self, input: FastLoopInput) -> FastLoopOutput {
        self.status.last_bus_voltage = input.bus_voltage;
        self.status.last_rotor_source = Some(input.rotor.source);

        if let Some(fault) = self.active_fault {
            self.state = MotorState::Faulted;
            self.refresh_status();
            return self.neutral_output(fault);
        }

        if let Err(fault) = validate_fast_loop_input(&input, &self.inverter) {
            self.latch_fault(fault);
            self.refresh_status();
            return self.neutral_output(fault);
        }

        let electrical_angle = input.rotor.electrical_angle.wrapped_pm_pi().get();
        let phase_currents = input.phase_currents.map(|current| current.get());
        let measured_alpha_beta = clarke(phase_currents);
        let measured_idq_f32 = park(measured_alpha_beta, electrical_angle);
        let measured_idq = measured_idq_f32.map(Amps::new);
        self.status.last_measured_idq = measured_idq;

        if !dq_is_finite(measured_idq_f32.d, measured_idq_f32.q) {
            self.latch_fault(FaultKind::NonFiniteComputation);
            self.refresh_status();
            return self.neutral_output(FaultKind::NonFiniteComputation);
        }

        if self.state == MotorState::Disabled || self.mode == ControlMode::Disabled {
            self.refresh_status();
            return FastLoopOutput {
                phase_duty: neutral_phase_duty(),
                measured_idq,
                commanded_vdq: zero_voltage_dq(),
                saturated: false,
                fault: None,
            };
        }

        if !matches!(self.mode, ControlMode::Current | ControlMode::Torque) {
            self.refresh_status();
            return FastLoopOutput {
                phase_duty: neutral_phase_duty(),
                measured_idq,
                commanded_vdq: zero_voltage_dq(),
                saturated: false,
                fault: None,
            };
        }

        let current_ref = CurrentReference {
            id: self.id_target,
            iq: self.iq_target,
        };
        let voltage_limit = self.dynamic_voltage_limit(input.bus_voltage);
        self.set_pi_output_limits(voltage_limit);

        let vd = self
            .d_pi
            .update(current_ref.id.get() - measured_idq_f32.d, input.dt_seconds);
        let vq = self
            .q_pi
            .update(current_ref.iq.get() - measured_idq_f32.q, input.dt_seconds);

        if !dq_is_finite(vd, vq) {
            self.latch_fault(FaultKind::NonFiniteComputation);
            self.refresh_status();
            return self.neutral_output(FaultKind::NonFiniteComputation);
        }

        let requested_vdq = fluxkit_math::frame::Dq::new(vd, vq);
        let limited_vdq = limit_norm_dq(requested_vdq, voltage_limit);
        let controller_saturated = limited_vdq != requested_vdq;
        let voltage_alpha_beta = inverse_park(limited_vdq, electrical_angle);
        let modulation = Svpwm.modulate(voltage_alpha_beta, input.bus_voltage);

        let phase_duty = self.clamp_phase_duty(modulation.duty);
        let commanded_vdq = limited_vdq.map(Volts::new);
        self.status.last_commanded_vdq = commanded_vdq;
        self.status.last_saturated = controller_saturated || modulation.saturated;

        if !duty_is_finite(phase_duty) {
            self.latch_fault(FaultKind::NonFiniteComputation);
            self.refresh_status();
            return self.neutral_output(FaultKind::NonFiniteComputation);
        }

        if matches!(self.state, MotorState::Ready | MotorState::Disabled) {
            self.state = MotorState::Running;
        }

        self.refresh_status();

        FastLoopOutput {
            phase_duty,
            measured_idq,
            commanded_vdq,
            saturated: self.status.last_saturated,
            fault: None,
        }
    }

    /// Reserved medium-rate hook for future supervisory control.
    pub fn medium_tick(&mut self, _dt_seconds: f32) {}

    /// Reserved slow-rate hook for future state transitions and calibration.
    pub fn slow_tick(&mut self, _dt_seconds: f32) {}

    /// Returns the latest compact status snapshot.
    #[inline]
    pub const fn status(&self) -> MotorStatus {
        self.status
    }

    fn dynamic_voltage_limit(&self, bus_voltage: Volts) -> f32 {
        let modulation_limit = fluxkit_math::svpwm_linear_limit(bus_voltage).get();
        self.config
            .max_voltage_mag
            .get()
            .min(self.inverter.max_voltage_command.get())
            .min(modulation_limit)
            .max(0.0)
    }

    fn set_pi_output_limits(&mut self, limit: f32) {
        self.d_pi.cfg.out_min = -limit;
        self.d_pi.cfg.out_max = limit;
        self.q_pi.cfg.out_min = -limit;
        self.q_pi.cfg.out_max = limit;
    }

    fn clamp_phase_duty(
        &self,
        duty: fluxkit_math::modulation::PhaseDuty,
    ) -> fluxkit_math::modulation::PhaseDuty {
        duty.map(|phase| {
            fluxkit_math::units::Duty::new(clamp(
                phase.get(),
                self.inverter.min_duty.get(),
                self.inverter.max_duty.get(),
            ))
        })
    }

    fn latch_fault(&mut self, fault: FaultKind) {
        self.active_fault = Some(fault);
        self.state = MotorState::Faulted;
        self.d_pi.reset();
        self.q_pi.reset();
    }

    fn neutral_output(&self, fault: FaultKind) -> FastLoopOutput {
        FastLoopOutput {
            phase_duty: neutral_phase_duty(),
            measured_idq: self.status.last_measured_idq,
            commanded_vdq: zero_voltage_dq(),
            saturated: false,
            fault: Some(fault),
        }
    }

    fn refresh_status(&mut self) {
        self.status.state = self.state;
        self.status.mode = self.mode;
        self.status.active_fault = self.active_fault;
    }
}

#[inline]
fn current_limit(configured: Amps, motor_limit: Amps) -> f32 {
    configured.get().min(motor_limit.get())
}

#[inline]
fn dq_is_finite(d: f32, q: f32) -> bool {
    d.is_finite() && q.is_finite()
}

#[inline]
fn duty_is_finite(duty: fluxkit_math::modulation::PhaseDuty) -> bool {
    duty.a.get().is_finite() && duty.b.get().is_finite() && duty.c.get().is_finite()
}

#[cfg(test)]
mod tests {
    use super::MotorController;
    use crate::{
        config::CurrentLoopConfig,
        fault::FaultKind,
        io::{AngleSource, FastLoopInput, RotorEstimate},
        mode::ControlMode,
        params::{InverterParams, MotorParams},
        state::MotorState,
    };
    use fluxkit_math::{
        ElectricalAngle,
        frame::{Abc, Dq},
        units::{Amps, Duty, Henries, Hertz, Ohms, RadPerSec, Volts},
    };

    fn test_motor() -> MotorParams {
        MotorParams {
            pole_pairs: 7,
            phase_resistance_ohm: Ohms::new(0.12),
            d_inductance_h: Henries::new(0.000_03),
            q_inductance_h: Henries::new(0.000_03),
            flux_linkage_weber: None,
            max_phase_current: Amps::new(20.0),
            max_mech_speed: None,
            torque_constant_nm_per_amp: None,
        }
    }

    fn test_inverter() -> InverterParams {
        InverterParams {
            pwm_frequency_hz: Hertz::new(20_000.0),
            deadtime_ns: 500,
            min_duty: Duty::new(0.0),
            max_duty: Duty::new(1.0),
            min_bus_voltage: Volts::new(6.0),
            max_bus_voltage: Volts::new(60.0),
            max_voltage_command: Volts::new(24.0),
        }
    }

    fn test_config() -> CurrentLoopConfig {
        CurrentLoopConfig {
            kp_d: 2.0,
            ki_d: 50.0,
            kp_q: 2.0,
            ki_q: 50.0,
            max_voltage_mag: Volts::new(12.0),
            id_ref_default: Amps::ZERO,
            max_id_target: Amps::new(10.0),
            max_iq_target: Amps::new(10.0),
            medium_loop_decimation: Some(10),
            slow_loop_decimation: Some(100),
        }
    }

    fn test_input() -> FastLoopInput {
        FastLoopInput {
            phase_currents: Abc::new(Amps::ZERO, Amps::ZERO, Amps::ZERO),
            bus_voltage: Volts::new(24.0),
            rotor: RotorEstimate {
                electrical_angle: ElectricalAngle::new(0.0),
                mechanical_velocity: RadPerSec::ZERO,
                source: AngleSource::Encoder,
            },
            dt_seconds: 1.0 / 20_000.0,
        }
    }

    #[test]
    fn zero_input_zero_target_returns_neutral_output() {
        let mut controller = MotorController::new(test_motor(), test_inverter(), test_config());
        controller.set_mode(ControlMode::Current);
        controller.enable();

        let output = controller.fast_tick(test_input());

        assert_eq!(output.phase_duty.a.get(), 0.5);
        assert_eq!(output.phase_duty.b.get(), 0.5);
        assert_eq!(output.phase_duty.c.get(), 0.5);
        assert_eq!(output.measured_idq, Dq::new(Amps::ZERO, Amps::ZERO));
        assert_eq!(output.commanded_vdq, Dq::new(Volts::ZERO, Volts::ZERO));
        assert!(!output.saturated);
        assert_eq!(output.fault, None);
        assert_eq!(controller.status().state, MotorState::Running);
    }

    #[test]
    fn invalid_bus_voltage_latches_fault_and_centers_output() {
        let mut controller = MotorController::new(test_motor(), test_inverter(), test_config());
        controller.set_mode(ControlMode::Current);
        controller.enable();

        let mut input = test_input();
        input.bus_voltage = Volts::new(0.0);
        let output = controller.fast_tick(input);

        assert_eq!(output.fault, Some(FaultKind::InvalidBusVoltage));
        assert_eq!(output.phase_duty.a.get(), 0.5);
        assert_eq!(controller.status().state, MotorState::Faulted);
        assert_eq!(
            controller.status().active_fault,
            Some(FaultKind::InvalidBusVoltage)
        );
    }

    #[test]
    fn invalid_angle_latches_fault() {
        let mut controller = MotorController::new(test_motor(), test_inverter(), test_config());
        controller.set_mode(ControlMode::Current);
        controller.enable();

        let mut input = test_input();
        input.rotor.electrical_angle = ElectricalAngle::new(f32::NAN);
        let output = controller.fast_tick(input);

        assert_eq!(output.fault, Some(FaultKind::InvalidRotorAngle));
        assert_eq!(controller.status().state, MotorState::Faulted);
    }

    #[test]
    fn positive_iq_target_produces_positive_vq() {
        let mut controller = MotorController::new(test_motor(), test_inverter(), test_config());
        controller.set_mode(ControlMode::Current);
        controller.set_iq_target(Amps::new(3.0));
        controller.enable();

        let output = controller.fast_tick(test_input());

        assert!(output.commanded_vdq.q.get() > 0.0);
        assert_eq!(output.fault, None);
    }

    #[test]
    fn saturation_is_reported_for_aggressive_current_request() {
        let mut config = test_config();
        config.kp_q = 50.0;
        config.max_voltage_mag = Volts::new(4.0);

        let mut controller = MotorController::new(test_motor(), test_inverter(), config);
        controller.set_mode(ControlMode::Current);
        controller.set_id_target(Amps::new(10.0));
        controller.set_iq_target(Amps::new(10.0));
        controller.enable();

        let output = controller.fast_tick(test_input());

        assert!(output.saturated);
        assert!(output.phase_duty.a.get() >= 0.0 && output.phase_duty.a.get() <= 1.0);
        assert!(output.phase_duty.b.get() >= 0.0 && output.phase_duty.b.get() <= 1.0);
        assert!(output.phase_duty.c.get() >= 0.0 && output.phase_duty.c.get() <= 1.0);
    }

    #[test]
    fn duty_stays_within_configured_duty_window() {
        let mut inverter = test_inverter();
        inverter.min_duty = Duty::new(0.1);
        inverter.max_duty = Duty::new(0.9);

        let mut controller = MotorController::new(test_motor(), inverter, test_config());
        controller.set_mode(ControlMode::Current);
        controller.set_iq_target(Amps::new(10.0));
        controller.enable();

        let output = controller.fast_tick(test_input());

        for duty in [
            output.phase_duty.a,
            output.phase_duty.b,
            output.phase_duty.c,
        ] {
            assert!(duty.get() >= 0.1 && duty.get() <= 0.9);
        }
    }

    #[test]
    fn state_transitions_are_explicit() {
        let mut controller = MotorController::new(test_motor(), test_inverter(), test_config());

        assert_eq!(controller.status().state, MotorState::Disabled);

        controller.enable();
        assert_eq!(controller.status().state, MotorState::Ready);

        controller.set_mode(ControlMode::Current);
        controller.fast_tick(test_input());
        assert_eq!(controller.status().state, MotorState::Running);

        let mut bad_input = test_input();
        bad_input.bus_voltage = Volts::new(100.0);
        controller.fast_tick(bad_input);
        assert_eq!(controller.status().state, MotorState::Faulted);

        controller.clear_fault();
        assert_eq!(controller.status().state, MotorState::Disabled);

        controller.disable();
        assert_eq!(controller.status().state, MotorState::Disabled);
    }
}
