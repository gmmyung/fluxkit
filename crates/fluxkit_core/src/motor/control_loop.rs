use super::support::{current_limit, dq_is_finite, duty_is_finite};
use super::*;

struct CurrentControlFrame {
    estimated_idq_f32: fluxkit_math::frame::Dq<f32>,
    measured_idq: fluxkit_math::frame::Dq<Amps>,
    electrical_angle: f32,
    bus_voltage: Volts,
    winding_temperature_c: f32,
    mechanical_velocity: RadPerSec,
    dt_seconds: f32,
    voltage_limit: f32,
}

impl<M, CurrentEst> MotorController<M, CurrentEst>
where
    M: Modulator,
    CurrentEst: CurrentEstimator,
{
    pub(super) fn run_control_cycle(&mut self, input: ControlInput) -> ControlOutput {
        self.status.last_bus_voltage = input.bus_voltage;
        self.status.last_winding_temperature_c = input.winding_temperature_c;
        self.status.last_rotor_mechanical_angle = input.rotor.mechanical_angle;
        self.status.last_unwrapped_rotor_mechanical_angle =
            self.unwrap_rotor_angle(input.rotor.mechanical_angle);
        self.status.last_rotor_mechanical_velocity = input.rotor.mechanical_velocity;
        self.status.last_output_mechanical_angle = input.actuator.output_angle;
        self.status.last_unwrapped_output_mechanical_angle =
            self.unwrap_output_angle(input.actuator.output_angle);
        self.status.last_output_mechanical_velocity = input.actuator.output_velocity;

        if let Some(error) = self.active_error {
            self.state = MotorState::Faulted;
            self.refresh_status();
            return self.neutral_output(error);
        }

        if let Err(error) = validate_control_input(&input, &self.inverter) {
            self.latch_error(error);
            self.refresh_status();
            return self.neutral_output(error);
        }

        if self
            .motor
            .limits
            .max_winding_temperature_c
            .map(|limit_c| input.winding_temperature_c > limit_c)
            .unwrap_or(false)
        {
            self.latch_error(Error::OverTemperature);
            self.refresh_status();
            return self.neutral_output(Error::OverTemperature);
        }

        self.last_rotor = Some(input.rotor);
        let electrical_angle = self.electrical_angle_from_mechanical(input.rotor.mechanical_angle);
        let phase_currents = input.phase_currents.map(|current| current.get());
        let measured_alpha_beta = clarke(phase_currents);
        let measured_idq_f32 = park(measured_alpha_beta, electrical_angle.get());
        let measured_idq = measured_idq_f32.map(Amps::new);
        self.status.last_measured_idq = measured_idq;

        if !dq_is_finite(measured_idq_f32.d, measured_idq_f32.q) {
            self.latch_error(Error::NonFiniteComputation);
            self.refresh_status();
            return self.neutral_output(Error::NonFiniteComputation);
        }

        let estimated_idq = self
            .current_estimator
            .update(measured_idq, input.dt_seconds);
        let estimated_idq_f32 = estimated_idq.map(|current| current.get());
        if !dq_is_finite(estimated_idq_f32.d, estimated_idq_f32.q) {
            self.latch_error(Error::NonFiniteComputation);
            self.refresh_status();
            return self.neutral_output(Error::NonFiniteComputation);
        }

        if self.state == MotorState::Disabled || self.mode == ControlMode::Disabled {
            self.refresh_status();
            return ControlOutput {
                phase_duty: neutral_phase_duty(),
                measured_idq,
                commanded_vdq: zero_voltage_dq(),
                saturated: false,
            };
        }

        let voltage_limit = self.dynamic_voltage_limit(input.bus_voltage);
        match self.mode {
            ControlMode::Disabled => unreachable!("handled above"),
            ControlMode::Current
            | ControlMode::Torque
            | ControlMode::Mit
            | ControlMode::Velocity
            | ControlMode::Position => self.run_current_control(CurrentControlFrame {
                estimated_idq_f32,
                measured_idq,
                electrical_angle: electrical_angle.get(),
                bus_voltage: input.bus_voltage,
                winding_temperature_c: input.winding_temperature_c,
                mechanical_velocity: input.rotor.mechanical_velocity,
                dt_seconds: input.dt_seconds,
                voltage_limit,
            }),
            ControlMode::OpenLoopVoltage => self.run_open_loop_voltage(
                measured_idq,
                electrical_angle.get(),
                input.bus_voltage,
                voltage_limit,
            ),
        }
    }

    /// Executes one deterministic controller cycle.
    pub fn step(&mut self, input: ControlInput) -> ControlOutput {
        if input.clear_fault_requested {
            self.clear_error();
        }
        if input.armed {
            self.enable();
        } else {
            self.disable();
        }
        if input.command != self.command {
            self.apply_command(input.command);
        }
        let dt_seconds = input.dt_seconds;
        let output = self.run_control_cycle(input);
        self.update_supervisory_references(dt_seconds);
        output
    }

    fn dynamic_voltage_limit(&self, bus_voltage: Volts) -> f32 {
        let modulation_limit = self.modulator.linear_limit(bus_voltage).get();
        self.config
            .max_voltage_mag
            .get()
            .min(self.inverter.max_voltage_command.get())
            .min(modulation_limit)
            .max(0.0)
    }

    #[inline]
    fn electrical_angle_from_mechanical(
        &self,
        mechanical_angle: ContinuousMechanicalAngle,
    ) -> fluxkit_math::ElectricalAngle {
        let base = mechanical_to_electrical_with_direction(
            mechanical_angle,
            self.motor.pole_pairs as u32,
            self.motor.electrical_direction,
        );
        fluxkit_math::ElectricalAngle::new(base.get() + self.motor.electrical_angle_offset.get())
    }

    fn set_pi_output_limits(&mut self, limit: f32) {
        self.d_pi.cfg.out_min = -limit;
        self.d_pi.cfg.out_max = limit;
        self.q_pi.cfg.out_min = -limit;
        self.q_pi.cfg.out_max = limit;
    }

    fn run_current_control(&mut self, frame: CurrentControlFrame) -> ControlOutput {
        let base_current_ref = CurrentReference {
            id: self.id_target,
            iq: self.iq_target,
        };
        let current_ref = self.current_reference_with_flux_weakening(
            base_current_ref,
            frame.mechanical_velocity,
            frame.dt_seconds,
        );
        self.set_pi_output_limits(frame.voltage_limit);

        let feedforward = if self.config.enable_current_feedforward {
            let current_ref_derivative =
                self.current_reference_derivative(current_ref, frame.dt_seconds);
            self.current_feedforward(
                current_ref,
                current_ref_derivative,
                frame.mechanical_velocity,
                frame.winding_temperature_c,
            )
        } else {
            fluxkit_math::frame::Dq::new(0.0, 0.0)
        };
        self.last_current_ref = Some(fluxkit_math::frame::Dq::new(current_ref.id, current_ref.iq));

        let vd = self.d_pi.update_with_feedforward(
            current_ref.id.get() - frame.estimated_idq_f32.d,
            feedforward.d,
            frame.dt_seconds,
        );
        let vq = self.q_pi.update_with_feedforward(
            current_ref.iq.get() - frame.estimated_idq_f32.q,
            feedforward.q,
            frame.dt_seconds,
        );

        self.finalize_voltage_output(
            fluxkit_math::frame::Dq::new(vd, vq),
            frame.measured_idq,
            frame.electrical_angle,
            frame.bus_voltage,
            frame.voltage_limit,
        )
    }

    fn run_open_loop_voltage(
        &mut self,
        measured_idq: fluxkit_math::frame::Dq<Amps>,
        electrical_angle: f32,
        bus_voltage: Volts,
        voltage_limit: f32,
    ) -> ControlOutput {
        self.finalize_voltage_output(
            self.open_loop_voltage_target.map(|voltage| voltage.get()),
            measured_idq,
            electrical_angle,
            bus_voltage,
            voltage_limit,
        )
    }

    fn finalize_voltage_output(
        &mut self,
        requested_vdq: fluxkit_math::frame::Dq<f32>,
        measured_idq: fluxkit_math::frame::Dq<Amps>,
        electrical_angle: f32,
        bus_voltage: Volts,
        voltage_limit: f32,
    ) -> ControlOutput {
        if !dq_is_finite(requested_vdq.d, requested_vdq.q) {
            self.latch_error(Error::NonFiniteComputation);
            self.refresh_status();
            return self.neutral_output(Error::NonFiniteComputation);
        }

        let limited_vdq = limit_norm_dq(requested_vdq, voltage_limit);
        let controller_saturated = limited_vdq != requested_vdq;
        self.status.last_voltage_utilization = voltage_utilization(requested_vdq, voltage_limit);
        let voltage_alpha_beta = inverse_park(limited_vdq, electrical_angle);
        let modulation = self.modulator.modulate(voltage_alpha_beta, bus_voltage);

        let phase_duty = self.clamp_phase_duty(modulation.duty);
        let commanded_vdq = limited_vdq.map(Volts::new);
        self.status.last_commanded_vdq = commanded_vdq;
        self.status.last_saturated = controller_saturated || modulation.saturated;

        if !duty_is_finite(phase_duty) {
            self.latch_error(Error::NonFiniteComputation);
            self.refresh_status();
            return self.neutral_output(Error::NonFiniteComputation);
        }

        if matches!(self.state, MotorState::Ready | MotorState::Disabled) {
            self.state = MotorState::Running;
        }

        self.refresh_status();

        ControlOutput {
            phase_duty,
            measured_idq,
            commanded_vdq,
            saturated: self.status.last_saturated,
        }
    }

    fn current_feedforward(
        &self,
        current_ref: CurrentReference,
        current_ref_derivative: fluxkit_math::frame::Dq<f32>,
        mechanical_velocity: RadPerSec,
        winding_temperature_c: f32,
    ) -> fluxkit_math::frame::Dq<f32> {
        let omega_e = mechanical_velocity_to_electrical(
            mechanical_velocity,
            self.motor.pole_pairs as u32,
            self.motor.electrical_direction,
        )
        .get();
        let resistance = self.temperature_compensated_phase_resistance_ohm(winding_temperature_c);
        let ld = self.motor.d_inductance_h.get();
        let lq = self.motor.q_inductance_h.get();
        let flux_linkage = self.motor.flux_linkage_weber.get();
        let id = current_ref.id.get();
        let iq = current_ref.iq.get();

        fluxkit_math::frame::Dq::new(
            resistance * id + ld * current_ref_derivative.d - omega_e * lq * iq,
            resistance * iq + lq * current_ref_derivative.q + omega_e * (ld * id + flux_linkage),
        )
    }

    fn current_reference_with_flux_weakening(
        &mut self,
        base_current_ref: CurrentReference,
        mechanical_velocity: RadPerSec,
        dt_seconds: f32,
    ) -> CurrentReference {
        let config = self.config.flux_weakening;
        let electrical_speed = mechanical_velocity_to_electrical(
            mechanical_velocity,
            self.motor.pole_pairs as u32,
            self.motor.electrical_direction,
        )
        .get()
        .abs();
        if !config.enabled
            || !matches!(
                self.mode,
                ControlMode::Torque
                    | ControlMode::Mit
                    | ControlMode::Velocity
                    | ControlMode::Position
            )
            || electrical_speed < config.min_electrical_speed.get()
            || !dt_seconds.is_finite()
            || dt_seconds <= 0.0
        {
            self.flux_weakening_id = Amps::ZERO;
            self.status.last_flux_weakening_id = Amps::ZERO;
            self.status.last_flux_weakening_active = false;
            return base_current_ref;
        }

        let utilization_error =
            self.status.last_voltage_utilization - config.voltage_utilization_target;
        let id_step = utilization_error * config.bandwidth.get() * dt_seconds;
        let max_negative_id =
            current_limit(config.max_negative_id, self.motor.limits.max_phase_current).min(
                current_limit(
                    self.config.max_id_target,
                    self.motor.limits.max_phase_current,
                ),
            );
        let flux_weakening_id = clamp(
            self.flux_weakening_id.get() - id_step,
            -max_negative_id,
            0.0,
        );
        self.flux_weakening_id = Amps::new(flux_weakening_id);

        let id_limit = current_limit(
            self.config.max_id_target,
            self.motor.limits.max_phase_current,
        );
        let id = clamp(
            base_current_ref.id.get() + flux_weakening_id,
            -id_limit,
            id_limit,
        );
        let iq_limit = self.iq_limit_for_id(id);
        let iq = clamp(base_current_ref.iq.get(), -iq_limit, iq_limit);

        self.status.last_flux_weakening_id = self.flux_weakening_id;
        self.status.last_flux_weakening_active = flux_weakening_id < 0.0;

        CurrentReference {
            id: Amps::new(id),
            iq: Amps::new(iq),
        }
    }

    fn iq_limit_for_id(&self, id: f32) -> f32 {
        let configured_limit = current_limit(
            self.config.max_iq_target,
            self.motor.limits.max_phase_current,
        );
        let phase_limit = self.motor.limits.max_phase_current.get();
        let id_abs = id.abs();
        if id_abs >= phase_limit {
            return 0.0;
        }

        let remaining_phase_current = sqrt(phase_limit * phase_limit - id_abs * id_abs).max(0.0);
        configured_limit.min(remaining_phase_current)
    }

    fn temperature_compensated_phase_resistance_ohm(&self, winding_temperature_c: f32) -> f32 {
        let scale = 1.0
            + PHASE_RESISTANCE_TEMP_COEFF_PER_C
                * (winding_temperature_c - PHASE_RESISTANCE_REFERENCE_TEMP_C);
        let scale = if scale.is_finite() {
            scale.max(0.0)
        } else {
            0.0
        };
        self.motor.phase_resistance_ohm_ref.get() * scale
    }

    fn current_reference_derivative(
        &self,
        current_ref: CurrentReference,
        dt_seconds: f32,
    ) -> fluxkit_math::frame::Dq<f32> {
        if !dt_seconds.is_finite() || dt_seconds <= 0.0 {
            return fluxkit_math::frame::Dq::new(0.0, 0.0);
        }

        let Some(last_ref) = self.last_current_ref else {
            return fluxkit_math::frame::Dq::new(0.0, 0.0);
        };

        fluxkit_math::frame::Dq::new(
            clamp(
                (current_ref.id.get() - last_ref.d.get()) / dt_seconds,
                -self.config.max_current_ref_derivative_amps_per_sec,
                self.config.max_current_ref_derivative_amps_per_sec,
            ),
            clamp(
                (current_ref.iq.get() - last_ref.q.get()) / dt_seconds,
                -self.config.max_current_ref_derivative_amps_per_sec,
                self.config.max_current_ref_derivative_amps_per_sec,
            ),
        )
    }

    fn unwrap_rotor_angle(
        &mut self,
        wrapped_angle: ContinuousMechanicalAngle,
    ) -> ContinuousMechanicalAngle {
        let unwrapped = match self.last_wrapped_mechanical_angle {
            Some(previous_wrapped) => {
                let delta = shortest_angle_delta(previous_wrapped.get(), wrapped_angle.get());
                ContinuousMechanicalAngle::new(
                    self.status.last_unwrapped_rotor_mechanical_angle.get() + delta,
                )
            }
            None => wrapped_angle,
        };

        self.last_wrapped_mechanical_angle = Some(wrapped_angle);
        unwrapped
    }

    fn unwrap_output_angle(
        &mut self,
        wrapped_angle: ContinuousMechanicalAngle,
    ) -> ContinuousMechanicalAngle {
        let unwrapped = match self.last_wrapped_output_angle {
            Some(previous_wrapped) => {
                let delta = shortest_angle_delta(previous_wrapped.get(), wrapped_angle.get());
                ContinuousMechanicalAngle::new(
                    self.status.last_unwrapped_output_mechanical_angle.get() + delta,
                )
            }
            None => wrapped_angle,
        };

        self.last_wrapped_output_angle = Some(wrapped_angle);
        unwrapped
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
}

#[inline]
fn voltage_utilization(requested_vdq: fluxkit_math::frame::Dq<f32>, voltage_limit: f32) -> f32 {
    if !voltage_limit.is_finite() || voltage_limit <= 0.0 {
        return 0.0;
    }

    let mag2 = requested_vdq.d * requested_vdq.d + requested_vdq.q * requested_vdq.q;
    if !mag2.is_finite() || mag2 <= 0.0 {
        return 0.0;
    }

    sqrt(mag2) / voltage_limit
}
