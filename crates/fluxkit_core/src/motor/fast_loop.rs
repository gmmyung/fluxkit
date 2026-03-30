use super::support::{dq_is_finite, duty_is_finite};
use super::*;

impl<M> MotorController<M>
where
    M: Modulator,
{
    pub(super) fn fast_tick(&mut self, input: FastLoopInput) -> FastLoopOutput {
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

        if let Err(error) = validate_fast_loop_input(&input, &self.inverter) {
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

        if self.state == MotorState::Disabled || self.mode == ControlMode::Disabled {
            self.refresh_status();
            return FastLoopOutput {
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
            | ControlMode::Position => self.run_current_control(
                measured_idq_f32,
                measured_idq,
                electrical_angle.get(),
                input.bus_voltage,
                input.winding_temperature_c,
                input.rotor.mechanical_velocity,
                input.dt_seconds,
                voltage_limit,
            ),
            ControlMode::OpenLoopVoltage => self.run_open_loop_voltage(
                measured_idq,
                electrical_angle.get(),
                input.bus_voltage,
                voltage_limit,
            ),
        }
    }

    /// Executes one deterministic controller cycle.
    pub fn step(&mut self, input: FastLoopInput, dt_seconds: f32) -> FastLoopOutput {
        let output = self.fast_tick(input);
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

    fn run_current_control(
        &mut self,
        measured_idq_f32: fluxkit_math::frame::Dq<f32>,
        measured_idq: fluxkit_math::frame::Dq<Amps>,
        electrical_angle: f32,
        bus_voltage: Volts,
        winding_temperature_c: f32,
        mechanical_velocity: RadPerSec,
        dt_seconds: f32,
        voltage_limit: f32,
    ) -> FastLoopOutput {
        let current_ref = CurrentReference {
            id: self.id_target,
            iq: self.iq_target,
        };
        self.set_pi_output_limits(voltage_limit);

        let feedforward = if self.config.enable_current_feedforward {
            let current_ref_derivative = self.current_reference_derivative(current_ref, dt_seconds);
            self.current_feedforward(
                current_ref,
                current_ref_derivative,
                mechanical_velocity,
                winding_temperature_c,
            )
        } else {
            fluxkit_math::frame::Dq::new(0.0, 0.0)
        };
        self.last_current_ref = Some(fluxkit_math::frame::Dq::new(current_ref.id, current_ref.iq));

        let vd = self.d_pi.update_with_feedforward(
            current_ref.id.get() - measured_idq_f32.d,
            feedforward.d,
            dt_seconds,
        );
        let vq = self.q_pi.update_with_feedforward(
            current_ref.iq.get() - measured_idq_f32.q,
            feedforward.q,
            dt_seconds,
        );

        self.finalize_voltage_output(
            fluxkit_math::frame::Dq::new(vd, vq),
            measured_idq,
            electrical_angle,
            bus_voltage,
            voltage_limit,
        )
    }

    fn run_open_loop_voltage(
        &mut self,
        measured_idq: fluxkit_math::frame::Dq<Amps>,
        electrical_angle: f32,
        bus_voltage: Volts,
        voltage_limit: f32,
    ) -> FastLoopOutput {
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
    ) -> FastLoopOutput {
        if !dq_is_finite(requested_vdq.d, requested_vdq.q) {
            self.latch_error(Error::NonFiniteComputation);
            self.refresh_status();
            return self.neutral_output(Error::NonFiniteComputation);
        }

        let limited_vdq = limit_norm_dq(requested_vdq, voltage_limit);
        let controller_saturated = limited_vdq != requested_vdq;
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

        FastLoopOutput {
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
