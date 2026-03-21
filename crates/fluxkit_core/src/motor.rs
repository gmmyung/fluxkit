//! Main motor controller implementation.

use fluxkit_math::angle::shortest_angle_delta;
use fluxkit_math::{
    MechanicalAngle, Modulator, PiConfig, PiController, Svpwm, clamp, clarke, inverse_park,
    limit_norm_dq, park,
    units::{Amps, NewtonMeters, RadPerSec, Volts},
};

use crate::{
    config::CurrentLoopConfig,
    control::current::CurrentReference,
    error::Error,
    io::{FastLoopInput, FastLoopOutput, RotorEstimate},
    mode::ControlMode,
    params::{InverterParams, MotorParams},
    state::MotorState,
    status::MotorStatus,
    util::{neutral_phase_duty, zero_current_dq, zero_voltage_dq},
    validation::{validate_controller_config, validate_fast_loop_input},
};

#[cfg_attr(doc, aquamarine::aquamarine)]
/// Pure control-engine state for a single motor.
///
/// Control-loop ownership is intentionally explicit:
///
/// - `fast_tick()`
///   - validates input
///   - measures `id/iq`
///   - runs the current loop
///   - applies modulation
/// - `medium_tick()`
///   - updates supervisory references for `Torque`, `Velocity`, and `Position`
///   - in `Position` mode, both the position loop and the velocity loop run in the same call
/// - `slow_tick()`
///   - currently a reserved hook with no control behavior
///
/// ```mermaid
/// flowchart LR
///     subgraph S[medium_tick supervisory path]
///         PT[Position target] --> PP[Position PI]
///         RA[Mechanical angle] --> PP
///         PP --> VT[Velocity target]
///         TT[Torque target] --> TM[Torque to iq mapping]
///         VT --> VP[Velocity PI]
///         RV[Mechanical velocity] --> VP
///         VP --> IQ[iq target]
///         TM --> IQ
///     end
///     A[Phase currents abc] --> B[Input validation]
///     V[Bus voltage] --> B
///     R[Rotor estimate] --> B
///     T[dt_seconds] --> B
///     B --> C[Clarke transform]
///     C --> D[Park transform]
///     D --> E[Measure id iq]
///     ID[id target] --> F[Current reference]
///     IQ --> F
///     OL[Open-loop vdq target] --> VL[Voltage select]
///     F --> FD[Model feedforward]
///     F --> ER[Current error]
///     E --> ER
///     ER --> G[d-axis PI]
///     ER --> H[q-axis PI]
///     FD --> G
///     FD --> H
///     G --> CV[Closed-loop vdq]
///     H --> CV
///     CV --> VL
///     VL --> L[Vector limit in dq]
///     L --> M[Inverse Park]
///     M --> N[Configured modulator]
///     N --> O[Duty clamp]
///     O --> P[Phase duty output]
///     L --> Q[Status snapshot]
///     E --> Q
///     R --> Q
/// ```
#[derive(Clone, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct MotorController<M = Svpwm> {
    motor: MotorParams,
    inverter: InverterParams,
    config: CurrentLoopConfig,
    modulator: M,
    state: MotorState,
    mode: ControlMode,
    id_target: Amps,
    iq_target: Amps,
    torque_target: NewtonMeters,
    velocity_target: RadPerSec,
    position_target: MechanicalAngle,
    open_loop_voltage_target: fluxkit_math::frame::Dq<Volts>,
    d_pi: PiController,
    q_pi: PiController,
    velocity_pi: PiController,
    position_pi: PiController,
    active_error: Option<Error>,
    last_rotor: Option<RotorEstimate>,
    last_current_ref: Option<fluxkit_math::frame::Dq<Amps>>,
    status: MotorStatus,
}

impl MotorController<Svpwm> {
    /// Creates a new motor controller with explicit params and tuning.
    pub fn new(motor: MotorParams, inverter: InverterParams, config: CurrentLoopConfig) -> Self {
        Self::new_with_modulator(motor, inverter, config, Svpwm)
    }
}

impl<M> MotorController<M>
where
    M: Modulator,
{
    /// Creates a new motor controller with an explicit modulation strategy.
    pub fn new_with_modulator(
        motor: MotorParams,
        inverter: InverterParams,
        config: CurrentLoopConfig,
        modulator: M,
    ) -> Self {
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
        let velocity_limit = velocity_limit(config.max_velocity_target, motor.max_mech_speed);
        let current_limit = current_limit(config.max_iq_target, motor.max_phase_current);

        let mut controller = Self {
            motor,
            inverter,
            config,
            modulator,
            state: MotorState::Disabled,
            mode: ControlMode::Disabled,
            id_target: Amps::new(id_target),
            iq_target: Amps::ZERO,
            torque_target: NewtonMeters::ZERO,
            velocity_target: RadPerSec::ZERO,
            position_target: MechanicalAngle::new(0.0),
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
                out_min: -current_limit,
                out_max: current_limit,
            }),
            position_pi: PiController::new(PiConfig {
                kp: config.position_kp,
                ki: config.position_ki,
                out_min: -velocity_limit,
                out_max: velocity_limit,
            }),
            active_error: None,
            last_rotor: None,
            last_current_ref: None,
            status: MotorStatus {
                state: MotorState::Disabled,
                mode: ControlMode::Disabled,
                active_error: None,
                last_bus_voltage: Volts::ZERO,
                last_measured_idq: zero_current_dq(),
                last_commanded_vdq: zero_voltage_dq(),
                last_mechanical_angle: MechanicalAngle::new(0.0),
                last_mechanical_velocity: RadPerSec::ZERO,
                last_saturated: false,
            },
        };

        if !validate_controller_config(&controller.motor, &controller.inverter, &controller.config)
        {
            controller.latch_error(Error::ConfigurationInvalid);
        }

        controller.refresh_status();
        controller
    }

    /// Returns the configured modulation strategy.
    #[inline]
    pub const fn modulator(&self) -> &M {
        &self.modulator
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
    ///
    /// Changing modes resets the dynamic controller state so that PI integrators
    /// do not leak across fundamentally different control objectives.
    pub fn set_mode(&mut self, mode: ControlMode) {
        if self.mode != mode {
            self.reset_control_state();
        }
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

    /// Updates the torque target used by `Torque` mode.
    ///
    /// The actual torque-to-current mapping runs in `medium_tick()`.
    pub fn set_torque_target(&mut self, torque: NewtonMeters) {
        self.torque_target = torque;
    }

    /// Updates the mechanical velocity target used by `Velocity` mode.
    ///
    /// In `Position` mode this value is generated internally by the position loop
    /// during `medium_tick()`.
    pub fn set_velocity_target(&mut self, velocity: RadPerSec) {
        let limit = velocity_limit(self.config.max_velocity_target, self.motor.max_mech_speed);
        self.velocity_target = RadPerSec::new(clamp(velocity.get(), -limit, limit));
    }

    /// Updates the wrapped mechanical position target used by `Position` mode.
    pub fn set_position_target(&mut self, position: MechanicalAngle) {
        self.position_target = position.wrapped_pm_pi();
    }

    /// Updates the direct open-loop voltage target used by `OpenLoopVoltage` mode.
    ///
    /// This command is used directly by `fast_tick()` and bypasses the current PI.
    pub fn set_open_loop_voltage_target(&mut self, voltage: fluxkit_math::frame::Dq<Volts>) {
        self.open_loop_voltage_target = voltage;
    }

    /// Enables the controller if no error is latched.
    pub fn enable(&mut self) {
        if self.active_error.is_none() && self.state == MotorState::Disabled {
            self.state = MotorState::Ready;
        }
        self.refresh_status();
    }

    /// Disables the controller and clears dynamic controller state.
    pub fn disable(&mut self) {
        self.reset_control_state();
        self.state = MotorState::Disabled;
        self.refresh_status();
    }

    /// Clears a latched error when the static configuration is valid.
    pub fn clear_error(&mut self) {
        if validate_controller_config(&self.motor, &self.inverter, &self.config) {
            self.active_error = None;
            self.reset_control_state();
            self.state = MotorState::Disabled;
        }
        self.refresh_status();
    }

    /// Executes the synchronous high-rate current loop.
    ///
    /// The current-loop implementation is:
    ///
    /// 1. Validate `abc` current, bus voltage, rotor angle, and `dt`.
    /// 2. Transform measured currents into the rotating `dq` frame.
    /// 3. Compute `d`/`q` current error from the configured references.
    /// 4. Run one PI controller per axis.
    ///    Optional model-based feedforward is summed ahead of the PI output.
    ///    The current feedforward uses resistive, `L * d(i_ref)/dt`, cross-coupling,
    ///    and optional back-EMF terms.
    /// 5. Circularly limit the commanded `vd/vq` vector.
    /// 6. Modulate the limited voltage with the configured modulator.
    /// 7. Return bounded duty plus telemetry and error state.
    pub fn fast_tick(&mut self, input: FastLoopInput) -> FastLoopOutput {
        self.status.last_bus_voltage = input.bus_voltage;
        self.status.last_mechanical_angle = input.rotor.mechanical_angle;
        self.status.last_mechanical_velocity = input.rotor.mechanical_velocity;

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

        self.last_rotor = Some(input.rotor);
        let electrical_angle = input.rotor.electrical_angle.wrapped_pm_pi().get();
        let phase_currents = input.phase_currents.map(|current| current.get());
        let measured_alpha_beta = clarke(phase_currents);
        let measured_idq_f32 = park(measured_alpha_beta, electrical_angle);
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
                error: None,
            };
        }

        let voltage_limit = self.dynamic_voltage_limit(input.bus_voltage);
        match self.mode {
            ControlMode::Disabled => unreachable!("handled above"),
            ControlMode::Current
            | ControlMode::Torque
            | ControlMode::Velocity
            | ControlMode::Position => self.run_current_control(
                measured_idq_f32,
                measured_idq,
                electrical_angle,
                input.bus_voltage,
                input.rotor.mechanical_velocity,
                input.dt_seconds,
                voltage_limit,
            ),
            ControlMode::OpenLoopVoltage => self.run_open_loop_voltage(
                measured_idq,
                electrical_angle,
                input.bus_voltage,
                voltage_limit,
            ),
        }
    }

    /// Runs the medium-rate supervisory loop.
    ///
    /// Mode behavior:
    ///
    /// - `Disabled`, `Current`, `OpenLoopVoltage`
    ///   - no supervisory action
    /// - `Torque`
    ///   - maps torque target to `iq_target`
    /// - `Velocity`
    ///   - runs the velocity PI and updates `iq_target`
    /// - `Position`
    ///   - runs the position PI to update `velocity_target`
    ///   - then runs the velocity PI to update `iq_target`
    pub fn medium_tick(&mut self, dt_seconds: f32) {
        if self.active_error.is_some() || self.state == MotorState::Disabled {
            self.refresh_status();
            return;
        }

        match self.mode {
            ControlMode::Disabled | ControlMode::Current | ControlMode::OpenLoopVoltage => {}
            ControlMode::Torque => {
                if let Some(torque_constant) = self.motor.torque_constant_nm_per_amp {
                    self.set_iq_target(Amps::new(self.torque_target.get() / torque_constant));
                } else {
                    self.latch_error(Error::ConfigurationInvalid);
                }
            }
            ControlMode::Velocity | ControlMode::Position => {
                if let Some(rotor) = self.last_rotor {
                    if self.mode == ControlMode::Position {
                        let velocity_command = self.position_pi.update(
                            shortest_angle_delta(
                                rotor.mechanical_angle.wrapped_pm_pi().get(),
                                self.position_target.get(),
                            ),
                            dt_seconds,
                        );
                        self.set_velocity_target(RadPerSec::new(velocity_command));
                    }

                    let iq = self.velocity_pi.update(
                        self.velocity_target.get() - rotor.mechanical_velocity.get(),
                        dt_seconds,
                    );
                    self.set_iq_target(Amps::new(iq));
                }
            }
        }

        self.refresh_status();
    }

    /// Runs the slow-rate supervisory loop.
    ///
    /// This is currently a reserved hook only. Position and velocity control both
    /// run in `medium_tick()`.
    pub fn slow_tick(&mut self, _dt_seconds: f32) {
        self.refresh_status();
    }

    /// Returns the latest compact status snapshot.
    #[inline]
    pub const fn status(&self) -> MotorStatus {
        self.status
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
            self.current_feedforward(current_ref, current_ref_derivative, mechanical_velocity)
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
            error: None,
        }
    }

    fn current_feedforward(
        &self,
        current_ref: CurrentReference,
        current_ref_derivative: fluxkit_math::frame::Dq<f32>,
        mechanical_velocity: RadPerSec,
    ) -> fluxkit_math::frame::Dq<f32> {
        let omega_e = mechanical_velocity.get() * self.motor.pole_pairs as f32;
        let resistance = self.motor.phase_resistance_ohm.get();
        let ld = self.motor.d_inductance_h.get();
        let lq = self.motor.q_inductance_h.get();
        let flux_linkage = self
            .motor
            .flux_linkage_weber
            .map(|value| value.get())
            .unwrap_or(0.0);
        let id = current_ref.id.get();
        let iq = current_ref.iq.get();

        fluxkit_math::frame::Dq::new(
            resistance * id + ld * current_ref_derivative.d - omega_e * lq * iq,
            resistance * iq + lq * current_ref_derivative.q + omega_e * (ld * id + flux_linkage),
        )
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
            (current_ref.id.get() - last_ref.d.get()) / dt_seconds,
            (current_ref.iq.get() - last_ref.q.get()) / dt_seconds,
        )
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

    fn latch_error(&mut self, error: Error) {
        self.active_error = Some(error);
        self.state = MotorState::Faulted;
        self.reset_control_state();
    }

    fn reset_control_state(&mut self) {
        self.d_pi.reset();
        self.q_pi.reset();
        self.velocity_pi.reset();
        self.position_pi.reset();
        self.last_current_ref = None;
    }

    fn neutral_output(&self, error: Error) -> FastLoopOutput {
        FastLoopOutput {
            phase_duty: neutral_phase_duty(),
            measured_idq: self.status.last_measured_idq,
            commanded_vdq: zero_voltage_dq(),
            saturated: false,
            error: Some(error),
        }
    }

    fn refresh_status(&mut self) {
        self.status.state = self.state;
        self.status.mode = self.mode;
        self.status.active_error = self.active_error;
    }
}

#[inline]
fn current_limit(configured: Amps, motor_limit: Amps) -> f32 {
    configured.get().min(motor_limit.get())
}

#[inline]
fn velocity_limit(configured: RadPerSec, motor_limit: Option<RadPerSec>) -> f32 {
    motor_limit
        .map(|limit| configured.get().min(limit.get()))
        .unwrap_or(configured.get())
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
        error::Error,
        io::{FastLoopInput, RotorEstimate},
        mode::ControlMode,
        params::{InverterParams, MotorParams},
        state::MotorState,
    };
    use fluxkit_math::{
        ElectricalAngle, MechanicalAngle, SinePwm, Svpwm,
        frame::{Abc, Dq},
        units::{Amps, Duty, Henries, Hertz, NewtonMeters, Ohms, RadPerSec, Volts, Webers},
    };

    fn test_motor() -> MotorParams {
        MotorParams {
            pole_pairs: 7,
            phase_resistance_ohm: Ohms::new(0.12),
            d_inductance_h: Henries::new(0.000_03),
            q_inductance_h: Henries::new(0.000_03),
            flux_linkage_weber: Some(Webers::new(0.005)),
            max_phase_current: Amps::new(20.0),
            max_mech_speed: Some(RadPerSec::new(100.0)),
            torque_constant_nm_per_amp: Some(0.5),
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
            velocity_kp: 0.5,
            velocity_ki: 10.0,
            position_kp: 4.0,
            position_ki: 0.0,
            max_voltage_mag: Volts::new(12.0),
            id_ref_default: Amps::ZERO,
            max_id_target: Amps::new(10.0),
            max_iq_target: Amps::new(10.0),
            max_velocity_target: RadPerSec::new(100.0),
            enable_current_feedforward: true,
        }
    }

    fn test_input() -> FastLoopInput {
        FastLoopInput {
            phase_currents: Abc::new(Amps::ZERO, Amps::ZERO, Amps::ZERO),
            bus_voltage: Volts::new(24.0),
            rotor: RotorEstimate {
                electrical_angle: ElectricalAngle::new(0.0),
                mechanical_angle: MechanicalAngle::new(0.0),
                mechanical_velocity: RadPerSec::ZERO,
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
        assert_eq!(output.error, None);
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

        assert_eq!(output.error, Some(Error::InvalidBusVoltage));
        assert_eq!(output.phase_duty.a.get(), 0.5);
        assert_eq!(controller.status().state, MotorState::Faulted);
        assert_eq!(
            controller.status().active_error,
            Some(Error::InvalidBusVoltage)
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

        assert_eq!(output.error, Some(Error::InvalidRotorAngle));
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
        assert_eq!(output.error, None);
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

        controller.clear_error();
        assert_eq!(controller.status().state, MotorState::Disabled);

        controller.disable();
        assert_eq!(controller.status().state, MotorState::Disabled);
    }

    #[test]
    fn controller_can_use_alternate_modulator() {
        let mut controller = MotorController::new_with_modulator(
            test_motor(),
            test_inverter(),
            test_config(),
            SinePwm,
        );
        controller.set_mode(ControlMode::Current);
        controller.set_iq_target(Amps::new(3.0));
        controller.enable();

        let output = controller.fast_tick(test_input());

        assert_eq!(output.error, None);
        for duty in [
            output.phase_duty.a,
            output.phase_duty.b,
            output.phase_duty.c,
        ] {
            assert!((0.0..=1.0).contains(&duty.get()));
        }
    }

    #[test]
    fn torque_mode_updates_iq_target_in_medium_tick() {
        let mut controller = MotorController::new(test_motor(), test_inverter(), test_config());
        controller.set_mode(ControlMode::Torque);
        controller.set_torque_target(NewtonMeters::new(1.5));
        controller.enable();

        controller.medium_tick(0.001);

        assert_eq!(controller.iq_target, Amps::new(3.0));
    }

    #[test]
    fn velocity_mode_generates_positive_q_current_target() {
        let mut controller = MotorController::new(test_motor(), test_inverter(), test_config());
        controller.set_mode(ControlMode::Velocity);
        controller.set_velocity_target(RadPerSec::new(20.0));
        controller.enable();
        controller.fast_tick(test_input());

        controller.medium_tick(0.001);

        assert!(controller.iq_target.get() > 0.0);
    }

    #[test]
    fn position_mode_runs_position_and_velocity_loops_in_medium_tick() {
        let mut controller = MotorController::new(test_motor(), test_inverter(), test_config());
        controller.set_mode(ControlMode::Position);
        controller.set_position_target(MechanicalAngle::new(1.0));
        controller.enable();
        controller.fast_tick(test_input());

        controller.medium_tick(0.01);

        assert!(controller.velocity_target.get() > 0.0);
        assert!(controller.iq_target.get() > 0.0);
    }

    #[test]
    fn open_loop_voltage_mode_bypasses_current_pi() {
        let mut controller = MotorController::new(test_motor(), test_inverter(), test_config());
        controller.set_mode(ControlMode::OpenLoopVoltage);
        controller.set_open_loop_voltage_target(Dq::new(Volts::new(0.0), Volts::new(3.0)));
        controller.enable();

        let output = controller.fast_tick(test_input());

        assert!(output.commanded_vdq.q.get() > 0.0);
        assert!(
            output.phase_duty.a.get() != 0.5
                || output.phase_duty.b.get() != 0.5
                || output.phase_duty.c.get() != 0.5
        );
    }

    #[test]
    fn current_feedforward_adds_back_emf_compensation() {
        let mut controller = MotorController::new(test_motor(), test_inverter(), test_config());
        controller.set_mode(ControlMode::Current);
        controller.enable();

        let mut input = test_input();
        input.rotor.mechanical_velocity = RadPerSec::new(25.0);
        let output = controller.fast_tick(input);

        assert!(output.commanded_vdq.q.get() > 0.0);
    }

    #[test]
    fn current_feedforward_adds_reference_derivative_term_on_step() {
        let mut config = test_config();
        config.kp_d = 0.0;
        config.ki_d = 0.0;
        config.kp_q = 0.0;
        config.ki_q = 0.0;

        let mut controller = MotorController::new(test_motor(), test_inverter(), config);
        controller.set_mode(ControlMode::Current);
        controller.enable();

        let baseline = controller.fast_tick(test_input());
        assert_eq!(baseline.commanded_vdq.q, Volts::ZERO);

        controller.set_iq_target(Amps::new(3.0));
        let step_response = controller.fast_tick(test_input());
        let steady_response = controller.fast_tick(test_input());

        assert!(step_response.commanded_vdq.q.get() > steady_response.commanded_vdq.q.get());
    }

    #[test]
    fn modulation_limit_tracks_selected_modulator() {
        let mut config = test_config();
        config.max_voltage_mag = Volts::new(20.0);

        let mut sine_controller =
            MotorController::new_with_modulator(test_motor(), test_inverter(), config, SinePwm);
        sine_controller.set_mode(ControlMode::OpenLoopVoltage);
        sine_controller.set_open_loop_voltage_target(Dq::new(Volts::ZERO, Volts::new(13.0)));
        sine_controller.enable();

        let mut svpwm_controller =
            MotorController::new_with_modulator(test_motor(), test_inverter(), config, Svpwm);
        svpwm_controller.set_mode(ControlMode::OpenLoopVoltage);
        svpwm_controller.set_open_loop_voltage_target(Dq::new(Volts::ZERO, Volts::new(13.0)));
        svpwm_controller.enable();

        let sine_output = sine_controller.fast_tick(test_input());
        let svpwm_output = svpwm_controller.fast_tick(test_input());

        assert!(sine_output.commanded_vdq.q.get() <= 12.0);
        assert!(svpwm_output.commanded_vdq.q.get() > sine_output.commanded_vdq.q.get());
    }

    #[test]
    fn feedforward_can_be_disabled() {
        let mut config = test_config();
        config.enable_current_feedforward = false;

        let mut controller = MotorController::new(test_motor(), test_inverter(), config);
        controller.set_mode(ControlMode::Current);
        controller.enable();

        let mut input = test_input();
        input.rotor.mechanical_velocity = RadPerSec::new(25.0);
        let output = controller.fast_tick(input);

        assert_eq!(output.commanded_vdq.q, Volts::ZERO);
    }
}
