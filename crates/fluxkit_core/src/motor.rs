//! Main motor controller implementation.

use fluxkit_math::angle::{mechanical_to_electrical, shortest_angle_delta};
use fluxkit_math::{
    MechanicalAngle, Modulator, PiConfig, PiController, Svpwm, clamp, clarke, inverse_park,
    limit_norm_dq, park,
    units::{Amps, NewtonMeters, RadPerSec, Volts},
};

use crate::{
    actuator::{ActuatorCompensationTelemetry, ActuatorParams},
    config::CurrentLoopConfig,
    control::current::CurrentReference,
    error::Error,
    io::{FastLoopInput, FastLoopOutput, RotorEstimate},
    mode::ControlMode,
    params::{InverterParams, MotorParams},
    schedule::TickSchedule,
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
/// - `tick()`
///   - runs one deterministic fast-cycle entrypoint
///   - executes `fast_tick()`
///   - optionally executes due `medium_tick()` and `slow_tick()` afterward
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
///         RA[Unwrapped mechanical angle] --> PP
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
    actuator: ActuatorParams,
    config: CurrentLoopConfig,
    modulator: M,
    state: MotorState,
    mode: ControlMode,
    id_target: Amps,
    iq_target: Amps,
    output_torque_target: NewtonMeters,
    output_velocity_target: RadPerSec,
    output_position_target: MechanicalAngle,
    open_loop_voltage_target: fluxkit_math::frame::Dq<Volts>,
    d_pi: PiController,
    q_pi: PiController,
    velocity_pi: PiController,
    position_pi: PiController,
    active_error: Option<Error>,
    last_rotor: Option<RotorEstimate>,
    last_wrapped_mechanical_angle: Option<MechanicalAngle>,
    last_wrapped_output_angle: Option<MechanicalAngle>,
    last_current_ref: Option<fluxkit_math::frame::Dq<Amps>>,
    status: MotorStatus,
}

impl MotorController<Svpwm> {
    /// Creates a new motor controller with explicit params and tuning.
    pub fn new(
        motor: MotorParams,
        inverter: InverterParams,
        actuator: ActuatorParams,
        config: CurrentLoopConfig,
    ) -> Self {
        Self::new_with_modulator(motor, inverter, actuator, config, Svpwm)
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
        actuator: ActuatorParams,
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
        let velocity_limit = output_velocity_limit(config.max_velocity_target, actuator);
        let current_limit = current_limit(config.max_iq_target, motor.max_phase_current);
        let output_torque_limit = output_torque_limit(current_limit, motor, actuator);

        let mut controller = Self {
            motor,
            inverter,
            actuator,
            config,
            modulator,
            state: MotorState::Disabled,
            mode: ControlMode::Disabled,
            id_target: Amps::new(id_target),
            iq_target: Amps::ZERO,
            output_torque_target: NewtonMeters::ZERO,
            output_velocity_target: RadPerSec::ZERO,
            output_position_target: MechanicalAngle::new(0.0),
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
                last_measured_idq: zero_current_dq(),
                last_commanded_vdq: zero_voltage_dq(),
                last_rotor_mechanical_angle: MechanicalAngle::new(0.0),
                last_unwrapped_rotor_mechanical_angle: MechanicalAngle::new(0.0),
                last_rotor_mechanical_velocity: RadPerSec::ZERO,
                last_output_mechanical_angle: MechanicalAngle::new(0.0),
                last_unwrapped_output_mechanical_angle: MechanicalAngle::new(0.0),
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

    /// Returns the static actuator parameters.
    #[inline]
    pub const fn actuator_params(&self) -> &ActuatorParams {
        &self.actuator
    }

    /// Returns mutable access to the static actuator parameters.
    #[inline]
    pub fn actuator_params_mut(&mut self) -> &mut ActuatorParams {
        &mut self.actuator
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
    /// This target is expressed at the actuator output axis.
    /// The actual output-torque-to-current mapping runs in `medium_tick()`.
    pub fn set_torque_target(&mut self, torque: NewtonMeters) {
        self.output_torque_target = self.clamp_output_torque(torque);
    }

    /// Updates the output-axis mechanical velocity target used by `Velocity` mode.
    ///
    /// In `Position` mode this value is generated internally by the position loop
    /// during `medium_tick()`.
    pub fn set_velocity_target(&mut self, velocity: RadPerSec) {
        let limit = output_velocity_limit(self.config.max_velocity_target, self.actuator);
        self.output_velocity_target = RadPerSec::new(clamp(velocity.get(), -limit, limit));
    }

    /// Updates the output-axis mechanical position target used by `Position` mode.
    ///
    /// This target is not wrapped, so it can represent multi-turn positioning.
    pub fn set_position_target(&mut self, position: MechanicalAngle) {
        self.output_position_target = position;
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
        if validate_controller_config(&self.motor, &self.inverter, &self.actuator, &self.config) {
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
                electrical_angle.get(),
                input.bus_voltage,
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
    ///
    /// This is the preferred public entrypoint when a runtime has fast, medium,
    /// and slow timers. It preserves single-owner sequencing by:
    ///
    /// 1. running `fast_tick()` with the current measurements
    /// 2. optionally running `medium_tick()` if due
    /// 3. optionally running `slow_tick()` if due
    ///
    /// Medium- and slow-rate updates therefore affect the next fast cycle
    /// rather than racing the current one.
    pub fn tick(&mut self, input: FastLoopInput, schedule: TickSchedule) -> FastLoopOutput {
        let output = self.fast_tick(input);

        if let Some(dt_seconds) = schedule.medium_dt_seconds {
            self.medium_tick(dt_seconds);
        }

        if let Some(dt_seconds) = schedule.slow_dt_seconds {
            self.slow_tick(dt_seconds);
        }

        output
    }

    /// Runs the medium-rate supervisory loop.
    ///
    /// Mode behavior:
    ///
    /// - `Disabled`, `Current`, `OpenLoopVoltage`
    ///   - no supervisory action
    /// - `Torque`
    ///   - applies actuator compensation to the output torque target
    ///   - maps the compensated output torque to `iq_target`
    /// - `Velocity`
    ///   - runs the velocity PI and updates output torque
    ///   - applies actuator compensation
    ///   - maps the compensated output torque to `iq_target`
    /// - `Position`
    ///   - runs the position PI to update `output_velocity_target`
    ///   - then runs the velocity PI to update output torque
    ///   - applies actuator compensation
    ///   - maps the compensated output torque to `iq_target`
    ///
    /// Prefer [`Self::tick`] when integrating with timer interrupts so the
    /// controller remains single-owned and loop ordering stays explicit.
    pub fn medium_tick(&mut self, dt_seconds: f32) {
        if self.active_error.is_some() || self.state == MotorState::Disabled {
            self.refresh_status();
            return;
        }

        match self.mode {
            ControlMode::Disabled | ControlMode::Current | ControlMode::OpenLoopVoltage => {}
            ControlMode::Torque => {
                let total_output_torque =
                    self.total_output_torque_command(self.output_torque_target);
                if let Some(iq_target) = self.output_torque_to_iq(total_output_torque) {
                    self.set_iq_target(iq_target);
                } else {
                    self.latch_error(Error::ConfigurationInvalid);
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

                    if let Some(iq_target) = self.output_torque_to_iq(total_output_torque) {
                        self.set_iq_target(iq_target);
                    } else {
                        self.latch_error(Error::ConfigurationInvalid);
                    }
                }
            }
        }

        self.refresh_status();
    }

    /// Runs the slow-rate supervisory loop.
    ///
    /// This is currently a reserved hook only. Position and velocity control both
    /// run in `medium_tick()`.
    ///
    /// Prefer [`Self::tick`] when integrating with timer interrupts so the
    /// controller remains single-owned and loop ordering stays explicit.
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

    #[inline]
    fn electrical_angle_from_mechanical(
        &self,
        mechanical_angle: MechanicalAngle,
    ) -> fluxkit_math::ElectricalAngle {
        let base = mechanical_to_electrical(mechanical_angle, self.motor.pole_pairs as u32);
        fluxkit_math::ElectricalAngle::new(base.get() + self.motor.electrical_angle_offset.get())
            .wrapped_pm_pi()
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

    fn unwrap_rotor_angle(&mut self, wrapped_angle: MechanicalAngle) -> MechanicalAngle {
        let unwrapped = match self.last_wrapped_mechanical_angle {
            Some(previous_wrapped) => {
                let delta = shortest_angle_delta(previous_wrapped.get(), wrapped_angle.get());
                MechanicalAngle::new(
                    self.status.last_unwrapped_rotor_mechanical_angle.get() + delta,
                )
            }
            None => wrapped_angle,
        };

        self.last_wrapped_mechanical_angle = Some(wrapped_angle);
        unwrapped
    }

    fn unwrap_output_angle(&mut self, wrapped_angle: MechanicalAngle) -> MechanicalAngle {
        let unwrapped = match self.last_wrapped_output_angle {
            Some(previous_wrapped) => {
                let delta = shortest_angle_delta(previous_wrapped.get(), wrapped_angle.get());
                MechanicalAngle::new(
                    self.status.last_unwrapped_output_mechanical_angle.get() + delta,
                )
            }
            None => wrapped_angle,
        };

        self.last_wrapped_output_angle = Some(wrapped_angle);
        unwrapped
    }

    fn clamp_output_torque(&self, torque: NewtonMeters) -> NewtonMeters {
        let limit = self
            .actuator
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
            ControlMode::Velocity | ControlMode::Position => {
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
        // Near zero speed we keep breakaway direction tied to the command-side
        // hint. As motion becomes established, we transition toward measured
        // velocity so Coulomb and viscous drag compensation track the real
        // actuator state.
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

    fn output_torque_to_iq(&self, output_torque: NewtonMeters) -> Option<Amps> {
        let torque_constant = motor_torque_constant(self.motor)?;
        let motor_torque = output_torque.get() / self.actuator.gear_ratio;
        Some(Amps::new(motor_torque / torque_constant))
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
        self.output_torque_target = NewtonMeters::ZERO;
        self.status.last_actuator_compensation = ActuatorCompensationTelemetry::zero();
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
fn output_velocity_limit(configured: RadPerSec, actuator: ActuatorParams) -> f32 {
    actuator
        .max_output_velocity
        .map(|limit| configured.get().min(limit.get()))
        .unwrap_or(configured.get())
}

#[derive(Clone, Copy)]
struct FrictionCompensationBreakdown {
    breakaway: NewtonMeters,
    coulomb: NewtonMeters,
    viscous: NewtonMeters,
    total: NewtonMeters,
}

impl FrictionCompensationBreakdown {
    const fn zero() -> Self {
        Self {
            breakaway: NewtonMeters::ZERO,
            coulomb: NewtonMeters::ZERO,
            viscous: NewtonMeters::ZERO,
            total: NewtonMeters::ZERO,
        }
    }
}

#[inline]
fn output_torque_limit(max_iq_target: f32, motor: MotorParams, actuator: ActuatorParams) -> f32 {
    let configured_limit = motor_torque_constant(motor)
        .map(|torque_constant| max_iq_target * torque_constant * actuator.gear_ratio)
        .unwrap_or(0.0);

    actuator
        .max_output_torque
        .map(|limit| configured_limit.min(limit.get()))
        .unwrap_or(configured_limit)
}

#[inline]
fn motor_torque_constant(motor: MotorParams) -> Option<f32> {
    let flux_linkage = motor.flux_linkage_weber?;
    Some(1.5 * motor.pole_pairs as f32 * flux_linkage.get())
}

#[inline]
fn friction_direction_and_motion_weight(
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
        actuator::{
            ActuatorCompensationConfig, ActuatorEstimate, ActuatorParams, FrictionCompensation,
        },
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
            electrical_angle_offset: ElectricalAngle::new(0.0),
            max_phase_current: Amps::new(20.0),
            max_mech_speed: Some(RadPerSec::new(100.0)),
        }
    }

    fn test_actuator() -> ActuatorParams {
        ActuatorParams {
            gear_ratio: 5.0,
            max_output_velocity: Some(RadPerSec::new(20.0)),
            max_output_torque: Some(NewtonMeters::new(20.0)),
            compensation: ActuatorCompensationConfig::disabled(),
        }
    }

    fn test_inverter() -> InverterParams {
        InverterParams {
            pwm_frequency_hz: Hertz::new(20_000.0),
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
            max_current_ref_derivative_amps_per_sec: 10_000.0,
            enable_current_feedforward: true,
        }
    }

    fn test_input() -> FastLoopInput {
        FastLoopInput {
            phase_currents: Abc::new(Amps::ZERO, Amps::ZERO, Amps::ZERO),
            bus_voltage: Volts::new(24.0),
            rotor: RotorEstimate {
                mechanical_angle: MechanicalAngle::new(0.0),
                mechanical_velocity: RadPerSec::ZERO,
            },
            actuator: ActuatorEstimate {
                output_angle: MechanicalAngle::new(0.0),
                output_velocity: RadPerSec::ZERO,
            },
            dt_seconds: 1.0 / 20_000.0,
        }
    }

    #[test]
    fn zero_input_zero_target_returns_neutral_output() {
        let mut controller = MotorController::new(
            test_motor(),
            test_inverter(),
            test_actuator(),
            test_config(),
        );
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
        let mut controller = MotorController::new(
            test_motor(),
            test_inverter(),
            test_actuator(),
            test_config(),
        );
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
        let mut controller = MotorController::new(
            test_motor(),
            test_inverter(),
            test_actuator(),
            test_config(),
        );
        controller.set_mode(ControlMode::Current);
        controller.enable();

        let mut input = test_input();
        input.rotor.mechanical_angle = MechanicalAngle::new(f32::NAN);
        let output = controller.fast_tick(input);

        assert_eq!(output.error, Some(Error::InvalidRotorAngle));
        assert_eq!(controller.status().state, MotorState::Faulted);
    }

    #[test]
    fn positive_iq_target_produces_positive_vq() {
        let mut controller = MotorController::new(
            test_motor(),
            test_inverter(),
            test_actuator(),
            test_config(),
        );
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

        let mut controller =
            MotorController::new(test_motor(), test_inverter(), test_actuator(), config);
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

        let mut controller =
            MotorController::new(test_motor(), inverter, test_actuator(), test_config());
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
        let mut controller = MotorController::new(
            test_motor(),
            test_inverter(),
            test_actuator(),
            test_config(),
        );

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
            test_actuator(),
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
        let mut controller = MotorController::new(
            test_motor(),
            test_inverter(),
            test_actuator(),
            test_config(),
        );
        controller.set_mode(ControlMode::Torque);
        controller.set_torque_target(NewtonMeters::new(1.5));
        controller.enable();

        controller.medium_tick(0.001);

        assert!((controller.iq_target.get() - 5.714_286).abs() < 1.0e-6);
    }

    #[test]
    fn torque_mode_adds_bounded_friction_compensation() {
        let mut actuator = test_actuator();
        actuator.compensation = ActuatorCompensationConfig {
            friction: FrictionCompensation {
                enabled: true,
                positive_breakaway_torque: NewtonMeters::new(0.2),
                negative_breakaway_torque: NewtonMeters::new(0.3),
                positive_coulomb_torque: NewtonMeters::new(0.4),
                negative_coulomb_torque: NewtonMeters::new(0.5),
                positive_viscous_coefficient: 0.02,
                negative_viscous_coefficient: 0.03,
                zero_velocity_blend_band: RadPerSec::new(1.0),
            },
            max_total_torque: NewtonMeters::new(0.5),
        };

        let mut controller =
            MotorController::new(test_motor(), test_inverter(), actuator, test_config());
        controller.set_mode(ControlMode::Torque);
        controller.set_torque_target(NewtonMeters::new(0.2));
        controller.enable();

        controller.medium_tick(0.001);

        assert!(
            controller
                .status()
                .last_actuator_compensation
                .friction_torque
                .get()
                > 0.0
        );
        assert!(
            controller
                .status()
                .last_actuator_compensation
                .total_compensation_torque
                .get()
                <= 0.5
        );
        assert!(
            controller
                .status()
                .last_actuator_compensation
                .total_output_torque_command
                .get()
                > 0.2
        );
    }

    #[test]
    fn breakaway_compensation_only_fills_missing_margin() {
        let mut actuator = test_actuator();
        actuator.compensation = ActuatorCompensationConfig {
            friction: FrictionCompensation {
                enabled: true,
                positive_breakaway_torque: NewtonMeters::new(0.2),
                negative_breakaway_torque: NewtonMeters::new(0.2),
                positive_coulomb_torque: NewtonMeters::new(0.05),
                negative_coulomb_torque: NewtonMeters::new(0.05),
                positive_viscous_coefficient: 0.0,
                negative_viscous_coefficient: 0.0,
                zero_velocity_blend_band: RadPerSec::new(1.0),
            },
            max_total_torque: NewtonMeters::new(1.0),
        };

        let mut controller =
            MotorController::new(test_motor(), test_inverter(), actuator, test_config());
        controller.set_mode(ControlMode::Torque);
        controller.set_torque_target(NewtonMeters::new(0.25));
        controller.enable();

        controller.medium_tick(0.001);

        let telemetry = controller.status().last_actuator_compensation;
        assert!(
            telemetry.breakaway_torque.get().abs() < 1.0e-6,
            "expected breakaway term to be capped away once command exceeds breakaway, got {}",
            telemetry.breakaway_torque.get()
        );
        assert!(
            telemetry.coulomb_torque.get() > 0.0,
            "expected coulomb term to remain active, got {}",
            telemetry.coulomb_torque.get()
        );
    }

    #[test]
    fn torque_mode_friction_compensation_tracks_measured_velocity_for_viscous_drag() {
        let mut actuator = test_actuator();
        actuator.compensation = ActuatorCompensationConfig {
            friction: FrictionCompensation {
                enabled: true,
                positive_breakaway_torque: NewtonMeters::new(0.1),
                negative_breakaway_torque: NewtonMeters::new(0.1),
                positive_coulomb_torque: NewtonMeters::new(0.05),
                negative_coulomb_torque: NewtonMeters::new(0.05),
                positive_viscous_coefficient: 0.1,
                negative_viscous_coefficient: 0.1,
                zero_velocity_blend_band: RadPerSec::new(1.0),
            },
            max_total_torque: NewtonMeters::new(1.0),
        };

        let mut controller =
            MotorController::new(test_motor(), test_inverter(), actuator, test_config());
        controller.set_mode(ControlMode::Torque);
        controller.set_torque_target(NewtonMeters::new(0.2));
        controller.enable();

        controller.medium_tick(0.001);
        let near_zero = controller
            .status()
            .last_actuator_compensation
            .friction_torque
            .get();

        controller.status.last_output_mechanical_velocity = RadPerSec::new(4.0);
        controller.medium_tick(0.001);
        let moving = controller
            .status()
            .last_actuator_compensation
            .friction_torque
            .get();

        assert!(
            moving > near_zero,
            "expected viscous compensation to grow with measured velocity: near_zero={near_zero}, moving={moving}",
        );
    }

    #[test]
    fn velocity_mode_generates_positive_q_current_target() {
        let mut controller = MotorController::new(
            test_motor(),
            test_inverter(),
            test_actuator(),
            test_config(),
        );
        controller.set_mode(ControlMode::Velocity);
        controller.set_velocity_target(RadPerSec::new(20.0));
        controller.enable();
        controller.fast_tick(test_input());

        controller.medium_tick(0.001);

        assert!(controller.iq_target.get() > 0.0);
    }

    #[test]
    fn position_mode_runs_position_and_velocity_loops_in_medium_tick() {
        let mut controller = MotorController::new(
            test_motor(),
            test_inverter(),
            test_actuator(),
            test_config(),
        );
        controller.set_mode(ControlMode::Position);
        controller.set_position_target(MechanicalAngle::new(1.0));
        controller.enable();
        controller.fast_tick(test_input());

        controller.medium_tick(0.01);

        assert!(controller.output_velocity_target.get() > 0.0);
        assert!(controller.iq_target.get() > 0.0);
    }

    #[test]
    fn wrapped_encoder_angle_is_unwrapped_for_multi_turn_positioning() {
        let mut controller = MotorController::new(
            test_motor(),
            test_inverter(),
            test_actuator(),
            test_config(),
        );
        controller.set_mode(ControlMode::Position);
        controller.enable();

        let mut input = test_input();
        input.actuator.output_angle = MechanicalAngle::new(6.0);
        controller.fast_tick(input);

        let mut wrapped_input = test_input();
        wrapped_input.actuator.output_angle = MechanicalAngle::new(0.2);
        controller.fast_tick(wrapped_input);

        assert!(
            controller
                .status()
                .last_unwrapped_output_mechanical_angle
                .get()
                > 6.2
        );
    }

    #[test]
    fn open_loop_voltage_mode_bypasses_current_pi() {
        let mut controller = MotorController::new(
            test_motor(),
            test_inverter(),
            test_actuator(),
            test_config(),
        );
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
        let mut controller = MotorController::new(
            test_motor(),
            test_inverter(),
            test_actuator(),
            test_config(),
        );
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

        let mut controller =
            MotorController::new(test_motor(), test_inverter(), test_actuator(), config);
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
    fn current_reference_derivative_feedforward_is_clamped() {
        let mut config = test_config();
        config.kp_d = 0.0;
        config.ki_d = 0.0;
        config.kp_q = 0.0;
        config.ki_q = 0.0;
        config.max_current_ref_derivative_amps_per_sec = 1_000.0;

        let motor = test_motor();
        let mut controller = MotorController::new(motor, test_inverter(), test_actuator(), config);
        controller.set_mode(ControlMode::Current);
        controller.enable();
        controller.fast_tick(test_input());

        controller.set_iq_target(Amps::new(10.0));
        let output = controller.fast_tick(test_input());

        let expected_q = motor.phase_resistance_ohm.get() * 10.0
            + motor.q_inductance_h.get() * config.max_current_ref_derivative_amps_per_sec;
        assert!((output.commanded_vdq.q.get() - expected_q).abs() < 1.0e-5);
    }

    #[test]
    fn modulation_limit_tracks_selected_modulator() {
        let mut config = test_config();
        config.max_voltage_mag = Volts::new(20.0);

        let mut sine_controller = MotorController::new_with_modulator(
            test_motor(),
            test_inverter(),
            test_actuator(),
            config,
            SinePwm,
        );
        sine_controller.set_mode(ControlMode::OpenLoopVoltage);
        sine_controller.set_open_loop_voltage_target(Dq::new(Volts::ZERO, Volts::new(13.0)));
        sine_controller.enable();

        let mut svpwm_controller = MotorController::new_with_modulator(
            test_motor(),
            test_inverter(),
            test_actuator(),
            config,
            Svpwm,
        );
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

        let mut controller =
            MotorController::new(test_motor(), test_inverter(), test_actuator(), config);
        controller.set_mode(ControlMode::Current);
        controller.enable();

        let mut input = test_input();
        input.rotor.mechanical_velocity = RadPerSec::new(25.0);
        let output = controller.fast_tick(input);

        assert_eq!(output.commanded_vdq.q, Volts::ZERO);
    }
}
