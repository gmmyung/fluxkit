use super::*;

impl<PWM, CURRENT, BUS, ROTOR, OUTPUT, TEMP, MOD, RotorEst, OutputEst>
    InnerActuatorCalibrationRuntime<
        PWM,
        CURRENT,
        BUS,
        ROTOR,
        OUTPUT,
        TEMP,
        MOD,
        RotorEst,
        OutputEst,
    >
where
    PWM: PhasePwm,
    CURRENT: CurrentSampler,
    BUS: BusVoltageSensor,
    ROTOR: RotorSensor,
    OUTPUT: OutputSensor,
    TEMP: TemperatureSensor,
    MOD: Modulator,
    RotorEst: MechanicalMotionEstimator,
    OutputEst: MechanicalMotionEstimator,
{
    fn publish_tick_result(
        &self,
        shared: &Mutex<RefCell<SharedStatus<ActuatorCalibrationStatus>>>,
        result: Result<
            Option<()>,
            ActuatorCalibrationRuntimeError<
                PWM::Error,
                CURRENT::Error,
                BUS::Error,
                ROTOR::Error,
                OUTPUT::Error,
                TEMP::Error,
            >,
        >,
    ) -> Result<
        (),
        ActuatorCalibrationRuntimeError<
            PWM::Error,
            CURRENT::Error,
            BUS::Error,
            ROTOR::Error,
            OUTPUT::Error,
            TEMP::Error,
        >,
    > {
        match result {
            Ok(_) => {
                self.publish_status(shared, false);
                Ok(())
            }
            Err(error) => {
                self.publish_status(shared, true);
                Err(error)
            }
        }
    }

    fn phase(&self) -> Option<ActuatorCalibrationPhase> {
        self.active_routine
            .as_ref()
            .map(ActuatorCalibrationPhase::from)
            .or_else(|| self.next_phase())
    }

    pub fn tick(
        &mut self,
        shared: &Mutex<RefCell<SharedStatus<ActuatorCalibrationStatus>>>,
    ) -> Result<
        (),
        ActuatorCalibrationRuntimeError<
            PWM::Error,
            CURRENT::Error,
            BUS::Error,
            ROTOR::Error,
            OUTPUT::Error,
            TEMP::Error,
        >,
    > {
        let result = self.tick_inner();
        self.publish_tick_result(shared, result)
    }

    fn tick_inner(
        &mut self,
    ) -> Result<
        Option<()>,
        ActuatorCalibrationRuntimeError<
            PWM::Error,
            CURRENT::Error,
            BUS::Error,
            ROTOR::Error,
            OUTPUT::Error,
            TEMP::Error,
        >,
    > {
        let partial = self.partial_calibration();
        self.apply_live_calibration(&partial);

        if self.activate_next_routine_if_needed()? {
            return Ok(Some(()));
        }

        let mut routine = self
            .active_routine
            .take()
            .expect("active routine must exist");
        let delta = self.tick_active_routine(&mut routine)?;
        self.finish_routine_step(routine, delta)
    }

    fn publish_status(
        &self,
        shared: &Mutex<RefCell<SharedStatus<ActuatorCalibrationStatus>>>,
        fault_latched: bool,
    ) {
        write_status(shared, |status| {
            *status = ActuatorCalibrationStatus {
                active: true,
                phase: self.phase(),
                result: self.resolve_calibration().ok(),
                fault_latched,
            };
        });
    }

    fn activate_next_routine_if_needed(
        &mut self,
    ) -> Result<
        bool,
        ActuatorCalibrationRuntimeError<
            PWM::Error,
            CURRENT::Error,
            BUS::Error,
            ROTOR::Error,
            OUTPUT::Error,
            TEMP::Error,
        >,
    > {
        if self.active_routine.is_some() {
            return Ok(false);
        }

        self.active_routine = self
            .build_next_routine()
            .map_err(ActuatorCalibrationRuntimeError::Calibration)?;
        if self.active_routine.is_some() {
            return Ok(false);
        }

        self.resolve_calibration()
            .map_err(ActuatorCalibrationRuntimeError::Calibration)?;
        Ok(true)
    }

    fn finish_routine_step(
        &mut self,
        routine: ActuatorCalibrationRoutine,
        delta: Option<PartialActuatorCalibration>,
    ) -> Result<
        Option<()>,
        ActuatorCalibrationRuntimeError<
            PWM::Error,
            CURRENT::Error,
            BUS::Error,
            ROTOR::Error,
            OUTPUT::Error,
            TEMP::Error,
        >,
    > {
        if let Some(delta) = delta {
            self.merge_partial(delta);
            if self.activate_next_routine_if_needed()? {
                return Ok(Some(()));
            }
        } else {
            self.active_routine = Some(routine);
        }

        Ok(None)
    }

    fn next_phase(&self) -> Option<ActuatorCalibrationPhase> {
        if self.gear_ratio.is_none() {
            return Some(ActuatorCalibrationPhase::GearRatio);
        }
        if self.positive_coulomb_torque.is_none()
            || self.negative_coulomb_torque.is_none()
            || self.positive_viscous_coefficient.is_none()
            || self.negative_viscous_coefficient.is_none()
        {
            return Some(ActuatorCalibrationPhase::Friction);
        }
        if self.positive_breakaway_torque.is_none() || self.negative_breakaway_torque.is_none() {
            return Some(ActuatorCalibrationPhase::Breakaway);
        }
        if self.zero_velocity_blend_band.is_none() {
            return Some(ActuatorCalibrationPhase::BlendBand);
        }
        None
    }

    fn tick_active_routine(
        &mut self,
        routine: &mut ActuatorCalibrationRoutine,
    ) -> Result<
        Option<PartialActuatorCalibration>,
        ActuatorCalibrationRuntimeError<
            PWM::Error,
            CURRENT::Error,
            BUS::Error,
            ROTOR::Error,
            OUTPUT::Error,
            TEMP::Error,
        >,
    > {
        match routine {
            ActuatorCalibrationRoutine::GearRatio(calibrator) => self.tick_gear_ratio(calibrator),
            ActuatorCalibrationRoutine::Friction(calibrator) => self.tick_friction(calibrator),
            ActuatorCalibrationRoutine::Breakaway(calibrator) => self.tick_breakaway(calibrator),
            ActuatorCalibrationRoutine::BlendBand(calibrator) => self.tick_blend_band(calibrator),
        }
    }

    fn build_next_routine(&self) -> Result<Option<ActuatorCalibrationRoutine>, CalibrationError> {
        let limits = self.limits;

        if self.gear_ratio.is_none() {
            let mut cfg =
                fluxkit_core::ActuatorGearRatioCalibrationConfig::default_for_travel_ratio();
            cfg.velocity_target =
                clamp_abs_rad_per_sec(cfg.velocity_target, limits.max_velocity_target);
            cfg.timeout_seconds = cfg.timeout_seconds.min(limits.timeout_seconds);
            return ActuatorGearRatioCalibrator::new(cfg)
                .map(ActuatorCalibrationRoutine::GearRatio)
                .map(Some);
        }

        if self.positive_coulomb_torque.is_none()
            || self.negative_coulomb_torque.is_none()
            || self.positive_viscous_coefficient.is_none()
            || self.negative_viscous_coefficient.is_none()
        {
            let mut cfg =
                fluxkit_core::ActuatorFrictionCalibrationConfig::default_for_velocity_sweep();
            cfg.velocity_points = [
                clamp_abs_rad_per_sec(cfg.velocity_points[0], limits.max_velocity_target),
                clamp_abs_rad_per_sec(cfg.velocity_points[1], limits.max_velocity_target),
                clamp_abs_rad_per_sec(cfg.velocity_points[2], limits.max_velocity_target),
            ];
            cfg.timeout_seconds = cfg.timeout_seconds.min(limits.timeout_seconds);
            return ActuatorFrictionCalibrator::new(cfg)
                .map(ActuatorCalibrationRoutine::Friction)
                .map(Some);
        }

        if self.positive_breakaway_torque.is_none() || self.negative_breakaway_torque.is_none() {
            let mut cfg =
                fluxkit_core::ActuatorBreakawayCalibrationConfig::default_for_torque_ramp();
            cfg.positive_coulomb_torque = self
                .positive_coulomb_torque
                .expect("friction resolved before breakaway");
            cfg.negative_coulomb_torque = self
                .negative_coulomb_torque
                .expect("friction resolved before breakaway");
            cfg.max_torque = min_torque(cfg.max_torque, limits.max_torque_target);
            cfg.timeout_seconds = cfg.timeout_seconds.min(limits.timeout_seconds);
            return fluxkit_core::ActuatorBreakawayCalibrator::new(cfg)
                .map(ActuatorCalibrationRoutine::Breakaway)
                .map(Some);
        }

        if self.zero_velocity_blend_band.is_none() {
            let mut cfg =
                fluxkit_core::ActuatorBlendBandCalibrationConfig::default_for_release_ramp();
            cfg.max_torque = min_torque(cfg.max_torque, limits.max_torque_target);
            cfg.timeout_seconds = cfg.timeout_seconds.min(limits.timeout_seconds);
            return fluxkit_core::ActuatorBlendBandCalibrator::new(cfg)
                .map(ActuatorCalibrationRoutine::BlendBand)
                .map(Some);
        }

        Ok(None)
    }

    fn merge_partial(&mut self, delta: PartialActuatorCalibration) {
        if self.gear_ratio.is_none() {
            if let Some(value) = delta.gear_ratio {
                self.gear_ratio = Some(value);
            }
        }
        if self.positive_breakaway_torque.is_none() {
            if let Some(value) = delta.friction.positive_breakaway_torque {
                self.positive_breakaway_torque = Some(value);
            }
        }
        if self.negative_breakaway_torque.is_none() {
            if let Some(value) = delta.friction.negative_breakaway_torque {
                self.negative_breakaway_torque = Some(value);
            }
        }
        if self.positive_coulomb_torque.is_none() {
            if let Some(value) = delta.friction.positive_coulomb_torque {
                self.positive_coulomb_torque = Some(value);
            }
        }
        if self.negative_coulomb_torque.is_none() {
            if let Some(value) = delta.friction.negative_coulomb_torque {
                self.negative_coulomb_torque = Some(value);
            }
        }
        if self.positive_viscous_coefficient.is_none() {
            if let Some(value) = delta.friction.positive_viscous_coefficient {
                self.positive_viscous_coefficient = Some(value);
            }
        }
        if self.negative_viscous_coefficient.is_none() {
            if let Some(value) = delta.friction.negative_viscous_coefficient {
                self.negative_viscous_coefficient = Some(value);
            }
        }
        if self.zero_velocity_blend_band.is_none() {
            if let Some(value) = delta.friction.zero_velocity_blend_band {
                self.zero_velocity_blend_band = Some(value);
            }
        }
    }

    pub(crate) fn partial_calibration(&self) -> PartialActuatorCalibration {
        PartialActuatorCalibration {
            gear_ratio: self.gear_ratio,
            friction: fluxkit_core::ActuatorFrictionCalibration {
                positive_breakaway_torque: self.positive_breakaway_torque,
                negative_breakaway_torque: self.negative_breakaway_torque,
                positive_coulomb_torque: self.positive_coulomb_torque,
                negative_coulomb_torque: self.negative_coulomb_torque,
                positive_viscous_coefficient: self.positive_viscous_coefficient,
                negative_viscous_coefficient: self.negative_viscous_coefficient,
                zero_velocity_blend_band: self.zero_velocity_blend_band,
            },
        }
    }

    fn resolve_calibration(&self) -> Result<ActuatorCalibrationResult, CalibrationError> {
        Ok(ActuatorCalibrationResult {
            gear_ratio: self
                .gear_ratio
                .ok_or(CalibrationError::InvalidConfiguration)?,
            positive_breakaway_torque: self
                .positive_breakaway_torque
                .ok_or(CalibrationError::InvalidConfiguration)?,
            negative_breakaway_torque: self
                .negative_breakaway_torque
                .ok_or(CalibrationError::InvalidConfiguration)?,
            positive_coulomb_torque: self
                .positive_coulomb_torque
                .ok_or(CalibrationError::InvalidConfiguration)?,
            negative_coulomb_torque: self
                .negative_coulomb_torque
                .ok_or(CalibrationError::InvalidConfiguration)?,
            positive_viscous_coefficient: self
                .positive_viscous_coefficient
                .ok_or(CalibrationError::InvalidConfiguration)?,
            negative_viscous_coefficient: self
                .negative_viscous_coefficient
                .ok_or(CalibrationError::InvalidConfiguration)?,
            zero_velocity_blend_band: self
                .zero_velocity_blend_band
                .ok_or(CalibrationError::InvalidConfiguration)?,
        })
    }

    fn tick_gear_ratio(
        &mut self,
        calibrator: &mut ActuatorGearRatioCalibrator,
    ) -> Result<
        Option<PartialActuatorCalibration>,
        ActuatorCalibrationRuntimeError<
            PWM::Error,
            CURRENT::Error,
            BUS::Error,
            ROTOR::Error,
            OUTPUT::Error,
            TEMP::Error,
        >,
    > {
        self.tick_calibrator(
            calibrator,
            false,
            |calibrator, status, dt| {
                calibrator.tick(ActuatorGearRatioCalibrationInput {
                    rotor_mechanical_angle: status.last_rotor_mechanical_angle,
                    output_mechanical_angle: status.last_output_mechanical_angle,
                    output_velocity: status.last_output_mechanical_velocity,
                    dt_seconds: dt,
                })
            },
            |motor_system, command| {
                motor_system.apply_command_immediate(crate::MotorCommand::Velocity(
                    command.velocity_target,
                ));
            },
        )
    }

    fn tick_friction(
        &mut self,
        calibrator: &mut ActuatorFrictionCalibrator,
    ) -> Result<
        Option<PartialActuatorCalibration>,
        ActuatorCalibrationRuntimeError<
            PWM::Error,
            CURRENT::Error,
            BUS::Error,
            ROTOR::Error,
            OUTPUT::Error,
            TEMP::Error,
        >,
    > {
        self.tick_calibrator(
            calibrator,
            true,
            |calibrator, status, dt| {
                calibrator.tick(ActuatorFrictionCalibrationInput {
                    output_velocity: status.last_output_mechanical_velocity,
                    output_torque_command: status
                        .last_actuator_compensation
                        .total_output_torque_command,
                    dt_seconds: dt,
                })
            },
            |motor_system, command| {
                motor_system.apply_command_immediate(crate::MotorCommand::Velocity(
                    command.velocity_target,
                ));
            },
        )
    }

    fn tick_breakaway(
        &mut self,
        calibrator: &mut ActuatorBreakawayCalibrator,
    ) -> Result<
        Option<PartialActuatorCalibration>,
        ActuatorCalibrationRuntimeError<
            PWM::Error,
            CURRENT::Error,
            BUS::Error,
            ROTOR::Error,
            OUTPUT::Error,
            TEMP::Error,
        >,
    > {
        self.tick_calibrator(
            calibrator,
            true,
            |calibrator, status, dt| {
                calibrator.tick(ActuatorBreakawayCalibrationInput {
                    output_velocity: status.last_output_mechanical_velocity,
                    output_torque_command: status
                        .last_actuator_compensation
                        .total_output_torque_command,
                    dt_seconds: dt,
                })
            },
            |motor_system, command| {
                motor_system
                    .apply_command_immediate(crate::MotorCommand::Torque(command.torque_target));
            },
        )
    }

    fn tick_blend_band(
        &mut self,
        calibrator: &mut ActuatorBlendBandCalibrator,
    ) -> Result<
        Option<PartialActuatorCalibration>,
        ActuatorCalibrationRuntimeError<
            PWM::Error,
            CURRENT::Error,
            BUS::Error,
            ROTOR::Error,
            OUTPUT::Error,
            TEMP::Error,
        >,
    > {
        self.tick_calibrator(
            calibrator,
            true,
            |calibrator, status, dt| {
                calibrator.tick(ActuatorBlendBandCalibrationInput {
                    output_velocity: status.last_output_mechanical_velocity,
                    output_torque_command: status
                        .last_actuator_compensation
                        .total_output_torque_command,
                    dt_seconds: dt,
                })
            },
            |motor_system, command| {
                motor_system
                    .apply_command_immediate(crate::MotorCommand::Torque(command.torque_target));
            },
        )
    }

    fn tick_calibrator<R, Cal, Command, Build, Apply>(
        &mut self,
        calibrator: &mut Cal,
        require_friction_disabled: bool,
        build_command: Build,
        apply_command: Apply,
    ) -> Result<
        Option<PartialActuatorCalibration>,
        ActuatorCalibrationRuntimeError<
            PWM::Error,
            CURRENT::Error,
            BUS::Error,
            ROTOR::Error,
            OUTPUT::Error,
            TEMP::Error,
        >,
    >
    where
        Cal: RoutineState<R>,
        R: Into<PartialActuatorCalibration>,
        Build: FnOnce(&mut Cal, MotorStatus, f32) -> Command,
        Apply: FnOnce(
            &mut MotorRuntime<PWM, CURRENT, BUS, ROTOR, OUTPUT, TEMP, MOD, RotorEst, OutputEst>,
            Command,
        ),
    {
        if let Some(result) = self.finish_routine_state(calibrator)? {
            return Ok(Some(result));
        }

        self.prepare_motor(require_friction_disabled)?;
        let status = self.motor_system.controller_status();
        let command = build_command(calibrator, status, self.dt_seconds);
        apply_command(&mut self.motor_system, command);
        let _ = self
            .motor_system
            .ticker_internal()
            .tick()
            .map_err(ActuatorCalibrationRuntimeError::Motor)?;

        self.finish_routine_state(calibrator)
    }

    fn prepare_motor(
        &mut self,
        require_friction_disabled: bool,
    ) -> Result<
        (),
        ActuatorCalibrationRuntimeError<
            PWM::Error,
            CURRENT::Error,
            BUS::Error,
            ROTOR::Error,
            OUTPUT::Error,
            TEMP::Error,
        >,
    > {
        if require_friction_disabled && self.motor_system.friction_compensation_enabled() {
            self.motor_system
                .set_armed_immediate(false)
                .map_err(ActuatorCalibrationRuntimeError::Motor)?;
            return Err(ActuatorCalibrationRuntimeError::Calibration(
                CalibrationError::InvalidConfiguration,
            ));
        }

        self.motor_system
            .set_armed_immediate(true)
            .map_err(ActuatorCalibrationRuntimeError::Motor)?;
        Ok(())
    }

    fn finish_routine_state<R, Cal>(
        &mut self,
        calibrator: &Cal,
    ) -> Result<
        Option<PartialActuatorCalibration>,
        ActuatorCalibrationRuntimeError<
            PWM::Error,
            CURRENT::Error,
            BUS::Error,
            ROTOR::Error,
            OUTPUT::Error,
            TEMP::Error,
        >,
    >
    where
        Cal: RoutineState<R>,
        R: Into<PartialActuatorCalibration>,
    {
        if let Some(result) = calibrator.result() {
            self.motor_system
                .set_armed_immediate(false)
                .map_err(ActuatorCalibrationRuntimeError::Motor)?;
            let delta = result.into();
            self.apply_live_calibration(&delta);
            return Ok(Some(delta));
        }
        if let Some(error) = calibrator.error() {
            self.motor_system
                .set_armed_immediate(false)
                .map_err(ActuatorCalibrationRuntimeError::Motor)?;
            return Err(ActuatorCalibrationRuntimeError::Calibration(error));
        }
        Ok(None)
    }

    pub(crate) fn apply_live_calibration(&mut self, calibration: &PartialActuatorCalibration) {
        self.motor_system.apply_actuator_calibration(calibration);
    }
}

impl<'a, PWM, CURRENT, BUS, ROTOR, OUTPUT, TEMP, MOD, RotorEst, OutputEst>
    ActuatorCalibrationTicker<'a, PWM, CURRENT, BUS, ROTOR, OUTPUT, TEMP, MOD, RotorEst, OutputEst>
where
    PWM: PhasePwm,
    CURRENT: CurrentSampler,
    BUS: BusVoltageSensor,
    ROTOR: RotorSensor,
    OUTPUT: OutputSensor,
    TEMP: TemperatureSensor,
    MOD: Modulator,
    RotorEst: MechanicalMotionEstimator,
    OutputEst: MechanicalMotionEstimator,
{
    /// Advances the request-driven actuator calibration campaign by one fixed-period step.
    pub fn tick(
        &self,
    ) -> Result<
        (),
        ActuatorCalibrationRuntimeError<
            PWM::Error,
            CURRENT::Error,
            BUS::Error,
            ROTOR::Error,
            OUTPUT::Error,
            TEMP::Error,
        >,
    > {
        let mut inner = take_active_inner(&self.inner, || read_status(&self.shared).active, |active| {
            if active {
                ActuatorCalibrationRuntimeError::Busy
            } else {
                ActuatorCalibrationRuntimeError::Inactive
            }
        })?;
        let result = inner.tick(self.shared);
        critical_section::with(|cs| {
            *self.inner.borrow(cs).borrow_mut() = Some(inner);
        });
        result
    }
}
