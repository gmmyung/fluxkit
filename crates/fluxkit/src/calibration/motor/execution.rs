use super::*;

impl<PWM, CURRENT, BUS, ROTOR, TEMP, MOD, RotorEst>
    InnerMotorCalibrationRuntime<PWM, CURRENT, BUS, ROTOR, TEMP, MOD, RotorEst>
where
    PWM: PhasePwm,
    CURRENT: CurrentSampler,
    BUS: BusVoltageSensor,
    ROTOR: RotorSensor,
    TEMP: TemperatureSensor,
    MOD: Modulator,
    RotorEst: MechanicalMotionEstimator,
{
    /// Advances the request-driven motor calibration campaign by one fixed-period step.
    pub fn tick(
        &mut self,
        shared: &Mutex<RefCell<SharedStatus<MotorCalibrationStatus>>>,
    ) -> Result<
        (),
        MotorCalibrationRuntimeError<
            PWM::Error,
            CURRENT::Error,
            BUS::Error,
            ROTOR::Error,
            TEMP::Error,
        >,
    > {
        let result = self.tick_inner();
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

    fn phase(&self) -> Option<MotorCalibrationPhase> {
        self.active_routine
            .as_ref()
            .map(MotorCalibrationPhase::from)
            .or_else(|| self.next_phase())
    }

    fn tick_inner(
        &mut self,
    ) -> Result<
        Option<()>,
        MotorCalibrationRuntimeError<
            PWM::Error,
            CURRENT::Error,
            BUS::Error,
            ROTOR::Error,
            TEMP::Error,
        >,
    > {
        if self.active_routine.is_none() {
            self.active_routine = self
                .build_next_routine()
                .map_err(MotorCalibrationRuntimeError::Calibration)?;
            if self.active_routine.is_none() {
                self.resolve_calibration()
                    .map_err(MotorCalibrationRuntimeError::Calibration)?;
                return Ok(Some(()));
            }
        }

        let mut routine = self
            .active_routine
            .take()
            .expect("active routine must exist");
        if let Some(delta) = self.tick_active_routine(&mut routine, self.dt_seconds)? {
            self.merge_partial(delta);
            if self
                .build_next_routine()
                .map_err(MotorCalibrationRuntimeError::Calibration)?
                .is_none()
            {
                self.resolve_calibration()
                    .map_err(MotorCalibrationRuntimeError::Calibration)?;
                return Ok(Some(()));
            }
        } else {
            self.active_routine = Some(routine);
        }

        Ok(None)
    }

    fn publish_status(
        &self,
        shared: &Mutex<RefCell<SharedStatus<MotorCalibrationStatus>>>,
        fault_latched: bool,
    ) {
        write_status(shared, |status| {
            *status = MotorCalibrationStatus {
                active: true,
                phase: self.phase(),
                pole_pairs: self.pole_pairs,
                electrical_direction: self.electrical_direction,
                electrical_angle_offset: self.electrical_angle_offset,
                phase_resistance_ohm_ref: self.phase_resistance_ohm_ref,
                phase_inductance_h: self.phase_inductance_h,
                result: self.resolve_calibration().ok(),
                fault_latched,
            };
        });
    }

    fn set_neutral(
        &mut self,
    ) -> Result<
        (),
        MotorCalibrationRuntimeError<
            PWM::Error,
            CURRENT::Error,
            BUS::Error,
            ROTOR::Error,
            TEMP::Error,
        >,
    > {
        self.pwm
            .set_neutral()
            .map_err(MotorCalibrationRuntimeError::Pwm)
    }

    fn tick_active_routine(
        &mut self,
        routine: &mut MotorCalibrationRoutine,
        dt_seconds: f32,
    ) -> Result<
        Option<PartialMotorCalibration>,
        MotorCalibrationRuntimeError<
            PWM::Error,
            CURRENT::Error,
            BUS::Error,
            ROTOR::Error,
            TEMP::Error,
        >,
    > {
        match routine {
            MotorCalibrationRoutine::PolePairsAndOffset(calibrator) => {
                self.tick_pole_pairs_and_offset(calibrator, dt_seconds)
            }
            MotorCalibrationRoutine::PhaseResistance(calibrator) => {
                self.tick_phase_resistance(calibrator, dt_seconds)
            }
            MotorCalibrationRoutine::PhaseInductance(calibrator) => {
                self.tick_phase_inductance(calibrator, dt_seconds)
            }
            MotorCalibrationRoutine::FluxLinkage(calibrator) => {
                self.tick_flux_linkage(calibrator, dt_seconds)
            }
        }
    }

    fn build_next_routine(&self) -> Result<Option<MotorCalibrationRoutine>, CalibrationError> {
        let limits = self.limits;

        if self.pole_pairs.is_none()
            || self.electrical_direction.is_none()
            || self.electrical_angle_offset.is_none()
        {
            let cfg = self.config.pole_pairs_and_offset.to_core(limits);
            return PolePairsAndOffsetCalibrator::new(cfg)
                .map(MotorCalibrationRoutine::PolePairsAndOffset)
                .map(Some);
        }

        if self.phase_resistance_ohm_ref.is_none() {
            let cfg = self.config.phase_resistance.to_core(limits);
            return PhaseResistanceCalibrator::new(cfg)
                .map(MotorCalibrationRoutine::PhaseResistance)
                .map(Some);
        }

        if self.phase_inductance_h.is_none() {
            let cfg = self.config.phase_inductance.to_core(
                limits,
                self.phase_resistance_ohm_ref
                    .expect("phase resistance resolved before inductance"),
            );
            return PhaseInductanceCalibrator::new(cfg)
                .map(MotorCalibrationRoutine::PhaseInductance)
                .map(Some);
        }

        if self.flux_linkage_weber.is_none() {
            let cfg = self.config.flux_linkage.to_core(
                limits,
                self.phase_resistance_ohm_ref
                    .expect("phase resistance resolved before flux linkage"),
                self.phase_inductance_h
                    .expect("phase inductance resolved before flux linkage"),
                self.pole_pairs
                    .expect("electrical mapping resolved before flux linkage"),
                self.electrical_direction
                    .expect("electrical mapping resolved before flux linkage"),
                self.electrical_angle_offset
                    .expect("electrical mapping resolved before flux linkage"),
            );
            return FluxLinkageCalibrator::new(cfg)
                .map(MotorCalibrationRoutine::FluxLinkage)
                .map(Some);
        }

        Ok(None)
    }

    fn next_phase(&self) -> Option<MotorCalibrationPhase> {
        if self.pole_pairs.is_none()
            || self.electrical_direction.is_none()
            || self.electrical_angle_offset.is_none()
        {
            return Some(MotorCalibrationPhase::PolePairsAndOffset);
        }
        if self.phase_resistance_ohm_ref.is_none() {
            return Some(MotorCalibrationPhase::PhaseResistance);
        }
        if self.phase_inductance_h.is_none() {
            return Some(MotorCalibrationPhase::PhaseInductance);
        }
        if self.flux_linkage_weber.is_none() {
            return Some(MotorCalibrationPhase::FluxLinkage);
        }
        None
    }

    fn merge_partial(&mut self, delta: PartialMotorCalibration) {
        if self.pole_pairs.is_none() {
            if let Some(value) = delta.pole_pairs {
                self.pole_pairs = Some(value);
            }
        }
        if self.electrical_direction.is_none() {
            if let Some(value) = delta.electrical_direction {
                self.electrical_direction = Some(value);
            }
        }
        if self.electrical_angle_offset.is_none() {
            if let Some(value) = delta.electrical_angle_offset {
                self.electrical_angle_offset = Some(value);
            }
        }
        if self.phase_resistance_ohm_ref.is_none() {
            if let Some(value) = delta.phase_resistance_ohm_ref {
                self.phase_resistance_ohm_ref = Some(value);
            }
        }
        if self.phase_inductance_h.is_none() {
            if let Some(value) = delta.phase_inductance_h {
                self.phase_inductance_h = Some(value);
            }
        }
        if self.flux_linkage_weber.is_none() {
            if let Some(value) = delta.flux_linkage_weber {
                self.flux_linkage_weber = Some(value);
            }
        }
    }

    fn resolve_calibration(&self) -> Result<MotorCalibrationResult, CalibrationError> {
        Ok(MotorCalibrationResult {
            pole_pairs: self
                .pole_pairs
                .ok_or(CalibrationError::InvalidConfiguration)?,
            electrical_direction: self
                .electrical_direction
                .ok_or(CalibrationError::InvalidConfiguration)?,
            electrical_angle_offset: self
                .electrical_angle_offset
                .ok_or(CalibrationError::InvalidConfiguration)?,
            phase_resistance_ohm_ref: self
                .phase_resistance_ohm_ref
                .ok_or(CalibrationError::InvalidConfiguration)?,
            phase_inductance_h: self
                .phase_inductance_h
                .ok_or(CalibrationError::InvalidConfiguration)?,
            flux_linkage_weber: self
                .flux_linkage_weber
                .ok_or(CalibrationError::InvalidConfiguration)?,
        })
    }

    fn tick_pole_pairs_and_offset(
        &mut self,
        calibrator: &mut PolePairsAndOffsetCalibrator,
        dt_seconds: f32,
    ) -> Result<
        Option<PartialMotorCalibration>,
        MotorCalibrationRuntimeError<
            PWM::Error,
            CURRENT::Error,
            BUS::Error,
            ROTOR::Error,
            TEMP::Error,
        >,
    > {
        self.tick_alpha_beta_routine(calibrator, dt_seconds, false, |calibrator, rotor, _, dt| {
            calibrator.tick(PolePairsAndOffsetCalibrationInput {
                mechanical_angle: rotor.wrapped(),
                mechanical_velocity: rotor.velocity(),
                dt_seconds: dt,
            })
        })
    }

    fn tick_phase_resistance(
        &mut self,
        calibrator: &mut PhaseResistanceCalibrator,
        dt_seconds: f32,
    ) -> Result<
        Option<PartialMotorCalibration>,
        MotorCalibrationRuntimeError<
            PWM::Error,
            CURRENT::Error,
            BUS::Error,
            ROTOR::Error,
            TEMP::Error,
        >,
    > {
        let winding_temperature_c = self
            .temp
            .sample_temperature_c()
            .map_err(MotorCalibrationRuntimeError::Temp)?;
        self.tick_alpha_beta_routine(
            calibrator,
            dt_seconds,
            true,
            |calibrator, rotor, current, dt| {
                calibrator.tick(PhaseResistanceCalibrationInput {
                    phase_currents: current.expect("phase current required").currents,
                    mechanical_velocity: rotor.velocity(),
                    winding_temperature_c,
                    dt_seconds: dt,
                })
            },
        )
    }

    fn tick_phase_inductance(
        &mut self,
        calibrator: &mut PhaseInductanceCalibrator,
        dt_seconds: f32,
    ) -> Result<
        Option<PartialMotorCalibration>,
        MotorCalibrationRuntimeError<
            PWM::Error,
            CURRENT::Error,
            BUS::Error,
            ROTOR::Error,
            TEMP::Error,
        >,
    > {
        let pole_pairs =
            self.pole_pairs
                .expect("electrical mapping resolved before inductance") as u32;
        let electrical_direction = self
            .electrical_direction
            .expect("electrical mapping resolved before inductance");
        let electrical_angle_offset = self
            .electrical_angle_offset
            .expect("electrical mapping resolved before inductance");
        self.tick_alpha_beta_routine(
            calibrator,
            dt_seconds,
            true,
            |calibrator, rotor, current, dt| {
                let electrical_angle = fluxkit_math::ElectricalAngle::new(
                    fluxkit_math::angle::mechanical_to_electrical_with_direction(
                        rotor.unwrapped(),
                        pole_pairs,
                        electrical_direction,
                    )
                    .get()
                        + electrical_angle_offset.get(),
                );
                calibrator.tick(PhaseInductanceCalibrationInput {
                    phase_currents: current.expect("phase current required").currents,
                    electrical_angle,
                    mechanical_velocity: rotor.velocity(),
                    dt_seconds: dt,
                })
            },
        )
    }

    fn tick_flux_linkage(
        &mut self,
        calibrator: &mut FluxLinkageCalibrator,
        dt_seconds: f32,
    ) -> Result<
        Option<PartialMotorCalibration>,
        MotorCalibrationRuntimeError<
            PWM::Error,
            CURRENT::Error,
            BUS::Error,
            ROTOR::Error,
            TEMP::Error,
        >,
    > {
        self.tick_alpha_beta_routine(
            calibrator,
            dt_seconds,
            true,
            |calibrator, rotor, current, dt| {
                calibrator.tick(FluxLinkageCalibrationInput {
                    phase_currents: current.expect("phase current required").currents,
                    mechanical_angle: rotor.unwrapped(),
                    mechanical_velocity: rotor.velocity(),
                    dt_seconds: dt,
                })
            },
        )
    }

    fn tick_alpha_beta_routine<R, Cal, Build>(
        &mut self,
        calibrator: &mut Cal,
        dt_seconds: f32,
        needs_current: bool,
        build_command: Build,
    ) -> Result<
        Option<R>,
        MotorCalibrationRuntimeError<
            PWM::Error,
            CURRENT::Error,
            BUS::Error,
            ROTOR::Error,
            TEMP::Error,
        >,
    >
    where
        Cal: RoutineState<R>,
        Build: FnOnce(
            &mut Cal,
            MechanicalMotionEstimate,
            Option<PhaseCurrentSample>,
            f32,
        ) -> AlphaBeta<Volts>,
    {
        if let Some(result) = self.preflight(calibrator)? {
            return Ok(result);
        }

        let bus_voltage = self
            .bus
            .sample_bus_voltage()
            .map_err(MotorCalibrationRuntimeError::Bus)?;
        let rotor = self
            .rotor
            .read_rotor()
            .map_err(MotorCalibrationRuntimeError::Rotor)?;
        let rotor_motion = self.rotor_estimator.update(
            MechanicalMotionSample {
                wrapped_value: rotor.mechanical_angle,
                measured_rate: rotor.mechanical_velocity,
            },
            dt_seconds,
        );
        let current = if needs_current {
            Some(self.sample_valid_current()?)
        } else {
            None
        };

        let command = build_command(calibrator, rotor_motion, current, dt_seconds);
        self.apply_alpha_beta_command(command, bus_voltage)?;
        self.postflight(calibrator)
    }

    fn sample_valid_current(
        &mut self,
    ) -> Result<
        PhaseCurrentSample,
        MotorCalibrationRuntimeError<
            PWM::Error,
            CURRENT::Error,
            BUS::Error,
            ROTOR::Error,
            TEMP::Error,
        >,
    > {
        let current = self
            .current
            .sample_phase_currents()
            .map_err(MotorCalibrationRuntimeError::Current)?;
        if current.validity == CurrentSampleValidity::Invalid {
            self.set_neutral()?;
            return Err(MotorCalibrationRuntimeError::InvalidCurrentSample);
        }
        Ok(current)
    }

    fn apply_alpha_beta_command(
        &mut self,
        command: AlphaBeta<Volts>,
        bus_voltage: Volts,
    ) -> Result<
        (),
        MotorCalibrationRuntimeError<
            PWM::Error,
            CURRENT::Error,
            BUS::Error,
            ROTOR::Error,
            TEMP::Error,
        >,
    > {
        if command == AlphaBeta::new(Volts::ZERO, Volts::ZERO) {
            return self.set_neutral();
        }

        let modulation = self
            .modulator
            .modulate(command.map(|volts| volts.get()), bus_voltage);
        self.pwm
            .set_phase_duty(modulation.duty)
            .map_err(MotorCalibrationRuntimeError::Pwm)
    }

    fn preflight<R, Cal>(
        &mut self,
        calibrator: &Cal,
    ) -> Result<
        Option<Option<R>>,
        MotorCalibrationRuntimeError<
            PWM::Error,
            CURRENT::Error,
            BUS::Error,
            ROTOR::Error,
            TEMP::Error,
        >,
    >
    where
        Cal: RoutineState<R>,
    {
        if let Some(result) = calibrator.result() {
            self.set_neutral()?;
            return Ok(Some(Some(result)));
        }
        if let Some(error) = calibrator.error() {
            self.set_neutral()?;
            return Err(MotorCalibrationRuntimeError::Calibration(error));
        }
        Ok(None)
    }

    fn postflight<R, Cal>(
        &mut self,
        calibrator: &Cal,
    ) -> Result<
        Option<R>,
        MotorCalibrationRuntimeError<
            PWM::Error,
            CURRENT::Error,
            BUS::Error,
            ROTOR::Error,
            TEMP::Error,
        >,
    >
    where
        Cal: RoutineState<R>,
    {
        if let Some(result) = calibrator.result() {
            self.set_neutral()?;
            Ok(Some(result))
        } else if let Some(error) = calibrator.error() {
            self.set_neutral()?;
            Err(MotorCalibrationRuntimeError::Calibration(error))
        } else {
            Ok(None)
        }
    }
}

impl<'a, PWM, CURRENT, BUS, ROTOR, TEMP, MOD, RotorEst>
    MotorCalibrationTicker<'a, PWM, CURRENT, BUS, ROTOR, TEMP, MOD, RotorEst>
where
    PWM: PhasePwm,
    CURRENT: CurrentSampler,
    BUS: BusVoltageSensor,
    ROTOR: RotorSensor,
    TEMP: TemperatureSensor,
    MOD: Modulator,
    RotorEst: MechanicalMotionEstimator,
{
    /// Advances the request-driven motor calibration campaign by one fixed-period step.
    pub fn tick(
        &self,
    ) -> Result<
        (),
        MotorCalibrationRuntimeError<
            PWM::Error,
            CURRENT::Error,
            BUS::Error,
            ROTOR::Error,
            TEMP::Error,
        >,
    > {
        let mut inner = take_active_inner(self.inner, || read_status(self.shared).active, |active| {
            if active {
                MotorCalibrationRuntimeError::Busy
            } else {
                MotorCalibrationRuntimeError::Inactive
            }
        })?;
        let result = inner.tick(self.shared);
        critical_section::with(|cs| {
            *self.inner.borrow(cs).borrow_mut() = Some(inner);
        });
        result
    }
}
