use super::*;

impl<PWM, CURRENT, BUS, ROTOR, TEMP, MOD, RotorEst>
    MotorCalibrationRuntime<PWM, CURRENT, BUS, ROTOR, TEMP, MOD, RotorEst>
where
    PWM: PhasePwm,
    CURRENT: CurrentSampler,
    BUS: BusVoltageSensor,
    ROTOR: RotorSensor,
    TEMP: TemperatureSensor,
    MOD: Modulator,
    RotorEst: MechanicalMotionEstimator,
{
    /// Creates a new request-driven motor-calibration runtime.
    pub fn new(
        pwm: PWM,
        current: CURRENT,
        bus: BUS,
        rotor: ROTOR,
        temp: TEMP,
        modulator: MOD,
        rotor_estimator: RotorEst,
        request: MotorCalibrationRequest,
        limits: MotorCalibrationLimits,
        dt_seconds: f32,
    ) -> Result<Self, CalibrationError> {
        Self::new_with_config(
            pwm,
            current,
            bus,
            rotor,
            temp,
            modulator,
            rotor_estimator,
            request,
            limits,
            MotorCalibrationConfig::default(),
            dt_seconds,
        )
    }

    /// Creates a new request-driven motor-calibration runtime with explicit
    /// routine tuning.
    pub fn new_with_config(
        pwm: PWM,
        current: CURRENT,
        bus: BUS,
        rotor: ROTOR,
        temp: TEMP,
        modulator: MOD,
        rotor_estimator: RotorEst,
        request: MotorCalibrationRequest,
        limits: MotorCalibrationLimits,
        config: MotorCalibrationConfig,
        dt_seconds: f32,
    ) -> Result<Self, CalibrationError> {
        Self::from_parts_with_config(
            MotorCalibrationParts {
                pwm,
                current,
                bus,
                rotor,
                temp,
                modulator,
                rotor_estimator,
            },
            request,
            limits,
            config,
            dt_seconds,
        )
    }

    /// Creates a new request-driven motor-calibration runtime from owned parts
    /// with explicit routine tuning.
    pub fn from_parts_with_config(
        parts: MotorCalibrationParts<PWM, CURRENT, BUS, ROTOR, TEMP, MOD, RotorEst>,
        request: MotorCalibrationRequest,
        limits: MotorCalibrationLimits,
        config: MotorCalibrationConfig,
        dt_seconds: f32,
    ) -> Result<Self, CalibrationError> {
        if !validate_limits(limits)
            || !validate_request(request)
            || !validate_dt_seconds(dt_seconds)
            || !validate_config(config, limits)
        {
            return Err(CalibrationError::InvalidConfiguration);
        }

        Ok(Self {
            inner: Mutex::new(RefCell::new(Some(InnerMotorCalibrationRuntime {
                pwm: parts.pwm,
                current: parts.current,
                bus: parts.bus,
                rotor: parts.rotor,
                temp: parts.temp,
                modulator: parts.modulator,
                rotor_estimator: parts.rotor_estimator,
                config,
                limits,
                dt_seconds,
                pole_pairs: request.pole_pairs,
                electrical_direction: request.electrical_direction,
                electrical_angle_offset: request.electrical_angle_offset,
                phase_resistance_ohm_ref: request.phase_resistance_ohm_ref,
                phase_inductance_h: request.phase_inductance_h,
                flux_linkage_weber: request.flux_linkage_weber,
                active_routine: None,
            }))),
            shared: Mutex::new(RefCell::new(SharedStatus {
                status: MotorCalibrationStatus {
                    active: true,
                    phase: next_phase_for_request(request),
                    pole_pairs: request.pole_pairs,
                    electrical_direction: request.electrical_direction,
                    electrical_angle_offset: request.electrical_angle_offset,
                    phase_resistance_ohm_ref: request.phase_resistance_ohm_ref,
                    phase_inductance_h: request.phase_inductance_h,
                    result: None,
                    fault_latched: false,
                },
            })),
            split_taken: Cell::new(false),
        })
    }

    /// Creates a new request-driven motor-calibration runtime from owned parts.
    pub fn from_parts(
        parts: MotorCalibrationParts<PWM, CURRENT, BUS, ROTOR, TEMP, MOD, RotorEst>,
        request: MotorCalibrationRequest,
        limits: MotorCalibrationLimits,
        dt_seconds: f32,
    ) -> Result<Self, CalibrationError> {
        Self::from_parts_with_config(
            MotorCalibrationParts {
                pwm: parts.pwm,
                current: parts.current,
                bus: parts.bus,
                rotor: parts.rotor,
                temp: parts.temp,
                modulator: parts.modulator,
                rotor_estimator: parts.rotor_estimator,
            },
            request,
            limits,
            MotorCalibrationConfig::default(),
            dt_seconds,
        )
    }

    /// Attempts to take ownership of the active calibration parts for reuse in another phase.
    pub fn try_into_parts(
        &self,
    ) -> Option<MotorCalibrationParts<PWM, CURRENT, BUS, ROTOR, TEMP, MOD, RotorEst>> {
        let inner = critical_section::with(|cs| self.inner.borrow(cs).borrow_mut().take())?;
        write_status(&self.shared, |status| status.active = false);
        Some(MotorCalibrationParts {
            pwm: inner.pwm,
            current: inner.current,
            bus: inner.bus,
            rotor: inner.rotor,
            temp: inner.temp,
            modulator: inner.modulator,
            rotor_estimator: inner.rotor_estimator,
        })
    }

    /// Splits this calibration runtime into its unique handle and ticker.
    ///
    /// This can be called at most once for a given calibration owner.
    #[inline]
    pub fn split(
        &self,
    ) -> Result<
        (
            MotorCalibrationHandle<'_>,
            MotorCalibrationTicker<'_, PWM, CURRENT, BUS, ROTOR, TEMP, MOD, RotorEst>,
        ),
        CapabilitySplitError,
    > {
        split_once(read_status(&self.shared).active, &self.split_taken)?;
        Ok((
            MotorCalibrationHandle {
                shared: &self.shared,
            },
            MotorCalibrationTicker {
                inner: &self.inner,
                shared: &self.shared,
            },
        ))
    }

    #[cfg(test)]
    pub(crate) fn tick_active_routine_for_test(
        &self,
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
        let mut inner = take_active_inner(&self.inner, || read_status(&self.shared).active, |active| {
            if active {
                MotorCalibrationRuntimeError::Busy
            } else {
                MotorCalibrationRuntimeError::Inactive
            }
        })?;
        let result = inner.tick_active_routine(routine, dt_seconds);
        critical_section::with(|cs| {
            *self.inner.borrow(cs).borrow_mut() = Some(inner);
        });
        result
    }
}
