use super::*;

impl<PWM, CURRENT, BUS, ROTOR, OUTPUT, TEMP, MOD, CurrentEst, RotorEst, OutputEst>
    ActuatorCalibrationRuntime<
        PWM,
        CURRENT,
        BUS,
        ROTOR,
        OUTPUT,
        TEMP,
        MOD,
        CurrentEst,
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
    CurrentEst: CurrentEstimator,
    RotorEst: MechanicalMotionEstimator,
    OutputEst: MechanicalMotionEstimator,
{
    /// Creates a new actuator-calibration runtime from an owned runtime bundle.
    ///
    /// The bundle's actuator parameters are replaced with a neutral actuator
    /// placeholder before calibration starts:
    ///
    /// - `gear_ratio = 1.0`
    /// - output-axis velocity limit copied from calibration limits
    /// - output torque left unconstrained by placeholder actuator limits so the
    ///   calibration routines can still establish motion before gear ratio and
    ///   friction are known
    /// - friction compensation disabled
    ///
    /// The intended high-level flow is to run gear-ratio calibration first and
    /// then let subsequent completed actuator-calibration deltas patch the live
    /// controller parameters automatically.
    pub fn new(
        mut bundle: MotorRuntimeBundle<
            PWM,
            CURRENT,
            BUS,
            ROTOR,
            OUTPUT,
            TEMP,
            MOD,
            CurrentEst,
            RotorEst,
            OutputEst,
        >,
        request: ActuatorCalibrationRequest,
        limits: ActuatorCalibrationLimits,
    ) -> Result<Self, CalibrationError> {
        bundle.params.actuator = placeholder_actuator_params(limits);
        Self::from_parts(bundle, request, limits)
    }

    /// Creates a new actuator-calibration runtime from owned runtime parts.
    pub fn from_parts(
        parts: MotorRuntimeBundle<
            PWM,
            CURRENT,
            BUS,
            ROTOR,
            OUTPUT,
            TEMP,
            MOD,
            CurrentEst,
            RotorEst,
            OutputEst,
        >,
        request: ActuatorCalibrationRequest,
        limits: ActuatorCalibrationLimits,
    ) -> Result<Self, CalibrationError> {
        let dt_seconds = parts.params.dt_seconds;
        if !validate_limits(limits) || !validate_dt_seconds(dt_seconds) {
            return Err(CalibrationError::InvalidConfiguration);
        }

        let motor_system =
            MotorRuntime::from_parts(parts).map_err(|_| CalibrationError::InvalidConfiguration)?;
        let current_phase = next_phase_for_request(request);
        let resolved_result = resolved_result_for_request(request);

        let mut inner = InnerActuatorCalibrationRuntime {
            motor_system,
            limits,
            dt_seconds,
            gear_ratio: request.gear_ratio,
            positive_breakaway_torque: request.positive_breakaway_torque,
            negative_breakaway_torque: request.negative_breakaway_torque,
            positive_coulomb_torque: request.positive_coulomb_torque,
            negative_coulomb_torque: request.negative_coulomb_torque,
            positive_viscous_coefficient: request.positive_viscous_coefficient,
            negative_viscous_coefficient: request.negative_viscous_coefficient,
            zero_velocity_blend_band: request.zero_velocity_blend_band,
            current_phase,
            resolved_result,
            active_routine: None,
        };
        let partial = inner.partial_calibration();
        inner.apply_live_calibration(&partial);
        Ok(Self {
            inner: Mutex::new(RefCell::new(Some(inner))),
            shared: Mutex::new(RefCell::new(SharedStatus {
                status: ActuatorCalibrationStatus {
                    active: true,
                    phase: current_phase,
                    result: resolved_result,
                    fault_latched: false,
                },
            })),
            split_taken: Cell::new(false),
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
            ActuatorCalibrationHandle<'_>,
            ActuatorCalibrationTicker<
                '_,
                PWM,
                CURRENT,
                BUS,
                ROTOR,
                OUTPUT,
                TEMP,
                MOD,
                CurrentEst,
                RotorEst,
                OutputEst,
            >,
        ),
        CapabilitySplitError,
    > {
        split_once(read_status(&self.shared).active, &self.split_taken)?;
        Ok((
            ActuatorCalibrationHandle {
                shared: &self.shared,
            },
            ActuatorCalibrationTicker {
                inner: &self.inner,
                shared: &self.shared,
            },
        ))
    }

    /// Attempts to take ownership of the active runtime parts for reuse in another phase.
    #[inline]
    pub fn try_into_parts(
        &self,
    ) -> Option<
        MotorRuntimeBundle<
            PWM,
            CURRENT,
            BUS,
            ROTOR,
            OUTPUT,
            TEMP,
            MOD,
            CurrentEst,
            RotorEst,
            OutputEst,
        >,
    > {
        let inner = critical_section::with(|cs| self.inner.borrow(cs).borrow_mut().take())?;
        write_status(&self.shared, |status| status.active = false);
        inner.motor_system.try_into_parts()
    }
}
