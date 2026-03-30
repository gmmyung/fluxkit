use super::*;

impl<PWM, CURRENT, BUS, ROTOR, OUTPUT, TEMP, MOD, RotorEst, OutputEst>
    MotorRuntime<PWM, CURRENT, BUS, ROTOR, OUTPUT, TEMP, MOD, RotorEst, OutputEst>
where
    MOD: Modulator,
    RotorEst: MechanicalMotionEstimator,
    OutputEst: MechanicalMotionEstimator,
{
    /// Creates a new runtime with an explicit loop period, controller params,
    /// modulator, and rotor/output estimators.
    pub fn new(
        pwm: PWM,
        current: CURRENT,
        bus: BUS,
        rotor: ROTOR,
        output: OUTPUT,
        temp: TEMP,
        params: MotorRuntimeParams,
        modulator: MOD,
        rotor_estimator: RotorEst,
        output_estimator: OutputEst,
    ) -> Self {
        Self::from_parts(
            MotorRuntimeParts {
                pwm,
                current,
                bus,
                rotor,
                output,
                temp,
                motor: params.motor,
                inverter: params.inverter,
                actuator: params.actuator,
                current_loop: params.current_loop,
                modulator,
                rotor_estimator,
                output_estimator,
            },
            params.dt_seconds,
        )
    }

    /// Creates a runtime from previously owned runtime parts and a loop period.
    pub fn from_parts(
        parts: MotorRuntimeParts<PWM, CURRENT, BUS, ROTOR, OUTPUT, TEMP, MOD, RotorEst, OutputEst>,
        dt_seconds: f32,
    ) -> Self {
        let controller = MotorController::new(
            parts.motor,
            parts.inverter,
            parts.actuator,
            parts.current_loop,
            parts.modulator,
        );
        Self::from_controller_parts(
            Hardware {
                pwm: parts.pwm,
                current: parts.current,
                bus: parts.bus,
                rotor: parts.rotor,
                output: parts.output,
                temp: parts.temp,
            },
            controller,
            parts.rotor_estimator,
            parts.output_estimator,
            dt_seconds,
        )
    }

    fn from_controller_parts(
        hardware: Hardware<PWM, CURRENT, BUS, ROTOR, OUTPUT, TEMP>,
        controller: MotorController<MOD>,
        rotor_estimator: RotorEst,
        output_estimator: OutputEst,
        dt_seconds: f32,
    ) -> Self {
        Self {
            shared: Mutex::new(RefCell::new(SharedRuntimeState {
                command: MotorCommand::default(),
                command_dirty: false,
                status: MotorRuntimeStatus {
                    active: true,
                    controller: controller.status(),
                    last_fast_output: None,
                    armed: false,
                    fault_latched: false,
                },
                clear_fault_requested: false,
            })),
            inner: Mutex::new(RefCell::new(Some(InnerMotorRuntime {
                hardware,
                controller,
                rotor_estimator,
                output_estimator,
                dt_seconds,
                pwm_armed: false,
            }))),
            split_taken: Cell::new(false),
        }
    }

    /// Splits this runtime into its unique main-context handle and IRQ-side ticker.
    ///
    /// This can be called at most once for a given runtime owner.
    #[inline]
    pub fn split(
        &self,
    ) -> Result<
        (
            MotorHandle<'_>,
            MotorTicker<'_, PWM, CURRENT, BUS, ROTOR, OUTPUT, TEMP, MOD, RotorEst, OutputEst>,
        ),
        CapabilitySplitError,
    > {
        split_once(
            critical_section::with(|cs| self.shared.borrow(cs).borrow().status.active),
            &self.split_taken,
        )?;
        Ok((
            MotorHandle {
                shared: &self.shared,
            },
            self.ticker_internal(),
        ))
    }

    #[inline]
    pub(crate) fn ticker_internal(
        &self,
    ) -> MotorTicker<'_, PWM, CURRENT, BUS, ROTOR, OUTPUT, TEMP, MOD, RotorEst, OutputEst> {
        MotorTicker {
            inner: &self.inner,
            shared: &self.shared,
        }
    }

    /// Attempts to take ownership of the active runtime parts for reuse in another phase.
    pub fn try_into_parts(
        &self,
    ) -> Option<MotorRuntimeParts<PWM, CURRENT, BUS, ROTOR, OUTPUT, TEMP, MOD, RotorEst, OutputEst>>
    {
        let InnerMotorRuntime {
            hardware,
            controller,
            rotor_estimator,
            output_estimator,
            ..
        } = critical_section::with(|cs| self.inner.borrow(cs).borrow_mut().take())?;
        critical_section::with(|cs| {
            let mut shared = self.shared.borrow(cs).borrow_mut();
            shared.status.active = false;
            shared.status.armed = false;
            shared.clear_fault_requested = false;
        });
        let MotorControllerParts {
            motor,
            inverter,
            actuator,
            config,
            modulator,
        } = controller.into_parts();
        Some(MotorRuntimeParts {
            pwm: hardware.pwm,
            current: hardware.current,
            bus: hardware.bus,
            rotor: hardware.rotor,
            output: hardware.output,
            temp: hardware.temp,
            motor,
            inverter,
            actuator,
            current_loop: config,
            modulator,
            rotor_estimator,
            output_estimator,
        })
    }
}

impl<PWM, CURRENT, BUS, ROTOR, OUTPUT, TEMP, MOD, RotorEst, OutputEst>
    MotorRuntime<PWM, CURRENT, BUS, ROTOR, OUTPUT, TEMP, MOD, RotorEst, OutputEst>
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
    /// Sets the armed state immediately on the owned runtime.
    #[inline]
    pub(crate) fn set_armed_immediate(
        &self,
        armed: bool,
    ) -> Result<
        (),
        MotorRuntimeError<
            PWM::Error,
            CURRENT::Error,
            BUS::Error,
            ROTOR::Error,
            OUTPUT::Error,
            TEMP::Error,
        >,
    > {
        critical_section::with(|cs| {
            let mut shared = self.shared.borrow(cs).borrow_mut();
            if shared.status.active {
                shared.status.armed = armed;
            }
        });
        let mut inner = take_active_inner(
            &self.inner,
            || critical_section::with(|cs| self.shared.borrow(cs).borrow().status.active),
            |active| {
                if active {
                    MotorRuntimeError::Busy
                } else {
                    MotorRuntimeError::Inactive
                }
            },
        )?;
        {
            let mut runtime = RuntimeLoop {
                runtime: &mut inner,
                shared: &self.shared,
            };
            runtime.sync_runtime_requests()?;
            runtime.publish_runtime_status(None);
        }
        critical_section::with(|cs| {
            *self.inner.borrow(cs).borrow_mut() = Some(inner);
        });
        Ok(())
    }

    #[inline]
    pub(crate) fn apply_command_immediate(&self, command: MotorCommand) {
        critical_section::with(|cs| {
            let mut shared = self.shared.borrow(cs).borrow_mut();
            if shared.status.active {
                shared.command = command;
                shared.command_dirty = true;
            }
        });
    }

    #[inline]
    pub(crate) fn controller_status(&self) -> MotorStatus {
        critical_section::with(|cs| self.shared.borrow(cs).borrow().status.controller)
    }

    #[inline]
    pub(crate) fn friction_compensation_enabled(&self) -> bool {
        critical_section::with(|cs| {
            self.inner
                .borrow(cs)
                .borrow()
                .as_ref()
                .is_some_and(|inner| inner.controller.friction_compensation_enabled())
        })
    }

    #[inline]
    pub(crate) fn apply_actuator_calibration(&self, calibration: &ActuatorCalibration) {
        critical_section::with(|cs| {
            if let Some(inner) = self.inner.borrow(cs).borrow_mut().as_mut() {
                inner.controller.apply_actuator_calibration(calibration);
            }
        });
    }
}

impl<'a, PWM, CURRENT, BUS, ROTOR, OUTPUT, TEMP, MOD, RotorEst, OutputEst>
    MotorTicker<'a, PWM, CURRENT, BUS, ROTOR, OUTPUT, TEMP, MOD, RotorEst, OutputEst>
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
    /// Runs one full control cycle using the configured runtime period.
    pub fn tick(
        &self,
    ) -> Result<
        (),
        MotorRuntimeError<
            PWM::Error,
            CURRENT::Error,
            BUS::Error,
            ROTOR::Error,
            OUTPUT::Error,
            TEMP::Error,
        >,
    > {
        let mut inner = take_active_inner(
            self.inner,
            || critical_section::with(|cs| self.shared.borrow(cs).borrow().status.active),
            |active| {
                if active {
                    MotorRuntimeError::Busy
                } else {
                    MotorRuntimeError::Inactive
                }
            },
        )?;
        let result = {
            let mut runtime = RuntimeLoop {
                runtime: &mut inner,
                shared: self.shared,
            };
            runtime.tick()
        };
        critical_section::with(|cs| {
            *self.inner.borrow(cs).borrow_mut() = Some(inner);
        });
        result
    }
}
