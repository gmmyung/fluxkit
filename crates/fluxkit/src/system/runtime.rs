use super::*;

impl<PWM, CURRENT, BUS, ROTOR, OUTPUT, TEMP, MOD, CurrentEst, RotorEst, OutputEst>
    MotorRuntime<PWM, CURRENT, BUS, ROTOR, OUTPUT, TEMP, MOD, CurrentEst, RotorEst, OutputEst>
where
    MOD: Modulator,
    CurrentEst: CurrentEstimator,
    RotorEst: MechanicalMotionEstimator,
    OutputEst: MechanicalMotionEstimator,
{
    /// Creates a new runtime from grouped hardware, params, and algorithms.
    pub fn new(
        hardware: MotorHardware<PWM, CURRENT, BUS, ROTOR, OUTPUT, TEMP>,
        params: MotorRuntimeParams,
        algorithms: RuntimeAlgorithms<MOD, CurrentEst, RotorEst, OutputEst>,
    ) -> Result<Self, MotorRuntimeBuildError> {
        Self::from_parts(MotorRuntimeBundle {
            hardware,
            params,
            algorithms,
        })
    }

    /// Creates a runtime from previously owned runtime parts.
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
    ) -> Result<Self, MotorRuntimeBuildError> {
        if !validate_dt_seconds(parts.params.dt_seconds) {
            return Err(MotorRuntimeBuildError::InvalidDtSeconds);
        }

        let controller = MotorController::new(
            parts.params.motor,
            parts.params.inverter,
            parts.params.actuator,
            parts.params.current_loop,
            parts.algorithms.modulator,
            parts.algorithms.current_estimator,
        );
        Self::from_controller_parts(
            parts.hardware,
            controller,
            parts.algorithms.rotor_estimator,
            parts.algorithms.output_estimator,
            parts.params.dt_seconds,
        )
    }

    fn from_controller_parts(
        hardware: MotorHardware<PWM, CURRENT, BUS, ROTOR, OUTPUT, TEMP>,
        controller: MotorController<MOD, CurrentEst>,
        rotor_estimator: RotorEst,
        output_estimator: OutputEst,
        dt_seconds: f32,
    ) -> Result<Self, MotorRuntimeBuildError> {
        Ok(Self {
            shared: Mutex::new(RefCell::new(SharedRuntimeState {
                command: MotorCommand::default(),
                status: MotorRuntimeStatus {
                    active: true,
                    controller: controller.status(),
                    output_velocity: RadPerSec::ZERO,
                    last_control_output: None,
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
        })
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
            MotorTicker<
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
    ) -> MotorTicker<'_, PWM, CURRENT, BUS, ROTOR, OUTPUT, TEMP, MOD, CurrentEst, RotorEst, OutputEst>
    {
        MotorTicker {
            inner: &self.inner,
            shared: &self.shared,
        }
    }

    /// Attempts to take ownership of the active runtime parts for reuse in another phase.
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
        let InnerMotorRuntime {
            hardware,
            controller,
            rotor_estimator,
            output_estimator,
            dt_seconds,
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
            current_estimator,
        } = controller.into_parts();
        Some(MotorRuntimeBundle {
            hardware,
            params: MotorRuntimeParams {
                motor,
                inverter,
                actuator,
                current_loop: config,
                dt_seconds,
            },
            algorithms: RuntimeAlgorithms {
                modulator,
                current_estimator,
                rotor_estimator,
                output_estimator,
            },
        })
    }
}

impl<PWM, CURRENT, BUS, ROTOR, OUTPUT, TEMP, MOD, CurrentEst, RotorEst, OutputEst>
    MotorRuntime<PWM, CURRENT, BUS, ROTOR, OUTPUT, TEMP, MOD, CurrentEst, RotorEst, OutputEst>
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

impl<'a, PWM, CURRENT, BUS, ROTOR, OUTPUT, TEMP, MOD, CurrentEst, RotorEst, OutputEst>
    MotorTicker<'a, PWM, CURRENT, BUS, ROTOR, OUTPUT, TEMP, MOD, CurrentEst, RotorEst, OutputEst>
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
