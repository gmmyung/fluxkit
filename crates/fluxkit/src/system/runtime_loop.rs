use super::*;

impl<'a, PWM, CURRENT, BUS, ROTOR, OUTPUT, TEMP, MOD, CurrentEst, RotorEst, OutputEst>
    RuntimeLoop<'a, PWM, CURRENT, BUS, ROTOR, OUTPUT, TEMP, MOD, CurrentEst, RotorEst, OutputEst>
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
    fn finish_cycle_result(
        &mut self,
        result: Result<
            MotorRuntimeOutput,
            MotorRuntimeError<
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
        MotorRuntimeError<
            PWM::Error,
            CURRENT::Error,
            BUS::Error,
            ROTOR::Error,
            OUTPUT::Error,
            TEMP::Error,
        >,
    > {
        match result {
            Ok(_) => Ok(()),
            Err(error) => {
                if !matches!(error, MotorRuntimeError::InvalidCurrentSample) {
                    let _ = self.fault_and_neutral();
                }
                Err(error)
            }
        }
    }

    pub(crate) fn sync_runtime_requests(
        &mut self,
    ) -> Result<
        RuntimeRequest,
        MotorRuntimeError<
            PWM::Error,
            CURRENT::Error,
            BUS::Error,
            ROTOR::Error,
            OUTPUT::Error,
            TEMP::Error,
        >,
    > {
        let request = critical_section::with(|cs| {
            let mut shared = self.shared.borrow(cs).borrow_mut();
            let request = RuntimeRequest {
                command: shared.command,
                armed: shared.status.armed,
                clear_fault_requested: shared.clear_fault_requested,
            };
            shared.clear_fault_requested = false;
            request
        });

        if request.clear_fault_requested {
            self.runtime.controller.clear_error();
            critical_section::with(|cs| {
                self.shared.borrow(cs).borrow_mut().status.fault_latched =
                    self.runtime.controller.status().active_error.is_some();
            });
        }

        if request.armed != self.runtime.pwm_armed {
            if request.armed {
                self.runtime
                    .hardware
                    .pwm
                    .enable()
                    .map_err(MotorRuntimeError::Pwm)?;
            } else {
                self.runtime.controller.set_armed(false);
                self.runtime
                    .hardware
                    .pwm
                    .set_neutral()
                    .map_err(MotorRuntimeError::Pwm)?;
                self.runtime
                    .hardware
                    .pwm
                    .disable()
                    .map_err(MotorRuntimeError::Pwm)?;
            }
            self.runtime.pwm_armed = request.armed;
        }

        Ok(request)
    }

    pub(crate) fn publish_runtime_status(&self, last_fast_output: Option<MotorRuntimeOutput>) {
        critical_section::with(|cs| {
            let mut shared = self.shared.borrow(cs).borrow_mut();
            if let Some(output) = last_fast_output {
                shared.status.last_fast_output = Some(output);
            }
            let controller_status = self.runtime.controller.status();
            shared.status.output_velocity = controller_status.last_output_mechanical_velocity;
            shared.status.controller = controller_status;
            shared.status.fault_latched |= controller_status.active_error.is_some();
        });
    }

    fn mark_runtime_fault(&self) {
        critical_section::with(|cs| {
            self.shared.borrow(cs).borrow_mut().status.fault_latched = true;
        });
    }

    fn fault_and_neutral(
        &mut self,
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
        self.runtime
            .hardware
            .pwm
            .set_neutral()
            .map_err(MotorRuntimeError::Pwm)?;
        self.mark_runtime_fault();
        self.publish_runtime_status(None);
        Ok(())
    }

    /// Runs one full control cycle using the configured runtime period.
    ///
    /// This executes one full owned controller cycle using the configured `dt`.
    ///
    /// Internally that means:
    ///
    /// 1. sample hardware and run the electrical current loop
    /// 2. update the supervisory references for the next cycle
    ///
    /// so supervisory updates affect the next electrical step, not the current one.
    pub fn tick(
        &mut self,
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
        let result = self.execute_fast_cycle(self.runtime.dt_seconds);
        self.finish_cycle_result(result)
    }

    /// Samples hardware, runs one owned controller cycle, and applies duty.
    pub(crate) fn run_cycle(
        &mut self,
        dt_seconds: f32,
    ) -> Result<
        MotorRuntimeOutput,
        MotorRuntimeError<
            PWM::Error,
            CURRENT::Error,
            BUS::Error,
            ROTOR::Error,
            OUTPUT::Error,
            TEMP::Error,
        >,
    > {
        let request = self.sync_runtime_requests()?;
        if !request.armed {
            let output = MotorRuntimeOutput::from_fast_loop(ControlOutput {
                phase_duty: fluxkit_hal::centered_phase_duty(),
                measured_idq: fluxkit_math::frame::Dq::new(Amps::ZERO, Amps::ZERO),
                commanded_vdq: fluxkit_math::frame::Dq::new(Volts::ZERO, Volts::ZERO),
                saturated: false,
            });
            self.publish_runtime_status(Some(output));
            return Ok(output);
        }

        let current = self
            .runtime
            .hardware
            .current
            .sample_phase_currents()
            .map_err(MotorRuntimeError::Current)?;

        if current.validity == CurrentSampleValidity::Invalid {
            self.fault_and_neutral()?;
            return Err(MotorRuntimeError::InvalidCurrentSample);
        }

        let bus_voltage = self
            .runtime
            .hardware
            .bus
            .sample_bus_voltage()
            .map_err(MotorRuntimeError::Bus)?;
        let winding_temperature_c = self
            .runtime
            .hardware
            .temp
            .sample_temperature_c()
            .map_err(MotorRuntimeError::Temp)?;

        let rotor = self
            .runtime
            .hardware
            .rotor
            .read_rotor()
            .map_err(MotorRuntimeError::Rotor)?;
        let output_axis = self
            .runtime
            .hardware
            .output
            .read_output()
            .map_err(MotorRuntimeError::Output)?;
        let rotor_motion = self.runtime.rotor_estimator.update(
            MechanicalMotionSample {
                wrapped_value: rotor.mechanical_angle,
                measured_rate: rotor.mechanical_velocity,
            },
            dt_seconds,
        );
        let output_motion = self.runtime.output_estimator.update(
            MechanicalMotionSample {
                wrapped_value: output_axis.mechanical_angle,
                measured_rate: output_axis.mechanical_velocity,
            },
            dt_seconds,
        );

        let output = self.runtime.controller.step(ControlInput {
            command: core_command_from_runtime(request.command),
            armed: request.armed,
            clear_fault_requested: false,
            phase_currents: current.currents,
            bus_voltage,
            winding_temperature_c,
            rotor: RotorEstimate {
                mechanical_angle: rotor_motion.unwrapped(),
                mechanical_velocity: rotor_motion.velocity(),
            },
            actuator: ActuatorEstimate {
                output_angle: output_motion.unwrapped(),
                output_velocity: output_motion.velocity(),
            },
            dt_seconds,
        });

        self.runtime
            .hardware
            .pwm
            .set_phase_duty(output.phase_duty)
            .map_err(MotorRuntimeError::Pwm)?;

        let output = MotorRuntimeOutput::from_fast_loop(output);
        self.publish_runtime_status(Some(output));
        Ok(output)
    }

    #[inline]
    pub(crate) fn execute_fast_cycle(
        &mut self,
        dt_seconds: f32,
    ) -> Result<
        MotorRuntimeOutput,
        MotorRuntimeError<
            PWM::Error,
            CURRENT::Error,
            BUS::Error,
            ROTOR::Error,
            OUTPUT::Error,
            TEMP::Error,
        >,
    > {
        self.run_cycle(dt_seconds)
    }
}
