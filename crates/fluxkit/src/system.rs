//! Generic synchronous integration wrapper over Fluxkit core and HAL traits.

use core::fmt;

use fluxkit_core::{
    ActuatorEstimate, FastLoopInput, FastLoopOutput, MotorController, RotorEstimate, TickSchedule,
};
use fluxkit_hal::{
    BusVoltageSensor, CurrentSampleValidity, CurrentSampler, OutputSensor, PhasePwm, RotorSensor,
};
use fluxkit_math::{
    MechanicalMotionEstimate, MechanicalMotionSample, MechanicalMotionSeed, Modulator,
    WrappedEstimator,
};

/// Concrete hardware handles required to run one motor-control loop.
#[derive(Debug)]
pub struct MotorHardware<PWM, CURRENT, BUS, ROTOR, OUTPUT> {
    /// Three-phase PWM output stage.
    pub pwm: PWM,
    /// Phase-current acquisition path.
    pub current: CURRENT,
    /// DC bus-voltage acquisition path.
    pub bus: BUS,
    /// Absolute-encoder rotor sensing path.
    pub rotor: ROTOR,
    /// Output-axis sensing path.
    pub output: OUTPUT,
}

/// HAL and integration failures that can occur outside the pure controller.
#[derive(Debug)]
pub enum MotorSystemError<PwmE, CurrentE, BusE, RotorE, OutputE> {
    /// PWM output operation failed.
    Pwm(PwmE),
    /// Phase-current acquisition failed.
    Current(CurrentE),
    /// DC bus-voltage acquisition failed.
    Bus(BusE),
    /// Rotor-sensor acquisition failed.
    Rotor(RotorE),
    /// Output-sensor acquisition failed.
    Output(OutputE),
    /// The current sample was explicitly marked invalid for control use.
    InvalidCurrentSample,
}

impl<PwmE, CurrentE, BusE, RotorE, OutputE> fmt::Display
    for MotorSystemError<PwmE, CurrentE, BusE, RotorE, OutputE>
where
    PwmE: fmt::Display,
    CurrentE: fmt::Display,
    BusE: fmt::Display,
    RotorE: fmt::Display,
    OutputE: fmt::Display,
{
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Pwm(error) => write!(f, "pwm error: {error}"),
            Self::Current(error) => write!(f, "current-sensor error: {error}"),
            Self::Bus(error) => write!(f, "bus-voltage error: {error}"),
            Self::Rotor(error) => write!(f, "rotor-sensor error: {error}"),
            Self::Output(error) => write!(f, "output-sensor error: {error}"),
            Self::InvalidCurrentSample => f.write_str("invalid current sample"),
        }
    }
}

impl<PwmE, CurrentE, BusE, RotorE, OutputE> core::error::Error
    for MotorSystemError<PwmE, CurrentE, BusE, RotorE, OutputE>
where
    PwmE: core::error::Error + 'static,
    CurrentE: core::error::Error + 'static,
    BusE: core::error::Error + 'static,
    RotorE: core::error::Error + 'static,
    OutputE: core::error::Error + 'static,
{
    fn source(&self) -> Option<&(dyn core::error::Error + 'static)> {
        match self {
            Self::Pwm(error) => Some(error),
            Self::Current(error) => Some(error),
            Self::Bus(error) => Some(error),
            Self::Rotor(error) => Some(error),
            Self::Output(error) => Some(error),
            Self::InvalidCurrentSample => None,
        }
    }
}

/// Encapsulated synchronous motor stack: hardware plus pure controller.
#[derive(Debug)]
pub struct MotorSystem<PWM, CURRENT, BUS, ROTOR, OUTPUT, MOD, RotorEst, OutputEst> {
    hardware: MotorHardware<PWM, CURRENT, BUS, ROTOR, OUTPUT>,
    controller: MotorController<MOD>,
    rotor_estimator: RotorEst,
    output_estimator: OutputEst,
}

/// Helper trait for estimators that consume wrapped mechanical motion samples
/// and produce continuous mechanical motion estimates.
pub trait MechanicalMotionEstimator:
    WrappedEstimator<
        Input = MechanicalMotionSample,
        Output = MechanicalMotionEstimate,
        Seed = MechanicalMotionSeed,
    >
{
}

impl<T> MechanicalMotionEstimator for T where
    T: WrappedEstimator<
            Input = MechanicalMotionSample,
            Output = MechanicalMotionEstimate,
            Seed = MechanicalMotionSeed,
        >
{
}

impl<PWM, CURRENT, BUS, ROTOR, OUTPUT, MOD, RotorEst, OutputEst>
    MotorSystem<PWM, CURRENT, BUS, ROTOR, OUTPUT, MOD, RotorEst, OutputEst>
where
    RotorEst: MechanicalMotionEstimator,
    OutputEst: MechanicalMotionEstimator,
{
    /// Creates a new motor system with an explicit controller modulator and
    /// explicit rotor/output estimators.
    pub fn new(
        hardware: MotorHardware<PWM, CURRENT, BUS, ROTOR, OUTPUT>,
        controller: MotorController<MOD>,
        rotor_estimator: RotorEst,
        output_estimator: OutputEst,
    ) -> Self {
        Self {
            hardware,
            controller,
            rotor_estimator,
            output_estimator,
        }
    }

    /// Returns shared access to the rotor-side motion estimator.
    #[inline]
    pub const fn rotor_estimator(&self) -> &RotorEst {
        &self.rotor_estimator
    }

    /// Returns mutable access to the rotor-side motion estimator.
    #[inline]
    pub fn rotor_estimator_mut(&mut self) -> &mut RotorEst {
        &mut self.rotor_estimator
    }

    /// Returns shared access to the output-side motion estimator.
    #[inline]
    pub const fn output_estimator(&self) -> &OutputEst {
        &self.output_estimator
    }

    /// Returns mutable access to the output-side motion estimator.
    #[inline]
    pub fn output_estimator_mut(&mut self) -> &mut OutputEst {
        &mut self.output_estimator
    }
}

impl<PWM, CURRENT, BUS, ROTOR, OUTPUT, MOD, RotorEst, OutputEst>
    MotorSystem<PWM, CURRENT, BUS, ROTOR, OUTPUT, MOD, RotorEst, OutputEst>
where
    PWM: PhasePwm,
    CURRENT: CurrentSampler,
    BUS: BusVoltageSensor,
    ROTOR: RotorSensor,
    OUTPUT: OutputSensor,
    MOD: Modulator,
    RotorEst: MechanicalMotionEstimator,
    OutputEst: MechanicalMotionEstimator,
{
    /// Returns shared access to the owned hardware handles.
    #[inline]
    pub const fn hardware(&self) -> &MotorHardware<PWM, CURRENT, BUS, ROTOR, OUTPUT> {
        &self.hardware
    }

    /// Returns mutable access to the owned hardware handles.
    #[inline]
    pub fn hardware_mut(&mut self) -> &mut MotorHardware<PWM, CURRENT, BUS, ROTOR, OUTPUT> {
        &mut self.hardware
    }

    /// Returns shared access to the owned pure controller.
    #[inline]
    pub const fn controller(&self) -> &MotorController<MOD> {
        &self.controller
    }

    /// Returns mutable access to the owned pure controller.
    #[inline]
    pub fn controller_mut(&mut self) -> &mut MotorController<MOD> {
        &mut self.controller
    }

    /// Splits the system back into owned hardware and controller parts.
    #[inline]
    pub fn into_parts(
        self,
    ) -> (
        MotorHardware<PWM, CURRENT, BUS, ROTOR, OUTPUT>,
        MotorController<MOD>,
        RotorEst,
        OutputEst,
    ) {
        (
            self.hardware,
            self.controller,
            self.rotor_estimator,
            self.output_estimator,
        )
    }

    /// Enables the underlying PWM and then enables the controller.
    pub fn enable(
        &mut self,
    ) -> Result<
        (),
        MotorSystemError<PWM::Error, CURRENT::Error, BUS::Error, ROTOR::Error, OUTPUT::Error>,
    > {
        self.hardware.pwm.enable().map_err(MotorSystemError::Pwm)?;
        self.controller.enable();
        Ok(())
    }

    /// Forces a neutral output, disables the controller, then disables the PWM.
    pub fn disable(
        &mut self,
    ) -> Result<
        (),
        MotorSystemError<PWM::Error, CURRENT::Error, BUS::Error, ROTOR::Error, OUTPUT::Error>,
    > {
        self.controller.disable();
        self.hardware
            .pwm
            .set_neutral()
            .map_err(MotorSystemError::Pwm)?;
        self.hardware.pwm.disable().map_err(MotorSystemError::Pwm)?;
        Ok(())
    }

    /// Samples hardware, runs one scheduled controller cycle, and applies duty.
    ///
    /// This is the preferred entrypoint for multi-rate runtime integration:
    /// call it from the fast-loop owner, and express lower-rate work through
    /// `schedule` instead of calling separate controller hooks from other
    /// interrupts.
    pub fn tick(
        &mut self,
        dt_seconds: f32,
        schedule: TickSchedule,
    ) -> Result<
        FastLoopOutput,
        MotorSystemError<PWM::Error, CURRENT::Error, BUS::Error, ROTOR::Error, OUTPUT::Error>,
    > {
        let current = self
            .hardware
            .current
            .sample_phase_currents()
            .map_err(MotorSystemError::Current)?;

        if current.validity == CurrentSampleValidity::Invalid {
            self.hardware
                .pwm
                .set_neutral()
                .map_err(MotorSystemError::Pwm)?;
            return Err(MotorSystemError::InvalidCurrentSample);
        }

        let bus_voltage = self
            .hardware
            .bus
            .sample_bus_voltage()
            .map_err(MotorSystemError::Bus)?;

        let rotor = self
            .hardware
            .rotor
            .read_rotor()
            .map_err(MotorSystemError::Rotor)?;
        let output_axis = self
            .hardware
            .output
            .read_output()
            .map_err(MotorSystemError::Output)?;
        let rotor_motion = self.rotor_estimator.update(
            MechanicalMotionSample {
                wrapped_value: rotor.mechanical_angle,
                measured_rate: rotor.mechanical_velocity,
            },
            dt_seconds,
        );
        let output_motion = self.output_estimator.update(
            MechanicalMotionSample {
                wrapped_value: output_axis.mechanical_angle,
                measured_rate: output_axis.mechanical_velocity,
            },
            dt_seconds,
        );

        let output = self.controller.tick(
            FastLoopInput {
                phase_currents: current.currents,
                bus_voltage,
                rotor: RotorEstimate {
                    mechanical_angle: rotor_motion.unwrapped(),
                    mechanical_velocity: rotor_motion.velocity(),
                },
                actuator: ActuatorEstimate {
                    output_angle: output_motion.unwrapped(),
                    output_velocity: output_motion.velocity(),
                },
                dt_seconds,
            },
            schedule,
        );

        self.hardware
            .pwm
            .set_phase_duty(output.phase_duty)
            .map_err(MotorSystemError::Pwm)?;

        Ok(output)
    }

    /// Samples hardware, runs only the fast controller loop, and applies duty.
    ///
    /// This is a convenience wrapper around [`Self::tick`] with
    /// [`TickSchedule::none`].
    #[inline]
    pub fn fast_tick(
        &mut self,
        dt_seconds: f32,
    ) -> Result<
        FastLoopOutput,
        MotorSystemError<PWM::Error, CURRENT::Error, BUS::Error, ROTOR::Error, OUTPUT::Error>,
    > {
        self.tick(dt_seconds, TickSchedule::none())
    }
}

#[cfg(test)]
mod tests {
    use core::convert::Infallible;

    use fluxkit_core::{
        ActuatorLimits, ActuatorModel, ActuatorParams, ControlMode, CurrentLoopConfig,
        InverterParams, MotorLimits, MotorModel, MotorParams, MotorState, TickSchedule,
    };
    use fluxkit_hal::{
        BusVoltageSensor, CurrentSampleValidity, CurrentSampler, OutputReading, OutputSensor,
        PhaseCurrentSample, PhasePwm, RotorReading, RotorSensor, centered_phase_duty,
    };
    use fluxkit_math::{
        ContinuousMechanicalAngle, MechanicalAngle, WrappedEstimator,
        estimation::{
            AngularEstimate, EstimatorSeed, MechanicalMotionEstimate, MechanicalMotionSample,
            MechanicalMotionSeed,
        },
        frame::Abc,
        units::{Amps, Duty, Henries, Hertz, NewtonMeters, Ohms, RadPerSec, Volts},
    };

    use super::{MotorHardware, MotorSystem, MotorSystemError};

    #[derive(Debug)]
    struct FakePwm {
        enabled: bool,
        duty: Abc<Duty>,
    }

    impl Default for FakePwm {
        fn default() -> Self {
            Self {
                enabled: false,
                duty: centered_phase_duty(),
            }
        }
    }

    impl PhasePwm for FakePwm {
        type Error = Infallible;

        fn enable(&mut self) -> Result<(), Self::Error> {
            self.enabled = true;
            Ok(())
        }

        fn disable(&mut self) -> Result<(), Self::Error> {
            self.enabled = false;
            Ok(())
        }

        fn set_duty(&mut self, a: Duty, b: Duty, c: Duty) -> Result<(), Self::Error> {
            self.duty = Abc::new(a, b, c);
            Ok(())
        }
    }

    #[derive(Debug)]
    struct FakeCurrentSensor {
        sample: PhaseCurrentSample,
    }

    impl CurrentSampler for FakeCurrentSensor {
        type Error = Infallible;

        fn sample_phase_currents(&mut self) -> Result<PhaseCurrentSample, Self::Error> {
            Ok(self.sample)
        }
    }

    #[derive(Debug)]
    struct FakeBusSensor {
        voltage: Volts,
    }

    impl BusVoltageSensor for FakeBusSensor {
        type Error = Infallible;

        fn sample_bus_voltage(&mut self) -> Result<Volts, Self::Error> {
            Ok(self.voltage)
        }
    }

    #[derive(Debug)]
    struct FakeRotor {
        reading: RotorReading,
    }

    impl RotorSensor for FakeRotor {
        type Error = Infallible;

        fn read_rotor(&mut self) -> Result<RotorReading, Self::Error> {
            Ok(self.reading)
        }
    }

    #[derive(Debug)]
    struct FakeOutput {
        reading: OutputReading,
    }

    impl OutputSensor for FakeOutput {
        type Error = Infallible;

        fn read_output(&mut self) -> Result<OutputReading, Self::Error> {
            Ok(self.reading)
        }
    }

    #[derive(Clone, Copy, Debug)]
    struct FixedEstimator {
        output: MechanicalMotionEstimate,
    }

    impl WrappedEstimator for FixedEstimator {
        type Input = MechanicalMotionSample;
        type Output = MechanicalMotionEstimate;
        type Seed = MechanicalMotionSeed;

        fn initialize(&mut self, seed: Self::Seed) {
            match seed {
                EstimatorSeed::Uninitialized => {
                    self.output = AngularEstimate::new(
                        MechanicalAngle::new(0.0),
                        ContinuousMechanicalAngle::new(0.0),
                        RadPerSec::ZERO,
                    );
                }
                EstimatorSeed::Value(wrapped_value) => {
                    self.output = AngularEstimate::new(
                        wrapped_value.wrapped(),
                        wrapped_value,
                        RadPerSec::ZERO,
                    );
                }
                EstimatorSeed::ValueRate {
                    value: wrapped_value,
                    rate: measured_rate,
                } => {
                    self.output =
                        AngularEstimate::new(wrapped_value.wrapped(), wrapped_value, measured_rate);
                }
                EstimatorSeed::Estimate(estimate) => {
                    self.output = estimate;
                }
            }
        }

        fn output(&self) -> Self::Output {
            self.output
        }

        fn update(&mut self, _sample: Self::Input, _dt: f32) -> Self::Output {
            self.output
        }
    }

    fn motor_params() -> MotorParams {
        MotorParams::from_model_and_limits(
            MotorModel {
                pole_pairs: 7,
                phase_resistance_ohm: Ohms::new(0.08),
                d_inductance_h: Henries::new(0.00012),
                q_inductance_h: Henries::new(0.00012),
                flux_linkage_weber: fluxkit_math::units::Webers::new(0.05),
                electrical_angle_offset: fluxkit_math::ElectricalAngle::new(0.0),
            },
            MotorLimits {
                max_phase_current: Amps::new(20.0),
                max_mech_speed: None,
            },
        )
    }

    fn inverter_params() -> InverterParams {
        InverterParams {
            pwm_frequency_hz: Hertz::new(20_000.0),
            min_duty: Duty::new(0.0),
            max_duty: Duty::new(1.0),
            min_bus_voltage: Volts::new(6.0),
            max_bus_voltage: Volts::new(60.0),
            max_voltage_command: Volts::new(24.0),
        }
    }

    fn actuator_params() -> ActuatorParams {
        ActuatorParams::from_model_limits_and_compensation(
            ActuatorModel { gear_ratio: 5.0 },
            ActuatorLimits {
                max_output_velocity: Some(RadPerSec::new(10.0)),
                max_output_torque: Some(NewtonMeters::new(10.0)),
            },
            fluxkit_core::ActuatorCompensationConfig::disabled(),
        )
    }

    fn current_loop_config() -> CurrentLoopConfig {
        CurrentLoopConfig {
            kp_d: 0.2,
            ki_d: 25.0,
            kp_q: 0.3,
            ki_q: 30.0,
            velocity_kp: 0.5,
            velocity_ki: 10.0,
            position_kp: 4.0,
            position_ki: 0.0,
            max_voltage_mag: Volts::new(12.0),
            id_ref_default: Amps::ZERO,
            max_id_target: Amps::new(5.0),
            max_iq_target: Amps::new(10.0),
            max_velocity_target: RadPerSec::new(50.0),
            max_current_ref_derivative_amps_per_sec: 10_000.0,
            enable_current_feedforward: true,
        }
    }

    fn hardware(
        validity: CurrentSampleValidity,
    ) -> MotorHardware<FakePwm, FakeCurrentSensor, FakeBusSensor, FakeRotor, FakeOutput> {
        MotorHardware {
            pwm: FakePwm::default(),
            current: FakeCurrentSensor {
                sample: PhaseCurrentSample {
                    currents: Abc::new(Amps::new(0.0), Amps::new(0.0), Amps::new(0.0)),
                    validity,
                },
            },
            bus: FakeBusSensor {
                voltage: Volts::new(24.0),
            },
            rotor: FakeRotor {
                reading: RotorReading {
                    mechanical_angle: fluxkit_math::MechanicalAngle::new(0.0),
                    mechanical_velocity: RadPerSec::new(0.0),
                },
            },
            output: FakeOutput {
                reading: OutputReading {
                    mechanical_angle: fluxkit_math::MechanicalAngle::new(0.0),
                    mechanical_velocity: RadPerSec::new(0.0),
                },
            },
        }
    }

    #[test]
    fn fast_tick_reads_hal_and_applies_phase_duty() {
        let controller = fluxkit_core::MotorController::new(
            motor_params(),
            inverter_params(),
            actuator_params(),
            current_loop_config(),
        );
        let mut system = MotorSystem::new(
            hardware(CurrentSampleValidity::Valid),
            controller,
            fluxkit_math::PassThroughEstimator::new(),
            fluxkit_math::PassThroughEstimator::new(),
        );

        system.enable().unwrap();
        system.controller_mut().set_mode(ControlMode::Current);
        system.controller_mut().set_iq_target(Amps::new(2.0));

        let _output = system.fast_tick(0.000_05).unwrap();

        assert_eq!(system.controller().status().active_error, None);
        assert!(system.hardware().pwm.enabled);
        assert_eq!(system.controller().status().state, MotorState::Running);
        assert!(system.hardware().pwm.duty.a.get() >= 0.0);
        assert!(system.hardware().pwm.duty.a.get() <= 1.0);
        assert_ne!(system.hardware().pwm.duty, centered_phase_duty());
    }

    #[test]
    fn invalid_current_sample_returns_error_and_forces_neutral_pwm() {
        let controller = fluxkit_core::MotorController::new(
            motor_params(),
            inverter_params(),
            actuator_params(),
            current_loop_config(),
        );
        let mut system = MotorSystem::new(
            hardware(CurrentSampleValidity::Invalid),
            controller,
            fluxkit_math::PassThroughEstimator::new(),
            fluxkit_math::PassThroughEstimator::new(),
        );
        system.hardware_mut().pwm.duty = Abc::new(Duty::new(0.2), Duty::new(0.7), Duty::new(0.6));

        let error = system.fast_tick(0.000_05).unwrap_err();

        assert!(matches!(error, MotorSystemError::InvalidCurrentSample));
        assert_eq!(system.hardware().pwm.duty, centered_phase_duty());
    }

    #[test]
    fn scheduled_tick_runs_supervisory_work_after_fast_cycle() {
        let controller = fluxkit_core::MotorController::new(
            motor_params(),
            inverter_params(),
            actuator_params(),
            current_loop_config(),
        );
        let mut system = MotorSystem::new(
            hardware(CurrentSampleValidity::Valid),
            controller,
            fluxkit_math::PassThroughEstimator::new(),
            fluxkit_math::PassThroughEstimator::new(),
        );

        system.enable().unwrap();
        system.controller_mut().set_mode(ControlMode::Position);
        system
            .controller_mut()
            .set_position_target(ContinuousMechanicalAngle::new(1.0));

        let first = system
            .tick(0.000_05, TickSchedule::with_medium(0.001))
            .unwrap();
        let second = system.fast_tick(0.000_05).unwrap();

        assert_eq!(first.phase_duty, centered_phase_duty());
        assert_ne!(second.phase_duty, centered_phase_duty());
        assert!(
            system
                .controller()
                .status()
                .last_output_mechanical_angle
                .get()
                .abs()
                < 1.0e-6
        );
    }

    #[test]
    fn explicit_estimators_drive_controller_side_motion_estimates() {
        let controller = fluxkit_core::MotorController::new(
            motor_params(),
            inverter_params(),
            actuator_params(),
            current_loop_config(),
        );
        let mut system = MotorSystem::new(
            hardware(CurrentSampleValidity::Valid),
            controller,
            FixedEstimator {
                output: AngularEstimate::new(
                    MechanicalAngle::new(0.3),
                    ContinuousMechanicalAngle::new(1.3),
                    RadPerSec::new(4.0),
                ),
            },
            FixedEstimator {
                output: AngularEstimate::new(
                    MechanicalAngle::new(0.6),
                    ContinuousMechanicalAngle::new(2.6),
                    RadPerSec::new(1.5),
                ),
            },
        );

        system.enable().unwrap();
        system.controller_mut().set_mode(ControlMode::Current);
        let _ = system.fast_tick(0.000_05).unwrap();

        let status = system.controller().status();
        assert_eq!(
            status.last_rotor_mechanical_angle,
            ContinuousMechanicalAngle::new(1.3)
        );
        assert_eq!(
            status.last_output_mechanical_angle,
            ContinuousMechanicalAngle::new(2.6)
        );
        assert_eq!(status.last_output_mechanical_velocity, RadPerSec::new(1.5));
    }
}
