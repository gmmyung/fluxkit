//! Generic synchronous integration wrapper over Fluxkit core and HAL traits.

use core::fmt;

use fluxkit_core::{FastLoopInput, FastLoopOutput, MotorController, RotorEstimate};
use fluxkit_hal::{BusVoltageSensor, CurrentSampleValidity, CurrentSampler, PhasePwm, RotorSensor};
use fluxkit_math::{Modulator, Svpwm};

/// Concrete hardware handles required to run one motor-control loop.
#[derive(Debug)]
pub struct MotorHardware<PWM, CURRENT, BUS, ROTOR> {
    /// Three-phase PWM output stage.
    pub pwm: PWM,
    /// Phase-current acquisition path.
    pub current: CURRENT,
    /// DC bus-voltage acquisition path.
    pub bus: BUS,
    /// Absolute-encoder rotor sensing path.
    pub rotor: ROTOR,
}

/// HAL and integration failures that can occur outside the pure controller.
#[derive(Debug)]
pub enum MotorSystemError<PwmE, CurrentE, BusE, RotorE> {
    /// PWM output operation failed.
    Pwm(PwmE),
    /// Phase-current acquisition failed.
    Current(CurrentE),
    /// DC bus-voltage acquisition failed.
    Bus(BusE),
    /// Rotor-sensor acquisition failed.
    Rotor(RotorE),
    /// The current sample was explicitly marked invalid for control use.
    InvalidCurrentSample,
}

impl<PwmE, CurrentE, BusE, RotorE> fmt::Display for MotorSystemError<PwmE, CurrentE, BusE, RotorE>
where
    PwmE: fmt::Display,
    CurrentE: fmt::Display,
    BusE: fmt::Display,
    RotorE: fmt::Display,
{
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Pwm(error) => write!(f, "pwm error: {error}"),
            Self::Current(error) => write!(f, "current-sensor error: {error}"),
            Self::Bus(error) => write!(f, "bus-voltage error: {error}"),
            Self::Rotor(error) => write!(f, "rotor-sensor error: {error}"),
            Self::InvalidCurrentSample => f.write_str("invalid current sample"),
        }
    }
}

impl<PwmE, CurrentE, BusE, RotorE> core::error::Error
    for MotorSystemError<PwmE, CurrentE, BusE, RotorE>
where
    PwmE: core::error::Error + 'static,
    CurrentE: core::error::Error + 'static,
    BusE: core::error::Error + 'static,
    RotorE: core::error::Error + 'static,
{
    fn source(&self) -> Option<&(dyn core::error::Error + 'static)> {
        match self {
            Self::Pwm(error) => Some(error),
            Self::Current(error) => Some(error),
            Self::Bus(error) => Some(error),
            Self::Rotor(error) => Some(error),
            Self::InvalidCurrentSample => None,
        }
    }
}

/// Encapsulated synchronous motor stack: hardware plus pure controller.
#[derive(Debug)]
pub struct MotorSystem<PWM, CURRENT, BUS, ROTOR, MOD = Svpwm> {
    hardware: MotorHardware<PWM, CURRENT, BUS, ROTOR>,
    controller: MotorController<MOD>,
}

impl<PWM, CURRENT, BUS, ROTOR> MotorSystem<PWM, CURRENT, BUS, ROTOR, Svpwm> {
    /// Creates a new motor system using the default SVPWM modulator.
    pub fn new(
        hardware: MotorHardware<PWM, CURRENT, BUS, ROTOR>,
        controller: MotorController<Svpwm>,
    ) -> Self {
        Self {
            hardware,
            controller,
        }
    }
}

impl<PWM, CURRENT, BUS, ROTOR, MOD> MotorSystem<PWM, CURRENT, BUS, ROTOR, MOD>
where
    PWM: PhasePwm,
    CURRENT: CurrentSampler,
    BUS: BusVoltageSensor,
    ROTOR: RotorSensor,
    MOD: Modulator,
{
    /// Creates a new motor system with an explicit controller modulator.
    pub fn new_with_controller(
        hardware: MotorHardware<PWM, CURRENT, BUS, ROTOR>,
        controller: MotorController<MOD>,
    ) -> Self {
        Self {
            hardware,
            controller,
        }
    }

    /// Returns shared access to the owned hardware handles.
    #[inline]
    pub const fn hardware(&self) -> &MotorHardware<PWM, CURRENT, BUS, ROTOR> {
        &self.hardware
    }

    /// Returns mutable access to the owned hardware handles.
    #[inline]
    pub fn hardware_mut(&mut self) -> &mut MotorHardware<PWM, CURRENT, BUS, ROTOR> {
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
        MotorHardware<PWM, CURRENT, BUS, ROTOR>,
        MotorController<MOD>,
    ) {
        (self.hardware, self.controller)
    }

    /// Enables the underlying PWM and then enables the controller.
    pub fn enable(
        &mut self,
    ) -> Result<(), MotorSystemError<PWM::Error, CURRENT::Error, BUS::Error, ROTOR::Error>> {
        self.hardware.pwm.enable().map_err(MotorSystemError::Pwm)?;
        self.controller.enable();
        Ok(())
    }

    /// Forces a neutral output, disables the controller, then disables the PWM.
    pub fn disable(
        &mut self,
    ) -> Result<(), MotorSystemError<PWM::Error, CURRENT::Error, BUS::Error, ROTOR::Error>> {
        self.controller.disable();
        self.hardware
            .pwm
            .set_neutral()
            .map_err(MotorSystemError::Pwm)?;
        self.hardware.pwm.disable().map_err(MotorSystemError::Pwm)?;
        Ok(())
    }

    /// Samples hardware, runs the current loop, and applies the resulting duty.
    pub fn fast_tick(
        &mut self,
        dt_seconds: f32,
    ) -> Result<
        FastLoopOutput,
        MotorSystemError<PWM::Error, CURRENT::Error, BUS::Error, ROTOR::Error>,
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

        let output = self.controller.fast_tick(FastLoopInput {
            phase_currents: current.currents,
            bus_voltage,
            rotor: RotorEstimate {
                electrical_angle: rotor.electrical_angle,
                mechanical_angle: rotor.mechanical_angle,
                mechanical_velocity: rotor.mechanical_velocity,
            },
            dt_seconds,
        });

        self.hardware
            .pwm
            .set_phase_duty(output.phase_duty)
            .map_err(MotorSystemError::Pwm)?;

        Ok(output)
    }

    /// Runs the medium-rate controller hook.
    #[inline]
    pub fn medium_tick(&mut self, dt_seconds: f32) {
        self.controller.medium_tick(dt_seconds);
    }

    /// Runs the slow-rate controller hook.
    #[inline]
    pub fn slow_tick(&mut self, dt_seconds: f32) {
        self.controller.slow_tick(dt_seconds);
    }
}

#[cfg(test)]
mod tests {
    use core::convert::Infallible;

    use fluxkit_core::{ControlMode, CurrentLoopConfig, InverterParams, MotorParams, MotorState};
    use fluxkit_hal::{
        BusVoltageSensor, CurrentSampleValidity, CurrentSampler, PhaseCurrentSample, PhasePwm,
        RotorReading, RotorSensor, centered_phase_duty,
    };
    use fluxkit_math::{
        ElectricalAngle, MechanicalAngle,
        frame::Abc,
        units::{Amps, Duty, Henries, Hertz, Ohms, RadPerSec, Volts},
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

    fn motor_params() -> MotorParams {
        MotorParams {
            pole_pairs: 7,
            phase_resistance_ohm: Ohms::new(0.08),
            d_inductance_h: Henries::new(0.00012),
            q_inductance_h: Henries::new(0.00012),
            flux_linkage_weber: None,
            max_phase_current: Amps::new(20.0),
            max_mech_speed: None,
            torque_constant_nm_per_amp: None,
        }
    }

    fn inverter_params() -> InverterParams {
        InverterParams {
            pwm_frequency_hz: Hertz::new(20_000.0),
            deadtime_ns: 250,
            min_duty: Duty::new(0.0),
            max_duty: Duty::new(1.0),
            min_bus_voltage: Volts::new(6.0),
            max_bus_voltage: Volts::new(60.0),
            max_voltage_command: Volts::new(24.0),
        }
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
            enable_current_feedforward: true,
        }
    }

    fn hardware(
        validity: CurrentSampleValidity,
    ) -> MotorHardware<FakePwm, FakeCurrentSensor, FakeBusSensor, FakeRotor> {
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
                    electrical_angle: ElectricalAngle::new(0.0),
                    mechanical_angle: MechanicalAngle::new(0.0),
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
            current_loop_config(),
        );
        let mut system = MotorSystem::new(hardware(CurrentSampleValidity::Valid), controller);

        system.enable().unwrap();
        system.controller_mut().set_mode(ControlMode::Current);
        system.controller_mut().set_iq_target(Amps::new(2.0));

        let output = system.fast_tick(0.000_05).unwrap();

        assert!(output.error.is_none());
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
            current_loop_config(),
        );
        let mut system = MotorSystem::new(hardware(CurrentSampleValidity::Invalid), controller);
        system.hardware_mut().pwm.duty = Abc::new(Duty::new(0.2), Duty::new(0.7), Duty::new(0.6));

        let error = system.fast_tick(0.000_05).unwrap_err();

        assert!(matches!(error, MotorSystemError::InvalidCurrentSample));
        assert_eq!(system.hardware().pwm.duty, centered_phase_duty());
    }
}
