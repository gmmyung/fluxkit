//! Generic synchronous HAL wrappers for calibration procedures.
//!
//! `fluxkit_core` owns the pure calibration state machines. This module owns
//! the generic glue needed to:
//!
//! - sample HAL sensors
//! - feed those samples into the pure calibrators
//! - apply the returned drive requests through PWM or `MotorSystem`
//! - neutral or disable the drive path on completion and failure
//!
//! Recommended bring-up order:
//!
//! 1. `MotorCalibrationSystem`
//!    - pole pairs + electrical offset
//!    - phase resistance
//!    - phase inductance
//!    - flux linkage
//! 2. `ActuatorCalibrationSystem`
//!    - Coulomb + viscous friction
//!    - breakaway torque
//!    - zero-velocity blend band

use core::fmt;

use fluxkit_core::{
    ActuatorBlendBandCalibrationInput, ActuatorBlendBandCalibrationResult,
    ActuatorBlendBandCalibrator, ActuatorBreakawayCalibrationInput,
    ActuatorBreakawayCalibrationResult, ActuatorBreakawayCalibrator,
    ActuatorFrictionCalibrationInput, ActuatorFrictionCalibrationResult,
    ActuatorFrictionCalibrator, CalibrationError, ControlMode, FluxLinkageCalibrationInput,
    FluxLinkageCalibrationResult, FluxLinkageCalibrator, PhaseInductanceCalibrationInput,
    PhaseInductanceCalibrationResult, PhaseInductanceCalibrator, PhaseResistanceCalibrationInput,
    PhaseResistanceCalibrationResult, PhaseResistanceCalibrator,
    PolePairsAndOffsetCalibrationInput, PolePairsAndOffsetCalibrationResult,
    PolePairsAndOffsetCalibrator, TickSchedule,
};
use fluxkit_hal::{
    BusVoltageSensor, CurrentSampleValidity, CurrentSampler, OutputSensor, PhasePwm, RotorSensor,
};
use fluxkit_math::{AlphaBeta, Modulator, Svpwm, Volts};

use crate::{MotorSystem, MotorSystemError};

/// Concrete hardware handles required to run motor-side calibration procedures.
#[derive(Debug)]
pub struct MotorCalibrationHardware<PWM, CURRENT, BUS, ROTOR> {
    /// Three-phase PWM output stage.
    pub pwm: PWM,
    /// Phase-current acquisition path.
    pub current: CURRENT,
    /// DC bus-voltage acquisition path.
    pub bus: BUS,
    /// Absolute-encoder rotor sensing path.
    pub rotor: ROTOR,
}

/// Outcome of one calibration tick.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CalibrationTickResult<R> {
    /// Procedure is still running.
    Running,
    /// Procedure completed on this or a prior tick.
    Complete(R),
}

/// HAL and integration failures that can occur outside the pure calibration procedures.
#[derive(Debug)]
pub enum MotorCalibrationSystemError<PwmE, CurrentE, BusE, RotorE> {
    /// PWM output operation failed.
    Pwm(PwmE),
    /// Phase-current acquisition failed.
    Current(CurrentE),
    /// DC bus-voltage acquisition failed.
    Bus(BusE),
    /// Rotor-sensor acquisition failed.
    Rotor(RotorE),
    /// The current sample was explicitly marked invalid for calibration use.
    InvalidCurrentSample,
    /// The pure core calibration procedure failed.
    Calibration(CalibrationError),
}

impl<PwmE, CurrentE, BusE, RotorE> fmt::Display
    for MotorCalibrationSystemError<PwmE, CurrentE, BusE, RotorE>
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
            Self::Calibration(error) => write!(f, "calibration error: {error}"),
        }
    }
}

impl<PwmE, CurrentE, BusE, RotorE> core::error::Error
    for MotorCalibrationSystemError<PwmE, CurrentE, BusE, RotorE>
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
            Self::Calibration(error) => Some(error),
            Self::InvalidCurrentSample => None,
        }
    }
}

/// Encapsulated synchronous calibration stack: hardware plus a modulation strategy.
#[derive(Debug)]
pub struct MotorCalibrationSystem<PWM, CURRENT, BUS, ROTOR, MOD = Svpwm> {
    hardware: MotorCalibrationHardware<PWM, CURRENT, BUS, ROTOR>,
    modulator: MOD,
}

impl<PWM, CURRENT, BUS, ROTOR> MotorCalibrationSystem<PWM, CURRENT, BUS, ROTOR, Svpwm> {
    /// Creates a new motor-calibration system using the default SVPWM modulator.
    pub fn new(hardware: MotorCalibrationHardware<PWM, CURRENT, BUS, ROTOR>) -> Self {
        Self {
            hardware,
            modulator: Svpwm,
        }
    }
}

/// HAL and integration failures that can occur while running actuator-side
/// calibration through the full public motor-system wrapper.
#[derive(Debug)]
pub enum ActuatorCalibrationSystemError<PwmE, CurrentE, BusE, RotorE, OutputE> {
    /// Underlying motor-system operation failed.
    Motor(MotorSystemError<PwmE, CurrentE, BusE, RotorE, OutputE>),
    /// The pure core calibration procedure failed.
    Calibration(CalibrationError),
}

impl<PwmE, CurrentE, BusE, RotorE, OutputE> fmt::Display
    for ActuatorCalibrationSystemError<PwmE, CurrentE, BusE, RotorE, OutputE>
where
    PwmE: fmt::Display,
    CurrentE: fmt::Display,
    BusE: fmt::Display,
    RotorE: fmt::Display,
    OutputE: fmt::Display,
{
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Motor(error) => write!(f, "motor-system error: {error}"),
            Self::Calibration(error) => write!(f, "calibration error: {error}"),
        }
    }
}

impl<PwmE, CurrentE, BusE, RotorE, OutputE> core::error::Error
    for ActuatorCalibrationSystemError<PwmE, CurrentE, BusE, RotorE, OutputE>
where
    PwmE: core::error::Error + 'static,
    CurrentE: core::error::Error + 'static,
    BusE: core::error::Error + 'static,
    RotorE: core::error::Error + 'static,
    OutputE: core::error::Error + 'static,
{
    fn source(&self) -> Option<&(dyn core::error::Error + 'static)> {
        match self {
            Self::Motor(error) => Some(error),
            Self::Calibration(error) => Some(error),
        }
    }
}

/// Encapsulated synchronous actuator-calibration stack built on the public
/// `MotorSystem` wrapper.
#[derive(Debug)]
pub struct ActuatorCalibrationSystem<PWM, CURRENT, BUS, ROTOR, OUTPUT, MOD = Svpwm> {
    motor_system: MotorSystem<PWM, CURRENT, BUS, ROTOR, OUTPUT, MOD>,
}

impl<PWM, CURRENT, BUS, ROTOR, OUTPUT, MOD>
    ActuatorCalibrationSystem<PWM, CURRENT, BUS, ROTOR, OUTPUT, MOD>
where
    PWM: PhasePwm,
    CURRENT: CurrentSampler,
    BUS: BusVoltageSensor,
    ROTOR: RotorSensor,
    OUTPUT: OutputSensor,
    MOD: Modulator,
{
    /// Creates a new actuator-calibration system from an owned motor system.
    pub fn new(motor_system: MotorSystem<PWM, CURRENT, BUS, ROTOR, OUTPUT, MOD>) -> Self {
        Self { motor_system }
    }

    /// Returns shared access to the owned motor system.
    #[inline]
    pub const fn motor_system(&self) -> &MotorSystem<PWM, CURRENT, BUS, ROTOR, OUTPUT, MOD> {
        &self.motor_system
    }

    /// Returns mutable access to the owned motor system.
    #[inline]
    pub fn motor_system_mut(&mut self) -> &mut MotorSystem<PWM, CURRENT, BUS, ROTOR, OUTPUT, MOD> {
        &mut self.motor_system
    }

    /// Splits the actuator-calibration system back into the owned motor system.
    #[inline]
    pub fn into_motor_system(self) -> MotorSystem<PWM, CURRENT, BUS, ROTOR, OUTPUT, MOD> {
        self.motor_system
    }

    /// Runs one actuator-friction calibration tick through the public motor
    /// system.
    ///
    /// The wrapped controller must have friction compensation disabled so the
    /// sampled torque command reflects the plant friction rather than an
    /// already-compensated request.
    pub fn tick_friction(
        &mut self,
        calibrator: &mut ActuatorFrictionCalibrator,
        dt_seconds: f32,
        schedule: TickSchedule,
    ) -> Result<
        CalibrationTickResult<ActuatorFrictionCalibrationResult>,
        ActuatorCalibrationSystemError<
            PWM::Error,
            CURRENT::Error,
            BUS::Error,
            ROTOR::Error,
            OUTPUT::Error,
        >,
    > {
        if let Some(result) = calibrator.result() {
            self.disable_motor()?;
            return Ok(CalibrationTickResult::Complete(result));
        }
        if let Some(error) = calibrator.error() {
            self.disable_motor()?;
            return Err(ActuatorCalibrationSystemError::Calibration(error));
        }
        if self
            .motor_system
            .controller()
            .actuator_params()
            .compensation
            .friction
            .enabled
        {
            self.disable_motor()?;
            return Err(ActuatorCalibrationSystemError::Calibration(
                CalibrationError::InvalidConfiguration,
            ));
        }

        self.motor_system
            .controller_mut()
            .set_mode(ControlMode::Velocity);
        if self.motor_system.controller().status().state == fluxkit_core::MotorState::Disabled {
            self.motor_system
                .enable()
                .map_err(ActuatorCalibrationSystemError::Motor)?;
        }

        let status = self.motor_system.controller().status();
        let command = calibrator.tick(ActuatorFrictionCalibrationInput {
            output_velocity: status.last_output_mechanical_velocity,
            output_torque_command: status
                .last_actuator_compensation
                .total_output_torque_command,
            dt_seconds,
        });
        self.motor_system
            .controller_mut()
            .set_velocity_target(command.velocity_target);
        let _ = self
            .motor_system
            .tick(dt_seconds, schedule)
            .map_err(ActuatorCalibrationSystemError::Motor)?;

        if let Some(result) = calibrator.result() {
            self.disable_motor()?;
            Ok(CalibrationTickResult::Complete(result))
        } else if let Some(error) = calibrator.error() {
            self.disable_motor()?;
            Err(ActuatorCalibrationSystemError::Calibration(error))
        } else {
            Ok(CalibrationTickResult::Running)
        }
    }

    /// Runs one actuator-breakaway calibration tick through the public motor
    /// system.
    ///
    /// The wrapped controller must have friction compensation disabled so the
    /// measured release torque reflects the plant deadzone rather than an
    /// already-compensated request.
    pub fn tick_breakaway(
        &mut self,
        calibrator: &mut ActuatorBreakawayCalibrator,
        dt_seconds: f32,
        schedule: TickSchedule,
    ) -> Result<
        CalibrationTickResult<ActuatorBreakawayCalibrationResult>,
        ActuatorCalibrationSystemError<
            PWM::Error,
            CURRENT::Error,
            BUS::Error,
            ROTOR::Error,
            OUTPUT::Error,
        >,
    > {
        if let Some(result) = calibrator.result() {
            self.disable_motor()?;
            return Ok(CalibrationTickResult::Complete(result));
        }
        if let Some(error) = calibrator.error() {
            self.disable_motor()?;
            return Err(ActuatorCalibrationSystemError::Calibration(error));
        }
        if self
            .motor_system
            .controller()
            .actuator_params()
            .compensation
            .friction
            .enabled
        {
            self.disable_motor()?;
            return Err(ActuatorCalibrationSystemError::Calibration(
                CalibrationError::InvalidConfiguration,
            ));
        }

        self.motor_system
            .controller_mut()
            .set_mode(ControlMode::Torque);
        if self.motor_system.controller().status().state == fluxkit_core::MotorState::Disabled {
            self.motor_system
                .enable()
                .map_err(ActuatorCalibrationSystemError::Motor)?;
        }

        let status = self.motor_system.controller().status();
        let command = calibrator.tick(ActuatorBreakawayCalibrationInput {
            output_velocity: status.last_output_mechanical_velocity,
            output_torque_command: status
                .last_actuator_compensation
                .total_output_torque_command,
            dt_seconds,
        });
        self.motor_system
            .controller_mut()
            .set_torque_target(command.torque_target);
        let _ = self
            .motor_system
            .tick(dt_seconds, schedule)
            .map_err(ActuatorCalibrationSystemError::Motor)?;

        if let Some(result) = calibrator.result() {
            self.disable_motor()?;
            Ok(CalibrationTickResult::Complete(result))
        } else if let Some(error) = calibrator.error() {
            self.disable_motor()?;
            Err(ActuatorCalibrationSystemError::Calibration(error))
        } else {
            Ok(CalibrationTickResult::Running)
        }
    }

    /// Runs one zero-velocity blend-band calibration tick through the public
    /// motor system.
    ///
    /// The wrapped controller must have friction compensation disabled so the
    /// measured release speed reflects the plant transition out of the
    /// deadzone rather than an already-compensated request.
    pub fn tick_blend_band(
        &mut self,
        calibrator: &mut ActuatorBlendBandCalibrator,
        dt_seconds: f32,
        schedule: TickSchedule,
    ) -> Result<
        CalibrationTickResult<ActuatorBlendBandCalibrationResult>,
        ActuatorCalibrationSystemError<
            PWM::Error,
            CURRENT::Error,
            BUS::Error,
            ROTOR::Error,
            OUTPUT::Error,
        >,
    > {
        if let Some(result) = calibrator.result() {
            self.disable_motor()?;
            return Ok(CalibrationTickResult::Complete(result));
        }
        if let Some(error) = calibrator.error() {
            self.disable_motor()?;
            return Err(ActuatorCalibrationSystemError::Calibration(error));
        }
        if self
            .motor_system
            .controller()
            .actuator_params()
            .compensation
            .friction
            .enabled
        {
            self.disable_motor()?;
            return Err(ActuatorCalibrationSystemError::Calibration(
                CalibrationError::InvalidConfiguration,
            ));
        }

        self.motor_system
            .controller_mut()
            .set_mode(ControlMode::Torque);
        if self.motor_system.controller().status().state == fluxkit_core::MotorState::Disabled {
            self.motor_system
                .enable()
                .map_err(ActuatorCalibrationSystemError::Motor)?;
        }

        let status = self.motor_system.controller().status();
        let command = calibrator.tick(ActuatorBlendBandCalibrationInput {
            output_velocity: status.last_output_mechanical_velocity,
            output_torque_command: status
                .last_actuator_compensation
                .total_output_torque_command,
            dt_seconds,
        });
        self.motor_system
            .controller_mut()
            .set_torque_target(command.torque_target);
        let _ = self
            .motor_system
            .tick(dt_seconds, schedule)
            .map_err(ActuatorCalibrationSystemError::Motor)?;

        if let Some(result) = calibrator.result() {
            self.disable_motor()?;
            Ok(CalibrationTickResult::Complete(result))
        } else if let Some(error) = calibrator.error() {
            self.disable_motor()?;
            Err(ActuatorCalibrationSystemError::Calibration(error))
        } else {
            Ok(CalibrationTickResult::Running)
        }
    }

    fn disable_motor(
        &mut self,
    ) -> Result<
        (),
        ActuatorCalibrationSystemError<
            PWM::Error,
            CURRENT::Error,
            BUS::Error,
            ROTOR::Error,
            OUTPUT::Error,
        >,
    > {
        self.motor_system
            .disable()
            .map_err(ActuatorCalibrationSystemError::Motor)
    }
}

impl<PWM, CURRENT, BUS, ROTOR, MOD> MotorCalibrationSystem<PWM, CURRENT, BUS, ROTOR, MOD>
where
    PWM: PhasePwm,
    CURRENT: CurrentSampler,
    BUS: BusVoltageSensor,
    ROTOR: RotorSensor,
    MOD: Modulator,
{
    /// Creates a new motor-calibration system with an explicit modulator.
    pub fn new_with_modulator(
        hardware: MotorCalibrationHardware<PWM, CURRENT, BUS, ROTOR>,
        modulator: MOD,
    ) -> Self {
        Self {
            hardware,
            modulator,
        }
    }

    /// Returns shared access to the owned hardware handles.
    #[inline]
    pub const fn hardware(&self) -> &MotorCalibrationHardware<PWM, CURRENT, BUS, ROTOR> {
        &self.hardware
    }

    /// Returns mutable access to the owned hardware handles.
    #[inline]
    pub fn hardware_mut(&mut self) -> &mut MotorCalibrationHardware<PWM, CURRENT, BUS, ROTOR> {
        &mut self.hardware
    }

    /// Splits the system back into owned hardware and modulator parts.
    #[inline]
    pub fn into_parts(self) -> (MotorCalibrationHardware<PWM, CURRENT, BUS, ROTOR>, MOD) {
        (self.hardware, self.modulator)
    }

    /// Drives neutral PWM immediately.
    pub fn set_neutral(
        &mut self,
    ) -> Result<(), MotorCalibrationSystemError<PWM::Error, CURRENT::Error, BUS::Error, ROTOR::Error>>
    {
        self.hardware
            .pwm
            .set_neutral()
            .map_err(MotorCalibrationSystemError::Pwm)
    }

    /// Runs one slow-sweep pole-pair and electrical-offset calibration tick.
    pub fn tick_pole_pairs_and_offset(
        &mut self,
        calibrator: &mut PolePairsAndOffsetCalibrator,
        dt_seconds: f32,
    ) -> Result<
        CalibrationTickResult<PolePairsAndOffsetCalibrationResult>,
        MotorCalibrationSystemError<PWM::Error, CURRENT::Error, BUS::Error, ROTOR::Error>,
    > {
        if let Some(result) = calibrator.result() {
            self.set_neutral()?;
            return Ok(CalibrationTickResult::Complete(result));
        }
        if let Some(error) = calibrator.error() {
            self.set_neutral()?;
            return Err(MotorCalibrationSystemError::Calibration(error));
        }

        let bus_voltage = self
            .hardware
            .bus
            .sample_bus_voltage()
            .map_err(MotorCalibrationSystemError::Bus)?;
        let rotor = self
            .hardware
            .rotor
            .read_rotor()
            .map_err(MotorCalibrationSystemError::Rotor)?;

        let command = calibrator.tick(PolePairsAndOffsetCalibrationInput {
            mechanical_angle: rotor.mechanical_angle,
            mechanical_velocity: rotor.mechanical_velocity,
            dt_seconds,
        });
        self.apply_alpha_beta_command(command, bus_voltage)?;
        Self::pole_pairs_result_from_state(calibrator)
    }

    /// Runs one phase-resistance calibration tick through the HAL.
    pub fn tick_phase_resistance(
        &mut self,
        calibrator: &mut PhaseResistanceCalibrator,
        dt_seconds: f32,
    ) -> Result<
        CalibrationTickResult<PhaseResistanceCalibrationResult>,
        MotorCalibrationSystemError<PWM::Error, CURRENT::Error, BUS::Error, ROTOR::Error>,
    > {
        if let Some(result) = calibrator.result() {
            self.set_neutral()?;
            return Ok(CalibrationTickResult::Complete(result));
        }
        if let Some(error) = calibrator.error() {
            self.set_neutral()?;
            return Err(MotorCalibrationSystemError::Calibration(error));
        }

        let bus_voltage = self
            .hardware
            .bus
            .sample_bus_voltage()
            .map_err(MotorCalibrationSystemError::Bus)?;
        let rotor = self
            .hardware
            .rotor
            .read_rotor()
            .map_err(MotorCalibrationSystemError::Rotor)?;
        let current = self
            .hardware
            .current
            .sample_phase_currents()
            .map_err(MotorCalibrationSystemError::Current)?;

        if current.validity == CurrentSampleValidity::Invalid {
            self.set_neutral()?;
            return Err(MotorCalibrationSystemError::InvalidCurrentSample);
        }

        let command = calibrator.tick(PhaseResistanceCalibrationInput {
            phase_currents: current.currents,
            mechanical_velocity: rotor.mechanical_velocity,
            dt_seconds,
        });
        self.apply_alpha_beta_command(command, bus_voltage)?;
        Self::phase_resistance_result_from_state(calibrator)
    }

    /// Runs one phase-inductance calibration tick through the HAL.
    pub fn tick_phase_inductance(
        &mut self,
        calibrator: &mut PhaseInductanceCalibrator,
        dt_seconds: f32,
    ) -> Result<
        CalibrationTickResult<PhaseInductanceCalibrationResult>,
        MotorCalibrationSystemError<PWM::Error, CURRENT::Error, BUS::Error, ROTOR::Error>,
    > {
        if let Some(result) = calibrator.result() {
            self.set_neutral()?;
            return Ok(CalibrationTickResult::Complete(result));
        }
        if let Some(error) = calibrator.error() {
            self.set_neutral()?;
            return Err(MotorCalibrationSystemError::Calibration(error));
        }

        let bus_voltage = self
            .hardware
            .bus
            .sample_bus_voltage()
            .map_err(MotorCalibrationSystemError::Bus)?;
        let rotor = self
            .hardware
            .rotor
            .read_rotor()
            .map_err(MotorCalibrationSystemError::Rotor)?;
        let current = self
            .hardware
            .current
            .sample_phase_currents()
            .map_err(MotorCalibrationSystemError::Current)?;

        if current.validity == CurrentSampleValidity::Invalid {
            self.set_neutral()?;
            return Err(MotorCalibrationSystemError::InvalidCurrentSample);
        }

        let command = calibrator.tick(PhaseInductanceCalibrationInput {
            phase_currents: current.currents,
            mechanical_velocity: rotor.mechanical_velocity,
            dt_seconds,
        });
        self.apply_alpha_beta_command(command, bus_voltage)?;
        Self::phase_inductance_result_from_state(calibrator)
    }

    /// Runs one flux-linkage calibration tick through the HAL.
    pub fn tick_flux_linkage(
        &mut self,
        calibrator: &mut FluxLinkageCalibrator,
        dt_seconds: f32,
    ) -> Result<
        CalibrationTickResult<FluxLinkageCalibrationResult>,
        MotorCalibrationSystemError<PWM::Error, CURRENT::Error, BUS::Error, ROTOR::Error>,
    > {
        if let Some(result) = calibrator.result() {
            self.set_neutral()?;
            return Ok(CalibrationTickResult::Complete(result));
        }
        if let Some(error) = calibrator.error() {
            self.set_neutral()?;
            return Err(MotorCalibrationSystemError::Calibration(error));
        }

        let bus_voltage = self
            .hardware
            .bus
            .sample_bus_voltage()
            .map_err(MotorCalibrationSystemError::Bus)?;
        let rotor = self
            .hardware
            .rotor
            .read_rotor()
            .map_err(MotorCalibrationSystemError::Rotor)?;
        let current = self
            .hardware
            .current
            .sample_phase_currents()
            .map_err(MotorCalibrationSystemError::Current)?;

        if current.validity == CurrentSampleValidity::Invalid {
            self.set_neutral()?;
            return Err(MotorCalibrationSystemError::InvalidCurrentSample);
        }

        let command = calibrator.tick(FluxLinkageCalibrationInput {
            phase_currents: current.currents,
            mechanical_angle: rotor.mechanical_angle,
            mechanical_velocity: rotor.mechanical_velocity,
            dt_seconds,
        });
        self.apply_alpha_beta_command(command, bus_voltage)?;
        Self::flux_linkage_result_from_state(calibrator)
    }

    fn apply_alpha_beta_command(
        &mut self,
        command: AlphaBeta<Volts>,
        bus_voltage: Volts,
    ) -> Result<(), MotorCalibrationSystemError<PWM::Error, CURRENT::Error, BUS::Error, ROTOR::Error>>
    {
        if command == AlphaBeta::new(Volts::ZERO, Volts::ZERO) {
            return self.set_neutral();
        }

        let modulation = self
            .modulator
            .modulate(command.map(|volts| volts.get()), bus_voltage);
        self.hardware
            .pwm
            .set_phase_duty(modulation.duty)
            .map_err(MotorCalibrationSystemError::Pwm)
    }

    fn pole_pairs_result_from_state(
        calibrator: &PolePairsAndOffsetCalibrator,
    ) -> Result<
        CalibrationTickResult<PolePairsAndOffsetCalibrationResult>,
        MotorCalibrationSystemError<PWM::Error, CURRENT::Error, BUS::Error, ROTOR::Error>,
    > {
        if let Some(result) = calibrator.result() {
            Ok(CalibrationTickResult::Complete(result))
        } else if let Some(error) = calibrator.error() {
            Err(MotorCalibrationSystemError::Calibration(error))
        } else {
            Ok(CalibrationTickResult::Running)
        }
    }

    fn phase_resistance_result_from_state(
        calibrator: &PhaseResistanceCalibrator,
    ) -> Result<
        CalibrationTickResult<PhaseResistanceCalibrationResult>,
        MotorCalibrationSystemError<PWM::Error, CURRENT::Error, BUS::Error, ROTOR::Error>,
    > {
        if let Some(result) = calibrator.result() {
            Ok(CalibrationTickResult::Complete(result))
        } else if let Some(error) = calibrator.error() {
            Err(MotorCalibrationSystemError::Calibration(error))
        } else {
            Ok(CalibrationTickResult::Running)
        }
    }

    fn phase_inductance_result_from_state(
        calibrator: &PhaseInductanceCalibrator,
    ) -> Result<
        CalibrationTickResult<PhaseInductanceCalibrationResult>,
        MotorCalibrationSystemError<PWM::Error, CURRENT::Error, BUS::Error, ROTOR::Error>,
    > {
        if let Some(result) = calibrator.result() {
            Ok(CalibrationTickResult::Complete(result))
        } else if let Some(error) = calibrator.error() {
            Err(MotorCalibrationSystemError::Calibration(error))
        } else {
            Ok(CalibrationTickResult::Running)
        }
    }

    fn flux_linkage_result_from_state(
        calibrator: &FluxLinkageCalibrator,
    ) -> Result<
        CalibrationTickResult<FluxLinkageCalibrationResult>,
        MotorCalibrationSystemError<PWM::Error, CURRENT::Error, BUS::Error, ROTOR::Error>,
    > {
        if let Some(result) = calibrator.result() {
            Ok(CalibrationTickResult::Complete(result))
        } else if let Some(error) = calibrator.error() {
            Err(MotorCalibrationSystemError::Calibration(error))
        } else {
            Ok(CalibrationTickResult::Running)
        }
    }
}

#[cfg(test)]
mod tests {
    use core::convert::Infallible;

    use fluxkit_core::{
        FluxLinkageCalibrationConfig, FluxLinkageCalibrationState, FluxLinkageCalibrator,
        PhaseInductanceCalibrationConfig, PhaseInductanceCalibrationState,
        PhaseInductanceCalibrator, PhaseResistanceCalibrationConfig,
        PhaseResistanceCalibrationState, PhaseResistanceCalibrator,
    };
    use fluxkit_hal::{
        BusVoltageSensor, CurrentSampleValidity, CurrentSampler, PhaseCurrentSample, PhasePwm,
        RotorReading, RotorSensor, centered_phase_duty,
    };
    use fluxkit_math::{
        MechanicalAngle,
        frame::Abc,
        units::{Amps, Duty, RadPerSec, Volts},
    };

    use super::{MotorCalibrationHardware, MotorCalibrationSystem, MotorCalibrationSystemError};

    #[derive(Debug)]
    struct FakePwm {
        duty: Abc<Duty>,
    }

    impl Default for FakePwm {
        fn default() -> Self {
            Self {
                duty: centered_phase_duty(),
            }
        }
    }

    impl PhasePwm for FakePwm {
        type Error = Infallible;

        fn enable(&mut self) -> Result<(), Self::Error> {
            Ok(())
        }
        fn disable(&mut self) -> Result<(), Self::Error> {
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

    fn hardware() -> MotorCalibrationHardware<FakePwm, FakeCurrentSensor, FakeBusSensor, FakeRotor>
    {
        MotorCalibrationHardware {
            pwm: FakePwm::default(),
            current: FakeCurrentSensor {
                sample: PhaseCurrentSample {
                    currents: Abc::new(Amps::new(2.0), Amps::new(-1.0), Amps::new(-1.0)),
                    validity: CurrentSampleValidity::Valid,
                },
            },
            bus: FakeBusSensor {
                voltage: Volts::new(24.0),
            },
            rotor: FakeRotor {
                reading: RotorReading {
                    mechanical_angle: MechanicalAngle::new(0.2),
                    mechanical_velocity: RadPerSec::ZERO,
                },
            },
        }
    }

    #[test]
    fn resistance_wrapper_rejects_invalid_current_sample() {
        let mut hardware = hardware();
        hardware.current.sample.validity = CurrentSampleValidity::Invalid;
        let mut system = MotorCalibrationSystem::new(hardware);
        let mut calibrator = PhaseResistanceCalibrator::new(PhaseResistanceCalibrationConfig {
            settle_time_seconds: 0.01,
            sample_time_seconds: 0.01,
            timeout_seconds: 1.0,
            ..PhaseResistanceCalibrationConfig::default_for_hold()
        })
        .unwrap();

        let result = system.tick_phase_resistance(&mut calibrator, 0.005);
        assert!(matches!(
            result,
            Err(MotorCalibrationSystemError::InvalidCurrentSample)
        ));
        assert_eq!(system.hardware().pwm.duty, centered_phase_duty());
        assert_eq!(
            calibrator.state(),
            PhaseResistanceCalibrationState::Aligning
        );
    }

    #[test]
    fn inductance_wrapper_rejects_invalid_current_sample() {
        let mut hardware = hardware();
        hardware.current.sample.validity = CurrentSampleValidity::Invalid;
        let mut system = MotorCalibrationSystem::new(hardware);
        let mut calibrator = PhaseInductanceCalibrator::new(PhaseInductanceCalibrationConfig {
            phase_resistance_ohm: fluxkit_math::units::Ohms::new(0.12),
            settle_time_seconds: 0.01,
            sample_time_seconds: 200.0e-6,
            timeout_seconds: 1.0,
            ..PhaseInductanceCalibrationConfig::default_for_hold()
        })
        .unwrap();

        let result = system.tick_phase_inductance(&mut calibrator, 0.005);
        assert!(matches!(
            result,
            Err(MotorCalibrationSystemError::InvalidCurrentSample)
        ));
        assert_eq!(system.hardware().pwm.duty, centered_phase_duty());
        assert_eq!(
            calibrator.state(),
            PhaseInductanceCalibrationState::Aligning
        );
    }

    #[test]
    fn flux_linkage_wrapper_rejects_invalid_current_sample() {
        let mut hardware = hardware();
        hardware.current.sample.validity = CurrentSampleValidity::Invalid;
        let mut system = MotorCalibrationSystem::new(hardware);
        let mut calibrator = FluxLinkageCalibrator::new(FluxLinkageCalibrationConfig {
            phase_resistance_ohm: fluxkit_math::units::Ohms::new(0.12),
            phase_inductance_h: fluxkit_math::units::Henries::new(30.0e-6),
            pole_pairs: 7,
            electrical_angle_offset: fluxkit_math::ElectricalAngle::new(0.0),
            initial_settle_time_seconds: 0.01,
            sample_time_seconds: 0.01,
            timeout_seconds: 1.0,
            ..FluxLinkageCalibrationConfig::default_for_spin()
        })
        .unwrap();

        let result = system.tick_flux_linkage(&mut calibrator, 0.005);
        assert!(matches!(
            result,
            Err(MotorCalibrationSystemError::InvalidCurrentSample)
        ));
        assert_eq!(system.hardware().pwm.duty, centered_phase_duty());
        assert_eq!(calibrator.state(), FluxLinkageCalibrationState::Aligning);
    }
}
