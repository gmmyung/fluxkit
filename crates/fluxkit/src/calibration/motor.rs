use core::fmt;

use fluxkit_core::{
    CalibrationError, FluxLinkageCalibrationInput, FluxLinkageCalibrator,
    MotorCalibration as PartialMotorCalibration, MotorCalibrationRoutine,
    PhaseInductanceCalibrationInput, PhaseInductanceCalibrator, PhaseResistanceCalibrationInput,
    PhaseResistanceCalibrator, PolePairsAndOffsetCalibrationInput, PolePairsAndOffsetCalibrator,
};
use fluxkit_hal::{
    BusVoltageSensor, CurrentSampleValidity, CurrentSampler, PhaseCurrentSample, PhasePwm,
    RotorReading, RotorSensor,
};
use fluxkit_math::{
    AlphaBeta, ElectricalAngle, Modulator, Volts,
    units::{Henries, Ohms, RadPerSec, Webers},
};

use super::shared::RoutineState;

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

/// User intent for the motor-side calibration campaign.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct MotorCalibrationRequest {
    /// Provided electrical pole-pair count.
    ///
    /// This must be supplied together with `electrical_angle_offset` if you
    /// want to skip electrical-mapping calibration.
    pub pole_pairs: Option<u8>,
    /// Provided electrical zero offset after mechanical-to-electrical conversion.
    ///
    /// This must be supplied together with `pole_pairs` if you want to skip
    /// electrical-mapping calibration.
    pub electrical_angle_offset: Option<ElectricalAngle>,
    /// Provided phase resistance. When absent, phase resistance is calibrated.
    pub phase_resistance_ohm: Option<Ohms>,
    /// Provided common phase inductance. When absent, phase inductance is calibrated.
    pub phase_inductance_h: Option<Henries>,
    /// Provided flux linkage. When absent, flux linkage is calibrated.
    pub flux_linkage_weber: Option<Webers>,
}

/// User-facing operating limits for the motor-side calibration campaign.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct MotorCalibrationLimits {
    /// Maximum fixed alignment voltage used by hold-based routines.
    pub max_align_voltage_mag: Volts,
    /// Maximum spinning voltage magnitude used by flux-linkage calibration.
    pub max_spin_voltage_mag: Volts,
    /// Maximum electrical angular velocity used by sweep/spin routines.
    pub max_electrical_velocity: RadPerSec,
    /// Absolute timeout cap applied to each motor-side routine.
    pub timeout_seconds: f32,
}

/// Fully resolved motor-side calibration result returned by the request-driven
/// campaign API.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct MotorCalibrationResult {
    /// Estimated electrical pole-pair count.
    pub pole_pairs: u8,
    /// Electrical zero offset after mechanical-to-electrical conversion.
    pub electrical_angle_offset: ElectricalAngle,
    /// Estimated phase resistance.
    pub phase_resistance_ohm: Ohms,
    /// Estimated common phase inductance applied to both `d` and `q` axes.
    pub phase_inductance_h: Henries,
    /// Estimated flux linkage.
    pub flux_linkage_weber: Webers,
}

impl MotorCalibrationResult {
    /// Builds motor parameters directly from this resolved calibration plus
    /// independent operating limits.
    #[inline]
    pub const fn into_motor_params(
        self,
        limits: fluxkit_core::MotorLimits,
    ) -> fluxkit_core::MotorParams {
        fluxkit_core::MotorParams::from_model_and_limits(
            fluxkit_core::MotorModel {
                pole_pairs: self.pole_pairs,
                phase_resistance_ohm: self.phase_resistance_ohm,
                d_inductance_h: self.phase_inductance_h,
                q_inductance_h: self.phase_inductance_h,
                flux_linkage_weber: self.flux_linkage_weber,
                electrical_angle_offset: self.electrical_angle_offset,
            },
            limits,
        )
    }

    /// Applies this resolved record onto an existing motor-parameter record.
    #[inline]
    pub fn apply_to_motor_params(&self, motor: &mut fluxkit_core::MotorParams) {
        motor.pole_pairs = self.pole_pairs;
        motor.electrical_angle_offset = self.electrical_angle_offset;
        motor.phase_resistance_ohm = self.phase_resistance_ohm;
        motor.d_inductance_h = self.phase_inductance_h;
        motor.q_inductance_h = self.phase_inductance_h;
        motor.flux_linkage_weber = self.flux_linkage_weber;
    }
}

/// Encapsulated synchronous calibration stack: hardware plus a modulation strategy.
#[derive(Debug)]
pub struct MotorCalibrationSystem<PWM, CURRENT, BUS, ROTOR, MOD> {
    pwm: PWM,
    current: CURRENT,
    bus: BUS,
    rotor: ROTOR,
    modulator: MOD,
    limits: MotorCalibrationLimits,
    pole_pairs: Option<u8>,
    electrical_angle_offset: Option<ElectricalAngle>,
    phase_resistance_ohm: Option<Ohms>,
    phase_inductance_h: Option<Henries>,
    flux_linkage_weber: Option<Webers>,
    active_routine: Option<MotorCalibrationRoutine>,
}

impl<PWM, CURRENT, BUS, ROTOR, MOD> MotorCalibrationSystem<PWM, CURRENT, BUS, ROTOR, MOD>
where
    PWM: PhasePwm,
    CURRENT: CurrentSampler,
    BUS: BusVoltageSensor,
    ROTOR: RotorSensor,
    MOD: Modulator,
{
    /// Creates a new request-driven motor-calibration system.
    pub fn new(
        pwm: PWM,
        current: CURRENT,
        bus: BUS,
        rotor: ROTOR,
        modulator: MOD,
        request: MotorCalibrationRequest,
        limits: MotorCalibrationLimits,
    ) -> Result<Self, CalibrationError> {
        if !validate_limits(limits) || !validate_request(request) {
            return Err(CalibrationError::InvalidConfiguration);
        }

        Ok(Self {
            pwm,
            current,
            bus,
            rotor,
            modulator,
            limits,
            pole_pairs: request.pole_pairs,
            electrical_angle_offset: request.electrical_angle_offset,
            phase_resistance_ohm: request.phase_resistance_ohm,
            phase_inductance_h: request.phase_inductance_h,
            flux_linkage_weber: request.flux_linkage_weber,
            active_routine: None,
        })
    }

    /// Returns shared access to the owned PWM handle.
    #[inline]
    pub const fn pwm(&self) -> &PWM {
        &self.pwm
    }

    /// Returns mutable access to the owned PWM handle.
    #[inline]
    pub fn pwm_mut(&mut self) -> &mut PWM {
        &mut self.pwm
    }

    /// Returns shared access to the owned current-sampler handle.
    #[inline]
    pub const fn current(&self) -> &CURRENT {
        &self.current
    }

    /// Returns mutable access to the owned current-sampler handle.
    #[inline]
    pub fn current_mut(&mut self) -> &mut CURRENT {
        &mut self.current
    }

    /// Returns shared access to the owned bus-sensor handle.
    #[inline]
    pub const fn bus(&self) -> &BUS {
        &self.bus
    }

    /// Returns mutable access to the owned bus-sensor handle.
    #[inline]
    pub fn bus_mut(&mut self) -> &mut BUS {
        &mut self.bus
    }

    /// Returns shared access to the owned rotor-sensor handle.
    #[inline]
    pub const fn rotor(&self) -> &ROTOR {
        &self.rotor
    }

    /// Returns mutable access to the owned rotor-sensor handle.
    #[inline]
    pub fn rotor_mut(&mut self) -> &mut ROTOR {
        &mut self.rotor
    }

    /// Splits the system back into owned parts.
    #[inline]
    pub fn into_parts(self) -> (PWM, CURRENT, BUS, ROTOR, MOD) {
        (self.pwm, self.current, self.bus, self.rotor, self.modulator)
    }

    /// Drives neutral PWM immediately.
    pub fn set_neutral(
        &mut self,
    ) -> Result<(), MotorCalibrationSystemError<PWM::Error, CURRENT::Error, BUS::Error, ROTOR::Error>>
    {
        self.pwm
            .set_neutral()
            .map_err(MotorCalibrationSystemError::Pwm)
    }

    /// Advances the request-driven motor calibration campaign.
    pub fn tick(
        &mut self,
        dt_seconds: f32,
    ) -> Result<
        Option<MotorCalibrationResult>,
        MotorCalibrationSystemError<PWM::Error, CURRENT::Error, BUS::Error, ROTOR::Error>,
    > {
        if self.active_routine.is_none() {
            self.active_routine = self
                .build_next_routine()
                .map_err(MotorCalibrationSystemError::Calibration)?;
            if self.active_routine.is_none() {
                return Ok(Some(
                    self.resolve_calibration()
                        .map_err(MotorCalibrationSystemError::Calibration)?,
                ));
            }
        }

        let mut routine = self
            .active_routine
            .take()
            .expect("active routine must exist");
        if let Some(delta) = self.tick_active_routine(&mut routine, dt_seconds)? {
            self.merge_partial(delta);
            if self
                .build_next_routine()
                .map_err(MotorCalibrationSystemError::Calibration)?
                .is_none()
            {
                return Ok(Some(
                    self.resolve_calibration()
                        .map_err(MotorCalibrationSystemError::Calibration)?,
                ));
            }
        } else {
            self.active_routine = Some(routine);
        }

        Ok(None)
    }

    fn tick_active_routine(
        &mut self,
        routine: &mut MotorCalibrationRoutine,
        dt_seconds: f32,
    ) -> Result<
        Option<PartialMotorCalibration>,
        MotorCalibrationSystemError<PWM::Error, CURRENT::Error, BUS::Error, ROTOR::Error>,
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

        if self.pole_pairs.is_none() || self.electrical_angle_offset.is_none() {
            let mut cfg = fluxkit_core::PolePairsAndOffsetCalibrationConfig::default_for_sweep();
            cfg.align_voltage_mag = min_volts(cfg.align_voltage_mag, limits.max_align_voltage_mag);
            cfg.sweep_electrical_velocity = clamp_abs_rad_per_sec(
                cfg.sweep_electrical_velocity,
                limits.max_electrical_velocity,
            );
            cfg.timeout_seconds = cfg.timeout_seconds.min(limits.timeout_seconds);
            return PolePairsAndOffsetCalibrator::new(cfg)
                .map(MotorCalibrationRoutine::PolePairsAndOffset)
                .map(Some);
        }

        if self.phase_resistance_ohm.is_none() {
            let mut cfg = fluxkit_core::PhaseResistanceCalibrationConfig::default_for_hold();
            cfg.align_voltage_mag = min_volts(cfg.align_voltage_mag, limits.max_align_voltage_mag);
            cfg.timeout_seconds = cfg.timeout_seconds.min(limits.timeout_seconds);
            return PhaseResistanceCalibrator::new(cfg)
                .map(MotorCalibrationRoutine::PhaseResistance)
                .map(Some);
        }

        if self.phase_inductance_h.is_none() {
            let mut cfg = fluxkit_core::PhaseInductanceCalibrationConfig::default_for_hold();
            cfg.phase_resistance_ohm = self
                .phase_resistance_ohm
                .expect("phase resistance resolved before inductance");
            cfg.hold_voltage_mag = min_volts(cfg.hold_voltage_mag, limits.max_align_voltage_mag);
            cfg.step_voltage_mag = min_volts(cfg.step_voltage_mag, limits.max_align_voltage_mag);
            cfg.timeout_seconds = cfg.timeout_seconds.min(limits.timeout_seconds);
            return PhaseInductanceCalibrator::new(cfg)
                .map(MotorCalibrationRoutine::PhaseInductance)
                .map(Some);
        }

        if self.flux_linkage_weber.is_none() {
            let mut cfg = fluxkit_core::FluxLinkageCalibrationConfig::default_for_spin();
            cfg.phase_resistance_ohm = self
                .phase_resistance_ohm
                .expect("phase resistance resolved before flux linkage");
            cfg.phase_inductance_h = self
                .phase_inductance_h
                .expect("phase inductance resolved before flux linkage");
            cfg.pole_pairs = self
                .pole_pairs
                .expect("electrical mapping resolved before flux linkage");
            cfg.electrical_angle_offset = self
                .electrical_angle_offset
                .expect("electrical mapping resolved before flux linkage");
            cfg.align_voltage_mag = min_volts(cfg.align_voltage_mag, limits.max_align_voltage_mag);
            cfg.spin_voltage_mag = min_volts(cfg.spin_voltage_mag, limits.max_spin_voltage_mag);
            cfg.spin_electrical_velocity =
                clamp_abs_rad_per_sec(cfg.spin_electrical_velocity, limits.max_electrical_velocity);
            cfg.timeout_seconds = cfg.timeout_seconds.min(limits.timeout_seconds);
            return FluxLinkageCalibrator::new(cfg)
                .map(MotorCalibrationRoutine::FluxLinkage)
                .map(Some);
        }

        Ok(None)
    }

    fn merge_partial(&mut self, delta: PartialMotorCalibration) {
        if self.pole_pairs.is_none() {
            if let Some(value) = delta.pole_pairs {
                self.pole_pairs = Some(value);
            }
        }
        if self.electrical_angle_offset.is_none() {
            if let Some(value) = delta.electrical_angle_offset {
                self.electrical_angle_offset = Some(value);
            }
        }
        if self.phase_resistance_ohm.is_none() {
            if let Some(value) = delta.phase_resistance_ohm {
                self.phase_resistance_ohm = Some(value);
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
            electrical_angle_offset: self
                .electrical_angle_offset
                .ok_or(CalibrationError::InvalidConfiguration)?,
            phase_resistance_ohm: self
                .phase_resistance_ohm
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
        MotorCalibrationSystemError<PWM::Error, CURRENT::Error, BUS::Error, ROTOR::Error>,
    > {
        self.tick_alpha_beta_routine(calibrator, dt_seconds, false, |calibrator, rotor, _, dt| {
            calibrator.tick(PolePairsAndOffsetCalibrationInput {
                mechanical_angle: rotor.mechanical_angle,
                mechanical_velocity: rotor.mechanical_velocity,
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
        MotorCalibrationSystemError<PWM::Error, CURRENT::Error, BUS::Error, ROTOR::Error>,
    > {
        self.tick_alpha_beta_routine(
            calibrator,
            dt_seconds,
            true,
            |calibrator, rotor, current, dt| {
                calibrator.tick(PhaseResistanceCalibrationInput {
                    phase_currents: current.expect("phase current required").currents,
                    mechanical_velocity: rotor.mechanical_velocity,
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
        MotorCalibrationSystemError<PWM::Error, CURRENT::Error, BUS::Error, ROTOR::Error>,
    > {
        self.tick_alpha_beta_routine(
            calibrator,
            dt_seconds,
            true,
            |calibrator, rotor, current, dt| {
                calibrator.tick(PhaseInductanceCalibrationInput {
                    phase_currents: current.expect("phase current required").currents,
                    mechanical_velocity: rotor.mechanical_velocity,
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
        MotorCalibrationSystemError<PWM::Error, CURRENT::Error, BUS::Error, ROTOR::Error>,
    > {
        self.tick_alpha_beta_routine(
            calibrator,
            dt_seconds,
            true,
            |calibrator, rotor, current, dt| {
                calibrator.tick(FluxLinkageCalibrationInput {
                    phase_currents: current.expect("phase current required").currents,
                    mechanical_angle: rotor.mechanical_angle.into(),
                    mechanical_velocity: rotor.mechanical_velocity,
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
        MotorCalibrationSystemError<PWM::Error, CURRENT::Error, BUS::Error, ROTOR::Error>,
    >
    where
        Cal: RoutineState<R>,
        Build: FnOnce(&mut Cal, RotorReading, Option<PhaseCurrentSample>, f32) -> AlphaBeta<Volts>,
    {
        if let Some(result) = self.preflight(calibrator)? {
            return Ok(result);
        }

        let bus_voltage = self
            .bus
            .sample_bus_voltage()
            .map_err(MotorCalibrationSystemError::Bus)?;
        let rotor = self
            .rotor
            .read_rotor()
            .map_err(MotorCalibrationSystemError::Rotor)?;
        let current = if needs_current {
            Some(self.sample_valid_current()?)
        } else {
            None
        };

        let command = build_command(calibrator, rotor, current, dt_seconds);
        self.apply_alpha_beta_command(command, bus_voltage)?;
        self.postflight(calibrator)
    }

    fn sample_valid_current(
        &mut self,
    ) -> Result<
        PhaseCurrentSample,
        MotorCalibrationSystemError<PWM::Error, CURRENT::Error, BUS::Error, ROTOR::Error>,
    > {
        let current = self
            .current
            .sample_phase_currents()
            .map_err(MotorCalibrationSystemError::Current)?;
        if current.validity == CurrentSampleValidity::Invalid {
            self.set_neutral()?;
            return Err(MotorCalibrationSystemError::InvalidCurrentSample);
        }
        Ok(current)
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
        self.pwm
            .set_phase_duty(modulation.duty)
            .map_err(MotorCalibrationSystemError::Pwm)
    }

    fn preflight<R, Cal>(
        &mut self,
        calibrator: &Cal,
    ) -> Result<
        Option<Option<R>>,
        MotorCalibrationSystemError<PWM::Error, CURRENT::Error, BUS::Error, ROTOR::Error>,
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
            return Err(MotorCalibrationSystemError::Calibration(error));
        }
        Ok(None)
    }

    fn postflight<R, Cal>(
        &mut self,
        calibrator: &Cal,
    ) -> Result<
        Option<R>,
        MotorCalibrationSystemError<PWM::Error, CURRENT::Error, BUS::Error, ROTOR::Error>,
    >
    where
        Cal: RoutineState<R>,
    {
        if let Some(result) = calibrator.result() {
            self.set_neutral()?;
            Ok(Some(result))
        } else if let Some(error) = calibrator.error() {
            self.set_neutral()?;
            Err(MotorCalibrationSystemError::Calibration(error))
        } else {
            Ok(None)
        }
    }
}

#[inline]
fn min_volts(a: Volts, b: Volts) -> Volts {
    Volts::new(a.get().min(b.get()))
}

#[inline]
fn clamp_abs_rad_per_sec(value: RadPerSec, limit: RadPerSec) -> RadPerSec {
    let capped = value.get().abs().min(limit.get().abs());
    RadPerSec::new(value.get().signum() * capped)
}

#[inline]
fn validate_limits(limits: MotorCalibrationLimits) -> bool {
    limits.max_align_voltage_mag.get().is_finite()
        && limits.max_align_voltage_mag.get() > 0.0
        && limits.max_spin_voltage_mag.get().is_finite()
        && limits.max_spin_voltage_mag.get() > 0.0
        && limits.max_electrical_velocity.get().is_finite()
        && limits.max_electrical_velocity.get() > 0.0
        && limits.timeout_seconds.is_finite()
        && limits.timeout_seconds > 0.0
}

#[inline]
fn validate_request(request: MotorCalibrationRequest) -> bool {
    request.pole_pairs.is_some() == request.electrical_angle_offset.is_some()
}

#[cfg(test)]
mod tests {
    use core::convert::Infallible;

    use fluxkit_core::{
        FluxLinkageCalibrator, MotorCalibrationRoutine, PhaseInductanceCalibrator,
        PhaseResistanceCalibrator,
        calibration::motor::{
            FluxLinkageCalibrationConfig, PhaseInductanceCalibrationConfig,
            PhaseResistanceCalibrationConfig,
        },
    };
    use fluxkit_hal::{
        BusVoltageSensor, CurrentSampleValidity, CurrentSampler, PhaseCurrentSample, PhasePwm,
        RotorReading, RotorSensor, centered_phase_duty,
    };
    use fluxkit_math::{
        frame::Abc,
        modulation::Svpwm,
        units::{Amps, Duty, RadPerSec, Volts},
    };

    use super::{MotorCalibrationSystem, MotorCalibrationSystemError};

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

    fn hardware() -> (FakePwm, FakeCurrentSensor, FakeBusSensor, FakeRotor) {
        (
            FakePwm::default(),
            FakeCurrentSensor {
                sample: PhaseCurrentSample {
                    currents: Abc::new(Amps::new(2.0), Amps::new(-1.0), Amps::new(-1.0)),
                    validity: CurrentSampleValidity::Valid,
                },
            },
            FakeBusSensor {
                voltage: Volts::new(24.0),
            },
            FakeRotor {
                reading: RotorReading {
                    mechanical_angle: fluxkit_math::MechanicalAngle::new(0.2),
                    mechanical_velocity: RadPerSec::ZERO,
                },
            },
        )
    }

    fn system(
        pwm: FakePwm,
        current: FakeCurrentSensor,
        bus: FakeBusSensor,
        rotor: FakeRotor,
    ) -> MotorCalibrationSystem<FakePwm, FakeCurrentSensor, FakeBusSensor, FakeRotor, Svpwm> {
        MotorCalibrationSystem::new(
            pwm,
            current,
            bus,
            rotor,
            Svpwm,
            super::MotorCalibrationRequest {
                pole_pairs: Some(7),
                electrical_angle_offset: Some(fluxkit_math::ElectricalAngle::new(0.0)),
                phase_resistance_ohm: Some(fluxkit_math::units::Ohms::new(0.12)),
                phase_inductance_h: Some(fluxkit_math::units::Henries::new(30.0e-6)),
                flux_linkage_weber: Some(fluxkit_math::units::Webers::new(0.005)),
            },
            super::MotorCalibrationLimits {
                max_align_voltage_mag: Volts::new(2.0),
                max_spin_voltage_mag: Volts::new(3.0),
                max_electrical_velocity: RadPerSec::new(60.0),
                timeout_seconds: 2.0,
            },
        )
        .unwrap()
    }

    #[test]
    fn resistance_wrapper_rejects_invalid_current_sample() {
        let (pwm, mut current, bus, rotor) = hardware();
        current.sample.validity = CurrentSampleValidity::Invalid;
        let mut system = system(pwm, current, bus, rotor);
        let calibrator = PhaseResistanceCalibrator::new(PhaseResistanceCalibrationConfig {
            settle_time_seconds: 0.01,
            sample_time_seconds: 0.01,
            timeout_seconds: 1.0,
            ..PhaseResistanceCalibrationConfig::default_for_hold()
        })
        .unwrap();

        let mut routine = MotorCalibrationRoutine::PhaseResistance(calibrator);
        let result = system.tick_active_routine(&mut routine, 0.005);
        assert!(matches!(
            result,
            Err(MotorCalibrationSystemError::InvalidCurrentSample)
        ));
        assert_eq!(system.pwm().duty, centered_phase_duty());
        let MotorCalibrationRoutine::PhaseResistance(calibrator) = routine else {
            unreachable!();
        };
        assert_eq!(calibrator.result(), None);
        assert_eq!(calibrator.error(), None);
    }

    #[test]
    fn inductance_wrapper_rejects_invalid_current_sample() {
        let (pwm, mut current, bus, rotor) = hardware();
        current.sample.validity = CurrentSampleValidity::Invalid;
        let mut system = system(pwm, current, bus, rotor);
        let calibrator = PhaseInductanceCalibrator::new(PhaseInductanceCalibrationConfig {
            phase_resistance_ohm: fluxkit_math::units::Ohms::new(0.12),
            settle_time_seconds: 0.01,
            sample_time_seconds: 200.0e-6,
            timeout_seconds: 1.0,
            ..PhaseInductanceCalibrationConfig::default_for_hold()
        })
        .unwrap();

        let mut routine = MotorCalibrationRoutine::PhaseInductance(calibrator);
        let result = system.tick_active_routine(&mut routine, 0.005);
        assert!(matches!(
            result,
            Err(MotorCalibrationSystemError::InvalidCurrentSample)
        ));
        assert_eq!(system.pwm().duty, centered_phase_duty());
        let MotorCalibrationRoutine::PhaseInductance(calibrator) = routine else {
            unreachable!();
        };
        assert_eq!(calibrator.result(), None);
        assert_eq!(calibrator.error(), None);
    }

    #[test]
    fn flux_linkage_wrapper_rejects_invalid_current_sample() {
        let (pwm, mut current, bus, rotor) = hardware();
        current.sample.validity = CurrentSampleValidity::Invalid;
        let mut system = system(pwm, current, bus, rotor);
        let calibrator = FluxLinkageCalibrator::new(FluxLinkageCalibrationConfig {
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

        let mut routine = MotorCalibrationRoutine::FluxLinkage(calibrator);
        let result = system.tick_active_routine(&mut routine, 0.005);
        assert!(matches!(
            result,
            Err(MotorCalibrationSystemError::InvalidCurrentSample)
        ));
        assert_eq!(system.pwm().duty, centered_phase_duty());
        let MotorCalibrationRoutine::FluxLinkage(calibrator) = routine else {
            unreachable!();
        };
        assert_eq!(calibrator.result(), None);
        assert_eq!(calibrator.error(), None);
    }
}
