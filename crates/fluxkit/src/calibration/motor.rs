use core::cell::{Cell, RefCell};
use core::fmt;

use critical_section::Mutex;
use fluxkit_core::{
    CalibrationError, FluxLinkageCalibrationInput, FluxLinkageCalibrator,
    MotorCalibration as PartialMotorCalibration, MotorCalibrationRoutine, MotorLimits, MotorModel,
    MotorParams, PhaseInductanceCalibrationInput, PhaseInductanceCalibrator,
    PhaseResistanceCalibrationInput, PhaseResistanceCalibrator, PolePairsAndOffsetCalibrationInput,
    PolePairsAndOffsetCalibrator,
};
use fluxkit_hal::{
    BusVoltageSensor, CurrentSampleValidity, CurrentSampler, PhaseCurrentSample, PhasePwm,
    RotorSensor, TemperatureSensor,
};
use fluxkit_math::{
    AlphaBeta, ElectricalAngle, ElectricalDirection, MechanicalMotionEstimate,
    MechanicalMotionSample, Modulator, Volts,
    units::{Henries, Ohms, RadPerSec, Webers},
};

use super::shared::{
    RoutineState, SharedStatus, read_status, run_active_calibration_inner, write_status,
};
use crate::CapabilitySplitError;
use crate::capability::split_once;
#[cfg(test)]
use crate::capability::take_active_inner;
use crate::system::MechanicalMotionEstimator;

/// HAL and integration failures that can occur outside the pure calibration procedures.
#[derive(Debug)]
pub enum MotorCalibrationRuntimeError<PwmE, CurrentE, BusE, RotorE, TempE> {
    /// The runtime was temporarily unavailable because another context holds the active inner state.
    Busy,
    /// The runtime owner no longer contains an active inner runtime.
    Inactive,
    /// PWM output operation failed.
    Pwm(PwmE),
    /// Phase-current acquisition failed.
    Current(CurrentE),
    /// DC bus-voltage acquisition failed.
    Bus(BusE),
    /// Rotor-sensor acquisition failed.
    Rotor(RotorE),
    /// Temperature-sensor acquisition failed.
    Temp(TempE),
    /// The current sample was explicitly marked invalid for calibration use.
    InvalidCurrentSample,
    /// The pure core calibration procedure failed.
    Calibration(CalibrationError),
}

impl<PwmE, CurrentE, BusE, RotorE, TempE> fmt::Display
    for MotorCalibrationRuntimeError<PwmE, CurrentE, BusE, RotorE, TempE>
where
    PwmE: fmt::Display,
    CurrentE: fmt::Display,
    BusE: fmt::Display,
    RotorE: fmt::Display,
    TempE: fmt::Display,
{
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Busy => f.write_str("runtime busy"),
            Self::Inactive => f.write_str("runtime inactive"),
            Self::Pwm(error) => write!(f, "pwm error: {error}"),
            Self::Current(error) => write!(f, "current-sensor error: {error}"),
            Self::Bus(error) => write!(f, "bus-voltage error: {error}"),
            Self::Rotor(error) => write!(f, "rotor-sensor error: {error}"),
            Self::Temp(error) => write!(f, "temperature-sensor error: {error}"),
            Self::InvalidCurrentSample => f.write_str("invalid current sample"),
            Self::Calibration(error) => write!(f, "calibration error: {error}"),
        }
    }
}

impl<PwmE, CurrentE, BusE, RotorE, TempE> core::error::Error
    for MotorCalibrationRuntimeError<PwmE, CurrentE, BusE, RotorE, TempE>
where
    PwmE: core::error::Error + 'static,
    CurrentE: core::error::Error + 'static,
    BusE: core::error::Error + 'static,
    RotorE: core::error::Error + 'static,
    TempE: core::error::Error + 'static,
{
    fn source(&self) -> Option<&(dyn core::error::Error + 'static)> {
        match self {
            Self::Busy => None,
            Self::Inactive => None,
            Self::Pwm(error) => Some(error),
            Self::Current(error) => Some(error),
            Self::Bus(error) => Some(error),
            Self::Rotor(error) => Some(error),
            Self::Temp(error) => Some(error),
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
    /// This must be supplied together with `electrical_direction` and
    /// `electrical_angle_offset` if you
    /// want to skip electrical-mapping calibration.
    pub pole_pairs: Option<u8>,
    /// Provided electrical mapping direction.
    ///
    /// This must be supplied together with `pole_pairs` and
    /// `electrical_angle_offset` if you want to skip electrical-mapping calibration.
    pub electrical_direction: Option<ElectricalDirection>,
    /// Provided electrical zero offset after mechanical-to-electrical conversion.
    ///
    /// This must be supplied together with `pole_pairs` and
    /// `electrical_direction` if you want to skip electrical-mapping calibration.
    pub electrical_angle_offset: Option<ElectricalAngle>,
    /// Provided phase resistance normalized to `25°C`.
    ///
    /// When absent, phase resistance is calibrated and normalized from the
    /// required winding-temperature sample.
    pub phase_resistance_ohm_ref: Option<Ohms>,
    /// Provided common phase inductance. When absent, phase inductance is calibrated.
    pub phase_inductance_h: Option<Henries>,
    /// Provided flux linkage. When absent, flux linkage is calibrated.
    pub flux_linkage_weber: Option<Webers>,
}

impl MotorCalibrationRequest {
    /// Calibrate every motor-side quantity.
    #[inline]
    pub fn all() -> Self {
        Self::default()
    }

    /// Skip electrical-mapping calibration with known pole pairs and offset.
    #[inline]
    pub fn with_known_electrical_mapping(
        pole_pairs: u8,
        electrical_direction: ElectricalDirection,
        electrical_angle_offset: ElectricalAngle,
    ) -> Self {
        Self {
            pole_pairs: Some(pole_pairs),
            electrical_direction: Some(electrical_direction),
            electrical_angle_offset: Some(electrical_angle_offset),
            ..Self::default()
        }
    }
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

/// Wrapper-facing tuning for pole-pair and electrical-offset calibration.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct PolePairsAndOffsetRoutineConfig {
    /// Magnitude of the stator-frame alignment and sweep vector.
    pub align_voltage_mag: Volts,
    /// Initial stator-frame angle used before the sweep starts.
    pub align_stator_angle: ElectricalAngle,
    /// Signed electrical sweep speed.
    pub sweep_electrical_velocity: RadPerSec,
    /// Number of electrical cycles to sweep through.
    pub sweep_electrical_cycles: f32,
    /// Maximum acceptable rotor speed during the settle phases.
    pub settle_velocity_threshold: RadPerSec,
    /// Continuous settle time required before the sweep starts.
    pub initial_settle_time_seconds: f32,
    /// Continuous settle time required after the sweep ends.
    pub final_settle_time_seconds: f32,
    /// Maximum allowed deviation from an integer pole-pair estimate.
    pub pole_pair_rounding_tolerance: f32,
    /// Upper bound for the resolved pole-pair count.
    pub max_pole_pairs: u8,
    /// Absolute timeout for this routine.
    pub timeout_seconds: f32,
}

impl Default for PolePairsAndOffsetRoutineConfig {
    fn default() -> Self {
        Self {
            align_voltage_mag: Volts::new(2.5),
            align_stator_angle: ElectricalAngle::new(0.0),
            sweep_electrical_velocity: RadPerSec::new(10.0),
            sweep_electrical_cycles: 3.0,
            settle_velocity_threshold: RadPerSec::new(1.0),
            initial_settle_time_seconds: 0.05,
            final_settle_time_seconds: 0.05,
            pole_pair_rounding_tolerance: 0.1,
            max_pole_pairs: 32,
            timeout_seconds: 4.0,
        }
    }
}

/// Wrapper-facing tuning for phase-resistance calibration.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct PhaseResistanceRoutineConfig {
    /// Base hold voltage used for the first resistance measurement.
    pub align_voltage_mag: Volts,
    /// Added voltage between repeated measurements.
    pub voltage_increment_mag: Volts,
    /// Stator-frame angle of the hold vector.
    pub align_stator_angle: ElectricalAngle,
    /// Maximum mechanical speed considered settled.
    pub settle_velocity_threshold: RadPerSec,
    /// Continuous settle time required before each measurement.
    pub settle_time_seconds: f32,
    /// Averaging window for each steady-state current measurement.
    pub sample_time_seconds: f32,
    /// Number of resistance measurements to average.
    pub measurement_count: u16,
    /// Minimum usable projected current magnitude.
    pub min_projected_current: fluxkit_math::Amps,
    /// Absolute timeout for this routine.
    pub timeout_seconds: f32,
}

impl Default for PhaseResistanceRoutineConfig {
    fn default() -> Self {
        let cfg = fluxkit_core::PhaseResistanceCalibrationConfig::default_for_hold();
        Self {
            align_voltage_mag: cfg.align_voltage_mag,
            voltage_increment_mag: cfg.voltage_increment_mag,
            align_stator_angle: cfg.align_stator_angle,
            settle_velocity_threshold: cfg.settle_velocity_threshold,
            settle_time_seconds: cfg.settle_time_seconds,
            sample_time_seconds: cfg.sample_time_seconds,
            measurement_count: cfg.measurement_count,
            min_projected_current: cfg.min_projected_current,
            timeout_seconds: cfg.timeout_seconds,
        }
    }
}

/// Wrapper-facing tuning for phase-inductance calibration.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct PhaseInductanceRoutineConfig {
    /// Hold voltage used before pulse measurement begins.
    pub hold_voltage_mag: Volts,
    /// `d`-axis pulse voltage used during the repeated RL measurement.
    pub step_voltage_mag: Volts,
    /// Stator-frame angle of the initial hold vector.
    pub align_stator_angle: ElectricalAngle,
    /// Maximum mechanical speed considered settled before pulses start.
    pub settle_velocity_threshold: RadPerSec,
    /// Continuous settle time required before pulse measurement begins.
    pub settle_time_seconds: f32,
    /// Duration of each pulse on/off window.
    pub sample_time_seconds: f32,
    /// Number of repeated `0 -> +V -> 0` pulse cycles.
    pub repeat_count: u16,
    /// Minimum usable `d`-axis current rise during positive pulse windows.
    pub min_projected_current_step: fluxkit_math::Amps,
    /// Absolute timeout for this routine.
    pub timeout_seconds: f32,
}

impl Default for PhaseInductanceRoutineConfig {
    fn default() -> Self {
        let cfg = fluxkit_core::PhaseInductanceCalibrationConfig::default_for_hold();
        Self {
            hold_voltage_mag: cfg.hold_voltage_mag,
            step_voltage_mag: cfg.step_voltage_mag,
            align_stator_angle: cfg.align_stator_angle,
            settle_velocity_threshold: cfg.settle_velocity_threshold,
            settle_time_seconds: cfg.settle_time_seconds,
            sample_time_seconds: cfg.sample_time_seconds,
            repeat_count: cfg.repeat_count,
            min_projected_current_step: cfg.min_projected_current_step,
            timeout_seconds: cfg.timeout_seconds,
        }
    }
}

/// Wrapper-facing tuning for flux-linkage calibration.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct FluxLinkageRoutineConfig {
    /// Alignment voltage used before the electrical spin starts.
    pub align_voltage_mag: Volts,
    /// Voltage magnitude of the rotating stator vector during the spin.
    pub spin_voltage_mag: Volts,
    /// Initial stator-frame alignment angle.
    pub align_stator_angle: ElectricalAngle,
    /// Commanded electrical angular velocity during the spin.
    pub spin_electrical_velocity: RadPerSec,
    /// Maximum mechanical speed considered settled before the spin begins.
    pub initial_settle_velocity_threshold: RadPerSec,
    /// Continuous settle time required before the spin begins.
    pub initial_settle_time_seconds: f32,
    /// Minimum electrical speed required before flux-linkage samples are accepted.
    pub min_electrical_velocity: RadPerSec,
    /// Warmup time for the filtered `di_q/dt` estimate before flux samples are accepted.
    pub derivative_warmup_time_seconds: f32,
    /// Averaging window for the flux-linkage estimate.
    pub sample_time_seconds: f32,
    /// Absolute timeout for this routine.
    pub timeout_seconds: f32,
}

impl Default for FluxLinkageRoutineConfig {
    fn default() -> Self {
        let cfg = fluxkit_core::FluxLinkageCalibrationConfig::default_for_spin();
        Self {
            align_voltage_mag: cfg.align_voltage_mag,
            spin_voltage_mag: cfg.spin_voltage_mag,
            align_stator_angle: cfg.align_stator_angle,
            spin_electrical_velocity: cfg.spin_electrical_velocity,
            initial_settle_velocity_threshold: cfg.initial_settle_velocity_threshold,
            initial_settle_time_seconds: cfg.initial_settle_time_seconds,
            min_electrical_velocity: cfg.min_electrical_velocity,
            derivative_warmup_time_seconds: cfg.derivative_warmup_time_seconds,
            sample_time_seconds: cfg.sample_time_seconds,
            timeout_seconds: cfg.timeout_seconds,
        }
    }
}

/// Public wrapper-facing configuration for the motor-side calibration campaign.
#[derive(Clone, Copy, Debug, PartialEq, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct MotorCalibrationConfig {
    /// Tuning for pole-pair and electrical-offset calibration.
    pub pole_pairs_and_offset: PolePairsAndOffsetRoutineConfig,
    /// Tuning for phase-resistance calibration.
    pub phase_resistance: PhaseResistanceRoutineConfig,
    /// Tuning for phase-inductance calibration.
    pub phase_inductance: PhaseInductanceRoutineConfig,
    /// Tuning for flux-linkage calibration.
    pub flux_linkage: FluxLinkageRoutineConfig,
}

/// Current or next request-driven motor-calibration phase.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum MotorCalibrationPhase {
    /// Electrical mapping: pole-pair count and electrical offset.
    PolePairsAndOffset,
    /// Phase-resistance identification.
    PhaseResistance,
    /// Phase-inductance identification.
    PhaseInductance,
    /// Flux-linkage identification.
    FluxLinkage,
}

/// Fully resolved motor-side calibration result returned by the request-driven
/// campaign API.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct MotorCalibrationResult {
    /// Estimated electrical pole-pair count.
    pub pole_pairs: u8,
    /// Electrical mapping direction between positive mechanical and electrical motion.
    pub electrical_direction: ElectricalDirection,
    /// Electrical zero offset after mechanical-to-electrical conversion.
    pub electrical_angle_offset: ElectricalAngle,
    /// Estimated phase resistance normalized to `25°C`.
    pub phase_resistance_ohm_ref: Ohms,
    /// Estimated common phase inductance applied to both `d` and `q` axes.
    pub phase_inductance_h: Henries,
    /// Estimated flux linkage.
    pub flux_linkage_weber: Webers,
}

impl MotorCalibrationResult {
    /// Builds motor parameters directly from this resolved calibration plus
    /// independent operating limits.
    #[inline]
    pub const fn into_motor_params(self, limits: MotorLimits) -> MotorParams {
        MotorParams::from_model_and_limits(
            MotorModel {
                pole_pairs: self.pole_pairs,
                phase_resistance_ohm_ref: self.phase_resistance_ohm_ref,
                d_inductance_h: self.phase_inductance_h,
                q_inductance_h: self.phase_inductance_h,
                flux_linkage_weber: self.flux_linkage_weber,
                electrical_direction: self.electrical_direction,
                electrical_angle_offset: self.electrical_angle_offset,
            },
            limits,
        )
    }

    /// Applies this resolved record onto an existing motor-parameter record.
    #[inline]
    pub fn apply_to_motor_params(&self, motor: &mut MotorParams) {
        motor.pole_pairs = self.pole_pairs;
        motor.electrical_direction = self.electrical_direction;
        motor.electrical_angle_offset = self.electrical_angle_offset;
        motor.phase_resistance_ohm_ref = self.phase_resistance_ohm_ref;
        motor.d_inductance_h = self.phase_inductance_h;
        motor.q_inductance_h = self.phase_inductance_h;
        motor.flux_linkage_weber = self.flux_linkage_weber;
    }
}

/// Shared calibration status snapshot for non-owning contexts.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct MotorCalibrationStatus {
    /// `true` while the calibration owner still contains an active inner runtime.
    pub active: bool,
    /// Current or next phase while calibration is in progress.
    pub phase: Option<MotorCalibrationPhase>,
    /// Latest resolved electrical pole-pair count once that stage has completed.
    pub pole_pairs: Option<u8>,
    /// Latest resolved electrical mapping direction once that stage has completed.
    pub electrical_direction: Option<ElectricalDirection>,
    /// Latest resolved electrical zero offset once that stage has completed.
    pub electrical_angle_offset: Option<ElectricalAngle>,
    /// Latest resolved phase resistance once that stage has completed.
    pub phase_resistance_ohm_ref: Option<Ohms>,
    /// Latest resolved common phase inductance once that stage has completed.
    pub phase_inductance_h: Option<Henries>,
    /// Final resolved result once calibration completes.
    pub result: Option<MotorCalibrationResult>,
    /// `true` when the calibration runtime has latched a terminal fault.
    pub fault_latched: bool,
}

/// Owned motor-calibration parts that can be moved into another phase.
#[derive(Debug)]
pub struct MotorCalibrationParts<PWM, CURRENT, BUS, ROTOR, TEMP, MOD, RotorEst> {
    /// PWM output handle.
    pub pwm: PWM,
    /// Phase-current sampler.
    pub current: CURRENT,
    /// DC bus-voltage sensor.
    pub bus: BUS,
    /// Rotor sensor.
    pub rotor: ROTOR,
    /// Winding temperature sensor.
    pub temp: TEMP,
    /// Modulation strategy.
    pub modulator: MOD,
    /// Rotor-motion estimator.
    pub rotor_estimator: RotorEst,
}

/// Non-owning access to calibration progress and final result.
#[derive(Debug)]
pub struct MotorCalibrationHandle<'a> {
    shared: &'a Mutex<RefCell<SharedStatus<MotorCalibrationStatus>>>,
}

impl<'a> MotorCalibrationHandle<'a> {
    /// Returns the latest shared calibration status.
    pub fn status(&self) -> MotorCalibrationStatus {
        read_status(self.shared)
    }

    /// Returns `true` while the owning calibration runtime is still active.
    #[inline]
    pub fn is_active(&self) -> bool {
        self.status().active
    }

    /// Returns `true` once calibration has produced a final result.
    #[inline]
    pub fn is_complete(&self) -> bool {
        self.status().result.is_some()
    }

    /// Returns `true` when calibration has latched a terminal fault.
    #[inline]
    pub fn is_faulted(&self) -> bool {
        self.status().fault_latched
    }
}

/// Active inner motor-calibration runtime owned by the public wrapper.
#[derive(Debug)]
struct InnerMotorCalibrationRuntime<PWM, CURRENT, BUS, ROTOR, TEMP, MOD, RotorEst> {
    pwm: PWM,
    current: CURRENT,
    bus: BUS,
    rotor: ROTOR,
    temp: TEMP,
    modulator: MOD,
    rotor_estimator: RotorEst,
    config: MotorCalibrationConfig,
    limits: MotorCalibrationLimits,
    dt_seconds: f32,
    pole_pairs: Option<u8>,
    electrical_direction: Option<ElectricalDirection>,
    electrical_angle_offset: Option<ElectricalAngle>,
    phase_resistance_ohm_ref: Option<Ohms>,
    phase_inductance_h: Option<Henries>,
    flux_linkage_weber: Option<Webers>,
    current_phase: Option<MotorCalibrationPhase>,
    resolved_result: Option<MotorCalibrationResult>,
    active_routine: Option<MotorCalibrationRoutine>,
}

/// Main-context owner of one active motor-calibration runtime.
#[derive(Debug)]
pub struct MotorCalibrationRuntime<PWM, CURRENT, BUS, ROTOR, TEMP, MOD, RotorEst> {
    inner: Mutex<
        RefCell<
            Option<InnerMotorCalibrationRuntime<PWM, CURRENT, BUS, ROTOR, TEMP, MOD, RotorEst>>,
        >,
    >,
    shared: Mutex<RefCell<SharedStatus<MotorCalibrationStatus>>>,
    split_taken: Cell<bool>,
}

/// IRQ-side execution capability for one active motor-calibration runtime.
#[derive(Debug)]
pub struct MotorCalibrationTicker<'a, PWM, CURRENT, BUS, ROTOR, TEMP, MOD, RotorEst> {
    inner: &'a Mutex<
        RefCell<
            Option<InnerMotorCalibrationRuntime<PWM, CURRENT, BUS, ROTOR, TEMP, MOD, RotorEst>>,
        >,
    >,
    shared: &'a Mutex<RefCell<SharedStatus<MotorCalibrationStatus>>>,
}

fn next_phase_for_request(request: MotorCalibrationRequest) -> Option<MotorCalibrationPhase> {
    if request.pole_pairs.is_none()
        || request.electrical_direction.is_none()
        || request.electrical_angle_offset.is_none()
    {
        return Some(MotorCalibrationPhase::PolePairsAndOffset);
    }
    if request.phase_resistance_ohm_ref.is_none() {
        return Some(MotorCalibrationPhase::PhaseResistance);
    }
    if request.phase_inductance_h.is_none() {
        return Some(MotorCalibrationPhase::PhaseInductance);
    }
    if request.flux_linkage_weber.is_none() {
        return Some(MotorCalibrationPhase::FluxLinkage);
    }
    None
}

fn resolved_result_for_request(request: MotorCalibrationRequest) -> Option<MotorCalibrationResult> {
    Some(MotorCalibrationResult {
        pole_pairs: request.pole_pairs?,
        electrical_direction: request.electrical_direction?,
        electrical_angle_offset: request.electrical_angle_offset?,
        phase_resistance_ohm_ref: request.phase_resistance_ohm_ref?,
        phase_inductance_h: request.phase_inductance_h?,
        flux_linkage_weber: request.flux_linkage_weber?,
    })
}

#[inline]
fn min_volts(a: Volts, b: Volts) -> Volts {
    Volts::new(a.get().min(b.get()))
}

impl PolePairsAndOffsetRoutineConfig {
    fn to_core(
        self,
        limits: MotorCalibrationLimits,
    ) -> fluxkit_core::PolePairsAndOffsetCalibrationConfig {
        fluxkit_core::PolePairsAndOffsetCalibrationConfig {
            align_voltage_mag: min_volts(self.align_voltage_mag, limits.max_align_voltage_mag),
            align_stator_angle: self.align_stator_angle,
            sweep_electrical_velocity: clamp_abs_rad_per_sec(
                self.sweep_electrical_velocity,
                limits.max_electrical_velocity,
            ),
            sweep_electrical_cycles: self.sweep_electrical_cycles,
            settle_velocity_threshold: self.settle_velocity_threshold,
            initial_settle_time_seconds: self.initial_settle_time_seconds,
            final_settle_time_seconds: self.final_settle_time_seconds,
            pole_pair_rounding_tolerance: self.pole_pair_rounding_tolerance,
            max_pole_pairs: self.max_pole_pairs,
            timeout_seconds: self.timeout_seconds.min(limits.timeout_seconds),
        }
    }
}

impl PhaseResistanceRoutineConfig {
    fn to_core(
        self,
        limits: MotorCalibrationLimits,
    ) -> fluxkit_core::PhaseResistanceCalibrationConfig {
        let align_voltage_mag = min_volts(self.align_voltage_mag, limits.max_align_voltage_mag);
        let voltage_increment_mag = if self.measurement_count > 1 {
            let remaining = (limits.max_align_voltage_mag.get() - align_voltage_mag.get()).max(0.0);
            let max_increment = remaining / (self.measurement_count - 1) as f32;
            Volts::new(self.voltage_increment_mag.get().min(max_increment))
        } else {
            Volts::ZERO
        };
        fluxkit_core::PhaseResistanceCalibrationConfig {
            align_voltage_mag,
            voltage_increment_mag,
            align_stator_angle: self.align_stator_angle,
            settle_velocity_threshold: self.settle_velocity_threshold,
            settle_time_seconds: self.settle_time_seconds,
            sample_time_seconds: self.sample_time_seconds,
            measurement_count: self.measurement_count,
            min_projected_current: self.min_projected_current,
            timeout_seconds: self.timeout_seconds.min(limits.timeout_seconds),
        }
    }
}

impl PhaseInductanceRoutineConfig {
    fn to_core(
        self,
        limits: MotorCalibrationLimits,
        phase_resistance_ohm: Ohms,
    ) -> fluxkit_core::PhaseInductanceCalibrationConfig {
        fluxkit_core::PhaseInductanceCalibrationConfig {
            phase_resistance_ohm,
            hold_voltage_mag: min_volts(self.hold_voltage_mag, limits.max_align_voltage_mag),
            step_voltage_mag: min_volts(self.step_voltage_mag, limits.max_align_voltage_mag),
            align_stator_angle: self.align_stator_angle,
            settle_velocity_threshold: self.settle_velocity_threshold,
            settle_time_seconds: self.settle_time_seconds,
            sample_time_seconds: self.sample_time_seconds,
            repeat_count: self.repeat_count,
            min_projected_current_step: self.min_projected_current_step,
            timeout_seconds: self.timeout_seconds.min(limits.timeout_seconds),
        }
    }
}

impl FluxLinkageRoutineConfig {
    fn to_core(
        self,
        limits: MotorCalibrationLimits,
        phase_resistance_ohm: Ohms,
        phase_inductance_h: Henries,
        pole_pairs: u8,
        electrical_direction: ElectricalDirection,
        electrical_angle_offset: ElectricalAngle,
    ) -> fluxkit_core::FluxLinkageCalibrationConfig {
        fluxkit_core::FluxLinkageCalibrationConfig {
            phase_resistance_ohm,
            phase_inductance_h,
            pole_pairs,
            electrical_direction,
            electrical_angle_offset,
            align_voltage_mag: min_volts(self.align_voltage_mag, limits.max_align_voltage_mag),
            spin_voltage_mag: min_volts(self.spin_voltage_mag, limits.max_spin_voltage_mag),
            align_stator_angle: self.align_stator_angle,
            spin_electrical_velocity: clamp_abs_rad_per_sec(
                self.spin_electrical_velocity,
                limits.max_electrical_velocity,
            ),
            initial_settle_velocity_threshold: self.initial_settle_velocity_threshold,
            initial_settle_time_seconds: self.initial_settle_time_seconds,
            min_electrical_velocity: self.min_electrical_velocity,
            derivative_warmup_time_seconds: self.derivative_warmup_time_seconds,
            sample_time_seconds: self.sample_time_seconds,
            timeout_seconds: self.timeout_seconds.min(limits.timeout_seconds),
        }
    }
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
    request.pole_pairs.is_some() == request.electrical_direction.is_some()
        && request.pole_pairs.is_some() == request.electrical_angle_offset.is_some()
}

#[inline]
fn validate_dt_seconds(dt_seconds: f32) -> bool {
    dt_seconds.is_finite() && dt_seconds > 0.0
}

#[inline]
fn validate_config(config: MotorCalibrationConfig, limits: MotorCalibrationLimits) -> bool {
    PolePairsAndOffsetCalibrator::new(config.pole_pairs_and_offset.to_core(limits)).is_ok()
        && PhaseResistanceCalibrator::new(config.phase_resistance.to_core(limits)).is_ok()
        && PhaseInductanceCalibrator::new(config.phase_inductance.to_core(limits, Ohms::new(0.1)))
            .is_ok()
        && FluxLinkageCalibrator::new(config.flux_linkage.to_core(
            limits,
            Ohms::new(0.1),
            Henries::new(30.0e-6),
            7,
            ElectricalDirection::Positive,
            ElectricalAngle::new(0.0),
        ))
        .is_ok()
}

impl From<&MotorCalibrationRoutine> for MotorCalibrationPhase {
    fn from(value: &MotorCalibrationRoutine) -> Self {
        match value {
            MotorCalibrationRoutine::PolePairsAndOffset(_) => Self::PolePairsAndOffset,
            MotorCalibrationRoutine::PhaseResistance(_) => Self::PhaseResistance,
            MotorCalibrationRoutine::PhaseInductance(_) => Self::PhaseInductance,
            MotorCalibrationRoutine::FluxLinkage(_) => Self::FluxLinkage,
        }
    }
}

mod execution;
mod runtime;

#[cfg(test)]
mod tests;
