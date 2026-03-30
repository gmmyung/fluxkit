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
    RotorReading, RotorSensor, TemperatureSensor, centered_phase_duty,
};
use fluxkit_math::{
    estimation::PassThroughEstimator,
    frame::Abc,
    modulation::Svpwm,
    units::{Amps, Duty, RadPerSec, Volts},
};

use super::{MotorCalibrationConfig, MotorCalibrationRuntime, MotorCalibrationRuntimeError};

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

#[derive(Debug)]
struct FakeTempSensor {
    winding_temperature_c: f32,
}

impl TemperatureSensor for FakeTempSensor {
    type Error = Infallible;

    fn sample_temperature_c(&mut self) -> Result<f32, Self::Error> {
        Ok(self.winding_temperature_c)
    }
}

fn hardware() -> (
    FakePwm,
    FakeCurrentSensor,
    FakeBusSensor,
    FakeRotor,
    FakeTempSensor,
) {
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
        FakeTempSensor {
            winding_temperature_c: 25.0,
        },
    )
}

fn system(
    pwm: FakePwm,
    current: FakeCurrentSensor,
    bus: FakeBusSensor,
    rotor: FakeRotor,
    temp: FakeTempSensor,
) -> MotorCalibrationRuntime<
    FakePwm,
    FakeCurrentSensor,
    FakeBusSensor,
    FakeRotor,
    FakeTempSensor,
    Svpwm,
    PassThroughEstimator,
> {
    MotorCalibrationRuntime::new(
        pwm,
        current,
        bus,
        rotor,
        temp,
        Svpwm,
        PassThroughEstimator::new(),
        super::MotorCalibrationRequest {
            pole_pairs: Some(7),
            electrical_direction: Some(fluxkit_math::ElectricalDirection::Positive),
            electrical_angle_offset: Some(fluxkit_math::ElectricalAngle::new(0.0)),
            phase_resistance_ohm_ref: Some(fluxkit_math::units::Ohms::new(0.12)),
            phase_inductance_h: Some(fluxkit_math::units::Henries::new(30.0e-6)),
            flux_linkage_weber: Some(fluxkit_math::units::Webers::new(0.005)),
        },
        super::MotorCalibrationLimits {
            max_align_voltage_mag: Volts::new(2.0),
            max_spin_voltage_mag: Volts::new(3.0),
            max_electrical_velocity: RadPerSec::new(60.0),
            timeout_seconds: 2.0,
        },
        0.005,
    )
    .unwrap()
}

#[test]
fn resistance_wrapper_rejects_invalid_current_sample() {
    let (pwm, mut current, bus, rotor, temp) = hardware();
    current.sample.validity = CurrentSampleValidity::Invalid;
    let system = system(pwm, current, bus, rotor, temp);
    let calibrator = PhaseResistanceCalibrator::new(PhaseResistanceCalibrationConfig {
        settle_time_seconds: 0.01,
        sample_time_seconds: 0.01,
        timeout_seconds: 1.0,
        ..PhaseResistanceCalibrationConfig::default_for_hold()
    })
    .unwrap();

    let mut routine = MotorCalibrationRoutine::PhaseResistance(calibrator);
    let result = system.tick_active_routine_for_test(&mut routine, 0.005);
    assert!(matches!(
        result,
        Err(MotorCalibrationRuntimeError::InvalidCurrentSample)
    ));
    let parts = system.try_into_parts().unwrap();
    assert_eq!(parts.pwm.duty, centered_phase_duty());
    let MotorCalibrationRoutine::PhaseResistance(calibrator) = routine else {
        unreachable!();
    };
    assert_eq!(calibrator.result(), None);
    assert_eq!(calibrator.error(), None);
}

#[test]
fn phase_reports_next_unresolved_step() {
    let (pwm, current, bus, rotor, temp) = hardware();
    let system = MotorCalibrationRuntime::new(
        pwm,
        current,
        bus,
        rotor,
        temp,
        Svpwm,
        PassThroughEstimator::new(),
        super::MotorCalibrationRequest {
            pole_pairs: None,
            electrical_direction: None,
            electrical_angle_offset: None,
            phase_resistance_ohm_ref: Some(fluxkit_math::units::Ohms::new(0.12)),
            phase_inductance_h: Some(fluxkit_math::units::Henries::new(30.0e-6)),
            flux_linkage_weber: Some(fluxkit_math::units::Webers::new(0.005)),
        },
        super::MotorCalibrationLimits {
            max_align_voltage_mag: Volts::new(2.0),
            max_spin_voltage_mag: Volts::new(3.0),
            max_electrical_velocity: RadPerSec::new(60.0),
            timeout_seconds: 2.0,
        },
        0.005,
    )
    .unwrap();

    let (handle, ticker) = system.split().expect("calibration should split once");
    assert_eq!(
        handle.status().phase,
        Some(super::MotorCalibrationPhase::PolePairsAndOffset)
    );
    ticker.tick().unwrap();
    assert_eq!(
        handle.status().phase,
        Some(super::MotorCalibrationPhase::PolePairsAndOffset)
    );
}

#[test]
fn complete_request_publishes_result_immediately() {
    let (pwm, current, bus, rotor, temp) = hardware();
    let system = system(pwm, current, bus, rotor, temp);

    let (handle, _ticker) = system.split().expect("calibration should split once");
    let status = handle.status();

    assert_eq!(status.phase, None);
    assert_eq!(
        status.result,
        Some(super::MotorCalibrationResult {
            pole_pairs: 7,
            electrical_direction: fluxkit_math::ElectricalDirection::Positive,
            electrical_angle_offset: fluxkit_math::ElectricalAngle::new(0.0),
            phase_resistance_ohm_ref: fluxkit_math::units::Ohms::new(0.12),
            phase_inductance_h: fluxkit_math::units::Henries::new(30.0e-6),
            flux_linkage_weber: fluxkit_math::units::Webers::new(0.005),
        })
    );
}

#[test]
fn inductance_wrapper_rejects_invalid_current_sample() {
    let (pwm, mut current, bus, rotor, temp) = hardware();
    current.sample.validity = CurrentSampleValidity::Invalid;
    let system = system(pwm, current, bus, rotor, temp);
    let calibrator = PhaseInductanceCalibrator::new(PhaseInductanceCalibrationConfig {
        phase_resistance_ohm: fluxkit_math::units::Ohms::new(0.12),
        settle_time_seconds: 0.01,
        sample_time_seconds: 200.0e-6,
        timeout_seconds: 1.0,
        ..PhaseInductanceCalibrationConfig::default_for_hold()
    })
    .unwrap();

    let mut routine = MotorCalibrationRoutine::PhaseInductance(calibrator);
    let result = system.tick_active_routine_for_test(&mut routine, 0.005);
    assert!(matches!(
        result,
        Err(MotorCalibrationRuntimeError::InvalidCurrentSample)
    ));
    let parts = system.try_into_parts().unwrap();
    assert_eq!(parts.pwm.duty, centered_phase_duty());
    let MotorCalibrationRoutine::PhaseInductance(calibrator) = routine else {
        unreachable!();
    };
    assert_eq!(calibrator.result(), None);
    assert_eq!(calibrator.error(), None);
}

#[test]
fn flux_linkage_wrapper_rejects_invalid_current_sample() {
    let (pwm, mut current, bus, rotor, temp) = hardware();
    current.sample.validity = CurrentSampleValidity::Invalid;
    let system = system(pwm, current, bus, rotor, temp);
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
    let result = system.tick_active_routine_for_test(&mut routine, 0.005);
    assert!(matches!(
        result,
        Err(MotorCalibrationRuntimeError::InvalidCurrentSample)
    ));
    let parts = system.try_into_parts().unwrap();
    assert_eq!(parts.pwm.duty, centered_phase_duty());
    let MotorCalibrationRoutine::FluxLinkage(calibrator) = routine else {
        unreachable!();
    };
    assert_eq!(calibrator.result(), None);
    assert_eq!(calibrator.error(), None);
}

#[test]
fn extracted_calibration_marks_handles_and_tickers_inactive() {
    let (pwm, current, bus, rotor, temp) = hardware();
    let system = system(pwm, current, bus, rotor, temp);
    let (handle, ticker) = system.split().expect("calibration should split once");

    let _parts = system
        .try_into_parts()
        .expect("calibration parts should be available");

    assert!(!handle.status().active);
    assert!(matches!(
        ticker.tick(),
        Err(MotorCalibrationRuntimeError::Inactive)
    ));
}

#[test]
fn wrapper_rejects_invalid_public_motor_calibration_config() {
    let (pwm, current, bus, rotor, temp) = hardware();
    let mut config = MotorCalibrationConfig::default();
    config.phase_resistance.measurement_count = 0;

    let result = MotorCalibrationRuntime::new_with_config(
        pwm,
        current,
        bus,
        rotor,
        temp,
        Svpwm,
        PassThroughEstimator::new(),
        super::MotorCalibrationRequest::all(),
        super::MotorCalibrationLimits {
            max_align_voltage_mag: Volts::new(2.0),
            max_spin_voltage_mag: Volts::new(3.0),
            max_electrical_velocity: RadPerSec::new(60.0),
            timeout_seconds: 2.0,
        },
        config,
        0.005,
    );

    assert!(matches!(
        result,
        Err(fluxkit_core::CalibrationError::InvalidConfiguration)
    ));
}
