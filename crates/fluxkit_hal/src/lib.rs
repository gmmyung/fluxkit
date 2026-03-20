#![no_std]
#![forbid(unsafe_code)]
#![deny(rust_2018_idioms)]
#![warn(missing_docs, missing_debug_implementations)]
//! Narrow motor-control hardware abstraction contracts for Fluxkit.
//!
//! `fluxkit_hal` defines synchronous traits for platform integration without
//! owning peripherals, runtimes, or control logic. Platform crates use these
//! traits to acquire measurements and drive actuators around `fluxkit_core`.

pub mod bus;
pub mod current;
pub mod fault;
pub mod gate;
pub mod pwm;
pub mod rotor;
pub mod temperature;
pub mod time;
pub mod util;

pub use bus::*;
pub use current::*;
pub use fault::*;
pub use gate::*;
pub use pwm::*;
pub use rotor::*;
pub use temperature::*;
pub use time::*;
pub use util::*;

#[cfg(test)]
mod tests {
    use core::convert::Infallible;

    use crate::{
        BusVoltageSensor, CurrentSampleValidity, CurrentSampler, MonotonicMicros,
        PhaseCurrentSample, PhasePwm, RotorReading, RotorSensor, centered_phase_duty,
    };
    use fluxkit_math::{
        ElectricalAngle,
        frame::Abc,
        units::{Amps, Duty, RadPerSec, Volts},
    };

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
    struct FakeCurrentSensor;

    impl CurrentSampler for FakeCurrentSensor {
        type Error = Infallible;

        fn sample_phase_currents(&mut self) -> Result<PhaseCurrentSample, Self::Error> {
            Ok(PhaseCurrentSample {
                currents: Abc::new(Amps::new(1.0), Amps::new(-0.5), Amps::new(-0.5)),
                validity: CurrentSampleValidity::Valid,
            })
        }
    }

    #[derive(Debug)]
    struct FakeBusSensor;

    impl BusVoltageSensor for FakeBusSensor {
        type Error = Infallible;

        fn sample_bus_voltage(&mut self) -> Result<Volts, Self::Error> {
            Ok(Volts::new(24.0))
        }
    }

    #[derive(Debug)]
    struct FakeRotor;

    impl RotorSensor for FakeRotor {
        type Error = Infallible;

        fn read_rotor(&mut self) -> Result<RotorReading, Self::Error> {
            Ok(RotorReading {
                electrical_angle: ElectricalAngle::new(1.0),
                mechanical_velocity: RadPerSec::new(10.0),
            })
        }
    }

    #[derive(Debug)]
    struct FakeClock;

    impl MonotonicMicros for FakeClock {
        fn now_micros(&self) -> u64 {
            123_456
        }
    }

    #[test]
    fn pwm_helper_sets_neutral_output() {
        let mut pwm = FakePwm::default();
        pwm.enable().unwrap();
        pwm.set_phase_duty(centered_phase_duty()).unwrap();
        assert!(pwm.enabled);
        assert_eq!(pwm.duty.a.get(), 0.5);
        assert_eq!(pwm.duty.b.get(), 0.5);
        assert_eq!(pwm.duty.c.get(), 0.5);
    }

    #[test]
    fn mock_contracts_are_practical_together() {
        let mut currents = FakeCurrentSensor;
        let mut bus = FakeBusSensor;
        let mut rotor = FakeRotor;
        let clock = FakeClock;

        let current_sample = currents.sample_phase_currents().unwrap();
        let bus_voltage = bus.sample_bus_voltage().unwrap();
        let rotor_reading = rotor.read_rotor().unwrap();

        assert_eq!(current_sample.validity, CurrentSampleValidity::Valid);
        assert_eq!(bus_voltage, Volts::new(24.0));
        assert_eq!(rotor_reading.electrical_angle, ElectricalAngle::new(1.0));
        assert_eq!(clock.now_micros(), 123_456);
    }
}
