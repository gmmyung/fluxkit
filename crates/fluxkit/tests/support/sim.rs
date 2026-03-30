#![allow(dead_code)]

use std::{
    cell::RefCell,
    rc::Rc,
    sync::{Arc, Mutex},
};

use fluxkit::{
    Abc, ActuatorCompensationConfig, ActuatorLimits, ActuatorModel, ActuatorParams,
    BusVoltageSensor, ContinuousMechanicalAngle, CurrentLoopConfig, CurrentSampleValidity,
    CurrentSampler, ElectricalAngle, ElectricalDirection, InverterParams, MotorLimits, MotorModel,
    MotorParams, OutputReading, OutputSensor, PhaseCurrentSample, PhasePwm, RotorReading,
    RotorSensor, TemperatureSensor,
    units::{Amps, Duty, Henries, Hertz, NewtonMeters, Ohms, RadPerSec, Volts, Webers},
};
use fluxkit_math::{inverse_clarke, inverse_park};
use fluxkit_pmsm_sim::PmsmModel;

pub const FAST_DT_SECONDS: f32 = 1.0 / 20_000.0;
pub const GEAR_RATIO: f32 = 2.0;
pub const WINDING_TEMP_C: f32 = 25.0;

pub fn controller_motor_params() -> MotorParams {
    MotorParams::from_model_and_limits(
        MotorModel {
            pole_pairs: 7,
            phase_resistance_ohm_ref: Ohms::new(0.12),
            d_inductance_h: Henries::new(0.000_03),
            q_inductance_h: Henries::new(0.000_03),
            flux_linkage_weber: Webers::new(0.005),
            electrical_direction: ElectricalDirection::Positive,
            electrical_angle_offset: ElectricalAngle::new(0.0),
        },
        MotorLimits {
            max_phase_current: Amps::new(10.0),
            max_mech_speed: Some(RadPerSec::new(150.0)),
            max_winding_temperature_c: None,
        },
    )
}

pub fn inverter_params() -> InverterParams {
    InverterParams {
        pwm_frequency_hz: Hertz::new(20_000.0),
        min_duty: Duty::new(0.0),
        max_duty: Duty::new(1.0),
        min_bus_voltage: Volts::new(6.0),
        max_bus_voltage: Volts::new(60.0),
        max_voltage_command: Volts::new(24.0),
    }
}

pub fn current_loop_config() -> CurrentLoopConfig {
    CurrentLoopConfig {
        kp_d: 0.2,
        ki_d: 400.0,
        kp_q: 0.2,
        ki_q: 400.0,
        velocity_kp: 0.2,
        velocity_ki: 8.0,
        position_kp: 12.0,
        position_ki: 0.0,
        max_voltage_mag: Volts::new(12.0),
        id_ref_default: Amps::ZERO,
        max_id_target: Amps::new(5.0),
        max_iq_target: Amps::new(8.0),
        max_velocity_target: RadPerSec::new(120.0),
        max_current_ref_derivative_amps_per_sec: 10_000.0,
        enable_current_feedforward: true,
    }
}

pub fn runtime_actuator_params() -> ActuatorParams {
    ActuatorParams::from_model_limits_and_compensation(
        ActuatorModel {
            gear_ratio: GEAR_RATIO,
        },
        ActuatorLimits {
            max_output_velocity: Some(RadPerSec::new(30.0)),
            max_output_torque: Some(NewtonMeters::new(10.0)),
        },
        ActuatorCompensationConfig::disabled(),
    )
}

#[derive(Debug)]
pub struct SimHarness {
    pub plant: PmsmModel,
    pub bus_voltage: Volts,
    pub load_torque: NewtonMeters,
    pub last_duty: Abc<Duty>,
    pub rotor_bias: f32,
}

pub type LocalSharedHarness = Rc<RefCell<SimHarness>>;
pub type ThreadSharedHarness = Arc<Mutex<SimHarness>>;

fn phase_currents_from_harness(harness: &SimHarness) -> Abc<Amps> {
    let state = *harness.plant.state();
    let electrical_angle = fluxkit::angle::mechanical_to_electrical_with_direction(
        state.mechanical_angle.wrapped().into(),
        harness.plant.params().pole_pairs as u32,
        harness.plant.params().electrical_direction,
    );
    inverse_clarke(inverse_park(
        state.current_dq.map(|current| current.get()),
        electrical_angle.get(),
    ))
    .map(Amps::new)
}

#[derive(Clone, Debug)]
pub struct LocalSimPwm {
    shared: LocalSharedHarness,
}

impl PhasePwm for LocalSimPwm {
    type Error = core::convert::Infallible;

    fn enable(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }

    fn disable(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }

    fn set_duty(&mut self, a: Duty, b: Duty, c: Duty) -> Result<(), Self::Error> {
        let duty = Abc::new(a, b, c);
        let mut harness = self.shared.borrow_mut();
        harness.last_duty = duty;
        let bus_voltage = harness.bus_voltage;
        let load_torque = harness.load_torque;
        harness
            .plant
            .step_phase_duty(duty, bus_voltage, load_torque, FAST_DT_SECONDS)
            .unwrap();
        Ok(())
    }
}

#[derive(Clone, Debug)]
pub struct LocalSimCurrent {
    shared: LocalSharedHarness,
}

impl CurrentSampler for LocalSimCurrent {
    type Error = core::convert::Infallible;

    fn sample_phase_currents(&mut self) -> Result<PhaseCurrentSample, Self::Error> {
        Ok(PhaseCurrentSample {
            currents: phase_currents_from_harness(&self.shared.borrow()),
            validity: CurrentSampleValidity::Valid,
        })
    }
}

#[derive(Clone, Debug)]
pub struct LocalSimBus {
    shared: LocalSharedHarness,
}

impl BusVoltageSensor for LocalSimBus {
    type Error = core::convert::Infallible;

    fn sample_bus_voltage(&mut self) -> Result<Volts, Self::Error> {
        Ok(self.shared.borrow().bus_voltage)
    }
}

#[derive(Clone, Debug)]
pub struct LocalSimRotor {
    shared: LocalSharedHarness,
}

impl RotorSensor for LocalSimRotor {
    type Error = core::convert::Infallible;

    fn read_rotor(&mut self) -> Result<RotorReading, Self::Error> {
        let harness = self.shared.borrow();
        let state = *harness.plant.state();
        Ok(RotorReading {
            mechanical_angle: fluxkit::MechanicalAngle::new(
                state.mechanical_angle.get() + harness.rotor_bias,
            ),
            mechanical_velocity: state.mechanical_velocity,
        })
    }
}

#[derive(Clone, Debug)]
pub struct LocalSimOutput {
    shared: LocalSharedHarness,
}

impl OutputSensor for LocalSimOutput {
    type Error = core::convert::Infallible;

    fn read_output(&mut self) -> Result<OutputReading, Self::Error> {
        let harness = self.shared.borrow();
        let state = *harness.plant.state();
        Ok(OutputReading {
            mechanical_angle: ContinuousMechanicalAngle::new(
                state.mechanical_angle.get() / GEAR_RATIO,
            )
            .wrapped(),
            mechanical_velocity: RadPerSec::new(state.mechanical_velocity.get() / GEAR_RATIO),
        })
    }
}

#[derive(Clone, Debug)]
pub struct LocalSimOutputInverted {
    shared: LocalSharedHarness,
}

impl OutputSensor for LocalSimOutputInverted {
    type Error = core::convert::Infallible;

    fn read_output(&mut self) -> Result<OutputReading, Self::Error> {
        let harness = self.shared.borrow();
        let state = *harness.plant.state();
        Ok(OutputReading {
            mechanical_angle: ContinuousMechanicalAngle::new(
                -(state.mechanical_angle.get() / GEAR_RATIO),
            )
            .wrapped(),
            mechanical_velocity: RadPerSec::new(-(state.mechanical_velocity.get() / GEAR_RATIO)),
        })
    }
}

#[derive(Clone, Debug)]
pub struct LocalSimTemp {
    shared: LocalSharedHarness,
}

impl TemperatureSensor for LocalSimTemp {
    type Error = core::convert::Infallible;

    fn sample_temperature_c(&mut self) -> Result<f32, Self::Error> {
        Ok(self.shared.borrow().plant.winding_temperature_c())
    }
}

pub fn local_calibration_hardware(
    shared: &LocalSharedHarness,
) -> (
    LocalSimPwm,
    LocalSimCurrent,
    LocalSimBus,
    LocalSimRotor,
    LocalSimTemp,
) {
    (
        LocalSimPwm {
            shared: Rc::clone(shared),
        },
        LocalSimCurrent {
            shared: Rc::clone(shared),
        },
        LocalSimBus {
            shared: Rc::clone(shared),
        },
        LocalSimRotor {
            shared: Rc::clone(shared),
        },
        LocalSimTemp {
            shared: Rc::clone(shared),
        },
    )
}

pub fn local_runtime_hardware(
    shared: &LocalSharedHarness,
) -> (
    LocalSimPwm,
    LocalSimCurrent,
    LocalSimBus,
    LocalSimRotor,
    LocalSimOutput,
    LocalSimTemp,
) {
    (
        LocalSimPwm {
            shared: Rc::clone(shared),
        },
        LocalSimCurrent {
            shared: Rc::clone(shared),
        },
        LocalSimBus {
            shared: Rc::clone(shared),
        },
        LocalSimRotor {
            shared: Rc::clone(shared),
        },
        LocalSimOutput {
            shared: Rc::clone(shared),
        },
        LocalSimTemp {
            shared: Rc::clone(shared),
        },
    )
}

pub fn local_output(shared: &LocalSharedHarness) -> LocalSimOutput {
    LocalSimOutput {
        shared: Rc::clone(shared),
    }
}

pub fn local_output_inverted(shared: &LocalSharedHarness) -> LocalSimOutputInverted {
    LocalSimOutputInverted {
        shared: Rc::clone(shared),
    }
}

#[derive(Clone, Debug)]
pub struct ThreadedSimPwm {
    shared: ThreadSharedHarness,
}

impl PhasePwm for ThreadedSimPwm {
    type Error = core::convert::Infallible;

    fn enable(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }

    fn disable(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }

    fn set_duty(&mut self, a: Duty, b: Duty, c: Duty) -> Result<(), Self::Error> {
        let duty = Abc::new(a, b, c);
        let mut harness = self.shared.lock().unwrap();
        harness.last_duty = duty;
        let bus_voltage = harness.bus_voltage;
        let load_torque = harness.load_torque;
        harness
            .plant
            .step_phase_duty(duty, bus_voltage, load_torque, FAST_DT_SECONDS)
            .unwrap();
        Ok(())
    }
}

#[derive(Clone, Debug)]
pub struct ThreadedSimCurrent {
    shared: ThreadSharedHarness,
}

impl CurrentSampler for ThreadedSimCurrent {
    type Error = core::convert::Infallible;

    fn sample_phase_currents(&mut self) -> Result<PhaseCurrentSample, Self::Error> {
        Ok(PhaseCurrentSample {
            currents: phase_currents_from_harness(&self.shared.lock().unwrap()),
            validity: CurrentSampleValidity::Valid,
        })
    }
}

#[derive(Clone, Debug)]
pub struct ThreadedSimBus {
    shared: ThreadSharedHarness,
}

impl BusVoltageSensor for ThreadedSimBus {
    type Error = core::convert::Infallible;

    fn sample_bus_voltage(&mut self) -> Result<Volts, Self::Error> {
        Ok(self.shared.lock().unwrap().bus_voltage)
    }
}

#[derive(Clone, Debug)]
pub struct ThreadedSimRotor {
    shared: ThreadSharedHarness,
}

impl RotorSensor for ThreadedSimRotor {
    type Error = core::convert::Infallible;

    fn read_rotor(&mut self) -> Result<RotorReading, Self::Error> {
        let harness = self.shared.lock().unwrap();
        let state = *harness.plant.state();
        Ok(RotorReading {
            mechanical_angle: fluxkit::MechanicalAngle::new(
                state.mechanical_angle.get() + harness.rotor_bias,
            ),
            mechanical_velocity: state.mechanical_velocity,
        })
    }
}

#[derive(Clone, Debug)]
pub struct ThreadedSimOutput {
    shared: ThreadSharedHarness,
}

impl OutputSensor for ThreadedSimOutput {
    type Error = core::convert::Infallible;

    fn read_output(&mut self) -> Result<OutputReading, Self::Error> {
        let harness = self.shared.lock().unwrap();
        let state = *harness.plant.state();
        Ok(OutputReading {
            mechanical_angle: ContinuousMechanicalAngle::new(
                state.mechanical_angle.get() / GEAR_RATIO,
            )
            .wrapped(),
            mechanical_velocity: RadPerSec::new(state.mechanical_velocity.get() / GEAR_RATIO),
        })
    }
}

#[derive(Clone, Debug)]
pub struct ThreadedSimTemp {
    shared: ThreadSharedHarness,
}

impl TemperatureSensor for ThreadedSimTemp {
    type Error = core::convert::Infallible;

    fn sample_temperature_c(&mut self) -> Result<f32, Self::Error> {
        Ok(self.shared.lock().unwrap().plant.winding_temperature_c())
    }
}

pub fn threaded_calibration_hardware(
    shared: &ThreadSharedHarness,
) -> (
    ThreadedSimPwm,
    ThreadedSimCurrent,
    ThreadedSimBus,
    ThreadedSimRotor,
    ThreadedSimTemp,
) {
    (
        ThreadedSimPwm {
            shared: Arc::clone(shared),
        },
        ThreadedSimCurrent {
            shared: Arc::clone(shared),
        },
        ThreadedSimBus {
            shared: Arc::clone(shared),
        },
        ThreadedSimRotor {
            shared: Arc::clone(shared),
        },
        ThreadedSimTemp {
            shared: Arc::clone(shared),
        },
    )
}

pub fn threaded_runtime_hardware(
    shared: &ThreadSharedHarness,
) -> (
    ThreadedSimPwm,
    ThreadedSimCurrent,
    ThreadedSimBus,
    ThreadedSimRotor,
    ThreadedSimOutput,
    ThreadedSimTemp,
) {
    (
        ThreadedSimPwm {
            shared: Arc::clone(shared),
        },
        ThreadedSimCurrent {
            shared: Arc::clone(shared),
        },
        ThreadedSimBus {
            shared: Arc::clone(shared),
        },
        ThreadedSimRotor {
            shared: Arc::clone(shared),
        },
        ThreadedSimOutput {
            shared: Arc::clone(shared),
        },
        ThreadedSimTemp {
            shared: Arc::clone(shared),
        },
    )
}
