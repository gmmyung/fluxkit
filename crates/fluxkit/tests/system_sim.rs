use std::{cell::RefCell, rc::Rc};

use fluxkit::{
    ActuatorCompensationConfig, ActuatorParams, ControlMode, CurrentLoopConfig, InverterParams,
    MotorController, MotorHardware, MotorParams, MotorSystem, centered_phase_duty,
    hal::{
        BusVoltageSensor, CurrentSampleValidity, CurrentSampler, OutputReading, OutputSensor,
        PhaseCurrentSample, PhasePwm, RotorReading, RotorSensor,
    },
    math::{
        MechanicalAngle, inverse_clarke, inverse_park,
        units::{Amps, Duty, Henries, Hertz, NewtonMeters, Ohms, RadPerSec, Volts, Webers},
    },
};
use fluxkit_pmsm_sim::{ActuatorPlantParams, PmsmModel, PmsmParams};

const FAST_DT_SECONDS: f32 = 1.0 / 20_000.0;
const GEAR_RATIO: f32 = 2.0;

fn motor_params() -> MotorParams {
    MotorParams {
        pole_pairs: 7,
        phase_resistance_ohm: Ohms::new(0.12),
        d_inductance_h: Henries::new(0.000_03),
        q_inductance_h: Henries::new(0.000_03),
        flux_linkage_weber: Some(Webers::new(0.005)),
        electrical_angle_offset: fluxkit::ElectricalAngle::new(0.0),
        max_phase_current: Amps::new(10.0),
        max_mech_speed: Some(RadPerSec::new(150.0)),
    }
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

fn current_loop_config() -> CurrentLoopConfig {
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

fn actuator_params() -> ActuatorParams {
    ActuatorParams {
        gear_ratio: GEAR_RATIO,
        max_output_velocity: Some(RadPerSec::new(30.0)),
        max_output_torque: Some(NewtonMeters::new(10.0)),
        compensation: ActuatorCompensationConfig::disabled(),
    }
}

fn plant_params() -> PmsmParams {
    PmsmParams {
        pole_pairs: 7,
        phase_resistance_ohm: Ohms::new(0.12),
        d_inductance_h: Henries::new(0.000_03),
        q_inductance_h: Henries::new(0.000_03),
        flux_linkage_weber: Webers::new(0.005),
        actuator: ActuatorPlantParams {
            gear_ratio: GEAR_RATIO,
            output_inertia_kg_m2: 0.0008,
            positive_viscous_coefficient: 0.0002,
            negative_viscous_coefficient: 0.0002,
            ..ActuatorPlantParams::disabled()
        },
        max_voltage_mag: None,
    }
}

#[derive(Debug)]
struct SimHarness {
    plant: PmsmModel,
    bus_voltage: Volts,
    load_torque: NewtonMeters,
    last_duty: fluxkit::math::frame::Abc<Duty>,
}

type SharedHarness = Rc<RefCell<SimHarness>>;

fn phase_currents(shared: &SharedHarness) -> fluxkit::math::frame::Abc<Amps> {
    let harness = shared.borrow();
    let state = *harness.plant.state();
    let electrical_angle = fluxkit::math::angle::mechanical_to_electrical(
        state.mechanical_angle.wrapped_0_2pi(),
        harness.plant.params().pole_pairs as u32,
    )
    .wrapped_pm_pi();
    inverse_clarke(inverse_park(
        state.current_dq.map(|current| current.get()),
        electrical_angle.get(),
    ))
    .map(Amps::new)
}

#[derive(Clone, Debug)]
struct SimPwm {
    shared: SharedHarness,
}

impl PhasePwm for SimPwm {
    type Error = core::convert::Infallible;

    fn enable(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }

    fn disable(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }

    fn set_duty(&mut self, a: Duty, b: Duty, c: Duty) -> Result<(), Self::Error> {
        let duty = fluxkit::math::frame::Abc::new(a, b, c);
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
struct SimCurrent {
    shared: SharedHarness,
}

impl CurrentSampler for SimCurrent {
    type Error = core::convert::Infallible;

    fn sample_phase_currents(&mut self) -> Result<PhaseCurrentSample, Self::Error> {
        Ok(PhaseCurrentSample {
            currents: phase_currents(&self.shared),
            validity: CurrentSampleValidity::Valid,
        })
    }
}

#[derive(Clone, Debug)]
struct SimBus {
    shared: SharedHarness,
}

impl BusVoltageSensor for SimBus {
    type Error = core::convert::Infallible;

    fn sample_bus_voltage(&mut self) -> Result<Volts, Self::Error> {
        Ok(self.shared.borrow().bus_voltage)
    }
}

#[derive(Clone, Debug)]
struct SimRotor {
    shared: SharedHarness,
}

impl RotorSensor for SimRotor {
    type Error = core::convert::Infallible;

    fn read_rotor(&mut self) -> Result<RotorReading, Self::Error> {
        let harness = self.shared.borrow();
        let state = *harness.plant.state();
        Ok(RotorReading {
            mechanical_angle: state.mechanical_angle.wrapped_0_2pi(),
            mechanical_velocity: state.mechanical_velocity,
        })
    }
}

#[derive(Clone, Debug)]
struct SimOutput {
    shared: SharedHarness,
}

impl OutputSensor for SimOutput {
    type Error = core::convert::Infallible;

    fn read_output(&mut self) -> Result<OutputReading, Self::Error> {
        let harness = self.shared.borrow();
        let state = *harness.plant.state();
        Ok(OutputReading {
            mechanical_angle: MechanicalAngle::new(state.mechanical_angle.get() / GEAR_RATIO)
                .wrapped_0_2pi(),
            mechanical_velocity: RadPerSec::new(state.mechanical_velocity.get() / GEAR_RATIO),
        })
    }
}

#[test]
fn motor_system_closes_current_loop_against_simulator() {
    let shared = Rc::new(RefCell::new(SimHarness {
        plant: PmsmModel::new_zeroed(plant_params()).unwrap(),
        bus_voltage: Volts::new(24.0),
        load_torque: NewtonMeters::ZERO,
        last_duty: centered_phase_duty(),
    }));

    let hardware = MotorHardware {
        pwm: SimPwm {
            shared: Rc::clone(&shared),
        },
        current: SimCurrent {
            shared: Rc::clone(&shared),
        },
        bus: SimBus {
            shared: Rc::clone(&shared),
        },
        rotor: SimRotor {
            shared: Rc::clone(&shared),
        },
        output: SimOutput {
            shared: Rc::clone(&shared),
        },
    };
    let controller = MotorController::new(
        motor_params(),
        inverter_params(),
        actuator_params(),
        current_loop_config(),
    );
    let mut system = MotorSystem::new(hardware, controller);
    system.controller_mut().set_mode(ControlMode::Current);
    system.controller_mut().set_iq_target(Amps::new(3.0));
    system.enable().unwrap();

    for _ in 0..4_000 {
        let output = system.fast_tick(FAST_DT_SECONDS).unwrap();
        assert_eq!(output.error, None);
    }

    let status = system.controller().status();
    assert!((status.last_measured_idq.q.get() - 3.0).abs() < 0.05);
    assert!(shared.borrow().plant.state().mechanical_velocity.get() > 100.0);
}
