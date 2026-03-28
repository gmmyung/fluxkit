use std::{
    sync::{Arc, Mutex},
    thread,
};

use fluxkit::{
    Abc, ActuatorCompensationConfig, ActuatorLimits, ActuatorModel, ActuatorParams,
    BusVoltageSensor, ContinuousMechanicalAngle, CurrentLoopConfig, CurrentSampleValidity,
    CurrentSampler, ElectricalDirection, InverterParams, MotorCommand, MotorLimits, MotorModel,
    MotorParams, MotorRuntime, OutputReading, OutputSensor, PhaseCurrentSample, PhasePwm,
    RotorReading, RotorSensor, Svpwm, TemperatureSensor,
    units::{Amps, Duty, Henries, Hertz, NewtonMeters, Ohms, RadPerSec, Volts, Webers},
};
use fluxkit_hal::centered_phase_duty;
use fluxkit_math::{inverse_clarke, inverse_park};
use fluxkit_pmsm_sim::{ActuatorPlantParams, PmsmModel, PmsmParams, ThermalPlantParams};

const FAST_DT_SECONDS: f32 = 1.0 / 20_000.0;
const GEAR_RATIO: f32 = 2.0;
const WINDING_TEMP_C: f32 = 25.0;

fn motor_params() -> MotorParams {
    MotorParams::from_model_and_limits(
        MotorModel {
            pole_pairs: 7,
            phase_resistance_ohm_ref: Ohms::new(0.12),
            d_inductance_h: Henries::new(0.000_03),
            q_inductance_h: Henries::new(0.000_03),
            flux_linkage_weber: Webers::new(0.005),
            electrical_direction: ElectricalDirection::Positive,
            electrical_angle_offset: fluxkit::ElectricalAngle::new(0.0),
        },
        MotorLimits {
            max_phase_current: Amps::new(10.0),
            max_mech_speed: Some(RadPerSec::new(150.0)),
            max_winding_temperature_c: None,
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

fn plant_params() -> PmsmParams {
    PmsmParams {
        pole_pairs: 7,
        phase_resistance_ohm_ref: Ohms::new(0.12),
        d_inductance_h: Henries::new(0.000_03),
        q_inductance_h: Henries::new(0.000_03),
        flux_linkage_weber: Webers::new(0.005),
        electrical_direction: fluxkit_math::ElectricalDirection::Positive,
        thermal: ThermalPlantParams::default_for_ambient(WINDING_TEMP_C),
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
    last_duty: Abc<Duty>,
}

type SharedHarness = Arc<Mutex<SimHarness>>;

fn phase_currents(shared: &SharedHarness) -> Abc<Amps> {
    let harness = shared.lock().unwrap();
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
        Ok(self.shared.lock().unwrap().bus_voltage)
    }
}

#[derive(Clone, Debug)]
struct SimRotor {
    shared: SharedHarness,
}

impl RotorSensor for SimRotor {
    type Error = core::convert::Infallible;

    fn read_rotor(&mut self) -> Result<RotorReading, Self::Error> {
        let harness = self.shared.lock().unwrap();
        let state = *harness.plant.state();
        Ok(RotorReading {
            mechanical_angle: state.mechanical_angle.wrapped(),
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
struct SimTemp {
    shared: SharedHarness,
}

impl TemperatureSensor for SimTemp {
    type Error = core::convert::Infallible;

    fn sample_temperature_c(&mut self) -> Result<f32, Self::Error> {
        Ok(self.shared.lock().unwrap().plant.winding_temperature_c())
    }
}

#[test]
fn motor_runtime_closes_current_loop_against_simulator() {
    let shared = Arc::new(Mutex::new(SimHarness {
        plant: PmsmModel::new_zeroed(plant_params()).unwrap(),
        bus_voltage: Volts::new(24.0),
        load_torque: NewtonMeters::ZERO,
        last_duty: centered_phase_duty(),
    }));

    let runtime = MotorRuntime::new(
        SimPwm {
            shared: Arc::clone(&shared),
        },
        SimCurrent {
            shared: Arc::clone(&shared),
        },
        SimBus {
            shared: Arc::clone(&shared),
        },
        SimRotor {
            shared: Arc::clone(&shared),
        },
        SimOutput {
            shared: Arc::clone(&shared),
        },
        SimTemp {
            shared: Arc::clone(&shared),
        },
        fluxkit::MotorRuntimeParams::new(
            motor_params(),
            inverter_params(),
            actuator_params(),
            current_loop_config(),
            FAST_DT_SECONDS,
        ),
        Svpwm,
        fluxkit::PassThroughEstimator::new(),
        fluxkit::PassThroughEstimator::new(),
    );
    let (handle, ticker) = runtime.split().expect("runtime should split once");
    handle.set_command(MotorCommand::Current(fluxkit::Dq::new(
        Amps::ZERO,
        Amps::new(3.0),
    )));
    handle.arm();
    for _ in 0..4_000 {
        ticker.tick().unwrap();
        assert_eq!(handle.status().controller.active_error, None);
    }

    let status = handle.status().controller;
    assert!((status.last_measured_idq.q.get() - 3.0).abs() < 0.05);
    assert!(
        shared
            .lock()
            .unwrap()
            .plant
            .state()
            .mechanical_velocity
            .get()
            > 100.0
    );
}

#[test]
fn motor_runtime_supports_scoped_irq_thread_usage() {
    let shared = Arc::new(Mutex::new(SimHarness {
        plant: PmsmModel::new_zeroed(plant_params()).unwrap(),
        bus_voltage: Volts::new(24.0),
        load_torque: NewtonMeters::ZERO,
        last_duty: centered_phase_duty(),
    }));

    let runtime = MotorRuntime::new(
        SimPwm {
            shared: Arc::clone(&shared),
        },
        SimCurrent {
            shared: Arc::clone(&shared),
        },
        SimBus {
            shared: Arc::clone(&shared),
        },
        SimRotor {
            shared: Arc::clone(&shared),
        },
        SimOutput {
            shared: Arc::clone(&shared),
        },
        SimTemp {
            shared: Arc::clone(&shared),
        },
        fluxkit::MotorRuntimeParams::new(
            motor_params(),
            inverter_params(),
            actuator_params(),
            current_loop_config(),
            FAST_DT_SECONDS,
        ),
        Svpwm,
        fluxkit::PassThroughEstimator::new(),
        fluxkit::PassThroughEstimator::new(),
    );
    let (handle, ticker) = runtime.split().expect("runtime should split once");

    thread::scope(|scope| {
        let handle_for_thread = &handle;
        let ticker_for_thread = &ticker;
        let control_thread = scope.spawn(move || {
            handle_for_thread.set_command(MotorCommand::Current(fluxkit::Dq::new(
                Amps::ZERO,
                Amps::new(3.0),
            )));
            handle_for_thread.arm();

            for _ in 0..200 {
                let status = handle_for_thread.status();
                if status.controller.last_measured_idq.q.get() > 2.5 {
                    break;
                }
                thread::yield_now();
            }
        });

        let irq_thread = scope.spawn(move || {
            for _ in 0..4_000 {
                ticker_for_thread.tick().unwrap();
            }
        });

        control_thread.join().unwrap();
        irq_thread.join().unwrap();
    });

    assert_eq!(
        runtime.split().expect_err("runtime must not split twice"),
        fluxkit::CapabilitySplitError::AlreadySplit
    );
    let status = handle.status().controller;
    assert_eq!(status.active_error, None);
    assert_eq!(status.state, fluxkit::MotorState::Running);
    assert!((status.last_measured_idq.q.get() - 3.0).abs() < 0.05);
    assert!(status.last_rotor_mechanical_velocity.get() > 0.0);
}

#[test]
fn motor_runtime_supports_mit_command() {
    let shared = Arc::new(Mutex::new(SimHarness {
        plant: PmsmModel::new_zeroed(plant_params()).unwrap(),
        bus_voltage: Volts::new(24.0),
        load_torque: NewtonMeters::ZERO,
        last_duty: centered_phase_duty(),
    }));

    let runtime = MotorRuntime::new(
        SimPwm {
            shared: Arc::clone(&shared),
        },
        SimCurrent {
            shared: Arc::clone(&shared),
        },
        SimBus {
            shared: Arc::clone(&shared),
        },
        SimRotor {
            shared: Arc::clone(&shared),
        },
        SimOutput {
            shared: Arc::clone(&shared),
        },
        SimTemp {
            shared: Arc::clone(&shared),
        },
        fluxkit::MotorRuntimeParams::new(
            motor_params(),
            inverter_params(),
            actuator_params(),
            current_loop_config(),
            FAST_DT_SECONDS,
        ),
        Svpwm,
        fluxkit::PassThroughEstimator::new(),
        fluxkit::PassThroughEstimator::new(),
    );
    let (handle, ticker) = runtime.split().expect("runtime should split once");
    handle.set_command(MotorCommand::Mit {
        position: ContinuousMechanicalAngle::new(1.0),
        velocity: RadPerSec::ZERO,
        kp: 5.0,
        kd: 0.2,
        torque_ff: NewtonMeters::ZERO,
    });
    handle.arm();
    for _ in 0..8_000 {
        ticker.tick().unwrap();
    }

    let status = handle.status().controller;
    assert_eq!(status.mode, fluxkit::ControlMode::Mit);
    assert!(status.last_output_mechanical_angle.get() > 0.05);
    assert!(status.last_measured_idq.q.get() > 0.0);
}

#[test]
fn motor_runtime_velocity_mode_tracks_positive_output_speed_with_negative_direction() {
    let mut plant = plant_params();
    plant.electrical_direction = fluxkit_math::ElectricalDirection::Negative;
    let shared = Arc::new(Mutex::new(SimHarness {
        plant: PmsmModel::new_zeroed(plant).unwrap(),
        bus_voltage: Volts::new(24.0),
        load_torque: NewtonMeters::ZERO,
        last_duty: centered_phase_duty(),
    }));

    let mut motor = motor_params();
    motor.electrical_direction = ElectricalDirection::Negative;
    let runtime = MotorRuntime::new(
        SimPwm {
            shared: Arc::clone(&shared),
        },
        SimCurrent {
            shared: Arc::clone(&shared),
        },
        SimBus {
            shared: Arc::clone(&shared),
        },
        SimRotor {
            shared: Arc::clone(&shared),
        },
        SimOutput {
            shared: Arc::clone(&shared),
        },
        SimTemp {
            shared: Arc::clone(&shared),
        },
        fluxkit::MotorRuntimeParams::new(
            motor,
            inverter_params(),
            actuator_params(),
            current_loop_config(),
            FAST_DT_SECONDS,
        ),
        Svpwm,
        fluxkit::PassThroughEstimator::new(),
        fluxkit::PassThroughEstimator::new(),
    );
    let (handle, ticker) = runtime.split().expect("runtime should split once");
    handle.set_command(MotorCommand::Velocity(RadPerSec::new(10.0)));
    handle.arm();
    for _ in 0..8_000 {
        ticker.tick().unwrap();
        assert_eq!(handle.status().controller.active_error, None);
    }

    let status = handle.status().controller;
    assert!(status.last_output_mechanical_velocity.get() > 5.0);
}
