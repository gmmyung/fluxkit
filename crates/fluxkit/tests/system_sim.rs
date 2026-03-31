mod support;

use std::{
    sync::{Arc, Mutex},
    thread,
};

use fluxkit::{
    ContinuousMechanicalAngle, ElectricalDirection, MotorCommand, MotorRuntime, Svpwm,
    units::{Amps, Henries, NewtonMeters, Ohms, RadPerSec, Volts, Webers},
};
use fluxkit_hal::centered_phase_duty;
use fluxkit_pmsm_sim::{ActuatorPlantParams, PmsmModel, PmsmParams, ThermalPlantParams};
use support::sim::{
    FAST_DT_SECONDS, GEAR_RATIO, SimHarness, WINDING_TEMP_C,
    controller_motor_params as motor_params, current_loop_config, inverter_params,
    runtime_actuator_params as actuator_params, threaded_runtime_hardware as runtime_hardware,
};

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

#[test]
fn motor_runtime_closes_current_loop_against_simulator() {
    let shared = Arc::new(Mutex::new(SimHarness {
        plant: PmsmModel::new_zeroed(plant_params()).unwrap(),
        bus_voltage: Volts::new(24.0),
        load_torque: NewtonMeters::ZERO,
        last_duty: centered_phase_duty(),
        rotor_bias: 0.0,
    }));

    let (pwm, current, bus, rotor, output, temp) = runtime_hardware(&shared);
    let runtime = MotorRuntime::new(
        pwm,
        current,
        bus,
        rotor,
        output,
        temp,
        fluxkit::MotorRuntimeParams::new(
            motor_params(),
            inverter_params(),
            actuator_params(),
            current_loop_config(),
            FAST_DT_SECONDS,
        ),
        Svpwm,
        fluxkit::PassThroughCurrentEstimator::new(),
        fluxkit::PassThroughEstimator::new(),
        fluxkit::PassThroughEstimator::new(),
    )
    .expect("valid runtime config");
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
        rotor_bias: 0.0,
    }));

    let (pwm, current, bus, rotor, output, temp) = runtime_hardware(&shared);
    let runtime = MotorRuntime::new(
        pwm,
        current,
        bus,
        rotor,
        output,
        temp,
        fluxkit::MotorRuntimeParams::new(
            motor_params(),
            inverter_params(),
            actuator_params(),
            current_loop_config(),
            FAST_DT_SECONDS,
        ),
        Svpwm,
        fluxkit::PassThroughCurrentEstimator::new(),
        fluxkit::PassThroughEstimator::new(),
        fluxkit::PassThroughEstimator::new(),
    )
    .expect("valid runtime config");
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
        rotor_bias: 0.0,
    }));

    let (pwm, current, bus, rotor, output, temp) = runtime_hardware(&shared);
    let runtime = MotorRuntime::new(
        pwm,
        current,
        bus,
        rotor,
        output,
        temp,
        fluxkit::MotorRuntimeParams::new(
            motor_params(),
            inverter_params(),
            actuator_params(),
            current_loop_config(),
            FAST_DT_SECONDS,
        ),
        Svpwm,
        fluxkit::PassThroughCurrentEstimator::new(),
        fluxkit::PassThroughEstimator::new(),
        fluxkit::PassThroughEstimator::new(),
    )
    .expect("valid runtime config");
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
        rotor_bias: 0.0,
    }));

    let mut motor = motor_params();
    motor.electrical_direction = ElectricalDirection::Negative;
    let (pwm, current, bus, rotor, output, temp) = runtime_hardware(&shared);
    let runtime = MotorRuntime::new(
        pwm,
        current,
        bus,
        rotor,
        output,
        temp,
        fluxkit::MotorRuntimeParams::new(
            motor,
            inverter_params(),
            actuator_params(),
            current_loop_config(),
            FAST_DT_SECONDS,
        ),
        Svpwm,
        fluxkit::PassThroughCurrentEstimator::new(),
        fluxkit::PassThroughEstimator::new(),
        fluxkit::PassThroughEstimator::new(),
    )
    .expect("valid runtime config");
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
