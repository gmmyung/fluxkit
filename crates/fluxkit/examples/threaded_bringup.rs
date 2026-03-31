use core::cell::RefCell;
use std::{convert::Infallible, env, fs, thread};

use critical_section::Mutex;
use fluxkit::{
    Abc, ActuatorCalibrationLimits, ActuatorCalibrationRequest, ActuatorCalibrationRuntime,
    ActuatorLimits, ActuatorParams, BusVoltageSensor, ContinuousMechanicalAngle, CurrentLoopConfig,
    CurrentSampleValidity, CurrentSampler, Dq, InverterParams, MotorCalibrationLimits,
    MotorCalibrationRequest, MotorCalibrationResult, MotorCalibrationRuntime, MotorCommand,
    MotorLimits, MotorRuntime, OutputReading, OutputSensor, PassThroughCurrentEstimator,
    PassThroughEstimator, PhaseCurrentSample, PhasePwm, RadPerSec, RotorReading, RotorSensor,
    Svpwm, TemperatureSensor,
    units::{Amps, Duty, Henries, Hertz, NewtonMeters, Ohms, Volts, Webers},
};
use fluxkit_hal::centered_phase_duty;
use fluxkit_math::{inverse_clarke, inverse_park};
use fluxkit_pmsm_sim::{ActuatorPlantParams, PmsmModel, PmsmParams, PmsmState, ThermalPlantParams};
use plotters::prelude::*;
use static_cell::StaticCell;

const FAST_DT_SECONDS: f32 = 1.0 / 20_000.0;
const GEAR_RATIO: f32 = 2.0;
const RUNTIME_FAST_CYCLES: u32 = 100_000;
const WINDING_TEMP_C: f32 = 25.0;

#[derive(Debug)]
struct SharedCell<T> {
    inner: Mutex<RefCell<T>>,
}

impl<T> SharedCell<T> {
    const fn new(value: T) -> Self {
        Self {
            inner: Mutex::new(RefCell::new(value)),
        }
    }

    fn with<R>(&self, f: impl FnOnce(&T) -> R) -> R {
        critical_section::with(|cs| {
            let value = self.inner.borrow(cs).borrow();
            f(&value)
        })
    }

    fn with_mut<R>(&self, f: impl FnOnce(&mut T) -> R) -> R {
        critical_section::with(|cs| {
            let mut value = self.inner.borrow(cs).borrow_mut();
            f(&mut value)
        })
    }
}

#[derive(Debug)]
struct SimHarness {
    plant: PmsmModel,
    bus_voltage: Volts,
    load_torque: NewtonMeters,
    last_duty: Abc<Duty>,
    rotor_bias: f32,
}

fn plant_params() -> PmsmParams {
    PmsmParams {
        pole_pairs: 7,
        phase_resistance_ohm_ref: Ohms::new(0.12),
        d_inductance_h: Henries::new(0.000_03),
        q_inductance_h: Henries::new(0.000_03),
        flux_linkage_weber: Webers::new(0.005),
        electrical_direction: fluxkit::ElectricalDirection::Positive,
        thermal: ThermalPlantParams::default_for_ambient(WINDING_TEMP_C),
        actuator: ActuatorPlantParams {
            gear_ratio: GEAR_RATIO,
            output_inertia_kg_m2: 0.0208,
            positive_breakaway_torque: NewtonMeters::new(0.08),
            negative_breakaway_torque: NewtonMeters::new(0.09),
            positive_coulomb_torque: NewtonMeters::new(0.04),
            negative_coulomb_torque: NewtonMeters::new(0.05),
            positive_viscous_coefficient: 0.02,
            negative_viscous_coefficient: 0.03,
            zero_velocity_blend_band: RadPerSec::new(0.05),
        },
        max_voltage_mag: None,
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

fn current_loop_config(motor_calibration: MotorCalibrationResult) -> CurrentLoopConfig {
    CurrentLoopConfig::builder(
        RadPerSec::new(3_333.3333),
        motor_calibration.phase_inductance_h,
        motor_calibration.phase_resistance_ohm_ref,
    )
    .velocity_gains(0.2, 8.0)
    .position_gains(12.0, 0.0)
    .max_voltage_mag(Volts::new(12.0))
    .id_ref_default(Amps::ZERO)
    .max_id_target(Amps::new(5.0))
    .max_iq_target(Amps::new(8.0))
    .max_velocity_target(RadPerSec::new(120.0))
    .max_current_ref_derivative_amps_per_sec(10_000.0)
    .current_feedforward(true)
    .build()
}

fn phase_currents(shared: &SharedCell<SimHarness>) -> Abc<Amps> {
    shared.with(|harness| {
        let state = *harness.plant.state();
        let electrical_angle = fluxkit::angle::mechanical_to_electrical(
            state.mechanical_angle.wrapped().into(),
            harness.plant.params().pole_pairs as u32,
        );
        inverse_clarke(inverse_park(
            state.current_dq.map(|current| current.get()),
            electrical_angle.get(),
        ))
        .map(Amps::new)
    })
}

#[derive(Clone, Copy, Debug)]
struct SimPwm<'a> {
    shared: &'a SharedCell<SimHarness>,
}

impl PhasePwm for SimPwm<'_> {
    type Error = Infallible;

    fn enable(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }

    fn disable(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }

    fn set_duty(&mut self, a: Duty, b: Duty, c: Duty) -> Result<(), Self::Error> {
        let duty = Abc::new(a, b, c);
        self.shared.with_mut(|harness| {
            harness.last_duty = duty;
            let bus_voltage = harness.bus_voltage;
            let load_torque = harness.load_torque;
            harness
                .plant
                .step_phase_duty(duty, bus_voltage, load_torque, FAST_DT_SECONDS)
                .unwrap();
        });
        Ok(())
    }
}

#[derive(Clone, Copy, Debug)]
struct SimCurrent<'a> {
    shared: &'a SharedCell<SimHarness>,
}

impl CurrentSampler for SimCurrent<'_> {
    type Error = Infallible;

    fn sample_phase_currents(&mut self) -> Result<PhaseCurrentSample, Self::Error> {
        Ok(PhaseCurrentSample {
            currents: phase_currents(self.shared),
            validity: CurrentSampleValidity::Valid,
        })
    }
}

#[derive(Clone, Copy, Debug)]
struct SimBus<'a> {
    shared: &'a SharedCell<SimHarness>,
}

impl BusVoltageSensor for SimBus<'_> {
    type Error = Infallible;

    fn sample_bus_voltage(&mut self) -> Result<Volts, Self::Error> {
        Ok(self.shared.with(|harness| harness.bus_voltage))
    }
}

#[derive(Clone, Copy, Debug)]
struct SimRotor<'a> {
    shared: &'a SharedCell<SimHarness>,
}

impl RotorSensor for SimRotor<'_> {
    type Error = Infallible;

    fn read_rotor(&mut self) -> Result<RotorReading, Self::Error> {
        Ok(self.shared.with(|harness| {
            let state = *harness.plant.state();
            RotorReading {
                mechanical_angle: ContinuousMechanicalAngle::new(
                    state.mechanical_angle.get() + harness.rotor_bias,
                )
                .wrapped(),
                mechanical_velocity: state.mechanical_velocity,
            }
        }))
    }
}

#[derive(Clone, Copy, Debug)]
struct SimOutput<'a> {
    shared: &'a SharedCell<SimHarness>,
}

impl OutputSensor for SimOutput<'_> {
    type Error = Infallible;

    fn read_output(&mut self) -> Result<OutputReading, Self::Error> {
        Ok(self.shared.with(|harness| {
            let state = *harness.plant.state();
            OutputReading {
                mechanical_angle: ContinuousMechanicalAngle::new(
                    state.mechanical_angle.get() / GEAR_RATIO,
                )
                .wrapped(),
                mechanical_velocity: RadPerSec::new(state.mechanical_velocity.get() / GEAR_RATIO),
            }
        }))
    }
}

#[derive(Clone, Copy, Debug)]
struct SimTemp<'a> {
    shared: &'a SharedCell<SimHarness>,
}

impl TemperatureSensor for SimTemp<'_> {
    type Error = Infallible;

    fn sample_temperature_c(&mut self) -> Result<f32, Self::Error> {
        Ok(self
            .shared
            .with(|harness| harness.plant.winding_temperature_c()))
    }
}

fn calibration_hardware<'a>(
    shared: &'a SharedCell<SimHarness>,
) -> (
    SimPwm<'a>,
    SimCurrent<'a>,
    SimBus<'a>,
    SimRotor<'a>,
    SimTemp<'a>,
) {
    (
        SimPwm { shared },
        SimCurrent { shared },
        SimBus { shared },
        SimRotor { shared },
        SimTemp { shared },
    )
}

fn output_velocity(shared: &SharedCell<SimHarness>) -> f32 {
    shared.with(|harness| harness.plant.state().mechanical_velocity.get() / GEAR_RATIO)
}

type ExampleMotorCalibrationRuntime<'a> = MotorCalibrationRuntime<
    SimPwm<'a>,
    SimCurrent<'a>,
    SimBus<'a>,
    SimRotor<'a>,
    SimTemp<'a>,
    Svpwm,
    PassThroughEstimator,
>;
type ExampleActuatorCalibrationRuntime<'a> = ActuatorCalibrationRuntime<
    SimPwm<'a>,
    SimCurrent<'a>,
    SimBus<'a>,
    SimRotor<'a>,
    SimOutput<'a>,
    SimTemp<'a>,
    Svpwm,
    PassThroughCurrentEstimator,
    PassThroughEstimator,
    PassThroughEstimator,
>;
type ExampleMotorRuntime<'a> = MotorRuntime<
    SimPwm<'a>,
    SimCurrent<'a>,
    SimBus<'a>,
    SimRotor<'a>,
    SimOutput<'a>,
    SimTemp<'a>,
    Svpwm,
    PassThroughCurrentEstimator,
    PassThroughEstimator,
    PassThroughEstimator,
>;
type ExampleMotorCalibrationTicker<'a> = fluxkit::MotorCalibrationTicker<
    'a,
    SimPwm<'a>,
    SimCurrent<'a>,
    SimBus<'a>,
    SimRotor<'a>,
    SimTemp<'a>,
    Svpwm,
    PassThroughEstimator,
>;
type ExampleActuatorCalibrationHandle<'a> = fluxkit::ActuatorCalibrationHandle<'a>;
type ExampleActuatorCalibrationTicker<'a> = fluxkit::ActuatorCalibrationTicker<
    'a,
    SimPwm<'a>,
    SimCurrent<'a>,
    SimBus<'a>,
    SimRotor<'a>,
    SimOutput<'a>,
    SimTemp<'a>,
    Svpwm,
    PassThroughCurrentEstimator,
    PassThroughEstimator,
    PassThroughEstimator,
>;
type ExampleMotorHandle<'a> = fluxkit::MotorHandle<'a>;
type ExampleMotorTicker<'a> = fluxkit::MotorTicker<
    'a,
    SimPwm<'a>,
    SimCurrent<'a>,
    SimBus<'a>,
    SimRotor<'a>,
    SimOutput<'a>,
    SimTemp<'a>,
    Svpwm,
    PassThroughCurrentEstimator,
    PassThroughEstimator,
    PassThroughEstimator,
>;
enum AppState<'a> {
    Empty,
    MotorCalibrationRunning(ExampleMotorCalibrationTicker<'a>, u32),
    ActuatorCalibrationRunning(ExampleActuatorCalibrationTicker<'a>, u32),
    RuntimeRunning(ExampleMotorTicker<'a>, u32, Vec<(f32, f32)>),
    Finished,
    Faulted,
}

#[derive(Clone, Copy, Default)]
struct GlobalRefs {
    shared: Option<&'static SharedCell<SimHarness>>,
    app_state: Option<&'static SharedCell<AppState<'static>>>,
}

static SHARED: StaticCell<SharedCell<SimHarness>> = StaticCell::new();
static APP_STATE: StaticCell<SharedCell<AppState<'static>>> = StaticCell::new();
static MOTOR_CALIBRATION_RUNTIME: StaticCell<ExampleMotorCalibrationRuntime<'static>> =
    StaticCell::new();
static ACTUATOR_CALIBRATION_RUNTIME: StaticCell<ExampleActuatorCalibrationRuntime<'static>> =
    StaticCell::new();
static MOTOR_RUNTIME: StaticCell<ExampleMotorRuntime<'static>> = StaticCell::new();
static GLOBAL_REFS: SharedCell<GlobalRefs> = SharedCell::new(GlobalRefs {
    shared: None,
    app_state: None,
});

fn draw_velocity_plot(
    path: &str,
    samples: &[(f32, f32)],
) -> Result<(), Box<dyn std::error::Error>> {
    let root = SVGBackend::new(path, (960, 480)).into_drawing_area();
    root.fill(&WHITE)?;

    let end_time = samples
        .last()
        .map(|sample| sample.0)
        .unwrap_or(1.0)
        .max(1.0e-6);
    let (mut min_velocity, mut max_velocity) = (f32::INFINITY, f32::NEG_INFINITY);
    for &(_, velocity) in samples {
        min_velocity = min_velocity.min(velocity);
        max_velocity = max_velocity.max(velocity);
    }
    if !min_velocity.is_finite() || !max_velocity.is_finite() {
        min_velocity = -1.0;
        max_velocity = 1.0;
    }
    if (max_velocity - min_velocity).abs() < 1.0e-6 {
        min_velocity -= 1.0;
        max_velocity += 1.0;
    }

    let mut chart = ChartBuilder::on(&root)
        .caption("Threaded Bring-up Output Velocity", ("sans-serif", 24))
        .margin(16)
        .x_label_area_size(40)
        .y_label_area_size(70)
        .build_cartesian_2d(0.0_f32..end_time, min_velocity..max_velocity)?;

    chart
        .configure_mesh()
        .disable_mesh()
        .x_desc("time (s)")
        .y_desc("output velocity (rad/s)")
        .draw()?;

    chart.draw_series(LineSeries::new(samples.iter().copied(), &BLUE))?;
    root.present()?;
    Ok(())
}

fn irq_loop<'a>(app_state: &SharedCell<AppState<'a>>, shared: &SharedCell<SimHarness>) {
    loop {
        let state = app_state.with_mut(|slot| core::mem::replace(slot, AppState::Empty));
        let (next_state, should_exit) = match state {
            AppState::Empty => (AppState::Empty, false),
            AppState::MotorCalibrationRunning(ticker, mut cycles) => {
                cycles = cycles.wrapping_add(1);
                match ticker.tick() {
                    Ok(()) => {
                        if cycles % 20_000 == 0 {
                            println!("irq: motor calibration still running ({cycles} cycles)");
                        }
                        (AppState::MotorCalibrationRunning(ticker, cycles), false)
                    }
                    Err(error) => {
                        eprintln!("irq: motor calibration failed: {error}");
                        (AppState::Faulted, false)
                    }
                }
            }
            AppState::ActuatorCalibrationRunning(ticker, mut cycles) => {
                cycles = cycles.wrapping_add(1);
                match ticker.tick() {
                    Ok(()) => {
                        if cycles % 20_000 == 0 {
                            println!("irq: actuator calibration still running ({cycles} cycles)");
                        }
                        (AppState::ActuatorCalibrationRunning(ticker, cycles), false)
                    }
                    Err(error) => {
                        eprintln!("irq: actuator calibration failed: {error}");
                        (AppState::Faulted, false)
                    }
                }
            }
            AppState::RuntimeRunning(ticker, mut cycles, mut samples) => {
                cycles = cycles.wrapping_add(1);
                match ticker.tick() {
                    Ok(()) => {
                        samples.push((cycles as f32 * FAST_DT_SECONDS, output_velocity(shared)));
                        if cycles % 20_000 == 0 {
                            println!("irq: runtime executed {cycles} cycles");
                        }
                        (AppState::RuntimeRunning(ticker, cycles, samples), false)
                    }
                    Err(error) => {
                        eprintln!("irq: runtime failed: {error}");
                        (AppState::Faulted, false)
                    }
                }
            }
            AppState::Finished => (AppState::Finished, true),
            AppState::Faulted => (AppState::Faulted, true),
        };
        app_state.with_mut(|slot| *slot = next_state);

        if should_exit {
            break;
        }
        thread::yield_now();
    }
}

fn init_globals() -> GlobalRefs {
    let shared: &'static SharedCell<SimHarness> = SHARED.init(SharedCell::new(SimHarness {
        plant: PmsmModel::new(
            plant_params(),
            PmsmState {
                mechanical_angle: ContinuousMechanicalAngle::new(0.4),
                mechanical_velocity: RadPerSec::ZERO,
                current_dq: Dq::new(Amps::ZERO, Amps::ZERO),
                winding_temperature_c: WINDING_TEMP_C,
            },
        )
        .unwrap(),
        bus_voltage: Volts::new(24.0),
        load_torque: NewtonMeters::ZERO,
        last_duty: centered_phase_duty(),
        rotor_bias: 0.18,
    }));
    let app_state: &'static SharedCell<AppState<'static>> =
        APP_STATE.init(SharedCell::new(AppState::Empty));

    let refs = GlobalRefs {
        shared: Some(shared),
        app_state: Some(app_state),
    };
    GLOBAL_REFS.with_mut(|slot| *slot = refs);
    refs
}

fn main_context_loop(plot_path: &str) {
    let GlobalRefs {
        shared: Some(shared),
        app_state: Some(app_state),
    } = init_globals()
    else {
        unreachable!();
    };

    let mut motor_params = None;
    let mut actuator_calibration_handle: Option<ExampleActuatorCalibrationHandle<'static>> = None;
    let mut actuator_calibration_owner: Option<
        &'static ExampleActuatorCalibrationRuntime<'static>,
    > = None;
    let mut runtime_handle: Option<ExampleMotorHandle<'static>> = None;
    let mut runtime_owner: Option<&'static ExampleMotorRuntime<'static>> = None;
    let mut runtime_target_reported = false;

    println!("phase 1: motor calibration");
    let (pwm, current, bus, rotor, temp) = calibration_hardware(shared);
    let motor_calibration = MotorCalibrationRuntime::new(
        pwm,
        current,
        bus,
        rotor,
        temp,
        Svpwm,
        PassThroughEstimator::new(),
        MotorCalibrationRequest::all(),
        MotorCalibrationLimits {
            max_align_voltage_mag: Volts::new(2.0),
            max_spin_voltage_mag: Volts::new(3.0),
            max_electrical_velocity: RadPerSec::new(60.0),
            timeout_seconds: 6.0,
        },
        FAST_DT_SECONDS,
    )
    .unwrap();
    let motor_calibration_owner = MOTOR_CALIBRATION_RUNTIME.init(motor_calibration);
    let (handle, ticker) = motor_calibration_owner.split().unwrap();
    let mut motor_calibration_handle = Some(handle);
    app_state.with_mut(|slot| *slot = AppState::MotorCalibrationRunning(ticker, 0));

    loop {
        if motor_params.is_none() {
            let motor_result = motor_calibration_handle
                .as_ref()
                .and_then(|handle| handle.status().result);
            if let Some(result) = motor_result {
                let parts = loop {
                    let taken = app_state.with_mut(|slot| {
                        match core::mem::replace(slot, AppState::Empty) {
                            AppState::MotorCalibrationRunning(ticker, cycles) => {
                                match motor_calibration_owner.try_into_parts() {
                                    Some(parts) => Some(parts),
                                    None => {
                                        *slot = AppState::MotorCalibrationRunning(ticker, cycles);
                                        None
                                    }
                                }
                            }
                            other => {
                                *slot = other;
                                None
                            }
                        }
                    });
                    if let Some(parts) = taken {
                        break parts;
                    }
                    thread::yield_now();
                };
                println!(
                    "main: received motor calibration result: pole_pairs={}, R={:.4} ohm, L={:.8} H, psi={:.6} Wb, offset={:.4} rad",
                    result.pole_pairs,
                    result.phase_resistance_ohm_ref.get(),
                    result.phase_inductance_h.get(),
                    result.flux_linkage_weber.get(),
                    result.electrical_angle_offset.get(),
                );
                motor_params = Some(result.into_motor_params(MotorLimits {
                    max_phase_current: Amps::new(10.0),
                    max_mech_speed: Some(RadPerSec::new(150.0)),
                    max_winding_temperature_c: None,
                }));
                motor_calibration_handle = None;

                println!("phase 2: actuator calibration");
                let actuator_calibration = ActuatorCalibrationRuntime::from_parts(
                    fluxkit::MotorRuntimeParts {
                        pwm: parts.pwm,
                        current: parts.current,
                        bus: parts.bus,
                        rotor: parts.rotor,
                        output: SimOutput { shared },
                        temp: parts.temp,
                        motor: motor_params.unwrap(),
                        inverter: inverter_params(),
                        actuator: ActuatorParams::from_model_limits_and_compensation(
                            fluxkit::ActuatorModel { gear_ratio: 1.0 },
                            ActuatorLimits {
                                max_output_velocity: Some(RadPerSec::new(10.0)),
                                max_output_torque: None,
                            },
                            fluxkit::ActuatorCompensationConfig::disabled(),
                        ),
                        current_loop: current_loop_config(result),
                        modulator: parts.modulator,
                        current_estimator: PassThroughCurrentEstimator::new(),
                        rotor_estimator: parts.rotor_estimator,
                        output_estimator: PassThroughEstimator::new(),
                    },
                    ActuatorCalibrationRequest::all(),
                    ActuatorCalibrationLimits {
                        max_velocity_target: RadPerSec::new(10.0),
                        max_torque_target: NewtonMeters::new(0.3),
                        timeout_seconds: 20.0,
                    },
                    FAST_DT_SECONDS,
                )
                .unwrap();
                let actuator_calibration = ACTUATOR_CALIBRATION_RUNTIME.init(actuator_calibration);
                let (handle, ticker) = actuator_calibration.split().unwrap();
                actuator_calibration_handle = Some(handle);
                actuator_calibration_owner = Some(actuator_calibration);
                app_state.with_mut(|slot| *slot = AppState::ActuatorCalibrationRunning(ticker, 0));
            }
        } else if !app_state.with(|slot| matches!(slot, AppState::RuntimeRunning(..))) {
            let actuator_result = actuator_calibration_handle
                .as_ref()
                .and_then(|handle| handle.status().result);
            if let Some(result) = actuator_result {
                let parts = loop {
                    let taken = app_state.with_mut(|slot| {
                        match core::mem::replace(slot, AppState::Empty) {
                            AppState::ActuatorCalibrationRunning(ticker, cycles) => {
                                let owner = actuator_calibration_owner
                                    .expect("actuator calibration owner should be initialized");
                                match owner.try_into_parts() {
                                    Some(parts) => Some(parts),
                                    None => {
                                        *slot =
                                            AppState::ActuatorCalibrationRunning(ticker, cycles);
                                        None
                                    }
                                }
                            }
                            other => {
                                *slot = other;
                                None
                            }
                        }
                    });
                    if let Some(parts) = taken {
                        break parts;
                    }
                    thread::yield_now();
                };
                println!(
                    "main: received actuator calibration result: gear_ratio={:.4}, breakaway=({:.4}, {:.4}) Nm, coulomb=({:.4}, {:.4}) Nm, viscous=({:.4}, {:.4}), blend={:.4} rad/s",
                    result.gear_ratio,
                    result.positive_breakaway_torque.get(),
                    result.negative_breakaway_torque.get(),
                    result.positive_coulomb_torque.get(),
                    result.negative_coulomb_torque.get(),
                    result.positive_viscous_coefficient,
                    result.negative_viscous_coefficient,
                    result.zero_velocity_blend_band.get(),
                );
                let actuator_params = result.into_friction_compensated_actuator_params(
                    ActuatorLimits {
                        max_output_velocity: Some(RadPerSec::new(30.0)),
                        max_output_torque: Some(NewtonMeters::new(10.0)),
                    },
                    NewtonMeters::new(0.3),
                );

                println!("phase 3: runtime control");
                let runtime_system = MotorRuntime::from_parts(
                    fluxkit::MotorRuntimeParts {
                        actuator: actuator_params,
                        ..parts
                    },
                    FAST_DT_SECONDS,
                )
                .expect("valid runtime config");
                let runtime_system = MOTOR_RUNTIME.init(runtime_system);
                let (handle, ticker) = runtime_system.split().unwrap();
                handle.set_command(MotorCommand::Velocity(RadPerSec::new(2.0)));
                handle.arm();
                runtime_handle = Some(handle);
                runtime_owner = Some(runtime_system);
                actuator_calibration_handle = None;
                app_state.with_mut(|slot| {
                    *slot = AppState::RuntimeRunning(
                        ticker,
                        0,
                        Vec::with_capacity(RUNTIME_FAST_CYCLES as usize),
                    )
                });
            }
        } else {
            let runtime_status = runtime_handle.as_ref().map(|handle| handle.status());
            if let Some(status) = runtime_status {
                if status.fault_latched {
                    panic!("main: runtime fault latched");
                }
                if !runtime_target_reported
                    && status.controller.last_output_mechanical_velocity.get() > 1.8
                {
                    println!(
                        "main: runtime reached target: output_vel={:.3}, iq={:.3}",
                        status.controller.last_output_mechanical_velocity.get(),
                        status.controller.last_measured_idq.q.get(),
                    );
                    runtime_target_reported = true;
                }
            }

            let ready =
                app_state.with_mut(|slot| match core::mem::replace(slot, AppState::Empty) {
                    AppState::RuntimeRunning(ticker, cycles, samples)
                        if cycles >= RUNTIME_FAST_CYCLES =>
                    {
                        let owner = runtime_owner.expect("runtime owner should be initialized");
                        match owner.try_into_parts() {
                            Some(_parts) => Some(samples),
                            None => {
                                *slot = AppState::RuntimeRunning(ticker, cycles, samples);
                                None
                            }
                        }
                    }
                    state => {
                        *slot = state;
                        None
                    }
                });
            if let Some(trace) = ready {
                app_state.with_mut(|slot| *slot = AppState::Finished);
                draw_velocity_plot(plot_path, &trace).unwrap();
                println!("wrote output-velocity plot to {plot_path}");

                let final_status = runtime_handle
                    .as_ref()
                    .expect("runtime handle should be available")
                    .status()
                    .controller;
                println!(
                    "done: pole_pairs={}, iq={:.3}, rotor_vel={:.3}",
                    motor_params.unwrap().pole_pairs,
                    final_status.last_measured_idq.q.get(),
                    final_status.last_rotor_mechanical_velocity.get(),
                );
                break;
            }
        }

        let faulted = app_state.with(|slot| matches!(slot, AppState::Faulted));
        if faulted {
            panic!("main: irq context faulted");
        }
        thread::yield_now();
    }
}

fn main() {
    let plot_path = env::args()
        .nth(1)
        .unwrap_or_else(|| "target/plots/threaded_bringup_output_velocity.svg".to_owned());
    if let Some(parent) = std::path::Path::new(&plot_path).parent() {
        fs::create_dir_all(parent).unwrap();
    }

    println!("starting threaded bring-up example");

    thread::scope(|scope| {
        let irq = scope.spawn(|| {
            loop {
                let refs = GLOBAL_REFS.with(|slot| *slot);
                let GlobalRefs {
                    shared: Some(shared),
                    app_state: Some(app_state),
                } = refs
                else {
                    thread::yield_now();
                    continue;
                };
                irq_loop(app_state, shared);
                break;
            }
        });
        let main_context = scope.spawn(|| main_context_loop(&plot_path));

        main_context.join().unwrap();
        irq.join().unwrap();
    });
}
