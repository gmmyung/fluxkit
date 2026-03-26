use core::cell::RefCell;
use std::{convert::Infallible, env, fs, thread};

use critical_section::Mutex;
use fluxkit::{
    ActuatorCalibrationLimits, ActuatorCalibrationRequest, ActuatorCalibrationSystem,
    ActuatorLimits, ContinuousMechanicalAngle, ControlMode, CurrentLoopConfig, InverterParams,
    MotorCalibrationLimits, MotorCalibrationRequest, MotorCalibrationResult,
    MotorCalibrationSystem, MotorCommand, MotorHardware, MotorLimits, MotorSystem,
    PassThroughEstimator, centered_phase_duty,
    hal::{
        BusVoltageSensor, CurrentSampleValidity, CurrentSampler, OutputReading, OutputSensor,
        PhaseCurrentSample, PhasePwm, RotorReading, RotorSensor,
    },
    math::{
        Dq, inverse_clarke, inverse_park,
        units::{Amps, Duty, Henries, Hertz, NewtonMeters, Ohms, RadPerSec, Volts, Webers},
    },
};
use fluxkit_pmsm_sim::{ActuatorPlantParams, PmsmModel, PmsmParams, PmsmState};
use plotters::prelude::*;
use static_cell::StaticCell;

const FAST_DT_SECONDS: f32 = 1.0 / 20_000.0;
const GEAR_RATIO: f32 = 2.0;
const RUNTIME_FAST_CYCLES: u32 = 100_000;

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

struct ResultMailbox<T> {
    inner: Mutex<RefCell<Option<T>>>,
}

impl<T> ResultMailbox<T> {
    fn new() -> Self {
        Self {
            inner: Mutex::new(RefCell::new(None)),
        }
    }

    fn publish(&self, value: T) -> Result<(), T> {
        critical_section::with(|cs| {
            let mut slot = self.inner.borrow(cs).borrow_mut();
            if slot.is_some() {
                Err(value)
            } else {
                *slot = Some(value);
                Ok(())
            }
        })
    }

    fn take(&self) -> Option<T> {
        critical_section::with(|cs| self.inner.borrow(cs).borrow_mut().take())
    }
}

#[derive(Debug)]
struct SimHarness {
    plant: PmsmModel,
    bus_voltage: Volts,
    load_torque: NewtonMeters,
    last_duty: fluxkit::math::frame::Abc<Duty>,
    rotor_bias: f32,
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

fn phase_currents(shared: &SharedCell<SimHarness>) -> fluxkit::math::frame::Abc<Amps> {
    shared.with(|harness| {
        let state = *harness.plant.state();
        let electrical_angle = fluxkit::math::angle::mechanical_to_electrical(
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
        let duty = fluxkit::math::frame::Abc::new(a, b, c);
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

fn calibration_hardware<'a>(
    shared: &'a SharedCell<SimHarness>,
) -> (SimPwm<'a>, SimCurrent<'a>, SimBus<'a>, SimRotor<'a>) {
    (
        SimPwm { shared },
        SimCurrent { shared },
        SimBus { shared },
        SimRotor { shared },
    )
}

fn runtime_hardware<'a>(
    shared: &'a SharedCell<SimHarness>,
) -> MotorHardware<SimPwm<'a>, SimCurrent<'a>, SimBus<'a>, SimRotor<'a>, SimOutput<'a>> {
    MotorHardware {
        pwm: SimPwm { shared },
        current: SimCurrent { shared },
        bus: SimBus { shared },
        rotor: SimRotor { shared },
        output: SimOutput { shared },
    }
}

fn output_velocity(shared: &SharedCell<SimHarness>) -> f32 {
    shared.with(|harness| harness.plant.state().mechanical_velocity.get() / GEAR_RATIO)
}

type ExampleMotorCalibrationSystem<'a> = MotorCalibrationSystem<
    SimPwm<'a>,
    SimCurrent<'a>,
    SimBus<'a>,
    SimRotor<'a>,
    fluxkit::math::Svpwm,
>;
type ExampleActuatorCalibrationSystem<'a> = ActuatorCalibrationSystem<
    SimPwm<'a>,
    SimCurrent<'a>,
    SimBus<'a>,
    SimRotor<'a>,
    SimOutput<'a>,
    fluxkit::math::Svpwm,
    PassThroughEstimator,
    PassThroughEstimator,
>;
type ExampleMotorSystem<'a> = MotorSystem<
    SimPwm<'a>,
    SimCurrent<'a>,
    SimBus<'a>,
    SimRotor<'a>,
    SimOutput<'a>,
    fluxkit::math::Svpwm,
    PassThroughEstimator,
    PassThroughEstimator,
>;
enum AppState<'a> {
    Empty,
    MotorCalibrationRunning(ExampleMotorCalibrationSystem<'a>, u32),
    MotorCalibrationReady(ExampleMotorCalibrationSystem<'a>),
    ActuatorCalibrationRunning(ExampleActuatorCalibrationSystem<'a>, u32),
    ActuatorCalibrationReady(ExampleActuatorCalibrationSystem<'a>),
    RuntimeRunning(ExampleMotorSystem<'a>, u32, Vec<(f32, f32)>),
    RuntimeReady(ExampleMotorSystem<'a>, Vec<(f32, f32)>),
    Finished,
    Faulted,
}

#[derive(Clone, Copy, Default)]
struct GlobalRefs {
    shared: Option<&'static SharedCell<SimHarness>>,
    app_state: Option<&'static SharedCell<AppState<'static>>>,
    motor_mailbox: Option<&'static ResultMailbox<MotorCalibrationResult>>,
    actuator_mailbox: Option<&'static ResultMailbox<fluxkit::ActuatorCalibrationResult>>,
    runtime_status: Option<&'static SharedCell<Option<fluxkit::MotorRuntimeStatus>>>,
}

static SHARED: StaticCell<SharedCell<SimHarness>> = StaticCell::new();
static APP_STATE: StaticCell<SharedCell<AppState<'static>>> = StaticCell::new();
static MOTOR_MAILBOX: StaticCell<ResultMailbox<MotorCalibrationResult>> = StaticCell::new();
static ACTUATOR_MAILBOX: StaticCell<ResultMailbox<fluxkit::ActuatorCalibrationResult>> =
    StaticCell::new();
static RUNTIME_STATUS: StaticCell<SharedCell<Option<fluxkit::MotorRuntimeStatus>>> =
    StaticCell::new();
static GLOBAL_REFS: SharedCell<GlobalRefs> = SharedCell::new(GlobalRefs {
    shared: None,
    app_state: None,
    motor_mailbox: None,
    actuator_mailbox: None,
    runtime_status: None,
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

fn irq_loop<'a>(
    app_state: &SharedCell<AppState<'a>>,
    motor_mailbox: &ResultMailbox<MotorCalibrationResult>,
    actuator_mailbox: &ResultMailbox<fluxkit::ActuatorCalibrationResult>,
    runtime_status: &SharedCell<Option<fluxkit::MotorRuntimeStatus>>,
    shared: &SharedCell<SimHarness>,
) {
    loop {
        let should_exit = app_state.with_mut(|slot| {
            let state = core::mem::replace(slot, AppState::Empty);
            match state {
                AppState::Empty => {
                    *slot = AppState::Empty;
                    false
                }
                AppState::MotorCalibrationRunning(mut system, mut cycles) => {
                    cycles = cycles.wrapping_add(1);
                    match system.tick() {
                        Ok(Some(result)) => {
                            println!("irq: motor calibration complete after {cycles} cycles");
                            let _ = motor_mailbox.publish(result);
                            *slot = AppState::MotorCalibrationReady(system);
                        }
                        Ok(None) => {
                            if cycles % 20_000 == 0 {
                                println!(
                                    "irq: motor calibration phase {:?} still running ({cycles} cycles)",
                                    system.phase().unwrap()
                                );
                            }
                            *slot = AppState::MotorCalibrationRunning(system, cycles);
                        }
                        Err(error) => {
                            eprintln!("irq: motor calibration failed: {error}");
                            *slot = AppState::Faulted;
                        }
                    }
                    false
                }
                AppState::MotorCalibrationReady(system) => {
                    *slot = AppState::MotorCalibrationReady(system);
                    false
                }
                AppState::ActuatorCalibrationRunning(mut system, mut cycles) => {
                    cycles = cycles.wrapping_add(1);
                    match system.tick() {
                        Ok(Some(result)) => {
                            println!("irq: actuator calibration complete after {cycles} cycles");
                            let _ = actuator_mailbox.publish(result);
                            *slot = AppState::ActuatorCalibrationReady(system);
                        }
                        Ok(None) => {
                            if cycles % 20_000 == 0 {
                                println!(
                                    "irq: actuator calibration phase {:?} still running ({cycles} cycles)",
                                    system.phase().unwrap()
                                );
                            }
                            *slot = AppState::ActuatorCalibrationRunning(system, cycles);
                        }
                        Err(error) => {
                            eprintln!("irq: actuator calibration failed: {error}");
                            *slot = AppState::Faulted;
                        }
                    }
                    false
                }
                AppState::ActuatorCalibrationReady(system) => {
                    *slot = AppState::ActuatorCalibrationReady(system);
                    false
                }
                AppState::RuntimeRunning(mut system, mut cycles, mut samples) => {
                    cycles = cycles.wrapping_add(1);
                    match system.tick() {
                        Ok(_) => {
                            runtime_status.with_mut(|slot| *slot = Some(system.handle().status()));
                            samples.push((cycles as f32 * FAST_DT_SECONDS, output_velocity(shared)));
                            if cycles % 20_000 == 0 {
                                println!("irq: runtime executed {cycles} cycles");
                            }
                            if cycles >= RUNTIME_FAST_CYCLES {
                                println!("irq: runtime complete after {cycles} cycles");
                                *slot = AppState::RuntimeReady(system, samples);
                            } else {
                                *slot = AppState::RuntimeRunning(system, cycles, samples);
                            }
                        }
                        Err(error) => {
                            eprintln!("irq: runtime failed: {error}");
                            *slot = AppState::Faulted;
                        }
                    }
                    false
                }
                AppState::RuntimeReady(system, samples) => {
                    *slot = AppState::RuntimeReady(system, samples);
                    false
                }
                AppState::Finished => {
                    *slot = AppState::Finished;
                    true
                }
                AppState::Faulted => {
                    *slot = AppState::Faulted;
                    true
                }
            }
        });

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
    let motor_mailbox: &'static ResultMailbox<MotorCalibrationResult> =
        MOTOR_MAILBOX.init(ResultMailbox::<MotorCalibrationResult>::new());
    let actuator_mailbox: &'static ResultMailbox<fluxkit::ActuatorCalibrationResult> =
        ACTUATOR_MAILBOX.init(ResultMailbox::<fluxkit::ActuatorCalibrationResult>::new());
    let runtime_status: &'static SharedCell<Option<fluxkit::MotorRuntimeStatus>> =
        RUNTIME_STATUS.init(SharedCell::new(None));

    let refs = GlobalRefs {
        shared: Some(shared),
        app_state: Some(app_state),
        motor_mailbox: Some(motor_mailbox),
        actuator_mailbox: Some(actuator_mailbox),
        runtime_status: Some(runtime_status),
    };
    GLOBAL_REFS.with_mut(|slot| *slot = refs);
    refs
}

fn main_context_loop(plot_path: &str) {
    let GlobalRefs {
        shared: Some(shared),
        app_state: Some(app_state),
        motor_mailbox: Some(motor_mailbox),
        actuator_mailbox: Some(actuator_mailbox),
        runtime_status: Some(runtime_status),
    } = init_globals()
    else {
        unreachable!();
    };

    let mut motor_params = None;
    let mut runtime_started = false;
    let mut runtime_target_reported = false;

    println!("phase 1: motor calibration");
    let (pwm, current, bus, rotor) = calibration_hardware(shared);
    let motor_calibration = MotorCalibrationSystem::new(
        pwm,
        current,
        bus,
        rotor,
        fluxkit::math::Svpwm,
        MotorCalibrationRequest::default(),
        MotorCalibrationLimits {
            max_align_voltage_mag: Volts::new(2.0),
            max_spin_voltage_mag: Volts::new(3.0),
            max_electrical_velocity: RadPerSec::new(60.0),
            timeout_seconds: 6.0,
        },
        FAST_DT_SECONDS,
    )
    .unwrap();
    app_state.with_mut(|slot| {
        *slot = AppState::MotorCalibrationRunning(motor_calibration, 0);
    });

    loop {
        if motor_params.is_none() {
            if let Some(result) = motor_mailbox.take() {
                println!(
                    "main: received motor calibration result: pole_pairs={}, R={:.4} ohm, L={:.8} H, psi={:.6} Wb, offset={:.4} rad",
                    result.pole_pairs,
                    result.phase_resistance_ohm.get(),
                    result.phase_inductance_h.get(),
                    result.flux_linkage_weber.get(),
                    result.electrical_angle_offset.get(),
                );
                motor_params = Some(result.into_motor_params(MotorLimits {
                    max_phase_current: Amps::new(10.0),
                    max_mech_speed: Some(RadPerSec::new(150.0)),
                }));

                app_state.with_mut(|slot| {
                    let state = core::mem::replace(slot, AppState::Empty);
                    let AppState::MotorCalibrationReady(_system) = state else {
                        panic!("main: unexpected state while transitioning from motor calibration");
                    };
                });

                println!("phase 2: actuator calibration");
                let actuator_calibration = ActuatorCalibrationSystem::new(
                    runtime_hardware(shared),
                    motor_params.unwrap(),
                    inverter_params(),
                    current_loop_config(),
                    fluxkit::math::Svpwm,
                    PassThroughEstimator::new(),
                    PassThroughEstimator::new(),
                    ActuatorCalibrationRequest::default(),
                    ActuatorCalibrationLimits {
                        max_velocity_target: RadPerSec::new(10.0),
                        max_torque_target: NewtonMeters::new(0.3),
                        timeout_seconds: 20.0,
                    },
                    FAST_DT_SECONDS,
                )
                .unwrap();
                app_state.with_mut(|slot| {
                    *slot = AppState::ActuatorCalibrationRunning(actuator_calibration, 0);
                });
            }
        } else if !runtime_started {
            if let Some(result) = actuator_mailbox.take() {
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

                app_state.with_mut(|slot| {
                    let state = core::mem::replace(slot, AppState::Empty);
                    let AppState::ActuatorCalibrationReady(_system) = state else {
                        panic!(
                            "main: unexpected state while transitioning from actuator calibration"
                        );
                    };
                });

                println!("phase 3: runtime control");
                let runtime_system = MotorSystem::new(
                    runtime_hardware(shared),
                    fluxkit::MotorController::new(
                        motor_params.unwrap(),
                        inverter_params(),
                        actuator_params,
                        current_loop_config(),
                    ),
                    PassThroughEstimator::new(),
                    PassThroughEstimator::new(),
                    FAST_DT_SECONDS,
                );
                let handle = runtime_system.handle();
                handle.set_command(MotorCommand {
                    mode: ControlMode::Velocity,
                    output_velocity_target: RadPerSec::new(2.0),
                    ..MotorCommand::default()
                });
                handle.arm();
                runtime_status.with_mut(|slot| *slot = Some(handle.status()));
                runtime_started = true;
                app_state.with_mut(|slot| {
                    *slot = AppState::RuntimeRunning(
                        runtime_system,
                        0,
                        Vec::with_capacity(RUNTIME_FAST_CYCLES as usize),
                    );
                });
            }
        } else {
            if let Some(status) = runtime_status.with(|slot| *slot) {
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
                    AppState::RuntimeReady(system, samples) => {
                        *slot = AppState::Finished;
                        Some((system, samples))
                    }
                    state => {
                        *slot = state;
                        None
                    }
                });
            if let Some((system, trace)) = ready {
                draw_velocity_plot(plot_path, &trace).unwrap();
                println!("wrote output-velocity plot to {plot_path}");

                let final_status = system.handle().status().controller;
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
                    motor_mailbox: Some(motor_mailbox),
                    actuator_mailbox: Some(actuator_mailbox),
                    runtime_status: Some(runtime_status),
                } = refs
                else {
                    thread::yield_now();
                    continue;
                };
                irq_loop(
                    app_state,
                    motor_mailbox,
                    actuator_mailbox,
                    runtime_status,
                    shared,
                );
                break;
            }
        });
        let main_context = scope.spawn(|| main_context_loop(&plot_path));

        main_context.join().unwrap();
        irq.join().unwrap();
    });
}
