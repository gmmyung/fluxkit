# fluxkit

Project-facing crate for BLDC / PMSM bring-up and fixed-period runtime control.

`fluxkit` is the top-level crate you use in an application or firmware
project. It is meant for projects that already know their execution model:

- fixed-period control interrupt
- absolute rotor sensing
- winding temperature sensing
- explicit output / actuator sensing

The crate gives you:

- request-driven motor electrical calibration
- request-driven actuator calibration
- a main-context runtime owner, [`MotorRuntime`]
- a non-owning command/status handle, [`MotorHandle`]
- an IRQ-side executor, [`MotorTicker`]

`fluxkit` is intentionally not an MCU framework or executor. It focuses on
the motor-control problem itself and leaves board startup, interrupt
registration, persistence, and project-specific state machines to your
application code.

## Typical Flow

A normal project flow is:

1. implement the HAL traits needed by your board
2. run [`MotorCalibrationRuntime`]
3. build [`MotorParams`] from the resulting [`MotorCalibrationResult`]
4. run [`ActuatorCalibrationRuntime`]
5. build [`ActuatorParams`] from the resulting [`ActuatorCalibrationResult`]
6. construct [`MotorRuntime`]
7. call [`MotorRuntime::split`] to get the unique handle/ticker pair
8. use [`MotorHandle`] from non-IRQ code for commands and status

The full end-to-end example is:

- `cargo run -p fluxkit --example threaded_bringup`

That example shows:

- motor calibration
- actuator calibration
- transition into runtime control
- one main-owned runtime object
- one non-IRQ command/status path

## Main Types

Runtime:

- [`MotorRuntime`]
- [`MotorRuntimeParams`]
- [`MotorCommand`]
- [`MotorHandle`]
- [`MotorTicker`]
- [`MotorRuntimeStatus`]

Calibration:

- [`MotorCalibrationRuntime`]
- [`MotorCalibrationTicker`]
- [`MotorCalibrationRequest`]
- [`MotorCalibrationResult`]
- [`ActuatorCalibrationRuntime`]
- [`ActuatorCalibrationTicker`]
- [`ActuatorCalibrationRequest`]
- [`ActuatorCalibrationResult`]

Params and units:

- [`MotorParams`], [`ActuatorParams`], [`MotorLimits`], [`ActuatorLimits`]
- [`CurrentLoopConfig`], [`InverterParams`]
- units such as [`Amps`], [`Volts`], [`RadPerSec`], [`NewtonMeters`]

## Architecture

The workspace is intentionally split by responsibility:

- `fluxkit_math`
  - units, transforms, modulation, estimator primitives
- `fluxkit_core`
  - deterministic control engine and pure calibration procedures
- `fluxkit_hal`
  - narrow synchronous hardware contracts
- `fluxkit`
  - project-facing runtime and calibration wrappers

The intended ownership model is:

- one context owns [`MotorRuntime`]
- IRQ code executes it through [`MotorTicker`]
- non-owner code uses [`MotorHandle`]
- phase transitions happen through [`MotorRuntime::try_into_parts`]
- the same pattern applies to calibration through
  [`MotorCalibrationRuntime`] / [`MotorCalibrationTicker`] and
  [`ActuatorCalibrationRuntime`] / [`ActuatorCalibrationTicker`]

`fluxkit_core` is still available for lower-level engine integration, but
the normal application path should stay at the `fluxkit` layer.

## Minimal Runtime Shape

```ignore
use fluxkit::{
    MotorCommand, MotorRuntime, MotorRuntimeParams, PassThroughCurrentEstimator,
    PassThroughEstimator, Svpwm, units::RadPerSec,
};

# let pwm = todo!();
# let current = todo!();
# let bus = todo!();
# let rotor = todo!();
# let output = todo!();
# let temp = todo!();
# let motor_params = todo!();
# let inverter_params = todo!();
# let actuator_params = todo!();
# let current_loop_config = todo!();
let runtime = MotorRuntime::new(
    fluxkit::MotorHardware {
        pwm,
        current,
        bus,
        rotor,
        output,
        temp,
    },
    MotorRuntimeParams::new(
        motor_params,
        inverter_params,
        actuator_params,
        current_loop_config,
        1.0 / 20_000.0,
    ),
    fluxkit::RuntimeAlgorithms::default_pass_through(),
)?;

let (handle, ticker) = runtime.split()?;
handle.set_command(MotorCommand::Velocity(RadPerSec::new(2.0)));
handle.arm();

loop {
    ticker.tick()?;
}
# Ok::<(), Box<dyn std::error::Error>>(())
```

## Start Here

- For runtime owner / handle / ticker semantics, see [`system`].
- For calibration flow and result types, see [`calibration`].
