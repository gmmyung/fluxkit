# Fluxkit Agent Notes

This file summarizes the current architecture and the project-specific rules that future coding agents should follow.

## Workspace layout

- `crates/fluxkit_math`
  - math, units, transforms, modulation, PI primitives
- `crates/fluxkit_core`
  - pure deterministic control engine
- `crates/fluxkit_hal`
  - narrow motor-control HAL traits
- `crates/fluxkit`
  - umbrella crate and generic integration wrapper
- `crates/fluxkit_pmsm_sim`
  - ideal PMSM plant emulator for integration tests

## Dependency rules

- `fluxkit_core` depends on `fluxkit_math`
- `fluxkit_core` must not depend on `fluxkit_hal`
- `fluxkit_hal` must not depend on `fluxkit_core`
- generic glue between core and hal belongs in `fluxkit`
- `fluxkit_pmsm_sim` may depend on `fluxkit_math`, but should remain independent from `fluxkit_core` and `fluxkit_hal`
- runtime-, executor-, MCU-, DMA-, and Embassy-specific logic belongs in a future integration crate, not in `core`, `hal`, or the generic `fluxkit` wrapper

## Current implemented scope

The repo is intentionally trimmed to the current MVP. Do not reintroduce removed placeholder surface unless there is real implementation behind it.

### `fluxkit_core`

Implemented today:

- `ControlMode`
  - `Disabled`
  - `Current`
  - `Torque`
  - `Velocity`
  - `Position`
  - `OpenLoopVoltage`
- `MotorState`
  - `Disabled`
  - `Ready`
  - `Running`
  - `Faulted`
- `MotorController`
  - synchronous multi-rate FOC control path
  - configurable modulator through `MotorController<M>`
  - `MotorController::new(...)` defaults to `Svpwm`
  - `MotorController::new_with_modulator(...)` accepts any `fluxkit_math::Modulator`
  - explicit `ActuatorParams` for gear ratio, output-axis limits, and bounded compensation
  - `tick(input, TickSchedule)` is the preferred runtime entrypoint
  - `medium_tick()` owns torque, velocity, and position supervisory updates
  - `slow_tick()` is currently reserved as a no-op hook
- input validation
- explicit `Error`
- bounded duty output
- model-based current-loop feedforward
- multi-turn mechanical position unwrapping from absolute-encoder input
- status snapshot

Not present anymore:

- startup scaffolding
- calibration scaffolding
- command-dispatch placeholder API

### `fluxkit_hal`

Implemented today:

- `PhasePwm`
- `CurrentSampler`
- `BusVoltageSensor`
- `RotorSensor`
- `GateDriver`
- `FaultInput`
- `MonotonicMicros`
- `TemperatureSensor`

These are trait contracts only. Do not add MCU-specific implementations here.

### `fluxkit`

Implemented today:

- re-exports of `core`, `hal`, and `math`
- `MotorHardware<PWM, CURRENT, BUS, ROTOR, OUTPUT>`
- `MotorSystem<PWM, CURRENT, BUS, ROTOR, OUTPUT, MOD = Svpwm>`
- `MotorSystemError<...>`

`MotorSystem` is the generic synchronous wrapper that:

1. samples HAL inputs
2. builds `FastLoopInput`
3. runs `MotorController::tick(...)`
4. writes phase duty back to PWM

This is the preferred place for generic glue code between the controller and HAL traits.

### `fluxkit_pmsm_sim`

Implemented today:

- ideal PMSM electrical dynamics in `d/q`
- rigid-shaft mechanical dynamics
- viscous and static friction
- output-side actuator reduction and parasitic load model
- optional voltage-vector magnitude clamp
- stepping by `d/q`, `alpha/beta`, phase voltage, or PWM duty
- `PmsmSnapshot` outputs for controller integration tests

This crate is intended for test and simulation use. Keep it independent from
the controller and HAL crates.

## Hardware assumptions currently in scope

- rotor sensing is absolute-encoder only for now
- actuator/output sensing is explicit now
- `RotorEstimate` and `RotorReading` contain:
  - `electrical_angle`
  - `mechanical_angle`
  - `mechanical_velocity`
- `ActuatorEstimate` and `OutputReading` contain:
  - wrapped output angle
  - output velocity
- there is no rotor-source enum at the moment

Do not reintroduce Hall or sensorless-source abstractions unless there is a real use case and corresponding implementation plan.

## Control policy

- `Current` mode directly uses explicit `id_ref` and `iq_ref`
- `Torque`, `Velocity`, and `Position` commands are actuator-output quantities
- mechanical/load-side compensation is additive feedforward above the current loop
- actuator compensation is bounded and optional
- friction compensation uses a hybrid policy:
  - breakaway direction is command-guided near zero speed
  - Coulomb and viscous drag transition onto measured output velocity once moving
- `Torque`, `Velocity`, and `Position` are implemented through `medium_tick()`
- `Position` mode runs both the position loop and velocity loop in the same `medium_tick()`
- runtime integration should prefer `tick(..., TickSchedule)` over calling
  `fast_tick()`, `medium_tick()`, and `slow_tick()` from separate interrupts
- `OpenLoopVoltage` bypasses the current PI and modulates commanded `vdq` directly
- neutral duty means centered PWM output, not hardware-off
- fallback invalid-control output is neutral duty
- board/runtime code may still choose stronger safety actions such as disabling PWM or the gate driver

## Error policy

- use `core::error::Error` broadly
- `fluxkit_core::Error` is the controller error type
- HAL traits use associated `type Error: core::error::Error`
- integration-layer errors should also implement `core::error::Error`

Do not introduce ad hoc string-based error handling where a typed error enum is appropriate.

## Features

Supported workspace-facing features are:

- `defmt`
- `serde`

Removed:

- `approx-trig`
- `fixed-point`

Do not add placeholder Cargo features that do not change behavior.

## API direction

Prefer explicit typed APIs over loose tuples.

Good:

- `FastLoopInput`
- `FastLoopOutput`
- `MotorStatus`
- `PhaseCurrentSample`
- `MotorHardware`
- `MotorSystem`

Avoid:

- giant platform traits
- executor-coupled APIs in `core` or `hal`
- hidden hardware ownership inside `MotorController`

## Modulation

- modulation is configurable
- `MotorController::new(...)` uses `Svpwm`
- alternate modulators should go through the existing `Modulator` trait

Do not hardcode a new modulation strategy into the controller if it can be expressed through `Modulator`.

## Current sample policy

`MotorSystem::tick(...)` currently treats:

- `CurrentSampleValidity::Invalid` as an integration error and forces neutral PWM
- `Valid`, `Estimated`, and `Saturated` as acceptable inputs to pass through to the controller

If this policy changes, change it explicitly in `crates/fluxkit/src/system.rs`.

## Simulator guidance

- prefer using `fluxkit_pmsm_sim` in integration tests rather than adding fake plant math to `fluxkit_core` tests
- keep the simulator deterministic and allocation-free
- keep it at the “ideal plant model” level; do not turn it into an MCU runtime or switching-level inverter simulator inside this workspace

## Publishing / dependency guidance

When making crates publishable, prefer `version + path` for internal workspace dependencies so:

- local development uses path dependencies
- published crates resolve through crates.io

Do not leave publishable inter-crate dependencies as `path`-only.

## Verification

The repo is commonly verified through the Nix dev shell:

```bash
XDG_CACHE_HOME=/tmp/fluxkit-nix-cache nix develop -c cargo fmt
XDG_CACHE_HOME=/tmp/fluxkit-nix-cache nix develop -c cargo test -p fluxkit-core -p fluxkit-hal -p fluxkit -p fluxkit-pmsm-sim
```

If a change only touches the top-level wrapper, at minimum run:

```bash
XDG_CACHE_HOME=/tmp/fluxkit-nix-cache nix develop -c cargo test -p fluxkit
```

## Editing guidance

- preserve `no_std`
- preserve allocation-free control-path behavior
- keep `fast_tick()` synchronous
- keep hardware ownership out of `fluxkit_core`
- keep HAL traits narrow and synchronous
- keep `fluxkit` as generic glue only

If you are unsure where code belongs:

- pure math/control logic -> `fluxkit_core`
- hardware contract -> `fluxkit_hal`
- generic controller + HAL orchestration -> `fluxkit`
- executor / board / Embassy / ISR ownership -> future integration crate
