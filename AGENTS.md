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

## Dependency rules

- `fluxkit_core` depends on `fluxkit_math`
- `fluxkit_core` must not depend on `fluxkit_hal`
- `fluxkit_hal` must not depend on `fluxkit_core`
- generic glue between core and hal belongs in `fluxkit`
- runtime-, executor-, MCU-, DMA-, and Embassy-specific logic belongs in a future integration crate, not in `core`, `hal`, or the generic `fluxkit` wrapper

## Current implemented scope

The repo is intentionally trimmed to the current MVP. Do not reintroduce removed placeholder surface unless there is real implementation behind it.

### `fluxkit_core`

Implemented today:

- `ControlMode`
  - `Disabled`
  - `Current`
- `MotorState`
  - `Disabled`
  - `Ready`
  - `Running`
  - `Faulted`
- `MotorController`
  - synchronous current-loop FOC fast path
  - configurable modulator through `MotorController<M>`
  - `MotorController::new(...)` defaults to `Svpwm`
  - `MotorController::new_with_modulator(...)` accepts any `fluxkit_math::Modulator`
- input validation
- explicit `Error`
- bounded duty output
- status snapshot

Not present anymore:

- startup scaffolding
- calibration scaffolding
- torque / velocity / position placeholder loops
- medium / slow tick placeholders
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
- `MotorHardware<PWM, CURRENT, BUS, ROTOR>`
- `MotorSystem<PWM, CURRENT, BUS, ROTOR, MOD = Svpwm>`
- `MotorSystemError<...>`

`MotorSystem` is the generic synchronous wrapper that:

1. samples HAL inputs
2. builds `FastLoopInput`
3. runs `MotorController::fast_tick(...)`
4. writes phase duty back to PWM

This is the preferred place for generic glue code between the controller and HAL traits.

## Hardware assumptions currently in scope

- rotor sensing is absolute-encoder only for now
- `RotorEstimate` and `RotorReading` contain:
  - `electrical_angle`
  - `mechanical_velocity`
- there is no rotor-source enum at the moment

Do not reintroduce Hall or sensorless-source abstractions unless there is a real use case and corresponding implementation plan.

## Control policy

- current mode is the only real control mode
- `id_ref` and `iq_ref` are set explicitly on the controller
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

`MotorSystem::fast_tick(...)` currently treats:

- `CurrentSampleValidity::Invalid` as an integration error and forces neutral PWM
- `Valid`, `Estimated`, and `Saturated` as acceptable inputs to pass through to the controller

If this policy changes, change it explicitly in `crates/fluxkit/src/system.rs`.

## Publishing / dependency guidance

When making crates publishable, prefer `version + path` for internal workspace dependencies so:

- local development uses path dependencies
- published crates resolve through crates.io

Do not leave publishable inter-crate dependencies as `path`-only.

## Verification

The repo is commonly verified through the Nix dev shell:

```bash
XDG_CACHE_HOME=/tmp/fluxkit-nix-cache nix develop -c cargo fmt
XDG_CACHE_HOME=/tmp/fluxkit-nix-cache nix develop -c cargo test -p fluxkit-core -p fluxkit-hal -p fluxkit
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
