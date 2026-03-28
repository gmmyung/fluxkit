# Fluxkit Agent Notes

This file is the short current-truth guide for future coding agents.

## Project shape

Fluxkit is a `no_std` BLDC / PMSM workspace with:

- deterministic control and calibration logic
- narrow synchronous HAL traits
- a project-facing runtime/calibration crate
- simulator-backed examples and integration tests

The project is intentionally focused on motor-control usage, not on being a
general MCU framework or executor.

## Workspace layout

- `crates/fluxkit_math`
  - units, transforms, modulation, estimator primitives
- `crates/fluxkit_core`
  - deterministic control engine and pure calibration procedures
- `crates/fluxkit_hal`
  - narrow synchronous motor-control HAL traits
- `crates/fluxkit`
  - project-facing runtime and calibration wrappers
- `crates/fluxkit_pmsm_sim`
  - ideal plant model used by examples and integration tests

## Dependency boundaries

- `fluxkit_core` depends on `fluxkit_math`
- `fluxkit_core` must not depend on `fluxkit_hal`
- `fluxkit_hal` must not depend on `fluxkit_core`
- controller + HAL glue belongs in `fluxkit`
- `fluxkit_pmsm_sim` stays independent from `fluxkit_core` and `fluxkit_hal`
- board startup, DMA plumbing, interrupts, and framework integration do not belong in this workspace

## Current runtime model

`fluxkit::MotorRuntime` is the main user-facing runtime type.

- main-context code owns `MotorRuntime`
- non-owner code uses `MotorHandle`
- IRQ-side code uses `MotorTicker`
- `MotorTicker::tick()` performs one full wrapper-owned control step:
  - sample hardware
  - sample winding temperature
  - update estimators
  - run the control engine
  - apply PWM
  - publish status
- phase transitions happen through `MotorRuntime::try_into_parts()`
- after `try_into_parts()`, the old handle/ticker become inactive and should be discarded

Do not reintroduce the removed wrapper-level split runtime API.

## Current calibration model

Public calibration types:

- `MotorCalibrationRuntime`
- `ActuatorCalibrationRuntime`

Both are:

- fixed-period
- request-driven
- observed through `handle().status()`
- executed through `ticker().tick()`
- completed when `handle().status().result` becomes `Some(...)`
- transitioned out through `try_into_parts()`

Typical bring-up order:

1. `MotorCalibrationRuntime`
2. `ActuatorCalibrationRuntime`
3. `MotorRuntime`

## Current control-engine model

`fluxkit_core::MotorController` is the lower-level engine, not the preferred
project-facing API.

Public interaction is intentionally narrow:

- `new(...)`
- `apply_command(...)`
- `set_armed(...)`
- `apply_actuator_calibration(...)`
- `friction_compensation_enabled()`
- `clear_error()`
- `step(...)`
- `status()`

The controller still supports:

- `Disabled`
- `Current`
- `Torque`
- `Mit`
- `Velocity`
- `Position`
- `OpenLoopVoltage`

Do not grow the controller surface back into a broad user API unless there is a
concrete need.

## Current assumptions

- absolute rotor sensing
- explicit output/actuator sensing
- winding temperature sensing is required in runtime and motor calibration
- no Hall abstraction
- no sensorless abstraction
- output sensor direction is assumed aligned with the reduced actuator axis
- gear-ratio calibration faults on opposite output-sensor direction

Do not reintroduce removed placeholder abstractions without a concrete
implementation plan.

## Control policy

- `Current` mode uses explicit `d/q` current targets
- `Torque`, `Mit`, `Velocity`, and `Position` operate in output-space quantities
- actuator compensation is additive feedforward above the current loop
- `OpenLoopVoltage` bypasses the current PI
- neutral duty means centered PWM, not hardware-off
- invalid current samples force neutral PWM at the wrapper level

## API guidance

Prefer:

- explicit typed APIs
- strong ownership
- runtime/handle separation
- request/result-based calibration flow
- small HAL traits

Avoid:

- executor-coupled APIs in `core` or `hal`
- hidden hardware ownership in `fluxkit_core`
- broad convenience surfaces that duplicate `MotorRuntime`
- stringly typed error handling

## Error policy

- use typed error enums
- `fluxkit_core::Error` is the controller error type
- HAL traits use associated `type Error: core::error::Error`
- wrapper errors should also implement `core::error::Error`

## Features

Supported workspace-facing features:

- `defmt`
- `serde`

Do not add placeholder Cargo features that do not change behavior.

## Verification

Preferred verification path:

```bash
XDG_CACHE_HOME=/tmp/fluxkit-nix-cache nix develop -c cargo fmt --all
XDG_CACHE_HOME=/tmp/fluxkit-nix-cache nix develop -c cargo test -p fluxkit-core -p fluxkit-hal -p fluxkit -p fluxkit-pmsm-sim
```

If only `fluxkit` or `fluxkit_core` changed, at minimum run:

```bash
XDG_CACHE_HOME=/tmp/fluxkit-nix-cache nix develop -c cargo test -p fluxkit-core -p fluxkit
```

Coverage helper:

```bash
./scripts/coverage.sh --summary-only
```

## Editing guidance

- preserve `no_std`
- preserve allocation-free control-path behavior
- keep hardware ownership out of `fluxkit_core`
- keep HAL traits narrow and synchronous
- keep `fluxkit` as the project-facing runtime/calibration layer

If unsure where code belongs:

- pure math/control/calibration procedure -> `fluxkit_core`
- hardware contract -> `fluxkit_hal`
- controller + HAL orchestration -> `fluxkit`
- board/framework glue -> outside this workspace
