# Fluxkit Agent Notes

This file is the short current-truth guide for future coding agents.

## Project shape

Fluxkit is a `no_std` motor-control workspace for BLDC / PMSM projects with:

- absolute rotor sensing
- explicit output/actuator sensing
- fixed-period control execution
- request-driven calibration before runtime bring-up

The project is intentionally focused on real motor-control usage, not on being a
general embedded framework.

## Workspace layout

- `crates/fluxkit_math`
  - units, transforms, modulation, PI primitives, estimators
- `crates/fluxkit_core`
  - pure deterministic control and calibration procedures
- `crates/fluxkit_hal`
  - narrow HAL contracts only
- `crates/fluxkit`
  - user-facing runtime and calibration crate
- `crates/fluxkit_pmsm_sim`
  - ideal PMSM plant model for integration tests and examples

## Dependency boundaries

- `fluxkit_core` depends on `fluxkit_math`
- `fluxkit_core` must not depend on `fluxkit_hal`
- `fluxkit_hal` must not depend on `fluxkit_core`
- generic controller + HAL glue belongs in `fluxkit`
- `fluxkit_pmsm_sim` stays independent from `fluxkit_core` and `fluxkit_hal`
- board support, DMA plumbing, MCU startup, and framework integration do not belong in `core` or `hal`

## Current runtime model

`fluxkit::MotorSystem` is the project-facing runtime object.

- one owner calls `MotorSystem::tick()` at a fixed period
- non-IRQ code uses `MotorHandle`
- `tick()` does one full wrapper-owned control step:
  - sample hardware
  - update estimators
  - run controller work
  - apply PWM
  - publish status

Shared runtime metadata uses `critical-section` in `fluxkit`, but the owned
runtime object itself is not globally locked during the control step.

Do not reintroduce the older public split runtime API based on separate
wrapper-level fast/deferred entrypoints.

## Current calibration model

Calibration lives in `fluxkit`, while pure calibration procedures live in
`fluxkit_core`.

Public calibration systems:

- `MotorCalibrationSystem`
- `ActuatorCalibrationSystem`

Both are:

- fixed-period
- request-driven
- phase-aware through `phase()`
- finalized through:
  - `MotorCalibrationResult`
  - `ActuatorCalibrationResult`

Typical bring-up order:

1. motor calibration
2. actuator calibration
3. runtime `MotorSystem`

## What is implemented

### In `fluxkit_core`

- `MotorController`
- `Disabled`, `Current`, `Torque`, `Velocity`, `Position`, `OpenLoopVoltage`
- actuator-output control targets
- bounded actuator compensation
- status snapshot
- pure calibration procedures

### In `fluxkit`

- `MotorHardware`
- `MotorSystem`
- `MotorHandle`
- runtime command/status sharing
- request-driven motor calibration
- request-driven actuator calibration

### In `fluxkit_pmsm_sim`

- ideal `d/q` PMSM electrical model
- rigid-shaft mechanics
- actuator reduction, inertia, and parasitic friction/load

## Current assumptions

- rotor sensing is absolute-encoder only
- output sensing is explicit
- no sensorless or Hall abstraction
- no startup-state-machine framework in the public crate surface

Do not reintroduce removed placeholder abstractions without a concrete
implementation plan.

## Control policy

- `Current` mode uses explicit `id` / `iq` targets
- `Torque`, `Velocity`, and `Position` targets are actuator-output quantities
- mechanical/output compensation is additive feedforward above the current loop
- `OpenLoopVoltage` bypasses the current PI and modulates commanded `vdq`
- neutral duty is centered PWM, not hardware-off
- invalid current samples force neutral PWM at the wrapper level

## API guidance

Prefer:

- explicit typed APIs
- strong ownership
- request/result types
- small HAL traits

Avoid:

- giant platform traits
- executor-coupled APIs in `core` or `hal`
- hidden hardware ownership inside `MotorController`
- stringly typed error handling

## Error policy

- use typed error enums
- `fluxkit_core::Error` is the controller error type
- HAL traits use associated `type Error: core::error::Error`
- wrapper/integration errors should also implement `core::error::Error`

## Features

Supported workspace-facing features:

- `defmt`
- `serde`

Do not add placeholder Cargo features that do not change behavior.

## Verification

Preferred verification path:

```bash
XDG_CACHE_HOME=/tmp/fluxkit-nix-cache nix develop -c cargo fmt
XDG_CACHE_HOME=/tmp/fluxkit-nix-cache nix develop -c cargo test -p fluxkit-core -p fluxkit-hal -p fluxkit -p fluxkit-pmsm-sim
```

If only `fluxkit` changed, at minimum run:

```bash
XDG_CACHE_HOME=/tmp/fluxkit-nix-cache nix develop -c cargo test -p fluxkit
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
- keep `fluxkit` as the user-facing runtime and calibration layer

If unsure where code belongs:

- pure math/control/calibration procedure -> `fluxkit_core`
- hardware contract -> `fluxkit_hal`
- controller + HAL orchestration -> `fluxkit`
- board/framework glue -> outside this workspace
