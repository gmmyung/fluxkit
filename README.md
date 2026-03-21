# Fluxkit

`fluxkit` is a `no_std` Rust workspace for BLDC / PMSM control.

Current crates:

- `fluxkit_math`: units, transforms, modulation, PI primitives
- `fluxkit_core`: pure deterministic current-loop controller
- `fluxkit_hal`: narrow motor-control HAL contracts
- `fluxkit`: umbrella crate and generic `MotorSystem` wrapper

## Nix usage

This repo includes a Nix flake with a development shell.

### Enter the dev shell

```bash
nix develop
```

The shell provides:

- `rustc`
- `cargo`
- `clippy`
- `rustfmt`
- `rust-analyzer`
- `bacon`
- `cargo-nextest`
- `git`
- `just`
- `pkg-config`

### Run one-off commands through the dev shell

```bash
nix develop -c cargo test
nix develop -c cargo fmt
nix develop -c cargo clippy --all-targets --all-features
```

### Recommended cache setting

In this repo, it is often useful to set a temporary `XDG_CACHE_HOME` when running Nix commands:

```bash
XDG_CACHE_HOME=/tmp/fluxkit-nix-cache nix develop -c cargo test
```

That keeps Nix evaluation cache writes out of your normal user cache and has been the most reliable path for local verification.

### Format the flake itself

```bash
nix fmt
```

## Common commands

Format:

```bash
XDG_CACHE_HOME=/tmp/fluxkit-nix-cache nix develop -c cargo fmt
```

Test the main crates:

```bash
XDG_CACHE_HOME=/tmp/fluxkit-nix-cache nix develop -c cargo test -p fluxkit-core -p fluxkit-hal -p fluxkit
```

Test only the top-level wrapper:

```bash
XDG_CACHE_HOME=/tmp/fluxkit-nix-cache nix develop -c cargo test -p fluxkit
```

Build docs:

```bash
XDG_CACHE_HOME=/tmp/fluxkit-nix-cache nix develop -c cargo doc -p fluxkit-core --no-deps
```

## Architecture

Dependency direction:

```text
fluxkit-math
   ↑
fluxkit-core

fluxkit-hal

fluxkit
   ├─ depends on fluxkit-core
   ├─ depends on fluxkit-hal
   └─ depends on fluxkit-math
```

Important boundaries:

- `fluxkit_core` must not depend on `fluxkit_hal`
- `fluxkit_hal` is contracts only
- generic controller + HAL orchestration lives in `fluxkit`
- runtime / executor / MCU integration should live in a future integration crate

## Current scope

Implemented today:

- absolute-encoder rotor path
- `Disabled` and `Current` control modes
- synchronous current-loop `fast_tick()`
- configurable modulation through `MotorController<M>`
- generic `MotorSystem` wrapper in `fluxkit`

Not implemented:

- startup state machines
- calibration state machines
- velocity / position loops
- sensorless support
- Embassy integration
