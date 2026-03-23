# Fluxkit

`fluxkit` is a `no_std` Rust workspace for BLDC / PMSM control.

Current crates:

- `fluxkit_math`: units, transforms, modulation, PI primitives
- `fluxkit_core`: pure deterministic motor-control engine
- `fluxkit_hal`: narrow motor-control HAL contracts
- `fluxkit`: umbrella crate and generic `MotorSystem` wrapper
- `fluxkit_pmsm_sim`: ideal PMSM plant emulator for integration tests

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
XDG_CACHE_HOME=/tmp/fluxkit-nix-cache nix develop -c cargo test -p fluxkit-core -p fluxkit-hal -p fluxkit -p fluxkit-pmsm-sim
```

Test only the top-level wrapper:

```bash
XDG_CACHE_HOME=/tmp/fluxkit-nix-cache nix develop -c cargo test -p fluxkit
```

Build docs:

```bash
XDG_CACHE_HOME=/tmp/fluxkit-nix-cache nix develop -c cargo doc -p fluxkit-core --no-deps
```

Local rustdoc builds in this repo load a KaTeX header from `.cargo/config.toml`,
so LaTeX-style math in crate docs renders when you use `cargo doc`.

Run the simulator examples:

```bash
XDG_CACHE_HOME=/tmp/fluxkit-nix-cache nix develop -c cargo run -p fluxkit-pmsm-sim --example closed_loop_current
XDG_CACHE_HOME=/tmp/fluxkit-nix-cache nix develop -c cargo run -p fluxkit-pmsm-sim --example closed_loop_position
XDG_CACHE_HOME=/tmp/fluxkit-nix-cache nix develop -c cargo run -p fluxkit-pmsm-sim --example closed_loop_torque_command
XDG_CACHE_HOME=/tmp/fluxkit-nix-cache nix develop -c cargo run -p fluxkit-pmsm-sim --example closed_loop_velocity_command
```

These examples now generate SVG plots in `target/plots/`.

Embedded reference plots:

![Closed-loop current response](docs/plots/closed_loop_current.svg)

![Closed-loop position response](docs/plots/closed_loop_position.svg)

Actuator friction-compensation reference plot:

![Torque command response](docs/plots/closed_loop_torque_command.svg)

Velocity command reference plot:

![Velocity command response](docs/plots/closed_loop_velocity_command.svg)

Generate the modulation plots directly from `fluxkit_math`:

```bash
XDG_CACHE_HOME=/tmp/fluxkit-nix-cache nix develop -c cargo run -p fluxkit_math --example plot_modulation
```

Embedded modulation plot:

![SPWM vs SVPWM modulation comparison](docs/plots/modulation_comparison.svg)

To refresh the checked-in documentation images:

```bash
./scripts/generate-doc-plots.sh
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

fluxkit-pmsm-sim
   └─ depends on fluxkit-math
```

Important boundaries:

- `fluxkit_core` must not depend on `fluxkit_hal`
- `fluxkit_hal` is contracts only
- generic controller + HAL orchestration lives in `fluxkit`
- `fluxkit_pmsm_sim` is a separate plant-model crate for host-side and `no_std` integration tests
- runtime / executor / MCU integration should live in a future integration crate

## Current scope

Implemented today:

- absolute-encoder rotor path
- actuator layer with gear ratio and output-axis constraints
- optional bounded actuator compensation layer above the current loop
  - friction and drag
  - hybrid friction policy:
    - command-guided breakaway near zero speed
    - measured-velocity viscous drag once moving
- output-axis encoder path for supervisory control
- internal multi-turn unwrapping for both rotor and output axes
- `Disabled`, `Current`, `Torque`, `Velocity`, `Position`, and `OpenLoopVoltage` modes
- synchronous current-loop `fast_tick()`
- medium-rate supervisory loop in `medium_tick()`
- position and velocity loops both run in the same `medium_tick()` when `Position` mode is active
- single-owner scheduled entrypoint through `tick(..., TickSchedule)`
- model-based current-loop feedforward
- actuator-side compensation telemetry in controller status
- configurable modulation through `MotorController<M>`
- generic `MotorSystem` wrapper in `fluxkit`
- ideal PMSM plant simulation with `d/q`, `alpha/beta`, phase-voltage, and duty-driven stepping
- simulator-side actuator parasitics and output-axis snapshots for integration tests

The torque-command example compares uncompensated and compensated response for
an output torque step applied through output-side friction and inertia. The
velocity-command example separately shows velocity-mode tracking under the same
attached inertia and friction model. Both reference graphs are shown above with
the other generated example outputs.

Not implemented:

- startup state machines
- calibration state machines
- sensorless support
- Embassy integration

## Calibration

The repo now includes pure calibration procedures in `fluxkit_core` plus
HAL-facing wrappers in `fluxkit`.

Recommended order:

1. Motor electrical calibration
2. Actuator friction calibration

Motor calibration procedures:

- pole pairs + electrical angle offset
- phase resistance
- phase inductance
- flux linkage

Actuator calibration procedures:

- gear ratio from simultaneous rotor/output travel
- Coulomb + viscous friction from steady velocity sweeps
- breakaway torque from slow torque ramps
- `zero_velocity_blend_band` from low-speed release ramps

Persisted records:

- `MotorCalibration`
- `ActuatorCalibration`

Typical flow:

1. Run the pure procedure through `MotorCalibrationSystem` or `ActuatorCalibrationSystem`.
   Or run the full chained workflow through `MotorCalibrationWorkflow` / `ActuatorCalibrationWorkflow`.
2. Merge each completed result into the corresponding persisted record.
3. Apply the merged record onto `MotorParams` or `ActuatorParams`.
4. Persist those populated parameter structs in board/runtime code.

Current simulator-backed confidence:

- pole pairs are recovered exactly
- electrical offset is validated to within `0.03 rad`
- phase inductance and flux linkage are within about `1%` in the current simulator setup
- Coulomb and viscous friction are close fits
- breakaway and blend-band calibration are usable but lower-confidence than the motor electrical terms

These procedures are validated against the in-repo simulator. They should be
treated as a strong bring-up baseline, not as final proof of real-hardware
accuracy.

## PMSM simulator

`fluxkit_pmsm_sim` provides an allocation-free ideal PMSM plant model intended
for integration tests.

It models:

- `d/q` electrical dynamics
- electromagnetic torque generation
- rigid-shaft mechanical dynamics
- viscous friction
- static friction
- output-side actuator reduction, actuator inertia, attached load inertia, and parasitics
- optional voltage-vector magnitude limiting

Model equations:

$$v_d = R i_d + L_d \frac{d i_d}{dt} - \omega_e L_q i_q$$
$$v_q = R i_q + L_q \frac{d i_q}{dt} + \omega_e (L_d i_d + \psi_m)$$
$$\tau_e = \frac{3}{2} p \left(\psi_m i_q + (L_d - L_q) i_d i_q \right)$$
$$J \frac{d \omega_m}{dt} = \tau_e - \tau_{load} - B \omega_m - \tau_{static}$$

It can be stepped with:

- rotating-frame `d/q` voltage
- stationary-frame `alpha/beta` voltage
- phase-voltage vectors
- PWM duty plus DC bus voltage

The `a/b/c` and duty-driven entrypoints are averaged plant-input models. They
are suitable for controller integration tests, but they do not simulate
transistor switching edges, deadtime, PWM ripple within a carrier cycle, or
bridge parasitics.

This is meant for controller validation and regression testing, not inverter
switching simulation or high-fidelity electromagnetic analysis.

## Loop model

`MotorController` uses an explicit multi-rate API:

- `fast_tick()`
  - current control
  - current feedforward
  - voltage limiting
  - modulation
- actuator/output-axis commands
  - `Torque`, `Velocity`, and `Position` targets are expressed at the actuator output
  - rotor measurements remain motor-side for FOC transforms and feedforward
  - output-axis measurements drive the supervisory loops
- `tick(input, TickSchedule)`
  - preferred runtime entrypoint when fast/medium/slow timers exist
  - runs the fast loop first
  - then runs due medium/slow work in the same owner context
  - avoids sharing the controller across concurrent interrupts
- `medium_tick()`
  - torque-to-current mapping
  - velocity PI
  - position PI followed by velocity PI in `Position` mode
- `slow_tick()`
  - currently reserved only
