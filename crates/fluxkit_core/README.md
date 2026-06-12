# fluxkit-core

Deterministic `no_std` field-oriented control engine for Fluxkit.

`fluxkit_core` owns pure control logic only. It consumes validated loop
inputs, runs synchronous control math, and emits structured duty commands
and status snapshots without owning hardware resources or executor state.

In the full workspace layering:

- `fluxkit_math` provides units, transforms, modulation, and estimator primitives
- `fluxkit_core` provides the deterministic engine and pure calibration procedures
- `fluxkit_hal` provides synchronous hardware traits
- `fluxkit` provides the normal application-facing runtime and calibration wrappers

Most application code should start at `fluxkit`, not at `fluxkit_core`.

## Reference Plots

Representative closed-loop current response of the controller against the
ideal PMSM plant model used in integration tests.

- [Closed-loop current response](docs/plots/closed_loop_current.svg)

Flux weakening comparison showing high-speed velocity control with and without
negative `d`-axis current injection.

- [Flux weakening comparison](docs/plots/flux_weakening_comparison.svg)

The crate docs embed these SVGs directly so they remain visible in rustdoc.
