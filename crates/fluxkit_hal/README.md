# fluxkit-hal

Narrow motor-control hardware abstraction contracts for Fluxkit.

`fluxkit_hal` defines synchronous traits for platform integration without
owning peripherals, runtimes, or control logic. Platform crates use these
traits to acquire measurements and drive actuators around `fluxkit_core`.
