# fluxkit-pmsm-sim

Ideal PMSM plant emulator for host-side and `no_std` simulation.

The model integrates the standard `d/q` electrical equations and a rigid
shaft mechanical equation with reflected output inertia:

```text
v_d = R i_d + L_d d(i_d)/dt - ω_e L_q i_q
v_q = R i_q + L_q d(i_q)/dt + ω_e (L_d i_d + ψ_m)
τ_e = 3/2 p (ψ_m i_q + (L_d - L_q) i_d i_q)
J_eq d(ω_m)/dt = τ_e - τ_load - τ_mech - τ_actuator,ref
C_th dT/dt = P_cu - G_th (T - T_amb)
```

Here `J_eq` and `τ_mech` come from the actuator/drivetrain model, which owns
the combined equivalent output-side inertia and unified friction reflected
through the gear ratio. The electrical model also tracks a lumped winding
temperature state, using copper loss `P_cu` and a first-order thermal
conductance back to ambient.

Phase-domain excitation is supported too, but at the averaged plant-input
level:

- [`PmsmModel::step_phase_voltage`] accepts an `a/b/c` phase-voltage vector
- [`PmsmModel::step_phase_duty`] converts duty plus `Vbus` into an averaged
  zero-sum phase-voltage vector

These `a/b/c` paths are suitable for controller integration tests and
averaged inverter behavior. They do not model transistor switching edges,
deadtime, PWM ripple within a carrier cycle, or bridge parasitics.

The simulator is intentionally deterministic and allocation-free. It is
intended for controller integration tests rather than finite-element or
inverter-switching-accurate simulation.
