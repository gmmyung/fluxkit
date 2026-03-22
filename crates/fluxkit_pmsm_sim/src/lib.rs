#![no_std]
#![forbid(unsafe_code)]
#![deny(rust_2018_idioms)]
#![warn(missing_docs, missing_debug_implementations)]
//! Ideal PMSM plant emulator for host-side and `no_std` simulation.
//!
//! The model integrates the standard `d/q` electrical equations and a simple
//! rigid-shaft mechanical equation:
//!
//! $$v_d = R i_d + L_d \frac{d i_d}{dt} - \omega_e L_q i_q$$
//! $$v_q = R i_q + L_q \frac{d i_q}{dt} + \omega_e (L_d i_d + \psi_m)$$
//! $$\tau_e = \frac{3}{2} p \left(\psi_m i_q + (L_d - L_q) i_d i_q \right)$$
//! $$J \frac{d \omega_m}{dt} = \tau_e - \tau_{load} - B \omega_m - \tau_{static}$$
//!
//! Phase-domain excitation is supported too, but at the averaged plant-input
//! level:
//!
//! - [`PmsmModel::step_phase_voltage`] accepts an `a/b/c` phase-voltage vector
//! - [`PmsmModel::step_phase_duty`] converts duty plus `Vbus` into an averaged
//!   zero-sum phase-voltage vector
//!
//! These `a/b/c` paths are suitable for controller integration tests and
//! averaged inverter behavior. They do not model transistor switching edges,
//! deadtime, PWM ripple within a carrier cycle, or bridge parasitics.
//!
//! The simulator is intentionally deterministic and allocation-free. It is
//! intended for controller integration tests rather than finite-element or
//! inverter-switching-accurate simulation.
//!
pub mod error;
pub mod params;
pub mod state;

use fluxkit_math::angle::mechanical_to_electrical;
use fluxkit_math::{
    ElectricalAngle, MechanicalAngle, clamp, clarke,
    frame::{Abc, AlphaBeta, Dq},
    inverse_clarke, inverse_park,
    modulation::PhaseDuty,
    park,
    units::{Amps, NewtonMeters, RadPerSec, Volts},
};

pub use error::Error;
pub use params::{ActuatorPlantParams, PmsmParams};
pub use state::{PmsmSnapshot, PmsmState};

/// Ideal PMSM plant model with rigid-shaft mechanical dynamics.
#[derive(Clone, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct PmsmModel {
    params: PmsmParams,
    state: PmsmState,
}

impl PmsmModel {
    /// Creates a plant model with explicit parameters and initial state.
    pub fn new(params: PmsmParams, state: PmsmState) -> Result<Self, Error> {
        if !validate_params(&params) || !validate_state(&state) {
            return Err(Error::InvalidParameters);
        }

        Ok(Self { params, state })
    }

    /// Creates a plant model with zero current, angle, and velocity.
    pub fn new_zeroed(params: PmsmParams) -> Result<Self, Error> {
        Self::new(
            params,
            PmsmState {
                mechanical_angle: MechanicalAngle::new(0.0),
                mechanical_velocity: RadPerSec::ZERO,
                current_dq: Dq::new(Amps::ZERO, Amps::ZERO),
            },
        )
    }

    /// Returns the static plant parameters.
    #[inline]
    pub const fn params(&self) -> &PmsmParams {
        &self.params
    }

    /// Returns the current dynamic plant state.
    #[inline]
    pub const fn state(&self) -> &PmsmState {
        &self.state
    }

    /// Steps the plant using a rotating-frame `d/q` voltage input held constant over the step.
    pub fn step_vdq(
        &mut self,
        applied_vdq: Dq<Volts>,
        load_torque: NewtonMeters,
        dt_seconds: f32,
    ) -> Result<PmsmSnapshot, Error> {
        if !validate_step_input(applied_vdq.map(|v| v.get()), load_torque, dt_seconds) {
            return Err(Error::InvalidStepInput);
        }

        let applied_vdq =
            clamp_vdq(applied_vdq.map(|v| v.get()), self.params.max_voltage_mag).map(Volts::new);
        let next = self.integrate_dq(applied_vdq.map(|v| v.get()), load_torque.get(), dt_seconds);
        self.state = next;
        Ok(self.snapshot_from_vdq(applied_vdq, None))
    }

    /// Steps the plant using a stationary-frame `alpha/beta` voltage input held constant over the step.
    ///
    /// This is an averaged stator-voltage input, not a switching-waveform
    /// model.
    pub fn step_alpha_beta(
        &mut self,
        applied_alpha_beta: AlphaBeta<Volts>,
        load_torque: NewtonMeters,
        dt_seconds: f32,
    ) -> Result<PmsmSnapshot, Error> {
        if !validate_step_input_ab(applied_alpha_beta.map(|v| v.get()), load_torque, dt_seconds) {
            return Err(Error::InvalidStepInput);
        }

        let applied_alpha_beta = clamp_alpha_beta(
            applied_alpha_beta.map(|v| v.get()),
            self.params.max_voltage_mag,
        )
        .map(Volts::new);
        let next = self.integrate_alpha_beta(
            applied_alpha_beta.map(|v| v.get()),
            load_torque.get(),
            dt_seconds,
        );
        self.state = next;
        Ok(self.snapshot_from_alpha_beta(applied_alpha_beta, None))
    }

    /// Steps the plant using a zero-sum phase-voltage vector.
    ///
    /// Any common-mode component is ignored by the Clarke transform, and the
    /// optional voltage magnitude limit is still applied before integration.
    ///
    /// This is an averaged `a/b/c` plant input path. It does not model bridge
    /// switching events or intra-cycle PWM ripple.
    pub fn step_phase_voltage(
        &mut self,
        phase_voltage: Abc<Volts>,
        load_torque: NewtonMeters,
        dt_seconds: f32,
    ) -> Result<PmsmSnapshot, Error> {
        if !validate_phase_voltage(phase_voltage) || !dt_seconds.is_finite() || dt_seconds <= 0.0 {
            return Err(Error::InvalidStepInput);
        }

        self.step_alpha_beta(
            clarke(phase_voltage.map(|v| v.get())).map(Volts::new),
            load_torque,
            dt_seconds,
        )
    }

    /// Steps the plant using PWM duty plus DC bus voltage.
    ///
    /// The simulator removes common-mode voltage because the motor windings only
    /// see the zero-sum line-to-neutral component. The optional voltage
    /// magnitude limit is then applied to the resulting stator-voltage vector.
    ///
    /// This is still an averaged inverter model: duty is converted directly
    /// into averaged phase voltage over the step rather than simulating switch
    /// transitions.
    pub fn step_phase_duty(
        &mut self,
        phase_duty: PhaseDuty,
        bus_voltage: Volts,
        load_torque: NewtonMeters,
        dt_seconds: f32,
    ) -> Result<PmsmSnapshot, Error> {
        if !validate_phase_duty(phase_duty)
            || !bus_voltage.get().is_finite()
            || bus_voltage.get() <= 0.0
            || !load_torque.get().is_finite()
            || !dt_seconds.is_finite()
            || dt_seconds <= 0.0
        {
            return Err(Error::InvalidStepInput);
        }

        let phase_voltage = phase_voltage_from_duty(phase_duty, bus_voltage);
        let applied_alpha_beta = clamp_alpha_beta(
            clarke(phase_voltage.map(|v| v.get())),
            self.params.max_voltage_mag,
        )
        .map(Volts::new);
        let next = self.integrate_alpha_beta(
            applied_alpha_beta.map(|v| v.get()),
            load_torque.get(),
            dt_seconds,
        );
        self.state = next;
        Ok(self.snapshot_from_alpha_beta(applied_alpha_beta, Some(phase_duty)))
    }

    fn integrate_dq(&self, applied_vdq: Dq<f32>, load_torque: f32, dt_seconds: f32) -> PmsmState {
        let x0 = PlantState::from(self.state);
        let k1 = self.derivatives_from_dq(x0, applied_vdq, load_torque);
        let k2 = self.derivatives_from_dq(
            x0.add_scaled(k1, 0.5 * dt_seconds),
            applied_vdq,
            load_torque,
        );
        let k3 = self.derivatives_from_dq(
            x0.add_scaled(k2, 0.5 * dt_seconds),
            applied_vdq,
            load_torque,
        );
        let k4 = self.derivatives_from_dq(x0.add_scaled(k3, dt_seconds), applied_vdq, load_torque);
        x0.integrate(k1, k2, k3, k4, dt_seconds).into()
    }

    fn integrate_alpha_beta(
        &self,
        applied_alpha_beta: AlphaBeta<f32>,
        load_torque: f32,
        dt_seconds: f32,
    ) -> PmsmState {
        let x0 = PlantState::from(self.state);
        let k1 = self.derivatives_from_alpha_beta(x0, applied_alpha_beta, load_torque);
        let k2 = self.derivatives_from_alpha_beta(
            x0.add_scaled(k1, 0.5 * dt_seconds),
            applied_alpha_beta,
            load_torque,
        );
        let k3 = self.derivatives_from_alpha_beta(
            x0.add_scaled(k2, 0.5 * dt_seconds),
            applied_alpha_beta,
            load_torque,
        );
        let k4 = self.derivatives_from_alpha_beta(
            x0.add_scaled(k3, dt_seconds),
            applied_alpha_beta,
            load_torque,
        );
        x0.integrate(k1, k2, k3, k4, dt_seconds).into()
    }

    fn derivatives_from_alpha_beta(
        &self,
        state: PlantState,
        applied_alpha_beta: AlphaBeta<f32>,
        load_torque: f32,
    ) -> PlantDerivative {
        let theta_e = self.electrical_angle_from_theta(state.theta_mech);
        let applied_vdq = park(applied_alpha_beta, theta_e);
        self.derivatives_from_dq(state, applied_vdq, load_torque)
    }

    fn derivatives_from_dq(
        &self,
        state: PlantState,
        applied_vdq: Dq<f32>,
        load_torque: f32,
    ) -> PlantDerivative {
        let params = &self.params;
        let pole_pairs = params.pole_pairs as f32;
        let resistance = params.phase_resistance_ohm.get();
        let ld = params.d_inductance_h.get();
        let lq = params.q_inductance_h.get();
        let flux = params.flux_linkage_weber.get();
        let omega_e = state.omega_mech * pole_pairs;
        let applied_vdq = clamp_vdq(applied_vdq, params.max_voltage_mag);

        let did = (applied_vdq.d - resistance * state.id + omega_e * lq * state.iq) / ld;
        let diq = (applied_vdq.q - resistance * state.iq - omega_e * (ld * state.id + flux)) / lq;
        let torque = electromagnetic_torque(params, state.id, state.iq);
        let friction = params.viscous_friction_nm_per_rad_per_sec * state.omega_mech
            + static_friction(
                params.static_friction_nm.get(),
                state.omega_mech,
                torque - load_torque,
            );
        let actuator_parasitic = actuator_parasitic_torque(&params.actuator, state.omega_mech);
        let reflected_actuator_parasitic =
            actuator_parasitic / params.actuator.gear_ratio.max(f32::EPSILON);
        let domega =
            (torque - load_torque - friction - reflected_actuator_parasitic) / params.inertia_kg_m2;

        PlantDerivative {
            did,
            diq,
            domega,
            dtheta: state.omega_mech,
        }
    }

    fn snapshot_from_vdq(
        &self,
        applied_vdq: Dq<Volts>,
        phase_duty: Option<PhaseDuty>,
    ) -> PmsmSnapshot {
        let electrical_angle = self.electrical_angle();
        let applied_alpha_beta =
            inverse_park(applied_vdq.map(|v| v.get()), electrical_angle.get()).map(Volts::new);
        let phase_voltage = inverse_clarke(applied_alpha_beta.map(|v| v.get())).map(Volts::new);
        self.snapshot_common(
            electrical_angle,
            applied_alpha_beta,
            applied_vdq,
            phase_voltage,
            phase_duty,
        )
    }

    fn snapshot_from_alpha_beta(
        &self,
        applied_alpha_beta: AlphaBeta<Volts>,
        phase_duty: Option<PhaseDuty>,
    ) -> PmsmSnapshot {
        let electrical_angle = self.electrical_angle();
        let applied_vdq =
            park(applied_alpha_beta.map(|v| v.get()), electrical_angle.get()).map(Volts::new);
        let phase_voltage = inverse_clarke(applied_alpha_beta.map(|v| v.get())).map(Volts::new);
        self.snapshot_common(
            electrical_angle,
            applied_alpha_beta,
            applied_vdq,
            phase_voltage,
            phase_duty,
        )
    }

    fn snapshot_common(
        &self,
        electrical_angle: ElectricalAngle,
        applied_alpha_beta: AlphaBeta<Volts>,
        applied_vdq: Dq<Volts>,
        phase_voltage: Abc<Volts>,
        phase_duty: Option<PhaseDuty>,
    ) -> PmsmSnapshot {
        let current_alpha_beta = inverse_park(
            self.state.current_dq.map(|i| i.get()),
            electrical_angle.get(),
        );
        let phase_currents = inverse_clarke(current_alpha_beta).map(Amps::new);

        PmsmSnapshot {
            state: self.state,
            electrical_angle: electrical_angle.wrapped_pm_pi(),
            wrapped_mechanical_angle: self.state.mechanical_angle.wrapped_0_2pi(),
            wrapped_output_angle: self.output_angle().wrapped_0_2pi(),
            output_velocity: self.output_velocity(),
            phase_currents,
            applied_alpha_beta,
            applied_vdq,
            phase_voltage,
            phase_duty,
            electromagnetic_torque: NewtonMeters::new(electromagnetic_torque(
                &self.params,
                self.state.current_dq.d.get(),
                self.state.current_dq.q.get(),
            )),
            actuator_parasitic_torque: NewtonMeters::new(actuator_parasitic_torque(
                &self.params.actuator,
                self.state.mechanical_velocity.get(),
            )),
        }
    }

    fn electrical_angle(&self) -> ElectricalAngle {
        mechanical_to_electrical(self.state.mechanical_angle, self.params.pole_pairs as u32)
    }

    fn output_angle(&self) -> MechanicalAngle {
        MechanicalAngle::new(
            self.state.mechanical_angle.get() / self.params.actuator.gear_ratio.max(f32::EPSILON),
        )
    }

    fn output_velocity(&self) -> RadPerSec {
        RadPerSec::new(
            self.state.mechanical_velocity.get()
                / self.params.actuator.gear_ratio.max(f32::EPSILON),
        )
    }

    fn electrical_angle_from_theta(&self, theta_mech: f32) -> f32 {
        mechanical_to_electrical(
            MechanicalAngle::new(theta_mech),
            self.params.pole_pairs as u32,
        )
        .get()
    }
}

#[derive(Clone, Copy)]
struct PlantState {
    id: f32,
    iq: f32,
    omega_mech: f32,
    theta_mech: f32,
}

impl PlantState {
    fn add_scaled(self, rhs: PlantDerivative, scale: f32) -> Self {
        Self {
            id: self.id + rhs.did * scale,
            iq: self.iq + rhs.diq * scale,
            omega_mech: self.omega_mech + rhs.domega * scale,
            theta_mech: self.theta_mech + rhs.dtheta * scale,
        }
    }

    fn integrate(
        self,
        k1: PlantDerivative,
        k2: PlantDerivative,
        k3: PlantDerivative,
        k4: PlantDerivative,
        dt: f32,
    ) -> Self {
        let scale = dt / 6.0;
        Self {
            id: self.id + scale * (k1.did + 2.0 * k2.did + 2.0 * k3.did + k4.did),
            iq: self.iq + scale * (k1.diq + 2.0 * k2.diq + 2.0 * k3.diq + k4.diq),
            omega_mech: self.omega_mech
                + scale * (k1.domega + 2.0 * k2.domega + 2.0 * k3.domega + k4.domega),
            theta_mech: self.theta_mech
                + scale * (k1.dtheta + 2.0 * k2.dtheta + 2.0 * k3.dtheta + k4.dtheta),
        }
    }
}

impl From<PmsmState> for PlantState {
    fn from(value: PmsmState) -> Self {
        Self {
            id: value.current_dq.d.get(),
            iq: value.current_dq.q.get(),
            omega_mech: value.mechanical_velocity.get(),
            theta_mech: value.mechanical_angle.get(),
        }
    }
}

impl From<PlantState> for PmsmState {
    fn from(value: PlantState) -> Self {
        Self {
            mechanical_angle: MechanicalAngle::new(value.theta_mech),
            mechanical_velocity: RadPerSec::new(value.omega_mech),
            current_dq: Dq::new(Amps::new(value.id), Amps::new(value.iq)),
        }
    }
}

#[derive(Clone, Copy)]
struct PlantDerivative {
    did: f32,
    diq: f32,
    domega: f32,
    dtheta: f32,
}

fn electromagnetic_torque(params: &PmsmParams, id: f32, iq: f32) -> f32 {
    let pole_pairs = params.pole_pairs as f32;
    1.5 * pole_pairs
        * (params.flux_linkage_weber.get() * iq
            + (params.d_inductance_h.get() - params.q_inductance_h.get()) * id * iq)
}

fn static_friction(static_friction: f32, omega_mech: f32, net_torque_without_static: f32) -> f32 {
    if static_friction <= 0.0 {
        return 0.0;
    }

    if omega_mech > 0.0 {
        static_friction
    } else if omega_mech < 0.0 {
        -static_friction
    } else if net_torque_without_static > static_friction {
        static_friction
    } else if net_torque_without_static < -static_friction {
        -static_friction
    } else {
        net_torque_without_static
    }
}

fn actuator_parasitic_torque(params: &ActuatorPlantParams, motor_omega_mech: f32) -> f32 {
    let gear_ratio = params.gear_ratio.max(f32::EPSILON);
    let output_velocity = motor_omega_mech / gear_ratio;
    let blend_band = params.zero_velocity_blend_band.get().max(1.0e-6);
    let direction = clamp(output_velocity / blend_band, -1.0, 1.0);
    let positive_weight = 0.5 * (direction + 1.0);
    let negative_weight = 1.0 - positive_weight;
    let coulomb = positive_weight * params.positive_coulomb_torque.get()
        + negative_weight * params.negative_coulomb_torque.get();
    let breakaway = positive_weight * params.positive_breakaway_torque.get()
        + negative_weight * params.negative_breakaway_torque.get();
    let viscous_coefficient = positive_weight * params.positive_viscous_coefficient
        + negative_weight * params.negative_viscous_coefficient;
    let breakaway_weight = 1.0 - clamp(output_velocity.abs() / blend_band, 0.0, 1.0);
    direction * (coulomb + breakaway_weight * breakaway)
        + viscous_coefficient * output_velocity
        + params.constant_bias_torque.get()
}

fn clamp_vdq(vdq: Dq<f32>, max_voltage_mag: Option<Volts>) -> Dq<f32> {
    match max_voltage_mag {
        Some(limit) => fluxkit_math::limit_norm_dq(vdq, limit.get()),
        None => vdq,
    }
}

fn clamp_alpha_beta(v: AlphaBeta<f32>, max_voltage_mag: Option<Volts>) -> AlphaBeta<f32> {
    match max_voltage_mag {
        Some(limit) => fluxkit_math::limit_norm_ab(v, limit.get()),
        None => v,
    }
}

fn phase_voltage_from_duty(phase_duty: PhaseDuty, bus_voltage: Volts) -> Abc<Volts> {
    let bus_voltage = bus_voltage.get();
    let mean_duty = (phase_duty.a.get() + phase_duty.b.get() + phase_duty.c.get()) / 3.0;
    Abc::new(
        Volts::new((phase_duty.a.get() - mean_duty) * bus_voltage),
        Volts::new((phase_duty.b.get() - mean_duty) * bus_voltage),
        Volts::new((phase_duty.c.get() - mean_duty) * bus_voltage),
    )
}

fn validate_params(params: &PmsmParams) -> bool {
    params.pole_pairs > 0
        && finite_positive(params.phase_resistance_ohm.get())
        && finite_positive(params.d_inductance_h.get())
        && finite_positive(params.q_inductance_h.get())
        && finite_positive(params.flux_linkage_weber.get())
        && finite_positive(params.inertia_kg_m2)
        && finite_non_negative(params.viscous_friction_nm_per_rad_per_sec)
        && finite_non_negative(params.static_friction_nm.get())
        && finite_positive(params.actuator.gear_ratio)
        && finite_non_negative(params.actuator.positive_breakaway_torque.get())
        && finite_non_negative(params.actuator.negative_breakaway_torque.get())
        && finite_non_negative(params.actuator.positive_coulomb_torque.get())
        && finite_non_negative(params.actuator.negative_coulomb_torque.get())
        && finite_non_negative(params.actuator.positive_viscous_coefficient)
        && finite_non_negative(params.actuator.negative_viscous_coefficient)
        && finite_non_negative(params.actuator.zero_velocity_blend_band.get())
        && params.actuator.constant_bias_torque.get().is_finite()
        && params
            .max_voltage_mag
            .map(|voltage| finite_positive(voltage.get()))
            .unwrap_or(true)
}

fn validate_state(state: &PmsmState) -> bool {
    state.mechanical_angle.get().is_finite()
        && state.mechanical_velocity.get().is_finite()
        && state.current_dq.d.get().is_finite()
        && state.current_dq.q.get().is_finite()
}

fn validate_step_input(applied_vdq: Dq<f32>, load_torque: NewtonMeters, dt_seconds: f32) -> bool {
    applied_vdq.d.is_finite()
        && applied_vdq.q.is_finite()
        && load_torque.get().is_finite()
        && dt_seconds.is_finite()
        && dt_seconds > 0.0
}

fn validate_step_input_ab(
    applied_alpha_beta: AlphaBeta<f32>,
    load_torque: NewtonMeters,
    dt_seconds: f32,
) -> bool {
    applied_alpha_beta.alpha.is_finite()
        && applied_alpha_beta.beta.is_finite()
        && load_torque.get().is_finite()
        && dt_seconds.is_finite()
        && dt_seconds > 0.0
}

fn validate_phase_voltage(phase_voltage: Abc<Volts>) -> bool {
    phase_voltage.a.get().is_finite()
        && phase_voltage.b.get().is_finite()
        && phase_voltage.c.get().is_finite()
}

fn validate_phase_duty(phase_duty: PhaseDuty) -> bool {
    [phase_duty.a.get(), phase_duty.b.get(), phase_duty.c.get()]
        .into_iter()
        .all(|duty| duty.is_finite() && (0.0..=1.0).contains(&duty))
}

fn finite_positive(value: f32) -> bool {
    value.is_finite() && value > 0.0
}

fn finite_non_negative(value: f32) -> bool {
    value.is_finite() && value >= 0.0
}

#[cfg(test)]
mod tests {
    use super::{Error, PmsmModel, PmsmParams};
    use fluxkit_math::{
        MechanicalAngle,
        frame::{Abc, AlphaBeta, Dq},
        inverse_clarke,
        modulation::PhaseDuty,
        units::{Amps, Duty, Henries, NewtonMeters, Ohms, RadPerSec, Volts, Webers},
    };

    fn test_params() -> PmsmParams {
        PmsmParams {
            pole_pairs: 7,
            phase_resistance_ohm: Ohms::new(0.12),
            d_inductance_h: Henries::new(0.00003),
            q_inductance_h: Henries::new(0.00003),
            flux_linkage_weber: Webers::new(0.005),
            inertia_kg_m2: 0.0002,
            viscous_friction_nm_per_rad_per_sec: 0.0001,
            static_friction_nm: NewtonMeters::new(0.0),
            actuator: crate::ActuatorPlantParams::disabled(),
            max_voltage_mag: None,
        }
    }

    #[test]
    fn zero_voltage_keeps_zero_state() {
        let mut plant = PmsmModel::new_zeroed(test_params()).unwrap();
        let snapshot = plant
            .step_vdq(
                Dq::new(Volts::ZERO, Volts::ZERO),
                NewtonMeters::ZERO,
                1.0 / 20_000.0,
            )
            .unwrap();

        assert_eq!(snapshot.state.current_dq, Dq::new(Amps::ZERO, Amps::ZERO));
        assert_eq!(snapshot.state.mechanical_velocity, RadPerSec::ZERO);
    }

    #[test]
    fn positive_q_voltage_builds_positive_q_current() {
        let mut plant = PmsmModel::new_zeroed(test_params()).unwrap();
        let snapshot = plant
            .step_vdq(
                Dq::new(Volts::ZERO, Volts::new(3.0)),
                NewtonMeters::ZERO,
                1.0 / 20_000.0,
            )
            .unwrap();

        assert!(snapshot.state.current_dq.q.get() > 0.0);
    }

    #[test]
    fn positive_q_excitation_accelerates_rotor() {
        let mut plant = PmsmModel::new_zeroed(test_params()).unwrap();

        for _ in 0..200 {
            plant
                .step_vdq(
                    Dq::new(Volts::ZERO, Volts::new(4.0)),
                    NewtonMeters::ZERO,
                    1.0 / 20_000.0,
                )
                .unwrap();
        }

        assert!(plant.state().mechanical_velocity.get() > 0.0);
    }

    #[test]
    fn duty_helper_removes_common_mode() {
        let mut plant = PmsmModel::new_zeroed(test_params()).unwrap();
        let snapshot = plant
            .step_phase_duty(
                PhaseDuty::new(Duty::new(0.5), Duty::new(0.5), Duty::new(0.5)),
                Volts::new(24.0),
                NewtonMeters::ZERO,
                1.0 / 20_000.0,
            )
            .unwrap();

        assert_eq!(
            snapshot.phase_voltage,
            Abc::new(Volts::ZERO, Volts::ZERO, Volts::ZERO)
        );
    }

    #[test]
    fn alpha_beta_and_phase_voltage_paths_are_consistent() {
        let mut plant_ab = PmsmModel::new_zeroed(test_params()).unwrap();
        let mut plant_phase = PmsmModel::new_zeroed(test_params()).unwrap();
        let applied_ab = AlphaBeta::new(Volts::new(2.0), Volts::new(-1.0));
        let phase_voltage = inverse_clarke(applied_ab.map(|v| v.get())).map(Volts::new);

        let snapshot_ab = plant_ab
            .step_alpha_beta(applied_ab, NewtonMeters::ZERO, 1.0 / 20_000.0)
            .unwrap();
        let snapshot_phase = plant_phase
            .step_phase_voltage(phase_voltage, NewtonMeters::ZERO, 1.0 / 20_000.0)
            .unwrap();

        assert_eq!(snapshot_ab.state, snapshot_phase.state);
    }

    #[test]
    fn invalid_parameters_are_rejected() {
        let mut params = test_params();
        params.inertia_kg_m2 = 0.0;
        assert_eq!(
            PmsmModel::new_zeroed(params).unwrap_err(),
            Error::InvalidParameters
        );
    }

    #[test]
    fn wrapped_mechanical_angle_matches_unwrapped_state() {
        let mut plant = PmsmModel::new(
            test_params(),
            crate::PmsmState {
                mechanical_angle: MechanicalAngle::new(core::f32::consts::TAU + 0.25),
                mechanical_velocity: RadPerSec::ZERO,
                current_dq: Dq::new(Amps::ZERO, Amps::ZERO),
            },
        )
        .unwrap();

        let snapshot = plant
            .step_vdq(
                Dq::new(Volts::ZERO, Volts::ZERO),
                NewtonMeters::ZERO,
                1.0 / 20_000.0,
            )
            .unwrap();

        assert_eq!(
            snapshot.wrapped_mechanical_angle,
            snapshot.state.mechanical_angle.wrapped_0_2pi()
        );
        assert_eq!(
            snapshot.wrapped_output_angle,
            snapshot.wrapped_mechanical_angle
        );
    }

    #[test]
    fn invalid_phase_duty_step_input_is_rejected() {
        let mut plant = PmsmModel::new_zeroed(test_params()).unwrap();

        assert_eq!(
            plant
                .step_phase_duty(
                    PhaseDuty::new(Duty::new(0.5), Duty::new(0.25), Duty::new(0.75)),
                    Volts::new(24.0),
                    NewtonMeters::ZERO,
                    0.0,
                )
                .unwrap_err(),
            Error::InvalidStepInput
        );
    }

    #[test]
    fn voltage_limit_is_reflected_in_snapshot() {
        let mut params = test_params();
        params.max_voltage_mag = Some(Volts::new(2.0));
        let mut plant = PmsmModel::new_zeroed(params).unwrap();

        let snapshot = plant
            .step_vdq(
                Dq::new(Volts::new(3.0), Volts::new(4.0)),
                NewtonMeters::ZERO,
                1.0 / 20_000.0,
            )
            .unwrap();

        let applied_mag_sq = snapshot.applied_vdq.d.get() * snapshot.applied_vdq.d.get()
            + snapshot.applied_vdq.q.get() * snapshot.applied_vdq.q.get();

        assert!((applied_mag_sq - 4.0).abs() < 1.0e-5);
    }

    #[test]
    fn initial_state_can_be_non_zero() {
        let mut plant = PmsmModel::new(
            test_params(),
            crate::PmsmState {
                mechanical_angle: MechanicalAngle::new(1.0),
                mechanical_velocity: RadPerSec::new(2.0),
                current_dq: Dq::new(Amps::new(0.5), Amps::new(-0.25)),
            },
        )
        .unwrap();

        let snapshot = plant
            .step_vdq(
                Dq::new(Volts::ZERO, Volts::ZERO),
                NewtonMeters::ZERO,
                1.0 / 20_000.0,
            )
            .unwrap();

        assert!(snapshot.state.mechanical_angle.get() > 1.0);
    }
}
