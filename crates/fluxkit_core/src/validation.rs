//! Validation helpers for controller configuration and loop inputs.

use fluxkit_math::frame::Abc;

use crate::{
    actuator::ActuatorParams,
    config::CurrentLoopConfig,
    error::Error,
    io::FastLoopInput,
    params::{InverterParams, MotorParams},
};

/// Returns `true` when the static controller configuration is valid enough to run.
pub fn validate_controller_config(
    motor: &MotorParams,
    inverter: &InverterParams,
    actuator: &ActuatorParams,
    config: &CurrentLoopConfig,
) -> bool {
    motor.pole_pairs > 0
        && finite_positive(motor.phase_resistance_ohm.get())
        && finite_positive(motor.d_inductance_h.get())
        && finite_positive(motor.q_inductance_h.get())
        && motor
            .flux_linkage_weber
            .map(|flux| finite_non_negative(flux.get()))
            .unwrap_or(true)
        && finite_positive(motor.max_phase_current.get())
        && motor
            .max_mech_speed
            .map(|speed| finite_positive(speed.get()))
            .unwrap_or(true)
        && finite_positive(actuator.gear_ratio)
        && actuator
            .max_output_velocity
            .map(|speed| finite_positive(speed.get()))
            .unwrap_or(true)
        && actuator
            .max_output_torque
            .map(|torque| finite_positive(torque.get()))
            .unwrap_or(true)
        && actuator.compensation.max_total_torque.get().is_finite()
        && actuator.compensation.max_total_torque.get() >= 0.0
        && finite_non_negative(
            actuator
                .compensation
                .friction
                .positive_breakaway_torque
                .get(),
        )
        && finite_non_negative(
            actuator
                .compensation
                .friction
                .negative_breakaway_torque
                .get(),
        )
        && finite_non_negative(actuator.compensation.friction.positive_coulomb_torque.get())
        && finite_non_negative(actuator.compensation.friction.negative_coulomb_torque.get())
        && finite_non_negative(
            actuator
                .compensation
                .friction
                .zero_velocity_blend_band
                .get(),
        )
        && actuator
            .compensation
            .friction
            .positive_viscous_coefficient
            .is_finite()
        && actuator.compensation.friction.positive_viscous_coefficient >= 0.0
        && actuator
            .compensation
            .friction
            .negative_viscous_coefficient
            .is_finite()
        && actuator.compensation.friction.negative_viscous_coefficient >= 0.0
        && actuator
            .compensation
            .inertia
            .reflected_inertia_kg_m2
            .is_finite()
        && actuator.compensation.inertia.reflected_inertia_kg_m2 >= 0.0
        && actuator
            .compensation
            .inertia
            .max_acceleration_rad_per_sec2
            .is_finite()
        && actuator.compensation.inertia.max_acceleration_rad_per_sec2 >= 0.0
        && actuator
            .compensation
            .load
            .constant_bias_torque
            .get()
            .is_finite()
        && finite_positive(inverter.pwm_frequency_hz.get())
        && finite_in_range(inverter.min_duty.get(), 0.0, 1.0)
        && finite_in_range(inverter.max_duty.get(), 0.0, 1.0)
        && inverter.min_duty.get() <= inverter.max_duty.get()
        && finite_positive(inverter.min_bus_voltage.get())
        && finite_positive(inverter.max_bus_voltage.get())
        && inverter.min_bus_voltage.get() <= inverter.max_bus_voltage.get()
        && finite_positive(inverter.max_voltage_command.get())
        && config.kp_d.is_finite()
        && config.ki_d.is_finite()
        && config.kp_q.is_finite()
        && config.ki_q.is_finite()
        && config.velocity_kp.is_finite()
        && config.velocity_ki.is_finite()
        && config.position_kp.is_finite()
        && config.position_ki.is_finite()
        && finite_positive(config.max_voltage_mag.get())
        && finite_non_negative(config.max_id_target.get())
        && finite_non_negative(config.max_iq_target.get())
        && finite_non_negative(config.max_velocity_target.get())
        && finite_non_negative(config.max_current_ref_derivative_amps_per_sec)
}

/// Validates one fast-loop input frame.
pub fn validate_fast_loop_input(
    input: &FastLoopInput,
    inverter: &InverterParams,
) -> Result<(), Error> {
    if !abc_is_finite(input.phase_currents.map(|x| x.get())) {
        return Err(Error::InvalidPhaseCurrent);
    }

    let angle = input.rotor.electrical_angle.get();
    if !angle.is_finite()
        || !input.rotor.mechanical_angle.get().is_finite()
        || !input.rotor.mechanical_velocity.get().is_finite()
        || !input.actuator.output_angle.get().is_finite()
        || !input.actuator.output_velocity.get().is_finite()
    {
        return Err(Error::InvalidRotorAngle);
    }

    let bus_voltage = input.bus_voltage.get();
    if !bus_voltage.is_finite()
        || bus_voltage < inverter.min_bus_voltage.get()
        || bus_voltage > inverter.max_bus_voltage.get()
    {
        return Err(Error::InvalidBusVoltage);
    }

    if !input.dt_seconds.is_finite() || input.dt_seconds <= 0.0 {
        return Err(Error::TimingOverrun);
    }

    let expected_dt = 1.0 / inverter.pwm_frequency_hz.get();
    if input.dt_seconds > expected_dt * 2.0 {
        return Err(Error::TimingOverrun);
    }

    Ok(())
}

#[inline]
fn abc_is_finite(v: Abc<f32>) -> bool {
    v.a.is_finite() && v.b.is_finite() && v.c.is_finite()
}

#[inline]
fn finite_positive(value: f32) -> bool {
    value.is_finite() && value > 0.0
}

#[inline]
fn finite_non_negative(value: f32) -> bool {
    value.is_finite() && value >= 0.0
}

#[inline]
fn finite_in_range(value: f32, min: f32, max: f32) -> bool {
    value.is_finite() && value >= min && value <= max
}
