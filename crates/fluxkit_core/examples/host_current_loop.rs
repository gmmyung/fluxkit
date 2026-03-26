use fluxkit_core::{
    ActuatorEstimate, ActuatorLimits, ActuatorModel, ActuatorParams, ControlMode,
    CurrentLoopConfig, FastLoopInput, InverterParams, MotorController, MotorLimits, MotorModel,
    MotorParams, RotorEstimate,
};
use fluxkit_math::{
    ContinuousMechanicalAngle,
    frame::Dq,
    inverse_clarke, inverse_park,
    units::{Amps, Duty, Henries, Hertz, Ohms, RadPerSec, Volts, Webers},
};

fn main() {
    let motor = MotorParams::from_model_and_limits(
        MotorModel {
            pole_pairs: 7,
            phase_resistance_ohm_ref: Ohms::new(0.12),
            d_inductance_h: Henries::new(0.000_03),
            q_inductance_h: Henries::new(0.000_03),
            flux_linkage_weber: Webers::new(0.005),
            electrical_angle_offset: fluxkit_math::ElectricalAngle::new(0.0),
        },
        MotorLimits {
            max_phase_current: Amps::new(20.0),
            max_mech_speed: Some(RadPerSec::new(500.0)),
        },
    );
    let inverter = InverterParams {
        pwm_frequency_hz: Hertz::new(20_000.0),
        min_duty: Duty::new(0.0),
        max_duty: Duty::new(1.0),
        min_bus_voltage: Volts::new(6.0),
        max_bus_voltage: Volts::new(60.0),
        max_voltage_command: Volts::new(24.0),
    };
    let config = CurrentLoopConfig {
        kp_d: 1.5,
        ki_d: 25.0,
        kp_q: 1.5,
        ki_q: 25.0,
        velocity_kp: 0.5,
        velocity_ki: 10.0,
        position_kp: 4.0,
        position_ki: 0.0,
        max_voltage_mag: Volts::new(12.0),
        id_ref_default: Amps::ZERO,
        max_id_target: Amps::new(10.0),
        max_iq_target: Amps::new(10.0),
        max_velocity_target: RadPerSec::new(500.0),
        max_current_ref_derivative_amps_per_sec: 10_000.0,
        enable_current_feedforward: true,
    };
    let actuator = ActuatorParams::from_model_limits_and_compensation(
        ActuatorModel { gear_ratio: 5.0 },
        ActuatorLimits {
            max_output_velocity: Some(RadPerSec::new(100.0)),
            max_output_torque: Some(fluxkit_math::units::NewtonMeters::new(20.0)),
        },
        fluxkit_core::ActuatorCompensationConfig::disabled(),
    );

    let mut controller = MotorController::new(motor, inverter, actuator, config);
    controller.set_mode(ControlMode::Current);
    controller.enable();

    let dt = 1.0 / 20_000.0;
    let electrical_speed = 200.0;
    let mut measured_iq = 0.0;

    println!("step,iq_target,measured_iq,vd,vq,duty_a,duty_b,duty_c,saturated");

    for step in 0..400 {
        let time = step as f32 * dt;
        let iq_target = if step < 80 { 0.0 } else { 4.0 };
        controller.set_iq_target(Amps::new(iq_target));

        measured_iq += (iq_target - measured_iq) * 0.04;

        let angle = electrical_speed * time;
        let current_dq = Dq::new(0.0, measured_iq);
        let current_ab = inverse_park(current_dq, angle);
        let phase_currents = inverse_clarke(current_ab).map(Amps::new);

        let output = controller.fast_tick(FastLoopInput {
            phase_currents,
            bus_voltage: Volts::new(24.0),
            winding_temperature_c: 25.0,
            rotor: RotorEstimate {
                mechanical_angle: ContinuousMechanicalAngle::new(
                    angle / motor.model().pole_pairs as f32,
                ),
                mechanical_velocity: RadPerSec::new(
                    electrical_speed / motor.model().pole_pairs as f32,
                ),
            },
            actuator: ActuatorEstimate {
                output_angle: ContinuousMechanicalAngle::new(
                    angle / motor.model().pole_pairs as f32 / 5.0,
                ),
                output_velocity: RadPerSec::new(
                    electrical_speed / motor.model().pole_pairs as f32 / 5.0,
                ),
            },
            dt_seconds: dt,
        });

        println!(
            "{step},{iq_target:.3},{:.3},{:.3},{:.3},{:.5},{:.5},{:.5},{}",
            output.measured_idq.q.get(),
            output.commanded_vdq.d.get(),
            output.commanded_vdq.q.get(),
            output.phase_duty.a.get(),
            output.phase_duty.b.get(),
            output.phase_duty.c.get(),
            output.saturated
        );
    }
}
