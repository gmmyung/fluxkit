use fluxkit_core::{
    ControlMode, CurrentLoopConfig, FastLoopInput, InverterParams, MotorController, MotorParams,
    RotorEstimate,
};
use fluxkit_math::{
    ElectricalAngle,
    frame::Dq,
    inverse_clarke, inverse_park,
    units::{Amps, Duty, Henries, Hertz, Ohms, RadPerSec, Volts},
};

fn main() {
    let motor = MotorParams {
        pole_pairs: 7,
        phase_resistance_ohm: Ohms::new(0.12),
        d_inductance_h: Henries::new(0.000_03),
        q_inductance_h: Henries::new(0.000_03),
        flux_linkage_weber: None,
        max_phase_current: Amps::new(20.0),
        max_mech_speed: Some(RadPerSec::new(500.0)),
        torque_constant_nm_per_amp: None,
    };
    let inverter = InverterParams {
        pwm_frequency_hz: Hertz::new(20_000.0),
        deadtime_ns: 500,
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
        max_voltage_mag: Volts::new(12.0),
        id_ref_default: Amps::ZERO,
        max_id_target: Amps::new(10.0),
        max_iq_target: Amps::new(10.0),
    };

    let mut controller = MotorController::new(motor, inverter, config);
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
            rotor: RotorEstimate {
                electrical_angle: ElectricalAngle::new(angle),
                mechanical_velocity: RadPerSec::new(electrical_speed / motor.pole_pairs as f32),
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
