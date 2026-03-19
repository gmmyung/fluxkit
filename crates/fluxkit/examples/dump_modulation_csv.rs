use std::env;
use std::fs::File;
use std::io::{BufWriter, Write};
use std::process::ExitCode;

use fluxkit::frame::AlphaBeta;
use fluxkit::modulation::{Modulator, SinePwm, Svpwm, spwm_linear_limit, svpwm_linear_limit};
use fluxkit::trig::sin_cos;
use fluxkit::units::Volts;

const TWO_PI: f32 = core::f32::consts::TAU;

fn main() -> ExitCode {
    let output_path = env::args()
        .nth(1)
        .unwrap_or_else(|| "target/plots/modulation.csv".to_owned());

    let file = match File::create(&output_path) {
        Ok(file) => file,
        Err(err) => {
            eprintln!("failed to create {output_path}: {err}");
            return ExitCode::FAILURE;
        }
    };

    let mut writer = BufWriter::new(file);
    let vbus = Volts::new(24.0);
    let spwm_limit = spwm_linear_limit(vbus);
    let svpwm_limit = svpwm_linear_limit(vbus);
    let sp_mod = SinePwm;
    let sv_mod = Svpwm;
    let cases = [
        ("safe_both", 0.90 * spwm_limit.get()),
        ("spwm_clip_svpwm_linear", 1.02 * spwm_limit.get()),
        ("near_svpwm_limit", 0.99 * svpwm_limit.get()),
        ("over_svpwm_limit", 1.02 * svpwm_limit.get()),
    ];
    let samples = 721_u32;

    writeln!(
        writer,
        "case,theta_rad,theta_deg,vbus,spwm_limit,svpwm_limit,vref,spwm_a,spwm_b,spwm_c,spwm_saturated,svpwm_a,svpwm_b,svpwm_c,svpwm_saturated,svpwm_modulation_index"
    )
    .expect("csv header write failed");

    for (case_name, vref) in cases {
        for idx in 0..samples {
            let theta = TWO_PI * (idx as f32) / ((samples - 1) as f32);
            let (s, c) = sin_cos(theta);
            let v_ab = AlphaBeta::new(vref * c, vref * s);
            let sp = sp_mod.modulate(v_ab, vbus);
            let sv_common = sv_mod.modulate(v_ab, vbus);
            let sv = fluxkit::modulation::svpwm(v_ab, vbus.get());

            writeln!(
                writer,
                "{case_name},{theta:.8},{:.4},{:.4},{:.4},{:.4},{vref:.4},{:.8},{:.8},{:.8},{},{:.8},{:.8},{:.8},{}, {:.8}",
                theta.to_degrees(),
                vbus.get(),
                spwm_limit.get(),
                svpwm_limit.get(),
                sp.duty.a.get(),
                sp.duty.b.get(),
                sp.duty.c.get(),
                sp.saturated,
                sv_common.duty.a.get(),
                sv_common.duty.b.get(),
                sv_common.duty.c.get(),
                sv_common.saturated,
                sv.modulation_index
            )
            .expect("csv row write failed");
        }
    }

    writer.flush().expect("csv flush failed");
    println!("wrote {output_path}");
    ExitCode::SUCCESS
}
