use std::{env, error::Error, fs};

use fluxkit_math::{AlphaBeta, Modulator, SinePwm, Svpwm, trig::sin_cos, units::Volts};
use plotters::prelude::*;

const TWO_PI: f32 = core::f32::consts::TAU;
const SAMPLES: u32 = 721;
const VBUS_VOLTS: f32 = 24.0;
const LIMIT_RATIO: f32 = 0.95;

#[derive(Clone, Copy)]
struct ModulationSample {
    theta_deg: f32,
    spwm: [f32; 3],
    svpwm: [f32; 3],
}

fn main() -> Result<(), Box<dyn Error>> {
    let output_dir = env::args()
        .nth(1)
        .unwrap_or_else(|| "target/plots".to_owned());
    fs::create_dir_all(&output_dir)?;

    let output_path = format!("{output_dir}/modulation_comparison.svg");
    let samples = modulation_samples();
    draw_modulation_comparison(&output_path, &samples)?;

    println!("wrote {output_path}");
    Ok(())
}

fn modulation_samples() -> Vec<ModulationSample> {
    let vbus = Volts::new(VBUS_VOLTS);
    let spwm = SinePwm;
    let svpwm_mod = Svpwm;
    let spwm_vref = LIMIT_RATIO * spwm.linear_limit(vbus).get();
    let svpwm_vref = LIMIT_RATIO * svpwm_mod.linear_limit(vbus).get();

    (0..SAMPLES)
        .map(|idx| {
            let theta = TWO_PI * idx as f32 / (SAMPLES - 1) as f32;
            let (s, c) = sin_cos(theta);
            let spwm_ab = AlphaBeta::new(spwm_vref * c, spwm_vref * s);
            let svpwm_ab = AlphaBeta::new(svpwm_vref * c, svpwm_vref * s);
            let sp = spwm.modulate(spwm_ab, vbus);
            let sv = svpwm_mod.modulate(svpwm_ab, vbus);

            ModulationSample {
                theta_deg: theta.to_degrees(),
                spwm: [sp.duty.a.get(), sp.duty.b.get(), sp.duty.c.get()],
                svpwm: [sv.duty.a.get(), sv.duty.b.get(), sv.duty.c.get()],
            }
        })
        .collect()
}

fn draw_modulation_comparison(
    output_path: &str,
    samples: &[ModulationSample],
) -> Result<(), Box<dyn Error>> {
    let root = SVGBackend::new(output_path, (640, 720)).into_drawing_area();
    root.fill(&WHITE)?;

    let areas = root.split_evenly((2, 1));
    draw_panel(
        &areas[0],
        samples,
        "SPWM duties at 95% of SPWM linear limit",
        |sample| sample.spwm,
    )?;
    draw_panel(
        &areas[1],
        samples,
        "SVPWM duties at 95% of SVPWM linear limit",
        |sample| sample.svpwm,
    )?;

    root.present()?;
    Ok(())
}

fn draw_panel<F>(
    area: &DrawingArea<SVGBackend<'_>, plotters::coord::Shift>,
    samples: &[ModulationSample],
    title: &str,
    selector: F,
) -> Result<(), Box<dyn Error>>
where
    F: Fn(&ModulationSample) -> [f32; 3],
{
    let mut chart = ChartBuilder::on(area)
        .caption(title, ("sans-serif", 24))
        .margin(16)
        .x_label_area_size(40)
        .y_label_area_size(60)
        .build_cartesian_2d(0.0_f32..360.0_f32, -0.05_f32..1.05_f32)?;

    chart
        .configure_mesh()
        .disable_mesh()
        .x_desc("electrical angle (deg)")
        .y_desc("duty")
        .draw()?;

    let colors = [RED, RGBColor(237, 174, 73), RGBColor(0, 121, 140)];
    let labels = ["phase a", "phase b", "phase c"];

    for (phase_idx, (color, label)) in colors.into_iter().zip(labels).enumerate() {
        chart
            .draw_series(LineSeries::new(
                samples.iter().map(|sample| {
                    let series = selector(sample);
                    (sample.theta_deg, series[phase_idx])
                }),
                &color,
            ))?
            .label(label)
            .legend(move |(x, y)| PathElement::new(vec![(x, y), (x + 20, y)], color));
    }

    chart
        .configure_series_labels()
        .background_style(WHITE)
        .border_style(BLACK)
        .draw()?;

    Ok(())
}
