#!/usr/bin/env python3

import csv
import os
import sys


WIDTH = 1200
HEIGHT_PER_PANEL = 320
MARGIN_LEFT = 70
MARGIN_RIGHT = 24
MARGIN_TOP = 40
MARGIN_BOTTOM = 42
PLOT_HEIGHT = 210
PLOT_WIDTH = WIDTH - MARGIN_LEFT - MARGIN_RIGHT
COLORS = ["#d1495b", "#edae49", "#00798c"]
LABELS = ["phase a", "phase b", "phase c"]


def load_rows(path):
    grouped = {}
    with open(path, newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            case = row["case"]
            grouped.setdefault(case, []).append(
                {
                    "theta_deg": float(row["theta_deg"]),
                    "vref": float(row["vref"]),
                    "vbus": float(row["vbus"]),
                    "spwm_limit": float(row["spwm_limit"]),
                    "svpwm_limit": float(row["svpwm_limit"]),
                    "spwm": [
                        float(row["spwm_a"]),
                        float(row["spwm_b"]),
                        float(row["spwm_c"]),
                    ],
                    "spwm_saturated": row["spwm_saturated"].strip().lower() == "true",
                    "svpwm": [
                        float(row["svpwm_a"]),
                        float(row["svpwm_b"]),
                        float(row["svpwm_c"]),
                    ],
                    "saturated": row["svpwm_saturated"].strip().lower() == "true",
                    "mod_index": float(row["svpwm_modulation_index"]),
                }
            )
    return grouped


def sx(theta_deg):
    return MARGIN_LEFT + (theta_deg / 360.0) * PLOT_WIDTH


def sy(value, panel_top, value_min=-0.05, value_max=1.05):
    norm = (value - value_min) / (value_max - value_min)
    return panel_top + PLOT_HEIGHT - norm * PLOT_HEIGHT


def build_polyline(rows, selector, panel_top):
    points = []
    for row in rows:
        points.append(f"{sx(row['theta_deg']):.2f},{sy(selector(row), panel_top):.2f}")
    return " ".join(points)


def svg_header(height):
    return [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{WIDTH}" height="{height}" viewBox="0 0 {WIDTH} {height}">',
        '<rect width="100%" height="100%" fill="#fffdf8"/>',
        '<style>',
        'text { font-family: Menlo, Monaco, monospace; fill: #202124; }',
        '.title { font-size: 18px; font-weight: 700; }',
        '.axis { font-size: 12px; }',
        '.meta { font-size: 11px; }',
        '.grid { stroke: #d9d9d9; stroke-width: 1; }',
        '.frame { stroke: #3b3b3b; stroke-width: 1.4; fill: none; }',
        '</style>',
    ]


def draw_axes(parts, panel_top):
    left = MARGIN_LEFT
    right = MARGIN_LEFT + PLOT_WIDTH
    bottom = panel_top + PLOT_HEIGHT

    for value in [0.0, 0.25, 0.5, 0.75, 1.0]:
        y = sy(value, panel_top)
        parts.append(f'<line class="grid" x1="{left}" y1="{y:.2f}" x2="{right}" y2="{y:.2f}"/>')
        parts.append(f'<text class="axis" x="{left - 10}" y="{y + 4:.2f}" text-anchor="end">{value:.2f}</text>')

    for angle in range(0, 361, 60):
        x = sx(float(angle))
        parts.append(f'<line class="grid" x1="{x:.2f}" y1="{panel_top}" x2="{x:.2f}" y2="{bottom}"/>')
        parts.append(f'<text class="axis" x="{x:.2f}" y="{bottom + 18}" text-anchor="middle">{angle}</text>')

    parts.append(f'<rect class="frame" x="{left}" y="{panel_top}" width="{PLOT_WIDTH}" height="{PLOT_HEIGHT}"/>')
    parts.append(
        f'<text class="axis" x="{left - 48}" y="{panel_top + PLOT_HEIGHT / 2:.2f}" transform="rotate(-90 {left - 48},{panel_top + PLOT_HEIGHT / 2:.2f})" text-anchor="middle">duty</text>'
    )
    parts.append(f'<text class="axis" x="{left + PLOT_WIDTH / 2:.2f}" y="{bottom + 34}" text-anchor="middle">electrical angle (deg)</text>')


def draw_legend(parts, panel_top):
    x = WIDTH - 200
    y = panel_top - 8
    for idx, (color, label) in enumerate(zip(COLORS, LABELS)):
        line_y = y + idx * 18
        parts.append(f'<line x1="{x}" y1="{line_y}" x2="{x + 20}" y2="{line_y}" stroke="{color}" stroke-width="3"/>')
        parts.append(f'<text class="axis" x="{x + 28}" y="{line_y + 4}">{label}</text>')


def plot_family(grouped, family, output_path):
    cases = list(grouped.keys())
    height = HEIGHT_PER_PANEL * len(cases)
    parts = svg_header(height)
    parts.append(f'<text class="title" x="{MARGIN_LEFT}" y="24">{family.upper()} duty waveforms</text>')

    for idx, case in enumerate(cases):
        panel_top = MARGIN_TOP + idx * HEIGHT_PER_PANEL
        rows = grouped[case]
        vref = rows[0]["vref"]
        vbus = rows[0]["vbus"]
        spwm_limit = rows[0]["spwm_limit"]
        svpwm_limit = rows[0]["svpwm_limit"]
        if family == "spwm":
            sat_count = sum(1 for row in rows if row["spwm_saturated"])
            limit = spwm_limit
        else:
            sat_count = sum(1 for row in rows if row["saturated"])
            limit = svpwm_limit
        max_mod = max(row["mod_index"] for row in rows)

        draw_axes(parts, panel_top)
        draw_legend(parts, panel_top)
        parts.append(
            f'<text class="title" x="{MARGIN_LEFT}" y="{panel_top - 12}">{family.upper()} | case={case} | vref={vref:.2f} V | ratio-to-limit={vref / limit:.3f}</text>'
        )
        parts.append(
            f'<text class="meta" x="{WIDTH - 24}" y="{panel_top + PLOT_HEIGHT + 34}" text-anchor="end">vbus={vbus:.2f} V, spwm_limit={spwm_limit:.3f} V, svpwm_limit={svpwm_limit:.3f} V, max svpwm modulation index={max_mod:.3f}, saturated samples={sat_count}</text>'
        )

        for phase_idx, color in enumerate(COLORS):
            parts.append(
                f'<polyline fill="none" stroke="{color}" stroke-width="2.2" points="{build_polyline(rows, lambda row, i=phase_idx: row[family][i], panel_top)}"/>'
            )

    parts.append("</svg>")
    with open(output_path, "w", encoding="utf-8") as handle:
        handle.write("\n".join(parts))


def plot_phase_a_overlay(grouped, output_path):
    cases = list(grouped.keys())
    height = HEIGHT_PER_PANEL * len(cases)
    parts = svg_header(height)
    parts.append(f'<text class="title" x="{MARGIN_LEFT}" y="24">Phase-A SPWM vs SVPWM</text>')

    for idx, case in enumerate(cases):
        panel_top = MARGIN_TOP + idx * HEIGHT_PER_PANEL
        rows = grouped[case]
        vref = rows[0]["vref"]
        spwm_limit = rows[0]["spwm_limit"]
        svpwm_limit = rows[0]["svpwm_limit"]

        draw_axes(parts, panel_top)
        parts.append(
            f'<text class="title" x="{MARGIN_LEFT}" y="{panel_top - 12}">phase a overlay | case={case} | vref/spwm_limit={vref / spwm_limit:.3f} | vref/svpwm_limit={vref / svpwm_limit:.3f}</text>'
        )

        overlays = [
            ("spwm a", "#444444", lambda row: row["spwm"][0]),
            ("svpwm a", "#00798c", lambda row: row["svpwm"][0]),
            ("svpwm - spwm", "#d1495b", lambda row: row["svpwm"][0] - row["spwm"][0]),
        ]

        legend_x = WIDTH - 240
        legend_y = panel_top - 8
        for legend_idx, (label, color, _) in enumerate(overlays):
            y = legend_y + legend_idx * 18
            parts.append(f'<line x1="{legend_x}" y1="{y}" x2="{legend_x + 20}" y2="{y}" stroke="{color}" stroke-width="3"/>')
            parts.append(f'<text class="axis" x="{legend_x + 28}" y="{y + 4}">{label}</text>')

        for label, color, selector in overlays:
            parts.append(
                f'<polyline fill="none" stroke="{color}" stroke-width="2.2" points="{build_polyline(rows, selector, panel_top)}"/>'
            )

    parts.append("</svg>")
    with open(output_path, "w", encoding="utf-8") as handle:
        handle.write("\n".join(parts))


def main():
    csv_path = sys.argv[1] if len(sys.argv) > 1 else "target/plots/modulation.csv"
    output_dir = sys.argv[2] if len(sys.argv) > 2 else "target/plots"
    os.makedirs(output_dir, exist_ok=True)

    grouped = load_rows(csv_path)
    plot_family(grouped, "spwm", os.path.join(output_dir, "spwm_duties.svg"))
    plot_family(grouped, "svpwm", os.path.join(output_dir, "svpwm_duties.svg"))
    plot_phase_a_overlay(grouped, os.path.join(output_dir, "spwm_vs_svpwm_phase_a.svg"))

    print(f"wrote plots to {output_dir}")


if __name__ == "__main__":
    main()
