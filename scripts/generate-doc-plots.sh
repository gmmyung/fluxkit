#!/usr/bin/env bash

set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$repo_root"

plots_dir="${1:-docs/plots}"
core_plots_dir="crates/fluxkit_core/docs/plots"
math_plots_dir="crates/fluxkit_math/docs/plots"

mkdir -p "$plots_dir"
mkdir -p "$core_plots_dir" "$math_plots_dir"

cache_home="${XDG_CACHE_HOME:-/tmp/fluxkit-nix-cache}"
runner=(nix develop -c)

echo "Generating documentation plots into $plots_dir"

XDG_CACHE_HOME="$cache_home" "${runner[@]}" cargo run -p fluxkit-pmsm-sim --example closed_loop_current -- "$plots_dir/closed_loop_current.svg"
XDG_CACHE_HOME="$cache_home" "${runner[@]}" cargo run -p fluxkit-pmsm-sim --example closed_loop_position -- "$plots_dir/closed_loop_position.svg"
XDG_CACHE_HOME="$cache_home" "${runner[@]}" cargo run -p fluxkit-pmsm-sim --example closed_loop_torque_command -- "$plots_dir/closed_loop_torque_command.svg"
XDG_CACHE_HOME="$cache_home" "${runner[@]}" cargo run -p fluxkit-pmsm-sim --example closed_loop_velocity_command -- "$plots_dir/closed_loop_velocity_command.svg"
XDG_CACHE_HOME="$cache_home" "${runner[@]}" cargo run -p fluxkit_math --example plot_modulation -- "$plots_dir"

cp "$plots_dir/closed_loop_current.svg" "$core_plots_dir/closed_loop_current.svg"
cp "$plots_dir/closed_loop_torque_command.svg" "$core_plots_dir/closed_loop_torque_command.svg"
cp "$plots_dir/closed_loop_velocity_command.svg" "$core_plots_dir/closed_loop_velocity_command.svg"
cp "$plots_dir/modulation_comparison.svg" "$math_plots_dir/modulation_comparison.svg"

echo "Done."
