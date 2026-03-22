#!/usr/bin/env bash

set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$repo_root"

plots_dir="${1:-docs/plots}"

mkdir -p "$plots_dir"

cache_home="${XDG_CACHE_HOME:-/tmp/fluxkit-nix-cache}"
runner=(nix develop -c)

echo "Generating documentation plots into $plots_dir"

XDG_CACHE_HOME="$cache_home" "${runner[@]}" cargo run -p fluxkit-pmsm-sim --example closed_loop_current -- "$plots_dir/closed_loop_current.svg"
XDG_CACHE_HOME="$cache_home" "${runner[@]}" cargo run -p fluxkit-pmsm-sim --example closed_loop_position -- "$plots_dir/closed_loop_position.svg"
XDG_CACHE_HOME="$cache_home" "${runner[@]}" cargo run -p fluxkit-pmsm-sim --example closed_loop_actuator_compensation -- "$plots_dir/closed_loop_actuator_compensation.svg"
XDG_CACHE_HOME="$cache_home" "${runner[@]}" cargo run -p fluxkit_math --example plot_modulation -- "$plots_dir"

echo "Done."
