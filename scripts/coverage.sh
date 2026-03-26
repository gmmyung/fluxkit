#!/usr/bin/env bash

set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$repo_root"

cache_home="${XDG_CACHE_HOME:-/tmp/fluxkit-nix-cache}"
runner=(nix develop -c)

summary_only=false
html=true
workspace=true
extra_args=()

while [[ $# -gt 0 ]]; do
  case "$1" in
    --summary-only)
      summary_only=true
      html=false
      shift
      ;;
    --no-html)
      html=false
      shift
      ;;
    --package|-p)
      workspace=false
      extra_args+=("$1" "$2")
      shift 2
      ;;
    --)
      shift
      extra_args+=("$@")
      break
      ;;
    *)
      extra_args+=("$1")
      shift
      ;;
  esac
done

cmd=(cargo llvm-cov)
if [[ "$workspace" == true ]]; then
  cmd+=(--workspace)
fi
if [[ "$summary_only" == true ]]; then
  cmd+=(--summary-only)
elif [[ "$html" == true ]]; then
  cmd+=(--html)
fi
cmd+=("${extra_args[@]}")

echo "Running coverage: ${cmd[*]}"
XDG_CACHE_HOME="$cache_home" "${runner[@]}" "${cmd[@]}"

if [[ "$html" == true && "$summary_only" == false ]]; then
  echo "HTML report: target/llvm-cov/html/index.html"
fi
