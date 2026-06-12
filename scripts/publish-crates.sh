#!/usr/bin/env bash

set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$repo_root"

publish_order=(
  "fluxkit_math"
  "fluxkit-hal"
  "fluxkit-core"
  "fluxkit-pmsm-sim"
  "fluxkit"
)

dry_run=false
allow_dirty=false
from_package=""
max_attempts=20
retry_delay_seconds=15

usage() {
  cat <<'EOF'
Usage: scripts/publish-crates.sh [options]

Publishes the Fluxkit crates to crates.io in dependency order.

Options:
  --dry-run           Use `cargo publish --dry-run`
  --allow-dirty       Pass `--allow-dirty` through to cargo publish
  --from <package>    Start publishing from the named package in the publish order
  --max-attempts <n>  Max attempts per package when waiting for crates.io index propagation
  --retry-delay <s>   Delay between retries in seconds
  -h, --help          Show this help
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --dry-run)
      dry_run=true
      shift
      ;;
    --allow-dirty)
      allow_dirty=true
      shift
      ;;
    --from)
      from_package="${2:-}"
      shift 2
      ;;
    --max-attempts)
      max_attempts="${2:-}"
      shift 2
      ;;
    --retry-delay)
      retry_delay_seconds="${2:-}"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown argument: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

cargo_args=()
if [[ "$dry_run" == true ]]; then
  cargo_args+=(--dry-run)
fi
if [[ "$allow_dirty" == true ]]; then
  cargo_args+=(--allow-dirty)
fi

start_index=0
if [[ -n "$from_package" ]]; then
  found=false
  for i in "${!publish_order[@]}"; do
    if [[ "${publish_order[$i]}" == "$from_package" ]]; then
      start_index="$i"
      found=true
      break
    fi
  done
  if [[ "$found" == false ]]; then
    echo "Unknown package for --from: $from_package" >&2
    exit 2
  fi
fi

run_publish() {
  local package="$1"
  local attempt output status

  for ((attempt = 1; attempt <= max_attempts; attempt++)); do
    echo "Publishing $package (attempt $attempt/$max_attempts)"

    set +e
    output="$(cargo publish -p "$package" "${cargo_args[@]}" 2>&1)"
    status=$?
    set -e

    printf '%s\n' "$output"

    if [[ $status -eq 0 ]]; then
      return 0
    fi

    if [[ "$dry_run" == true ]]; then
      return $status
    fi

    if grep -q "already uploaded" <<<"$output"; then
      echo "Package $package is already published; continuing."
      return 0
    fi

    if grep -q "no matching package named" <<<"$output"; then
      if (( attempt < max_attempts )); then
        echo "Waiting ${retry_delay_seconds}s for crates.io index propagation before retrying $package"
        sleep "$retry_delay_seconds"
        continue
      fi
    fi

    return $status
  done
}

for ((i = start_index; i < ${#publish_order[@]}; i++)); do
  run_publish "${publish_order[$i]}"
done
