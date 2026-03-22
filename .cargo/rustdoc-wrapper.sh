#!/bin/sh
set -eu

script_dir=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
repo_root=$(CDPATH= cd -- "$script_dir/.." && pwd)
header_path="$repo_root/docs/katex-header.html"

exec rustdoc --html-in-header "$header_path" "$@"
