#!/usr/bin/env bash
# Wrapper so the module can be driven with the repo-standard `./main.sh <stage>` form.
set -euo pipefail
cd "$(dirname "$0")"
exec python3 main.py "$@"
