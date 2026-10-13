#!/usr/bin/env bash
set -euo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
exec .venv/bin/lerobot-teleoperate \
  --config_path=configs/teleoperate.yaml \
  --robot.enable_motors=true \
  "$@"
