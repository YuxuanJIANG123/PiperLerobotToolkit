#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
export UV_CACHE_DIR="${UV_CACHE_DIR:-/tmp/lerobot-piper-uv-cache}"
revision=8c920c4270460851cedd2737657584586d3dc66f
if [[ ! -d vendor/lerobot/.git ]]; then
  git clone https://github.com/huggingface/lerobot.git vendor/lerobot
  git -C vendor/lerobot checkout "$revision"
fi
if [[ "$(git -C vendor/lerobot rev-parse HEAD)" != "$revision" ]]; then
  echo "Unexpected LeRobot revision; review compatibility before installing." >&2
  exit 1
fi
if [[ ! -x .venv/bin/python ]]; then
  uv venv --python 3.12 .venv
fi
python3 scripts/apply_recording_patch.py
python3 scripts/apply_leader_pause_patch.py
python3 scripts/apply_dataset_stamp_patch.py
uv pip install --python .venv/bin/python -e 'vendor/lerobot[core_scripts,intelrealsense]' -e . pytest ruff
