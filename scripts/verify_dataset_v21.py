#!/usr/bin/env python3
"""Perform inexpensive structural checks on a local LeRobot v2.1 dataset."""

import argparse
import json
from pathlib import Path


def verify(root: Path) -> None:
    info_path = root / "meta" / "info.json"
    if not info_path.is_file():
        raise SystemExit(f"Missing {info_path}")
    info = json.loads(info_path.read_text(encoding="utf-8"))
    if info.get("codebase_version") != "v2.1":
        raise SystemExit(f"Expected v2.1, found {info.get('codebase_version')!r}")
    required = {"observation.state", "action", "observation.images.wrist", "observation.images.third_person"}
    missing = required - info.get("features", {}).keys()
    if missing:
        raise SystemExit(f"Missing required features: {sorted(missing)}")
    for key in ("observation.state", "action"):
        if info["features"][key].get("shape") != [7]:
            raise SystemExit(f"{key} must have shape [7]")
    print(f"OK: {root} is a two-camera Piper LeRobot Dataset v2.1")
    print(f"episodes={info.get('total_episodes')} frames={info.get('total_frames')} fps={info.get('fps')}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("root", type=Path)
    verify(parser.parse_args().root)
