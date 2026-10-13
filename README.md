# PiperLerobotToolkit

Direct joint-position teleoperation between two AgileX Piper-X arms using LeRobot.
One arm is a passive, hand-guided leader; the other follows its six joint angles
and gripper opening. There is no Cartesian mapping, FK, IK, or robot-model dependency.

## Install

Requires Linux SocketCAN, two USB-CAN adapters, Python 3.12, and `uv`.

```bash
git clone https://github.com/YuxuanJIANG123/PiperLerobotToolkit.git
cd PiperLerobotToolkit
bash scripts/setup.sh
source .venv/bin/activate
```

Setup pins LeRobot to `8c920c4270460851cedd2737657584586d3dc66f` and installs
`piper-sdk==0.6.2`. It applies local patches for stale-leader pause handling,
recording the bounded action returned by the adapter, and timestamped dataset paths.
The vendor checkout, virtual environment, datasets, and logs are not versioned.

## CAN setup

Identify which USB-CAN adapter connects to each arm before assigning roles:

```bash
ip -details link show type can
```

For the current installation, `can0` at USB `1-4` was the follower and `can1` at
USB `5-1` was the leader. Enumeration can change after reconnecting. With both
interfaces down, and only after checking that mapping:

```bash
sudo ip link set can0 name piper_follower
sudo ip link set can1 name piper_leader
sudo ip link set piper_follower up type can bitrate 1000000
sudo ip link set piper_leader up type can bitrate 1000000
```

These names are not persistent across reboot/reconnection. Alternatively override
`--robot.can_name=...` and `--teleop.can_name=...` in the launch command.

Inspect without transmitting commands:

```bash
piper-inspect --channel piper_leader --source feedback --seconds 3
piper-inspect --channel piper_follower --source feedback --seconds 3
python scripts/inspect_can.py
```

## Teleoperate

Prepare the leader for hand guiding using its supported hardware procedure. This
adapter only receives leader feedback: it does not enable teaching, disable torque,
change arm roles, or provide gravity compensation. The follower must have teaching
recording/playback stopped. Start near matching joint poses, clear the follower
workspace, and keep the physical stop accessible.

```bash
bash teleopeation.sh
# Equivalent:
lerobot-teleoperate --config_path=configs/teleoperate.yaml --robot.enable_motors=true
```

The launcher enables follower motors. It does not home the arm. Stop an existing
controller before launching another. Ctrl+C closes communication; it does not
disable torque or act as an emergency stop.

| Setting | Teleoperation | Recording |
| --- | --- | --- |
| Loop rate | 15 Hz | 30 Hz |
| Controller motion speed | 100% | 50% |
| Maximum joint target offset from feedback | 0.03 rad | 0.03 rad |
| Maximum gripper target offset from feedback | 0.01 m | 0.003 m |
| Leader source | Measured feedback | Measured feedback |
| Leader/follower feedback timeout | 0.5 s | 0.5 s |
| High-follow mode | Off | Off |
| Motion diagnostics | On | On |

These target-offset bounds are not velocity guarantees. Controller speed 100%
does not guarantee matching the leader's speed; 15 Hz updates arrive about every
67 ms. Grip effort remains 1000 in SDK units. Start with small movements after
changing settings. CLI overrides are accepted, for example:

```bash
bash teleopeation.sh --robot.motion_speed=50 --robot.max_joint_step_rad=0.012
```

For a leader explicitly configured to emit teaching control targets instead of
feedback, append `--teleop.source=control --teleop.feedback_timeout_s=3`.

Joint positions are radians (`joint_1.pos` through `joint_6.pos`); `gripper.pos`
is opening in metres. No implicit sign flips or offsets are applied. Supplied
follower limits are from the existing installation: J1 ±150°, J2 0–180°,
J3 −170–0°, J4/J5 ±89°, J6 ±170°, gripper 0–70 mm. Verify these if replacing the
follower. Do not reset hardware zeros or widen limits to bypass a startup failure.

On missing/stale leader frames, following pauses at a captured follower pose.
All four leader frames must refresh before resuming. CAN faults and stale follower
feedback abort operation. Recorded actions contain the adapter's bounded,
quantized target, not a wire-level acknowledgement or guaranteed achieved position.

## Follower stuck in teaching mode

`ctrl_mode=2, teach_status=1` means teaching recording remains active. Stop it on
the follower using its teaching button and verify the teaching indicator is off.
The recovery helper refuses active teaching/playback. Its `--prepared` flag does
not stop teaching or check physical preparation for you.

If ordinary startup still cannot enter CAN control, stop teleoperation, support
the follower at physical zero with teaching off, then run:

```bash
python scripts/switch_follower_mode.py                       # Read-only preflight
python scripts/switch_follower_mode.py --execute --prepared # Standby -> CAN
```

The helper seeds current joint targets and waits for mode acknowledgements. It
sends no motor-enable, calibration, zero-reset, or arm-role commands.

## Record demonstrations

`configs/record.yaml` uses the existing wrist and third-person RealSense cameras.
Update their serial numbers for your installation; preview with
`python scripts/check_setup.py` (no motor commands).

```bash
lerobot-record --config_path=configs/record.yaml --robot.enable_motors=true
```

Defaults: 10 episodes, 30 seconds each, 10-second reset windows, 30 FPS, no Hub
upload. New runs create matching timestamped IDs/folders beneath
`local/piper_x_pair_pick_TIMESTAMP` and `data/piper_x_pair_pick_TIMESTAMP`.
Data uses LeRobot v3 format. Leader-silence holds do not pause the episode clock.

- `n` / Right arrow: end capture early, proceed through reset and save.
- `r` / Left arrow: discard the unsaved episode and repeat it.
- `q` / Esc: save the current episode and exit normally.

Use `q` rather than Ctrl+C when saving the current episode matters. To resume:

```bash
lerobot-record --config_path=configs/record.yaml --robot.enable_motors=true \
  --resume=true --dataset.root=data/piper_x_pair_pick_TIMESTAMP \
  --dataset.repo_id=local/piper_x_pair_pick_TIMESTAMP
```

Use the exact existing path and the same camera, unit, and control configuration.
Resume preserves the supplied name and adds the configured number of episodes.

## Additional tools

- `scripts/audit_arms.py`: firmware queries and frame capture.
- `scripts/query_limits.py`: query stored follower limits.
- `scripts/inspect_motor_status.py`: receive motor enable/fault status.
- `scripts/probe_wrist.py`: bounded single-joint diagnostic; dry-run by default.
- `configs/rollout.yaml`: compatible LeRobot policy rollout; requires a supplied
  checkpoint producing absolute joint radians and gripper metres. No policy has
  been validated by this project on the current pair.

Optional `--robot.reset_to_zero_on_connect=true` performs a bounded joint-space
return to existing zero angles while retaining gripper opening. It requires
motor enable and a clear return path; it does not reset encoder calibration.

## Development and verification

```bash
pytest -q
ruff check src tests scripts
```

Tests use fake hardware and cover CAN decoding/freshness, position limits,
startup/mode recovery, bounded commands, optional return to zero, leader pause
and resume, the actual LeRobot loops, and v3 dataset writes. They do not establish
physical tracking accuracy or collision safety. This adapter has no collision
avoidance or force-feedback controller.
