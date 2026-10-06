# Piper-H leader / Piper-X follower with LeRobot

This workspace connects a user-identified Piper-H teaching leader to a Piper-X
follower through LeRobot's robot and teleoperator interfaces. **Follower motion has
been confirmed by the user and recorded joint feedback.** The current adapter
copies joint angles directly; matching the two arms' physical poses or gripper
trajectories has not been validated. H-to-X kinematic retargeting is not implemented.

## Setup

Run commands from this project directory:

```bash
cd /path/to/PiperLerobotToolkit/piper_h_x
bash scripts/setup.sh                    # First installation or reinstall
source .venv/bin/activate                # Every new terminal
```

The environment uses Python 3.12, `piper-sdk==0.6.2`, and LeRobot source pinned to
`8c920c4270460851cedd2737657584586d3dc66f` under `vendor/lerobot`.
Setup installs the adapter and reapplies the three local LeRobot patches below.

| Device | Connection | Verified information |
| --- | --- | --- |
| Piper-H leader (user identification) | `piper_leader`, 1 Mbps | Firmware `S-V1.7-3`; control targets can stop arriving while stationary |
| Piper-X follower | `piper_follower`, 1 Mbps | Firmware `S-V1.9-0`, identifier `Piper_X_MC`; feedback approximately 200 Hz |
| Wrist RealSense D435i | Serial `346122071508` | 640×480 RGB at configured 30 FPS |
| Third-person RealSense D435i | Serial `405622074798` | 640×480 RGB at configured 30 FPS |

Firmware results are saved in `outputs/read_only_check/arm_audit.json`.
The `H-V` prefix in firmware replies is a hardware-version field, not proof of
arm model. Camera names use the prior deployment's mapping: inspect the previews
and aim the third-person camera at the manipulation workspace before recording.

## Changes made during troubleshooting

| Change | Current behavior / implementation |
| --- | --- |
| Follower control mode | `high_follow=false` is now the default: planned position/velocity joint control. This setting produced observed movement; the earlier `true` setting did not produce the expected following in the user's initial test. |
| Leader silence | Teleoperation and recording pause at the follower's measured pose after target freshness expires, rather than crashing or continuing toward a stale leader target. All four leader target frames must refresh before resuming. |
| Strict fault handling | CAN transport errors and stale follower feedback still stop the operation. Silence cannot distinguish a stationary leader from an unplugged one; both pause following. |
| Receive timestamps | Frame freshness accounts for SocketCAN receipt timestamps, so buffered old messages are not marked fresh merely because Python just processed them. |
| Diagnostics | `debug_motion=true` logs measured positions, leader/hold targets, limited SDK targets, motor-enable flags, and follower feedback timing once per second. `piper-inspect` includes decoded arm status and timing. Startup mode errors include SDK status values. |
| Startup limits | Initial positions are checked before motor enable; supplied YAML files use limits queried from this follower. No limits were widened and no hardware zeros were reset to bypass errors. |
| Recorded actions | The recorder stores the bounded, quantized action returned by this adapter's `send_action`, instead of the unrestricted leader request. The SDK may also apply its own limits; this is not a wire-level acknowledgement. |
| Dataset names | New recordings timestamp both the dataset ID and explicit folder, including microseconds. Existing datasets are preserved; resume uses the exact supplied path without stamping. |
| Diagnostic scripts | Firmware audit and explicit standby→CAN transition remain available. The unvalidated standalone motion probe was removed during cleanup. |

The LeRobot modifications are reproducible using:

```bash
python scripts/apply_recording_patch.py
python scripts/apply_leader_pause_patch.py
python scripts/apply_dataset_stamp_patch.py
```

These are applied by `scripts/setup.sh`. Installing the adapter against an
unpatched LeRobot checkout does not provide the pause hook or these recording fixes.
The timestamp patch is in the shared dataset config, so it also applies to other
creation paths that call `stamp_repo_id()` with stamping enabled.

## Inspect without commanding motion

```bash
python scripts/inspect_can.py
piper-inspect --channel piper_follower --source feedback
piper-inspect --channel piper_leader --source control --seconds 5
python scripts/check_setup.py
```

These commands receive CAN traffic only. The setup check also opens both cameras,
saves `wrist.png`, `third_person.png`, and `report.json` under
`outputs/read_only_check/`, and forces motor enable off.

These two queries transmit read requests, but no motion or setting commands:

```bash
python scripts/audit_arms.py             # Firmware query and eight-second capture
python scripts/query_limits.py           # Stored follower joint limits
```

The leader adapter never enables, homes, reconfigures, or sends force feedback to
the leader. During connection, gently move the supported leader and operate its
gripper so all `0x155`, `0x156`, `0x157`, and `0x159` targets arrive within 10 seconds.
It cannot reuse targets from a previous process. Follower feedback expires after
0.5 seconds; leader targets default to a 3-second freshness window, with a recent
`0x151` able to retain previously received targets.

`piper-inspect` remains strict: a stationary leader can report stale frames even
though the patched teleoperation loop would pause. Also, the installed SDK sets
SocketCAN `local_loopback=False`: a second process inspecting follower *control*
frames cannot infer that the SDK sent nothing just because those frames are absent.

## Teleoperate

Clear the follower's workspace, keep the physical stop accessible, and run only
one process that commands the follower at a time. Start with small leader movements.

The most recent settings exercised in the user's teleoperation logs are:

```bash
lerobot-teleoperate \
  --config_path=configs/teleoperate.yaml \
  --robot.enable_motors=true \
  --robot.high_follow=false \
  --robot.motion_speed=20 \
  --robot.max_joint_step_rad=0.012 \
  --robot.debug_motion=true
```

Move the supported leader and gripper gently during startup. Press **Ctrl+C** to
end teleoperation. **Exiting closes communication but does not disable motor torque
or home the arm.** It is not an emergency stop.

| Setting | Default | Latest exercised override | Meaning |
| --- | --- | --- | --- |
| `high_follow` | `false` | `false` | Planned position/velocity mode; keep this for the current setup |
| `motion_speed` | `10` | `20` | Controller speed percentage, not degrees per second |
| `max_joint_step_rad` | `0.003` | `0.012` | Maximum target offset from the latest measured angle: about 0.172° vs 0.688° |
| `max_gripper_step_m` | `0.001` | unchanged | Maximum gripper target offset from measured opening: 1 mm |
| `fps` | `30` in YAML | unchanged | Application loop frequency |

The faster values above are **CLI overrides**, not new YAML defaults. Step limits
are position-error bounds, not guaranteed velocity or collision limits. Raising
them allows stronger catch-up; it does not guarantee a particular speed.

To capture diagnostics, append `> /tmp/piper-teleop.log 2>&1` to the command, then
inspect the log after stopping:

```bash
rg 'Motion diagnostic|pausing|resuming|Error' /tmp/piper-teleop.log
```

The first six logged numbers are degrees; the seventh is gripper millimetres.
`max_receive_delay_ms` measures kernel-to-reader delay; `max_frame_age_ms` measures
age of the relevant feedback. During a pause, the field labelled `leader` contains
the latched follower hold target, not a new leader measurement.

## Record demonstrations

To record using the same motion settings as the command above:

```bash
lerobot-record \
  --config_path=configs/record.yaml \
  --robot.enable_motors=true \
  --robot.high_follow=false \
  --robot.motion_speed=20 \
  --robot.max_joint_step_rad=0.012
```

Configuration: 30 FPS, 10 episodes, up to 30 seconds per episode, and 10 seconds
between episodes to reset the scene. Both cameras are recorded. Hub upload is off.
Leader-silence pauses do not pause the episode clock; hold actions can be recorded.

Each new run gets a matching local dataset ID and directory, for example:

```text
repo_id: local/piper_x_pick_20261006_123505_139132
folder:  data/piper_x_pick_20261006_123505_139132
```

`configs/record.yaml` sets `dataset.no_stamp: false`. There is no need to delete
an old folder or edit the timestamp before starting a new recording.
The timestamp uses local system time. Setting `--dataset.no_stamp=true` opts out
and requires you to supply an unused folder for a new recording.

### End, save, discard, or quit

Keep the recording terminal focused. Letter shortcuts are convenient with the
terminal keyboard backend used on this machine.

| Key | Action |
| --- | --- |
| **`n`** or **Right arrow** | End current capture early; enter the reset phase, then save the episode |
| **`r`** or **Left arrow** | Discard the unsaved episode and repeat it |
| **`q`** or **Esc** | End capture, save the current episode, and exit normally |

Episodes also end automatically at the configured duration. During the reset
phase, `r` can still discard the just-captured episode before it is saved; `n`
ends the reset wait early. Wait for saving/encoding and normal exit to finish.
Use `q` instead of Ctrl+C when you want the current episode saved: interruption
can occur before `save_episode()`. As with teleoperation, exit does not release torque.

### Resume an existing recording

Replace `TIMESTAMP` below with the exact suffix of an existing compatible dataset:

```bash
lerobot-record \
  --config_path=configs/record.yaml \
  --robot.enable_motors=true \
  --robot.high_follow=false \
  --robot.motion_speed=20 \
  --robot.max_joint_step_rad=0.012 \
  --resume=true \
  --dataset.root=data/piper_x_pick_TIMESTAMP \
  --dataset.repo_id=local/piper_x_pick_TIMESTAMP
```

Resume does not append a timestamp. It appends the configured number of additional
episodes. Use the same cameras, units, features, and control settings as the dataset.

### Remove already saved episodes

Stop recording first. Use the dataset editor, rather than deleting individual
Parquet/video files, because episode metadata and media references must stay aligned.
This example removes episode index `2` (the third episode) into a new filtered copy:

```bash
lerobot-edit-dataset \
  --repo_id=local/piper_x_pick_TIMESTAMP \
  --root=data/piper_x_pick_TIMESTAMP \
  --new_repo_id=local/piper_x_pick_TIMESTAMP_filtered \
  --new_root=data/piper_x_pick_TIMESTAMP_filtered \
  --operation.type=delete_episodes \
  --operation.episode_indices='[2]'
```

Replace `TIMESTAMP` with the actual suffix and use an unused output folder.
For multiple episodes use, for example, `'[0, 2, 5]'`. The original dataset is preserved.
Data is LeRobot **v3.0**; the older `PiperLerobotToolkit` produced v2.1. Verify your
training loader before collecting a large dataset.

## Limits, mapping, and remaining issues

Actions and observations use `joint_1.pos` through `joint_6.pos` in **radians**, then
`gripper.pos` in **metres**. Images are `observation.images.wrist` and
`observation.images.third_person`. There are no implicit sign flips, offsets,
joint aliases, or Cartesian transformations.

The supplied configs use limits queried from this follower on 2026-10-06:
J1 ±150°, J2 0–180°, J3 −170–0°, J4/J5 ±89°, J6 ±170°. Gripper range is 0–0.07 m.
The class's fallback limits differ; use the supplied configs for this installation.
Startup rejects out-of-envelope positions before creating the command SDK.
Earlier positive J3 readings caused this check to fail. Verify the current physical
pose and calibration; do not widen limits or reset zeros merely to bypass it.

Findings from the supervised user runs:

- At speed 20 / step 0.006, follower feedback age stayed below 5.12 ms and sampled
  receive delay below 0.17 ms. Tracking error still reached tens of degrees. This
  supports target limiting/physical tracking as a major cause of the apparent lag;
  it does not measure every source of end-to-end latency.
- At speed 20 / step 0.012, J3 moved about 9.8°/s during a catch-up segment. Faster
  leader motion still outran it. These are observations, not guaranteed rates.
- J3 numerically tracked negative leader angles: one sample was leader −62.294°,
  follower −62.293°. A later +0.883° leader request was correctly clipped to 0°.
- J4 and J6 moved in later logs; their earlier stationary readings did not establish
  failed motors.
- H and X have different joint-frame orientations and wrist geometry. Matching
  joint numbers is not proof of matching physical poses or gripper trajectories.
  Mapping/calibration verification and, if desired, Cartesian retargeting remain
  unfinished. Do not guess sign flips from appearance alone.

Reference robot models: [AgileX Piper-H URDF](https://github.com/agilexrobotics/agx_arm_urdf/blob/main/piper_h/urdf/piper_h_description.urdf)
and [AgileX Piper-X URDF](https://github.com/agilexrobotics/agx_arm_urdf/blob/main/piper_x/urdf/piper_x_description.urdf).

## Mode recovery

If startup says the follower did not enter CAN joint control, inspect its decoded
status first. Healthy joint control expects `ctrl_mode=1`, `mode_feed=1`,
`arm_status=0`, and `err_code=0`. Teaching recording stopped (`teach_status=2`)
does not itself imply CAN control.

For the explicit standby→CAN transition, stop other controllers, support the
follower at the manufacturer's physical zero pose, and turn the teaching light off:

```bash
python scripts/switch_follower_mode.py                         # Read-only preflight
python scripts/switch_follower_mode.py --execute --prepared    # Sends mode/target commands
```

The switch script checks limits and status, requests standby, waits for acknowledgement,
seeds current joint targets, and requests CAN joint mode. It sends no motor-enable,
reset, calibration, or arm-role commands. Normal teleoperation startup does not
perform this explicit intermediate standby sequence.

The [AgileX manual, pages 6–7](https://static.generation-robots.com/media/agilex-piper-user-manual.pdf)
describes the physical-zero and standby prerequisites. Single-click a solid-green
teaching button to stop recording; double-click starts playback. Do not use
`ResetPiper()` as a shortcut: the SDK documents immediate torque loss and possible falling.

## Run a trained model

No policy checkpoint has been supplied or executed on this hardware. For a
compatible LeRobot checkpoint, first match camera mapping, units, normalization,
control limits, and frequency to training, then use:

```bash
lerobot-rollout \
  --config_path=configs/rollout.yaml \
  --policy.path=/absolute/path/to/checkpoint/pretrained_model \
  --robot.enable_motors=true
```

The rollout YAML uses 30 FPS, a 30-second duration, and no automatic return to the
initial position. Supply policy-specific dependencies/device settings as required.
The model's postprocessed actions must be absolute joint radians and gripper metres
in the seven-value order above. OpenPI servers, Cartesian actions, delta actions,
and other custom interfaces require an explicit bridge.

## Verification

```bash
pytest -q
ruff check src tests scripts
```

The tests cover real LeRobot
teleoperation/recording loops with fake hardware, v3 dataset write/read, silent-leader
pause/resume, rejection of stale queued feedback, limit checks, and mode-switch sequencing.
Timestamp creation was separately checked with two successive temporary dataset
folders and preservation of the existing base folder. Unit tests do not establish
physical safety, exact H→X pose mapping, or policy performance.
