# Piper-H leader / Piper-X follower with LeRobot

This workspace connects a user-identified Piper-H teaching leader to a Piper-X
follower through LeRobot's robot and teleoperator interfaces. **Follower motion has
been confirmed by the user and recorded joint feedback.** Joint mode copies joint angles directly. Optional relative end-effector mode now
maps Piper-H TCP motion to Piper-X joint targets using separate pinned models.
The new mapping is tested in software but has not been validated on the physical
arms; base alignment, tool frames, and joint conventions must be checked first.

## Setup

Run commands from this project directory:

```bash
cd /path/to/PiperLerobotToolkit
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
| Relative end-effector mode | Added model-based H→X FK/IK, relative anchors, translation scaling, optional orientation tracking, bounded IK, and a receive-only preview. Hardware frame verification remains required. |
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

## Relative end-effector teleoperation (experimental)

This mode uses the official Piper-H and Piper-X URDFs at revision
`f6642ce0d7872c686f29c99e9e10cd23d1d49313`. Model sources and the MIT license are
bundled in `src/lerobot_robot_piper_x/models/`; no runtime downloads are required.
The numerical solver uses NumPy; no SciPy, ROS, or external IK service is needed.

At the first observation, the mapper records both TCP poses and gripper openings.
There is no initial jump to the leader's absolute pose. With base rotation `A`,
translation scale `s`, and initial poses `L0`, `F0`, subsequent targets are:

```text
p_target = p_F0 + s * A * (p_L - p_L0)
R_target = R_F0                                      # track_orientation=false
R_target = A * (R_L * R_L0^T) * A^T * R_F0           # track_orientation=true
```

Gripper opening follows the relative leader-opening change, independently of IK,
and is clamped to the follower range. IK failures hold the gripper as well as joints.
The first six output actions remain follower joint angles, so existing limiters,
recording features, and policy action units are preserved.

### Check the model and frames before enabling motors

Reapply the updated loop hook if updating an existing installation:

```bash
python scripts/apply_leader_pause_patch.py
```

First run the offline demonstration; it does not open CAN sockets:

```bash
python scripts/preview_end_effector.py --synthetic --seconds 5
```

Then stop any controller commanding the follower and run the receive-only preview:

```bash
python scripts/preview_end_effector.py \
  --config=configs/teleoperate_eef.yaml \
  --seconds=20
```

This opens receive-only readers and never creates the command SDK or enables motors.
Gently move the supported leader and gripper to provide initial frames. Output
includes leader TCP position, follower/target position, target orientation as a
rotation vector, candidate joint targets, solve residuals/timing, and
`motor_commands_sent: false`. The real follower stays where it is, so a large
leader displacement may trigger the solution-distance guard in this preview.
A successful preview validates calculations, not physical motion or collision safety.

Set these fields in **both** EEF YAML files for your actual installation:

| `teleop.eef` field | Meaning |
| --- | --- |
| `leader_to_follower_rpy` | Roll, pitch, yaw in radians for the rotation mapping leader **base_link** axes into follower **base_link** axes. Identity is correct only if those axes are aligned. Relative control cancels the base translation. |
| `leader_tcp_xyz`, `follower_tcp_xyz` | Tool-center position in metres relative to each model's `link6`. Defaults `[0,0,0]` control `link6`, **not the fingertips**. Measure the gripper/tool offsets for tip control. |
| `leader_tcp_rpy`, `follower_tcp_rpy` | Tool-center orientation relative to each `link6`, in radians. |
| `leader_joint_offsets_rad`, `follower_joint_offsets_rad` | Optional verified encoder-to-model offsets: model angle = reported angle + offset. Defaults are zero; no guessed sign flips are applied. |
| `translation_scale` | Defaults to `0.5`: 2 cm of leader TCP movement requests 1 cm of follower movement. |
| `track_orientation` | The EEF YAML configs set `true`: map relative position and rotation. The library default remains `false`; explicitly set `false` to hold the starting tool orientation. |
| `frames_verified` | Defaults to `false`. Live teleoperator construction refuses EEF mode until you explicitly confirm the transforms/conventions by setting it true. Dry-run preview does not require it. |

The leader source defaults to its transmitted control targets, not independent
joint feedback. Verify that those values represent the leader pose in the selected
model conventions. Checking physical motion of each joint and computed TCP
movement is necessary; do not infer correctness merely from the model name.

### Live commands after frame verification

Clear the workspace and keep the physical stop accessible. Begin with small leader
movements. The local `teleoperate_eef.yaml` saves the tested settings: motors
enabled, frames verified, orientation tracking enabled, translation scale 0.5,
leader connection timeout 30 s, speed 20, joint step 0.012 rad, diagnostics on,
and no startup zeroing. Its workspace bounds are 1 m translation and 2 rad
rotation. These settings apply to this installation; the recording config
retains its separate defaults.

```bash
lerobot-teleoperate --config_path=configs/teleoperate_eef.yaml
```

Both EEF configs enable relative orientation tracking. You can make this explicit
with `--teleop.eef.track_orientation=true`; use `false` to hold the starting
orientation. The controller represents orientation with rotation matrices and
SO(3) rotation errors, which encode the same 3D rotations as unit quaternions.
Position scale does not scale rotation: a 10-degree leader rotation requests a
10-degree relative follower tool rotation, transformed by the base alignment.
The teleoperation rotation boundary is 2 rad (about 115 degrees) from the anchor;
exceeding it holds the follower. Joint and IK limits still apply. Matching tool
rotation does not require matching individual wrist joint angles.
The preview script is receive-only: `motor_commands_sent: false` means the
follower is intentionally stationary, even when IK reports `tracking`.
Use the live command above to execute targets after frame verification.

### Optional return to zero before teleoperation

Add `--robot.reset_to_zero_on_connect=true` to either joint or EEF teleoperation
with `--robot.enable_motors=true`. For example, for joint teleoperation:

```bash
lerobot-teleoperate --config_path=configs/teleoperate.yaml \
  --robot.enable_motors=true --robot.reset_to_zero_on_connect=true
```

This moves follower J1–J6 toward their existing zero angles at 30 Hz using the
configured joint step and speed limits; it retains the current gripper opening.
It never changes encoder calibration or moves the leader. Clear the entire
return path first: this joint-space move does not check for obstacles.
Completion requires all six joints within 0.005 rad (about 0.29°) of zero.
Fresh feedback and healthy controller status are required throughout; failure
or the default 60-second timeout aborts startup. Override the timeout with
`--robot.reset_to_zero_timeout_s=90` if needed. Ctrl+C interrupts the return;
disconnect retains the last bounded target rather than disabling torque.

After returning, joint mode follows the leader through the normal step limiter;
place the leader near zero before starting. EEF mode anchors at the follower's
new position. The option defaults to false and never causes a return on exit.

To record the same mapping with both cameras:

```bash
lerobot-record \
  --config_path=configs/record_eef.yaml \
  --teleop.eef.frames_verified=true \
  --robot.enable_motors=true
```

EEF recordings use `data/piper_x_eef_pick_TIMESTAMP` and
`local/piper_x_eef_pick_TIMESTAMP`; episode keys and resume/delete procedures are
unchanged. Use the same frame settings in recording as in teleoperation. In EEF
mode the motion diagnostic's `leader` field is the **mapped follower joint request**;
the separate `End-effector mapping` log reports Cartesian state and solver status.

### Bounds and failure behavior

For a wrist that appears stationary, read motor fault/enable flags without motion:

```bash
python scripts/inspect_motor_status.py
```

With other controllers stopped, inspect a one-degree J4 probe without enabling:

```bash
python scripts/probe_wrist.py --joint 4
```

Only after clearing the wrist path and placing the physical stop within reach,
add `--execute --prepared`. This enables the follower, holds the other joints
and gripper at their starting targets, and requests a one-degree positive move
for three seconds. It uses speed 10 and a 0.006 rad feedback-relative step.
Use `--joint 5` to test J5 separately; `--direction -1` selects a negative move.
`--step-rad 0.003` reproduces the smaller teleoperation offset.
The probe aborts on stale motor status, motor faults, or unexpected joint travel.
Ctrl+C stops commands; there is no automatic return or torque disable.

If a valid full IK target exceeds `max_solution_delta_rad`, the mapper tries
nearer Cartesian waypoints from the measured pose within the same solve-time
budget. Every accepted waypoint still satisfies the joint-distance guard and
the final motor step limiter. This prevents ordinary following lag from latching
a permanent hold. It does not bypass unreachable targets or workspace limits.

`tracking_position_error_m` is the measured TCP-to-requested-target error;
`position_error_m` is the full-target IK residual, not physical tracking accuracy.
`waypoint_fraction` below 1 indicates catch-up through an intermediate target.
`relative_workspace_limit` means the requested displacement exceeds the configured
radius (1 m in the saved teleoperation config). Reduce `translation_scale` to map larger leader motions
inside this radius. `track_orientation=false` holds the initial orientation;
it does not allow free tool rotation. Returning to zero is optional and puts
J2/J3 at firmware limit boundaries, so it is not a general EEF working pose.

- Library/recording relative workspace defaults: 10 cm translation and,
  when orientation is enabled, 0.7 rad rotation; the saved teleoperation config
  uses 1 m and 2 rad. Requests beyond the configured bounds hold rather
  than silently clipping the Cartesian pose.
- IK intersects the configured follower limits with the bundled URDF limits.
  The URDF currently caps J6 at ±120°, even though the stored firmware limit is
  ±170°. An initial pose outside that intersection is rejected.
- IK uses the current/nearby prior solution, damping, bounded iteration steps,
  line search, and a 20 ms solve budget checked between iterations. It accepts
  only solutions within 0.5 mm translation and 0.005 rad orientation residual.
  A candidate more than 0.5 rad from any measured joint is rejected.
- `relative_workspace_limit`, `singular`, `solve_timeout`,
  `unreachable_or_no_convergence`, or `joint_solution_too_far` means the mapper
  holds the captured follower pose; it does not send the failed partial solution.
  Returning to a solvable target can resume following through the normal limiter.
- Leader silence retains the existing pause behavior. When all targets refresh,
  **EEF mode re-anchors at both current poses**, discarding movement made during
  the pause. Restarting a session also captures new references.
- The final follower limiter still applies after IK. Consequently an intermediate
  executed joint target need not exactly meet the requested Cartesian pose.
  This mode does not eliminate speed limits or guarantee a straight TCP path.
- There is **no collision checking, obstacle avoidance, force control, or physical
  calibration solver**. Passing software tests does not validate the physical
  frame mapping or make autonomous operation safe.

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
`observation.images.third_person`. Joint mode has no implicit sign flips, offsets, joint aliases, or Cartesian
transformations. End-effector mode applies only the explicit model/frame mapping
configured below; its output is still joint radians and gripper metres.

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
  Model-based relative Cartesian retargeting is now implemented, but physical
  mapping/calibration verification remains unfinished. Do not guess sign flips
  from appearance alone.

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
pause/resume, rejection of stale queued feedback, limit checks, mode-switch
sequencing, FK/Jacobian consistency, constrained IK, relative frame mapping,
no-jump anchoring, and EEF integration through both actual LeRobot loops.
Timestamp creation was separately checked with two successive temporary dataset
folders and preservation of the existing base folder. Unit tests do not establish
physical safety, exact H→X pose mapping, or policy performance.
