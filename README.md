# PiperLerobotToolkit

Direct joint-position teleoperation between two AgileX Piper-X arms using LeRobot.
One arm is a hand-guided leader during capture; the other follows its six joint angles
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

CAN numbers such as `can0` and `can1` can change after reconnecting. Use the
adapters' unique USB serial numbers to assign persistent role names. The mapping
below was read from the adapters at the previously documented USB ports (leader
`5-1`, follower `1-4`); verify the physical CAN wiring before installing it:

| Role | USB adapter serial | Persistent interface name |
| --- | --- | --- |
| Leader | `0027002F4648570B20303731` | `piper_leader` |
| Follower | `002E00254648570A20303731` | `piper_follower` |

These serials identify the USB-CAN adapters, not the arms. Keep each adapter with
its assigned arm. Swapping USB ports is fine; swapping the CAN cables between
arms changes the physical roles and requires updating the mapping.

Inspect currently connected adapters without transmitting CAN commands:

```bash
ip -details link show type can
udevadm info --query=property --path=/sys/class/net/can0
udevadm info --query=property --path=/sys/class/net/can1
```

Look for `ID_SERIAL_SHORT`. Substitute the current interface names if needed.
Recording and teleoperation require two USB-CAN interfaces. If only one appears,
check the second adapter's USB cable and port. This lists CAN adapters; it does
not confirm that the arms are powered or sending feedback.

For different adapters, update the serials in `configs/udev/99-piper-can.rules`.
After verifying the mapping, install the persistent naming rules once:

```bash
sudo install -m 0644 configs/udev/99-piper-can.rules /etc/udev/rules.d/99-piper-can.rules
sudo udevadm control --reload-rules
```

Reloading udev rules does not rename adapters that are already connected. The
rules apply when the adapters are added. After installing the rules:

1. Stop any robot controllers.
2. Unplug both adapters' USB cables and plug them back in.
3. Verify the interface names:

```bash
ip -details link show type can
```

The output should contain `piper_follower` and `piper_leader`. Once both names
appear, bring up the interfaces:

```bash
sudo ip link set piper_follower up type can bitrate 1000000
sudo ip link set piper_leader up type can bitrate 1000000
```

If you get `Cannot find device "piper_follower"` or
`Cannot find device "piper_leader"`, the expected name has not been assigned.
If the interfaces still appear as `can0` and `can1` after reconnecting, inspect
their USB attributes and the installed rules:

```bash
udevadm info --attribute-walk --path=/sys/class/net/can0
udevadm info --attribute-walk --path=/sys/class/net/can1
cat /etc/udev/rules.d/99-piper-can.rules
```

Substitute the current interface names if needed. Check that each adapter's
`serial`, `idVendor`, and `idProduct` match its rule in
`configs/udev/99-piper-can.rules`. Correct any mismatches, reinstall and reload
the rules, then unplug and reconnect both USB adapters again.

The names now follow the adapter serials across reconnections and reboots.
The rules only assign names: run the `ip link ... up` commands after each
reconnection or reboot. Replay needs only `piper_follower`; recording and
teleoperation need both interfaces.

Inspect without transmitting commands:

```bash
piper-inspect --channel piper_leader --source feedback --seconds 3
piper-inspect --channel piper_follower --source feedback --seconds 3
python scripts/inspect_can.py
```

## Teleoperate

Prepare the leader for hand guiding using its supported hardware procedure. The
standalone teleoperation config only receives leader feedback; automatic mode
transitions below apply to recording. It does not disable torque, change arm roles,
or implement gravity compensation. The follower must have teaching
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
| Motion diagnostics | Off | Off |

These target-offset bounds are not velocity guarantees. Controller speed 100%
does not guarantee matching the leader's speed; 15 Hz updates arrive about every
67 ms. Grip effort remains 1000 in SDK units. Start with small movements after
changing settings. CLI overrides are accepted, for example:

```bash
bash teleopeation.sh --robot.motion_speed=50 --robot.max_joint_step_rad=0.012
```

Normal recording output keeps button prompts, episode messages, and errors.
Repeated follower/leader motion values, the full startup config dump, and timing
summaries are hidden. Enable `--robot.debug_motion=true --teleop.debug_motion=true`
when detailed recording diagnostics are needed. Alignment reminders repeat every
five seconds in normal output instead of every second. Video encoder output may
still appear while saving episodes.

If following feels delayed, tune one setting at a time with small leader motions:

| Parameter | Effect |
| --- | --- |
| `robot.motion_speed` | Planned-mode controller speed, 1–100. Recording defaults to 50; try 75 first. |
| `fps` (teleoperation) | Command update rate. Try 30 instead of the default 15 (about 33 ms instead of 67 ms per update). |
| `dataset.fps` (recording) | Recording and control rate, currently 30. Keep it consistent with the camera and dataset; existing datasets require their original FPS. |
| `robot.max_joint_step_rad` | Maximum commanded lead over measured joints, currently 0.03 rad (about 1.72°). A tight bound can make fast leader motions lag; increasing it permits larger target steps. |
| `robot.max_gripper_step_m` | Equivalent target bound for the gripper (recording: 0.003 m). Tune this if only the gripper lags. |
| `robot.high_follow` | Selects SDK instant-response mode (`0xAD`). Leave off until compatibility with the installed firmware is verified. |

For recording, first try the existing 30 FPS with a moderate speed increase:

```bash
lerobot-record --config_path=configs/record.yaml --robot.enable_motors=true \
  --robot.motion_speed=75
```

`feedback_timeout_s` is a stale-feedback cutoff, not an intentional delay;
reducing it does not make following faster. Reset timeouts and reset windows
also do not affect following latency. With `robot.debug_motion=true`, compare
measured, requested, and transmitted targets plus `feedback_timing` in the logs:
a transmitted target much closer to measured than requested indicates target
clipping. Camera acquisition and dataset writing share the recording loop and
can also reduce its achieved rate. Compare with camera-free teleoperation at
30 FPS to distinguish recording overhead from arm response:

```bash
bash teleopeation.sh --fps=30 --robot.motion_speed=75
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

`configs/record.yaml` and `configs/rollout.yaml` use only the wrist RealSense
camera (serial `346122071508`, 640 × 480 at 30 FPS). The third-person camera is
not required. Update `robot.cameras.wrist.serial_number_or_name` in both files
if you replace the wrist camera; preview with
`python scripts/check_setup.py` (no motor commands).

Start a new dataset for wrist-only recordings. Existing two-camera datasets
must be resumed with their original camera configuration, and policies that
require third-person images need that camera restored in the rollout config.

```bash
lerobot-record --config_path=configs/record.yaml --robot.enable_motors=true
```

Defaults: 10 episodes, 30 seconds each, no extra environment hold, 30 FPS, no Hub
upload. New runs create matching timestamped IDs/folders beneath
`local/piper_x_pair_pick_TIMESTAMP` and `data/piper_x_pair_pick_TIMESTAMP`.
Data uses LeRobot v3 format. Leader-silence holds do not pause the episode clock.

`robot.recording_reset_pose` stores the wrist-camera starting pose captured on
2026-10-22 (joint radians and gripper metres). With
`robot.recording_reset_on_connect: true`, an enabled follower returns to this
pose on connection before the first episode. Read-only checks do not move it.
The leader is motorized and is hand-guided in teaching mode during capture.
`teleop.automatic_reset: true` enables this sequence before every episode,
including retries, outside episode timing:

1. Hold the follower at the saved pose and prompt you to turn OFF the leader's
   teaching button. Recording config uses `teleop.teaching_mode_switch: manual`;
   it sends no SDK teaching commands. Wait for the prompt before switching.
2. Confirm teaching exit with fresh status, seed measured leader joint targets,
   then confirm standby and CAN joint control.
3. Enable leader motors and return its joints and gripper to the saved pose.
   Leader reset uses independent `teleop.reset_joint_limits_rad`, 30% controller
   speed, 0.003 rad joint target offsets, and 0.001 m gripper target offsets.
   Verify the conservative leader limits against the installation; follower
   limits are not copied to the leader.
   `teleop.reset_joint_tolerance_rad` is 0.0610865 rad (3.5°) in the recording
   config, accepting the observed 2.34°/2.96° wrist residuals. Gripper reset
   tolerance is 30 mm (`teleop.reset_gripper_tolerance_m`). This accepts an
   approximate starting pose; it does not fix the reason those joints stalled.
   Command bounds and absolute limits remain enforced. The follower's reset
   tolerance is unchanged. Keep leader reset tolerances no larger than alignment
   tolerances so an accepted return can pass the subsequent alignment check.
   During return, `Leader reset progress` logs report leader measurements,
   commands and remaining joint/gripper errors once per second. Small bounded
   targets can still take several seconds; repeated follower hold
   diagnostics alone do not indicate a leader failure. A return timeout lists
   the axes that have not reached the target.
   Gripper feedback may extend up to 1 mm beyond an endpoint (or the configured
   gripper target-offset bound, whichever is smaller), to tolerate small encoder
   offsets such as -0.7 mm at closure. The recording config sets the leader's
   `teleop.reset_gripper_max_m` to 0.10 m, accommodating its reported travel beyond
   95 mm. This 100 mm ceiling is provisional; verify the actual maximum reading.
   Leader commands remain inside that range; the follower retains its separate
   0–70 mm range. Motion bounds use the raw feedback. Larger gripper feedback
   excursions still abort.
   Leader joint feedback alone allows up to 0.01° beyond an endpoint for encoder
   quantization (for example, 90.002° at a 90° limit). Larger joint overruns still
   abort. Seed commands, movement commands and saved targets retain the original
   joint limits; the feedback tolerance does not widen the command range.
   A 95 mm leader opening is within 30 mm of the saved 69.4 mm target, so it can
   pass reset/alignment. Teleoperation still copies openings in metres directly:
   leader openings above 70 mm saturate at the follower limit, with no proportional
   scaling. Wider tolerance accepts an approximate gripper start, not matching
   opening percentages between different grippers.
4. Print `ACTION: reset complete` and prompt you to support the leader and turn
   ON its teaching button. Require fresh status reporting control mode 2 and
   teaching state 1, then fresh position frames. Keep the leader near its starting
   pose through preparation and episode saving. Start the task only when
   `Recording episode ...` appears; `preparation complete` does not start capture.
5. Check both arms' alignment before allowing capture. Teaching status is also
   checked during capture. In manual-button recording, a healthy teaching-OFF
   transition ends the episode normally, just like `n`, without sending another
   follower command. Faults, stale status and unsupported mode changes still abort.

Between episodes the follower returns first, then the leader reset prompts run
immediately, before episode encoding. Turn teaching OFF
when prompted (or before pressing `n` to end capture), keep clear during powered
return, then turn teaching ON only after `ACTION: reset complete`. After alignment,
`Both arms prepared; saving episode with no extra environment hold` confirms
preparation. `dataset.reset_time_s: 0` removes the extra environment hold; arm
returns and teaching/alignment checks still run. Reset the object during the arm
reset sequence while keeping clear of moving arms. The next episode rechecks
alignment without resetting the leader again.
If teaching is OFF at that recheck (including after episode encoding), the code
holds the follower and prompts you to turn teaching ON instead of crashing.
It rechecks alignment for the full dwell time after teaching resumes. Keep the
leader near its starting pose while waiting for `Recording episode ...`.
Keep both arms' return paths clear, then support the leader for the teaching
handoff. No one should hand-guide it during the motorized return. Mode changes
between standby and CAN have a 5-second timeout each; leader return has a
60-second timeout. Button prompts allow `teleop.teaching_button_timeout_s`
(120 seconds). Unsupported
firmware acknowledgements, faults or stale feedback abort without starting
capture. SDK teaching transitions remain available through
`teleop.teaching_mode_switch: sdk` only for firmware verified to support them;
button teaching mode can override protocol requests. This code never requests
playback, motor disable, role changes, or zero resets.
On cancellation or failure it closes the SDK connection without disabling
torque or attempting another mode change; check the physical mode before handling
the leader. These transitions have automated simulation coverage but have not
yet been verified on this hardware.

With `teleop.require_start_alignment: true`, logs show signed joint differences
in degrees and gripper difference in millimetres (leader minus saved pose).
Both arms must remain within `teleop.alignment_joint_tolerance_rad` (0.0610865 rad,
3.5° in the recording config) and `teleop.alignment_gripper_tolerance_m` (30 mm) for
`teleop.alignment_hold_s` (0.5 s). Capture starts automatically after alignment;
waiting adds no frames and consumes no episode time. Press `q`/Esc to stop;
Right arrow cannot bypass alignment. `teleop.alignment_timeout_s` defaults to
120 s and aborts on timeout. To use manual leader reset instead, set
`teleop.automatic_reset: false` and place the leader in teaching mode yourself;
the alignment check then sends no leader commands.

Between episodes, including retries, the follower returns to this pose using the
configured target-offset bounds. After leader preparation, it holds for
`dataset.reset_time_s` only if that setting is positive (default: 0, no hold).
Return travel has a separate 60-second timeout;
stale feedback, faults, or timeout abort the run. Reset movement is not recorded,
and the leader does not control the follower during reset. The gripper also
returns to its saved opening (69.4 mm). Keep the return path clear. Right arrow can
end the hold early but cannot skip reaching the pose; `q` stops reset. There is
no automatic reset after the final episode or on quit. Set
`robot.recording_reset_on_connect: false` to skip the startup return. Set that
flag to false, `teleop.automatic_reset: false`,
`teleop.require_start_alignment: false`, and `robot.recording_reset_pose: null`
to restore manual teleoperation during reset.

- `n` / Right arrow: end capture early, proceed through reset and save.
- Leader teaching OFF button (manual-button automatic-reset recording): end capture
  early and proceed through reset and save. No extra `n` is required.
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

## Replay a demonstration

`lerobot-replay` sends one episode's recorded actions to the follower.
`configs/replay.yaml` uses the same follower settings as `configs/record.yaml`;
no leader or cameras are needed. Use a joint-position dataset recorded with this
adapter (`piper_x_pair_pick_*`). Set `dataset.repo_id` and `dataset.root` in
`configs/replay.yaml` to the exact existing dataset ID and folder, replacing
`TIMESTAMP`. Set `dataset.episode` to the desired episode index, starting at zero.

Verify the follower CAN interface is up and matches `robot.can_name`. Stop any
recording or teleoperation process, turn off teaching recording/playback on the
follower, and prepare it near the episode's starting pose. Clear its workspace
and keep the physical stop accessible. From the project directory, the following
command enables motors and moves the follower:

```bash
source .venv/bin/activate
lerobot-replay --config_path=configs/replay.yaml --robot.enable_motors=true
```

You can also override the dataset and episode on the command line:

```bash
lerobot-replay --config_path=configs/replay.yaml --robot.enable_motors=true \
  --dataset.repo_id=local/piper_x_pair_pick_TIMESTAMP \
  --dataset.root=data/piper_x_pair_pick_TIMESTAMP \
  --dataset.episode=0
```

Replay uses the dataset's recorded frame rate (30 FPS with the supplied recording
configuration). In this pinned LeRobot version, `--dataset.fps` does not override
replay timing. Joint limits and target-offset bounds still apply, so physical
tracking may differ from the recording. There is no automatic move to the first
recorded pose. Ctrl+C closes communication but does not disable torque or act as
an emergency stop.

To watch the recorded camera videos instead of moving the robot, open the MP4
files beneath `data/piper_x_pair_pick_TIMESTAMP/videos/`.

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
