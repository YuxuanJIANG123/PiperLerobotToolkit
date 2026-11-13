# PiperLerobotToolkit

Joint-position teleoperation and demonstration recording for two AgileX Piper-X
arms with LeRobot. A hand-guided leader controls the follower's six joints and
gripper directly, using radians and metres.

For OpenPI π₀.₅ deployment, see the [remote inference guide](REMOTE_INFERENCE.md).
The RTX 4090 hosts the policy, D70 runs the robot client, and Superpod is
available as a backup.

## 1. Environment setup

Requires Linux with SocketCAN, two USB-CAN adapters, a C compiler and Linux input
headers (`build-essential` and `linux-libc-dev` on Ubuntu), and
[`uv`](https://docs.astral.sh/uv/getting-started/installation/).
Recording also requires a wrist Intel RealSense camera.

```bash
git clone https://github.com/YuxuanJIANG123/PiperLerobotToolkit.git
cd PiperLerobotToolkit
uv venv --managed-python --python 3.12 .venv
source .venv/bin/activate
bash scripts/setup.sh
```

Setup applies the local teleoperation and recording patches and installs the
pinned LeRobot revision, `piper-sdk==0.6.2`, and this adapter. Run the commands
below from the project directory with the environment activated.

### Recover from a failed `evdev` build

On Ubuntu, install the compiler and Linux input headers if the build reports
`gcc` or `linux/input.h` missing, then rerun setup.

```bash
sudo apt update && sudo apt install build-essential linux-libc-dev &&
  bash scripts/setup.sh
```

If the build reports `Python.h` missing, open a fresh shell in the project
directory. Download uv-managed Python 3.12, back up the existing environment,
and rebuild it.

```bash
uv python install --no-bin 3.12 &&
  backup_dir=$(mktemp -d .venv.backup.XXXXXX) &&
  mv .venv "$backup_dir/" &&
  uv venv --managed-python --python 3.12 .venv &&
  source .venv/bin/activate &&
  bash scripts/setup.sh
```

### Configure CAN names once

Connect both adapters and check their interface names and serial numbers:

```bash
ip -details link show type can
udevadm info --query=property --path=/sys/class/net/can0
udevadm info --query=property --path=/sys/class/net/can1
```

Use the actual interface names if different. Match `ID_SERIAL_SHORT` to the
leader/follower wiring in [configs/udev/99-piper-can.rules](configs/udev/99-piper-can.rules).
Update the rules for your adapters before installing them. Names follow the
USB adapters, so keep each adapter connected to its assigned arm.

```bash
sudo install -m 0644 configs/udev/99-piper-can.rules /etc/udev/rules.d/99-piper-can.rules
sudo udevadm control --reload-rules
```

Stop controllers, unplug and reconnect both USB adapters, then verify that
`piper_leader` and `piper_follower` appear:

```bash
ip -details link show type can
```

### Bring up CAN after each reboot or reconnection

Check the current state with `ip -details link show type can`. Run the matching
command below for each interface that is down.

```bash
sudo ip link set piper_follower up type can bitrate 1000000
sudo ip link set piper_leader up type can bitrate 1000000
```

If setting the bitrate returns `Device or resource busy` and the interface
already shows `UP` with `bitrate 1000000`, leave it running.

#### Recover a missing `piper_leader` interface

If the command reports `Cannot find device "piper_leader"` and the CAN link
listing shows an unnamed adapter such as `can0`, check its serial.

```bash
udevadm info --query=property --path=/sys/class/net/can0 | grep '^ID_SERIAL_SHORT='
```

If the serial matches the `piper_leader` rule and `can0` is down, rename and
bring up that interface. Substitute the actual interface name if it differs.

```bash
sudo ip link set dev can0 name piper_leader
sudo ip link set dev piper_leader up type can bitrate 1000000
```

The manual rename lasts until the adapter disconnects or the machine reboots.
To apply the installed udev rule, stop the controllers, run
`sudo udevadm control --reload-rules`, and unplug and reconnect the leader's USB
adapter. Check that it appears as `piper_leader`, then bring it up again.

#### Verify arm feedback

Use `piper-inspect` to check for joint feedback without sending motion commands.
An output containing `positions_rad_m` confirms that joint feedback was received.

```bash
piper-inspect --channel piper_leader --source feedback --seconds 3
piper-inspect --channel piper_follower --source feedback --seconds 3
```

## 2. Teleoperation

1. Stop other controllers. Turn OFF teaching recording/playback on the follower.
2. Prepare the leader for hand guiding using its hardware procedure. Start both
   arms near matching joint poses, clear the follower workspace, and keep the
   physical stop accessible.
3. Start teleoperation (this enables follower motors):

   ```bash
   bash teleopeation.sh
   ```

4. Move the leader gently and check that the follower tracks it. Press Ctrl+C
   to close the program.

The launcher filename is `teleopeation.sh`. It uses
[configs/teleoperate.yaml](configs/teleoperate.yaml): 15 Hz, 100% controller
speed, and bounded targets. It does not home either arm or switch the leader's
teaching mode. Override settings as needed, for example:

```bash
bash teleopeation.sh --fps=30 --robot.motion_speed=50
```

**Closing any program does not disable torque or act as an emergency stop.**
Target bounds do not provide collision avoidance.

## 3. Record demonstrations

### Prepare

- Review [configs/record.yaml](configs/record.yaml). Verify the saved reset pose
  and both arms' limits before allowing powered returns.
- Connect the wrist camera. The supplied serial is `346122071508`; update
  `robot.cameras.wrist.serial_number_or_name` for your camera.
- Clear both arms' return paths, stop other controllers, and ensure follower
  teaching recording/playback is OFF.
- Optionally run `python scripts/check_setup.py` for a read-only state check
  and camera preview at `outputs/read_only_check/wrist.png`.

### Start and follow the prompts

```bash
lerobot-record --config_path=configs/record.yaml --robot.enable_motors=true
```

1. Keep clear while the follower returns to the saved starting pose.
2. When prompted, ensure the leader teaching button is **OFF**. Wait for the
   leader's powered return; do not hand-guide it during this movement.
3. At **`ACTION: reset complete`**, support the leader and turn teaching **ON**.
   Keep the leader and gripper near the returned pose while alignment is checked.
4. Begin the demonstration only at **`Recording episode ...`**.
5. End capture with **`n` / Right arrow**, or turn leader teaching **OFF**.
   Capture also ends after 30 seconds.
6. Follow the same return/button prompts for the next episode. Reset the object
   while keeping clear of moving arms. After preparation, keep teaching ON and
   hold the starting pose through saving; the next episode starts automatically.

| Key during capture | Action |
| --- | --- |
| `n` / Right arrow | End capture, reset, and save |
| `r` / Left arrow | Discard the unsaved episode and retry |
| `q` / Esc | Save the current episode and exit normally |

Wait for saving to finish. Prefer `q` to Ctrl+C when saving matters. There is no
automatic return after the final episode or on quit. See
[RECORDING_STEPS.md](RECORDING_STEPS.md) for detailed prompts and recovery guidance.

### Diagnose a leader return timeout

If the recorder reports `Leader return to saved pose timed out`, check that the
saved pose is reachable and both arm paths are clear. To log the remaining joint
and gripper errors during the return, run recording with leader motion logging.
The timeout override restores the 60-second return window if you shortened it
in your local configuration. This command enables motor power.

```bash
lerobot-record --config_path=configs/record.yaml --robot.enable_motors=true \
  --teleop.debug_motion=true --teleop.reset_timeout_s=60.0
```

### Dataset settings and resume

Defaults: **100 episodes × 30 seconds, 30 FPS, wrist camera only, no Hub upload**.
Reset movement is excluded from capture; there is no extra environment hold.
New runs use matching timestamped IDs and folders:

- Dataset ID: `local/piper_x_pair_pick_TIMESTAMP`
- Local folder: `data/piper_x_pair_pick_TIMESTAMP`

Override the task or episode count when starting a run:

```bash
lerobot-record --config_path=configs/record.yaml --robot.enable_motors=true \
  --dataset.single_task="Pick up the object and place it in the container" \
  --dataset.num_episodes=5
```

To add episodes, replace `TIMESTAMP` with the exact existing dataset name and
keep its original camera, FPS, unit, and control settings:

```bash
lerobot-record --config_path=configs/record.yaml --robot.enable_motors=true \
  --resume=true --dataset.root=data/piper_x_pair_pick_TIMESTAMP \
  --dataset.repo_id=local/piper_x_pair_pick_TIMESTAMP
```

## Other tools

- [configs/replay.yaml](configs/replay.yaml): replay recorded actions on the
  follower. Set the exact dataset ID, folder, and episode before running
  `lerobot-replay --config_path=configs/replay.yaml --robot.enable_motors=true`.
  Prepare the follower near the episode's starting pose; replay enables motors.
- [configs/rollout.yaml](configs/rollout.yaml): policy rollout configuration.
- `python scripts/switch_follower_mode.py`: read-only mode preflight if the
  follower cannot enter CAN control. Stop active teaching/playback first.
- Development checks: `pytest -q` and `ruff check src tests scripts`.
  Tests use simulated hardware.
