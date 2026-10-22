# Recording demonstrations with the wrist camera

This guide uses `configs/record.yaml`: two Piper arms, one wrist camera,
automatic powered returns, and manual teaching-button changes on the leader.

**Move the leader by hand only in teaching mode. Begin the demonstration only
when `Recording episode ...` appears.** `Preparation complete` is not the start
of capture. Episode numbering starts at zero.

## 1. Prepare

1. Power and connect both arms, both USB-CAN adapters, and the wrist camera.
2. Stop other robot controllers. Clear both arms' return paths and keep the
   physical stop accessible.
3. Open a terminal in this project and activate the environment:

   ```bash
   source .venv/bin/activate
   ```

4. Check and bring up the CAN interfaces:

   ```bash
   ip -details link show type can
   sudo ip link set piper_follower up type can bitrate 1000000
   sudo ip link set piper_leader up type can bitrate 1000000
   ```

   Both `piper_follower` and `piper_leader` must exist. If they still appear as
   `can0`/`can1`, follow the persistent naming instructions in `README.md`.

5. Place the object where the wrist camera can see it from the saved starting
   pose. An optional read-only preview is available with
   `python scripts/check_setup.py`; inspect `outputs/read_only_check/wrist.png`.
   This previews the current pose, not a simulated saved pose.

## 2. Start the first episode

Run:

```bash
lerobot-record --config_path=configs/record.yaml --robot.enable_motors=true
```

1. The follower returns to its saved starting pose. Keep clear of its movement.
2. When prompted to **turn OFF the leader teaching button**, ensure teaching is
   OFF and keep clear. If it is already OFF, do not toggle it ON and OFF.
3. Wait while the leader returns under motor power. Do not hand-guide it or
   turn teaching ON during this return.
4. At **`ACTION: reset complete`**, support the leader and turn teaching **ON
   once**. Keep the leader and gripper near the returned starting pose.
5. Wait for teaching confirmation and alignment. If prompted about alignment,
   gently adjust the supported leader in teaching mode toward the saved pose.
6. At **`Recording episode 0`**, begin the task by moving the leader and its
   gripper. The follower tracks them and the wrist camera records.

There is no between-episode environment hold before the first episode.

## 3. Finish an episode and prepare the next

1. Finish capture using either method:
   - Press **`n` / Right arrow**, then turn teaching OFF when prompted.
   - Turn teaching **OFF directly** to end capture. No additional `n` is needed.
   Capture also ends automatically after the configured 30 seconds.
2. Keep clear while the follower returns to the saved starting pose.
3. At the teaching-OFF prompt, ensure teaching is OFF. Wait for the leader's
   powered return; do not turn teaching ON yet.
4. At **`ACTION: reset complete`**, support the leader and turn teaching **ON
   once**. Keep it near the starting pose.
5. Reposition the object during the arm reset sequence, keeping clear of both
   moving arms. There is **no extra environment hold** after alignment
   (`dataset.reset_time_s: 0`).
6. At **`Both arms prepared; saving episode with no extra environment hold`**,
   leave teaching ON and keep the leader near the starting pose. Do not press
   `n` again to advance; saving and the next episode proceed automatically.
7. Wait for episode saving/video encoding and the final alignment check. Do not
   move the leader away or toggle teaching while waiting.
8. At **`Recording episode 1`**, begin your second demonstration. Repeat this
   sequence for subsequent episodes.

The current recorder saves the completed episode after its reset phase. Wait
for saving to finish; do not assume an episode is on disk immediately after `n`.

## 4. Understand the waiting messages

| Message | Your action |
| --- | --- |
| `turn OFF the leader teaching button` | Ensure teaching is OFF; powered reset follows. |
| `Leader returning under motor power` | Keep clear; do not hand-guide or toggle teaching. |
| `ACTION: reset complete ... turn ON` | Support the leader and turn teaching ON once. |
| `Waiting up to 120 s` | This is a timeout, not a required delay. Continue by performing the requested button action. |
| `checking start alignment` / `Waiting for start alignment` | Keep the leader and gripper near the saved pose; capture has not started. |
| `preparation complete` | Keep the leader near its starting pose; wait for the episode message. |
| `saving episode with no extra environment hold` | Leave teaching ON and hold the starting pose during saving. No extra `n` is needed. |
| `teaching is OFF before capture` | Support the leader, turn teaching ON, and wait for alignment. Do not press `n` to bypass this check. |
| `Recording episode ...` | Begin the demonstration. |

## 5. Retry or stop

- **`r` / Left arrow during capture:** discard the current unsaved episode and
  repeat it through the reset sequence.
- **`q` / Esc during capture:** end capture, save the current episode, and quit
  normally. Prefer this to Ctrl+C when saving matters.
- **`q` / Esc during a reset/button/alignment wait:** request cancellation.
- A keyboard quit is not an emergency stop. Closing the program does not
  automatically disable torque or return either arm after the final episode.

Defaults are 10 episodes, 30 seconds per capture, 30 FPS, and no extra
between-episode environment hold. New runs create timestamped dataset folders.
Leader powered reset uses 30% controller speed (`teleop.reset_motion_speed`),
with the existing 0.003 rad joint and 0.001 m gripper target-offset bounds.
The leader accepts a 3.5° joint reset/alignment error and 30 mm gripper error;
the follower retains its own limits and tighter return check. Large leader
openings saturate at the follower's 70 mm maximum rather than scaling.
Tiny leader joint feedback overruns up to 0.01° at a configured limit are
accepted; seed and movement commands remain within the original limits.
