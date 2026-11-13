# Piper-X remote inference

Use the RTX 4090 as the primary OpenPI π₀.₅ server. D70 runs the Piper-X
client and keeps the robot CAN interface and wrist camera local. Use
[Superpod](#backup-superpod) when the 4090 is unavailable.

## Primary setup on the RTX 4090

From D70, connect over Tailscale with `ssh hkust-4090`. Run `nvidia-smi`,
then start the server from the customized OpenPI checkout on the 4090.
The command assumes its CUDA environment and Piper-X checkpoint are in place.

```bash
cd /home/yuxuan/projects/openpi
export XLA_PYTHON_CLIENT_PREALLOCATE=false

uv run --no-sync python -u scripts/serve_policy.py --port=8000 \
  policy:checkpoint --policy.config=pi05_piper_x \
  --policy.dir=checkpoints/pi05_piper_x/piper_x_pair_pick/9999
```

Once the server is listening, open a separate terminal on D70 and leave
this tunnel running:

```bash
ssh -NT -o ExitOnForwardFailure=yes -o ServerAliveInterval=15 \
  -L 127.0.0.1:18000:127.0.0.1:8000 hkust-4090
```

Then follow the [shared D70 client instructions](#shared-d70-client-instructions).
Keep port 8000 off the public internet.

## RTC check on the 4090

With the 4090 server and tunnel running, use the D70 environment from the shared client instructions below:

```bash
python -m examples.piper_x.main --host=127.0.0.1 --port=18000 \
  --robot-config="$TOOLKIT/configs/record.yaml" \
  --prompt="press the button" --rtc --action-horizon=8
```

This check leaves motors disabled and omits the live camera preview. The
server must advertise `rtc: true`. At 30 Hz with a 16-action prediction,
8 actions per replan gives the largest latency budget, about 267 ms.
The measured 311 ms peak exceeded that budget. If the check still fails,
compare server inference time with D70's guided end-to-end latency.
Once calibration passes consistently, add `--execute --duration=10`
for a short motion test with the precautions in the shared client instructions.

The D70 Wi-Fi test passed calibration but missed a runtime deadline at
about 300 ms. Wired IPv6 ping reached the 4090, but wired SSH timed out,
so the Ethernet RTC test is still pending.

## Shared D70 client instructions

### Run read-only inference

**In a second D70 terminal**, activate the robot toolkit environment and
check the tunnel. The health endpoint should print `OK`.

```bash
cd /home/jiayun/Documents/Openpi
TOOLKIT=/home/jiayun/Documents/PiperLerobotToolkit
source "$TOOLKIT/.venv/bin/activate"
curl -fsS http://127.0.0.1:18000/healthz
```

If you are setting up a fresh toolkit environment, install the client
dependencies once from the OpenPI checkout:

```bash
uv pip install --no-deps -e packages/openpi-client
uv pip install 'msgpack>=1.0.5,<2' 'websockets>=14,<16' \
  'dm-tree>=0.1.8' 'tyro>=0.9.5'
```

Stop any other process using the robot or wrist camera, then run the client
with motors disabled:

```bash
python -m examples.piper_x.main --host=127.0.0.1 --port=18000 \
  --robot-config="$TOOLKIT/configs/record.yaml" \
  --prompt="press the button" --show-camera
```

Look for `Server metadata` containing `pi05_piper_x`, followed by
`WARMUP model output (discarded)` and `ACTION` lines. The health check
verifies the tunnel. A valid action chunk verifies inference. The client
writes predictions to `outputs/piper_x_inference.jsonl`.

### Run a short motion test

After read-only inference succeeds, stop that client and clear the
robot workspace. Keep the physical stop within reach. Rerun the command
with execution enabled:

```bash
python -m examples.piper_x.main --host=127.0.0.1 --port=18000 \
  --robot-config="$TOOLKIT/configs/record.yaml" \
  --prompt="press the button" --show-camera \
  --execute --duration=10 --action-horizon=8
```

The client enables the motors after warmup and moves to the saved home
pose before applying policy actions. It applies up to eight actions from
each 16-step prediction before replanning. The 10-second duration starts
after warmup and homing. On Ctrl+C or a connection failure, the last
bounded target can remain active. Use the physical stop if immediate
stopping is required.

### Troubleshooting

| Symptom | What to check |
| --- | --- |
| `/healthz` fails | Check that the policy log says `server listening`, the SSH tunnel is still open, and its hostname matches the current allocation. |
| Server logs `connection rejected (200 OK)` | This is the normal response to an HTTP health check. |
| `Server metadata: {}` | Check `policy_metadata` in the Superpod checkout's `pi05_piper_x` entry in `src/openpi/training/config.py`, then restart the server so it reads the config. |
| Checkpoint path fails | Run the server from `/project/assembly/yuxuan/openpi_yuxuan` with `--policy.dir=checkpoints/pi05_piper_x/piper_x_pair_pick/9999`. |

## Backup: Superpod

Choose either server launch method below, then open the Superpod SSH
tunnel and follow the [shared D70 client instructions](#shared-d70-client-instructions).

### Interactive GPU node

**On D70**, open a terminal and log in:

```bash
ssh -o IdentityAgent=none hkust-superpod
```

**On the Superpod login node**, request a GPU shell. This example uses one GPU
for 30 minutes in `normal`. Slurm may wait for an allocation.

```bash
module load slurm
srun --account=assembly --partition=normal --nodes=1 --ntasks=1 \
  --gpus-per-node=1 --cpus-per-task=8 --mem=64G --time=00:30:00 \
  --pty bash -l
```

**Inside the allocated GPU shell**, record the hostname printed by
`hostname -s`. Use it later when opening the D70 tunnel.

```bash
hostname -s
nvidia-smi

cd /project/assembly/yuxuan/openpi_yuxuan
module purge
export SPACK_ROOT=/scratch/spack/2025
source "$SPACK_ROOT/dist/bin/setup-envs.sh" -y
module load gcc/13.3.0 cmake cuda/12.8.0 cudnn/9.2.0 boost/1.81.0 ffmpeg/7.1-bgiwpoy

export BOOST_ROOT=/cm/local/apps/boost/1.81.0/include
export PATH="$HOME/.local/bin:$PATH"
export UV_PROJECT_ENVIRONMENT="$PWD/.venv"
export OPENPI_DATA_HOME=/project/assembly/yuxuan/.cache/openpi
export XLA_PYTHON_CLIENT_PREALLOCATE=false

uv run --no-sync python -u scripts/serve_policy.py --port=8000 \
  policy:checkpoint --policy.config=pi05_piper_x \
  --policy.dir=checkpoints/pi05_piper_x/piper_x_pair_pick/9999
```

Wait for `server listening on 0.0.0.0:8000` and keep this terminal open.
When finished, press Ctrl+C to stop the policy server, then run `exit`
to release the interactive allocation.

### Batch job

**On the Superpod login node**, submit the existing script. The script in
this checkout requests one GPU in `preempt` for up to 30 minutes and starts
the same policy server on port `8000`.

```bash
cd /project/assembly/yuxuan/openpi_yuxuan
module load slurm
JOB_ID=$(sbatch --parsable slurm/piper_x_inference.sbatch)
printf 'Submitted job %s\n' "$JOB_ID"
squeue -j "$JOB_ID"
```

`PD` means the job is pending. `R` means it has a compute node. Follow the
job log from the same shell:

```bash
tail -F "logs/piper_x_inference_${JOB_ID}.log"
```

The log may take time to appear while the job is pending. Once it starts,
look for `Job ... running on dgx-..., policy port 8000`. Record the hostname
and wait for `server listening on 0.0.0.0:8000`. You can disconnect from the
login shell after submission. The batch job runs until cancelled or until
it reaches its time limit.

When finished, press Ctrl+C to exit `tail`, then run `scancel "$JOB_ID"` from
the submission shell to release the GPU. In a new shell, use the numeric
job ID printed at submission. If the job disappears from `squeue`, check
its final state with `sacct -j "$JOB_ID" --format=JobID,State,Elapsed,ExitCode`.

### Open the Superpod SSH tunnel on D70

**On D70**, close any old tunnel using local port `18000`. Open a new
terminal and enter the compute hostname printed by the interactive shell
or batch log when prompted.

```bash
read -r -p 'Superpod compute hostname: ' NODE
ssh -NT -o ExitOnForwardFailure=yes -o ServerAliveInterval=15 \
  -o IdentityAgent=none \
  -L "127.0.0.1:18000:${NODE}:8000" hkust-superpod
```

The SSH terminal stays quiet while the tunnel runs. Leave it open. The
Superpod login node connects to the policy port on the allocated compute
node. Repeat this step with the new hostname whenever Slurm assigns a
different node.
