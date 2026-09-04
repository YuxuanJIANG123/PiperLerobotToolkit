# PiperLerobotToolkit

AgileX Piper 单臂工具仓库：用键盘进行低速关节遥操作，同时录制腕部和第三视角两台
Intel RealSense D435i，并保存为与 OpenPI π0.5 官方加载器兼容的 LeRobot Dataset v2.1。

> 真机首次运行必须保持急停可达并清空周围空间。本项目尚未在你的机械臂上验收，请先核对
> 关节方向、零位和限位。

## 数据定义

- `observation.state`：六关节（rad）及夹爪开度（m），共 7 维 float32。
- `action`：实际下发的同单位 7 维位置目标。
- `observation.images.wrist`：腕部 D435i RGB。
- `observation.images.third_person`：第三视角 D435i RGB。
- Dataset `codebase_version`：`v2.1`。

目标由最新编码器反馈加一个小增量产生；松开按键即保持当前位置，不会跳回零位。每帧还会
执行相对位移限制和 Piper 软件绝对限位。

默认 `disable_on_disconnect: false`：正常退出或保存异常时，Piper 保持使能并维持当前位置，
避免失能后受重力下坠。只有机械臂已被物理支撑时，才可改成 `true`。退出程序不等于急停，
需要彻底断能时请按 Piper 官方安全流程操作。

## 安装与硬件

项目通过 `.python-version` 固定 Python 3.10，由 uv 创建和维护仓库内的 `.venv`：

```bash
# 首次使用若尚未安装 uv：curl -LsSf https://astral.sh/uv/install.sh | sh
uv python install 3.10
uv sync --extra piper

# 视频编码和 CAN 工具属于系统依赖
sudo apt update
sudo apt install ffmpeg can-utils ethtool
sudo bash piper_scripts/can_activate.sh can0 1000000
ip -details link show can0
```

以后无需手动激活环境，直接使用 `uv run`。如有需要也可以执行 `source .venv/bin/activate`。

识别两个 D435i 的序列号：

```bash
uv run python lerobot/common/robot_devices/cameras/intelrealsense.py \
  --images-dir outputs/realsense-identification
```

两台相机名称相同，必须用 `serial_number` 区分。尽量接到不同 USB 3.x 控制器。

## 键盘遥操作

`Q/A`、`W/S`、`E/D`、`R/F`、`T/G`、`Y/H` 分别控制 J1 到 J6 正反向；
`O/L` 打开/关闭夹爪。松开即保持，`Ctrl+C` 退出。

```bash
uv run python lerobot/scripts/control_robot.py \
  --robot.type=piper --robot.inference_time=false \
  --robot.can_name=can0 --robot.cameras='{}' \
  --robot.joint_step_rad=0.003 --robot.gripper_step_m=0.0002 \
  --robot.max_relative_target_rad=0.006 --robot.motion_speed=15 \
  --control.type=teleoperate --control.fps=30
```

键盘直接从当前 POSIX 终端读取，不依赖 `pynput`、X11 或 Wayland。stdin 必须是交互式 TTY；
不要通过管道或重定向启动控制程序。按住按键时依靠系统键盘自动重复连续移动。

## 双 D435i 录制

双相机嵌套配置放在 `configs/piper_record.yaml`，其中已经写入当前实测序列号：

```bash
uv run python lerobot/scripts/control_robot.py \
  --config_path configs/piper_record.yaml
```

旧版 `draccus` 不会把 CLI 中的 `--robot.cameras="{...}"` 字符串解码成相机配置。请在 YAML
中修改相机序列号。任务和 episode 数等简单字段仍可在命令后覆盖：

```bash
uv run python lerobot/scripts/control_robot.py \
  --config_path configs/piper_record.yaml \
  --control.single_task="Pick up the red cube" \
  --control.num_episodes=10
```

录制时右方向键结束当前 episode，左方向键废弃并重录，`Esc` 保存已有 episode 并结束。
episode 之间的 reset 阶段仍可用同一套键盘按键重新摆放机械臂，同时手动恢复场景。

视频默认使用 FFmpeg 的 `libx264` 编码器。可用 `ffmpeg -encoders | grep libx264` 检查。

```bash
uv run python scripts/verify_dataset_v21.py data/piper_pick
uv run python lerobot/scripts/visualize_dataset.py \
  --repo-id local/piper_pick --root data/piper_pick --episode-index 0
```

## OpenPI π0.5

Piper 的 OpenPI `DataConfig` 通常映射：

```text
images.wrist      <- observation.images.wrist
images.base_0_rgb <- observation.images.third_person
state             <- observation.state
actions           <- action
prompt            <- task
```

Dataset v2.1 兼容不代表 ALOHA/LIBERO transform 能直接用于 Piper；还需声明 Piper 的 7 维动作
和相机键。首次运行建议 `motion_speed=10~15`、`joint_step_rad=0.001~0.003`。软件限位不能
替代固件保护和实体急停。退出时本项目保持反馈姿态后失能，不主动回零。

本项目基于 Hugging Face LeRobot 和社区 Piper 集成，采用 Apache-2.0 许可证。

## 开发与测试

```bash
uv sync --extra piper --extra test --extra dev
uv run pytest -q tests/test_piper_keyboard.py
```

依赖变化后运行 `uv lock` 更新锁文件。其他机器可执行
`uv sync --frozen --extra piper`，严格复现 `uv.lock` 环境。
