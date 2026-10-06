# rl_sar: Go2 / Go2W + MuJoCo

A focused C++17 version of [rl_sar](https://github.com/fan-ziqi/rl_sar) for running
Unitree Go2 and Go2W policies in MuJoCo. It retains TorchScript/LibTorch and ONNX
Runtime, historical observations, and wheel velocity control. The default build
has no ROS, ROS2, Gazebo, Python development library, or robot SDK dependency.
Go2/Go2W hardware deployment and Linux USB gamepads are optional.

[中文说明](README_CN.md)

## Build

On Ubuntu:

```bash
sudo apt install cmake g++ libyaml-cpp-dev libtbb-dev libglfw3-dev git curl wget unzip
./setup_deps.sh
./build.sh
ctest --test-dir build --output-on-failure
```

Setup downloads both inference backends, MuJoCo 3.2.7, and only Go2/Go2W model
resources. No recursive submodule checkout is needed. Builds are offline and
use ordinary CMake, defaulting to Release and two parallel jobs (`BUILD_JOBS`).
The bundled MuJoCo UI requires matching 3.2.7 headers/library.

On macOS, install `cmake yaml-cpp tbb glfw` with Homebrew; simulation is supported,
but hardware deployment and Linux USB gamepads are not. Linux ARM64 requires a
compatible LibTorch installation through `LIBTORCH_ROOT`; the Jetson setup helper
is retained. ONNX Runtime can be used independently.

## Run

```bash
./build/bin/rl_sim_mujoco go2
./build/bin/rl_sim_mujoco go2w
./build/bin/rl_sim_mujoco go2 scene robot_lab
```

Arguments: `<go2|go2w> [scene] [policy]`. The default scene is `scene`; names omit
`.xml`. Models load from `assets/<robot>/mjcf/<scene>.xml`. The default policy
comes from `policy/<robot>/base.yaml`, under `default_policy`.

| Key | Action |
|---|---|
| `0` | Stand up |
| `1` | Enter policy control after standing |
| `9` | Lie down |
| `P` | Passive damping |
| `W/S`, `A/D`, `Q/E` | Forward/lateral/yaw commands |
| Space | Clear velocity commands |
| `R` | Reset physics |
| Enter | Pause/resume |
| Ctrl+C or close window | Exit |

## Policies and ONNX

`policy/<robot>/base.yaml` contains robot parameters. A selected policy's
`config.yaml` overrides matching keys. The `model_name` extension selects the
backend automatically: `.pt` for TorchScript or `.onnx` for ONNX Runtime.

| Policy | Single frame | Model input | Actions |
|---|---:|---:|---:|
| `go2/himloco` | 45 | 270 (six historical frames) | 12 |
| `go2/robot_lab` | 45 | 45 | 12 |
| `go2w/robot_lab` | 57 | 57 | 16 |

ONNX actors must have one float32 input and one float32 output, normally
`[1, input_dimension]` and `[1, action_count]`. Dynamic batch/feature dimensions
are supported; inference operates on one robot. Keep observation order, history,
joint mapping, and action scaling consistent with training. Go2W's last four
actions represent wheel velocities (`wheel_indices: [12, 13, 14, 15]`).

Python is optional and used only by the offline export tool:

```bash
python3 -m pip install torch onnx
python3 scripts/convert_policy.py policy/go2/robot_lab/policy.pt --input-dim 45
python3 scripts/convert_policy.py policy/go2w/robot_lab/policy.pt --input-dim 57
# HIMLoco needs --input-dim 270
```

Then set the selected configuration's `model_name` to `policy.onnx`.
ONNX models exported directly by a training project also work.

To omit LibTorch completely:

```bash
./setup_deps.sh onnx mujoco assets
BUILD_DIR=build-onnx ./build.sh -DENABLE_TORCH=OFF
./build-onnx/bin/rl_sim_mujoco go2 scene robot_lab
```

Select an existing `.onnx` file in the configuration before running.
For TorchScript only, use `-DENABLE_ONNX=OFF` instead.

## Optional features

```bash
./setup_deps.sh joystick
./build.sh -DENABLE_JOYSTICK=ON
```

USB gamepads use `/dev/input/js0`. `A` stands up, `B` lies down, `RB+D-pad up`
enters policy control, `LB+X` selects passive damping, `RB+Y` resets and `RB+X`
pauses. The left stick commands translation and the right stick commands yaw.

```bash
./setup_deps.sh real
./build.sh -DBUILD_REAL=ON
./build/bin/rl_real_go2 <network_interface>
./build/bin/rl_real_go2 <network_interface> wheel
```

Hardware deployment requires Linux and Unitree SDK2. The default MuJoCo build
does not configure or download this SDK. Add `-DBUILD_MUJOCO=OFF` for hardware
only. The hardware entry point uses `default_policy` from the robot's base
configuration and takes over low-level motion control through Unitree SDK2.
Prepare the robot and operating environment before running it.

`-DENABLE_CSV_LOGGER=ON` records motor data to `policy/<robot>/motor.csv`.

## Layout and options

```text
src/             MuJoCo and optional hardware entry points
include/         Robot interfaces and shared MuJoCo I/O
core/            Inference, observations, loops and math
fsm_robot/       Shared Go2/Go2W state machine
policy/          Configurations and pretrained models for two robots
assets/          Downloaded MJCF, meshes and textures
third_party/     MuJoCo Simulate UI sources
scripts/         Dependency setup and offline policy export
tests/           Observation, policy, inference and model checks
library/         Downloaded dependencies, ignored by Git
```

| CMake option | Default |
|---|---|
| `BUILD_MUJOCO` | ON |
| `BUILD_REAL` | OFF |
| `ENABLE_TORCH` | ON |
| `ENABLE_ONNX` | ON |
| `ENABLE_JOYSTICK` | OFF |
| `ENABLE_CSV_LOGGER` | OFF |
| `BUILD_TESTING` | ON |

Use `MUJOCO_ROOT`, `LIBTORCH_ROOT`, `ONNXRUNTIME_ROOT`, `UNITREE_SDK2_DIR`, or
`JOYSTICK_DIR` to reuse existing installations. `ASSET_DIR` and `POLICY_DIR`
select absolute model/configuration paths. Pass options directly to `build.sh`.

The upstream Apache-2.0 license and copyright notices are retained.
