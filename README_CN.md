# rl_sar：Go2 / Go2W + MuJoCo

本仓库从 [rl_sar](https://github.com/fan-ziqi/rl_sar) 精简而来，使用 C++17
在 MuJoCo 中运行 Unitree Go2 和 Go2W 的强化学习策略。
保留 LibTorch/TorchScript 和 ONNX Runtime 两种推理后端，支持历史观测和轮子速度控制。
默认构建不需要 ROS、ROS2、Gazebo、Python 开发库或厂商机器人 SDK。
Go2/Go2W 实机部署和 Linux USB 手柄作为可选功能保留。

[English](README.md)

## 安装与构建

Ubuntu 的基础依赖：

```bash
sudo apt install cmake g++ libyaml-cpp-dev libtbb-dev libglfw3-dev git curl wget unzip
./setup_deps.sh          # 下载两种推理库、MuJoCo 3.2.7，以及 Go2/Go2W 资源
./build.sh              # 普通 CMake 构建，默认 Release
ctest --test-dir build --output-on-failure
```

不再需要递归拉取 Git 子模块。依赖下载与编译分开，`build.sh` 不访问网络。
MuJoCo 的界面代码与 3.2.7 配套，使用其他版本的头文件会在配置阶段报错。
编译后的程序位于 `build/bin/`。默认并行数为 2，可设置 `BUILD_JOBS=4`。

macOS 仿真可使用 `brew install cmake yaml-cpp tbb glfw`；不支持实机和 Linux USB 手柄。
Linux ARM64 的 LibTorch 需要匹配平台的安装；可通过 `LIBTORCH_ROOT` 指定路径。
Jetson 的下载脚本保留其 PyTorch 安装流程。ONNX Runtime 可独立使用。

## 运行仿真

```bash
./build/bin/rl_sim_mujoco go2                  # scene + himloco
./build/bin/rl_sim_mujoco go2w                 # scene + robot_lab
./build/bin/rl_sim_mujoco go2 scene robot_lab   # 指定场景和策略
```

参数格式为 `<go2|go2w> [scene] [policy]`。场景名不带 `.xml` 后缀。
场景从 `assets/<robot>/mjcf/<scene>.xml` 加载。
默认策略由 `policy/<robot>/base.yaml` 的 `default_policy` 指定。

| 按键 | 功能 |
|---|---|
| `0` | 起立到默认姿态 |
| `1` | 进入策略控制，起立完成后使用 |
| `9` | 趴下 |
| `P` | 被动阻尼模式 |
| `W/S`、`A/D`、`Q/E` | 前后、左右、转向速度 |
| 空格 | 速度指令清零 |
| `R` | 重置 MuJoCo 物理状态 |
| Enter | 暂停/继续仿真 |
| Ctrl+C 或关闭窗口 | 退出 |

## 策略与 ONNX

机器人基础参数放在 `policy/<robot>/base.yaml`；策略参数放在
`policy/<robot>/<policy>/config.yaml`。后者覆盖前者的同名参数。
模型文件名由 `model_name` 指定，后端根据 `.pt` / `.onnx` 自动选择。

| 策略目录 | 单帧观测 | 模型输入 | 动作数 |
|---|---:|---:|---:|
| `go2/himloco` | 45 | 270（6 帧历史） | 12 |
| `go2/robot_lab` | 45 | 45 | 12 |
| `go2w/robot_lab` | 57 | 57 | 16 |

ONNX 模型应有一个 float32 输入和一个 float32 输出，通常为
`[1, 模型输入维度]` 和 `[1, 动作数]`。支持动态 batch/特征维度；单次只运行一台机器人。
更换策略时，观测顺序、历史帧、关节映射和动作缩放必须与训练配置一致。
Go2W 的最后四个动作是轮子目标速度，保留 `wheel_indices: [12, 13, 14, 15]`。

若需要从现有 TorchScript 导出 ONNX，Python 只用于这个离线工具：

```bash
python3 -m pip install torch onnx
python3 scripts/convert_policy.py policy/go2/robot_lab/policy.pt --input-dim 45
python3 scripts/convert_policy.py policy/go2w/robot_lab/policy.pt --input-dim 57
# HIMLoco 的 --input-dim 是 270
```

然后把对应 `config.yaml` 的 `model_name` 改为 `policy.onnx`。
也可以直接使用训练程序导出的 ONNX，不必经过本项目的转换工具。

仅使用 ONNX Runtime 时，可以省去 LibTorch：

```bash
./setup_deps.sh onnx mujoco assets
BUILD_DIR=build-onnx ./build.sh -DENABLE_TORCH=OFF
./build-onnx/bin/rl_sim_mujoco go2 scene robot_lab
```

运行前要把所选策略的 `model_name` 改成已存在的 `.onnx` 文件。
反过来，仅使用 TorchScript 可以传入 `-DENABLE_ONNX=OFF`。

## 可选功能

Linux USB 手柄默认关闭，键盘控制始终可用：

```bash
./setup_deps.sh joystick
./build.sh -DENABLE_JOYSTICK=ON
```

程序读取 `/dev/input/js0`。基础按键为 `A` 起立、`B` 趴下、`RB+方向上` 进入策略、
`LB+X` 被动模式、`RB+Y` 重置、`RB+X` 暂停，左摇杆平移、右摇杆转向。

Go2/Go2W 实机程序默认不编译，MuJoCo 构建不会查找或下载 Unitree SDK2：

```bash
./setup_deps.sh real
./build.sh -DBUILD_REAL=ON
./build/bin/rl_real_go2 <network_interface>         # Go2
./build/bin/rl_real_go2 <network_interface> wheel   # Go2W
```

只编译实机可加 `-DBUILD_MUJOCO=OFF`。实机策略同样由 `base.yaml/default_policy` 选择。
实机程序会通过 Unitree SDK2 接管底层运动控制；运行前确保机器人和操作环境准备就绪。

CSV 日志可通过 `-DENABLE_CSV_LOGGER=ON` 开启，输出在 `policy/<robot>/motor.csv`。

## 目录与构建选项

```text
src/             MuJoCo 与可选实机入口
include/         对应接口
core/            策略推理、观测、控制循环和数学工具
fsm_robot/       Go2/Go2W 共用状态机
policy/          两款机器人的配置和预训练模型
assets/          下载的 MJCF、网格与纹理
third_party/     MuJoCo Simulate 界面源码
scripts/         依赖下载与离线模型转换
tests/          观测、策略接口、推理和模型加载检查
library/         下载的依赖，Git 忽略
```

| CMake 选项 | 默认值 | 作用 |
|---|---|---|
| `BUILD_MUJOCO` | ON | 构建仿真程序 |
| `BUILD_REAL` | OFF | 构建可选实机程序 |
| `ENABLE_TORCH` | ON | TorchScript 推理 |
| `ENABLE_ONNX` | ON | ONNX 推理 |
| `ENABLE_JOYSTICK` | OFF | Linux USB 手柄 |
| `ENABLE_CSV_LOGGER` | OFF | CSV 日志 |
| `BUILD_TESTING` | ON | 构建 CTest 检查 |

已有依赖可通过 `MUJOCO_ROOT`、`LIBTORCH_ROOT`、`ONNXRUNTIME_ROOT`、
`UNITREE_SDK2_DIR` 和 `JOYSTICK_DIR` 指定。模型与策略目录可分别通过
`ASSET_DIR` 和 `POLICY_DIR` 指定绝对路径。示例：

```bash
./build.sh -DMUJOCO_ROOT=/path/to/mujoco -DONNXRUNTIME_ROOT=/path/to/onnxruntime
```

本项目保留上游 Apache-2.0 许可证和相关版权声明。
