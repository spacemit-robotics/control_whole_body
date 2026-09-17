# Whole Body Control Component

## 项目简介

`whole_body` 是机型无关的人形机器人整机硬件控制组件。组件从应用层 YAML
读取电机、IMU、关节映射、耦合和安全参数，对上提供统一关节顺序和 SI 单位的
C 接口，可作为 `humanoid_common` 的实机 driver backend。

本组件负责整机关节硬件抽象与安全执行，不实现动力学 Whole-Body Control 算法，
也不包含行为状态机或具体机型配置。

## 功能特性

支持：

- 聚合 SocketCAN 电机与 UART IMU。
- direct 和 parallel-ankle 关节映射。
- HYBRID、位置、速度和力矩四种执行器模式。
- motor、software 和 split 三种关节阻抗执行方式。
- 位置、速度、力矩、KP/KD 命令及关节、IMU 状态反馈。
- read-only / disabled 启动门控、反馈超时、命令 watchdog 和 SAFETY 锁存。
- 物理电机、虚拟关节、IMU 和运行健康状态诊断。
- 命令或失能发送失败时，通过 `whole_body_last_error()` 报告失败电机名称、驱动、
  总线与设备、命令和反馈 ID、操作、控制模式、驱动返回值及系统错误。
  一批命令中的全部电机仍会尝试发送；多个失败分别列出。
  驱动未提供系统错误时标记为 `errno=unavailable`。
- 使用 fake 设备完成配置、映射和安全逻辑的离线测试。

不包含：

- 电机或 IMU 的具体通信协议，相关实现由 `motor` 和 `imu` 组件提供。
- 固定机型名称、关节数量、CAN ID、极性、零位或限位。
- 行为决策、轨迹生成、RL 推理或动力学 WBC。

## 快速开始

### 环境准备

- CMake 3.16 或更高版本
- 支持 C++17 的编译器
- `libyaml-cpp-dev`
- 已构建并安装到 SDK staging 目录的 `motor` 和 `imu` 组件

### 构建编译

在 SDK 根目录执行：

```bash
source build/envsetup.sh
cd components/control/whole_body
mm
```

构建产物安装到：

```text
output/staging/lib/libwhole_body.so
output/staging/include/whole_body.h
```

### 离线测试

在 SDK 根目录执行：

```bash
bash components/control/whole_body/tests/test_offline_contract.sh
```

该测试只使用 fake 设备，不访问 CAN、串口或真实机器人。

### 运行示例

`whole_body` 是共享库，不提供独立控制进程。应用层主配置通过以下字段选择后端：

```yaml
driver:
  backend: whole_body

whole_body:
  config_file: hardware.yaml
```

完成对应机型的硬件 YAML 后，使用该机型的 `run_driver_*.sh` 启动
`driver_runtime`。首次接入硬件时应使用只读模式：

```yaml
whole_body:
  startup_mode: read_only
  allow_actuation: false
```

确认所有电机和 IMU 反馈、关节顺序、零位及方向正确后，再切换到
`startup_mode: disabled` 并由上层 FSM 显式使能。

## 详细使用

### 公共 API

公共头文件为 `include/whole_body.h`。

| 接口 | 说明 |
| --- | --- |
| `whole_body_create()` | 读取应用主配置和硬件配置并创建设备 |
| `whole_body_init()` | 初始化电机、IMU 和运行状态 |
| `whole_body_read()` | 读取统一关节状态和机体 IMU 状态 |
| `whole_body_write()` | 下发统一关节命令 |
| `whole_body_tick()` | 执行命令 watchdog 检查 |
| `whole_body_set_mode()` | 切换 POWER_OFF、DAMP、HOME、ZERO、RL 或 SAFETY |
| `whole_body_get_health()` | 获取读写周期、watchdog 和健康状态 |
| `whole_body_get_diagnostics()` | 获取物理电机、虚拟关节和 IMU 调试快照 |
| `whole_body_get_motor_command_diagnostics()` | 获取映射及限幅后的物理电机命令快照 |
| `whole_body_get_diagnostics_v2()` | 获取含接收时间、错误分类和耦合指标的扩展快照 |
| `whole_body_get_motor_command_diagnostics_v2()` | 获取含估算力矩和变化率的扩展命令快照 |
| `whole_body_last_error()` | 获取最近一次错误说明 |
| `whole_body_destroy()` | 关闭设备并释放资源 |

最小生命周期：

```c
#include <stdio.h>

#include "whole_body.h"

int main(void) {
    struct whole_body_dev *dev = NULL;
    struct whole_body_state state = {0};

    if (whole_body_create("config/robot.yaml", &dev) != WHOLE_BODY_OK)
        return 1;
    if (whole_body_init(dev) != WHOLE_BODY_OK) {
        whole_body_destroy(dev);
        return 1;
    }

    if (whole_body_read(dev, &state) != WHOLE_BODY_OK)
        fprintf(stderr, "%s\n", whole_body_last_error(dev));

    whole_body_destroy(dev);
    return 0;
}
```

实际控制循环还必须按 `whole_body_get_cycle_s()` 返回的周期调用 read、write 和
tick，并检查每个接口的返回值。

### 配置分层

应用主配置定义机型的 canonical 关节顺序，并引用独立硬件配置：

```yaml
robot_base:
  name: robot_name
  num_dof: 2
  joint_names: [joint_0, joint_1]

whole_body:
  config_file: hardware.yaml
```

硬件配置包含：

- `buses`：总线名称、类型、设备和速率。
- `motors`：驱动、型号、总线、CAN ID、极性、零位、可选位置周期、错误分类、命令包络和
  `driver_options`。
- `joints`：关节映射、位置/速度/力矩限制和阻抗模式。
- `couplings`：并联机构的几何参数和数值求解限制。
- `imu`：驱动、设备、波特率、安装矩阵和零偏。

`joints` 的数量及顺序必须与 `robot_base.joint_names` 完全一致。
当电机位置的等价分支相差固定周期时，可在对应电机配置 `position_period`。组件以
`zero_offset` 为首次反馈的分支参考，并在反馈跨周期时保持关节坐标连续；反馈换算和
位置命令反算使用同一运行期分支。未配置时保持原始多圈坐标语义。
`driver_options` 由组件展开为键值项后交给具体外设驱动解析，`whole_body`
不识别驱动私有协议。完整格式可参考 `tests/data/main.yaml` 和
`tests/data/hardware.yaml`。

电机级安全项放在对应 `motors` 条目中：

```yaml
non_fatal_error_codes: [0x04]
command_limits:
  kp_max: 500.0
  kd_max: 5.0
  estimated_torque_max: 90.0
  position_rate_max: 0.0
  velocity_rate_max: 0.0
  torque_rate_max: 0.0
```

`non_fatal_error_codes` 按完整错误码匹配，只改变错误分类，不修改驱动返回的原始值。
列表内错误保留为 warning，其他非零错误触发 SAFETY。`command_limits` 在关节映射之后、协议编码之前检查
物理电机命令。HYBRID 模式的 `estimated_torque` 按
`torque + kp * (target_position - position) + kd * (target_velocity - velocity)` 计算，
TORQUE 模式使用直接力矩目标；位置和速度模式无法从通用接口推导驱动器内部力矩，诊断值为
`NaN`，不执行该项门限。位置、速度和力矩变化率也只检查当前模式实际使用的命令字段。
各上限为 `0` 时表示未启用，不能用未经实机验证的猜测值代替标定结果。

### 行为状态与执行器模式

`whole_body_mode` 表示整机行为和安全状态：

| 状态 | 语义 |
| --- | --- |
| `POWER_OFF` | 维持电机失能 |
| `DAMP` | 强制 HYBRID，`kp=0`，仅保留受限阻尼 |
| `HOME` / `ZERO` / `RL` | 接受上层关节目标并执行安全检查 |
| `SAFETY` | 安全衰减或立即失能，并锁存故障 |

`whole_body_actuation_mode` 表示电机如何解释命令字段：

| 模式 | 语义 |
| --- | --- |
| `HYBRID` | 位置/速度 PD 与前馈力矩叠加 |
| `POSITION` | 位置为主目标 |
| `VELOCITY` | 速度为主目标 |
| `TORQUE` | 力矩为直接目标 |

行为状态与执行器模式相互独立，但 DAMP 会覆盖上层执行器模式以保证阻尼行为。

### 阻抗与关节映射

`impedance.mode` 决定 PD 增益的执行位置：

- `motor`：KP/KD 原样交给电机驱动。
- `software`：在关节空间计算完整 PD 力矩，电机侧 KP/KD 为 0。
- `split`：电机执行硬件允许范围内的 KP/KD，剩余等效 PD 力矩作为前馈力矩。

`split` 的 `motor_kp_max` 和 `motor_kd_max` 必须由应用层硬件配置提供。
`software` 和 `split` 在发送使能命令前必须获得完整反馈。

`direct` 映射由一个物理电机对应一个 canonical 关节。
`parallel_ankle` 映射由两个物理电机共同实现两个虚拟关节，并固定使用
`software` 阻抗；组件在关节空间计算 PD 力矩，再通过当前姿态的
`J^-T` 映射为电机力矩。

并联映射可配置 `jacobian_condition_limit` 和 `torque_amplification_limit`。组件在每次
反馈解算后计算 Jacobian 条件数和最坏方向的力矩放大倍数；任一已启用阈值超限都会
进入 SAFETY。阈值为 `0` 时只记录指标，不执行门控。

### 安全与诊断

`startup_mode: read_only` 只读取反馈，拒绝执行器命令。
`startup_mode: disabled` 在初始化后发送真实协议失能帧，并在 POWER_OFF 下持续
维持失能；只有上层明确进入 DAMP、HOME、ZERO 或 RL 后才允许使能。

以下情况会触发安全处理：

- 反馈未到达或超过 `feedback_timeout_s`。
- 控制命令超过 `command_timeout_s`。
- 命令包含非有限值或超出配置限制。
- 关节映射、外设读写或硬件状态异常。

SAFETY 锁存后，仅切换到 POWER_OFF 才能清除。这是 `whole_body` 进程内的硬件执行
保护；上层 common 会独立保存跨进程故障锁存，即使本地锁存已在 POWER_OFF 清除，
上层仍需在故障条件消失后由操作者确认，才能再次上电。反馈超时返回
`WHOLE_BODY_ERR_TIMEOUT`，设备主动报告的致命错误返回 `WHOLE_BODY_ERR_DEVICE`。
锁存期间重复发送 `SAFETY` 失能命令只会维持失能，不会清除或覆盖最初故障。
原有 `whole_body_get_diagnostics()` 和 `whole_body_get_motor_command_diagnostics()` 保持
初始版结构布局，已有二进制调用方可继续使用。扩展诊断使用对应的 `v2` 接口：
`whole_body_get_diagnostics_v2()` 可读取每台物理电机的原始值、极性/零偏校准值、
反馈时间、时间来源、年龄以及 raw/warning/fatal 错误码；IMU 同时保留传感器采样时间和主机
接收时间，并提供有效帧、CRC、解码、重同步和缓冲溢出累计计数。整机
`feedback_window_s` 表示本次拼帧所用各设备反馈时间的跨度。诊断还包含映射后的
虚拟关节与并联机构数值指标；
`whole_body_get_motor_command_diagnostics_v2()` 提供映射/限幅后、协议编码前的最近一次
电机命令及其估算力矩和变化率。两个接口都不会发送控制命令。
电机驱动提供真实硬件接收时间时，诊断来源为 `HARDWARE_RECEIVE`，可准确检测缓存状态
是否过期。旧驱动未提供该时间时，组件保留兼容路径并使用成功读取 API 的完成时间，来源
明确标为 `READ_COMPLETION`。硬件机型可配置 `require_motor_receive_timestamps: true` 拒绝
兼容时间，避免通信中断后把重复读取的缓存值误判为新反馈。

## 常见问题

| 现象 | 处理 |
| --- | --- |
| CMake 找不到 `motor` 或 `imu` | 先在对应组件目录执行 `mm`，确认依赖已安装到 `output/staging` |
| `whole_body_create()` 返回 `WHOLE_BODY_ERR_CONFIG` | 检查配置文件路径、必填字段及 `robot_base.joint_names` 与 `joints` 顺序 |
| 写命令返回 `WHOLE_BODY_ERR_READ_ONLY` | 当前为 read-only 启动模式；确认反馈与标定后再由应用配置允许执行 |
| 返回 `WHOLE_BODY_ERR_TIMEOUT` | 检查电机/IMU 反馈频率、控制循环周期和命令 watchdog |
| 进入 SAFETY 后无法继续控制 | 读取 health、diagnostics 和 last_error，排除故障后切换到 POWER_OFF 清除锁存 |

## 版本与发布

| 版本 | 说明 |
| --- | --- |
| 0.1.0 | 初始版本，提供机型无关的整机电机、IMU、关节映射和安全控制接口 |

## 贡献方式

欢迎通过 GitHub Issue 或 Pull Request 提交问题与改进。

## License

本组件使用 Apache-2.0 License，最终以本目录 `LICENSE` 文件为准。
