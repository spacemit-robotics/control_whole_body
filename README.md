# Whole Body Control Component

## 项目简介

`whole_body` 是机型无关的人形机器人整机硬件控制组件。组件从应用层 YAML
读取电机、IMU、关节映射、耦合和安全参数，对上提供统一关节顺序和 SI 单位的
C 接口，可作为 `humanoid_common` 的实机 driver backend。

本组件负责整机关节硬件抽象与安全执行，不实现动力学 Whole-Body Control 算法，
也不包含行为状态机或具体机型配置。

## 功能特性

- 聚合 SocketCAN/UART 电机与可选 UART IMU。
- 支持单电机关节、并联连杆及二维表格映射。
- 支持电机 PD、主机 PD、混合分担及耦合比例力矩投影。
- 提供位置、速度、力矩、KP/KD 命令和统一关节、IMU 状态反馈。
- 支持固定 IMU 安装方向配置和活动关节链补偿。
- 提供启动反馈检查、通信容错、命令超时、故障锁存及诊断。
- 提供不访问真实硬件的离线测试。

具体通信协议由 `motor` 和 `imu` 组件实现；行为决策、轨迹生成及 RL 推理由上层负责。

## 快速开始

### 环境准备

- CMake 3.16 或更高版本、支持 C++17 的编译器。
- `libyaml-cpp-dev`。
- 已构建并安装到 SDK staging 目录的 `motor` 和 `imu` 组件。

Delaunator 头文件由 `cmake/FindDelaunator.cmake` 自动获取固定版本并校验，
缓存到 `~/.cache/thirdparty/delaunator`，随本组件编译，无额外运行时库。
可通过 `SROBOTIS_THIRDPARTY_CACHE` 指定缓存目录，或通过 `DELAUNATOR_DIR`
指定已有源码/头文件目录。离线构建使用 CMake 选项
`SROBOTIS_THIRDPARTY_FETCH_OFF=ON`，并提前准备匹配依赖。

### 构建编译

在 SDK 根目录执行：

```bash
source build/envsetup.sh
cd components/control/whole_body
mm
```

构建产物：

```text
output/staging/lib/libwhole_body.so
output/staging/include/whole_body.h
output/staging/include/whole_body_transport.h
```

### 运行示例

本组件是共享库，不提供独立控制进程。使用 `humanoid_common` 时，在应用主配置中选择后端：

```yaml
driver:
  backend: whole_body

whole_body:
  config_file: hardware.yaml
```

在独立的 `hardware.yaml` 中填写硬件配置，首次接入先设置：

```yaml
whole_body:
  startup_mode: read_only
  allow_actuation: false
```

随后用机型的 `run_driver_*.sh` 启动 `driver_runtime`。
只读模式可能发送失能查询帧，不可与其他控制器同时操作总线。
确认反馈、关节顺序、零位及方向后，将硬件配置改为
`startup_mode: disabled`、`allow_actuation: true`，由上层显式使能。

完整配置格式见 [配置模块](src/config/README.md)；直接调用共享库的示例见
[公共接口模块](src/api/README.md)。

### 离线测试

在 SDK 根目录执行：

```bash
bash components/control/whole_body/tests/test_offline_contract.sh
```

测试使用合成数据和假设备，不访问 CAN、串口或真实机器人。

## 详细使用

### 配置与功能选择

| 接入需求 | 配置方式 | 说明 |
| --- | --- | --- |
| 定义硬件和关节顺序 | `robot_base.joint_names`、独立硬件 YAML | [配置模块](src/config/README.md) |
| 接入电机及总线 | `buses`、`motors`、`driver_options` | [设备管理](src/devices/README.md) |
| 选择关节映射及 PD 执行方式 | `joints`、`couplings`、`impedance` | [关节映射](src/mapping/README.md) |
| 配置 IMU 方向、可选关节链或禁用 IMU | `imu` | [IMU 配置与状态转换](src/core/README.md#imu-配置与状态转换) |
| 配置超时、容错和故障恢复 | 周期、反馈及命令期限 | [整机执行](src/core/README.md) |

机型的设备路径、零位、极性、增益和限位都放在上层硬件 YAML，不写入组件。

### 接口调用

基本流程为创建句柄、初始化、周期读写、销毁：

```text
create → init → 按 cycle_s 循环 read / write / tick → destroy
```

通过 `whole_body_set_mode()` 设置硬件执行状态；HOME、ZERO、RL 等目标由上层生成。
返回 `WHOLE_BODY_RETRY` 时继续周期调用，不将缓存状态当作新鲜反馈。
出现错误时读取 `whole_body_last_error()`、health 和 diagnostics，再按故障恢复流程处理。
完整接口表、数据单位和调用示例见 [api](src/api/README.md)。

### 目录结构

源码按职责分目录，统一构建为 `libwhole_body.so`；公开头文件保留在 `include/`。

| 路径 | 职责 |
| --- | --- |
| `include/` | 对外 C 接口和数据结构 |
| `src/api/` | 句柄生命周期及公共接口封装 |
| `src/config/` | YAML 解析和配置校验 |
| `src/devices/` | 电机、IMU 设备管理与收发 |
| `src/core/` | 整机执行、IMU 变换、安全检查和诊断 |
| `src/mapping/` | 并联几何、表格映射和阻抗转换 |
| `cmake/` | 第三方依赖发现与缓存规则 |
| `tests/` | 离线功能和异常路径测试 |

## 常见问题

| 现象 | 处理 |
| --- | --- |
| CMake 找不到 `motor` 或 `imu` | 先构建依赖，确认已安装到 `output/staging` |
| 创建返回 `WHOLE_BODY_ERR_CONFIG` | 检查文件路径、必填字段、关节顺序和设备引用 |
| 写命令返回 `WHOLE_BODY_ERR_READ_ONLY` | 当前禁止执行；确认硬件后调整启动配置 |
| 返回 `WHOLE_BODY_RETRY` | 继续周期调用，查看反馈年龄及通信诊断 |
| 返回 `WHOLE_BODY_ERR_TIMEOUT` | 检查反馈频率、调用周期和超时配置 |
| 故障后无法重新使能 | 排除故障，切 POWER_OFF 清除组件锁存，并完成上层故障确认 |

## 版本与发布

| 版本 | 说明 |
| --- | --- |
| 0.1.0 | 提供机型无关的整机电机、IMU、关节映射和安全控制接口 |

## 贡献方式

欢迎通过 GitHub Issue 或 Pull Request 提交问题与改进。

## License

本组件使用 Apache-2.0 License，最终以本目录 `LICENSE` 文件为准。
