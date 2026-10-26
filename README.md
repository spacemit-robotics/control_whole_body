# Whole Body Control Component

## 项目简介

`whole_body` 是机型无关的人形机器人整机硬件控制组件。组件从应用层 YAML
读取电机、IMU、关节映射、耦合和安全参数，对上提供统一关节顺序和 SI 单位的
C 接口，可作为 `humanoid_common` 的实机 driver backend。

本组件负责整机关节硬件抽象与安全执行，不实现动力学 Whole-Body Control 算法，
也不包含行为状态机或具体机型配置。

## 功能特性

支持：

- 聚合 SocketCAN/UART 电机与 可选 UART IMU。
- direct、parallel_ankle 几何及 table_2d 表格关节映射。
- 可配置的活动关节链 IMU 到基座姿态和角速度变换。
- HYBRID、位置、速度和力矩四种执行器模式。
- motor、motor_projected、software 和 split 四种关节阻抗执行方式。
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
- 首次构建需使用 Git 获取 Delaunator 头文件依赖，或提前准备离线缓存

### 构建编译

在 SDK 根目录执行：

```bash
source build/envsetup.sh
cd components/control/whole_body
mm
```

`cmake/FindDelaunator.cmake` 自动查找 Delaunator；缺失时通过
`FetchThirdParty.cmake` 获取固定版本源码到 `~/.cache/thirdparty/delaunator`，
并校验头文件 SHA256。该依赖仅包含头文件，随 `whole_body` 编译，无额外运行时库。
环境变量 `SROBOTIS_THIRDPARTY_CACHE` 可指定公共缓存目录；CMake 变量或环境变量
`DELAUNATOR_DIR` 可指定已有源码或头文件目录。离线构建时设置 CMake 选项
`SROBOTIS_THIRDPARTY_FETCH_OFF=ON`，并提前准备匹配版本的依赖。

构建产物安装到：

```text
output/staging/lib/libwhole_body.so
output/staging/include/whole_body.h
output/staging/include/whole_body_transport.h
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
| `whole_body_has_imu()` | 只读查询配置是否启用 IMU，返回 1/0，无效句柄返回负值 |
| `whole_body_init()` | 初始化电机、已启用的 IMU 和运行状态 |
| `whole_body_read()` | 读取统一关节状态和机体 IMU 状态 |
| `whole_body_write()` | 下发统一关节命令 |
| `whole_body_tick()` | 执行命令 watchdog 检查 |
| `whole_body_set_mode()` | 切换 POWER_OFF、DAMP、HOME、ZERO、RL、TRAJECTORY 或 SAFETY |
| `whole_body_get_health()` | 获取读写周期、watchdog 和健康状态 |
| `whole_body_get_diagnostics()` | 获取物理电机、虚拟关节和 IMU 调试快照 |
| `whole_body_get_motor_command_diagnostics()` | 获取映射及限幅后的物理电机命令快照 |
| `whole_body_get_diagnostics_v2()` | 获取含接收时间、错误分类和耦合指标的扩展快照 |
| `whole_body_get_motor_command_diagnostics_v2()` | 获取含估算力矩和变化率的扩展命令快照 |
| `whole_body_get_transport_diagnostics()` | 从独立头文件 `whole_body_transport.h` 读取每条总线的通信计数和最近失败事务 |
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

通信诊断使用独立结构，不改变既有 v2 快照布局。初始化时确定每条配置总线的
代表电机；运行期只复制已有计数，不访问串口、不按电机重复采样、不改变超时或
错误处理。未提供通信诊断的驱动不出现在结果中。调用线程需与设备读写串行；
格式化和落盘应由上层后台线程处理。

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
- `couplings`：并联机构的几何参数或表格文件、映射方向和数值求解限制。
- `imu`：启用开关、驱动、设备、波特率、安装矩阵、零偏和可选基座到传感器的活动关节链。

`joints` 的数量及顺序必须与 `robot_base.joint_names` 完全一致。
`position_limit` 始终约束使用位置目标的命令，默认也检查使能期间的实测关节角。
若机型的参考角范围不是实测越界停机阈值，可显式设置 `feedback_position_check: false`；
该设置只关闭这一项反馈角比较，不跳过有限值、映射有效域/奇异点、电机故障或通信超时检查。
不得用它代替尚未完成的标定；应在机型文档中给出保护归属及依据。
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

### 可选 IMU

`imu.enabled` 默认 `true`，省略开关时仍要求完整的原有 IMU 配置：

```yaml
whole_body:
  imu:
    enabled: true
    driver: drv_uart_forsense
    device: /dev/ttyUSB0
    baud: 460800
    mounting_matrix: [1, 0, 0, 0, 1, 0, 0, 0, 1]
    acceleration_bias: [0, 0, 0]
    gyro_bias: [0, 0, 0]
```

没有安装 IMU 时，必须在硬件 YAML 中显式关闭；不能靠缺少配置或读取失败自动降级：

```yaml
whole_body:
  imu:
    enabled: false
```

只有此时才跳过 IMU 其他字段的必填检查、分配、初始化、读取和反馈门控；电机逐台
新鲜度检查、启动超时和命令 watchdog 保持不变。构建时仍依赖 `imu` 组件。
`whole_body_has_imu(dev)` 创建后即可调用，不执行设备读写，也不表示传感器健康。
上层必须据此明确标记“无 IMU”，不能把它展示为健康测量。

无 IMU 时，成功读取的 `state.timestamp_s` 使用主机单调时钟，
`base_quat` 为 `[1, 0, 0, 0]`（w, x, y, z）的数学默认值，`gyro` 和 `acceleration`
为零；这些字段不是测量值，不可用于姿态估计或平衡反馈。
两版诊断中的 IMU `driver/device` 均为空，`feedback_received/feedback_fresh` 均为
`false`，其他 IMU 诊断字段为零。反馈超时详情只排除已禁用的 IMU，仍逐台报告异常电机。
公开 state 和 diagnostic 结构布局不变。

### 行为状态与执行器模式

`whole_body_mode` 表示整机行为和安全状态：

| 状态 | 语义 |
| --- | --- |
| `POWER_OFF` | 维持电机失能 |
| `DAMP` | 强制 HYBRID，`kp=0`，仅保留受限阻尼 |
| `HOME` / `ZERO` / `RL` | 接受上层关节目标并执行安全检查 |
| `TRAJECTORY` | 接受上层轨迹的逐帧关节目标，执行相同映射、限幅和安全检查 |
| `SAFETY` | 安全衰减或立即失能，并锁存故障 |

`WHOLE_BODY_MODE_TRAJECTORY = 6`，既有模式值不变；轨迹生成和插值由上层负责。
TRAJECTORY 不要求关闭 IMU，也不绕过 read-only、启动反馈、故障锁存或命令 watchdog。

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

- `motor`：直驱关节的 KP/KD 原样交给电机；两轴耦合关节使用下述显式对角近似。
- `software`：在关节空间计算完整 PD 力矩，电机侧 KP/KD 为 0。
- `split`：电机执行硬件允许范围内的 KP/KD，剩余等效 PD 力矩作为前馈力矩。
- `motor_projected`：两轴耦合关节的比例力矩投影到电机位置目标，阻尼由电机执行。

`split` 的 `motor_kp_max` 和 `motor_kd_max` 必须由应用层硬件配置提供。
`software` 和 `split` 在发送使能命令前必须获得完整反馈。

可选 `impedance.velocity_filter_time_constant_s` 是主机阻尼速度的一阶低通时间常数
（秒，默认 `0` 关闭）。滤波在反馈接收时间前进时更新，系数为
`1-exp(-dt/tau)`；耦合关节取两电机较早的接收时间，重复读取缓存不重复积分。
首次反馈、故障恢复、时间回退或超过反馈超时的间隔均以当前速度重新初始化。
没有电机时间戳的旧驱动使用本机单调时间。它只改变 `software/split` 的主机
阻尼计算，不改变发布给上层的速度、IMU 关节链、原始诊断或速度保护；电机端
PD 不使用此滤波。`split` 仍用原始速度扣除电机端实际 PD，避免重复叠加。

`direct` 映射由一个物理电机对应一个 canonical 关节。
`parallel_ankle` 映射由两个物理电机共同实现两个虚拟关节，默认使用
`software` 阻抗；组件在关节空间计算 PD 力矩，再通过当前姿态的
`J^-T` 映射为电机力矩。

`table_2d` 使用相同的两电机/两关节接口，默认也是 `software` 阻抗。
对应 coupling 必须设置 `type: table_2d`、`table_file` 和
`table_direction: motor_to_joint` 或 `joint_to_motor`。
路径相对硬件 YAML 解析，方向不能省略。关节和电机的先后顺序由对应数组显式确定，
表输入/输出必须与该顺序、极性和零位一致。

表格直接读取无表头的四列 CSV：`input_0,input_1,output_0,output_1`，角度单位 rad。
输入坐标须组成规则网格，允许两轴步长不同；行序不限，重复坐标、无穷值、
多余列或不规则输入会拒绝加载。缺失格点或 NaN 输出表示不可达点。
可选 `table_output_scale: [1.0, 1.0]` 按配置转换两个输出的符号/比例；
系数必须有限且非零，组件不内置任何机型的左右镜像规则或表文件格式转换脚本。
默认使用双线性插值及其解析导数；`table_interpolation: delaunay` 选择
CSV 原始行序上的 Delaunay 三角平面及解析导数。三角剖分仅在加载阶段执行，
运行期通过规则格子直接定位，不做全表/空间搜索。供应方确定的 CSV 行序必须保留：
共圆格点可能有不止一种合法对角线。两种模式都要求格子的四个角全部有效，
不会越界外推、夹紧输入或重新分配 NaN 点权重。反向映射使用有界数值求解，
失败进入安全路径，不继续输出近似目标。
两种表方向对外均使用 `J = d(motor)/d(joint)`；速度用 `J`，
关节力矩到电机力矩用 `J^-T`，反馈电机力矩到关节力矩用 `J^T`。
`table_2d` 不会用 `ankle_limit` 扩大应用配置的单关节限位。
`motor` 阻抗的位置控制检查两个关节目标组合是否可解及其已配置的雅可比阈值，
失败进入失能 SAFETY 并锁存。`software` 阻抗的位置参考用于计算关节 PD 力矩，
不反算该参考姿态；电机力矩转换使用当前实测姿态的雅可比。
两种模式均保留单轴参考限位、实测姿态/雅可比检查及最终电机命令保护。
电机位置目标反算仅使用缓存和零点两次有界局部求解，不扫描整张表，不覆盖反馈求解的缓存；
所以拒绝可能意味着局部求解未收敛，而不一定是机械不可达。
DAMP、纯速度及纯力矩模式不检查未使用的位置目标，反馈侧门禁仍然有效。
原 `parallel_ankle` 的正反解、速度/力矩转换和指标仍调用原几何实现。

两种耦合映射可在**两个关节上同时**显式配置 `impedance: {mode: motor}`，
不接受两轴阻抗配置不一致。该模式先将目标关节位置反算为
电机位置、用当前雅可比转换目标速度和前馈力矩，再把位置/速度闭环交给电机。
令 `A = J^-1 = d(joint)/d(motor)`，实际下发的每个电机增益为
`diag(A^T diag(Kq) A)`、`diag(A^T diag(Dq) A)`。增益恒为非负；不把非对角项
或基于主机测得速度的 PD 再加回前馈力矩，避免重复 PD 和延迟阻尼闭环。
DAMP 固定为电机 KP=0、速度目标=0、前馈力矩=0，只有转换后的正 KD。
纯力矩模式仍使用 `J^-T` 前馈力矩，KP/KD 为 0。

这是局部对角阻抗近似，**不等价于原虚拟关节空间的完整 PD**，也不保证离散系统稳定。
位置目标仍采用完整运动学逆解。电机命令保护检查变换后的增益和估算总力矩，
超限失能而非静默截断；驱动协议的增益分辨率必须由机型测试覆盖，过小 KD 可能量化为 0。
此模式为显式选用，不改变已有 software/split 配置或其他机型的默认行为。

耦合关节也可显式选用 `impedance: {mode: motor_projected}`。增益、目标速度、
前馈和 DAMP 语义与 `motor` 相同，但位置目标改为
`m_des = m_measured + [J^-T Kq (q_des - q_measured)] / Km`（逐电机除法）。
它把完整的关节比例力矩投影到电机位置目标，保留比例项的交叉轴作用，
不再将关节参考直接做运动学逆解，也不添加主机速度阻尼残差。
电机 KP 小于 `1e-4` 时位置目标保持当前测量值，避免除以近零增益。
该模式可配置 `motor_kp_max` / `motor_kd_max`：变换后的增益先限幅、按电机接口
转成 float，再用实际下发 KP 计算目标位置；超出上限不是命令拒绝。
两项须同时提供，0 或省略代表该模式不设对应上限。具体数值来自机型 YAML。
这与 `command_limits.kp_max/kd_max` 的发包前拒绝语义不同，也与 split 中
`motor_kp_max: 0` 表示电机不分担刚度的语义不同。
非位置模式不使用关节位置参考。该模式只适用于耦合映射，须两轴同时配置；
原有单轴参考限位、反馈检查和电机命令保护仍生效。
采样点的比例力矩等价不意味着完整 PD 等价：电机阻尼仍是变换后矩阵的对角部分，
实际闭环还受时延和协议量化影响。

`impedance.rl_mode` 可选，仅在 RL 状态覆盖 `mode`；其他状态仍使用原执行路径。
耦合 `split` 在关节空间计算完整 PD，再由当前雅可比转换力矩，包含交叉项。
电机端分担的 KP/KD 取完整增益矩阵最小特征值构成的非负对角部分，且受
`motor_kp_max` / `motor_kd_max` 限制；将电机 PD 产生的力矩从主机前馈中扣除，
避免重复叠加。这保证采样点处的总力矩一致，不等于连续时间或实际量化后的严格等价。
设置 `motor_kp_max: 0` 时无需反算参考姿态，位置项由主机计算，电机只分担阻尼。
该选项不改变未配置机型的默认行为，也不放宽反馈、参考角或电机命令保护。

并联映射可配置 `jacobian_condition_limit` 和 `torque_amplification_limit`。组件在每次
反馈解算后计算 Jacobian 条件数和最坏方向的力矩放大倍数；任一已启用阈值超限都会
进入 SAFETY。阈值为 `0` 时只记录指标，不执行门控。

### 活动关节链上的 IMU

如果传感器位于活动关节之后，可在 `imu.base_joint_chain` 中按
基座到传感器的顺序配置 `{joint: 关节名, axis: [x, y, z]}`。
每个轴是对应父关节坐标系中的单位轴；关节名必须存在且不能重复。
`mounting_matrix` 先完成传感器到其安装连杆的固定变换，活动链再转换到基座。
链为空时保持原有固定安装行为。

输出姿态扣除活动链旋转，角速度扣除这些关节相对基座的角速度。
加速度只改变表达坐标系，不补偿安装位置的杆臂加速度；因此并不等于
从连杆 IMU 重建了基座位置处的完整加速度。
`test_imu_chain` 验证不交换旋转轴和相对角速度变换，仍需机型物理标定。

### 安全与诊断

周期内的电机反馈、命令、限幅统计及失能帧复用初始化时分配的缓冲区。

DeviceManager 使用 motor 的可选批量接口收发，并逐电机保留错误码和实际接收时间。
是否能够同时调度多个总线由电机驱动决定；共享层不判断具体机器人或 UART 型号。
不支持批量的驱动回退到原来的单设备接口，启动新鲜度门控和失能全量尝试保持不变。

`startup_mode: read_only` 拒绝使能与执行器目标命令，但读取反馈可能由外设驱动
发送失能查询帧（例如 UART 驱动的 `idle_poll`）。它不是保证线路无发送的被动监听模式，
不能与另一个控制器同时操作总线。
`startup_mode: disabled` 在初始化后发送真实协议失能帧，并在 POWER_OFF 下持续
维持失能；只有上层明确进入 DAMP、HOME、ZERO、RL 或 TRAJECTORY 后才允许使能。
初始化时必须取得晚于本次启动、且均未超过 `feedback_timeout_s` 的所有电机与已启用 IMU 反馈；
在 `startup_feedback_timeout_s` 内未收齐时保持初始化状态和失能，不接受运动命令。
超出启动窗口才报反馈超时；一旦就绪，后续反馈中断按运行期
`feedback_timeout_s` 处理。设备主动报告的故障与非法反馈不等待启动窗口。
首次收齐反馈已超出启动窗口时，也先报告超时；后续新鲜反馈允许恢复读取，
但不会自动清除 SAFETY 锁存。

运行中可通过 `communication_tolerance_s` 开启有限通信容错，省略或 0 保持原行为。
这是从有效反馈/连续失败事务计时的总上限，不是在 `feedback_timeout_s` 后再追加的时间；
配置值必须不小于反馈期限、不大于命令期限，并小于启动期限。调用方必须识别正返回值
`WHOLE_BODY_RETRY`，继续周期调用 read/write/tick，不能当作致命错误或新鲜反馈。

- 已使能或已完成失能的 POWER_OFF 均允许容错，须已有有效状态和已发送命令；启动仍严格握手。
- 仅 ETIMEDOUT 的发送失败在窗口内返回 RETRY；混合 EIO、设备消失等硬错误不容错。
- 反馈超过常规期限但未到容错上限时，read 返回最后有效状态及原时间戳，诊断仍标记
  各端点真实年龄和非新鲜状态；write/tick 重发最后成功发送的电机目标及增益，
  不用旧观测重新计算耦合目标/软件 PD，不增加阻塞等待或慢速退避。
- 新鲜反馈和发送恢复后继续接受最新命令，不进入 POWER_OFF、不要求人工重新使能。
  容错期间 POWER_OFF/失能指令仍立即执行；无效反馈、致命电机错误和超出总窗口仍锁存。

该选项用于短时控制连续性，不代表能覆盖电机固件自身的失联保护或维持任意时长平衡；
机型需验证保持旧电机命令的实际效果。未启用该选项的机型无需变更调用方。

`refresh_last_command: true` 让驱动在上层暂未发来新命令时，按原关节目标和当前
新鲜反馈重新计算 PD/耦合命令。刷新不会延长 `command_timeout_s`；超过期限仍故障。
反馈不新鲜时仍使用有限通信容错，不用旧反馈重算软件 PD。默认关闭。

以下情况会触发安全处理：

- 超过启动窗口仍未收齐新鲜反馈，或运行期反馈超过配置的容错上限
  （未启用容错时仍是 `feedback_timeout_s`）。
- 控制命令超过 `command_timeout_s`。
- 命令包含非有限值或超出配置限制。
- 关节映射、外设读写或硬件状态异常。

物理电机的 `feedback_limits.velocity_max` 设置反馈速度上限（rad/s，0 或省略为不启用）。
该检查使用实际反馈速度绝对值，不是速度目标；使能前检查，并在使能期间每次读取新鲜
反馈后检查。超限立即对所有电机发送 IDLE 并锁存 SAFETY。POWER_OFF/只读时仍能读取
超限数据供诊断。上限由硬件配置提供。

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
是否过期。旧驱动未提供该时间时，诊断仍保留成功读取 API 的完成时间，来源明确标为
`READ_COMPLETION`，但该时间不证明设备收到了新反馈，不能用于启动就绪或反馈保鲜判断。
配置 `require_motor_receive_timestamps: true` 时，缺少电机接收时间立即报配置错误；
否则保持等待，并在启动窗口截止后报反馈超时。IMU 同样只使用组件提供的主机接收时间，
不以读取 API 的完成时间替代。

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
