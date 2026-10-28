# core — 整机执行与状态转换

`WholeBodyCore` 将设备反馈转换成统一关节状态，执行上层命令、关节映射和安全检查。
它不生成 HOME、ZERO 或 RL 目标，也不负责上层行为决策。

## 周期调用

上层按配置周期调用 `read`、`write`、`tick`。反馈经电机极性和零位校准、关节映射、
可选 IMU 关节链变换后返回；关节命令经映射和电机级检查后交给设备管理器。

运行缓冲在初始化阶段分配并复用。日志显示和落盘应使用诊断快照，在控制循环外执行。

## 执行状态与命令模式

| `whole_body_mode` | 执行行为 |
| --- | --- |
| `POWER_OFF` | 维持电机失能 |
| `DAMP` | 强制 HYBRID，KP 为 0，执行受限阻尼 |
| `HOME`、`ZERO`、`RL`、`TRAJECTORY` | 接受上层逐帧关节目标并执行安全检查 |
| `SAFETY` | 执行安全处理并锁存故障 |

HOME/ZERO 插值、RL 推理和轨迹生成均由上层负责。

| `whole_body_actuation_mode` | 命令含义 |
| --- | --- |
| `HYBRID` | 位置/速度 PD 与前馈力矩叠加 |
| `POSITION` | 位置为主目标 |
| `VELOCITY` | 速度为主目标 |
| `TORQUE` | 力矩为直接目标 |

DAMP 会覆盖上层执行器模式。PD 由主机还是电机执行，取决于
[mapping 的阻抗配置](../mapping/README.md#阻抗方式)。

## 启动与反馈

- `startup_mode: read_only` 要求 `allow_actuation: false`，拒绝使能和执行器目标命令。
  外设读取可能发送失能查询帧，因此不能与另一个控制器同时操作总线。
- `startup_mode: disabled` 要求 `allow_actuation: true`，初始化后发送失能帧，
  在 POWER_OFF 下持续维持失能，等待上层显式切换状态。
- 在 `startup_feedback_timeout_s` 内等待全部电机及已启用 IMU 的新鲜反馈；
  未收齐时不接受运动命令。
- 启动窗口到期仍未就绪会报超时。设备主动报告的致命故障、非法反馈立即处理。
  晚到的新反馈可以恢复读取，不会自动清除故障锁存。

接收时间的来源要求见 [devices](../devices/README.md#反馈时间与启动就绪)。

## 通信容错与命令超时

`communication_tolerance_s` 默认 0；启用后表示从有效反馈或连续失败事务计时的总上限，
不是在 `feedback_timeout_s` 之后追加一段等待。配置约束见 [config](../config/README.md)。

窗口内的处理规则：

- 必须已有有效状态和成功发送的命令；已使能和已完成失能的 POWER_OFF 均可容错，启动仍需收齐反馈。
- 仅超时类发送失败可返回 `WHOLE_BODY_RETRY`；设备消失、EIO 等硬错误立即进入安全处理。
- 反馈过期时，读取返回最后有效状态及原时间戳；写入和 tick 重发最后成功发送的电机目标及增益。
  不使用旧观测重新计算软件 PD 或耦合目标。
- 调用方应继续周期调用，不因 RETRY 退出、退避，也不把缓存状态标记为新鲜反馈。
- 通信恢复后接受最新命令。容错期间的 POWER_OFF/失能请求仍立即执行。

超过总窗口、无效反馈或致命电机错误会触发故障锁存。容错不能覆盖电机固件自身的失联保护，
保持旧命令的时间须根据设备和控制策略验证。

`refresh_last_command: true` 用于上层暂无新命令而反馈仍新鲜时，以原关节目标重新计算
PD/耦合命令。默认关闭；刷新不会延长 `command_timeout_s`，控制命令过期仍会触发故障。

## 命令检查与故障恢复

以下情况触发安全处理：

- 启动或运行期反馈超时，控制命令超时。
- 命令、反馈包含非有限值，或超出已配置限制。
- 关节映射失败、数值条件超限、致命电机错误或外设读写硬错误。

电机级 `command_limits` 在关节映射之后、协议编码之前检查实际命令。
HYBRID 的估算总力矩为
`torque + kp * (target_position - position) + kd * (target_velocity - velocity)`；
TORQUE 使用直接力矩目标。POSITION/VELOCITY 的电机内部力矩不可由通用接口推导，
诊断记为 `NaN`，不执行估算力矩检查。变化率检查只使用当前模式有效的字段。

`feedback_limits.velocity_max` 检查电机实测速度绝对值，单位 rad/s。
超限在使能前或使能期间触发全电机失能及锁存；POWER_OFF/只读状态仍可读取该反馈供诊断。

排除故障后，通过 POWER_OFF 清除组件内的 SAFETY 锁存。上层控制服务的故障锁存独立管理，
还需按其确认流程解除。重复发送 SAFETY 只维持失能，不清除或覆盖最初故障。
反馈超时返回 `WHOLE_BODY_ERR_TIMEOUT`，设备致命错误返回 `WHOLE_BODY_ERR_DEVICE`。

## IMU 配置与状态转换

固定安装方向由 `imu` 外设组件按安装矩阵变换。本模块只在配置了活动关节链时，
继续将安装连杆的姿态和角速度转换到基座。

硬件配置示例：

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
    base_joint_chain: []
```

`enabled` 默认 true，启用时需要完整设备、安装矩阵和零偏配置。
若 IMU 安装在活动关节之后，按基座到传感器的顺序配置，例如：

```yaml
base_joint_chain:
  - {joint: joint_0, axis: [0, 0, 1]}
  - {joint: joint_1, axis: [1, 0, 0]}
```

关节名必须来自上层关节列表且不能重复；轴为对应父关节坐标系中的单位向量。
`mounting_matrix` 先将传感器转换到安装连杆坐标，关节链再结合实际关节角及角速度，
扣除连杆相对基座的旋转和角速度。省略关节链或设为 `[]` 时不做此项补偿。
加速度只旋转表达方向，不补偿安装距离和关节运动带来的额外加速度。

没有 IMU 时显式配置 `imu: {enabled: false}`，不再要求其他 IMU 字段。
此时反馈中的四元数为 `[1, 0, 0, 0]`，角速度和加速度为零，均为默认值而非测量值；
上层不可将其用于姿态估计或平衡反馈。时间戳使用主机单调时钟，诊断将 IMU 标为未接收、非新鲜。
电机反馈及命令超时检查保持生效，构建仍依赖 `imu` 组件。

## 诊断

扩展状态快照包含物理电机原始值、校准值、反馈时间及年龄、错误分类、虚拟关节和耦合指标。
IMU 诊断包含采样/接收时间、有效帧、CRC、解码、重同步及溢出累计计数。
`feedback_window_s` 表示本次状态所用各设备反馈接收时间的跨度。

扩展命令快照包含映射及限幅后、协议编码前的电机命令、估算力矩和变化率。
基础快照与扩展快照分别通过公共 API 获取，见 [api](../api/README.md)。

## 源码与测试

| 文件 | 职责 |
| --- | --- |
| `whole_body_core.*` | 整机执行、命令检查、故障恢复和诊断 |
| `imu_chain.*` | 活动关节链 IMU 姿态及角速度变换 |

回归测试见 [test_core.cpp](../../tests/test_core.cpp) 和
[test_imu_chain.cpp](../../tests/test_imu_chain.cpp)，使用合成反馈和假设备。
