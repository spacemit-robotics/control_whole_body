# Whole Body

`whole_body` 是人形机器人整机关节硬件后端。组件从应用层 YAML 加载电机、IMU、
关节映射、耦合和安全参数，对上提供固定的 canonical 关节顺序和 SI 单位。

当前实现支持：

- SocketCAN 电机设备与 UART IMU 设备聚合。
- direct 和 parallel-ankle 关节映射。
- HYBRID、位置、速度和力矩四种执行器命令模式。
- 位置、速度、力矩、KP/KD 命令与反馈。
- read-only/disabled 启动门控、有限值检查、反馈超时和本地 watchdog。
- 超时、非法反馈、映射失败和硬件故障触发 SAFETY 锁存，仅 POWER_OFF 可清除。
- dummy 注入的纯离线配置、映射和安全测试。

本组件不包含机型名称或固定关节拓扑，也不实现动力学 WBC 算法。

每个 motor 的总线、command/feedback ID、方向和零位属于通用拓扑字段；其余协议参数
放在 YAML 的 `driver_options`。`whole_body` 仅递归展开为键值项并交给 motor driver，
不识别达妙、Encos 等具体驱动名，也不包含驱动私有配置头文件。

公共状态输出包含关节位置、速度、力矩、温度、错误码以及 IMU 姿态、角速度和加速度；
公共命令包含整条命令共享的 `actuation_mode`、位置、速度、力矩和 KP/KD。
`whole_body_get_diagnostics()` 另提供只读调试快照，包含每台物理电机的原始值、
极性/零偏校准值、总线与 CAN ID、反馈年龄，以及映射后的虚拟关节和 IMU 状态。
该接口不发送命令，也不改变安全状态。
`whole_body_mode` 表示 POWER_OFF/DAMP/HOME/ZERO/RL/SAFETY 行为状态，
`whole_body_actuation_mode` 表示电机如何解释命令字段：

- `HYBRID`：力矩字段是与位置/速度 PD 叠加的前馈力矩。
- `POSITION`：位置字段是主目标。
- `VELOCITY`：速度字段是主目标。
- `TORQUE`：力矩字段是直接目标力矩。

DAMP 行为状态会强制使用 HYBRID（`kp=0`、受限 `kd`），不执行上层传入的
其他执行器模式。

关节的 `impedance.mode` 决定 PD 增益在哪里执行：

- `motor`：KP/KD 原样交给电机驱动，保持原有行为。
- `software`：在关节空间计算完整 PD 力矩，电机命令中的 KP/KD 为 0。
- `split`：电机执行 YAML 限定范围内的 KP/KD，超出的等效 PD 力矩作为前馈力矩发送。

`split` 的 `motor_kp_max`、`motor_kd_max` 是具体关节的硬件协议上限，必须由应用层
硬件 YAML 提供；组件不根据机型或电机名称硬编码。`software` 和 `split` 发送使能命令
前必须至少成功读取一次完整反馈。

parallel-ankle 等耦合关节固定使用 `software`：`whole_body` 在关节空间计算 PD 力矩，
再按当前姿态的 `J^-T` 映射为电机力矩。
`position_limit` 约束实际关节反馈；耦合关节的 PD 参考位置允许在机构
`ankle_limit` 内变化，零电机增益的命令位置使用当前姿态对应的电机位置。

FSM 进入 SAFETY 后，可在 `enable=true` 阶段继续发送逐步衰减的 HYBRID 命令；
衰减完成后的 `enable=false`、底层故障或直接调用 `whole_body_set_mode(SAFETY)`
都会立即发送失能帧并锁存故障。

启动门控由机型硬件 YAML 显式选择：

```yaml
whole_body:
  startup_mode: disabled       # 或 read_only
  allow_actuation: true        # disabled 必须为 true；read_only 必须为 false
  startup_feedback_timeout_s: 1.5
```

`disabled` 会在设备初始化后立即发送真实协议失能帧，并在 POWER_OFF 下持续维持
IDLE；只有上层 FSM 明确进入 DAMP/HOME/ZERO/RL 后才允许电机使能。首次反馈使用
`startup_feedback_timeout_s`，已有反馈变陈旧后使用更短的 `feedback_timeout_s`。

离线测试只使用 fake 设备，不访问 CAN、串口或其他真实硬件：

```bash
bash tests/test_offline_contract.sh
```
