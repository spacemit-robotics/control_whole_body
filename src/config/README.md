# config — 硬件配置

读取应用主配置和独立硬件 YAML，校验字段、关节顺序、设备引用及参数范围。
加载结果为内部 `RuntimeConfig`，供设备管理和整机执行模块使用。

## 配置入口

应用主配置声明统一关节顺序，并引用硬件文件：

```yaml
robot_base:
  name: robot_name
  num_dof: 2
  joint_names: [joint_0, joint_1]

whole_body:
  config_file: hardware.yaml
```

`config_file` 相对主配置文件所在目录解析；耦合表格路径相对硬件 YAML 解析。
完整字段示例见 [main.yaml](../../tests/data/main.yaml) 和
[hardware.yaml](../../tests/data/hardware.yaml)，其中设备、增益和限位需按实际硬件配置。

## 硬件文件结构

硬件文件的根节点为 `whole_body`：

| 字段 | 内容 |
| --- | --- |
| `buses` | 总线名称、类型、设备和速率 |
| `motors` | 驱动、型号、总线、命令/反馈 ID、极性、零位和电机级限制 |
| `joints` | 统一关节名称、映射方式、关联电机、限位和阻抗方式 |
| `couplings` | 两电机/两关节的几何参数或表格映射配置 |
| `imu` | 启用开关、驱动、设备、安装方向、零偏和活动关节链 |

`joints` 的数量及顺序必须与 `robot_base.joint_names` 一致。名称须唯一，
设备和关节引用须存在；未知字段、非法维度和非有限参数会被拒绝。

## 周期与启动

| 字段 | 要求或默认值 |
| --- | --- |
| `cycle_s` | 必填，调用周期，单位秒 |
| `startup_feedback_timeout_s` | 必填，启动收齐反馈的最大等待时间 |
| `feedback_timeout_s` | 必填，运行期反馈有效期限 |
| `command_timeout_s` | 必填，控制命令有效期限 |
| `startup_mode` | 必填，`read_only` 或 `disabled` |
| `allow_actuation` | 必填；`read_only` 必须为 false，`disabled` 必须为 true |
| `communication_tolerance_s` | 默认 0，关闭额外通信容错 |
| `refresh_last_command` | 默认 false，不在无新命令时重新计算 PD/耦合命令 |
| `require_motor_receive_timestamps` | 默认 false；为 true 时拒绝不提供接收时间的驱动 |

反馈期限和命令期限必须大于控制周期；启动期限必须大于反馈期限。
启用通信容错时，其值须不小于反馈期限、不大于命令期限，并小于启动期限。
容错期间的命令处理及故障恢复见 [core](../core/README.md)。

## 电机与关节

电机的 `driver_options` 按键值项传给外设驱动；本组件不解释驱动私有协议。
具体配置方式及接收时间要求见 [devices](../devices/README.md)。

| 配置项 | 用途 |
| --- | --- |
| `motors[].polarity`、`zero_offset` | 电机反馈与统一坐标之间的方向和零位标定 |
| `motors[].position_period` | 可选、正数；指定电机位置等价分支的周期 |
| `motors[].non_fatal_error_codes` | 按完整错误码匹配的 warning 列表；其他非零错误为致命错误 |
| `motors[].command_limits` | 电机命令的增益、估算力矩和变化率上限；各项 0 或省略表示不启用 |
| `motors[].feedback_limits.velocity_max` | 反馈速度绝对值上限，rad/s；0 或省略表示不启用 |
| `joints[].position_limit` | 位置参考范围，默认也用于使能期间的反馈角检查 |
| `joints[].feedback_position_check` | 默认 true；false 只关闭反馈角与此范围的比较 |
| `joints[].velocity_limit`、`torque_limit` | 关节速度、力矩限制 |
| `joints[].impedance` | PD 执行方式及可选阻尼滤波 |

`position_period` 以 `zero_offset` 为首次反馈的分支参考，随后保持跨周期的关节坐标连续；
反馈换算和位置命令反算使用同一分支。省略时使用原始多圈坐标。

关闭反馈角范围检查后，有限值、映射有效域、奇异点、电机故障和通信检查仍然生效。
硬件参数应来自标定及设备规格，不能用关闭检查代替标定。

映射、阻抗及表格字段见 [mapping](../mapping/README.md)；IMU 配置见
[core 的 IMU 章节](../core/README.md#imu-配置与状态转换)。

## 源码与测试

| 文件 | 职责 |
| --- | --- |
| `whole_body_config.h` | 内部配置类型 |
| `whole_body_config.cpp` | YAML 加载、路径解析和校验 |

回归测试为 [test_config.cpp](../../tests/test_config.cpp) 和
[test_table_config.cpp](../../tests/test_table_config.cpp)。
