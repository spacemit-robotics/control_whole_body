# devices — 电机与 IMU 设备管理

根据硬件配置创建 `motor`、`imu` 设备，负责初始化、批量读写、反馈接收时间和通信诊断。
设备协议由对应外设组件实现；本模块不包含具体机型的接线、关节编号或安装参数。

## 接口说明

`DeviceManager` 是内部设备接口，由整机执行模块调用，也可在离线测试中替换为假设备。

| 接口 | 用途 |
| --- | --- |
| `CreatePeripheralDevices()` | 根据 `RuntimeConfig` 创建设备管理器 |
| `Init()` | 初始化设备及 IMU 固定安装变换参数 |
| `Read()` | 读取原始电机状态、IMU 和逐设备反馈状态 |
| `Write()` | 下发一批物理电机命令 |
| `GetTransportDiagnostics()` | 复制驱动已有的逐总线通信统计 |
| `Shutdown()` | 关闭设备 |
| `LastWriteError()`、`LastWriteTimedOut()` | 查询最近写入错误及是否仅为超时 |

## 设备配置

- `buses[].type` 支持 `socketcan` 和 `uart`，`device` 与 `bitrate` 由上层填写。
- `motors[].driver` 选择外设驱动，`bus` 引用已配置总线。
- `command_id` 和 `feedback_id` 分别用于命令及反馈标识。
- `driver_options` 中的嵌套项展开为键值项后传给外设驱动，具体键名见该驱动文档。
- IMU 的设备、波特率、安装矩阵和偏置来自 `imu` 节点，见
  [IMU 配置](../core/README.md#imu-配置与状态转换)。

支持批量接口的电机驱动由批量接口收发，其他驱动使用单设备接口。
跨总线是否并行由外设驱动实现决定。一批写入中即使部分电机失败，其他电机仍会尝试发送。

## 反馈时间与启动就绪

初始化后，全部电机和已启用 IMU 都必须收到本次启动之后的新反馈，且未超过配置的反馈期限。
启动等待及运行期故障处理由 [core](../core/README.md) 决定。

电机诊断区分两种时间来源：

| 来源 | 含义 |
| --- | --- |
| `HARDWARE_RECEIVE` | 驱动提供的实际反馈接收时间，用于启动和新鲜度判断 |
| `READ_COMPLETION` | 读取 API 的完成时间，仅用于诊断，不作为新反馈证明 |

配置 `require_motor_receive_timestamps: true` 时，缺少接收时间立即报配置错误；
否则继续等待，启动窗口到期仍没有有效接收时间则报反馈超时。
IMU 同时保留传感器采样时间与主机接收时间，新鲜度检查使用后者。

## 错误与通信诊断

写入错误信息包含失败电机、驱动、总线及设备、命令/反馈 ID、操作、控制模式、
驱动返回值和系统错误。一批多台电机失败时分别列出；驱动未提供系统错误时显示
`errno=unavailable`。

通信诊断按总线读取已有计数及最近失败事务，不另发查询帧。
未提供该能力的驱动不出现在通信诊断结果中。诊断调用应与设备读写串行，
格式化、显示和落盘放到上层后台线程。

## 源码与测试

`device_manager.*` 提供设备接口和外设实现。离线回归见
[test_device_manager.cpp](../../tests/test_device_manager.cpp) 和
[test_motor_write_errors.cpp](../../tests/test_motor_write_errors.cpp)，使用假设备，不打开硬件端口。
