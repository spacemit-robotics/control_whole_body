# api — 公共接口

封装配置加载、设备创建和整机执行，向应用提供 C 接口。
公共头文件为 [whole_body.h](../../include/whole_body.h) 和
[whole_body_transport.h](../../include/whole_body_transport.h)。

## 接口说明

| 接口 | 用途 |
| --- | --- |
| `whole_body_create()` | 读取应用主配置和硬件配置，创建设备句柄 |
| `whole_body_has_imu()` | 查询是否配置 IMU；1 表示启用，0 表示禁用，负值表示句柄无效 |
| `whole_body_init()` | 初始化电机、已启用的 IMU 和运行状态 |
| `whole_body_get_cycle_s()` | 获取调用周期，单位秒 |
| `whole_body_read()` | 读取统一关节状态和机体 IMU 状态 |
| `whole_body_write()` | 下发统一关节命令 |
| `whole_body_tick()` | 检查命令超时并执行周期维护 |
| `whole_body_set_mode()` | 设置硬件执行状态 |
| `whole_body_get_health()` | 获取运行状态、读写周期和超时信息 |
| `whole_body_get_diagnostics()` | 获取电机、虚拟关节和 IMU 快照 |
| `whole_body_get_motor_command_diagnostics()` | 获取物理电机命令快照 |
| `whole_body_get_diagnostics_v2()` | 获取反馈时间、错误分类和耦合指标等扩展快照 |
| `whole_body_get_motor_command_diagnostics_v2()` | 获取估算力矩和变化率等扩展命令快照 |
| `whole_body_get_transport_diagnostics()` | 获取逐总线通信计数和最近失败事务 |
| `whole_body_last_error()` | 获取最近一次错误说明 |
| `whole_body_destroy()` | 关闭设备并释放句柄 |

诊断接口不发送控制命令。对同一句柄的读写、状态切换及诊断调用应串行执行；
需要后台显示或落盘时，将快照交给后台线程，不在控制周期内格式化日志。

## 最小读取示例

先在硬件配置中设置 `startup_mode: read_only` 和 `allow_actuation: false`。
只读模式仍可能由外设驱动发送失能查询帧，不能与其他控制器共用同一总线。

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

这是一次读取示例。持续控制时，按 `whole_body_get_cycle_s()` 返回的周期调用
`read`、`write` 和 `tick`，逐项检查返回值。启动阶段可能需要多个周期收齐反馈，
进入使能状态前应确认设备已就绪。

## 数据与返回值

- 关节数组遵循主配置 `robot_base.joint_names` 的顺序。
- 位置、速度、力矩分别使用 rad、rad/s、Nm；加速度使用 m/s²。
- 姿态四元数顺序为 `w, x, y, z`，角速度使用 rad/s。
- `whole_body_has_imu()` 表示配置能力，不表示传感器健康；健康情况查看诊断快照。
- 操作成功返回 `WHOLE_BODY_OK`。启用通信容错时，`WHOLE_BODY_RETRY` 表示继续周期调用，
  不应当作致命错误，也不代表读到了新鲜反馈。
- 负值表示错误。错误枚举见公共头文件，具体原因由 `whole_body_last_error()` 提供。

状态切换、容错窗口和故障恢复见 [core](../core/README.md)。

## 源码与测试

`whole_body.cpp` 负责句柄生命周期及 C/C++ 接口封装，执行逻辑位于 `core/`。
接口与配置回归见 [test_config.cpp](../../tests/test_config.cpp)。
