# mapping — 关节映射与阻抗转换

将物理电机状态转换成统一关节状态，并将关节目标、增益和力矩转换成物理电机命令。
映射参数由上层硬件 YAML 提供，模块不包含机型的零位、极性、几何尺寸或镜像规则。

## 映射方式

| `joints[].mapping` | 关系 | 配置 |
| --- | --- | --- |
| `direct` | 一个电机对应一个关节 | 在关节条目引用一个电机 |
| `parallel_ankle` | 两个电机共同驱动两个虚拟关节 | 在 `couplings` 配置几何参数 |
| `table_2d` | 两个电机/两个关节的表格关系 | 在 `couplings` 配置表格及方向 |

两轴耦合的关节、电机顺序由数组明确指定，两个关节的阻抗配置必须一致。
耦合配置的完整几何示例见 [test_parallel_ankle.cpp](../../tests/test_parallel_ankle.cpp)，
表格示例见 [test_table_config.cpp](../../tests/test_table_config.cpp)。

## 表格配置

```yaml
couplings:
  - type: table_2d
    joints: [joint_0, joint_1]
    motors: [motor_0, motor_1]
    table_file: mapping.csv
    table_direction: motor_to_joint
    table_interpolation: bilinear
    table_output_scale: [1.0, 1.0]
```

该片段展示表格选择字段；还需按机构填写求解及范围限制。

| 字段 | 含义 |
| --- | --- |
| `table_file` | 相对硬件 YAML 的 CSV 路径 |
| `table_direction` | 必填；`motor_to_joint` 或 `joint_to_motor` |
| `table_interpolation` | `bilinear`（默认）或 `delaunay` |
| `table_output_scale` | 两个输出的比例/符号，默认 `[1, 1]`，须有限且非零 |
| `motor_limit`、`ankle_limit` | 数值求解的电机侧、关节侧范围，不替代单关节配置限位 |
| `max_iterations`、`squared_tolerance` | 反向求解的最大迭代次数及平方误差阈值 |
| `jacobian_condition_limit` | 可选，运动变换矩阵的条件数上限；0 为只记录、不拒绝 |
| `torque_amplification_limit` | 可选，最坏方向力矩放大倍数上限；0 为只记录、不拒绝 |

CSV 无表头，四列为 `input_0,input_1,output_0,output_1`，角度单位 rad。
输入、输出的顺序及标定坐标必须与配置一致。输入坐标组成规则网格，允许两轴步长不同；
重复坐标、无穷值、多余列和不规则输入会拒绝加载。缺失格点或 NaN 输出表示不可达点。

`bilinear` 使用双线性插值；`delaunay` 使用三角平面插值。Delaunay 模式需保留表格行序，
因为共圆格点可能对应不同的合法对角线。两种插值都要求格子四角有效，不越界外推，
不夹紧输入，也不重新分配 NaN 点权重。

三角剖分在加载阶段完成，运行期按规则格子定位。反向映射采用有界局部求解；
电机位置目标使用缓存和零点两次求解尝试，不扫描整张表。失败会报告错误并进入安全处理，
因此求解失败也可能是局部未收敛，需要结合输入和映射有效域排查。

## 阻抗方式

`impedance.mode` 决定 PD 在主机还是电机执行：

| 模式 | 执行方式 | 适用范围 |
| --- | --- | --- |
| `motor` | 电机执行位置/速度 PD | 直驱关节默认；耦合关节需显式选择 |
| `software` | 主机在关节空间计算 PD，再转换成电机力矩；电机 KP/KD 为 0 | 耦合关节默认，也可用于直驱 |
| `split` | 电机承担一部分 PD，主机以力矩补足剩余部分 | 直驱及耦合关节 |
| `motor_projected` | 将关节比例力矩投影到电机位置目标，阻尼由电机执行 | 仅耦合关节 |

`software` 和 `split` 发送使能命令前必须获得完整反馈。
可选 `impedance.rl_mode` 仅在 RL 状态覆盖 `mode`，其他状态使用 `mode`。

### 耦合转换

约定 `J = d(motor)/d(joint)`：关节速度到电机速度使用 `J`，关节力矩到电机力矩
使用 `J^-T`，电机反馈力矩到关节力矩使用 `J^T`。两种耦合映射采用相同约定。

`motor` 将目标关节位置做运动学逆解，再转换目标速度和前馈力矩。
令 `A = J^-1`，电机增益取 `A^T diag(Kq) A` 与 `A^T diag(Dq) A` 的对角元素。
这是一种局部对角近似，未包含交叉轴 PD；模式选择需结合机构和闭环测试。
DAMP 使用电机 KP=0、速度目标=0、前馈力矩=0 及转换后的 KD；纯力矩模式的 KP/KD 为 0。

`motor_projected` 的位置目标为：

```text
m_des = m_measured + [J^-T Kq (q_des - q_measured)] / Km
```

除法按电机逐项执行，`Km` 使用实际下发的电机 KP，保留关节比例项的交叉轴作用。
电机 KP 小于 `1e-4` 时目标保持当前角度。阻尼使用变换矩阵的对角项，不添加主机速度阻尼残差。

`split` 在关节空间计算完整 PD 并转换力矩，从中扣除电机承担的 PD 后下发剩余前馈。
耦合电机承担部分取完整增益矩阵最小特征值构成的非负对角项，并受配置上限约束。
采样点处按总力矩匹配，实际闭环仍受反馈时延和协议量化影响。

### 增益上限

| 配置位置 | 行为 |
| --- | --- |
| `split` 的 `motor_kp_max` / `motor_kd_max` | 必填，限制电机分担部分；KP 上限为 0 表示位置 PD 全部由主机计算 |
| `motor_projected` 的同名字段 | 可选，须成对配置；正值限制电机增益，0 或省略表示不设相应上限 |
| `motors[].command_limits.kp_max/kd_max` | 发包前检查最终电机命令，超限拒绝，不进行静默截断 |

`motor_projected` 先限幅增益并转为电机接口的 float，再使用该 KP 生成目标位置。
增益上限取自机型硬件配置，协议量化造成的有效增益变化也需在选型和配置时考虑。

### 主机阻尼滤波

`impedance.velocity_filter_time_constant_s` 为一阶低通时间常数，单位秒；默认 0 关闭。
它只作用于 `software/split` 的主机阻尼，不改变对上发布的关节速度、IMU 关节链或速度保护。

滤波按新反馈的接收时间更新，系数为 `1-exp(-dt/tau)`；耦合关节取两电机较早时间，
重复缓存不重复更新。首次反馈、故障恢复、时间回退或超出反馈期限时以当前速度重新初始化。
无电机接收时间时使用主机单调时间；设备就绪要求仍由设备层检查。
`split` 扣除电机实际 PD 时使用原始速度，避免重复叠加。

## 有效域与限位

`motor` 的位置目标需通过耦合逆解及已配置的雅可比阈值检查。
`software` 使用位置参考计算关节 PD，不反算参考姿态；力矩转换使用当前实测姿态的雅可比。
`motor_projected` 使用比例力矩生成电机位置目标，不对关节参考做运动学逆解。
DAMP、纯速度、纯力矩模式不检查未使用的位置目标。

各模式仍执行实际使用字段的参考限位、实测映射有效域/数值检查和最终电机命令检查。
求解或阈值检查失败交由 [core](../core/README.md) 处理，不输出未经检查的近似目标。

## 源码与测试

| 文件 | 职责 |
| --- | --- |
| `coupled_mapping.*` | 统一耦合接口、速度/力矩及增益转换 |
| `parallel_ankle.*` | 并联连杆几何正反解与雅可比 |
| `table_mapping.*` | CSV 加载、插值和反向求解 |

回归测试见 [test_parallel_ankle.cpp](../../tests/test_parallel_ankle.cpp) 和
[test_table_mapping.cpp](../../tests/test_table_mapping.cpp)。
