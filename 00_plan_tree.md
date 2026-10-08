# ETHERCAT_SEABOX Plan Tree

## 1. 目标

构建一套面向 SEA 单关节测试的基础软件架构，使 `Orin` 可以同时完成：

1. 通过 EtherCAT 与 `RS485/EtherCAT Box` 和 `SEA Joint Drive` 建立周期通信。
2. 通过 `RS485/EtherCAT Box` 的 RS485 透传能力读取 `eCoder35 / 485 扭转传感器`。
3. 在同一主循环中完成状态采集、控制指令下发、日志记录和后处理数据输出。

## 2. 当前确认的系统拓扑

```text
Orin
  |
  | EtherCAT
  v
RS485/EtherCAT Box
  |\
  | \ EtherCAT
  |  v
  |  SEA Joint Drive
  |
  | RS485
  v
485 Torsion Sensor / eCoder35
```

## 3. 第一阶段总原则

1. 不一次性铺开全部控制算法代码。
2. 先跑通通信，再接控制。
3. 先完成最小闭环：
   `初始化 -> OP -> PDO收发 -> eCoder透传读取 -> 数据记录`
4. 控制算法文件先保留接口边界，后续逐步补内容。

## 4. 实施树

### 4.1 通信层

#### 4.1.1 EtherCAT 主站封装

- 目标：建立 Orin 侧主站初始化、从站枚举、状态切换、PDO 周期收发。
- 输出：
  - 主站初始化模块
  - 从站状态检查模块
  - PDO 周期任务
  - 通信异常与恢复接口

#### 4.1.2 SEA Joint Drive 接口

- 目标：完成 CIA402 风格驱动的上电、使能、模式切换、状态读取、目标值写入。
- 优先读取对象：
  - `Statusword`
  - `Modes of operation display`
  - `Position actual value`
  - `Velocity actual value`
  - `Torque actual value`
- 优先写入对象：
  - `Controlword`
  - `Modes of operation`
  - `Target Torque`
  - `Target position`
  - `Target velocity`

#### 4.1.3 RS485/EtherCAT Box 接口

- 目标：将该盒子作为 EtherCAT 从站接入，并开启 RS485 透传工作模式。
- 输出：
  - 盒子工作模式配置说明
  - 盒子 PDO 通道映射说明
  - 透传收发缓冲封装
  - 盒子在线/掉线状态检测

#### 4.1.4 eCoder35 / 485 扭转传感器读取

- 目标：经 RS485/EtherCAT Box 透传读取弹性体形变数据。
- 已知约束：
  - 采用 RS485 协议透传
  - 使用 `Data ID 0 (0x02)`
  - 设备 ID 采用 `ID0`
  - 数据格式按当前需求记录为 `130/130`
- 输出：
  - 请求帧定义
  - 响应帧解析
  - 异常校验策略
  - 形变量换算接口

### 4.2 控制层

#### 4.2.1 控制算法文件边界

- `lqr_setup`
- `lqr`
- `PD_DOB_setup`
- `PD_DOB`
- `PPO_train`
- `PPO`

说明：

- 初版只要求把这些模块的职责与调用边界梳理清楚。
- `PD_DOB` 与 `PPO` 暂不展开具体实现。

#### 4.2.2 环境封装

- 目标：提供统一 `Env` 接口，供主流程和控制器调用。
- 建议接口：
  - `init()`
  - `enable_joint()`
  - `set_mode(mode)`
  - `set_target_position(value)`
  - `set_target_velocity(value)`
  - `set_target_torque(value)`
  - `read_drive_state()`
  - `read_elastic_sensor()`
  - `step(command)`

### 4.3 主流程

- `main_lqr`
- `main_pddob`
- `main_ppo`

每个主流程都应包含：

1. 通信初始化
2. 轨迹生成或轨迹加载
3. 状态读取
4. 控制量计算
5. 指令下发
6. 数据记录
7. 控制台打印和异常退出

### 4.4 轨迹层

- 正弦轨迹
- 阶跃轨迹
- 多正弦叠加轨迹
- 力轨迹
- 位置轨迹

### 4.5 数据记录与后处理

- 原始 PDO 数据日志
- eCoder 原始透传帧日志
- 关节状态日志
- 控制量日志
- 画图与统计分析脚本

## 5. 第一批必须先完成的工作包

### WP1 通信文档固化

- 明确从站拓扑
- 明确 PDO 分工
- 明确 eCoder 透传约束

### WP2 通信最小验证

- 主站能扫描到两个 EtherCAT 从站
- 两个从站都能切换到 `OP`
- 能周期读取 SEA 驱动器状态 PDO
- 能通过透传方式发出 eCoder 请求帧
- 能收到并解析 eCoder 响应帧

### WP3 通信封装代码骨架

- EtherCAT 主站类
- Drive 接口类
- RS485 Box 接口类
- eCoder 协议解析类
- 最小测试脚本

## 6. 风险与待确认项

1. `130/130` 的精确定义目前先按需求记录，后续建议在联调时对照编码器手册原始帧进一步确认字段含义。
2. `RS485/EtherCAT Box` 的透传 PDO 字节宽度需要结合实际 ESI/对象字典确认，当前 ENI 中可见 `User MOSI` / `User MISO` 通道，但实际是否足够承载完整串口帧仍需实机确认。
3. `SEA Joint Drive` 与 `RS485/EtherCAT Box` 是串联在同一 EtherCAT 链上，还是由盒子继续向下桥接 EtherCAT，需以实际接线和从站枚举结果为准。

## 7. 下一步建议

下一步优先进入 `WP2`，即先做通信最小验证，并在代码层只实现最薄的一层：

1. 扫描 EtherCAT 从站
2. 打印驱动器 PDO 状态
3. 发送一次 `eCoder ID0 / Data ID0 (0x02)` 请求
4. 解析回传值并输出弹性体形变量
