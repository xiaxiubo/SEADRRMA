# 通信架构总览

## 1. 新拓扑

新的通信架构定义为：

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
485 扭转传感器 / eCoder35
```

与旧描述相比，核心变化有两点：

1. `RS485/EtherCAT Box` 不再只是单纯挂接编码器，而是整个系统中的一个正式 EtherCAT 从站。
2. `Orin` 需要同时管理两个对象：
   - `SEA Joint Drive`
   - `RS485/EtherCAT Box`

## 2. 数据流拆分

系统中的数据流分为两条：

### 2.1 驱动器主控制流

`Orin <-> EtherCAT <-> SEA Joint Drive`

用途：

- 关节使能
- 模式切换
- 目标位置/速度/力矩下发
- 位置/速度/力矩/状态字反馈

### 2.2 弹性体形变采集流

`Orin <-> EtherCAT <-> RS485/EtherCAT Box <-> RS485 <-> eCoder35`

用途：

- 由 `Orin` 周期发送编码器读数请求
- 由 `RS485/EtherCAT Box` 负责在 EtherCAT PDO 与 RS485 串口之间转发
- 返回 `eCoder35` 读数后，`Orin` 解析并换算出弹性体形变

## 3. 通讯方式要求

### 3.1 EtherCAT 侧

- 主通讯方式：`PDO`
- 目标：在实时循环中同时读取两个对象的状态

对象 1：`SEA Joint Drive`

- 读取驱动器状态 PDO
- 写入驱动器命令 PDO

对象 2：`RS485/EtherCAT Box`

- 读取盒子状态和透传回包
- 写入盒子透传发送数据

### 3.2 RS485 侧

- `RS485` 为 `RS485/EtherCAT Box` 与 `eCoder35 / 485 扭转传感器` 的通信接口
- `eCoder35` 读取采用透传形式
- 透传的本质是：
  `Orin` 在 EtherCAT PDO 中写入串口发送数据，盒子将其从 RS485 发出；盒子接收到 RS485 响应后，再经 PDO 返回给 `Orin`

## 4. 关键实现原则

### 4.1 先通信，后控制

当前阶段优先完成：

1. EtherCAT 从站在线
2. OP 状态切换
3. PDO 周期收发
4. eCoder 透传回读

控制算法只保留边界，不抢先展开。

### 4.2 统一时基

建议将驱动器状态读取与编码器透传读取统一放入同一个周期任务中，避免：

- 驱动器状态和弹性体形变时间戳错位
- 控制输入和反馈不同步

### 4.3 透传通道独立封装

建议把透传读写封装成独立模块，不要把串口协议解析散落在主循环中。原因：

- 便于后续更换编码器型号
- 便于添加 CRC、超时、重发
- 便于离线回放原始帧做调试

## 5. 与现有 ENI 的对应关系

在现有 [SEA1JointBoxENI0414.xml](/home/buaa/workspace/SEA_BOX/SEA1JointBoxENI0414.xml) 中，可以直接看到驱动器 PDO 基础映射：

- `TxPDO Mapping 1`：
  `Statusword`、`Modes of operation display`、`Position actual value`、`Velocity actual value`、`Torque actual value`
- `RxPDO Mapping 1`：
  `Controlword`、`Modes of operation`、`Target Torque`、`Target position`、`Target velocity`

同时还能看到一对适合透传的用户通道：

- `RxPDO Mapping 3 -> User MOSI (0x2703)`
- `TxPDO Mapping 4 -> User MISO (0x2704)`

这说明当前 ENI 至少已经存在“主站写入用户数据”和“从站返回用户数据”的 PDO 槽位，后续可以优先把它作为 RS485 透传承载通道来验证。

## 6. 状态读取范围

“对两对象的状态读取”建议在软件中拆成下面两类状态：

### 6.1 SEA Joint Drive 状态

- EtherCAT 从站状态机状态
- CIA402 `Statusword`
- 实际位置、速度、力矩
- 故障与告警位

实现口径上需要明确区分：
- EtherCAT `OP` 只说明从站状态机已进入可交换过程数据阶段，不等于 CiA 402 已完成使能。
- CiA 402 ready 需要继续通过 `0x6040/0x6041` 状态迁移确认，例如 `Ready to switch on`、`Switched on`、`Operation enabled`。
- `Statusword == 0x0000` 更接近“当前 PDO 中还没有可用 drive 状态”，不能单独作为 EtherCAT 建链失败的依据。
- `WKC` 期望值必须以主站运行时检测到的 `expected_wkc` 为准，不能依赖硬编码常量。

### 6.2 RS485/EtherCAT Box 状态

- EtherCAT 从站状态机状态
- 透传发送是否完成
- 透传接收缓冲是否有新数据
- 串口收发错误计数或异常标志

如果盒子对象字典没有直接暴露这些状态位，至少要保留：

- PDO 收发计数
- 最近一次发送时间
- 最近一次收到有效帧时间

## 7. 最小落地结构

建议后续代码结构至少包含：

```text
ETHERCAT_SEABOX/
  comm/
    ethercat_master.py
    sea_drive.py
    rs485_ec_box.py
    ecoder35_protocol.py
  control/
    lqr_setup.py
    lqr.py
  scripts/
    test_scan.py
    test_drive_pdo.py
    test_ecoder_passthrough.py
```

当前文档阶段先不展开代码细节，但后续实现建议按这个边界推进。
