# PDO 与 eCoder 透传设计

## 1. 设计目标

通过 `PDO` 周期通讯同时完成两件事：

1. 读取 `SEA Joint Drive` 的运行状态
2. 通过 `RS485/EtherCAT Box` 透传读取 `eCoder35`

其中，`eCoder35` 读取遵循当前需求约束：

- 采用透传形式
- 遵循 485 协议
- 设备 ID 使用 `ID0`
- 对应码值记录为 `0x02`
- 数据读取格式记录为 `130/130`

说明：

- `eCoder编码器用户手册V2.2` 中可确认 `Data ID 0` 为 `0x02`
- `130/130` 的具体字节定义在当前仓库中没有直接展开，本文先按需求约束固化，待联调时进一步确认帧字段

## 2. PDO 角色划分

### 2.1 SEA Joint Drive PDO

建议作为实时控制主通道。

读方向：

- `Statusword`
- `Modes of operation display`
- `Position actual value`
- `Velocity actual value`
- `Torque actual value`

写方向：

- `Controlword`
- `Modes of operation`
- `Target Torque`
- `Target position`
- `Target velocity`

### 2.2 RS485/EtherCAT Box PDO

建议分成两部分：

读方向：

- 盒子运行状态
- 透传返回数据

写方向：

- 透传发送数据
- 透传触发/握手位

结合现有 ENI，可优先关注：

- `0x2703 User MOSI`
- `0x2704 User MISO`

这两个对象很适合承载最小透传帧验证。

## 3. 周期任务建议

一个 EtherCAT 控制周期内的处理顺序建议如下：

1. 读取驱动器 TxPDO
2. 读取盒子 TxPDO
3. 判断是否到达编码器轮询时刻
4. 若到达，则向盒子 RxPDO 写入一帧 `eCoder` 请求
5. 向驱动器 RxPDO 写入当前控制指令
6. 发送本周期 PDO
7. 在下一个周期解析盒子返回数据

这样做的好处是：

- 驱动器控制和传感器读取共享统一周期
- 编码器读取可以自然并入控制日志
- 后续容易做时序分析

## 4. eCoder 透传链路

### 4.1 请求路径

```text
Orin
  -> EtherCAT RxPDO of RS485/EtherCAT Box
  -> Box serial TX
  -> RS485
  -> eCoder35
```

### 4.2 响应路径

```text
eCoder35
  -> RS485
  -> Box serial RX
  -> EtherCAT TxPDO of RS485/EtherCAT Box
  -> Orin
```

## 5. 请求帧与响应帧

当前建议的软件接口不要直接把“裸整数”散落在业务逻辑中，而是统一定义协议层函数：

- `build_read_id0_request()`
- `parse_id0_response(frame_bytes)`
- `decode_deformation(raw_value)`

其中：

- `build_read_id0_request()` 负责按 `ID0 (0x02)` 与 `130/130` 约束拼帧
- `parse_id0_response(frame_bytes)` 负责校验长度、校验位、分隔符和数据区
- `decode_deformation(raw_value)` 负责把编码器数据换算成弹性体形变量

## 6. 软件状态机建议

### 6.1 盒子透传状态机

- `IDLE`
- `TX_PENDING`
- `WAIT_RX`
- `RX_READY`
- `TIMEOUT`
- `FRAME_ERROR`

### 6.2 编码器协议状态机

- `NO_FRAME`
- `HEADER_OK`
- `ID_OK`
- `DATA_OK`
- `CRC_OK`
- `DONE`

即使初版逻辑很简单，也建议先保留这些状态枚举，后面联调会省很多时间。

## 7. 最小测试项

### 7.1 PDO 链路测试

目标：

- 主站成功进入周期态
- 驱动器状态 PDO 能稳定刷新
- 盒子用户 PDO 能稳定收发

### 7.2 透传发送测试

目标：

- 周期性向盒子写入固定测试帧
- 在示波器或串口工具中确认 RS485 实际发出

### 7.3 eCoder 回读测试

目标：

- 接收到 `ID0 / Data ID0 (0x02)` 对应响应
- 能解析出稳定的原始读数
- 原始读数变化与手工扭转方向一致

### 7.4 形变量换算测试

目标：

- 把原始读数转成可用于控制的形变量
- 输出到日志并可画图检查连续性

## 8. 当前建议的实现优先级

1. 先用固定假数据验证 `User MOSI/User MISO` 通道打通
2. 再接 `eCoder ID0 (0x02)` 真实请求帧
3. 再补超时、重发、错误统计
4. 最后再把该通道接入控制主循环

## 9. 待联调确认项

1. `User MOSI/User MISO` 的实际有效字节数是否足够承载完整串口帧
2. 盒子透传模式下是否需要额外握手位或长度字段
3. `130/130` 的精确定义是“请求/响应长度”还是“手册中的字段格式代号”
4. `ID0 (0x02)` 在当前设备配置下是否需要附加地址位、奇偶位或校验位
