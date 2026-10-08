# 补充仿真实验建议

## 1. 立即修正论文数据（必须）

### 问题：
- Table 5 (attention_analysis) 包含不存在的 Event 2 (5.0s)
- 实际只有 2 个事件：t=3s 和 t=7s
- 需要重新生成注意力分析数据

### 修正方案：
```python
# 运行 DR-RMA 并记录注意力权重
# 在 t=3s 和 t=7s 前后各记录 0.1s 的注意力分布
# 计算熵值和权重变化
```

## 2. 补充 Test 1 真实数据（重要）

### 当前问题：
- Test 1 表格数据疑似复制 Test 2
- 需要真实的固定惯量测试

### 实验设置：
```python
test1_config = {
    "inertia": 0.45,  # 固定惯量
    "trajectory": "sine",
    "amplitude": 0.5,
    "frequency": 0.3,
    "duration": 10.0
}
```

## 3. 消融实验（证明架构有效性）

### 实验组：
1. **DR-RMA (完整)**：Fast Attention + Slow TCN
2. **仅 Fast Branch**：只用注意力估计惯量
3. **仅 Slow Branch**：只用 TCN 估计所有参数
4. **RMA (baseline)**：单一 TCN
5. **固定注意力**：均匀权重，无注意力机制

### 对比指标：
- 突变响应时间（10ms vs 50ms vs 100ms）
- 稳态跟踪精度
- 惯量估计误差

## 4. 鲁棒性测试

### 4.1 传感器噪声
```python
noise_levels = {
    "position": [0, 0.001, 0.005, 0.01],  # rad
    "velocity": [0, 0.01, 0.05, 0.1],     # rad/s
    "torque": [0, 0.1, 0.5, 1.0]          # N*m
}
```

### 4.2 模型误差
```python
model_mismatch = {
    "spring_stiffness": [0.8, 0.9, 1.0, 1.1, 1.2],  # ×K_nominal
    "damping": [0.5, 0.75, 1.0, 1.25, 1.5],
    "friction": [0, 0.5, 1.0, 2.0]  # ×nominal
}
```

### 4.3 延迟测试
```python
delays = {
    "sensor_delay": [0, 5, 10, 20],  # ms
    "actuator_delay": [0, 5, 10, 20]
}
```

## 5. 多样化轨迹测试

### 轨迹类型：
1. **正弦轨迹**（已有）：0.3 Hz, 0.5 rad
2. **方波轨迹**：测试快速换向
3. **梯形轨迹**：测试加速/减速段
4. **随机轨迹**：测试泛化能力
5. **实际任务轨迹**：抓取-放置动作

## 6. 极限测试

### 6.1 更极端的惯量变化
```python
extreme_tests = [
    {"range": "20×", "values": [0.05, 1.0]},
    {"range": "30×", "values": [0.03, 0.9]},
    {"sudden_drop": "payload_detachment", "change": "0.5 → 0.05"}
]
```

### 6.2 连续快速变化
```python
rapid_changes = {
    "interval": [0.5, 1.0, 2.0],  # seconds between changes
    "num_changes": [5, 10, 20]
}
```

## 7. 计算性能分析

### 测试内容：
- 不同硬件平台的推理时间
- 内存占用
- 功耗（嵌入式平台）
- 并行化效率

### 平台：
1. Intel i7 (已测试)
2. NVIDIA Jetson AGX Orin
3. Raspberry Pi 4
4. ARM Cortex-M7 (STM32H7)

## 8. 对比更多基线

### 建议添加：
1. **MRAC**：Model Reference Adaptive Control
2. **Adaptive PD**：在线参数调整的 PD
3. **Sliding Mode Control**：滑模控制
4. **H-infinity**：鲁棒控制

## 优先级排序

### 🔴 紧急（修正论文）
1. 修正 Table 5 注意力分析数据
2. 补充真实 Test 1 数据
3. 生成缺失的图表

### 🟡 重要（增强说服力）
4. 消融实验（证明双速率必要性）
5. 鲁棒性测试（噪声、延迟）
6. 多样化轨迹测试

### 🟢 可选（锦上添花）
7. 极限测试
8. 更多基线对比
9. 计算性能详细分析
