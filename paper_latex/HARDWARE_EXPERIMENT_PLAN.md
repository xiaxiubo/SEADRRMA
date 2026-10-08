# 实物实验建议

## 硬件平台搭建

### 1. SEA 机械本体

#### 推荐方案 A：旋转 SEA 关节
```
组成：
- 电机：Maxon EC90 flat (200W) 或 T-Motor U8 II
- 减速器：Harmonic Drive CSF-14-100 (100:1)
- 弹簧：经双向静态砝码标定，后续统一采用扭转刚度 $K_s=2400$ N·m/rad
  - 或使用串联弹性元件（如 Series Elastic Element）
- 编码器：
  - 电机侧：增量编码器 2048 PPR
  - 负载侧：绝对编码器 17-bit (Renishaw RESOLUTE)
- 负载：可调质量块（滑块机构）
  - 质量范围：0.5-10 kg
  - 滑动距离：0-0.3 m
  - 实现惯量范围：0.05-0.75 kg·m²
```

#### 推荐方案 B：直线 SEA（更简单）
```
组成：
- 电机：直线电机或滚珠丝杠驱动
- 弹簧：线性弹簧 K = 10000 N/m
- 传感器：
  - 位置：光栅尺 1μm 分辨率
  - 力：六维力传感器（ATI Mini40）
- 负载：可更换质量块（0.5-5 kg）
```

### 2. 控制硬件

#### 主控制器
```
推荐：NVIDIA Jetson AGX Orin
- 理由：
  - 支持 PyTorch 推理
  - 算力充足（275 TOPS）
  - 实时 Linux 支持
  - 价格：~$2000

备选：Raspberry Pi 4 + PREEMPT_RT
- 成本低（~$100）
- 需要优化推理速度
```

#### 底层驱动
```
推荐：ELMO Gold Twitter
- 电流环：20 kHz
- EtherCAT 通信
- 支持力矩模式

备选：ODrive Pro
- 开源方案
- 成本低（~$200）
- CAN/UART 通信
```

### 3. 传感器配置

#### 必需传感器
```
1. 电机侧编码器：2048 PPR 增量式
2. 负载侧编码器：17-bit 绝对式
3. 电流传感器：内置于驱动器
4. （可选）六维力传感器：验证力控性能
```

#### 传感器同步
```
- 所有传感器通过 EtherCAT 同步采样
- 时间戳精度：< 1 μs
- 采样频率：1 kHz（与仿真一致）
```

## 实验阶段规划

### 阶段 1：系统辨识与建模（2-3 周）

#### 1.1 参数辨识
```python
parameters_to_identify = {
    "spring_stiffness": "K_s",
    "motor_inertia": "J_m",
    "load_inertia": "J_l (各个质量配置)",
    "damping": "B_m, B_l",
    "friction": "Coulomb + viscous",
    "backlash": "齿隙量"
}

methods = [
    "频率响应法（扫频）",
    "阶跃响应法",
    "最小二乘辨识"
]
```

#### 1.2 模型验证
```
- 开环响应对比
- 频率响应对比（Bode 图）
- 阶跃响应对比
- 目标：仿真与实物误差 < 10%
```

### 阶段 2：基线控制器验证（1-2 周）

#### 2.1 PD 控制器
```
- 调参：Ziegler-Nichols 方法
- 验证稳定性
- 记录性能基线
```

#### 2.2 LQR 控制器
```
- 使用辨识的模型参数
- 验证线性化模型的有效范围
- 对比仿真结果
```

### 阶段 3：DR-RMA 部署（2-3 周）

#### 3.1 Sim-to-Real 迁移
```python
domain_randomization_real = {
    "sensor_noise": "实测噪声水平",
    "actuator_delay": "实测延迟（5-10ms）",
    "model_mismatch": "±20% 参数偏差",
    "friction_variation": "温度相关的摩擦变化"
}
```

#### 3.2 在线微调（可选）
```
- 收集实物数据 1000 episodes
- 微调 student network
- 冻结 teacher policy
```

#### 3.3 安全机制
```python
safety_checks = {
    "torque_limit": "软限幅 + 硬件限位",
    "position_limit": "工作空间限制",
    "velocity_limit": "最大速度保护",
    "emergency_stop": "硬件急停按钮",
    "watchdog": "通信超时检测"
}
```

### 阶段 4：性能测试（2-3 周）

#### 4.1 复现仿真实验
```
Test 1: 固定惯量跟踪
- 惯量：0.45 kg·m²
- 轨迹：正弦 0.5 rad, 0.3 Hz
- 对比：DR-RMA vs PPO vs RMA vs PD+DOB vs LQR

Test 2: 惯量突变
- 初始：0.3 kg·m²
- t=3s: 0.3 → 0.05 kg·m² (释放负载)
- t=7s: 0.05 → 0.75 kg·m² (抓取重物)
```

#### 4.2 实际任务测试
```
Task 1: 抓取-放置
- 抓取轻物体（0.5 kg）
- 移动到目标位置
- 放置（惯量突变）
- 返回初始位置

Task 2: 人机交互
- 人手施加外力
- 控制器适应惯量变化
- 保持轨迹跟踪

Task 3: 连续操作
- 连续抓取不同质量物体
- 测试长时间稳定性
```

#### 4.3 鲁棒性测试
```
- 不同温度环境（20-40°C）
- 不同运行速度
- 不同负载配置
- 传感器故障模拟
```

### 阶段 5：数据分析与论文补充（1-2 周）

#### 5.1 对比分析
```
metrics = [
    "跟踪误差 RMSE",
    "最大误差",
    "力矩平滑度",
    "饱和率",
    "适应时间",
    "能耗"
]
```

#### 5.2 失效案例分析
```
- 记录所有失败案例
- 分析失败原因
- 提出改进方案
```

## 预期挑战与解决方案

### 挑战 1：Sim-to-Real Gap

**问题**：
- 仿真模型简化（无摩擦非线性、齿隙等）
- 传感器噪声
- 执行器动态特性

**解决方案**：
```python
# 1. 增强域随机化
enhanced_randomization = {
    "friction_model": "Stribeck + Coulomb + viscous",
    "backlash": "齿隙建模",
    "sensor_noise": "实测噪声谱",
    "actuator_bandwidth": "实测频响"
}

# 2. 在线适应
online_adaptation = {
    "method": "Meta-learning (MAML)",
    "data": "100 episodes on real hardware",
    "update": "Fine-tune student network only"
}

# 3. 鲁棒性增强
robustness = {
    "observation_filter": "Kalman filter",
    "action_filter": "Low-pass filter (50 Hz)",
    "safety_layer": "CBF (Control Barrier Function)"
}
```

### 挑战 2：实时性能

**问题**：
- 推理延迟 > 5ms
- 通信延迟
- 操作系统抖动

**解决方案**：
```bash
# 1. 模型优化
- TensorRT 加速（2-3× 提速）
- 量化（INT8）
- 剪枝（减少 30% 参数）

# 2. 实时 Linux
- PREEMPT_RT patch
- CPU 隔离（isolcpus）
- 高优先级线程

# 3. 通信优化
- EtherCAT（确定性通信）
- 共享内存（进程间通信）
```

### 挑战 3：安全性

**问题**：
- 学习控制器不可预测
- 可能产生危险动作

**解决方案**：
```python
# 1. 分层安全架构
safety_layers = {
    "layer_1": "硬件限位开关",
    "layer_2": "驱动器力矩限制",
    "layer_3": "软件安全监督器",
    "layer_4": "急停按钮"
}

# 2. 安全监督器
class SafetySupervisor:
    def check_action(self, action, state):
        # 检查力矩限制
        if abs(action) > tau_max:
            return clip(action, -tau_max, tau_max)
        
        # 检查位置限制
        if state.position > pos_max:
            return -abs(action)  # 强制反向
        
        # 检查速度限制
        if abs(state.velocity) > vel_max:
            return 0  # 停止
        
        return action

# 3. 渐进式测试
progressive_testing = [
    "Step 1: 小幅度运动（±0.1 rad）",
    "Step 2: 低速运动（0.1 rad/s）",
    "Step 3: 小质量变化（±20%）",
    "Step 4: 逐步增加到全范围"
]
```

## 成本估算

### 最小配置（~$5000）
```
- 电机 + 减速器：$1500
- 编码器 ×2：$800
- 驱动器：$600
- Jetson Orin：$2000
- 机械结构：$500
- 传感器 + 线缆：$600
```

### 推荐配置（~$10000）
```
- 高性能电机系统：$3000
- 高精度编码器：$1500
- 专业驱动器（ELMO）：$1500
- Jetson Orin + 备件：$2500
- 六维力传感器：$1000
- 机械加工：$1500
```

## 时间规划

### 总计：8-12 周

```
Week 1-2:   硬件采购与组装
Week 3-4:   系统辨识与建模
Week 5-6:   基线控制器验证
Week 7-9:   DR-RMA 部署与调试
Week 10-11: 性能测试与数据采集
Week 12:    数据分析与论文撰写
```

## 论文实验部分建议

### 必须包含的内容

1. **硬件描述**：
   - 系统照片
   - 参数表格
   - 控制架构图

2. **Sim-to-Real 对比**：
   - 相同测试在仿真和实物上的结果
   - 误差分析

3. **实际任务演示**：
   - 抓取-放置任务
   - 视频截图或曲线

4. **失效案例**：
   - 诚实报告失败情况
   - 分析原因

### 可选但加分的内容

1. **长时间稳定性测试**：
   - 连续运行 1000 次
   - 性能退化分析

2. **能耗对比**：
   - DR-RMA vs 基线的能耗

3. **用户研究**（如果是人机交互应用）：
   - 主观评价
   - 任务完成时间

## 总结

**当前仿真可以证明算法有效性**，但需要：
1. ✅ 修正论文中的数据错误
2. ✅ 补充缺失的实验数据
3. ✅ 增加消融实验和鲁棒性测试

**实物实验是必要的**，因为：
1. 验证 Sim-to-Real 迁移能力
2. 发现仿真未建模的现象
3. 增强论文说服力
4. 满足期刊/会议要求

**建议优先级**：
1. 🔴 立即修正论文数据错误
2. 🟡 补充仿真实验（消融、鲁棒性）
3. 🟢 规划实物实验（如果时间和预算允许）
