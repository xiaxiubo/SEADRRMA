# DR-RMA 论文改进计划清单

> 基于对 `paper_dr_rma.tex` (2026-06-09)、PDF 编译结果及相关分析文档的全面审阅。

---

## 🔴 紧急修正（数据一致性 & 事实错误）

### ~~1. 摘要中的占位符数值~~
- **问题**: 摘要第 66 行 `XXXXXXXX ms`、第 70 行 `XXXXXXX rad` 仍为占位符
- **状态**: [x] 已初步修正（2026-06-10，仍建议用原始日志复核响应时间口径）
- **建议**: 
  - 响应时间填 `5 ms`（对应注意力坍缩速度）
  - RMSE 若采用 Test 2 结果，应填 `0.0333 rad`，并明确这是变惯量场景的结果
  - 响应时间必须在统一控制频率并重新核对原始日志后填写，不能直接沿用当前互相冲突的 `5 ms/10 ms`

### ~~2. Table V (Attention Analysis) 仍包含 3 个事件~~
- **问题**: `.tex` 第 656-669 行 Table 5 显示 Event 1 (3.0s)、Event 2 (5.0s)、Event 3 (7.0s)，但实际测试只有 2 个惯量变化事件（3s 和 7s）
- **状态**: [x] 已初步修正（正文已改为 2 个事件；表格数值仍需原始日志最终确认）
- **建议**: 删除 Event 2 (5.0s)，重新运行仿真获得 2 事件的真实注意力数据，注意 PAPER_MODIFICATIONS.md 中的数据（α_before=0.020/0.018, α_after=0.892/0.947）与 .tex 中的数据（α_before=0.014/0.056/0.004, α_after=0.944/0.944/0.524）不一致

### ~~3. 注意力热力图标题描述错误~~
- **问题**: 第 648 行 fig:attention_heatmap 标题写 "three inertia step changes at t=3s, 5s, 7s"
- **状态**: [x] 已修正
- **建议**: 改为 "two inertia step changes at t=3s and t=7s"

### ~~4. 控制频率不一致~~
- **问题**: 
  - 第 316 行 "Real-time control at 100Hz requires inference latency below 10ms"
  - 第 505 行 "operates at 100Hz"
  - 第 541 行 Table I: Control frequency = 200 Hz
  - 第 777 行 "200 Hz control frequency"
  - 第 787 行 "well within the 5 ms control period (200 Hz)" vs "10ms control period" 自相矛盾
- **状态**: [x] 已用 `deployment_package` 轻量化导出并实测修正
- **修正结果**:
  - 控制频率统一为 **200 Hz**，周期 5 ms
  - 新增 `deployment_package/benchmark_latency.py`、`deployment_package/export_lightweight.py`、`deployment_package/benchmark_lightweight.py`、`deployment_package/export_onnx.py`、`deployment_package/benchmark_onnx.py`
  - 原 PyTorch/SB3 部署路径 optimized control-only：mean 2.305 ms, P95 2.639 ms, P99 2.888 ms, max 3.561 ms, deadline miss 0/5000
  - 轻量化 TorchScript control-only：mean 1.911 ms, P95 2.122 ms, P99 2.291 ms, max 2.883 ms, deadline miss 0/5000
  - ONNX Runtime control-only：mean 0.916 ms, P95 1.222 ms, P99 1.274 ms, max 2.018 ms, deadline miss 0/5000
  - 轻量化路径不依赖 Stable-Baselines3 PPO zip 加载和 VecNormalize pickle 对象；control-only 参数量 504,650，FP32 权重约 1.93 MB
  - current-like full step：mean 2.448 ms, P99 3.131 ms, max 5.823 ms, deadline miss 2/5000
  - 论文 Section Computational Performance 已精简为只报告最终 ONNX Runtime 部署数据，不展开 PyTorch/TorchScript 对比

### ~~5. Test 1/Test 2 结果疑似复用或导出错误~~
- **问题**: 两个表中 PPO、RMA、PD+DOB 的 RMSE 完全相同；PD+DOB 的 Max Error、Torque RMS、Sat. Rate 完全相同；LQR 的 Torque RMS、Sat. Rate 也完全相同。多列跨场景精确重复，强烈暗示结果复用、统计脚本读取了同一文件或表格粘贴错误
- **状态**: [x] 已修正（使用当前旧版模型/旧程序重跑）
- **修正结果**:
  - 修复 `SEAEnv.reset()`，使 `options["inertia_schedule"]` 真正生效；原先 PPO/RMA/PD+DOB/LQR 的 Test2 没有应用惯量突变
  - 更新 `compare_baselines.py`，统一为论文测试配置：Test1 固定惯量 0.45 kg·m²；Test2 为 0.3→0.05→0.75 kg·m²；轨迹为 0.5 rad / 0.3 Hz
  - 新增 Torque RMS 与 Sat. Rate 指标输出，并保存 CSV/JSON/NPZ 日志
  - 结果文件：`figures/baseline_metrics_test1_fixed_inertia.csv`、`figures/baseline_metrics_test2_inertia_steps.csv`
  - 论文 Table III/Table IV、相关倍数和讨论文字已按新结果更新

### ~~6. Fig.1 子图引用不一致~~
- **问题**: 第 258 行引用 "Fig. 1(b) and (c)"，但 Fig. 1 只有 (a) 和 (b) 两部分
- **状态**: [x] 已修正
- **建议**: 修正为 "Fig. 1(a) and (b)" 或重新设计 Fig. 1 结构

---

## 🟡 重要补充（增强论文说服力）

### 7. 缺少消融实验 (Ablation Study)
- **问题**: 论文声称双速率架构是核心创新，但没有独立验证 Fast Branch 和 Slow Branch 各自贡献的实验
- **状态**: [~] 已在 Discussion 加入初步结构消融解释；待 RMA 新 checkpoint 更新后可重跑
- **当前处理**:
  - PPO 作为 `w/o adaptation`：去掉 Fast 和 Slow 两个适应分支
  - RMA 作为 `w/o Fast Branch`：保留单一时间适应模块，去掉 Fast Attention 显式惯量估计
  - 由于当前 RMA checkpoint 仍是半成品，正文将该比较表述为 preliminary structural ablation，而不是最终严格消融结论
- **建议实验**:
  - (a) DR-RMA 完整版
  - (b) 仅 Fast Attention Branch（去掉 Slow TCN）
  - (c) 仅 Slow TCN Branch（去掉 Fast Attention）
  - (d) 固定均匀注意力权重（验证注意力坍缩的必要性）
  - (e) RMA baseline（已有数据，可作为消融组）

### 8. 缺少鲁棒性测试
- **问题**: 所有实验在无噪声、无延迟的理想条件下进行，无法证明实际部署的可行性
- **状态**: [ ] 待补充
- **建议实验**:
  - 传感器噪声测试（位置噪声 0.001-0.01 rad, 速度噪声 0.01-0.1 rad/s）
  - 控制/传感延迟测试（5ms, 10ms, 20ms）
  - 模型参数误差测试（±20% 刚度、阻尼偏差）

### 9. 缺少多样化轨迹测试
- **问题**: 仅测试了 0.3 Hz 正弦轨迹，无法证明泛化能力
- **状态**: [ ] 待补充
- **建议实验**:
  - 方波轨迹（快速换向场景）
  - 梯形轨迹（加速/匀速/减速）
  - 多频率正弦（0.1, 0.5, 1.0, 1.5 Hz）
  - 随机参考轨迹

### 10. 惯量估计瞬态尖峰未讨论
- **问题**: t=3s 处惯量估计瞬间飙升至 ~2.4 kg·m²（真实值 0.05 kg·m²），论文中未提及此现象
- **状态**: [~] 已在 Discussion 增加瞬态估计行为讨论；峰值和持续时间仍需用原始日志最终核对
- **建议**: 在 Discussion 中如实报告峰值、持续时间和控制影响；不要将其无证据地表述为“必然代价”。同时修正正文“50 ms 内收敛”和“remarkable accuracy”等与日志不一致的表述，并考虑输出范围约束或估计滤波

### 11. 缺少更多基线对比
- **问题**: 仅对比了 PPO, RMA, PD+DOB, LQR 四种方法
- **状态**: [ ] 待补充（可选）
- **建议**: 增加 MRAC、Sliding Mode Control、ADRC 等自适应/鲁棒方法

---

## 🟢 可选改进（锦上添花）

### 12. 注意力坍缩的理论分析薄弱
- **问题**: Section VII-A 仅给出直观解释（"分布偏移检测"），缺少严格的数学分析
- **状态**: [ ] 待改进
- **建议**: 从互信息最大化角度推导坍缩的充分条件，或从变化点检测（Change Point Detection）理论建立联系

### 13. 计算性能分析不够详细
- **问题**: 仅在 CPU 上测试了推理延迟，且算子拆分不精确（Fast + Slow 并行但给出了独立延迟）
- **状态**: [ ] 待改进
- **建议**: 
  - 补充 Jetson AGX Orin 上的推理延迟（论文声称可部署但未验证）
  - 补充 GPU (TensorRT) 加速后的延迟
  - 补充内存带宽和功耗分析

### 14. 缺少长期稳定性/重复性测试
- **问题**: 所有测试都是单次 episode，不涉及统计显著性
- **状态**: [ ] 待改进
- **建议**: 多次重复实验（≥10 次），报告均值和标准差

### 15. Teacher Policy 训练细节不足
- **问题**: MLP 网络结构描述不够精确："two hidden layers with 128 units"——缺少对 Value Network 的描述、缺少训练曲线
- **状态**: [ ] 待改进
- **建议**: 补充完整的网络结构表、训练收敛曲线

### ~~16. Student Network 结构参数不一致~~
- **问题**: 
  - Fast Branch 的 query/key/value 维度 d=32 (line 409)，但 W_Q ∈ R^{d×9}, W_K ∈ R^{d×9} —— 输入维度正确
  - Slow Branch dilated conv 参数: RF = 1+1×7+2×7+4×3 = 34 steps，但 control freq = 200Hz 时 34 steps = 170ms（不是 340ms）；如 control freq = 100Hz 则为 340ms
- **状态**: [x] 已按部署模型修正：`L=100`，200 Hz 下历史窗口 500 ms；Slow TCN 理论 RF=122 steps，但有效真实观测上下文受 FIFO 限制为 100 steps
- **建议**: 核实 receptive field 计算，确保与声称的 "340ms receptive field" (line 461) 一致

### 17. 参考文献格式/质量
- **问题**: `wang2025novel` 和 `smith2026robust` 的作者写为 "Wang, Y. and others" 和 "Smith, J. and others"，缺少完整作者信息
- **状态**: [ ] 待改进
- **建议**: 补全参考文献的作者信息

### 18. 致谢占位符
- **问题**: 当前第 923 行 "[funding sources and collaborators]" 为占位符
- **状态**: [ ] 待填写

### ~~19. Basline Comparison 图中 Fig. 2 标注错误~~
- **问题**: 正文第 747 行写 “all three methods”，但紧接着的图和图注比较 DR-RMA、PPO、RMA、PD+DOB、LQR 共五种方法
- **状态**: [x] 正文文字已改为 “all five methods”；PNG 内容仍需人工/脚本核实
- **建议**: 将正文改为 “all five methods”，并检查导出的 PNG 是否确实包含全部五条方法曲线

---

## 🆕 新增内容

### 20. 添加实物实验章节
- **问题**: 论文全部基于仿真，缺少实物验证支撑
- **状态**: [~] 已写入实验台与实验协议第一版（见 `paper_dr_rma.tex` 的 Section VIII）；真实硬件结果仍在进行中，待补实测曲线、统计表和 sim-to-real 分析
- **建议**: 
  - 描述硬件平台（SEA 关节、控制器、传感器）
  - 报告 Sim-to-Real 迁移结果
  - 对比仿真与实物性能
  - 分析 Sim-to-Real Gap

### 21. 添加实验台搭建章节
- **问题**: 缺少对实验硬件平台的系统描述
- **状态**: [~] 已补充硬件平台、SEA 关节、20 位编码器、Kollmorgen 5008D 电机、LCSG-25-100 减速器、NVIDIA Orin 控制盒、200 Hz EtherCAT、DLY0-10A 电磁离合器、砝码组合和 3--10 s 切换协议；已插入未标注实验台照片 `figxExam.jpg`
- **建议**: 继续补充机械设计图/照片、电气连接框图、控制系统架构图、惯量标定表、离合器延迟标定和安全保护说明

---

## 🔎 二次审阅新增问题（2026-06-10）

### ~~22. Cross-Attention 公式矩阵维度不成立~~
- **问题**: 当前定义 $\mathcal{H}_t\in\mathbb{R}^{L\times9}$、$W_K,W_V\in\mathbb{R}^{d\times9}$，但正文写成 $W_K\mathcal{H}_t$ 和 $W_V\mathcal{H}_t$，矩阵无法相乘；后续 $Q^T K$ 与 $V_i$ 的方向也因此含糊
- **状态**: [x] 已修正公式维度
- **建议**: 统一采用 $K=\mathcal{H}_tW_K^T\in\mathbb{R}^{L\times d}$、$V=\mathcal{H}_tW_V^T\in\mathbb{R}^{L\times d}$、$\alpha=\mathrm{softmax}(KQ/\sqrt d)$，并逐式标注维度

### 23. 可变惯量动力学模型可能缺项
- **问题**: 式 (1)--(2) 直接使用 $J_l(t)\ddot\theta_l$。若惯量通过滑块连续改变，角动量方程通常还涉及 $\dot J_l\dot\theta_l$；若通过抓取/释放发生跳变，则应说明混杂事件、冲量或速度连续性假设
- **状态**: [~] 已在正文补充 piecewise-constant payload change 建模假设；仍建议理论上确认是否符合仿真实现
- **建议**: 明确惯量变化的物理实现和建模假设；连续变惯量时补充相应项，阶跃切换时说明仿真如何处理状态跳变及能量/动量一致性

### ~~24. 观测向量中的电机侧误差未定义完整~~
- **问题**: $e_m$、$\dot e_m$ 被称为 motor-side tracking errors，但正文没有定义电机侧参考 $\theta_{m,d}$；若直接相对负载参考 $\theta_d$ 计算，需要明确说明，否则 9 维观测不可复现
- **状态**: [x] 已补充 $e_m=\theta_d-\theta_m$ 与 $\dot e_m=\dot\theta_d-\dot\theta_m$ 定义
- **建议**: 给出 $e_m$、$\dot e_m$ 的明确公式及参考信号来源，并说明速度由传感器、差分还是观测器获得

### 25. “近零加速度仍可辨识惯量”的主张缺少可辨识性条件
- **问题**: 惯量通常需要足够激励才能从输入输出关系中辨识；仅凭历史注意力并不能绕过物理不可辨识性。当前贡献表述容易被审稿人认为过度宣称
- **状态**: [ ] 待论证
- **建议**: 增加持续激励/局部可观测性讨论，报告不同振幅和频率下的估计误差；将绝对表述改为“在本文测试轨迹提供的历史激励下”

### 26. 数据集规模与 episode 数量不一致
- **问题**: 正文称收集 3000 个 episode、约 150000 个样本，即平均每个 episode 仅 50 个有效样本；但测试/训练场景按 10 s、100/200 Hz 描述时，每个 episode 应产生约 950/1950 个有效窗口
- **状态**: [~] 正文已改为 subsampled history windows；仍需从数据生成脚本确认真实数量
- **建议**: 从数据生成脚本重新统计 episode 长度、采样步长、窗口抽样策略和 train/validation 数量，给出可复现的精确数字

### ~~27. 推理延迟计算和实时性结论互相矛盾~~
- **问题**: 若 Fast/Slow 分支并行，总延迟应按 `0.3 + max(1.8,2.1) + 1.5 + 0.2 ≈ 4.1 ms` 计算，而不是 5.9 ms；若 5.9 ms 是实测端到端时间，则不能再按算子简单相加解释。且 5.9 ms 明显大于 200 Hz 的 5 ms 周期
- **状态**: [x] 已端到端重测 P95/P99
- **修正结果**: `deployment_package/latency_results_cpu_1thread.json`、`deployment_package/latency_results_cpu_1thread_diagnostics.json`、`deployment_package/lightweight/latency_results_lightweight_cpu_1thread.json`、`deployment_package/onnx/latency_results_onnx_cpu_1thread.json` 已保存；论文采用 ONNX Runtime control-only 结果
- **建议**: 明确串并行执行图，使用端到端 wall-clock 测量并报告均值、P95/P99、预热、线程数和 batch size；在满足 deadline 前不要声称“leaving sufficient margin”

### 28. 注意力分析与实际 Test 2 配置整体脱节
- **问题**: 不仅 Table V 和图注有三个旧事件，第 643、654、675--681、684、689 行的结论、倍率、过冲示例和收敛时间也全部基于旧的 `0.3→0.8→0.15→0.6` 配置
- **状态**: [~] 正文旧三事件描述已整体改为两事件；仍需确认图文件与原始日志完全一致
- **建议**: 以当前 `0.3→0.05→0.75` 两事件原始日志重新生成热力图、细节图、表格和整段文字，不能只删除表格一行或修改 caption

### ~~29. Discussion 中的互信息公式不是实际优化目标~~
- **问题**: 式 $\alpha^*=\arg\max I(c(\alpha);J_l)$ 在训练损失中并未出现，当前只是解释性假设，却被写成数学最优化结论
- **状态**: [x] 已弱化为解释性假设并删除互信息最优化公式
- **建议**: 将其明确标为 interpretation/hypothesis，或补充可验证推导和互信息估计实验；否则删除该公式，避免把事后解释包装成训练机制

### ~~30. “Counter-Intuitive Transient Advantage” 章节数据错误~~
- **问题**: 该节声称 transient RMSE 为 `0.0194 rad` 且优于 steady-state 的 `0.0234 rad`，但 Table IV 的 Test 2 RMSE 是 `0.0333 rad`，实际更差；`0.0194` 在当前结果表中没有来源
- **状态**: [x] 已删除原“Transient Advantage”论证并改写为瞬态估计行为讨论
- **建议**: 追溯 `0.0194` 的定义（是否仅事件后窗口），明确统计区间；若无独立数据支持，删除整个“transient advantage”论证及噪声过滤假设

### ~~31. RMA 性能提升倍数前后不一致~~
- **问题**: Test 2 数据 `0.0402/0.0333≈1.21`，Results 写 1.2×，但 Discussion 第 809 行和 Conclusion 第 915 行写 1.6×；1.6× 实际对应 PPO
- **状态**: [x] 已修正
- **建议**: 全文统一为 DR-RMA vs RMA 约 1.2×、vs PPO 约 1.6×，并统一小数位和“improvement”计算口径

### ~~32. 对经典控制器的结论存在过度泛化和公平性风险~~
- **问题**: 仅凭一组 PD+DOB/LQR 参数就断言 DOB 在超过 `2--3×` 变化时“fundamentally/catastrophically fails”，缺少参数扫描、稳定性分析和同等调参预算；`g_DOB=-6.5` 的定义及负号来源也不清楚
- **状态**: [x] 已根据重跑结果重写，不再声称 PD+DOB 灾难性失败；仍可选做基线调参/敏感性实验
- **建议**: 给出基线离散实现、滤波器带宽、调参范围和最优选择依据；进行参数敏感性扫描，并把结论限制在“本文配置和测试范围内”

### 33. 基线训练公平性与可复现信息不足
- **问题**: PPO/RMA 的网络规模、训练步数、随机化范围、seed、模型选择规则和参数量没有完整列出，无法判断 DR-RMA 的提升来自架构还是更多容量/训练资源
- **状态**: [ ] 待补充
- **建议**: 增加 baseline configuration 表，报告参数量、训练环境步数、wall-clock、seed 数和超参数搜索预算；至少使用 5--10 个 seed 报告均值±标准差

### 34. Fast Branch 输出缺少物理约束与归一化说明
- **问题**: $\hat J_l$ 由线性层直接输出，没有正值/范围约束，现有 2.4 kg·m² 尖峰也超过训练范围 `[0.05,1.0]`；同时未说明 inertia 与 latent target 是否标准化，双损失量纲和尺度可能失衡
- **状态**: [ ] 待补充
- **建议**: 说明训练目标归一化；考虑 sigmoid/softplus 映射、范围约束、rate limiter 或不确定度输出，并用消融实验说明约束是否改善尖峰而不牺牲响应速度

### 35. TCN 实现细节不足
- **问题**: 只写“causal padding”与 LayerNorm，未说明 padding/cropping 数量、LayerNorm 作用维度、是否存在残差连接；这些细节会改变输出长度和实时实现
- **状态**: [ ] 待补充
- **建议**: 增加逐层结构表，列出 input/output channels、kernel、dilation、padding、normalization axis、activation、参数量和输出尺寸

### 36. 论文文字和排版仍有明显编辑问题
- **问题**: 包括摘要重复逗号（第 59--60 行）、`bandwidth,,`、`deployed on hardware minimal sensor configurations` 缺介词、标题中 `control` 大小写不一致、Section `RELATED WORK` 风格不统一；源文件编译注释还误写 `paper_drd_rma.tex`
- **状态**: [~] 已修正部分明显语法/风格问题；仍需全篇英文校对
- **建议**: 完成技术内容后进行一次全篇英文语法和 IEEE 风格校对，并修正编译说明与已删除的旧图文件清单

### 37. 空实验台章节不应以当前状态投稿
- **问题**: Section VIII 原先为空 subsection，PDF 中只会出现连续空标题；同时架构图声称已部署到 AGX Orin 且“without real-world fine-tuning”，与正文“全部仿真、未来验证”冲突
- **状态**: [~] 空章节问题已处理，已填入真实平台和协议；仍需补充实测结果，且最终投稿前要避免把“计划部署/正在实验”写成“已完成硬件验证”
- **建议**: 在实验结果完成前保留“Hardware experiments are in progress”的边界表述；若投稿版本不包含硬件结果，则不要在摘要和结论中过度声称实物验证

### 38. 投稿结论缺少与结果一致的边界表述
- **问题**: Conclusion 重复声称 100 Hz、6 ms 实时部署，同时将纯仿真结果概括为 embedded suitability；未提及估计尖峰、单 seed 和无实物验证等关键限制
- **状态**: [~] 结论中的实时性和内存口径已按 ONNX 轻量化部署包实测修正；摘要最终仍需随新实验和真实硬件数据重写
- **建议**: 待频率、延迟和实验数据定稿后重写摘要与结论，只保留已由实验直接支持的数字和适用范围

### 39. 编译与版式警告需要最终清理
- **问题**: 当前日志存在 unused global option、多个高 badness 的 Underfull hbox，以及“Label(s) may have changed”警告；虽无缺失引用，但说明尚未完成稳定的最终编译
- **状态**: [ ] 待处理
- **建议**: 使用 `latexmk -pdf` 完整编译，检查最终 PDF 的断行、表格宽度和浮动体位置；不要为了消除 Underfull 警告进行破坏可读性的手工空格调整

### 40. 实测弹簧刚度与旧仿真参数不一致
- **问题**: 19 位弹性编码器的双向静态砝码标定得到等效刚度约 `2401.2 N·m/rad`，后续采用四舍五入值 `2400 N·m/rad`；现有论文参数表、旧仿真结果和已训练 ONNX 仍基于 `3800 N·m/rad`
- **状态**: [~] 实物部署、标定、控制器和未来训练活动源已统一为 `2400 N·m/rad` 且不再乘 `0.628`；旧 checkpoint、历史日志和论文旧结果暂未改写
- **建议**: 使用 `K_s=2400 N·m/rad` 重新训练或微调并重跑论文仿真后，再同步修改 `paper_TRACE_title_intro.tex` 的参数表、结果图和结论；不得只改论文表格而沿用 `K_s=3800` 的旧结果

---

## 优先级排序

| 优先级 | 编号 | 工作量 | 影响 |
|--------|------|--------|------|
| 🔴 紧急 | ~~#1 摘要占位符~~ | 5 min | 论文基本完整性 |
| 🔴 紧急 | ~~#2 Table V 数据错误~~ | 30 min | 实验数据真实性 |
| 🔴 紧急 | ~~#3 热力图标题错误~~ | 5 min | 图表描述准确性 |
| 🔴 紧急 | ~~#4 控制频率矛盾~~ | 15 min | 技术参数一致性 |
| 🔴 紧急 | ~~#5 Test 1 可疑数据~~ | 1-2 h | 实验可信度 |
| 🔴 紧急 | ~~#6 Fig.1 引用错误~~ | 5 min | 交叉引用准确性 |
| 🔴 紧急 | ~~#22 注意力公式维度~~ | 30 min | 方法数学正确性 |
| 🔴 紧急 | #23 可变惯量模型 | 1-3 h | 物理建模正确性 |
| 🔴 紧急 | #26 数据集数量矛盾 | 30-60 min | 可复现性与可信度 |
| 🔴 紧急 | ~~#27 延迟与实时性~~ | 1-2 h | 部署结论真实性 |
| 🔴 紧急 | #28 注意力章节旧数据 | 1-3 h | 核心结果真实性 |
| 🔴 紧急 | ~~#30 瞬态优势数据错误~~ | 30 min | 讨论结论正确性 |
| 🔴 紧急 | ~~#31 提升倍数错误~~ | 10 min | 数据一致性 |
| 🟡 重要 | #7 消融实验 | 2-4 h | 核心贡献验证 |
| 🟡 重要 | #8 鲁棒性测试 | 2-4 h | 实用性证明 |
| 🟡 重要 | #9 多样化轨迹 | 2-3 h | 泛化能力证明 |
| 🟡 重要 | #10 惯量尖峰讨论 | 30 min | 诚实报告 |
| 🟡 重要 | #20 #21 实物实验 | 4-8 周 | 论文档次提升 |
| 🟢 可选 | #11-#19 其他 | 各 5-30 min | 综合质量 |

---

## 建议工作流

1. **先锁定事实基线**：从代码和原始日志确认控制频率、episode 长度、Test 1/2 数据、注意力事件和端到端延迟
2. **修正数学与物理定义**：优先处理 #22--#25，确保公式维度、变惯量模型和观测定义可复现
3. **重生成核心结果**：处理 #2、#5、#10、#26--#34，禁止继续沿用当前互相冲突的表格与文字
4. **补充实验**：完成 #7--#9、#14、#33，使用多 seed 与统一调参预算
5. **完善硬件章节**：Section VIII 已有第一版平台描述；下一步补照片/结构图、电气框图、惯量标定、离合器延迟标定和真实实验结果，未完成前避免在摘要和结论中过度声称实物验证
6. **最后统一写作与编译**：重写摘要/结论，完成 #17--#19、#36、#38--#39

---

*首次生成: 2026-06-09；二次审阅更新: 2026-06-10*
*审阅范围: `paper_dr_rma.tex`, `paper_dr_rma.pdf`, `PAPER_MODIFICATIONS.md`, `SIMULATION_IMPROVEMENTS.md`, `HARDWARE_EXPERIMENT_PLAN.md`, `INERTIA_SPIKE_ANALYSIS.md`, `references.bib`*
