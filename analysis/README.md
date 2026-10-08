# LQR 控制数据分析工具

这个文件夹包含用于分析和可视化 LQR 控制结果的工具。

通信与日志口径说明：
- EtherCAT 从站状态机是否到达 `OP`，和 CiA 402 是否进入 `Operation enabled` 是两层不同状态，日志里要分别判断。
- `statusword == 0x0000` 只表示当前 drive TxPDO 还没有给出有效状态，不能单独推出 EtherCAT 网络已经失败。
- `wkc` 需要以运行时 `master.expected_wkc` 为准，不能再用写死常量判断。
- `plot_lqr_results.py` 默认会自动选择 `logs/` 中最新生成的 CSV。

## 文件结构

```
analysis/
├── README.md                    # 本文件
├── plot_lqr_results.py         # LQR 结果可视化脚本
└── figures/                     # 生成的图表保存目录
    └── lqr_data_*.png          # 各次实验的结果图
```

## 使用方法

### 1. 绘制单次实验结果

```bash
python analysis/plot_lqr_results.py logs/lqr_data_YYYYMMDD_HHMMSS.csv --output-dir analysis/figures
```

参数说明：
- `csv_file`: CSV 数据文件路径（必需）
- `--output-dir`: 图片保存目录（可选，不指定则只显示不保存）

### 2. 只显示不保存

```bash
python analysis/plot_lqr_results.py logs/lqr_data_YYYYMMDD_HHMMSS.csv
```

## 图表说明

生成的图表包含 6 个子图：

1. **Motor Side Position Tracking** - 电机侧位置跟踪
   - 蓝色虚线：参考轨迹
   - 红色实线：实际测量值

2. **Load Side Position Tracking** - 负载侧位置跟踪
   - 蓝色虚线：参考轨迹
   - 红色实线：实际测量值

3. **Position Tracking Error** - 位置跟踪误差
   - 绿色：电机侧误差
   - 品红色：负载侧误差

4. **Load Side Velocity Tracking** - 负载侧速度跟踪
   - 蓝色虚线：参考速度
   - 红色实线：实际速度

5. **Control Torque** - 控制力矩
   - 蓝色：指令力矩
   - 红色：测量力矩

6. **Performance Metrics** - 性能指标统计
   - 位置和速度误差的均值和标准差
   - 最大力矩值

## 示例

最近一次成功运行的结果：
- 数据文件：`logs/lqr_data_20260416_183124.csv`
- 图表文件：`analysis/figures/lqr_data_20260416_183124.png`

性能指标：
- 电机侧位置误差：0.2221 ± 0.0577 rad
- 负载侧位置误差：0.2721 ± 0.1127 rad
- 最大力矩：19.87 Nm

## 依赖

- Python 3.8+
- pandas
- matplotlib
- numpy

安装依赖：
```bash
pip install pandas matplotlib numpy
```
