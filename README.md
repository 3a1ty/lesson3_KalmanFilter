# 装甲板位姿卡尔曼滤波（作业六）

对 `armor_data.csv` 中带噪声的装甲板位姿序列（位置 x/y/z + 偏航角 yaw）做卡尔曼滤波，
输出平滑后的位姿与速度估计。

## 1. 数据

`armor_data.csv`：470 帧，列为 `position_x, position_y, position_z, yaw`（单位 m、rad），
无时间戳，按题意假设帧间隔相同。

> 数据未提供采样率，取 **dt = 1 帧**，因此速度单位为 **m/帧**。

## 2. 构建与运行

依赖：CMake ≥ 3.16、g++（C++17）、Eigen3；绘图需要 numpy + matplotlib。

```bash
# 构建
cmake -S . -B build
cmake --build build -j

# 运行（生成 result.csv）
cd build && ./ekf_armor && cd ..

# 画图（4 张：fig_x / fig_y / fig_z / fig_yaw）
python3 plot.py build/result.csv
```

也可指定数据文件：`./build/ekf_armor 其他数据.csv`。
程序同时会打印全局平均 NIS 与各轴 NIS，用于检验参数是否自洽。

## 3. 模型

状态 8 维：`x = [px, py, pz, yaw, vx, vy, vz, vyaw]^T`，观测 4 维：`z = [px, py, pz, yaw]^T`。

```
F = [ I4   dt·I4 ]      H = [ I4  0 ]      Q = diag(qx,qy,qz,qyaw, qx,qy,qz,qyaw)
    [ 0     I4   ]                          R = diag(rx, ry, rz, ryaw)
```

每帧流程：`predict(dt)` → `update(z)`；第一帧仅用于初始化（`x0` 取第一帧观测，速度置 0，
`P0` 的速度项取 0.25，表示"速度完全未知"）。

## 4. 观测噪声 R 的确定

`R` 由数据估计，而非手填：对相邻帧差分取中位数绝对偏差（MAD）后换算为方差：

```
σ ≈ 1.4826 · median(|Δz|)
R ≈ σ² / 2
```

yaw 在差分前先做 mod π 归一化（`std::remainder(Δ, π)`），否则 58 次 ±π 跳变会把方差抬高。

估计结果（MAD 法）：`R ≈ diag(4.96e-3, 2.97e-4, 1.24e-4, 3.17e-4)`。

**修正**：在调参时发现差分法低估了 R。将 R 适度上调，
最终取 `diag(6.0e-3, 3.0e-4, 4.0e-4, 3.5e-3)`。

## 5. 过程噪声 Q 的确定

各轴噪声水平相差近两个数量级（R_x 与 R_z 相差 40 倍），若共用一个标量 q，
会导致低噪声轴过度跟随、高噪声轴严重滞后。因此按轴分别设定 q。

调参指标为**分轴归一化新息平方**：

```
NIS_i = innovation_i² / (P⁻_ii + R_ii)      理论期望值 = 1，实用区间 1~3
```

扫描 q 使各轴 NIS 落入 1~3 区间，当前取值：

| 轴 | q | 分轴 NIS |
|---|---|---|
| x | 8e-4 | 1.89 |
| y | 3e-3 | 2.03 |
| z | 3e-6 | 2.89 |
| yaw | 5e-6 | 1.13 |

（全局平均 NIS ≈ 7.9，理论值 4）

## 6. yaw 的 π 周期性处理

装甲板为平板，PnP 存在正反解歧义，观测到的 yaw 会在相差 π 的两个值之间跳变（数据中 58 次）。
处理方式：状态保存连续角，**新息**做 mod π 归一化后再参与更新：

```cpp
innov(3) = std::remainder(z(3) - (H*x)(3), M_PI);   // 归一到 [-π/2, π/2)
```

## 7. 结果

| 图 | 内容 | 残差 RMSE |
|---|---|---|
| `fig_x.png` | 位置 X | 0.067 m |
| `fig_y.png` | 位置 Y | 0.005 m |
| `fig_z.png` | 位置 Z | 0.027 m |
| `fig_yaw.png` | 偏航角 | 0.054 rad |

滤波后 yaw 稳定跟随 −1.5 rad 附近的缓慢漂移，原始数据中的 ±π 尖峰被完全抑制。

## 8. 观察与局限

1. **目标并非匀速运动**：x、y 呈周期约 28 帧的锯齿形轨迹，与匀速模型假设不符。
   折返处存在固有的滞后/过冲，这是模型失配而非参数问题，通过增大该轴 q 只能部分缓解。

2. **周期性位姿跳变**：约每 28 帧出现一次位置跳变（0.6~0.9 m，且几乎只发生在水平面）
   与 yaw 跳变（≈π），推测为装甲板切换与正反面解歧义所致。

## 9. 文件结构

```
work/
├── CMakeLists.txt
├── main.cpp               # 读数据 → 滤波 → 写 result.csv → 打印 NIS
├── csv_reader.{hpp,cpp}   # CSV 解析
├── kalman_filter.{hpp,cpp}# 8 维卡尔曼滤波器
├── plot.py                # 4 张对比图
├── armor_data.csv
└── result.csv             # 程序输出（k + 4 观测 + 8 状态，共 13 列）
```

