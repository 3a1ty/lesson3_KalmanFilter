"""把 result.csv 画成 4 张独立的对比图：x / y / z / yaw

用法（在 work/ 目录下）：
    python3 plot.py                    # 默认读 build/result.csv
    python3 plot.py 路径.csv           # 指定其它文件

输出：
    fig_x.png  fig_y.png  fig_z.png  fig_yaw.png
"""

import sys

import matplotlib
matplotlib.use("Agg")           # 无显示器环境也能存图
import matplotlib.pyplot as plt
import numpy as np

# 中文字体（本机实测可用），找不到也不会画出方块
plt.rcParams["font.sans-serif"] = ["Noto Sans CJK JP", "AR PL UKai CN", "DejaVu Sans"]
plt.rcParams["axes.unicode_minus"] = False


def load(path):
    """读取 result.csv，返回按列名索引的结构化数组"""
    d = np.genfromtxt(path, delimiter=",", names=True, encoding="utf-8")
    if d.size == 0:
        sys.exit(f"{path} 是空的")
    return d


def plot_one(t, raw, filt, xlabel, ylabel, title, outfile, wrap=None):
    fig, ax = plt.subplots(figsize=(10, 3.6))

    ax.plot(t, raw, color="tab:blue", lw=0.8, alpha=0.55, label="原始观测")
    ax.plot(t, filt, color="tab:orange", lw=1.8, label="滤波结果")

    ax.set_xlabel(xlabel)
    ax.set_ylabel(ylabel)
    ax.set_title(title)
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best")

    fig.tight_layout()
    fig.savefig(outfile, dpi=150)
    plt.close(fig)

    # 残差统计（yaw 用 π 周期差）
    diff = raw - filt
    if wrap is not None:
        diff = np.remainder(diff + wrap / 2, wrap) - wrap / 2
    rmse = np.sqrt(np.mean(diff ** 2))
    print(f"{outfile:<12} 残差 RMSE = {rmse:.5f}")
    return rmse


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else "build/result.csv"
    d = load(path)
    t = d["k"]

    print(f"读取 {path}，共 {len(t)} 帧\n")

    plot_one(t, d["x"], d["fx"], "帧号", "x [m]",
             "位置 X：原始观测 vs 卡尔曼滤波", "fig_x.png")
    plot_one(t, d["y"], d["fy"], "帧号", "y [m]",
             "位置 Y：原始观测 vs 卡尔曼滤波", "fig_y.png")
    plot_one(t, d["z"], d["fz"], "帧号", "z [m]",
             "位置 Z：原始观测 vs 卡尔曼滤波", "fig_z.png")
    plot_one(t, d["yaw"], d["fyaw"], "帧号", "yaw [rad]",
             "偏航角：原始观测 vs 卡尔曼滤波", "fig_yaw.png",
             wrap=np.pi)

    print("\n已生成 4 张图：fig_x.png / fig_y.png / fig_z.png / fig_yaw.png")


if __name__ == "__main__":
    main()
