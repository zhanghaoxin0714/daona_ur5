import argparse
import os
from typing import List, Tuple

import matplotlib.pyplot as plt
import numpy as np


def read_six_axis_file(path: str) -> np.ndarray:
    """
    Read a comma-separated file where each line has six float numbers.
    Returns an array of shape (N, 6).
    """
    rows: List[List[float]] = []
    with open(path, "r", encoding="utf-8", errors="ignore") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            parts = [p for p in line.split(",") if p != ""]
            if len(parts) < 6:
                # skip invalid/incomplete lines silently
                continue
            try:
                row = [float(x) for x in parts[:6]]
                rows.append(row)
            except ValueError:
                # skip lines with non-numeric tokens
                continue
    if not rows:
        raise ValueError("未读取到任何有效数据，请检查输入文件格式。")
    return np.asarray(rows, dtype=float)


def build_time_axis(num_points: int, dt: float) -> np.ndarray:
    return np.arange(num_points, dtype=float) * dt


def plot_six_axes(
    t: np.ndarray, data: np.ndarray, title: str, save_path: str
) -> Tuple[plt.Figure, np.ndarray]:
    """
    Plot six channels over time. Data shape should be (N, 6).
    Saves figure to save_path. Returns fig and axes array.
    """
    fig, axes = plt.subplots(3, 2, figsize=(14, 8), sharex=True)
    force_labels = ["Fx(N)", "Fy(N)", "Fz(N)"]
    torque_labels = ["Tx(N·m)", "Ty(N·m)", "Tz(N·m)"]

    for row, label in enumerate(force_labels):
        ax = axes[row, 0]
        ax.plot(t, data[:, row], linewidth=1.0)
        ax.set_ylabel(label)
        ax.grid(True, linestyle="--", alpha=0.4)

    for row, label in enumerate(torque_labels):
        ax = axes[row, 1]
        ax.plot(t, data[:, row + 3], linewidth=1.0)
        ax.set_ylabel(label)
        ax.grid(True, linestyle="--", alpha=0.4)

    axes[2, 0].set_xlabel("Time (s)")
    axes[2, 1].set_xlabel("Time (s)")
    # fig.suptitle(title, fontsize=14)
    fig.tight_layout(rect=[0, 0.03, 1, 0.95])

    os.makedirs(os.path.dirname(save_path), exist_ok=True)
    fig.savefig(save_path, dpi=150)
    return fig, axes
def plot_force_torque_two_subplots(
    t: np.ndarray, data: np.ndarray, title: str, save_path: str
) -> Tuple[plt.Figure, np.ndarray]:
    """
    一张图两个子图：
    上图 Fx/Fy/Fz，下图 Tx/Ty/Tz
    """
    force_labels = ["Fx", "Fy", "Fz"]
    torque_labels = ["Tx", "Ty", "Tz"]

    fig, axes = plt.subplots(2, 1, figsize=(14, 8), sharex=True)

    # 上：力
    for i in range(3):
        axes[0].plot(t, data[:, i], linewidth=1.0, label=force_labels[i])
    axes[0].set_ylabel("Force(N)")
    axes[0].set_title("Force (Fx, Fy, Fz)")
    axes[0].grid(True, linestyle="--", alpha=0.4)
    axes[0].legend(loc="best")

    # 下：力矩
    for i in range(3, 6):
        axes[1].plot(t, data[:, i], linewidth=1.0, label=torque_labels[i - 3])
    axes[1].set_xlabel("Time (s)")
    axes[1].set_ylabel("Torque(N·m)")
    axes[1].set_title("Torque (Tx, Ty, Tz)")
    axes[1].grid(True, linestyle="--", alpha=0.4)
    axes[1].legend(loc="best")

    # fig.suptitle(title, fontsize=14)
    fig.tight_layout(rect=[0, 0.03, 1, 0.95])

    os.makedirs(os.path.dirname(save_path), exist_ok=True)
    fig.savefig(save_path, dpi=150)
    return fig, axes


def main() -> None:
    parser = argparse.ArgumentParser(
        description="读取每行六个逗号分隔的数据，按0.2s采样间隔画出六通道曲线并保存PNG。"
    )
    parser.add_argument("--input", "-i", required=False,
                        default=r"C:\Users\张皓鑫\Desktop\ORU数据\六维力数据\oru_force_3\oru_force_3.txt",
                        help="输入txt路径（每行六个数字）。")
    parser.add_argument(
        "--dt",
        type=float,
        default=0.2,
        help="相邻两条数据的时间间隔(秒)，默认0.2。",
    )
    parser.add_argument("--output", "-o",
                        default=r"C:\Users\张皓鑫\Desktop\ORU数据\六维力数据\oru_force_3\oru_force.png",
                        help="输出图片路径（.png）。")
    args = parser.parse_args()

    input_path = os.path.abspath(args.input)
    if not os.path.exists(input_path):
        raise FileNotFoundError(f"输入文件不存在: {input_path}")

    if args.output:
        save_path = os.path.abspath(args.output)
    else:
        root, _ = os.path.splitext(input_path)
        save_path = f"{root}_plot.png"

    data = read_six_axis_file(input_path)
    t = build_time_axis(data.shape[0], args.dt)

    title = os.path.basename(input_path)
    plot_six_axes(t, data, title, save_path)
    print(f"已保存图像 -> {save_path}")

    root, ext = os.path.splitext(save_path)
    save_path_two_subplots = f"{root}_force_torque_2subplots{ext}"
    plot_force_torque_two_subplots(t, data, title, save_path_two_subplots)
    print(f"已保存两子图图像 -> {save_path_two_subplots}")


if __name__ == "__main__":
    main()

