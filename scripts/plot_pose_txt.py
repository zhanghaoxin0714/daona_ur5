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
                continue
            try:
                row = [float(x) for x in parts[:6]]
                rows.append(row)
            except ValueError:
                continue
    if not rows:
        raise ValueError("未读取到任何有效数据，请检查输入文件格式。")
    return np.asarray(rows, dtype=float)


def build_time_axis(num_points: int, dt: float) -> np.ndarray:
    return np.arange(num_points, dtype=float) * dt


def plot_six_subplots(t: np.ndarray, data: np.ndarray, save_path: str) -> Tuple[plt.Figure, np.ndarray]:
    """
    One figure with 6 subplots (3x2), each subplot for one pose channel.
    """
    fig, axes = plt.subplots(3, 2, figsize=(14, 8), sharex=True)
    left_labels = ["X", "Y", "Z"]
    right_labels = ["Rx", "Ry", "Rz"]

    for row, label in enumerate(left_labels):
        ax = axes[row, 0]
        ax.plot(t, data[:, row], linewidth=1.0)
        ax.set_ylabel(label)
        ax.grid(True, linestyle="--", alpha=0.4)

    for row, label in enumerate(right_labels):
        ax = axes[row, 1]
        ax.plot(t, data[:, row + 3], linewidth=1.0)
        ax.set_ylabel(label)
        ax.grid(True, linestyle="--", alpha=0.4)

    axes[2, 0].set_xlabel("Time (s)")
    axes[2, 1].set_xlabel("Time (s)")
    fig.tight_layout()

    os.makedirs(os.path.dirname(save_path), exist_ok=True)
    fig.savefig(save_path, dpi=150)
    return fig, axes


def plot_all_in_one(t: np.ndarray, data: np.ndarray, save_path: str) -> plt.Figure:
    """
    One figure with all 6 pose curves in one subplot.
    """
    channel_labels = ["X", "Y", "Z", "Rx", "Ry", "Rz"]

    fig, ax = plt.subplots(1, 1, figsize=(14, 6))
    for i in range(6):
        ax.plot(t, data[:, i], linewidth=1.0, label=channel_labels[i])

    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Pose value")
    ax.grid(True, linestyle="--", alpha=0.4)
    ax.legend(loc="best")
    fig.tight_layout()

    os.makedirs(os.path.dirname(save_path), exist_ok=True)
    fig.savefig(save_path, dpi=150)
    return fig


def plot_xyz_rpy_two_subplots(
    t: np.ndarray, data: np.ndarray, save_path: str
) -> Tuple[plt.Figure, np.ndarray]:
    """
    One figure with 2 subplots:
    top for X/Y/Z, bottom for Rx/Ry/Rz.
    """
    fig, axes = plt.subplots(2, 1, figsize=(14, 8), sharex=True)

    for i, label in enumerate(["X", "Y", "Z"]):
        axes[0].plot(t, data[:, i], linewidth=1.0, label=label)
    axes[0].set_ylabel("Position(m)")
    axes[0].grid(True, linestyle="--", alpha=0.4)
    axes[0].legend(loc="best")

    for i, label in zip(range(3, 6), ["Rx", "Ry", "Rz"]):
        axes[1].plot(t, data[:, i], linewidth=1.0, label=label)
    axes[1].set_xlabel("Time (s)")
    axes[1].set_ylabel("Rotation(rad)")
    axes[1].grid(True, linestyle="--", alpha=0.4)
    axes[1].legend(loc="best")

    fig.tight_layout()

    os.makedirs(os.path.dirname(save_path), exist_ok=True)
    fig.savefig(save_path, dpi=150)
    return fig, axes


def main() -> None:
    parser = argparse.ArgumentParser(
        description="读取机械臂位姿六列数据，输出六子图、XYZ+RxRyRz两子图、单图六曲线三种PNG。"
    )
    parser.add_argument(
        "--input",
        "-i",
        required=False,
        default=r"C:\Users\张皓鑫\Desktop\ORU数据\位姿\oru_pose.txt",
        help="输入txt路径（每行六个数字，逗号分隔）。",
    )
    parser.add_argument(
        "--dt",
        type=float,
        default=0.2,
        help="相邻两条数据的时间间隔(秒)，默认0.2。",
    )
    parser.add_argument(
        "--output",
        "-o",
        default=r"C:\Users\张皓鑫\Desktop\ORU数据\位姿\oru_pose_6subplots.png",
        help="六子图输出路径（.png）。",
    )
    args = parser.parse_args()

    input_path = os.path.abspath(args.input)
    if not os.path.exists(input_path):
        raise FileNotFoundError(f"输入文件不存在: {input_path}")

    if args.output:
        save_path_6subplots = os.path.abspath(args.output)
    else:
        root, _ = os.path.splitext(input_path)
        save_path_6subplots = f"{root}_6subplots.png"

    data = read_six_axis_file(input_path)
    t = build_time_axis(data.shape[0], args.dt)

    plot_six_subplots(t, data, save_path_6subplots)
    print(f"已保存六子图图像 -> {save_path_6subplots}")

    root, ext = os.path.splitext(save_path_6subplots)
    save_path_two_subplots = f"{root}_xyz_rpy_2subplots{ext}"
    plot_xyz_rpy_two_subplots(t, data, save_path_two_subplots)
    print(f"已保存两子图图像 -> {save_path_two_subplots}")

    save_path_all_in_one = f"{root}_all_in_one{ext}"
    plot_all_in_one(t, data, save_path_all_in_one)
    print(f"已保存单图六曲线图像 -> {save_path_all_in_one}")


if __name__ == "__main__":
    main()

