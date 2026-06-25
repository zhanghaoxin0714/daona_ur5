import argparse
import os
import re
from typing import List

import matplotlib.pyplot as plt
import numpy as np


FLOAT_PATTERN = re.compile(r"[-+]?(?:\d*\.\d+|\d+)(?:[eE][-+]?\d+)?")


def read_single_column_floats(path: str) -> np.ndarray:
    """
    Read a text file with one number per line (or any text containing numbers).
    Returns a 1D numpy array of floats.
    """
    values: List[float] = []
    with open(path, "r", encoding="utf-8", errors="ignore") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            m = FLOAT_PATTERN.findall(line)
            if not m:
                continue
            # If a line has multiple numbers, we take the first one.
            values.append(float(m[0]))

    if not values:
        raise ValueError(f"未读取到任何数值: {path}")
    return np.asarray(values, dtype=float)


def main() -> None:
    parser = argparse.ArgumentParser(
        description="读取单列电路/力矩数据，按 1ms 生成 Time(s) 曲线，并将数据除以100得到 T(N·m)。"
    )
    parser.add_argument(
        "--input",
        "-i",
        default=r"e:\xwechat_files\wxid_9azmddaece6z21_4377\msg\file\2026-04\液路.txt",
        help="输入txt路径（单列数字）。",
    )
    parser.add_argument(
        "--output",
        "-o",
        default=r"C:\Users\张皓鑫\Desktop\ORU数据\力矩\液路.png",
        help="输出png路径（默认在输入同目录，文件名加 _torque.png）。",
    )
    parser.add_argument(
        "--dt",
        type=float,
        default=0.001,
        help="采样间隔(s)，默认1ms=0.001。",
    )
    parser.add_argument(
        "--scale",
        type=float,
        default=100.0,
        help="用于换算：画图时 y = raw / scale，默认 scale=100。",
    )
    args = parser.parse_args()

    input_path = os.path.abspath(args.input)
    if not os.path.exists(input_path):
        raise FileNotFoundError(f"输入文件不存在: {input_path}")

    if args.output.strip():
        output_path = os.path.abspath(args.output)
    else:
        root, ext = os.path.splitext(input_path)
        output_path = f"{root}_torque.png"

    raw = read_single_column_floats(input_path)
    t = np.arange(raw.shape[0], dtype=float) * args.dt
    y = raw / args.scale

    plt.figure(figsize=(12, 4.5))
    plt.plot(t, y, linewidth=1.0)
    plt.xlabel("Time (s)")
    plt.ylabel("T (N·m)")
    plt.grid(True, linestyle="--", alpha=0.4)
    plt.tight_layout()

    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    plt.savefig(output_path, dpi=150)
    print(f"已保存曲线图 -> {output_path}")


if __name__ == "__main__":
    main()

