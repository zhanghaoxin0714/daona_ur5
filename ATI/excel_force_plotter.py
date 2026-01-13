#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
简化版六维力Excel数据图像生成器
快速将Excel数据生成图像
"""

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import os


def plot_force_data(excel_file, save_dir='plots'):
    """
    快速生成六维力数据图像

    参数:
    excel_file: Excel文件路径
    save_dir: 图像保存目录
    """

    # 设置中文字体
    plt.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei']
    plt.rcParams['axes.unicode_minus'] = False

    # 读取Excel数据
    print(f"📁 读取Excel文件: {excel_file}")
    data = pd.read_excel(excel_file)
    print(f"✅ 数据加载成功，共 {len(data)} 行数据")

    # 创建保存目录
    if not os.path.exists(save_dir):
        os.makedirs(save_dir)

    # 定义六维力列名
    force_columns = ['Fx', 'Fy', 'Fz', 'Mx', 'My', 'Mz']

    # 1. 生成时间序列图
    print("📈 生成时间序列图...")
    fig, axes = plt.subplots(2, 3, figsize=(15, 10))
    fig.suptitle('六维力时间序列图', fontsize=16, fontweight='bold')

    for i, col in enumerate(force_columns):
        row = i // 3
        col_idx = i % 3
        ax = axes[row, col_idx]

        if col in data.columns:
            ax.plot(data[col], linewidth=1.5)
            ax.set_title(f'{col} 时间序列')
            ax.set_xlabel('时间点')
            ax.set_ylabel('力值 (N)')
            ax.grid(True, alpha=0.3)
        else:
            ax.text(0.5, 0.5, f'未找到 {col} 列', ha='center', va='center')
            ax.set_title(f'{col} (缺失)')

    plt.tight_layout()
    plt.savefig(os.path.join(save_dir, 'force_time_series.png'), dpi=300, bbox_inches='tight')
    plt.show()

    # 2. 生成分布直方图
    print("📊 生成分布直方图...")
    fig, axes = plt.subplots(2, 3, figsize=(15, 10))
    fig.suptitle('六维力分布直方图', fontsize=16, fontweight='bold')

    for i, col in enumerate(force_columns):
        row = i // 3
        col_idx = i % 3
        ax = axes[row, col_idx]

        if col in data.columns:
            ax.hist(data[col], bins=30, alpha=0.7, edgecolor='black')
            ax.set_title(f'{col} 分布')
            ax.set_xlabel('力值 (N)')
            ax.set_ylabel('频次')
            ax.grid(True, alpha=0.3)
        else:
            ax.text(0.5, 0.5, f'未找到 {col} 列', ha='center', va='center')
            ax.set_title(f'{col} (缺失)')

    plt.tight_layout()
    plt.savefig(os.path.join(save_dir, 'force_histogram.png'), dpi=300, bbox_inches='tight')
    plt.show()

    # 3. 生成3D轨迹图（如果有Fx, Fy, Fz）
    if all(col in data.columns for col in ['Fx', 'Fy', 'Fz']):
        print("🌐 生成3D轨迹图...")
        fig = plt.figure(figsize=(10, 8))
        ax = fig.add_subplot(111, projection='3d')

        ax.plot(data['Fx'], data['Fy'], data['Fz'], linewidth=2, alpha=0.8)
        ax.scatter(data['Fx'].iloc[0], data['Fy'].iloc[0], data['Fz'].iloc[0],
                   color='green', s=100, label='起点')
        ax.scatter(data['Fx'].iloc[-1], data['Fy'].iloc[-1], data['Fz'].iloc[-1],
                   color='red', s=100, label='终点')

        ax.set_xlabel('Fx (N)')
        ax.set_ylabel('Fy (N)')
        ax.set_zlabel('Fz (N)')
        ax.set_title('三维力轨迹图')
        ax.legend()

        plt.savefig(os.path.join(save_dir, 'force_3d_trajectory.png'), dpi=300, bbox_inches='tight')
        plt.show()

    print(f"✅ 所有图像已保存到: {save_dir}")


def quick_plot(excel_file):
    """一键生成所有图像"""
    plot_force_data(excel_file)


if __name__ == "__main__":
    # 使用示例
    excel_file = input("请输入Excel文件路径: ").strip()

    if os.path.exists(excel_file):
        quick_plot(excel_file)
    else:
        print(f"❌ 文件不存在: {excel_file}")