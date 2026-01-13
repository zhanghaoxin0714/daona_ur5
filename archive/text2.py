# -*- coding: utf-8 -*-
"""
脚本2：使用 ForceTxtLogger 记录保存六维力数据
功能：将生成的六维力数据保存到 txt 文件中
"""

import os
from datetime import datetime
import numpy as np
import random  # 用于生成模拟数据
import time  # 用于模拟采集间隔


# 导入 ForceTxtLogger 类（你需要把之前提供的 ForceTxtLogger 类代码也放在这个文件里，或者单独导入）
# 这里我直接把 ForceTxtLogger 类也写在这里，方便你使用

class ForceTxtLogger:
    """
    简单的六维力 txt 记录器：
        - 每一帧六维力数据写一行到 txt 文件
        - 一行的格式大致是：[[Fx Fy Fz Tx Ty Tz]]
    """

    def __init__(self, save_dir="force_logs"):
        """
        初始化函数，在你创建 ForceTxtLogger(...) 实例时自动执行

        参数：
            save_dir: 保存 txt 文件的文件夹名（默认是 "force_logs"）
        """
        self.save_dir = save_dir

        # 1) 如果保存目录不存在，就自动创建
        os.makedirs(self.save_dir, exist_ok=True)

        # 2) 生成一个带时间戳的文件名，例如：force_20250101_153045.txt
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")  # 当前时间 → 字符串
        filename = f"force_{timestamp}.txt"  # 拼成文件名
        self.filepath = os.path.join(self.save_dir, filename)  # 拼出完整路径

        # 3) 以"追加 + 读写"的方式打开文件
        #    "a+" 的含义：
        #       - 如果文件不存在，就创建文件
        #       - 如果已存在，就在文件末尾追加写入
        #    encoding="utf-8" 是设置文件的编码格式
        self.f = open(self.filepath, "a+", encoding="utf-8")

        print(f"六维力日志文件已创建/打开: {self.filepath}")

    def log_force(self, ft):
        """
        记录一帧六维力数据

        参数：
            ft: 长度为 6 的 list 或 numpy 数组
                [Fx, Fy, Fz, Tx, Ty, Tz]
        使用方式：
            - 每当你从 TwinCAT 或其他地方得到一帧六维力，就调用一次这个函数
        """
        # 1) 把传进来的 ft 转成 numpy 数组，并 reshape 成 1 行 6 列（1x6）
        #    这样做的原因是：
        #       - 和你当前项目里的写法保持一致（f"{ft_display.T}\\n"）
        #       - 方便后续统一处理
        arr = np.array(ft, dtype=float).reshape(1, 6)  # 形状：(1, 6)

        # 2) 把这个 1x6 的数组转成字符串，并在末尾加上换行符 "\n"
        #    举例：
        #       如果 arr = [[1.0, 0.0, -3.0, 0.0, 0.0, 0.0]]
        #       那 line 大概长这样：'[[ 1.  0. -3.  0.  0.  0.]]\n'
        line = f"{arr}\n"

        # 3) 写入到文件
        self.f.write(line)

        # 4) 如果你想"实时"把数据刷到硬盘，而不是等缓冲区满了再写，
        #    可以取消下面这一行的注释：
        # self.f.flush()

    def close(self):
        """
        关闭文件

        使用时机：
            - 当你不再需要继续记录六维力（程序结束、或一段测试结束）时，
              一定要调用一次 close()，把文件句柄关掉，防止文件损坏。
        """
        if self.f:  # 如果文件句柄还存在
            self.f.close()  # 关闭文件
            self.f = None  # 把句柄置空，防止重复关闭


# ================== 生成模拟数据的函数 ==================
def generate_random_force():
    """
    随机生成一帧六维力数据

    返回：
        list: [Fx, Fy, Fz, Tx, Ty, Tz]
    """
    Fx = round(random.uniform(-10.0, 10.0), 3)  # X方向力
    Fy = round(random.uniform(-10.0, 10.0), 3)  # Y方向力
    Fz = round(random.uniform(-10.0, 10.0), 3)  # Z方向力
    Tx = round(random.uniform(-1.0, 1.0), 3)  # X方向力矩
    Ty = round(random.uniform(-1.0, 1.0), 3)  # Y方向力矩
    Tz = round(random.uniform(-1.0, 1.0), 3)  # Z方向力矩

    return [Fx, Fy, Fz, Tx, Ty, Tz]


# ================== 主程序：模拟采集并保存数据 ==================
if __name__ == "__main__":
    # 步骤1：创建日志记录器
    # 参数 "textshuju" 是你新建的文件夹名，所有 txt 文件会保存在这个文件夹里
    logger = ForceTxtLogger(save_dir="textshuju")

    # 步骤2：模拟采集多帧数据并保存
    print("\n开始模拟采集六维力数据...")
    total_frames = 20  # 总共采集 20 帧数据（你可以改成任意数字）

    for i in range(total_frames):
        # 2.1) 生成一帧模拟的六维力数据
        force_data = generate_random_force()

        # 2.2) 打印到控制台（方便你看到数据）
        print(f"第 {i + 1}/{total_frames} 帧: {force_data}")

        # 2.3) 保存到 txt 文件
        logger.log_force(force_data)

        # 2.4) 模拟采集间隔（每 100ms 采集一次，你可以改成其他值）
        time.sleep(0.1)

    # 步骤3：关闭文件
    logger.close()
    print(f"\n数据采集完成！共保存了 {total_frames} 帧数据")
    print(f"文件保存在: {logger.filepath}")