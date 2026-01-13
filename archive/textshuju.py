import random
import time


def generate_random_force():
    """
    随机生成一帧六维力数据

    返回：
        list: [Fx, Fy, Fz, Tx, Ty, Tz]
              Fx, Fy, Fz: 力（单位：N，牛顿）
              Tx, Ty, Tz: 力矩（单位：Nm，牛米）
    """
    # 模拟真实的力传感器数据范围：
    # 力：通常在 -10N 到 10N 之间
    # 力矩：通常在 -1Nm 到 1Nm 之间

    Fx = round(random.uniform(-10.0, 10.0), 3)  # X方向力，保留3位小数
    Fy = round(random.uniform(-10.0, 10.0), 3)  # Y方向力
    Fz = round(random.uniform(-10.0, 10.0), 3)  # Z方向力
    Tx = round(random.uniform(-1.0, 1.0), 3)  # X方向力矩
    Ty = round(random.uniform(-1.0, 1.0), 3)  # Y方向力矩
    Tz = round(random.uniform(-1.0, 1.0), 3)  # Z方向力矩

    return [Fx, Fy, Fz, Tx, Ty, Tz]


# ================== 使用示例 ==================
if __name__ == "__main__":
    # 示例1：生成单次数据
    print("示例1：生成单次六维力数据")
    force_data = generate_random_force()
    print(f"生成的六维力数据: {force_data}")
    print()

    # 示例2：循环生成多帧数据（模拟实时采集）
    print("示例2：循环生成10帧数据（模拟实时采集）")
    for i in range(10):
        force_data = generate_random_force()
        print(f"第 {i + 1} 帧: {force_data}")
        time.sleep(0.1)  # 模拟每100ms采集一次