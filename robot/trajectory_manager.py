import numpy as np
import os
from typing import Tuple, Optional

class TrajectoryManager:
    def __init__(self):
        self.trajectory = None  # 存储轨迹数据，numpy数组 (N, 6)
        self.trajectory_file_path = None  # 文件路径
        self.trajectory_name = None  # 文件名
        self.trajectory_info = {}  # 轨迹信息字典

    def load_from_file(self, file_path: str) -> Tuple[bool, str]:
        """
        从TXT文件加载轨迹数据

        参数:
            file_path: 轨迹文件路径

        返回:
            (是否成功, 消息字符串)
        """
        # 1. 检查文件是否存在
        if not os.path.exists(file_path):
            return False, f"文件不存在: {file_path}"

        try:
            # 2. 读取文件内容
            raw_data = self._read_txt_file(file_path) #调用私有方法读取文件 返回二维列表

            # 3. 检查是否读取到数据 检查数据是否为空
            if raw_data is None or len(raw_data) == 0:
                return False, "文件为空或格式错误"

            # 4. 验证和转换数据
            success, validated_data, message = self._validate_and_convert(raw_data)#调用验证转换方法

            if not success:
                return False, message

            # 5. 保存轨迹数据
            self.trajectory = validated_data
            self.trajectory_file_path = file_path
            self.trajectory_name = os.path.basename(file_path) #提取文件名称

            # 6. 计算轨迹信息
            self._calculate_trajectory_info()#调用方法计算轨迹信息

            return True, f"成功加载轨迹: {self.trajectory_name}，共 {len(self.trajectory)} 个点"

        except Exception as e:
            return False, f"加载轨迹文件失败: {str(e)}"

    def _read_txt_file(self, file_path: str) -> Optional[list]:
        """
        读取TXT文件，解析每行的6个数据

        参数:
            file_path: 文件路径

        返回:
            二维列表，每行是一个包含6个浮点数的列表
            例如: [[x1,y1,z1,rx1,ry1,rz1], [x2,y2,z2,rx2,ry2,rz2], ...]
        """
        data = []#创建空列表 用于存储解析后的数据

        try:
            with open(file_path, 'r', encoding='utf-8') as f:#open打开文件 r 只读模式 as f 文件对象命名为f with 自动关闭文件
                for line_num, line in enumerate(f, start=1):#遍历文件 同时获取行号和内容
                    # 去除行首行尾的空白字符（空格、换行符等）
                    line = line.strip()#strip去掉首尾空白字符

                    # 跳过空行
                    if not line:
                        continue

                    # 跳过注释行（以#开头的行）
                    if line.startswith('#'):
                        continue

                    # 按逗号分割
                    parts = line.split(',')#split(',')按逗号分割

                    # 去除每个部分的首尾空格
                    parts = [part.strip() for part in parts]

                    # 检查是否有6个数据
                    if len(parts) != 6:
                        print(f"警告: 第{line_num}行数据格式错误，期望6个数据，实际{len(parts)}个，已跳过")
                        continue

                    # 尝试转换为浮点数
                    try:
                        row_data = [float(part) for part in parts]
                        data.append(row_data)
                    except ValueError as e:
                        print(f"警告: 第{line_num}行包含非数字数据，已跳过: {e}")
                        continue

            return data if data else None

        except Exception as e:
            print(f"读取文件错误: {e}")
            return None

    def _validate_and_convert(self, raw_data: list) -> Tuple[bool, Optional[np.ndarray], str]:
        """
        验证数据并转换为numpy数组

        参数:
            raw_data: 原始数据列表，每行6个数字

        返回:
            (是否成功, 转换后的numpy数组, 消息)
        """
        if not raw_data:
            return False, None, "数据为空"

        try:
            # 转换为numpy数组
            # raw_data是二维列表，例如: [[x1,y1,z1,rx1,ry1,rz1], [x2,y2,z2,rx2,ry2,rz2]]
            # 转换为numpy数组后形状是 (N, 6)
            data_array = np.array(raw_data, dtype=np.float64)

            # 检查数组形状
            if len(data_array.shape) != 2 or data_array.shape[1] != 6:
                return False, None, f"数据格式错误：期望形状为(N, 6)，实际为{data_array.shape}"

            return True, data_array, f"成功验证 {len(data_array)} 个位姿点"

        except Exception as e:
            return False, None, f"数据转换失败: {str(e)}"

    def _calculate_trajectory_info(self):
        """计算轨迹的统计信息"""
        if self.trajectory is None:
            return

        # 轨迹信息
        self.trajectory_info = {
            'num_points': len(self.trajectory),  # 点数
            'start_position': self.trajectory[0, :3].copy(),  # 起点位置 [x, y, z]
            'end_position': self.trajectory[-1, :3].copy(),  # 终点位置 [x, y, z]
            'start_orientation': self.trajectory[0, 3:].copy(),  # 起点姿态 [rx, ry, rz]
            'end_orientation': self.trajectory[-1, 3:].copy(),  # 终点姿态 [rx, ry, rz]
        }

        # 计算轨迹总长度（位置路径长度）
        positions = self.trajectory[:, :3]  # 提取所有位置 (N, 3)

        # 计算相邻点之间的距离
        # np.diff计算相邻元素的差值，得到 (N-1, 3)
        # np.linalg.norm计算每个差向量的长度，得到 (N-1,) 的距离数组
        distances = np.linalg.norm(np.diff(positions, axis=0), axis=1)

        # 累加所有距离得到总长度
        self.trajectory_info['total_length'] = np.sum(distances)

        # 计算平均步长
        if len(distances) > 0:
            self.trajectory_info['average_step_size'] = np.mean(distances)
        else:
            self.trajectory_info['average_step_size'] = 0.0

    def get_trajectory(self) -> Optional[np.ndarray]:
        """获取轨迹数据"""
        return self.trajectory

    def get_trajectory_info(self) -> dict:
        """获取轨迹信息"""
        return self.trajectory_info.copy() if self.trajectory_info else {}

    def clear_trajectory(self):
        """清除轨迹数据"""
        self.trajectory = None
        self.trajectory_file_path = None
        self.trajectory_name = None
        self.trajectory_info = {}

    def has_trajectory(self) -> bool:
        """检查是否有轨迹"""
        return self.trajectory is not None and len(self.trajectory) > 0

    def get_trajectory_name(self) -> Optional[str]:
        """获取轨迹文件名"""
        return self.trajectory_name