"""
机械臂坐标系转换示例代码
这个文件展示了常见的坐标系转换操作，帮助理解理论知识
"""

import numpy as np
from scipy.spatial.transform import Rotation as R
import math


class CoordinateTransformDemo:
    """坐标系转换演示类"""
    
    def __init__(self):
        pass
    
    # ==================== 1. 位姿表示方法转换 ====================
    
    def rotvec_to_rotation_matrix(self, rotvec):
        """
        旋转向量 → 旋转矩阵
        
        参数:
            rotvec: [rx, ry, rz] 旋转向量（弧度）
        返回:
            3×3旋转矩阵
        """
        r = R.from_rotvec(rotvec)
        return r.as_matrix()
    
    def rotation_matrix_to_rotvec(self, R_matrix):
        """
        旋转矩阵 → 旋转向量
        
        参数:
            R_matrix: 3×3旋转矩阵
        返回:
            [rx, ry, rz] 旋转向量（弧度）
        """
        r = R.from_matrix(R_matrix)
        return r.as_rotvec()
    
    def rpy_to_rotation_matrix(self, rpy):
        """
        RPY角 → 旋转矩阵 (ZYX顺序)
        
        参数:
            rpy: [roll, pitch, yaw] 弧度
        返回:
            3×3旋转矩阵
        """
        # 分别计算绕X、Y、Z轴的旋转矩阵
        Rx = np.array([[1, 0, 0],
                       [0, np.cos(rpy[0]), -np.sin(rpy[0])],
                       [0, np.sin(rpy[0]), np.cos(rpy[0])]])
        
        Ry = np.array([[np.cos(rpy[1]), 0, np.sin(rpy[1])],
                       [0, 1, 0],
                       [-np.sin(rpy[1]), 0, np.cos(rpy[1])]])
        
        Rz = np.array([[np.cos(rpy[2]), -np.sin(rpy[2]), 0],
                       [np.sin(rpy[2]), np.cos(rpy[2]), 0],
                       [0, 0, 1]])
        
        # ZYX顺序：先Z，再Y，最后X
        return Rz @ Ry @ Rx
    
    def rotation_matrix_to_rpy(self, R_matrix):
        """
        旋转矩阵 → RPY角 (ZYX顺序)
        
        参数:
            R_matrix: 3×3旋转矩阵
        返回:
            [roll, pitch, yaw] 弧度
        """
        # 从旋转矩阵提取RPY角
        sy = np.sqrt(R_matrix[0, 0]**2 + R_matrix[1, 0]**2)
        
        singular = sy < 1e-6  # 检查是否奇异
        
        if not singular:
            roll = np.arctan2(R_matrix[2, 1], R_matrix[2, 2])
            pitch = np.arctan2(-R_matrix[2, 0], sy)
            yaw = np.arctan2(R_matrix[1, 0], R_matrix[0, 0])
        else:
            roll = np.arctan2(-R_matrix[1, 2], R_matrix[1, 1])
            pitch = np.arctan2(-R_matrix[2, 0], sy)
            yaw = 0
        
        return np.array([roll, pitch, yaw])
    
    # ==================== 2. 齐次变换矩阵 ====================
    
    def pose_to_transform_matrix(self, pose):
        """
        位姿向量 → 齐次变换矩阵
        
        参数:
            pose: [x, y, z, rx, ry, rz]
                 位置(米) + 旋转向量(弧度)
        返回:
            4×4齐次变换矩阵
        """
        T = np.eye(4)
        
        # 提取位置
        T[0:3, 3] = pose[0:3]
        
        # 旋转向量转旋转矩阵
        rotvec = pose[3:6]
        T[0:3, 0:3] = self.rotvec_to_rotation_matrix(rotvec)
        
        return T
    
    def transform_matrix_to_pose(self, T):
        """
        齐次变换矩阵 → 位姿向量
        
        参数:
            T: 4×4齐次变换矩阵
        返回:
            [x, y, z, rx, ry, rz]
        """
        pose = np.zeros(6)
        
        # 提取位置
        pose[0:3] = T[0:3, 3]
        
        # 旋转矩阵转旋转向量
        R_matrix = T[0:3, 0:3]
        pose[3:6] = self.rotation_matrix_to_rotvec(R_matrix)
        
        return pose
    
    # ==================== 3. 坐标系转换 ====================
    
    def transform_point(self, T, point):
        """
        使用齐次变换矩阵转换点的坐标
        
        参数:
            T: 4×4变换矩阵 (从源坐标系到目标坐标系)
            point: [x, y, z] 点在源坐标系中的坐标
        返回:
            [x, y, z] 点在目标坐标系中的坐标
        """
        # 转换为齐次坐标
        point_homogeneous = np.array([point[0], point[1], point[2], 1])
        
        # 应用变换
        transformed = T @ point_homogeneous
        
        return transformed[0:3]
    
    def chain_transforms(self, *transforms):
        """
        链式组合多个变换矩阵
        
        参数:
            *transforms: 多个4×4变换矩阵
        返回:
            组合后的4×4变换矩阵
        
        示例:
            T_C_to_A = chain_transforms(T_C_to_B, T_B_to_A)
            表示: 从A到C的变换 = 从A到B的变换 × 从B到C的变换
        """
        result = np.eye(4)
        for T in transforms:
            result = result @ T
        return result
    
    def inverse_transform(self, T):
        """
        计算变换矩阵的逆（从目标坐标系到源坐标系）
        
        参数:
            T: 4×4变换矩阵 (从A到B)
        返回:
            4×4变换矩阵 (从B到A)
        """
        T_inv = np.eye(4)
        
        # 旋转矩阵的逆 = 转置
        R = T[0:3, 0:3]
        t = T[0:3, 3]
        
        T_inv[0:3, 0:3] = R.T
        T_inv[0:3, 3] = -R.T @ t
        
        return T_inv
    
    # ==================== 4. 实际应用示例 ====================
    
    def example_camera_to_base(self):
        """
        示例：相机坐标系到基坐标系的转换
        
        这个示例展示了视觉定位中的坐标转换流程
        """
        print("=" * 60)
        print("示例1：相机坐标系 → 基坐标系")
        print("=" * 60)
        
        # 1. 手眼标定参数：相机到工具的变换
        # 这是通过手眼标定得到的固定参数
        camera_to_tool_pose = np.array([
            0.0,      # x (米)
            0.061,    # y (米) - 相机在工具坐标系中的位置
            0.066,    # z (米)
            0.0,      # rx (弧度)
            0.0,      # ry (弧度)
            np.pi     # rz (弧度) - 相机绕Z轴旋转180度
        ])
        T_tool_to_camera = self.pose_to_transform_matrix(camera_to_tool_pose)
        T_camera_to_tool = self.inverse_transform(T_tool_to_camera)
        
        print(f"相机到工具的变换矩阵:\n{T_camera_to_tool}\n")
        
        # 2. 当前工具在基坐标系中的位姿（从机械臂获取）
        tool_in_base_pose = np.array([
            0.3,      # x (米)
            0.2,      # y (米)
            0.5,      # z (米)
            0.0,      # rx (弧度)
            0.0,      # ry (弧度)
            0.0       # rz (弧度)
        ])
        T_base_to_tool = self.pose_to_transform_matrix(tool_in_base_pose)
        
        print(f"工具在基坐标系中的位姿:\n{T_base_to_tool}\n")
        
        # 3. 物体在相机坐标系中的位置（从图像处理得到）
        object_in_camera = np.array([0.01, 0.02, 0.5])  # [x, y, z] (米)
        print(f"物体在相机坐标系中的位置: {object_in_camera}\n")
        
        # 4. 计算物体在基坐标系中的位置
        # 转换链：相机 → 工具 → 基坐标
        T_base_to_camera = T_base_to_tool @ T_camera_to_tool
        object_in_base = self.transform_point(T_base_to_camera, object_in_camera)
        
        print(f"物体在基坐标系中的位置: {object_in_base}\n")
        
        return object_in_base
    
    def example_force_transform(self):
        """
        示例：力传感器坐标系到基坐标系的转换
        
        这个示例展示了力控制中的坐标转换
        """
        print("=" * 60)
        print("示例2：力传感器坐标系 → 基坐标系")
        print("=" * 60)
        
        # 1. 传感器在基坐标系中的位姿
        sensor_in_base_pose = np.array([
            0.3,      # x (米)
            0.2,      # y (米)
            0.5,      # z (米)
            0.0,      # rx (弧度)
            0.0,      # ry (弧度)
            0.0       # rz (弧度)
        ])
        T_base_to_sensor = self.pose_to_transform_matrix(sensor_in_base_pose)
        
        # 2. 传感器测量的力（在传感器坐标系中）
        force_in_sensor = np.array([
            10.0,     # Fx (N)
            5.0,      # Fy (N)
            -2.0,     # Fz (N)
            0.1,      # Tx (Nm)
            0.05,     # Ty (Nm)
            0.0       # Tz (Nm)
        ])
        
        print(f"传感器坐标系中的力: {force_in_sensor}\n")
        
        # 3. 转换到基坐标系
        force_in_base = self.transform_force(T_base_to_sensor, force_in_sensor)
        
        print(f"基坐标系中的力: {force_in_base}\n")
        
        return force_in_base
    
    def transform_force(self, T, force_sensor):
        """
        将6维力/力矩从传感器坐标系转换到基坐标系
        
        参数:
            T: 传感器在基坐标系中的变换矩阵
            force_sensor: [Fx, Fy, Fz, Tx, Ty, Tz] 在传感器坐标系中
        返回:
            [Fx, Fy, Fz, Tx, Ty, Tz] 在基坐标系中
        """
        # 提取旋转矩阵和平移向量
        R = T[0:3, 0:3]
        t = T[0:3, 3]
        
        # 提取力和力矩
        F_sensor = force_sensor[0:3]
        T_sensor = force_sensor[3:6]
        
        # 力只需要旋转
        F_base = R @ F_sensor
        
        # 力矩需要考虑力的作用点
        # T_base = R @ T_sensor + t × (R @ F_sensor)
        t_cross_F = np.cross(t, F_base)
        T_base = R @ T_sensor + t_cross_F
        
        return np.concatenate([F_base, T_base])
    
    def example_relative_motion(self):
        """
        示例：在工具坐标系中定义相对运动
        
        这个示例展示了如何在工具坐标系中定义偏移，然后转换到基坐标系
        """
        print("=" * 60)
        print("示例3：工具坐标系中的相对运动")
        print("=" * 60)
        
        # 1. 当前工具在基坐标系中的位姿
        current_tool_pose = np.array([
            0.3, 0.2, 0.5,  # 位置
            0.0, 0.0, 0.0   # 姿态
        ])
        T_base_to_tool = self.pose_to_transform_matrix(current_tool_pose)
        
        # 2. 在工具坐标系中定义偏移（例如：沿工具Z轴向前移动0.1米）
        offset_in_tool = np.array([0.0, 0.0, 0.1])  # 在工具坐标系中
        
        # 3. 将偏移转换到基坐标系
        offset_in_base = self.transform_point(T_base_to_tool, offset_in_tool)
        
        print(f"当前工具位置: {current_tool_pose[0:3]}")
        print(f"工具坐标系中的偏移: {offset_in_tool}")
        print(f"基坐标系中的偏移: {offset_in_base}")
        print(f"新位置: {current_tool_pose[0:3] + offset_in_base}\n")
        
        return current_tool_pose[0:3] + offset_in_base


# ==================== 主函数：运行示例 ====================

if __name__ == "__main__":
    demo = CoordinateTransformDemo()
    
    print("\n" + "=" * 60)
    print("机械臂坐标系转换示例代码")
    print("=" * 60 + "\n")
    
    # 运行示例
    demo.example_camera_to_base()
    print("\n")
    
    demo.example_force_transform()
    print("\n")
    
    demo.example_relative_motion()
    print("\n")
    
    # ==================== 额外练习 ====================
    print("=" * 60)
    print("练习：理解变换链")
    print("=" * 60)
    
    # 创建几个变换矩阵
    T_A_to_B = np.eye(4)
    T_A_to_B[0:3, 3] = [1, 0, 0]  # B在A中沿X轴偏移1米
    
    T_B_to_C = np.eye(4)
    T_B_to_C[0:3, 3] = [0, 1, 0]  # C在B中沿Y轴偏移1米
    
    # 计算从A到C的变换
    T_A_to_C = demo.chain_transforms(T_A_to_B, T_B_to_C)
    
    print(f"从A到B的变换:\n{T_A_to_B}\n")
    print(f"从B到C的变换:\n{T_B_to_C}\n")
    print(f"从A到C的变换（链式）:\n{T_A_to_C}\n")
    
    # 验证：点P在A坐标系中为[0,0,0]，在C坐标系中应该是[-1,-1,0]
    point_in_A = np.array([0, 0, 0])
    point_in_C = demo.transform_point(T_A_to_C, point_in_A)
    print(f"点P在A坐标系中: {point_in_A}")
    print(f"点P在C坐标系中: {point_in_C}")
    print("(注意：这里使用了逆变换，所以结果是负的)\n")
    
    print("=" * 60)
    print("示例代码运行完成！")
    print("=" * 60)
