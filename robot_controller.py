# robot_controller.py
import numpy as np
import time
import copy
from scipy.spatial.transform import Rotation
from TransPose import TransPose
from Robots.URControlAPI import URControlAPI
from force_controller import ForceController
from controller.AdmittancePoseController import AdmittancePoseController

class RobotController(URControlAPI):
    """独立的机械臂控制器 - 只迁移ad_control中的部分"""

    def __init__(self, HOST):
        # 调用父类URControlAPI的构造函数
        super().__init__(HOST)
        time.sleep(1)
        # 初始化状态变量
        self.pose = None
        self.ee_velocity = None
        self.f_base = None

        # 初始化导纳控制相关变量（但不创建实例）
        self.dt = 0.005  # 采样率200Hz
        self.adcontrol = None  # 导纳控制器实例，启动时创建
        self.force_controller = None  # 力传感器控制器引用
        # 更新状态
        self.update_status()


    def get_end_tip(self):
        """
        获得工件末端的坐标，即加上了偏差
        """
        pose = np.array(self.get_ee_pose())
        T1 = TransPose.getT_fromRotvec(pose)
        #T = T1 @ self.pegtip_deltaT
        end_tip = TransPose.getRotvec_fromT(T1)
        return end_tip

    def go_startPose(self, start_point):#机械臂移动到目标位置
        speed = 0.01
        acc = 0.003
        self.moveL(start_point, asy=False, speed=speed, acc=acc)#moveL是直线运动 asy=False 同步执行模式 机械臂会等待运动完成才会返回

    def pose_stable(self, M, B, K):
        """
        定点位姿导纳控制
        """
        if self.adcontrol is None:
            raise RuntimeError("导纳控制器未初始化，请先启动导纳控制")

        self.update_status()  #更新当前状态
        #计算导纳控制增量
        control_e = self.adcontrol.cal_increment(M=M, B=B, K=K, ft=self.f_base, pose=self.pose, pose_target=self.pose)#调用导纳控制算法 计算导纳控制增量
        self.control = copy.deepcopy(self.pose) #计算新的控制位置
        self.control = self.add_pose_increment(self.control, control_e)
        self.servoL(self.control)

    def update_status(self):
        """
        更新当前位姿和力
        """
        # print("------------------------------------------------")
        self.pose = np.array(self.get_ee_pose())#获取当前位姿
        self.ee_velocity = np.array(self.get_ee_velocity())#获取当前速度
        # print('pose:', self.pose)
        self.f_base = np.array(self.get_cur_force(0))#获取当前力
        # print('f_base:', self.f_base)

    def add_pose_increment(self, control, control_e):
        """
        增量式的位姿控制
        """
        control = np.array(control).reshape(6, 1)#当前位姿
        control_e = np.array(control_e).reshape(6, 1)#位姿增量
        #分离位置和姿态
        trans_pre = control[:3].reshape(-1)#当前位置
        rotvec_pre = control[3:].reshape(-1)#当前姿态
        trans_e = control_e[:3].reshape(-1)#位置增量
        rotvec_e = control_e[3:].reshape(-1)#位姿增量

        # 姿态增量计算
        R_pre = Rotation.from_rotvec(rotvec_pre).as_matrix()#当前旋转矩
        R_e = Rotation.from_rotvec(rotvec_e).as_matrix()#增量旋转矩阵
        R_next = R_e @ R_pre#矩阵乘法
        rotvec_next = Rotation.from_matrix(R_next).as_rotvec().reshape(-1)#转换回旋转向量

        # 位置增量
        trans_next = trans_pre + trans_e#位置直接相加
        #组合新的位姿
        pose_next = np.array([trans_next, rotvec_next]).reshape(-1)
        return pose_next

    def set_force_controller(self, force_controller):
        """设置力传感器控制器"""
        self.force_controller = force_controller

    def get_cur_force(self, mode=0, for_display=False):
        """获取当前力传感器数据
        Args:
            mode: 数据模式
            for_display: True=显示用原始数据, False=控制用滤波数据
        """
        if self.force_controller is not None:
            return self.force_controller.get_cur_force(mode=mode, robot_controller=self, for_display=for_display)
        else:
            return np.zeros(6)