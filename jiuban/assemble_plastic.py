# -*- coding: utf-8 -*-
import copy
import os
import sys
import threading
import time
import numpy as np

from ati_force_sensor.ForceUpdateThread import ForceUpdateThread
from controller.AdmittancePoseController import AdmittancePoseController
from TransPose import TransPose
from scipy.spatial.transform import Rotation
from Robots.URControlAPI import URControlAPI

__dir__ = os.path.dirname(os.path.abspath(__file__))
sys.path.append(__dir__)
sys.path.append(
    os.path.abspath(os.path.join(__dir__, "ati_force_sensor/rpi_ati_net_ft/"))
)

UR_HOST = "192.168.111.10"
FT_HOST = "192.168.111.20"
PORT = 63352


class ad_control(URControlAPI):
    """
    测试一下阻抗控制的代码，同时方便测力传感器相对于机械臂末端的偏置?
    """

    def __init__(self, HOST, auto_connect_force_sensor=False):
        # super().__init__(HOST)
        super(ad_control, self).__init__(HOST)  # 进入父类URControlAPI构造函数



        # 只有在需要时才连接力传感器
        #if auto_connect_force_sensor: #目前是False不运行
        #    self.connect_force_sensor() #连接力传感器

        time.sleep(1)

        # 初始化变量
        self.dt = 0.005  # 采样率200Hz

        # 定义一些量，这些量是依赖于机器人的
        self.delta_ur_sensor = np.array([0, 0, 35 / 1000, 0, 0, -0.262])  # 传感器相对于机械臂末端偏置
        self.sensor_deltaT = TransPose.getT_fromRotvec(self.delta_ur_sensor)#传感器偏移的变换矩阵
        self.delta_ur_pegtip = np.array([0, 0, 35 / 1000, 0, 0, -0.262])#工具末端相对于传感器末端的偏移量
        self.pegtip_deltaT = TransPose.getT_fromRotvec(self.delta_ur_pegtip)#工具末端便宜的变换矩阵
        #为导纳控制做坐标变换计算
        # T为4*4的位姿变换矩阵
        self.target_tip = np.array(object=[78.57 / 1000, 397.57 / 1000, 226.95 / 1000, 3.140, -0.009, 0.001])
        T1 = TransPose.getT_fromRotvec(self.target_tip)  # 负载到末端的变换矩阵
        T = T1 @ self.pegtip_deltaT  # @表示矩阵乘法 考虑工具偏移
        self.target_hole = TransPose.getRotvec_fromT(T)
        # 只有在力传感器连接时才获取初始力
        if self.ati_force is not None:
            self.force_init = self.get_cur_force()  # 基坐标系下的六维力
        else:
            self.force_init = np.zeros(6)

        # 导纳初始位姿
        self.init_pos = np.array([0.27396, -0.17934, 0.416, 0.688, -3.049, 0.033])  # 单位：m；rad

        self.update_status() #更新状态
        self.adcontrol = AdmittancePoseController(self.pose, self.dt)#创建导纳控制器

    def get_end_tip(self):
        """
        获得工件末端的坐标，即加上了偏差
        """
        pose = np.array(self.get_ee_pose())
        T1 = TransPose.getT_fromRotvec(pose)
        T = T1 @ self.pegtip_deltaT
        end_tip = TransPose.getRotvec_fromT(T)
        return end_tip

    def connect_force_sensor(self, force_host="192.168.111.20"):
        """单独连接力传感器"""
        if self.ati_force is None:#检查力传感器连接状态
            try:
                self.ati_force = ForceUpdateThread()#创建线程对象 工作逻辑 力传感器数据获取的线程
                #创建线程
                self.force_thread = threading.Thread(
                    target=self.ati_force.thread_job,#线程要执行的函数 工作内容
                    daemon=True,
                    kwargs={"force_host": force_host} #传递给线程函数的参数
                )
                self.force_thread.setDaemon(True)
                #启动线程
                self.force_thread.start()
                print("force thread open")
                time.sleep(1)

                # 获取初始力
                self.force_init = self.get_cur_force()

                return True
            except Exception as e:
                print(f"连接力传感器失败: {e}")
                self.ati_force = None
                self.force_thread = None
                return False
        else:
            print("力传感器已连接")
            return False

    def disconnect_force_sensor(self):
        """断开力传感器连接"""
        if self.ati_force is not None:
            try:
                # 停止力传感器线程
                if hasattr(self.ati_force, 'stop_thread'):
                    self.ati_force.stop_thread()
                self.ati_force = None
                self.force_thread = None
                print("力传感器已断开")
                return True
            except Exception as e:
                print(f"断开力传感器失败: {e}")
                return False
        return False

    def is_force_sensor_connected(self):
        """检查力传感器是否已连接"""
        return self.ati_force is not None

    def move_up(self):
        pose = np.array(self.get_ee_pose())
        pose[2] += 40 / 1000
        self.moveL(pose, False)

    def rotate_angle(self, angle, zheng=True):
        """
        绕着末端进行旋转
        """
        times = int(angle / 0.1)

        for _ in range(times):
            T = TransPose.getT_fromRotvec(self.get_ee_pose())
            T = T @ self.pegtip_deltaT
            # pdb.set_trace()
            if zheng is True:
                T = T @ TransPose.getT_fromRotvec(np.array([0, 0, 0, 0, 0.1 / 180 * np.pi, 0]))
            else:
                T = T @ TransPose.getT_fromRotvec(np.array([0, 0, 0, 0, -0.1 / 180 * np.pi, 0]))
            T1 = T @ self.pegtip_deltaT.I

            end_pose = TransPose.getRotvec_fromT(T1)
            self.servoL(end_pose)
            time.sleep(0.1)

    def get_cur_force(self, mode=0, ref=np.zeros(6)):
        """
        mode=0，代表直接正常读取
        mode=1，代表所有传感器数据直接清零
        mode=2，代表给某一个方向一个虚拟力
        mode=3，代表去除偏差（此时ref必须传入进来，否则与mode=0效果一样）
        """
        if self.ati_force is None:
            print("警告：力传感器未连接，返回零力")
            return np.zeros(6)

        tcp_pose = self.get_ee_pose()#获取机械臂位姿
        tcp_T = TransPose.getT_fromRotvec(tcp_pose)#将位姿转换为变换矩阵
        sensor_T = tcp_T @ self.sensor_deltaT  #计算完整的传感器变换矩阵

        if mode == 0:
            force_cur_tcp = np.array(
                [
                    self.ati_force.current_force.fx,
                    self.ati_force.current_force.fy,
                    self.ati_force.current_force.fz,
                    self.ati_force.current_force.tx,
                    self.ati_force.current_force.ty,
                    self.ati_force.current_force.tz,
                ]
            ).reshape(-1)
        elif mode == 1:
            force_cur_tcp = np.array([0 for _ in range(6)]).reshape(-1)
        elif mode == 2:
            force_cur_tcp = np.array([0 for _ in range(6)]).reshape(-1)
            force_cur_tcp[2] -= 10
        elif mode == 3:
            force_cur_tcp = np.array(
                [
                    self.ati_force.current_force.fx,
                    self.ati_force.current_force.fy,
                    self.ati_force.current_force.fz,
                    self.ati_force.current_force.tx,
                    self.ati_force.current_force.ty,
                    self.ati_force.current_force.tz,
                ]
            ).reshape(-1) - ref.reshape(-1)

            # 打印原始传感器数据
        #print("【传感器原始数据】", force_cur_tcp)

        # 力/力矩限幅
        # 确保 force_cur_tcp 是 numpy 数组，且长度为 6
        if not isinstance(force_cur_tcp, np.ndarray):
            raise ValueError("force_cur_tcp 必须是 numpy 数组")
        if len(force_cur_tcp) != 6:
            raise ValueError("force_cur_tcp 必须为 6X1 向量")
        #限幅处理
        force_cur_tcp[:3][force_cur_tcp[:3] > 50] = 50
        force_cur_tcp[:3][force_cur_tcp[:3] < -50] = -50
        force_cur_tcp[:3][np.abs(force_cur_tcp[:3]) < 1] = 0

        force_cur_tcp[3:][force_cur_tcp[3:] > 5] = 5
        force_cur_tcp[3:][force_cur_tcp[3:] < -5] = -5
        force_cur_tcp[3:][np.abs(force_cur_tcp[3:]) < 0.1] = 0

        f_base = TransPose.trans_fromsensor_tobase(sensor_T, force_cur_tcp)#坐标变换

        # 打印变换后的数据
        #print("【基坐标系数据】", f_base.flatten())

        return f_base

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

    def read_status(self):
        """
        读取位置和力数据
        """
        data = np.append(self.pose, self.f_base)
        return data.reshape(-1)

    def go_startPose(self, start_point):#机械臂移动到目标位置
        speed = 0.005
        acc = 0.001
        self.moveL(start_point, asy=False, speed=speed, acc=acc)#moveL是直线运动 asy=False 同步执行模式 机械臂会等待运动完成才会返回

    def pose_stable(self, M, B, K):
        """
        定点位姿导纳控制
        """
        self.update_status()  #更新当前状态
        #计算导纳控制增量
        control_e = self.adcontrol.cal_increment(M=M, B=B, K=K, ft=self.f_base, pose=self.pose, pose_target=self.pose)#调用导纳控制算法 计算导纳控制增量
        self.control = copy.deepcopy(self.pose) #计算新的控制位置
        self.control = self.add_pose_increment(self.control, control_e)
        self.servoL(self.control)

    def pose_stable_velocityBased(self, M, B, K):
        maxVelocity = 0.02
        self.update_status()
        control_de = self.adcontrol.cal_velocity_increment(M=M, B=B, K=K, ft=self.f_base, pose=self.pose,
                                                           velocity=self.ee_velocity, pose_target=self.pose)
        self.control_velocity = copy.deepcopy(self.ee_velocity).reshape(6, 1)
        self.control_velocity = self.control_velocity + control_de.reshape(6, 1)
        self.control_velocity = np.clip(self.control_velocity, -maxVelocity, maxVelocity)
        self.speedL(self.control_velocity)

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

    def daona(self, M, B, K, TargetX, TargetY, TargetZ, TargetRr, TargetRp, TargetRy):
        # 定点导纳
        targetPos = np.array(
            [TargetX, TargetY, TargetZ, TargetRr, TargetRp, TargetRy]
        )

        self.go_startPose(targetPos)
        while True:
            self.pose_stable(M, B, K)


# Main function
if __name__ == "__main__":
    M = np.diag([0.2, 0.2, 0.05, 0.008, 0.008, 0.01])
    B = np.diag([20, 20, 20, 20, 20, 20])
    K = np.diag([50, 50, 100, 30, 30, 30])

    # 速度
    # M = np.diag([1, 1, 1, 0.5, 0.5, 0.5])
    # B = np.diag([100, 100, 100, 0, 0, 0])
    # K = np.diag([100, 100, 100, 0, 0, 0])

    # 初始化控制器
    controller = ad_control(UR_HOST)

    # 定点导纳
    controller.go_startPose(controller.init_pos)

    ctime = time.time()
    utime = ctime

    while True:
        try:
            controller.pose_stable(M, B, K)
            # controller.pose_stable_velocityBased(M, B, K)
        except:
            controller.disconnect()
        utime = time.time()
        print(utime, 's\n', controller.f_base.reshape(6), '\n', controller.ee_velocity.reshape(6),
              '\n------------------\n')
        # time.sleep(0.01)

    # controller.disconnect()
