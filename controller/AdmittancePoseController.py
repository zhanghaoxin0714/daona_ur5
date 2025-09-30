import numpy as np


class AdmittancePoseController:
    def __init__(self, pose_init, control_period):
        #初始化误差变量
        self.e_cur = np.zeros([6, 1]) #当前位姿误差
        self.de_cur = np.zeros([6, 1]) #当前速度误差
        self.dde_pre = np.zeros([6, 1]) #前一时刻的位姿加速度误差
        self.dde_cur = np.zeros([6, 1]) #当前位姿加速度误差
        self.e_pre = np.zeros([6, 1]) #前一时刻位姿误差
        #初始化位姿变量
        self.pose_pre_desire = pose_init.reshape(6, 1)#前一时刻期望位姿误差
        self.pose_pre = pose_init.reshape(6, 1)#前一时刻实际位姿
        self.de_pre = np.zeros([6, 1])#前一时刻位姿速度误差
        self.d_pose_pre = np.zeros([6, 1])#前一时刻位姿速度
        #初始化当前状态
        self.dd_pose_cur = np.zeros([6, 1])#当前位姿加速度
        self.d_pose_cur = np.zeros([6, 1])#当前位姿速度
        self.pose_cur_desire = pose_init.reshape(6, 1)#当前期望位姿
        self.pose_cur = pose_init.reshape(6, 1)#当前实际位姿

        self.dt = control_period#控制周期时间间隔

    # def cal_velocity_increment(self, M, B, K, ft, pose, pose_target):
    #     self.M = M
    #     self.B = B
    #     self.K = K
    #     # print('M:', self.M)
    #     # print('B:', self.B)
    #     # print('K:', self.K)
    # 
    #     pose = np.array(pose).reshape(6, 1)
    #     pose_target = np.array(pose_target).reshape(6, 1)
    #     ft = np.array(ft).reshape(6, 1)
    # 
    #     # 前一时刻的位姿误差、位姿速度误差
    #     self.pose_cur = pose
    #     self.pose_cur_desire = pose_target
    # 
    #     self.d_pose_cur = (
    #             (self.pose_cur - self.pose_cur_desire) / self.dt).reshape(6, 1)
    #     self.dd_pose_cur = (
    #             (self.d_pose_cur - self.d_pose_pre) / self.dt).reshape(6, 1)
    #     self.de_pre = ((self.pose_cur - self.pose_cur_desire)
    #                    - (self.pose_pre - self.pose_pre_desire)
    #                    - self.dd_pose_cur * self.dt * self.dt
    #                    ).reshape(6, 1) / self.dt
    #     self.e_pre = (self.pose_cur - self.pose_cur_desire - self.d_pose_cur * self.dt).reshape(6, 1)
    # 
    #     # 二次积分得到当前时刻的位姿误差
    #     self.dde_cur = np.diag(1 / np.diag(self.M)) @ (
    #             ft - self.B @ self.de_pre - self.K @ self.e_pre
    #     )
    #     self.de_cur = (
    #             self.de_pre + self.dt * (self.dde_cur + self.dde_pre) / 2
    #     )
    # 
    # 
    #     # 更新内部变量
    #     self.pose_pre = self.pose_cur
    #     self.dde_pre = self.dde_cur
    #     self.pose_pre_desire = self.pose_cur_desire
    #     self.d_pose_pre = self.d_pose_cur
    # 
    #     return self.de_cur

    def cal_velocity_increment(self, M, B, K, ft, pose, velocity, pose_target):
        self.M = M
        self.B = B
        self.K = K

        pose = np.array(pose).reshape(6, 1)
        velocity = np.array(velocity).reshape(6, 1)
        pose_target = np.array(pose_target).reshape(6, 1)
        ft = np.array(ft).reshape(6, 1)

        # 前一时刻的位姿误差、位姿速度误差
        self.pose_cur = pose
        self.pose_cur_desire = pose_target

        self.d_pose_cur = velocity.reshape(6, 1)
        self.dd_pose_cur = (
                (self.d_pose_cur - self.d_pose_pre) / self.dt).reshape(6, 1)
        self.de_pre = ((self.pose_cur - self.pose_cur_desire)
                       - (self.pose_pre - self.pose_pre_desire)
                       - self.dd_pose_cur * self.dt * self.dt
                       ).reshape(6, 1) / self.dt
        self.e_pre = (self.pose_cur - self.pose_cur_desire - self.d_pose_cur * self.dt).reshape(6, 1)

        # 二次积分得到当前时刻的位姿误差
        self.dde_cur = np.diag(1 / np.diag(self.M)) @ (
                ft - self.B @ self.de_pre - self.K @ self.e_pre
        )
        self.de_cur = (
                self.de_pre + self.dt * (self.dde_cur + self.dde_pre) / 2
        )

        # 更新内部变量
        self.pose_pre = self.pose_cur
        self.dde_pre = self.dde_cur
        self.pose_pre_desire = self.pose_cur_desire
        self.d_pose_pre = self.d_pose_cur

        return self.de_cur

    def cal_increment(self, M, B, K, ft, pose, pose_target):
        """
        计算位置、姿态导纳控制增量

        :param pose: current pose
        :param ft: current force/torque
        :param pose_target: desired pose
        :param M: virtual mass
        :param B: virtual damping
        :param K: virtual stiffness
        """
        self.M = M
        self.B = B
        self.K = K
        # print('M:', self.M)
        # print('B:', self.B)
        # print('K:', self.K)
        #数据预处理
        pose = np.array(pose).reshape(6, 1)#当前位姿
        pose_target = np.array(pose_target).reshape(6, 1)#目标位姿
        ft = np.array(ft).reshape(6, 1)#当前力

        # 更新当前状态
        self.pose_cur = pose
        self.pose_cur_desire = pose_target
        #计算速度和加速度
        self.d_pose_cur = (
                (self.pose_cur - self.pose_cur_desire) / self.dt).reshape(6, 1)
        self.dd_pose_cur = (
                (self.d_pose_cur - self.d_pose_pre) / self.dt).reshape(6, 1)
        #计算误差
        self.de_pre = ((self.pose_cur - self.pose_cur_desire)
                       - (self.pose_pre - self.pose_pre_desire)
                       - self.dd_pose_cur * self.dt * self.dt
                       ).reshape(6, 1) / self.dt
        self.e_pre = (self.pose_cur - self.pose_cur_desire - self.d_pose_cur * self.dt).reshape(6, 1)

        # 二次积分得到当前时刻的位姿误差
        self.dde_cur = np.diag(1 / np.diag(self.M)) @ (
                ft - self.B @ self.de_pre - self.K @ self.e_pre
        )#求解加速度
        self.de_cur = (
                self.de_pre + self.dt * (self.dde_cur + self.dde_pre) / 2
        )#积分得到速度
        self.e_cur = (
                np.array((self.dt * (self.de_cur + self.de_pre) / 2)) + self.e_pre
        )#积分得到位置

        # 更新内部变量
        self.pose_pre = self.pose_cur
        self.dde_pre = self.dde_cur
        self.pose_pre_desire = self.pose_cur_desire
        self.d_pose_pre = self.d_pose_cur

        return self.e_cur
