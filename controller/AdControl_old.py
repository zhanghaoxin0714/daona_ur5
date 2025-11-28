
"""
2022.8.30
slc
阻抗控制基础版本
"""
import numpy as np
import spatialmath as sm


class AdControl:
    def __init__(self, pose_init, pose_target, dt) -> None:
        """
        初始化各个阻抗参数
        所有输入均为三维
        """
        # diag生成对角阵
        self.M = np.diag([0.05 for i in range(3)])  # 惯性
        self.B = np.diag([25 for i in range(3)])  # 阻尼
        self.K = np.diag([1000 for i in range(3)])  # 刚度

        # 导纳中间变量
        # pre:上次更新，cur:当前
        self.pose_pre_e = np.zeros([3, 1])
        self.attitude_pre_e = np.zeros([3, 1])
        self.pose_pre_de = np.zeros([3, 1])
        self.attitude_pre_de = np.zeros([3, 1])
        self.pose_pre_dde = np.zeros([3, 1])
        self.attitude_pre_dde = np.zeros([3, 1])
        self.pose_cur_e = np.zeros([3, 1])
        self.attitude_cur_e = np.zeros([3, 1])
        self.pose_cur_de = np.zeros([3, 1])
        self.attitude_cur_de = np.zeros([3, 1])
        self.pose_cur_dde = np.zeros([3, 1])
        self.attitude_cur_dde = np.zeros([3, 1])

        self.pose_cur = pose_init[:3].reshape(-1)
        self.attitude_cur = pose_init[3:].reshape(-1)
        self.pose_pre = pose_init[:3].reshape(-1)
        self.attitude_pre = pose_init[3:].reshape(-1)
        self.pose_cur_target = pose_target[:3].reshape(-1)
        self.attitude_cur_target = pose_target[3:].reshape(-1)
        self.pose_pre_target = pose_target[:3].reshape(-1)
        self.attitude_pre_target = pose_target[3:].reshape(-1)
        self.pose_cur_d = np.zeros([3])
        self.attitude_cur_d = np.zeros([3])
        self.pose_pre_d = np.zeros([3])
        self.attitude_pre_d = np.zeros([3])

        self.attitude_total_e = np.zeros([3])
        self.attitude_e = np.zeros([3])
        self.attitude_target_e = np.zeros([3])
        self.attitude_cur_dd = np.zeros([3])
        self.pose_cur_dd = np.zeros([3])

        self.dt = dt

    def cal_e(self, pose, force, pose_target, M=None, B=None, K=None):
        """
        input:pose(3),force(3),pose_target(3)
        MBK必须同时输入才会更新值，否则直接使用上一轮的值
        output:pose_e(3),pose_next(3)
        输入输出都为np数组
        目前第一版只实现位置控制
        """
        if M != None and B != None and K != None:
            self.M = np.diag([M for i in range(3)])
            self.B = np.diag([B for i in range(3)])
            self.K = np.diag([K for i in range(3)])

        force = np.array(force).reshape(3, 1)
        pose = np.array(pose).reshape(-1)

        # 求偏差
        self.pose_cur = pose
        self.pose_cur_target = pose_target
        self.pose_pre_de = (
            (self.pose_cur + self.pose_pre_target)
            - (self.pose_pre + self.pose_cur_target)
        ).reshape(3, 1) / self.dt
        self.pose_pre_e = (self.pose_cur - self.pose_cur_target).reshape(3, 1)

        # 求阻抗二次积分
        self.pose_cur_dde = np.diag(1 / np.diag(self.M)) @ (
            force - self.B @ self.pose_pre_de - self.K @ self.pose_pre_e
        )
        self.pose_cur_de = (
            self.pose_pre_de + self.dt *
            (self.pose_cur_dde + self.pose_pre_dde) / 2
        )
        self.pose_cur_e = (
            np.array((self.dt * (self.pose_cur_de + self.pose_pre_de) / 2))
            + self.pose_pre_e
        )

        self.pose_pre = self.pose_cur
        self.pose_pre_dde = self.pose_cur_dde
        self.pose_pre_target = self.pose_cur_target

        pose_next = self.pose_cur_target + self.pose_cur_e

        return self.pose_cur_e.reshape(-1), pose_next.reshape(-1)

    def cal_e_path(self, pose, force, pose_target, M=None, B=None, K=None):
        """
        input:pose(3),force(3),pose_target(3)
        MBK必须同时输入才会更新值，否则直接使用上一轮的值
        output:pose_e(3),pose_next(3)
        输入输出都为np数组
        目前第一版只实现位置控制
        """
        if M != None and B != None and K != None:
            self.M = np.diag([M for i in range(3)])
            self.B = np.diag([B for i in range(3)])
            self.K = np.diag([K for i in range(3)])

        force = np.array(force).reshape(3, 1)
        pose = np.array(pose).reshape(-1)
        pose_target = np.array(pose_target).reshape(-1)

        # 求偏差
        self.pose_cur = pose
        self.pose_cur_target = pose_target
        # pdb.set_trace()
        self.pose_cur_d = (
            (self.pose_cur_target - self.pose_pre_target) / self.dt
        ).reshape(-1)
        self.pose_cur_dd = (
            (self.pose_cur_d - self.pose_pre_d) / self.dt).reshape(-1)
        # pdb.set_trace()
        self.pose_pre_de = (
            (self.pose_cur + self.pose_pre_target)
            - (self.pose_pre + self.pose_cur_target)
            + self.pose_cur_dd * self.dt * self.dt
        ).reshape(3, 1) / self.dt
        self.pose_pre_e = (
            self.pose_cur - self.pose_cur_target + self.pose_cur_d * self.dt
        ).reshape(3, 1)

        # 求阻抗二次积分
        self.pose_cur_dde = np.diag(1 / np.diag(self.M)) @ (
            force - self.B @ self.pose_pre_de - self.K @ self.pose_pre_e
        )
        self.pose_cur_de = (
            self.pose_pre_de + self.dt *
            (self.pose_cur_dde + self.pose_pre_dde) / 2
        )
        self.pose_cur_e = (
            np.array((self.dt * (self.pose_cur_de + self.pose_pre_de) / 2))
            + self.pose_pre_e
        )

        self.pose_pre = self.pose_cur
        self.pose_pre_dde = self.pose_cur_dde
        self.pose_pre_target = self.pose_cur_target
        self.pose_pre_d = self.pose_cur_d

        pose_next = self.pose_cur_target + self.pose_cur_e.reshape(-1)
        # self.pose_pre_target = pose_next.reshape(-1)

        return self.pose_cur_e.reshape(-1), pose_next.reshape(-1)

    def cal_e_pose(self, pose, ft, pose_des, Mp=None, Bp=None, Kp=None, Mo=None, Bo=None, Ko=None):
        """
        input:pose(6),force(6),pose_target(6)
        MBK必须同时输入才会更新值，否则直接使用上一轮的值
        output:pose_e(3),pose_next(3)
        输入输出都为np数组
        位置姿态控制
        """
        print('ft:', ft)
        print('pose:', pose)

        if Mp != None and Bp != None and Kp != None and Mo != None and Bo != None and Ko != None:
            self.Mp = np.diag([Mp for i in range(3)])
            self.Bp = np.diag([Bp for i in range(3)])
            self.Kp = np.diag([Kp for i in range(3)])
            self.Mo = np.diag([Mo for i in range(3)])
            self.Bo = np.diag([Bo for i in range(3)])
            self.Ko = np.diag([Ko for i in range(3)])
            # self.M = np.block([[Mp, np.zeros([3, 3])],
            #                    [np.zeros([3, 3]), Mo]])
            # self.B = np.block([[Bp, np.zeros([3, 3])],
            #                    [np.zeros([3, 3]), Bo]])
            # self.K = np.block([[Kp, np.zeros([3, 3])],
            #                    [np.zeros([3, 3]), Ko]])

        force = np.array(ft[:3]).reshape(3, 1)
        torque = np.array(ft[3:]).reshape(3, 1)
        pose = np.array(pose).reshape(-1)
        pose_target = np.array(pose_des[:3]).reshape(-1)
        attitude_target = np.array(pose_des[3:]).reshape(-1)

        # 求偏差
        self.pose_cur = pose[:3]
        self.attitude_cur = pose[3:]
        self.pose_cur_target = pose_target
        self.attitude_cur_target = attitude_target

        self.pose_cur_d = (
            (self.pose_cur_target - self.pose_pre_target) / self.dt
        ).reshape(-1)
        self.attitude_cur_d = (
            (self.attitude_error(self.attitude_cur_target,
             self.attitude_pre_target)) / self.dt
        ).reshape(-1)

        self.pose_cur_dd = (
            (self.pose_cur_d - self.pose_pre_d) / self.dt).reshape(-1)
        self.attitude_cur_dd = ((self.attitude_error(
            self.attitude_cur_d, self.attitude_pre_d)) / self.dt).reshape(-1)

        self.pose_pre_de = (
            (self.pose_cur + self.pose_pre_target)
            - (self.pose_pre + self.pose_cur_target)
            + self.pose_cur_dd * self.dt * self.dt
        ).reshape(3, 1) / self.dt

        self.attitude_target_e = self.attitude_error(
            self.attitude_pre_target, self.attitude_cur_target)
        self.attitude_e = self.attitude_error(
            self.attitude_cur, self.attitude_pre)
        self.attitude_total_e = self.attitude_sum(
            self.attitude_target_e, self.attitude_e)
        self.attitude_pre_de = self.attitude_sum(
            self.attitude_total_e, self.attitude_cur_dd * self.dt * self.dt
        ).reshape(3, 1) / self.dt

        self.pose_pre_e = (
            self.pose_cur - self.pose_cur_target + self.pose_cur_d * self.dt
        ).reshape(3, 1)
        self.attitude_pre_e = (
            self.attitude_sum(self.attitude_error(self.attitude_cur, self.attitude_cur_target),
                              self.attitude_cur_d * self.dt)
        ).reshape(3, 1)

        # 求阻抗二次积分
        self.pose_cur_dde = np.diag(1 / np.diag(self.Mp)) @ (
            force - self.Bp @ self.pose_pre_de - self.Kp @ self.pose_pre_e
        )
        self.attitude_cur_dde = np.diag(1 / np.diag(self.Mo)) @ (
            torque - self.Bo @ self.attitude_pre_de - self.Ko @ self.attitude_pre_e
        )

        self.pose_cur_de = (
            self.pose_pre_de + self.dt *
            (self.pose_cur_dde + self.pose_pre_dde) / 2
        )
        self.attitude_cur_de = (
            self.attitude_sum(self.attitude_pre_de,
                              self.dt * self.attitude_sum(self.attitude_cur_dde, self.attitude_pre_dde) / 2)
        )

        self.pose_cur_e = (
            np.array((self.dt * (self.pose_cur_de + self.pose_pre_de) / 2))
            + self.pose_pre_e
        )
        self.attitude_cur_e = (
            self.attitude_sum(self.attitude_pre_e,
                              self.dt * self.attitude_sum(self.attitude_cur_de, self.attitude_pre_de) / 2)
        )

        self.pose_pre = self.pose_cur
        self.pose_pre_dde = self.pose_cur_dde
        self.pose_pre_target = self.pose_cur_target
        self.pose_pre_d = self.pose_cur_d

        self.attitude_pre = self.attitude_cur
        self.attitude_pre_dde = self.attitude_cur_dde
        self.attitude_pre_target = self.attitude_cur_target
        self.attitude_pre_d = self.attitude_cur_d

        pose_next = self.pose_cur_target + self.pose_cur_e.reshape(-1)
        attitude_next = self.attitude_sum(
            self.attitude_cur_target, self.attitude_cur_e.reshape(-1))
        # self.pose_pre_target = pose_next.reshape(-1)

        self.command_cur_e = np.append(
            self.pose_cur_e.reshape(-1), self.attitude_cur_e.reshape(-1))
        self.cart_next = np.append(
            pose_next.reshape(-1), attitude_next.reshape(-1))

        return self.command_cur_e, self.cart_next

    def attitude_error(self, attitude, attitude_d):
        """
        计算姿态误差
        attitude - attitude_d
        """
        attitude = np.array(attitude).reshape(-1)
        attitude_d = np.array(attitude_d).reshape(-1)
        R = sm.SE3.RPY(attitude)
        Rd = sm.SE3.RPY(attitude_d)
        Re = R * Rd.inv()
        return Re.rpy()

    def attitude_sum(self, attitude, attitude_d):
        """
        姿态增量
        :param attitude:
        :param attitude_d:
        :return:
        """
        attitude = np.array(attitude).reshape(-1)
        attitude_d = np.array(attitude_d).reshape(-1)
        R = sm.SE3.RPY(attitude)
        Rd = sm.SE3.RPY(attitude_d)
        Re = R * Rd
        return Re.rpy()

    def get_ee(self):
        return self.pose_cur_e.reshape(-1)
