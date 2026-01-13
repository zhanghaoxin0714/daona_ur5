"""
2022.8.25
TransPose用于各类坐标转换，之前的PoseTool写的稍微有点乱，因此修改为这一版
"""
import numpy as np
from scipy.spatial.transform import Rotation as R


class TransPose():
    '''
    用于各类坐标转换
    '''

    def __init__(self) -> None:
        pass

    def getT_fromRotvec(pose):
        '''
        从位姿向量变换为变换矩阵
        '''
        x = pose[0]
        y = pose[1]
        z = pose[2]
        rx = pose[3]
        ry = pose[4]
        rz = pose[5]
        #从旋转向量创建旋转矩阵
        r = R.from_rotvec(np.array([rx, ry, rz]), False)
        R2 = r.as_matrix()
        t = np.mat([[x], [y], [z]]) #创建平移向量

        R_ = np.array(R2)
        t_ = np.array(t)
        T_1 = np.append(R_, t_, axis=1)
        # print(T_1)

        zero = np.mat([0, 0, 0, 1])
        T_2 = np.array(zero)

        T = np.append(T_1, T_2, axis=0)
        T = np.mat(T)

        return T

    def getRotvec_fromT(T):
        x = T[0, 3]
        y = T[1, 3]
        z = T[2, 3]
        r = R.from_matrix(T[0:3, 0:3])
        [rx, ry, rz] = r.as_rotvec()

        # pdb.set_trace()

        return np.array([x, y, z, rx, ry, rz])

    def trans_fromsensor_tobase(T, f_sensor):
        '''
        将六维力从传感器坐标转换到基坐标系下表示
        '''
        f_sensor = np.mat(f_sensor.reshape(-1, 1))
        x = T[0, 3]
        y = T[1, 3]
        z = T[2, 3]

        Rba = np.mat(T[:3, :3])
        # print(Rba)
        # aP_borg = np.mat([
        #     [0, -x, y],
        #     [x, 0, -x],
        #     [-y, x, 0]
        # ])
        aP_borg = np.mat([
            [0, -z, y],
            [z, 0, -x],
            [-y, x, 0]
        ])
        # aP_borg = np.mat([
        #     [x,0,0],
        #     [0,y,0],
        #     [0,0,z]
        # ])
        a1 = np.hstack((Rba, np.zeros((3, 3))))
        a2 = np.hstack((aP_borg @ Rba, Rba))
        # a1 = np.hstack((Rba,-Rba @ aP_borg ))
        # a2 = np.hstack((np.zeros((3,3)),Rba))
        # a1 = np.hstack((Rba,aP_borg @ Rba ))
        # a2 = np.hstack((np.zeros((3,3)),Rba))
        Tf = np.vstack((a1, a2))
        # pdb.set_trace()
        f_base = Tf @ f_sensor
        return f_base

    def get_cur_force(self, force, mode=0, ref=0):
        '''
        获得当前力传感数据
        '''
        force_pose_T = force.get_matrix()
        if mode == 0:  # 正常读取
            force_cur_tcp = np.array(force.read()).reshape(-1)  # 读取力传感数据
        elif mode == 1:  # 全部归零，看能不能归位
            force_cur_tcp = np.array([0 for _ in range(6)]).reshape(-1)
        elif mode == 2:  # 给某一方向一个特殊的力
            force_cur_tcp = np.array([0 for _ in range(6)]).reshape(-1)
            force_cur_tcp[0] -= 10
        elif mode == 3:
            force_cur_tcp = np.array(
                force.read()).reshape(-1) - ref.reshape(-1)
        f_base = self.trans_fromsensor_tobase(force_pose_T, force_cur_tcp)
        return f_base

    @staticmethod
    def trans_force_reference_point(force_old, r_vector):
        """
        在同一坐标系下，将力从一个参考点转换到另一个参考点

        Args:
            force_old: 旧参考点处的力/力矩 [Fx, Fy, Fz, Tx, Ty, Tz] (6×1 或 1×6)
            r_vector: 从旧参考点指向新参考点的向量 [dx, dy, dz] (3×1 或 1×3)

        Returns:
            force_new: 新参考点处的力/力矩 [Fx, Fy, Fz, Tx, Ty, Tz] (6×1)
        """
        force_old = np.array(force_old).reshape(6, 1)
        r_vector = np.array(r_vector).reshape(3, 1)

        # 提取力和力矩
        F_old = force_old[:3]  # [Fx, Fy, Fz]
        T_old = force_old[3:]  # [Tx, Ty, Tz]

        # 力向量不变
        F_new = F_old.copy()

        # 力矩转换：T_new = T_old - r × F_old
        # 计算叉积 r × F
        r_cross_F = np.cross(r_vector.flatten(), F_old.flatten()).reshape(3, 1)

        # 新参考点的力矩
        T_new = T_old - r_cross_F

        # 组合新的力/力矩
        force_new = np.vstack([F_new, T_new])

        return force_new


if __name__ == "__main__":
    target_tip = np.array(
        [78.57 / 1000, 397.57 / 1000, 226.95 / 1000, 3.140, -0.009, 0.001]
    )
    T_target = TransPose.getT_fromRotvec(target_tip)
    print(T_target)
