import numpy as np
from Robots.URControlAPI import URControlAPI


class CartesianVelocityController:
    def __init__(self, robotModel, robotIp=None):
        self.robot = robotModel

        if robotIp is not None:
            self.api = URControlAPI(robotIp)
        else:
            self.api = None

    def get_dq_withCartesianVelocity(self, cartesian_vel, q):
        jacobian0 = self.robot.getBaseJocobianWith_q(q)
        dq = np.linalg.pinv(jacobian0) @ cartesian_vel
        return dq

    def ControlWithCartesianVelocity(self, cartesian_vel, q):
        if self.api is None:
            print('请先连接机器人')
            return
        try:
            self.api.speedJ(self.get_dq_withCartesianVelocity(cartesian_vel, q))
        except:
            print('发送笛卡尔速度指令失败')
