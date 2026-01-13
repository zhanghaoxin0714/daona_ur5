import roboticstoolbox as rtb


class UR5_model:
    def __init__(self):
        self.robot = rtb.models.URDF.UR5()

    def updateStatus(self, q):
        """
        更新工具箱中机器人模型的属性，未正常更新会导致雅可比求解错误
        """
        self.robot.q = q

    def get_q(self):
        return self.robot.q

    def getBaseJocobian(self):
        """
        获取基座系下的雅可比矩阵
        """
        return self.robot.jacob0(self.robot.q)

    def getBaseJocobianWith_q(self, current_q):
        """
        获取基座系下的雅可比矩阵
        """
        return self.robot.jacob0(current_q)
