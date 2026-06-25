import rtde_control
import rtde_receive
import numpy as np


class URControlAPI(object):
    def __init__(self, HOST):
        self.hostname = HOST #保存主机地址
        self.rtde_c = rtde_control.RTDEControlInterface(HOST) #创建控制接口 用于发送命令给机械臂 机械臂的遥控器
        self.rtde_r = rtde_receive.RTDEReceiveInterface(HOST) #创建接收接口 用于接收机械臂状态数据 机械臂的传感器
        self.current_tcp = self.get_ee_pose() #获取当前TCP位姿

    def move_joint_path(self, path):
        joints = [0, 0, 0, 0, 0, 0]
        for i in range(len(path)):
            for n in range(6):
                joints[n] = path[i][n] * np.pi / 180

            self.rtde_c.moveJ(joints, 2, 1.5, True)  # 代表函数不阻塞，可以进行改变，从而实现控制

    def  stop_robot(self):
        self.rtde_c.speedStop()#停止机器人执行的speedl or speedj等控制命令
        self.rtde_c.servoStop()#停止伺服控制
        self.rtde_c.stopScript()#停止URScript脚本

    def disconnect(self):
        self.stop_robot()
        self.rtde_c.disconnect()

    # 工具的笛卡尔坐标(x,y,z,rx,ry,rz)
    def get_ee_pose(self):
        return self.rtde_r.getActualTCPPose()

    def get_ee_velocity(self):
        return self.rtde_r.getActualTCPSpeed()

    def get_q(self):
        return self.rtde_r.getActualQ()

    def moveL(self, pose, asy=True, speed=0.1, acc=0.2):
        # pose全部采用的是rotvec的形式，因此传输的时候也全部转化为rotvec的形式
        pose = pose.tolist()
        self.rtde_c.moveL(pose, speed, acc, asy)

    def moveJ(self, pose, asy=True, speed=0.1, acc=0.2):
        # pose全部采用的是rotvec的形式，因此传输的时候也全部转化为rotvec的形式
        pose = pose.tolist()
        self.rtde_c.moveJ(pose, speed, acc, asy)

    def servoL(self, pose, speed=0.05, acc=0.02):
        pose = pose.tolist()
        self.rtde_c.servoL(pose, speed, acc, 0.12, 0.09, 120)

        # time：伺服命令保持的时间（阻塞时间），单位秒。机器人会在这段时间里尽量向目标位姿靠拢，值越大动作越慢、越平滑。
        # lookahead_time：0.03–0.2 s，用于平滑轨迹；值越大越平滑、越“柔”。
        # gain：100–2000
        # 的比例增益，越大响应越快、越刚性。

    def speedJ(self, joint_speed, acc=0.25, control_period=0.02):
        joint_speed = joint_speed.tolist()
        self.rtde_c.speedJ(joint_speed, acc, control_period)
        
    def stopSpeedJ(self):
        self.rtde_c.speedStop()

    def speedL(self, ee_speed, acc=0.25, control_period=0.02):
        self.rtde_c.speedL(ee_speed, acc, control_period)
