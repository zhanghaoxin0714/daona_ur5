# force_controller.py
import numpy as np
import threading
import time
from TransPose import TransPose
from ati_force_sensor.ForceUpdateThread import ForceUpdateThread
from ati_force_sensor.rpi_ati_net_ft import rpi_ati_net_ft  # 添加这个导入
from ati_force_sensor.Force import Force  # 导入Force类


class ForceController:
    """独立的力传感器控制器"""

    def __init__(self, force_host=None):
        # # 初始化力传感器相关变量
        # self.ati_force = None  # 存储力传感器线程对象
        # self.force_thread = None  # 存储力传感器获取线程
        # # 传感器偏移量
        # # self.delta_ur_sensor = np.array([0, 0, 35 / 1000, 0, 0, -0.262])
        # # self.sensor_deltaT = TransPose.getT_fromRotvec(self.delta_ur_sensor)
        # # 初始力
        # self.force_init = np.zeros(6)
        self.delta_ur_sensor = np.array([0, 0, 35 / 1000, 0, 0, -0.262])
        self.sensor_deltaT = TransPose.getT_fromRotvec(self.delta_ur_sensor)#定义传感器位置

        self.netft = None#六维力连接实例
        self.is_connected = False#连接标志位
        self.force_update_thread = None
        self._stop_thread = False  # 添加线程停止标志
        self.current_force = Force([0, 0, 0, 0, 0, 0])  # 初始化数据
        self.force_init = np.zeros(6)
        self.xvni = False  # 添加虚拟力标志
        self.virtual_force = np.zeros(6)  # [Fx, Fy, Fz, Tx, Ty, Tz]

        # 如果提供了IP地址，直接连接
        if force_host is not None:
            self.connect_force_sensor(force_host)
            time.sleep(1)  # 等待连接稳定
            self.start_force_update_thread()  # 启动数据更新线程

    def connect_force_sensor(self, force_host):
        """单独连接力传感器"""

        """内部连接方法"""
        try:
            self.netft = rpi_ati_net_ft.NET_FT(force_host)
            self.netft.set_tare_from_ft()
            self.netft.start_streaming()

            # 测试连接
            test_data = self.netft.try_read_ft_streaming(0.01)
            if test_data is not None:
                self.is_connected = True
                return True
            else:
                self.is_connected = False
                return False
        except Exception as e:
            print(f"连接力传感器失败: {e}")
            self.is_connected = False
            return False

    def is_force_sensor_connected(self):
        """获取力传感器数据

    Args:
        mode: 数据模式
        robot_controller: 机械臂控制器，用于坐标变换
    """
        return self.netft is not None and self.is_connected

    def start_force_update_thread(self):
        """启动力传感器数据更新线程"""
        if self.is_connected and self.force_update_thread is None:
            self._stop_thread = False
            self.force_update_thread = threading.Thread(
                target=self._force_update_loop,
                daemon=True
            )
            self.force_update_thread.start()
            print("力传感器数据更新线程已启动")

    def set_xvni(self, value):
        """设置虚拟力标志"""
        self.xvni = value

    def get_cur_force(self, mode=0, robot_controller=None, for_display=False,xvni = None):
        """获取当前力传感器数据
    Args:
        mode: 数据模式
        robot_controller: 机械臂控制器，用于坐标变换
        for_display: True=显示用原始数据, False=控制用滤波数据
    """
        if not self.is_force_sensor_connected():
            return np.zeros(6)

        if self.current_force is not None:
            force_cur_tcp = np.array([
                self.current_force.fx,  # 直接读取属性
                self.current_force.fy,
                self.current_force.fz,
                self.current_force.tx,
                self.current_force.ty,
                self.current_force.tz,
            ]).reshape(-1)
        else:
            return np.zeros(6)

        # 在这里进行数据处理（限幅、坐标变换等）
        # 力/力矩限幅
            # 确保 force_cur_tcp 是 numpy 数组，且长度为 6
        if not isinstance(force_cur_tcp, np.ndarray):
            raise ValueError("force_cur_tcp 必须是 numpy 数组")
        if len(force_cur_tcp) != 6:
            raise ValueError("force_cur_tcp 必须为 6X1 向量")
        force_cur_tcp[:3] = np.clip(force_cur_tcp[:3], -50, 50)
        force_cur_tcp[3:] = np.clip(force_cur_tcp[3:], -5, 5)
        # print(force_cur_tcp);

        # 根据用途决定是否小值清零
        if not for_display:

            # 控制用：小值清零
            if np.abs(force_cur_tcp[0]) < 1:
                force_cur_tcp[0] = 0
            if np.abs(force_cur_tcp[1]) < 1:
                force_cur_tcp[1] = 0
            if np.abs(force_cur_tcp[2]) < 1:
                force_cur_tcp[2] = 0

            if np.abs(force_cur_tcp[3]) < 0.3:
                force_cur_tcp[3] = 0
            if np.abs(force_cur_tcp[4]) < 0.3:
                force_cur_tcp[4] = 0
            if np.abs(force_cur_tcp[5]) < 0.3:
                force_cur_tcp[5] = 0



            # force_cur_tcp[:3][np.abs(force_cur_tcp[:3]) < 3] = 0
            # force_cur_tcp[3:][np.abs(force_cur_tcp[3:]) < 0.3] = 0
                # 如果 xvni 参数为 None，使用类属性
        if xvni is None:
            xvni = self.xvni
        # 根据机械臂连接状态决定是否进行坐标变换
        if robot_controller is not None:
            fg = self._transform_to_base_coordinate(force_cur_tcp, robot_controller)
            # print(fg)
            # print(self.virtual_force)
            if xvni:
                vf = self.get_virtual_force().reshape(6, 1)  # 确保是列向量
                fg = fg + vf
                # fg[2] = fg[2]-3
                return fg
            else:
                return fg

        else:
            return force_cur_tcp

    def _transform_to_base_coordinate(self, force_data, robot_controller):
        """将力传感器数据转换到机械臂基坐标系"""
        try:
            # 获取机械臂位姿
            tcp_pose = robot_controller.get_ee_pose()
            tcp_T = TransPose.getT_fromRotvec(tcp_pose)
            # 计算传感器变换矩阵
            sensor_T = tcp_T @ self.sensor_deltaT
            # 进行坐标变换
            f_base = TransPose.trans_fromsensor_tobase(sensor_T, force_data)
            return f_base


        except Exception as e:
            print(f"坐标变换失败: {e}")
            return force_data  # 返回原始数据

    def set_virtual_force(self, values):
        # values 是长度 6 的可迭代对象，里面必须是数字（你自己保证）
        self.virtual_force = np.array(values, dtype=float).reshape(6)


    def get_virtual_force(self):
        return self.virtual_force


    def _force_update_loop(self):
        """力传感器数据更新循环 - 完全参考旧版本的 thread_job"""
        while not self._stop_thread:
            try:
                if self.netft is not None:
                    # 完全参考旧版本：current_force = Force(netft.try_read_ft_streaming(0.01)[1])
                    current_force = Force(self.netft.try_read_ft_streaming(0.01)[1])
                    # print(current_force);
                    #print(f"力传感器数据: {current_force.fx}, {current_force.fy}, {current_force.fz}")  # 添加调试信息
                    self.current_force = current_force  # 实时更新

                time.sleep(0.005)  # 200Hz更新频率
            except Exception as e:
                print(f"力传感器数据更新失败: {e}")
                time.sleep(0.01)




