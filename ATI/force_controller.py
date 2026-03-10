# force_controller.py
import numpy as np
import threading
import time
from tool.TransPose import TransPose
from ati_force_sensor.rpi_ati_net_ft import rpi_ati_net_ft  # 添加这个导入
from ati_force_sensor.Force import Force  # 导入Force类


class ForceController:
    """独立的力传感器控制器"""

    def __init__(self, force_host=None):

        self.delta_ur_sensor = np.array([0, 0, 35 / 1000, 0, 0, -0.262])
        self.sensor_deltaT = TransPose.getT_fromRotvec(self.delta_ur_sensor)#定义传感器位置
        # 工装末端到被动端中心点的偏移量（基坐标系下）
        # 格式：[dx, dy, dz] 单位：米
        R_ws = np.array([
            [-0.9944, -0.1041, 0.01721],
            [-0.1049, 0.9932, -0.05024],
            [-0.01186, -0.05177, -0.9986]
        ])  # 工件坐标系相对于传感器坐标系的旋转矩阵
        t_ws = np.array([0.05403, -0.05297, 0.7109])  # 平移向量（米）
        # 构建4×4齐次变换矩阵
        T_workpiece_to_sensor = np.eye(4)
        T_workpiece_to_sensor[:3, :3] = R_ws  # 旋转部分
        T_workpiece_to_sensor[:3, 3] = t_ws  # 平移部分
        # 暂时设为 a，后续需要根据实际测量值更新
        self.T_workpiece_to_sensor = T_workpiece_to_sensor  # 保存为类属性
        self.tool_tip_to_passive_offset = np.array([0, 0, -0.2])  # 示例：假设被动端在工装末端下方20cm

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
            self.netft.set_tare_from_ft()#去皮
            self.netft.start_streaming()

            # 测试连接
            test_data = self.netft.try_read_ft_streaming(0.01)
            if test_data[0]:  # 检查第一个元素（成功标志）
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
        # force_cur_tcp[:3] = np.clip(force_cur_tcp[:3], -50, 50)
        # force_cur_tcp[3:] = np.clip(force_cur_tcp[3:], -5, 5)
        # print(force_cur_tcp);

        # 根据用途决定是否小值清零
        if not for_display:

            # 控制用：小值清零
            if np.abs(force_cur_tcp[0]) < 5:
                force_cur_tcp[0] = 0
            if np.abs(force_cur_tcp[1]) < 5:
                force_cur_tcp[1] = 0
            if np.abs(force_cur_tcp[2]) < 1:
                force_cur_tcp[2] = 0

            if np.abs(force_cur_tcp[3]) < 1:
                force_cur_tcp[3] = 0
            if np.abs(force_cur_tcp[4]) < 1:
                force_cur_tcp[4] = 0
            if np.abs(force_cur_tcp[5]) < 1:
                force_cur_tcp[5] = 0
            # force_cur_tcp[:3][np.abs(force_cur_tcp[:3]) < 3] = 0
            # force_cur_tcp[3:][np.abs(force_cur_tcp[3:]) < 0.3] = 0
                # 如果 xvni 参数为 None，使用类属性
        if xvni is None:
            xvni = self.xvni
        # 根据机械臂连接状态决定是否进行坐标变换
        if robot_controller is not None:
            # # # 方法1：直接转换到基坐标系（旧方法，传感器中心）
            # fg = self._transform_to_base_coordinate(force_cur_tcp, robot_controller)

            # # # 方法2：通过工件坐标系转换（新方法，工件中心）
            force_workpiece = self.transform_sensor_to_workpiece(force_cur_tcp)
            force_base = self.transform_workpiece_to_base(force_workpiece, robot_controller)
            fg = force_base  # 使用工件中心的力


            # 第二步：传感器中心基坐标系 → 被动端基坐标系（新增）
            # fg_passive = self.transform_force_to_passive_end(fg, robot_controller)
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
            #从基坐标系到工具坐标系的变换矩阵
            tcp_T = TransPose.getT_fromRotvec(tcp_pose)#getT 将位姿向量转换为4*4的齐次变换矩阵
            # 从基坐标系到传感器坐标系的变换矩阵
            sensor_T = tcp_T @ self.sensor_deltaT
            # 进行坐标变换
            f_base = TransPose.trans_fromsensor_tobase(sensor_T, force_data)
            return f_base


        except Exception as e:
            print(f"坐标变换失败: {e}")
            return force_data  # 返回原始数据

    def transform_sensor_to_workpiece(self, force_sensor):
        """
        将力从传感器坐标系转换到工件坐标系

        Args:
            force_sensor: 传感器坐标系中的力/力矩 [Fx, Fy, Fz, Tx, Ty, Tz] (6×1 或 1×6)

        Returns:
            force_workpiece: 工件坐标系中的力/力矩 [Fx, Fy, Fz, Tx, Ty, Tz] (6×1)
        """
        try:
            # 使用 trans_fromsensor_tobase 方法转换
            # 注意：虽然方法名是"tobase"，但实际是通用的力坐标转换方法
            # 可以用于任意两个坐标系之间的转换
            force_workpiece = TransPose.trans_fromsensor_tobase(
                self.T_workpiece_to_sensor,  # 工件相对于传感器的变换矩阵
                force_sensor  # 传感器坐标系中的力
            )
            return force_workpiece
        except Exception as e:
            print(f"传感器到工件坐标系转换失败: {e}")
            return force_sensor  # 返回原始数据

    # 在 transform_sensor_to_workpiece 方法之后添加：

    def transform_workpiece_to_base(self, force_workpiece, robot_controller):
        """
        将力从工件坐标系转换到基坐标系

        Args:
            force_workpiece: 工件坐标系中的力/力矩 [Fx, Fy, Fz, Tx, Ty, Tz] (6×1 或 1×6)
            robot_controller: 机械臂控制器，用于获取传感器位姿

        Returns:
            force_base: 基坐标系中的力/力矩 [Fx, Fy, Fz, Tx, Ty, Tz] (6×1)
        """
        try:
            # 获取机械臂TCP位姿
            tcp_pose = robot_controller.get_ee_pose()
            tcp_T = TransPose.getT_fromRotvec(tcp_pose)

            # 传感器在基坐标系中的位姿
            sensor_T_base = tcp_T @ self.sensor_deltaT

            # 计算工件在基坐标系中的位姿
            workpiece_T_base = sensor_T_base @ self.T_workpiece_to_sensor

            # 转换到基坐标系
            force_base = TransPose.trans_fromsensor_tobase(
                workpiece_T_base,
                force_workpiece
            )

            return force_base

        except Exception as e:
            print(f"工件到基坐标系转换失败: {e}")
            return force_workpiece  # 返回原始数据

    def transform_force_to_passive_end(self, force_sensor_center, robot_controller):
        """
        将传感器中心处的力（基坐标系下）转换到被动端中心位置（基坐标系下）

        Args:
            force_sensor_center: 传感器中心处的力/力矩 [Fx, Fy, Fz, Tx, Ty, Tz] (基坐标系下)
            robot_controller: 机械臂控制器，用于获取传感器位姿

        Returns:
            force_passive_end: 被动端中心处的力/力矩 [Fx, Fy, Fz, Tx, Ty, Tz] (基坐标系下)
        """
        try:
            # 获取传感器在基坐标系中的位姿
            tcp_pose = robot_controller.get_ee_pose()
            tcp_T = TransPose.getT_fromRotvec(tcp_pose)
            sensor_T = tcp_T @ self.sensor_deltaT
            sensor_R = sensor_T[:3, :3]  # 旋转矩阵

            # 计算从传感器中心到被动端的向量（基坐标系下）
            # 1. 工装首端（传感器中心）到工装末端的偏移量（工装坐标系下）
            tool_start_to_end_local = np.array([0.05403, -0.05297, 0.7109]).reshape(3, 1)
            # 转换到基坐标系
            tool_start_to_end_base = sensor_R @ tool_start_to_end_local

            # 2. 工装末端到被动端中心点的偏移量（工装坐标系下）
            tool_end_to_passive_local = self.tool_tip_to_passive_offset.reshape(3, 1)
            # 转换到基坐标系
            tool_end_to_passive_base = sensor_R @ tool_end_to_passive_local

            # 3. 从传感器中心到被动端的总向量（基坐标系下）
            sensor_to_passive_vector = (tool_start_to_end_base + tool_end_to_passive_base).flatten()

            # 将力从传感器中心转换到被动端位置（同一坐标系下，只转换参考点）
            force_passive_end = TransPose.trans_force_reference_point(
                force_sensor_center,
                sensor_to_passive_vector
            )

            return force_passive_end.flatten()

        except Exception as e:
            print(f"力转换到被动端失败: {e}")
            return force_sensor_center  # 返回原始力

    def transform_base_sensor_to_base_workpiece(self, force_base_sensor, robot_controller):
        """
        将基坐标系下传感器中心的力转换到基坐标系下工件中心的力

        Args:
            force_base_sensor: 基坐标系下传感器中心的力/力矩 [Fx, Fy, Fz, Tx, Ty, Tz]
            robot_controller: 机械臂控制器，用于获取传感器位姿

        Returns:
            force_base_workpiece: 基坐标系下工件中心的力/力矩 [Fx, Fy, Fz, Tx, Ty, Tz]
        """
        try:
            # 获取传感器在基坐标系中的位姿
            tcp_pose = robot_controller.get_ee_pose()
            tcp_T = TransPose.getT_fromRotvec(tcp_pose)#传感器位姿矩阵
            sensor_T_base = tcp_T @ self.sensor_deltaT

            # 传感器中心在基坐标系中的位置
            sensor_pos_base = sensor_T_base[:3, 3]

            # 工件在传感器坐标系中的位置
            workpiece_pos_sensor = self.T_workpiece_to_sensor[:3, 3]

            # 传感器坐标系到基坐标系的旋转矩阵
            R_sensor_to_base = sensor_T_base[:3, :3]

            # 将工件位置转换到基坐标系
            workpiece_pos_base = sensor_pos_base + R_sensor_to_base @ workpiece_pos_sensor

            # 计算从传感器中心到工件中心的向量（在基坐标系中）
            r_sensor_to_workpiece = workpiece_pos_base - sensor_pos_base

            # 转换参考点（同一坐标系下）
            force_base_workpiece = TransPose.trans_force_reference_point(
                force_base_sensor,
                r_sensor_to_workpiece
            )

            return force_base_workpiece.flatten()

        except Exception as e:
            print(f"传感器中心到工件中心转换失败: {e}")
            return force_base_sensor  # 返回原始数据

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

