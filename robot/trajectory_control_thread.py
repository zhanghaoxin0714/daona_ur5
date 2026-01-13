# -*- coding: utf-8 -*-
# trajectory_control_thread.py
import time
import numpy as np
from PyQt5 import QtCore
from PyQt5.QtWidgets import QApplication


class TrajectoryControlThread(QtCore.QThread):
    """轨迹跟踪控制线程 - 按轨迹点逐点移动"""
    # 信号定义（用于向主线程发送消息）
    # ========== 信号定义（必须在类级别定义）==========
    signal_progress = QtCore.pyqtSignal(int, int)  # 发送进度 (当前点, 总点数)
    signal_finished = QtCore.pyqtSignal(bool, str)  # 发送完成信号 (是否成功, 消息)
    signal_point_reached = QtCore.pyqtSignal(int, object)  # 发送到达点信号 (点序号, 位姿)

    def __init__(self, controller=None, trajectory=None, speed=0.05, acc=0.02, dt=0.1,test_mode=False):
        super().__init__()
        self._isPaused = False
        self._isRunning = True
        self.condition = QtCore.QWaitCondition()
        self.mutex = QtCore.QMutex()

        self.controller = controller
        self.trajectory = trajectory  # 轨迹数据 numpy数组 (N, 6)
        self.speed = speed  # 移动速度
        self.acc = acc  # 加速度
        self.dt = dt  # 每个点之间的时间间隔（秒）
        self.test_mode = test_mode  # 测试模式标志，True时不实际控制机械臂



    def pause(self):
        """暂停轨迹跟踪"""
        self._isPaused = True

    def resume(self):
        """恢复轨迹跟踪"""
        self._isPaused = False
        self.condition.wakeAll()

    def stop(self):
        """停止轨迹跟踪"""
        self._isRunning = False
        self.condition.wakeAll()

    def __del__(self):
        self.wait()

    def run(self):
        """主循环 - 遍历轨迹点并移动"""
        try:
            # # 检查控制器和轨迹是否有效
            # if self.controller is None:
            #     self.signal_finished.emit(False, "机械臂控制器未初始化")
            #     return

            if self.trajectory is None or len(self.trajectory) == 0:
                self.signal_finished.emit(False, "轨迹数据为空")
                return

            num_points = len(self.trajectory)
            self.signal_progress.emit(0, num_points)  # 发送开始信号

            # 遍历轨迹点
            for i in range(num_points):
                # 检查是否需要停止
                if not self._isRunning:
                    self.signal_finished.emit(False, "轨迹跟踪已停止")
                    return

                # 检查是否暂停
                self.mutex.lock()
                if self._isPaused:
                    self.condition.wait(self.mutex)
                    self.mutex.unlock()
                    continue
                self.mutex.unlock()

                # 获取当前目标点
                # 获取当前目标点
                target_pose = self.trajectory[i]  # 获取第i个点 [x, y, z, rx, ry, rz]

                try:
                    if self.test_mode:
                        # 测试模式：只打印信息，不实际控制机械臂
                        print(f"【测试模式】第 {i + 1} 个点已到达，位姿为: {target_pose}")
                        # 发送到达点信号
                        self.signal_point_reached.emit(i + 1, target_pose)
                    else:
                        # 正常模式：实际控制机械臂
                        # 使用servoL移动到目标点
                        # servoL是伺服控制，会平滑移动到目标位置
                        self.controller.moveL(target_pose,asy=False, speed=self.speed, acc=self.acc)
                        self.signal_point_reached.emit(i + 1, target_pose)
                        # print(f"第 {i + 1} 个点已到达，位姿为: {target_pose}")

                    # 发送进度信号
                    self.signal_progress.emit(i + 1, num_points)

                    # 等待一段时间，让机械臂有时间移动到目标点
                    # dt是每个点之间的时间间隔，可以根据需要调整
                    time.sleep(self.dt)

                except Exception as e:
                    error_msg = f"移动到第{i + 1}个点时出错: {str(e)}"
                    print(error_msg)
                    self.signal_finished.emit(False, error_msg)
                    return

                # 处理Qt事件，保持UI响应
                QApplication.processEvents()

            # 所有点都执行完毕
            self.signal_finished.emit(True, f"轨迹跟踪完成，共执行 {num_points} 个点")

        except Exception as e:
            error_msg = f"轨迹跟踪过程出错: {str(e)}"
            print(error_msg)
            self.signal_finished.emit(False, error_msg)