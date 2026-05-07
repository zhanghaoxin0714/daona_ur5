# -*- coding: utf-8 -*-
# velocity_control_thread.py
import time
import numpy as np
from PyQt5 import QtCore
from PyQt5.QtWidgets import QApplication
from PyQt5.QtCore import QMutexLocker


class VelocityControlThread(QtCore.QThread):
    """速度控制线程 - 持续发送速度命令控制机器人运动"""

    def __init__(self, controller=None, target_velocity=None):
        super().__init__()
        self._isPaused = False
        self._isRunning = True
        self.condition = QtCore.QWaitCondition()
        self.mutex = QtCore.QMutex()

        self.controller = controller
        self.target_velocity = target_velocity  # 目标速度 [Vx, Vy, Vz, Rx, Ry, Rz]

    def pause(self):
        """暂停控制"""
        self._isPaused = True

    def resume(self):
        """恢复控制"""
        self._isPaused = False
        self.condition.wakeAll()

    def stop(self):
        """停止控制"""
        self._isRunning = False
        self.condition.wakeAll()

    def set_target_velocity(self, velocity):
        locker = QMutexLocker(self.mutex)
        self.target_velocity = np.array(velocity).reshape(6, 1)

    def __del__(self):
        self.wait()

    def run(self):
        """主循环 - 持续发送速度命令"""
        while self._isRunning:
            self.mutex.lock()
            if self._isPaused:
                self.condition.wait(self.mutex)
                self.mutex.unlock()
                continue

            # 检查控制器和目标速度是否有效
            if self.controller is None or self.target_velocity is None:
                self.mutex.unlock()
                time.sleep(0.1)
                continue

            try:
                # 速度限幅（安全保护）
                max_linear_velocity = 0.1  # 最大线速度 m/s
                max_angular_velocity = 0.5  # 最大角速度 rad/s

                velocity = self.target_velocity.copy()

                # 限制线速度
                linear_vel = velocity[:3]
                linear_vel_norm = np.linalg.norm(linear_vel)
                if linear_vel_norm > max_linear_velocity:
                    velocity[:3] = linear_vel / linear_vel_norm * max_linear_velocity

                # 限制角速度
                angular_vel = velocity[3:]
                angular_vel_norm = np.linalg.norm(angular_vel)
                if angular_vel_norm > max_angular_velocity:
                    velocity[3:] = angular_vel / angular_vel_norm * max_angular_velocity

                # 发送速度命令到机器人（基坐标系下的速度）
                self.controller.speedL(velocity.flatten().tolist(), acc=0.25, control_period=0.02)

            except Exception as e:
                print(f"速度控制错误: {e}")

            self.mutex.unlock()
            time.sleep(0.02)  # 50Hz控制频率（每20ms发送一次命令）
            QApplication.processEvents()
