# admittance_thread.py
import time
import numpy as np
from PyQt5 import QtCore
from PyQt5.QtWidgets import QApplication


class AdControlThread(QtCore.QThread):
    """导纳控制线程"""

    def __init__(self, controller=None, M=None, B=None, K=None):
        super().__init__()  # 调用Qt基本功能
        self._isPaused = False  # 设置暂停标志 不暂停
        self._stop = False #停止标志位
        #self._isRunning = True
        self.condition = QtCore.QWaitCondition()  # 创建线程同步条件，用于暂停/恢复机制
        self.mutex = QtCore.QMutex()  # 互斥锁 保证线程安全

        self.controller = controller
        self.M = M
        self.B = B
        self.K = K

    def pause(self):
        self._isPaused = True

    def stop(self):
        self.mutex.lock()
        self._stop = True
        self._isPaused = False
        self.condition.wakeAll()
        self.mutex.unlock()

    def resume(self):
        self._isPaused = False
        self.condition.wakeAll()

    def __del__(self):
        self.wait()

    def run(self):
        while not self._stop:
            self.mutex.lock()
            if self._isPaused:
                self.condition.wait(self.mutex)
                if self._stop:
                    self.mutex.unlock()
                    break
                self.mutex.unlock()
                continue

            self.controller.pose_stable(  # 定点位姿导纳控制
                            self.M, self.B, self.K)  # 执行导纳控制算法
                # 使用速度控制方式（让末端随力移动）
            # self.controller.pose_stable_velocityBased(
            #     self.M, self.B, self.K)

            self.mutex.unlock()
            time.sleep(0.005)  # 200Hz控制频率
            QApplication.processEvents()