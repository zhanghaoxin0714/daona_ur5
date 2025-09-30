# robot_data_thread.py
import time
import numpy as np
from PyQt5 import QtCore
from PyQt5.QtWidgets import QApplication

class RobotDataThread(QtCore.QThread):
    """数据刷新线程 - 实时更新界面数据"""
    _signal_pose = QtCore.pyqtSignal(float, float, float, float, float, float)  # 位姿信号
    _signal_joint = QtCore.pyqtSignal(list)
    _signal_timestamp = QtCore.pyqtSignal(float)

    def __init__(self, controller=None):
        super().__init__()  # 调用父类函数 初始化线程的基本属性
        self._isPaused = False  # 暂停标志
        self.condition = QtCore.QWaitCondition()  # 实现线程的等待和唤醒
        self.mutex = QtCore.QMutex()  # 互斥锁 保护资源防止数据竞争
        self.controller = controller  # 保存机械臂控制器的引用

    def pause(self):  # 暂停
        self._isPaused = True

    def resume(self):  # 唤醒
        self._isPaused = False
        self.condition.wakeAll()

    def __del__(self):
        self.wait()  # 等待线程完成 确保线程安全退出

    def run(self):
        while True:
            self.mutex.lock()
            if self._isPaused:
                self.condition.wait(self.mutex)

            # 获取机械臂数据
            x, y, z, rr, rp, ry = self.controller.get_end_tip()  # 获取机械臂位姿
            q = self.controller.get_q()  # 获取关节角

            # 发送数据信号到界面
            self._signal_timestamp.emit(time.time())  # emit发射信号到连接的槽函数
            self._signal_pose.emit(x, y, z, rr, rp, ry)  # 发射位姿信号
            self._signal_joint.emit(q)
            QApplication.processEvents()  # 处理Qt事件队列 确保信号能及时传递到ui

            self.mutex.unlock()  # 释放互斥锁
            time.sleep(0.2)