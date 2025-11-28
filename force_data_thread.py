# force_data_thread.py
import time
import numpy as np
from PyQt5 import QtCore
from PyQt5.QtWidgets import QApplication



class ForceDataThread(QtCore.QThread):
    """力传感器数据刷新线程 - 只负责力传感器数据"""
    _signal_ft = QtCore.pyqtSignal(list)#调用父类构造函数 初始化QT基本功能
    _signal_timestamp = QtCore.pyqtSignal(float)

    def __init__(self, force_sensor=None,robot_controller=None):
        super().__init__()#初始化暂停标志
        self._isPaused = False#创建线程同步的等待条件 实现线程暂停恢复
        self.condition = QtCore.QWaitCondition()#创建互斥锁保证安全
        self.mutex = QtCore.QMutex()#保存传入的机械臂控制器对象
        self.force_sensor = force_sensor
        self.robot_controller = robot_controller  # 添加机械臂控制器参数

    def pause(self):
        self._isPaused = True

    def resume(self):
        self._isPaused = False
        self.condition.wakeAll()

    def __del__(self):
        self.wait()

    def run(self):
        while True:
            self.mutex.lock()#获取互斥锁 保护
            if self._isPaused:#暂停检查
                self.condition.wait(self.mutex)

            # 只获取力传感器数据
            if self.force_sensor is not None and self.force_sensor.is_force_sensor_connected():
                #机械臂控制器存在 力传感器已连接
                ft = self.force_sensor.get_cur_force(robot_controller=self.robot_controller,for_display = True)#获取力传感器数据

                ft = np.array(ft).reshape(-1).tolist()#转换列表形式

                # print(ft);
            else:
                ft = [0, 0, 0, 0, 0, 0]  # 默认值

            # print(ft);

            # 发送数据信号到界面
            self._signal_timestamp.emit(time.time())#发送时间戳
            self._signal_ft.emit(ft)#发送力数据信号
            QApplication.processEvents()

            self.mutex.unlock()
            time.sleep(0.2)


