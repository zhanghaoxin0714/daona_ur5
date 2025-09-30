# -*- coding: utf-8 -*-
import sys
from PyQt5.QtWidgets import QApplication, QMainWindow
from PyQt5 import QtCore  # 需要添加这个导入
from Qt.widget import Ui_Widget
import time
import ipaddress
import numpy as np  # 需要添加这个导入
from datetime import datetime
from PyQt5.QtChart import *
from PyQt5.QtCore import QMargins
from save_force_data import ForceDataSaver #保存数据按钮
# main.py
#机械臂数据刷新线程
from robot_controller import RobotController
from robot_data_thread import RobotDataThread
#六维力连接控制器
from force_controller import ForceController  # 添加这行
# 六维力数据刷新线程
from force_data_thread import ForceDataThread
# 在 main.py 文件开头添加：
from admittance_thread import AdControlThread
from controller.AdmittancePoseController import AdmittancePoseController
# 其他导入...
import threading
from Robots.URControlAPI import URControlAPI
from controller.AdmittancePoseController import AdmittancePoseController

class MyUi(Ui_Widget):
    def __init__(self):
        super().setupUi(MainWindow) #设置ui界面 调用父类Ui_Widget的setupUi函数
        self.logs = [] #初始化日志列表 创建一个空列表存储日志信息
        self.initUi() #初始化ui事件 绑定按钮事件 初始化图表
        self.force_saver = ForceDataSaver()  # 初始化力数据保存器 将力传感器数据保存excel
        # 控制器和线程初始化
        self.controller = None  # 机械臂控制器  先创建容易存放将来对象 None意味还没有连接机械臂
        self.controlThread = None  # 导纳控制线程
        self.robotDataThread = None  # 机械臂数据线程
        self.forceDataThread = None  # 力传感器数据线程
        self.force_controller = None  # 力传感器控制器

        # 初始化力传感器相关变量
        self.ati_force = None  # 初始化 力传感器线程
        self.force_thread = None  # 初始化 力传感器获取线程

        # 设置默认IP地址
        self.lineEdit_11.setText('192.168.111.10')  # 机器人IP
        self.lineEdit_12.setText('192.168.111.20')  # 力传感器IP

        # 设置默认目标位姿
        self.lineEdit_9.setText('0.153')  # TargetX
        self.lineEdit_10.setText('-0.288')  # TargetY
        self.lineEdit_14.setText('0.585')  # TargetZ
        self.lineEdit_27.setText('1.155')  # TargetRr
        self.lineEdit_28.setText('0.943')  # TargetRp
        self.lineEdit_29.setText('0.173')  # TargetRy

        # 设置导纳参数默认值
        # M矩阵参数 (0.2, 0.2, 0.05, 0.008, 0.008, 0.01)
        self.lineEdit_55.setText('0.200')
        self.lineEdit_59.setText('0.200')
        self.lineEdit_61.setText('0.050')
        self.lineEdit_63.setText('0.008')
        self.lineEdit_53.setText('0.008')
        self.lineEdit_58.setText('0.010')

        # B矩阵参数 (20, 20, 20, 20, 20, 20)
        self.lineEdit_56.setText('20.000')
        self.lineEdit_60.setText('20.000')
        self.lineEdit_62.setText('20.000')
        self.lineEdit_64.setText('20.000')
        self.lineEdit_54.setText('20.000')
        self.lineEdit_57.setText('20.000')

        # K矩阵参数 (50, 50, 100, 30, 30, 30)
        self.lineEdit_95.setText('50.000')
        self.lineEdit_99.setText('50.000')
        self.lineEdit_98.setText('100.000')
        self.lineEdit_96.setText('30.000')
        self.lineEdit_97.setText('30.000')
        self.lineEdit_100.setText('30.000')

         #初始化导纳控制矩阵 创建导纳控制算法需要的数学矩阵
        self.M = np.diag([0.2, 0.2, 0.05, 0.008, 0.008, 0.01])
        self.B = np.diag([20, 20, 20, 20, 20, 20])
        self.K = np.diag([50, 50, 100, 30, 30, 30])

    def initUi(self):
        print("初始化UI...")

        #绑定按钮事件
        self.pushButton_25.clicked.connect(self.ConnectRobotBtnClicked)# 连接机械臂按钮事件
        self.pushButton_26.clicked.connect(self.ConnectForceSensorBtnClicked)# 连接六维力传感器按钮事件
        self.jtbutton.clicked.connect(self.EmergencyStopBtnClicked)# 急停按钮事件
        self.pushButton_4.clicked.connect(self.saveForceData) #保存事件
        self.pushButton_10.clicked.connect(self.startAdmControlButtonClicked) #启动导纳控制按钮
        self.pushButton_22.clicked.connect(self.stopAdmControlButtonClicked)#停止导纳控制按钮

        self.initFTChart() #初始化力传感器图表

        self.addLogs("【INFO】初始化UI成功") #记录日志

    def initFTChart(self):
        self.maxForce = 3
        self.maxTorque = 1
        self.minForce = -3
        self.minTorque = -1

        # 初始化图框
        self.forceChart = QChart()
        self.torqueChart = QChart()
        self.forceChart.setBackgroundVisible(False)
        self.torqueChart.setBackgroundVisible(False)
        self.forceChart.setMargins(QMargins(0, 0, 0, 0))
        self.torqueChart.setMargins(QMargins(0, 0, 0, 0))
        self.forceChart.layout().setContentsMargins(0, 0, 0, 0)
        self.torqueChart.layout().setContentsMargins(0, 0, 0, 0)
        self.forceChart.setBackgroundRoundness(0)
        self.torqueChart.setBackgroundRoundness(0)

        # 初始化曲线
        self.forceXSeries = QLineSeries()
        self.forceYSeries = QLineSeries()
        self.forceZSeries = QLineSeries()
        self.torqueXSeries = QLineSeries()
        self.torqueYSeries = QLineSeries()
        self.torqueZSeries = QLineSeries()

        # 设置曲线名称
        self.forceXSeries.setName("Fx")
        self.forceYSeries.setName("Fy")
        self.forceZSeries.setName("Fz")
        self.torqueXSeries.setName("Tx")
        self.torqueYSeries.setName("Ty")
        self.torqueZSeries.setName("Tz")

        # 将曲线添加到图框中
        self.forceChart.addSeries(self.forceXSeries)
        self.forceChart.addSeries(self.forceYSeries)
        self.forceChart.addSeries(self.forceZSeries)
        self.torqueChart.addSeries(self.torqueXSeries)
        self.torqueChart.addSeries(self.torqueYSeries)
        self.torqueChart.addSeries(self.torqueZSeries)

        # 设置坐标轴
        self.TimeAxis1 = QValueAxis()
        self.TimeAxis2 = QValueAxis()
        self.ForceAxis = QValueAxis()
        self.TorqueAxis = QValueAxis()
        self.TimeAxis1.setRange(0, 30)
        self.TimeAxis2.setRange(0, 30)
        self.ForceAxis.setRange(-1, 1)
        self.TorqueAxis.setRange(-1, 1)
        self.TimeAxis1.setTitleText("Time(s)")
        self.TimeAxis2.setTitleText("Time(s)")
        self.ForceAxis.setTitleText("Force(N)")
        self.TorqueAxis.setTitleText("Torque(Nm)")


        self.forceChart.setAxisX(self.TimeAxis1)
        self.forceChart.setAxisY(self.ForceAxis)
        self.torqueChart.setAxisX(self.TimeAxis2)
        self.torqueChart.setAxisY(self.TorqueAxis)

        # 关联曲线
        self.forceXSeries.attachAxis(self.TimeAxis1)
        self.forceXSeries.attachAxis(self.ForceAxis)
        self.forceYSeries.attachAxis(self.TimeAxis1)
        self.forceYSeries.attachAxis(self.ForceAxis)
        self.forceZSeries.attachAxis(self.TimeAxis1)
        self.forceZSeries.attachAxis(self.ForceAxis)
        self.torqueXSeries.attachAxis(self.TimeAxis2)
        self.torqueXSeries.attachAxis(self.TorqueAxis)
        self.torqueYSeries.attachAxis(self.TimeAxis2)
        self.torqueYSeries.attachAxis(self.TorqueAxis)
        self.torqueZSeries.attachAxis(self.TimeAxis2)
        self.torqueZSeries.attachAxis(self.TorqueAxis)

        # 设置更新动画
        self.forceChart.setAnimationOptions(QChart.SeriesAnimations)
        self.torqueChart.setAnimationOptions(QChart.SeriesAnimations)
         # 将chart显示到界面 - 修正组件名称
        self.plotF.setChart(self.forceChart)
        self.plotT.setChart(self.torqueChart)

    def ConnectRobotBtnClicked(self):
        """连接捕获机械臂"""

        print('连接机器人...')
        self.targetIP = self.lineEdit_11.text()  # 获取机器人IP地址 #获取IP地址

        # 验证机械臂IP地址有效性
        if self.isIP(self.targetIP):
            try:
                # 创建机器人控制器实例
                self.controller = RobotController(self.targetIP)
            except:
                print('连接目标星机械臂失败')
                self.addLogs('【ERROR】连接目标星机械臂失败，请确认UR已运行合适的脚本')
                return
            self.addLogs('【INFO】连接目标星机械臂，IP为：', self.targetIP)#日志添加信息
            # 记录开始时间
            self.StartTime = time.time()#获取当前时间戳，记录连接成功的时间
            # 启动机械臂数据刷新线程
            self.start_robot_data_thread()
            # 更新连接状态指示器（红色变为绿色）
            self.label_139.setStyleSheet(
                "background-color: green; border-radius: 10px; min-height: 20px; max-height: 20px; min-width: 20px; max-width: 20px;")
            # 更新六维力IP地址
            self.lineEdit_12.setText('192.168.111.20')

            return True
        else:
            print('IP地址无效，请重新输入')
            self.addLogs('【ERROR】IP地址无效，请重新输入')
            return False

    def ConnectForceSensorBtnClicked(self):
        """单独连接力传感器"""
        try:
            force_host = self.lineEdit_12.text()#从ui界面获取传感器ip地址
            if not self.isIP(force_host):#验证IP地址有效性
                self.addLogs('【ERROR】力传感器IP地址无效')
                return

             # 创建力传感器控制器（在构造函数中自动连接）
            self.force_controller = ForceController(force_host)

            if self.force_controller.is_connected:
                self.addLogs('【INFO】连接力传感器成功，IP为：', force_host)
                # 启动力传感器数据记录功能
                self.force_saver.start_recording()
                # 启动力传感器数据刷新线程
                self.start_force_data_thread()
                # 更新力传感器状态指示器
                self.label_140.setStyleSheet(
                    "background-color: green; border-radius: 10px; min-height: 20px; max-height: 20px; min-width: 20px; max-width: 20px;")
            else:
                self.addLogs('【WARNING】力传感器连接失败')


        except Exception as e:
            self.addLogs('【ERROR】连接力传感器失败：', str(e))
            # 更新力传感器状态指示器为红色
            self.label_140.setStyleSheet(
                "background-color: red; border-radius: 10px; min-height: 20px; max-height: 20px; min-width: 20px; max-width: 20px;")


    def start_robot_data_thread(self):
        """启动机械臂数据刷新线程"""
        if self.controller and self.forceDataThread is None:
            #检查六维力控制器是否创建 检查数据刷新线程是否未启动

            # 创建位姿数据记录文件
            self.posePath = f"res/pose/{str(datetime.now())[:-7].replace(':', '-')}.txt"
            self.poseRecord = open(self.posePath, "a+")#创建并打开数据文件

            # 开启机械臂数据刷新线程
            self.robotDataThread = RobotDataThread(self.controller)#创建机械臂数据获取线程 传入机械臂控制器
            self.robotDataThread._signal_pose.connect(self.ShowTargetEEPoseCallback)#将线程的位姿信号连接到ui更新函数
            self.robotDataThread._signal_joint.connect(self.ShowTargetJointCallback)#将线程的关节信号连接到ui更新函数
            self.robotDataThread._signal_timestamp.connect(self.RefreshTimeCallback)#将线程的时间戳信号连接到ui更新函数
            self.robotDataThread.start()#启动线程

            self.addLogs('【INFO】机械臂数据刷新线程已启动')

    def start_force_data_thread(self):
        """启动力传感器数据刷新线程"""
        if self.force_controller is not None and self.forceDataThread is None:
            # 力传感器控制器已创建 力传感器已连接 力传感器数据线程未启动
            # 创建力传感器数据记录文件
            self.forcePath = f"res/force/{str(datetime.now())[:-7].replace(':', '-')}.txt"
            self.forceRecord = open(self.forcePath, "a+")

            # 开启力传感器数据刷新线程
            self.forceDataThread = ForceDataThread(self.force_controller,self.controller)  # 创建线程对象传入机械臂控制器
            self.forceDataThread._signal_ft.connect(self.DrawFTCallback)  # 将力传感器数据连接ui
            self.forceDataThread._signal_timestamp.connect(self.RefreshTimeCallback)  # 连接时间戳
            self.forceDataThread.start()

            self.addLogs('【INFO】力传感器数据刷新线程已启动')

    def startAdmControlButtonClicked(self):
        """启动导纳控制"""
        if self.controller is None:
            self.addLogs('【WARNING】请先连接机械臂')
            return
        #
        # if not self.controller.is_force_sensor_connected():
        #     self.addLogs('【WARNING】请先连接力传感器')
        #     return
#检查机械臂与六维力是否连接

        # 更新导纳参数 - 使用字典映射
        input_controls = {
            "M": {
                "px": (self.lineEdit_55, (0, 0)),
                "py": (self.lineEdit_59, (1, 1)),
                "pz": (self.lineEdit_61, (2, 2)),
                "rx": (self.lineEdit_63, (3, 3)),
                "ry": (self.lineEdit_53, (4, 4)),
                "rz": (self.lineEdit_58, (5, 5))
            },
            "B": {
                "px": (self.lineEdit_56, (0, 0)),
                "py": (self.lineEdit_60, (1, 1)),
                "pz": (self.lineEdit_62, (2, 2)),
                "rx": (self.lineEdit_64, (3, 3)),
                "ry": (self.lineEdit_54, (4, 4)),
                "rz": (self.lineEdit_57, (5, 5))
            },
            "K": {
                "px": (self.lineEdit_95, (0, 0)),
                "py": (self.lineEdit_99, (1, 1)),
                "pz": (self.lineEdit_98, (2, 2)),
                "rx": (self.lineEdit_96, (3, 3)),
                "ry": (self.lineEdit_97, (4, 4)),
                "rz": (self.lineEdit_100, (5, 5))
            }
        }

        # 遍历控件字典并更新矩阵
        for param, textBoxNames in input_controls.items():
            for _, (textbox, idx) in textBoxNames.items():
                if textbox.text() != '' and self.is_float(textbox.text()):
                    if param == "M":
                        self.M[idx] = float(textbox.text())
                    elif param == "B":
                        self.B[idx] = float(textbox.text())
                    elif param == "K":
                        self.K[idx] = float(textbox.text())

        self.addLogs(f"质量参数：{self.M}")
        self.addLogs(f"阻尼参数：{self.B}")
        self.addLogs(f"刚度参数：{self.K}")
#更新导纳参数
        # 获取目标位置
        try:
            TargetX = float(self.lineEdit_9.text())
            TargetY = float(self.lineEdit_10.text())
            TargetZ = float(self.lineEdit_14.text())
            TargetRr = float(self.lineEdit_27.text())
            TargetRp = float(self.lineEdit_28.text())
            TargetRy = float(self.lineEdit_29.text())
        except ValueError:
            self.addLogs('【ERROR】目标位置输入的数值无效，请重新输入')
            return

        targetPos = np.array([TargetX, TargetY, TargetZ, TargetRr, TargetRp, TargetRy])

        try:
            self.controller.go_startPose(targetPos)
            self.addLogs("【INFO】已移动到目标位置")
        except Exception as e:
            self.addLogs(f'【ERROR】移动到工作点失败: {e}')
            return

        self.controller.adcontrol = AdmittancePoseController(targetPos, self.controller.dt)

        # 设置力传感器控制器到机械臂控制器
        if self.force_controller is not None:
            self.controller.set_force_controller(self.force_controller)

        try:
            self.controlThread = AdControlThread(self.controller, self.M, self.B, self.K)
            self.controlThread.start()
            self.addLogs("【INFO】导纳控制已启动")
        except Exception as e:
            self.addLogs(f'【ERROR】控制线程启动失败: {e}')

    def stopAdmControlButtonClicked(self):
        """停止导纳控制"""
        try:
            # 停止导纳控制线程
            if self.controlThread is not None:
                self.controlThread.pause()
                self.addLogs("【INFO】导纳控制已停止")

            # 停止机器人运动
            if self.controller is not None:
                self.controller.stop_robot()
                self.addLogs("【INFO】机器人已停止")

        except Exception as e:
            self.addLogs(f'【ERROR】停止导纳控制失败: {e}')

    def is_float(self, s):
        """检查字符串是否为浮点数"""
        try:
            float(s)
            return True
        except ValueError:
            return False

    def isIP(self, ip_str):
        """
        判断IP地址合法性

        Args:
            ip_str (str): IP地址

        Returns:
            boolean: True为合法，False为不合法
        """
        try:
            ipaddress.ip_address(ip_str)#调用Python的ip_address模块 验证IP格式
            #1 是否四个数字 2 0-255范围 3 数字之间点分符 4 格式是否正确
            return True
        except ValueError:
            return False

    def EmergencyStopBtnClicked(self):
        """急停按钮事件"""
        print('急停按钮被按下！')
        self.addLogs('【WARNING】急停按钮被按下！')

    def addLogs(self, *args, split=''):
        """添加日志到界面"""
        newLog = split.join(args)
        self.logs.append(newLog)
        self.plainTextEdit.setPlainText("\n".join(self.logs))
        # 自动滚动到最新内容
        self.plainTextEdit.verticalScrollBar().setValue(
            self.plainTextEdit.verticalScrollBar().maximum()
        )

    def ShowTargetEEPoseCallback(self, x, y, z, rr, rp, ry):
        """更新机械臂末端姿态显示"""
        # 记录数据到文件
        dataline = f"{x}, {y}, {z}, {rr}, {rp}, {ry}" + "\n"#将机械臂位姿数据格式化为字符串
        self.poseRecord.write(dataline)#将格式化的数据写入到记录文件中

        # 更新捕获机械臂位姿显示（位置单位：mm，姿态单位：度）
        self.lineEdit_70.setText(f"{x * 1000:.2f}")  # X位置（m转mm）
        self.lineEdit_73.setText(f"{y * 1000:.2f}")  # Y位置（m转mm）
        self.lineEdit_71.setText(f"{z * 1000:.2f}")  # Z位置（m转mm）
        self.lineEdit_67.setText(f"{np.rad2deg(rr):.2f}")  # Rr姿态（弧度转度）
        self.lineEdit_65.setText(f"{np.rad2deg(rp):.2f}")  # Rp姿态（弧度转度）
        self.lineEdit_68.setText(f"{np.rad2deg(ry):.2f}")  # Ry姿态（弧度转度）

    def ShowTargetJointCallback(self, q):
        """更新机械臂关节角显示"""
        # 更新捕获机械臂关节角显示（单位：度）
        self.lineEdit_17.setText(f"{np.rad2deg(q[0]):.2f}")  # 关节角1 弧度转为角度
        self.lineEdit_15.setText(f"{np.rad2deg(q[1]):.2f}")  # 关节角2
        self.lineEdit_16.setText(f"{np.rad2deg(q[2]):.2f}")  # 关节角3
        self.lineEdit_41.setText(f"{np.rad2deg(q[3]):.2f}")  # 关节角4
        self.lineEdit_42.setText(f"{np.rad2deg(q[4]):.2f}")  # 关节角5
        self.lineEdit_46.setText(f"{np.rad2deg(q[5]):.2f}")  # 关节角6

    def DrawFTCallback(self, ft):
        """
        # 【作用】更新六维力传感器曲线
        """
        ft = np.array(ft).reshape(6, 1)
        self.maxForce = max(self.maxForce, np.linalg.norm(ft[0:3]))
        self.maxTorque = max(self.maxTorque, np.linalg.norm(ft[3:6]))

        # 第442-447行，需要修改为：
        self.forceXSeries.append(self.controlTime, float(ft[0]))
        self.forceYSeries.append(self.controlTime, float(ft[1]))
        self.forceZSeries.append(self.controlTime, float(ft[2]))
        self.torqueXSeries.append(self.controlTime, float(ft[3]))
        self.torqueYSeries.append(self.controlTime, float(ft[4]))
        self.torqueZSeries.append(self.controlTime, float(ft[5]))

        # self.forceXSeries.append(self.controlTime, ft[0])
        # self.forceYSeries.append(self.controlTime, ft[1])
        # self.forceZSeries.append(self.controlTime, ft[2])
        # self.torqueXSeries.append(self.controlTime, ft[3])
        # self.torqueYSeries.append(self.controlTime, ft[4])
        # self.torqueZSeries.append(self.controlTime, ft[5])

        self.lineEdit_109.setText(str('%.2f' % ft[0].item()))
        self.lineEdit_110.setText(str('%.2f' % ft[1].item()))
        self.lineEdit_141.setText(str('%.2f' % ft[2].item()))
        self.lineEdit_142.setText(str('%.2f' % ft[3].item()))
        self.lineEdit_143.setText(str('%.2f' % ft[4].item()))
        self.lineEdit_144.setText(str('%.2f' % ft[5].item()))

        if self.controlTime > 30:
            self.TimeAxis1.setRange(self.controlTime - 30, self.controlTime)
            self.TimeAxis2.setRange(self.controlTime - 30, self.controlTime)
        else:
            self.TimeAxis1.setRange(0, self.controlTime)
            self.TimeAxis2.setRange(0, self.controlTime)

        maxRange = max(self.maxForce, self.maxTorque)
        self.ForceAxis.setRange(-maxRange, maxRange)
        self.TorqueAxis.setRange(-maxRange, maxRange)

        dataline = f"{ft.T}" + "\n"
        self.forceRecord.write(dataline)
        # print(f"DrawFTCallback 接收到数据: {ft}")  # 添加调试信息

        # ft = np.array(ft).reshape(6, 1)#力数据转换为数组格式
        # self.maxForce = max(self.maxForce, np.linalg.norm(ft[0:3]))#取最大值
        # self.maxTorque = max(self.maxTorque, np.linalg.norm(ft[3:6]))
        # #更新图表数据
        #
        # self.forceXSeries.append(self.controlTime, ft[0].item())
        # self.forceYSeries.append(self.controlTime, ft[1].item())
        # self.forceZSeries.append(self.controlTime, ft[2].item())
        # self.torqueXSeries.append(self.controlTime, ft[3].item())
        # self.torqueYSeries.append(self.controlTime, ft[4].item())
        # self.torqueZSeries.append(self.controlTime, ft[5].item())
        # #更新文本界面
        #
        # self.lineEdit_109.setText(str('%.2f' % ft[0].item()))
        # self.lineEdit_110.setText(str('%.2f' % ft[1].item()))
        # self.lineEdit_141.setText(str('%.2f' % ft[2].item()))
        # self.lineEdit_142.setText(str('%.2f' % ft[3].item()))
        # self.lineEdit_143.setText(str('%.2f' % ft[4].item()))
        # self.lineEdit_144.setText(str('%.2f' % ft[5].item()))
        # #调整时间轴范围
        #
        # if self.controlTime > 30:
        #     self.TimeAxis1.setRange(self.controlTime - 30, self.controlTime)
        #     self.TimeAxis2.setRange(self.controlTime - 30, self.controlTime)
        # else:
        #     self.TimeAxis1.setRange(0, self.controlTime)
        #     self.TimeAxis2.setRange(0, self.controlTime)
        #     #根据最大值调整力轴范围
        #
        # maxRange = max(self.maxForce, self.maxTorque)
        # self.ForceAxis.setRange(-maxRange, maxRange)
        # self.TorqueAxis.setRange(-maxRange, maxRange)
        # if hasattr(self, 'forceRecord') and self.forceRecord is not None:
        #
        #     dataline = f"{ft.T}" + "\n"
        #     self.forceRecord.write(dataline)
        #
        # self.force_saver.add_force_data(ft) #保存六维力数据

    def RefreshTimeCallback(self, refreshTime):
        """
        【作用】更新界面中的绘图时间
        """
        self.refreshTime = refreshTime#保存当前时间戳
        self.controlTime = refreshTime - self.StartTime#当前时间-程序启动时间戳 记录运行时间
        # print('TimeStamp:', self.controlTime)

    def saveForceData(self):
        """保存六维力数据到Excel"""
        try:
            # 在保存前检查是否有数据
            if not self.force_saver.force_data:
                self.addLogs('【WARNING】没有可保存的力传感器数据')
                return

            self.addLogs('【INFO】开始保存力传感器数据...')

            # 直接在主线程中保存，不使用后台线程
            self.force_saver.save_to_excel(MainWindow)

            self.addLogs('【INFO】力传感器数据保存完成')

        except Exception as e:
            self.addLogs(f'【ERROR】保存数据失败: {e}')

    def _save_force_data_thread(self):
        """在后台线程中保存数据"""
        try:
            self.addLogs('【INFO】开始保存数据...')

            # 检查数据
            if not hasattr(self.force_saver, 'force_data'):
                self.addLogs('【ERROR】force_saver.force_data 不存在')
                return

            if not self.force_saver.force_data:
                self.addLogs('【ERROR】没有可保存的数据')
                return

            self.addLogs(f'【INFO】准备保存 {len(self.force_saver.force_data)} 条数据')

            # 调用保存方法
            self.force_saver.save_to_excel(MainWindow)

            self.addLogs('【INFO】力传感器数据保存完成')

        except Exception as e:
            # 简化异常处理
            error_msg = f'【ERROR】保存数据失败: {str(e)}'
            self.addLogs(error_msg)
            print(f"保存失败: {e}")

if __name__ == '__main__':#Python的标准入口点检查
    # 创建Qt界面
    app = QApplication(sys.argv) #创建Qt应用程序案例 理解：创建一个应用程序的容器
    MainWindow = QMainWindow() #创建主窗口对象 理解 ：有一个框架 放置各种控件
    ui = MyUi()#调用函数 创建ui实例

    MainWindow.show() #显示主界面
    sys.exit(app.exec_()) #启动Qt事件