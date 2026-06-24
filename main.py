# -*- coding: utf-8 -*-

# ==================== 标准库导入 ====================
import sys
import time
import threading
import ipaddress
import cv2
from datetime import datetime
# ==================== 第三方库导入 ====================
import numpy as np
# ==================== PyQt5 相关导入 ====================
from PyQt5.QtWidgets import QApplication, QMainWindow, QFileDialog
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtCore import Qt
from PyQt5 import QtCore
from PyQt5.QtChart import *
from PyQt5.QtCore import QMargins
from nltk import add_logs

# ==================== 项目内部UI相关 ====================
from Qt.widget2 import Ui_Widget
# ==================== 项目内部模块导入 - 机械臂相关 ====================
from robot.robot_controller import RobotController
from robot.robot_data_thread import RobotDataThread
from robot.admittance_thread import AdControlThread#导纳控制
from robot.trajectory_manager import TrajectoryManager#轨迹跟踪相关
from robot.trajectory_control_thread import TrajectoryControlThread
from controller.AdmittancePoseController import AdmittancePoseController
from robot.Robots.URControlAPI import URControlAPI
# ==================== 项目内部模块导入 - 力传感器相关 ====================
from ATI.force_controller import ForceController
from ATI.force_data_thread import ForceDataThread
from ATI.save_force_data import ForceDataSaver
# ==================== 项目内部模块导入 - 详细实现相关 ====================
from detail import Detail
# ==================== 项目内部模块导入 - 视觉伺服相关 ====================
from visual_servo.camera_handler import CameraHandler
from visual_servo.camera_preview_thread import CameraPreviewThread
# ==================== 项目内部模块导入 - ORU项目 ====================
import oru_actions


class MyUi(Ui_Widget):
    def __init__(self):
        # ==================== UI初始化 ====================
        super().setupUi(MainWindow)  # 设置ui界面，调用父类Ui_Widget的setupUi函数
        # ==================== 基础数据初始化 ====================
        self.logs = []  # 初始化日志列表，创建一个空列表存储日志信息
        # ==================== 创建Detail实例 ====================
        self.detail = Detail(self)  # 传入self作为ui_instance
        self.initUi()  # 初始化ui事件，绑定按钮事件，初始化图表
        # ==================== 控制器初始化 ====================
        self.controller = None  # 机械臂控制器，先创建容易存放将来对象，None意味还没有连接机械臂
        self.force_controller = None  # 力传感器控制器
        self.force_saver = ForceDataSaver()  # 初始化力数据保存器，将力传感器数据保存excel
        # ==================== 线程初始化 ====================
        self.controlThread = None  # 导纳控制线程
        self.robotDataThread = None  # 机械臂数据线程
        self.forceDataThread = None  # 力传感器数据线程
        self.velocity_control_thread = None  # 速度控制线程
        self.ati_force = None  # 初始化力传感器线程
        self.force_thread = None  # 初始化力传感器获取线程
        # =================== 状态标志初始化 ====================
        self.is_velocity_control_running = False  # 速度控制运行标志
        self.xvni = False  # 虚拟力开关，默认关闭
        # ==================== 轨迹管理初始化 ====================
        self.trajectory_manager = TrajectoryManager()  # 轨迹数据管理器
        self.trajectory_control_thread = None  # 轨迹跟踪控制线程


        # ==================== UI默认值设置 ====================
        self.detail.init_ui_default_values()  # 初始化UI默认值
        # ==================== 导纳控制矩阵初始化 ====================
        self.detail.init_admittance_matrices()  # 初始化导纳控制矩阵

        # ==================== 视觉伺服相关初始化 ====================
        # ==================== 视觉伺服相关初始化 ====================
        self.camera_handler = None  # 相机处理器
        self.camera_preview_thread = None  # 相机预览线程
        self.detection_enabled = False  # ArUco检测开关

    def initUi(self):
        print("初始化UI...")

        #绑定按钮事件
        # 绑定按钮事件 - 修改按钮名称
        self.pushButton_CRobot.clicked.connect(self.ConnectRobotBtnClicked)  # 连接机械臂按钮事件
        self.pushButton_CForce.clicked.connect(self.ConnectForceSensorBtnClicked)  # 连接六维力传感器按钮事件
        self.jtbutton.clicked.connect(self.EmergencyStopBtnClicked)  # 急停按钮事件
        self.pushButton_CForceSave.clicked.connect(self.saveForceData)  # 保存事件
        self.pushButton_8.clicked.connect(self.targetMove) #进行目标点移动
        self.pushButton_adStart.clicked.connect(self.startAdmControlButtonClicked)  # 启动导纳控制按钮
        self.pushButton_adStop.clicked.connect(self.stopAdmControlButtonClicked)  # 停止导纳控制按钮
        # 速度控制按钮绑定
        self.pushButton_9.clicked.connect(self.startVelocityControlBtnClicked)  # 开始速度控制
        self.pushButton_17.clicked.connect(self.stopVelocityControlBtnClicked)  # 停止速度控制
        self.PushButtonstartxn.clicked.connect(self.startxn) #开启虚拟力
        self.PushButtonclosexn.clicked.connect(self.closexn) #关闭虚拟力
        # 轨迹控制按钮绑定
        self.pushButton_12.clicked.connect(self.importTrajectoryBtnClicked)  # 导入轨迹按钮
        self.pushButton_13.clicked.connect(self.deleteTrajectoryBtnClicked)  # 删除轨迹按钮
        self.pushButton_14.clicked.connect(self.startTrajectoryBtnClicked)  # 开始轨迹跟踪按钮
        self.pushButton_18.clicked.connect(self.stopTrajectoryBtnClicked)  # 停止轨迹跟踪按钮
        # 视觉伺服相关按钮绑定
        self.pushButton_16.clicked.connect(self.startCameraBtnClicked)  # 启动摄像头按钮
        self.pushButton_11.clicked.connect(self.closeCameraBtnClicked)  # 关闭摄像头按钮
        self.pushButton_6.clicked.connect(self.targetObservationBtnClicked)  # 目标观测按钮
        #oru相关
        self.pushButton_5.clicked.connect(self.on_prepare_passive_clicked) #移动至被动端上方
        self.pushButton_7.clicked.connect(self.on_grab_passive_clicked)  # 抓取被动端
        self.pushButton_10.clicked.connect(self.on_passive_insert_clicked)   # 被动端插入
        self.pushButton_22.clicked.connect(self.A_yelu_clicked)  # 被动端插入
        self.pushButton_24.clicked.connect(self.yelu_dianlu_clicked) #液体移动至电路
        self.pushButton_25.clicked.connect(self.bei_pian_clicked)  # 被动端偏执
        self.pushButton_26.clicked.connect(self.bei_hui_clicked)  # 被动端回调
        self.pushButton_29.clicked.connect(self.luosi_clicked)  # 被动端回调
        self.pushButton.clicked.connect(self.luosiA_clicked)  # 移动到螺丝A
        self.pushButton_2.clicked.connect(self.open_shijiao)  # 开启示教模式
        self.pushButton_4.clicked.connect(self.close_shijiao)  # 关闭示教模式





        self.detail.init_ft_chart()  # 初始化力传感器图表
        self.addLogs("【INFO】初始化UI成功") #记录日志



    def ConnectRobotBtnClicked(self):
        """连接捕获机械臂"""
        print('连接机器人...')
        self.targetIP = self.lineEdit_CrobotIP.text()  # 获取机器人IP地址 #获取IP地址
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
            self.Robot_StartTime = time.time()#获取当前时间戳，记录连接成功的时间
            # 启动机械臂数据刷新线程
            self.start_robot_data_thread()
            # 更新连接状态指示器（红色变为绿色）
            self.label_139.setStyleSheet(
                "background-color: green; border-radius: 10px; min-height: 20px; max-height: 20px; min-width: 20px; max-width: 20px;")
            # 更新六维力IP地址
            self.lineEdit_CforceIP.setText('192.168.111.20')
            return True
        else:
            print('IP地址无效，请重新输入')
            self.addLogs('【ERROR】IP地址无效，请重新输入')
            return False

    def ConnectForceSensorBtnClicked(self):
        """单独连接力传感器"""
        try:
            force_host = self.lineEdit_CforceIP.text()#从ui界面获取传感器ip地址
            if not self.isIP(force_host):#验证IP地址有效性
                self.addLogs('【ERROR】力传感器IP地址无效')
                return
             # 创建力传感器控制器（在构造函数中自动连接）
            self.force_controller = ForceController(force_host)
            if self.force_controller.is_connected:
                self.addLogs('【INFO】连接力传感器成功，IP为：', force_host)
                # 记录开始时间
                self.Force_StartTime = time.time()  # 获取当前时间戳，记录连接成功的时间
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

            self.jointPath = f"res/joint/{str(datetime.now())[:-7].replace(':', '-')}.txt"
            self.jointRecord = open(self.jointPath, "a+")
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



    def targetMove(self):
        if self.controller is None:
            self.addLogs('【WARNING】请先连接机械臂')
            return
            # 获取目标位置
        try:
            TargetX = float(self.lineEdit_CtargetX.text())
            TargetY = float(self.lineEdit_CtargetY.text())
            TargetZ = float(self.lineEdit_CtargetZ.text())
            TargetRr = float(self.lineEdit_CtargetRr.text())
            TargetRp = float(self.lineEdit_CtargetRp.text())
            TargetRy = float(self.lineEdit_CtargetRy.text())
        except ValueError:
            self.addLogs('【ERROR】目标位置输入的数值无效，请重新输入')
            return

        targetPose = np.array([TargetX, TargetY, TargetZ, TargetRr, TargetRp, TargetRy])

        try:
            #self.controller.go_startPose(targetPose)
            # self.addLogs("【INFO】已移动到目标位置")
            oru_actions.run_sequence_thread(
                controller=self.controller,
                poses=[targetPose],  # 单点也放进列表
                addLogs=self.addLogs,
                desc="移动到目标位置",
                speed=0.01,  # 与 go_startPose 里一致，可按需改
                acc=0.003,
            )
        except Exception as e:
            self.addLogs(f'【ERROR】移动到工作点失败: {e}')
            return

    def startVelocityControlBtnClicked(self):
        """开始速度控制按钮事件"""
        try:
            if self.controller is None:
                self.addLogs('【WARNING】请先连接机械臂')
                return

            # 获取速度输入值（从界面输入框读取）
            try:
                Vx = float(self.lineEdit_30.text()) if self.lineEdit_30.text() else 0.0 #读取速度输入值 若空即为0
                Vy = float(self.lineEdit_31.text()) if self.lineEdit_31.text() else 0.0
                Vz = float(self.lineEdit_32.text()) if self.lineEdit_32.text() else 0.0
                Rx = float(self.lineEdit_34.text()) if self.lineEdit_34.text() else 0.0
                Ry = float(self.lineEdit_35.text()) if self.lineEdit_35.text() else 0.0
                Rz = float(self.lineEdit_37.text()) if self.lineEdit_37.text() else 0.0
            except ValueError:#将输入内容转化为浮点数时出现错误
                self.addLogs('【ERROR】速度输入值无效，请输入数字')
                return

            # 检查速度是否全为零
            target_velocity = np.array([Vx, Vy, Vz, Rx, Ry, Rz])
            if np.allclose(target_velocity, 0):#np.allclose 判断数组每个元素是否都接近于0
                self.addLogs('【WARNING】所有速度分量为0，请设置速度值')
                return

            # 创建或更新速度控制线程
            if self.velocity_control_thread is None:#速度线程没有启动
                from robot.velocity_control_thread import VelocityControlThread
                self.velocity_control_thread = VelocityControlThread(
                    controller=self.controller,
                    target_velocity=target_velocity
                )
                self.velocity_control_thread.start()#启动速度控制线程
                self.is_velocity_control_running = True#速度线程标志位置1
                self.addLogs(
                    f'【INFO】速度控制已启动: Vx={Vx:.3f}, Vy={Vy:.3f}, Vz={Vz:.3f}, Rx={Rx:.3f}, Ry={Ry:.3f}, Rz={Rz:.3f}')
            else:
                # 如果线程已存在，更新目标速度
                if self.is_velocity_control_running:
                    self.velocity_control_thread.set_target_velocity(target_velocity)
                    self.addLogs(
                        f'【INFO】速度已更新: Vx={Vx:.3f}, Vy={Vy:.3f}, Vz={Vz:.3f}, Rx={Rx:.3f}, Ry={Ry:.3f}, Rz={Rz:.3f}')
                else:
                    self.velocity_control_thread.set_target_velocity(target_velocity)
                    self.velocity_control_thread.resume()
                    self.is_velocity_control_running = True
                    self.addLogs(f'【INFO】速度控制已恢复')

        except Exception as e:
            self.addLogs(f'【ERROR】启动速度控制失败: {str(e)}')

    def stopVelocityControlBtnClicked(self):
        """停止速度控制按钮事件"""
        try:
            if self.velocity_control_thread is not None and self.is_velocity_control_running:
                # 暂停速度控制线程
                self.velocity_control_thread.pause()
                self.is_velocity_control_running = False

                # 停止机器人运动
                if self.controller is not None:
                    self.controller.rtde_c.speedStop()
                    # 或封装一个只停速度的接口，例如 stop_speed_motion()

                self.addLogs('【INFO】速度控制已停止')
            else:
                self.addLogs('【WARNING】速度控制未启动')
        except Exception as e:
            self.addLogs(f'【ERROR】停止速度控制失败: {str(e)}')

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
                "px": (self.lineEdit_adMx, (0, 0)),
                "py": (self.lineEdit_adMy, (1, 1)),
                "pz": (self.lineEdit_adMz, (2, 2)),
                "rx": (self.lineEdit_adMRr, (3, 3)),
                "ry": (self.lineEdit_adMRp, (4, 4)),
                "rz": (self.lineEdit_adMRy, (5, 5))
            },
            "B": {
                "px": (self.lineEdit_adBx, (0, 0)),
                "py": (self.lineEdit_adBy, (1, 1)),
                "pz": (self.lineEdit_adBz, (2, 2)),
                "rx": (self.lineEdit_adBRr, (3, 3)),
                "ry": (self.lineEdit_adBRp, (4, 4)),
                "rz": (self.lineEdit_adBRy, (5, 5))
            },
            "K": {
                "px": (self.lineEdit_adKx, (0, 0)),
                "py": (self.lineEdit_adKy, (1, 1)),
                "pz": (self.lineEdit_adKz, (2, 2)),
                "rx": (self.lineEdit_adKRr, (3, 3)),
                "ry": (self.lineEdit_adKRp, (4, 4)),
                "rz": (self.lineEdit_adKRy, (5, 5))
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

        targetPos = np.array(self.controller.get_ee_pose())

        self.controller.adcontrol = AdmittancePoseController(targetPos, self.controller.dt)

        # 设置力传感器控制器到机械臂控制器
        if self.force_controller is not None:
            self.controller.set_force_controller(self.force_controller)
        try:
            # if self.controlThread == None:
            #     self.controlThread = AdControlThread(self.controller, self.M, self.B, self.K)
            #     self.controlThread.start()
            #     self.addLogs("【INFO】导纳控制已启动")
            # else:
            #     self.controlThread.resume()
            #     self.addLogs("【INFO】导纳控制已启动/恢复")
            self.controlThread = AdControlThread(self.controller, self.M, self.B, self.K)
            self.controlThread.start()
            self.addLogs("【INFO】导纳控制已启动")
        except Exception as e:
            self.addLogs(f'【ERROR】控制线程启动失败: {e}')

    # def stopAdmControlButtonClicked(self):
    #     """停止导纳控制"""
    #     try:
    #         # 停止导纳控制线程
    #         if self.controlThread is not None:
    #             self.controlThread.stop()
    #             self.controlThread.wait()
    #             self.controlThread = None
    #             self.controller.rtde_c.servoStop()
    #             self.controller.rtde_c.speedStop()
    #             # self.controller.stop_robot()
    #             # self.controlThread.pause()
    #             # if self.controller is not None:
    #             #     self.controller.stop_robot()
    #             self.addLogs("【INFO】导纳控制已停止")
    #     except Exception as e:
    #         self.addLogs(f'【ERROR】停止导纳控制失败: {e}')

    def stopAdmControlButtonClicked(self):
        """停止导纳控制"""
        try:
            # 1）先停导纳线程
            if self.controlThread is not None:
                self.controlThread.stop()
                self.controlThread.wait()
                self.controlThread = None

            # 2）退出伺服/速度模式（关键）
            if self.controller is not None:
                self.controller.rtde_c.servoStop()
                self.controller.rtde_c.speedStop()

                # 可选：清空导纳对象，让状态更干净
                self.controller.adcontrol = None
                self.controller.pose_target_fixed = None

            self.addLogs("【INFO】导纳控制已停止")
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
        # 停止机器人运动
        if self.controller is not None:
            self.controller.stop_robot()
            self.addLogs("【INFO】机器人已停止")

    def startxn(self):
        if self.force_controller is None:
            self.addLogs('【WARNING】请先连接力传感器，再启动虚拟力')
            return

        vf = np.array([
            float(self.BHxnfx.text()),
            float(self.BHxnfy.text()),
            float(self.BHxnfz.text()),
            float(self.BHxntx.text()),
            float(self.BHxnty.text()),
            float(self.BHxntz.text()),
        ])
        self.force_controller.set_virtual_force(vf)
        self.force_controller.set_xvni(True)
        self.addLogs(f'【INFO】虚拟力开启: {vf}')

    def closexn(self):
        """关闭虚拟力"""
        if self.force_controller is not None:
            self.force_controller.set_xvni(False)
            self.addLogs('【INFO】虚拟力已关闭')



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
        self.lineEdit_CPoseX.setText(f"{x * 1000:.2f}")  # X位置（m转mm）
        self.lineEdit_CPoseY.setText(f"{y * 1000:.2f}")  # Y位置（m转mm）
        self.lineEdit_CPoseZ.setText(f"{z * 1000:.2f}")  # Z位置（m转mm）
        self.lineEdit_CPoseRr.setText(f"{np.rad2deg(rr):.2f}")  # Rr姿态（弧度转度）
        self.lineEdit_CPoseRp.setText(f"{np.rad2deg(rp):.2f}")  # Rp姿态（弧度转度）
        self.lineEdit_CPoseRy.setText(f"{np.rad2deg(ry):.2f}")  # Ry姿态（弧度转度）

    def ShowTargetJointCallback(self, q):
        """更新机械臂关节角显示"""
        dataline = f"{q[0]}, {q[1]}, {q[2]}, {q[3]}, {q[4]}, {q[5]}\n"
        self.jointRecord.write(dataline)
        # 更新捕获机械臂关节角显示（单位：度）
        # 更新捕获机械臂关节角显示（单位：度）
        self.lineEdit_CJoint1.setText(f"{np.rad2deg(q[0]):.2f}")  # 关节角1 弧度转为角度
        self.lineEdit_CJoint2.setText(f"{np.rad2deg(q[1]):.2f}")  # 关节角2
        self.lineEdit_CJoint3.setText(f"{np.rad2deg(q[2]):.2f}")  # 关节角3
        self.lineEdit_CJoint4.setText(f"{np.rad2deg(q[3]):.2f}")  # 关节角4
        self.lineEdit_CJoint5.setText(f"{np.rad2deg(q[4]):.2f}")  # 关节角5
        self.lineEdit_CJoint6.setText(f"{np.rad2deg(q[5]):.2f}")  # 关节角6

    # 在 DrawFTCallback 中使用显示用数据
    def DrawFTCallback(self, ft):
        """
        【作用】更新六维力传感器曲线
        """
        # 使用显示用数据（原始数据，连续显示）
        ft_display = np.array(ft).reshape(6, 1)
        # print(ft_display);

        self.maxForce = max(self.maxForce, np.linalg.norm(ft_display[0:3]))
        self.maxTorque = max(self.maxTorque, np.linalg.norm(ft_display[3:6]))

        # 更新图表数据（使用连续数据）
        self.forceXSeries.append(self.controlTime, float(ft_display[0]))
        self.forceYSeries.append(self.controlTime, float(ft_display[1]))
        self.forceZSeries.append(self.controlTime, float(ft_display[2]))
        self.torqueXSeries.append(self.controlTime, float(ft_display[3]))
        self.torqueYSeries.append(self.controlTime, float(ft_display[4]))
        self.torqueZSeries.append(self.controlTime, float(ft_display[5]))

        # 更新文本界面
        self.lineEdit_CFx.setText(str('%.2f' % ft_display[0].item()))
        self.lineEdit_CFy.setText(str('%.2f' % ft_display[1].item()))
        self.lineEdit_CFz.setText(str('%.2f' % ft_display[2].item()))
        self.lineEdit_CTx.setText(str('%.2f' % ft_display[3].item()))
        self.lineEdit_CTy.setText(str('%.2f' % ft_display[4].item()))
        self.lineEdit_CTz.setText(str('%.2f' % ft_display[5].item()))

        # 时间轴更新
        if self.controlTime > 30:
            self.TimeAxis1.setRange(self.controlTime - 30, self.controlTime)
            self.TimeAxis2.setRange(self.controlTime - 30, self.controlTime)
        else:
            self.TimeAxis1.setRange(0, self.controlTime)
            self.TimeAxis2.setRange(0, self.controlTime)

        # 保存数据（使用显示用数据）
        dataline = f"{ft_display.T}" + "\n"
        self.forceRecord.write(dataline)
        self.force_saver.add_force_data(ft_display)


    def RefreshTimeCallback(self, refreshTime):
        """
        【作用】更新界面中的绘图时间
        """
        self.refreshTime = refreshTime#保存当前时间戳
        # 检查 Robot_StartTime 是否存在，如果不存在则使用 Force_StartTime
        if hasattr(self, 'Robot_StartTime'):
            self.controlTime = refreshTime - self.Robot_StartTime
        elif hasattr(self, 'Force_StartTime'):
            self.controlTime = refreshTime - self.Force_StartTime
        else:
            # 如果两者都不存在，使用当前时间作为基准
            self.controlTime = 0.0#当前时间-程序启动时间戳 记录运行时间
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

    def importTrajectoryBtnClicked(self):
        """导入轨迹按钮事件 - 测试用"""
        try:
            # 打开文件选择对话框
            file_path, _ = QFileDialog.getOpenFileName(
                MainWindow,  # 父窗口
                "选择轨迹文件",  # 对话框标题
                "",  # 默认目录（空表示当前目录）
                "文本文件 (*.txt);;所有文件 (*.*)"  # 文件过滤器
            )

            # 检查用户是否取消了选择
            if not file_path:
                self.addLogs("【INFO】用户取消了文件选择")
                return

            # 更新UI中的文件路径显示框（lineEdit_45）
            self.lineEdit_45.setText(file_path)

            # 调用轨迹管理器加载文件
            success, message = self.trajectory_manager.load_from_file(file_path)

            # 根据结果显示日志
            if success:
                # 获取轨迹信息
                info = self.trajectory_manager.get_trajectory_info()

                # 显示成功消息
                self.addLogs(message)

                # 显示详细信息
                self.addLogs(f"【INFO】轨迹信息:")
                self.addLogs(f"  - 点数: {info['num_points']}")
                self.addLogs(
                    f"  - 起点位置: [{info['start_position'][0]:.4f}, {info['start_position'][1]:.4f}, {info['start_position'][2]:.4f}]")
                self.addLogs(
                    f"  - 终点位置: [{info['end_position'][0]:.4f}, {info['end_position'][1]:.4f}, {info['end_position'][2]:.4f}]")
                self.addLogs(f"  - 轨迹总长度: {info['total_length']:.4f} m")
                self.addLogs(f"  - 平均步长: {info['average_step_size']:.4f} m")

                # 测试：打印轨迹数据的前几行
                trajectory = self.trajectory_manager.get_trajectory()
                if trajectory is not None:
                    self.addLogs(f"【DEBUG】轨迹数据形状: {trajectory.shape}")
                    self.addLogs(f"【DEBUG】前3个点:")
                    for i in range(min(3, len(trajectory))):
                        self.addLogs(f"  点{i + 1}: {trajectory[i]}")
            else:
                # 显示错误消息
                self.addLogs(f"【ERROR】{message}")
                # 清空文件路径显示
                self.lineEdit_45.setText("")

        except Exception as e:
            self.addLogs(f"【ERROR】导入轨迹失败: {str(e)}")
            import traceback
            self.addLogs(f"【ERROR】详细错误: {traceback.format_exc()}")
            self.lineEdit_45.setText("")

    def deleteTrajectoryBtnClicked(self):
        """删除轨迹按钮事件 - 测试用"""
        try:
            if self.trajectory_manager.has_trajectory():
                # 获取轨迹名称
                trajectory_name = self.trajectory_manager.get_trajectory_name()

                # 清除轨迹数据
                self.trajectory_manager.clear_trajectory()

                # 清空UI显示
                self.lineEdit_45.setText("")

                # 显示日志
                self.addLogs(f"【INFO】已删除轨迹: {trajectory_name}")
            else:
                self.addLogs("【WARNING】当前没有加载的轨迹")

        except Exception as e:
            self.addLogs(f"【ERROR】删除轨迹失败: {str(e)}")

    def startTrajectoryBtnClicked(self):
        """开始轨迹跟踪按钮事件"""
        speed_value = 0.02  # 移动速度
        acc_value = 0.004  # 加速度
        dt_value = 0.2  # 点间隔

        try:
            # 1. 检查机械臂是否连接
            if self.controller is None:
                self.addLogs('【WARNING】请先连接机械臂')
                return

            # 2. 检查是否有轨迹数据
            if not self.trajectory_manager.has_trajectory():
                self.addLogs('【WARNING】请先导入轨迹文件')
                return

            # 3. 检查是否已有轨迹跟踪线程在运行
            if self.trajectory_control_thread is not None and self.trajectory_control_thread.isRunning():
                self.addLogs('【WARNING】轨迹跟踪已在运行中')
                return

            # 4. 获取轨迹数据
            trajectory = self.trajectory_manager.get_trajectory()
            if trajectory is None:
                self.addLogs('【ERROR】无法获取轨迹数据')
                return

            # 5. 创建轨迹跟踪线程
            # 参数说明：
            #   controller: 机械臂控制器
            #   trajectory: 轨迹数据
            #   speed: 移动速度（默认0.05 m/s）
            #   acc: 加速度（默认0.02 m/s²）
            #   dt: 每个点之间的时间间隔（默认0.1秒，可以根据需要调整）
            self.trajectory_control_thread = TrajectoryControlThread(
                controller=self.controller,  # 测试模式下即使没有连接也可以传None
                trajectory=trajectory,
                speed=speed_value,  # 可以调整速度，值越大移动越快
                acc=acc_value,  # 可以调整加速度
                dt=dt_value,  # 每个点之间的等待时间（秒），可以调整
                test_mode=False  # 设置为True，启用测试模式
            )

            # 6. 连接信号（用于接收线程的消息）
            self.trajectory_control_thread.signal_progress.connect(self.onTrajectoryProgress)
            self.trajectory_control_thread.signal_finished.connect(self.onTrajectoryFinished)
            self.trajectory_control_thread.signal_point_reached.connect(self.onTrajectoryPointReached)  # 添加这行

            # 7. 启动线程
            self.trajectory_control_thread.start()

            # 8. 显示日志
            # 8. 显示日志
            num_points = len(trajectory)
            self.addLogs(f'【INFO】开始轨迹跟踪，共 {num_points} 个点')
            self.addLogs(f'【INFO】速度: {speed_value} m/s, 加速度: {acc_value} m/s², 点间隔: {dt_value} 秒')
            # self.addLogs('【INFO】测试模式：不会实际控制机械臂，只打印信息')

        except Exception as e:
            self.addLogs(f'【ERROR】启动轨迹跟踪失败: {str(e)}')
            import traceback
            self.addLogs(f'【ERROR】详细错误: {traceback.format_exc()}')

    def stopTrajectoryBtnClicked(self):
        """停止轨迹跟踪按钮事件"""
        try:
            # 1. 检查线程是否存在且正在运行
            if self.trajectory_control_thread is not None and self.trajectory_control_thread.isRunning():
                # 2. 停止线程
                self.trajectory_control_thread.stop()
                self.trajectory_control_thread.wait()  # 等待线程结束

                # 3. 停止机械臂运动
                if self.controller is not None:
                    self.controller.stop_robot()

                self.addLogs('【INFO】轨迹跟踪已停止')
            else:
                self.addLogs('【WARNING】轨迹跟踪未在运行')

        except Exception as e:
            self.addLogs(f'【ERROR】停止轨迹跟踪失败: {str(e)}')

    def onTrajectoryProgress(self, current_point, total_points):
        """轨迹跟踪进度回调函数"""
        # 这个函数会在每移动到一个点时被调用
        # current_point: 当前已完成的点数
        # total_points: 总点数
        progress_percent = (current_point / total_points) * 100
        # 可以选择在日志中显示进度，或者更新进度条（如果有的话）
        self.addLogs(f'【INFO】轨迹跟踪进度: {current_point}/{total_points} ({progress_percent:.1f}%)')

    def onTrajectoryFinished(self, success, message):
        """轨迹跟踪完成回调函数"""
        # 这个函数会在轨迹跟踪完成或出错时被调用
        if success:
            self.addLogs(f'【INFO】{message}')
        else:
            self.addLogs(f'【ERROR】{message}')

        # 清理线程引用
        if self.trajectory_control_thread is not None:
            self.trajectory_control_thread = None

    def onTrajectoryPointReached(self, point_index, pose):
        """轨迹点到达回调函数 - 测试用"""
        # 这个函数会在每到达一个点时被调用（测试模式）
        # point_index: 点序号（从1开始）
        # pose: 位姿数组 [x, y, z, rx, ry, rz]

        # 格式化位姿显示
        pose_str = f"[{pose[0]:.4f}, {pose[1]:.4f}, {pose[2]:.4f}, {pose[3]:.4f}, {pose[4]:.4f}, {pose[5]:.4f}]"

        # 在日志中显示
        self.addLogs(f"第 {point_index} 个点已到达，位姿为: {pose_str}")

        # 同时在控制台打印（方便调试）
        print(f"第 {point_index} 个点已到达，位姿为: {pose_str}")

    def startCameraBtnClicked(self):
        """启动摄像头按钮点击事件"""
        try:

            # 2. 检查是否已经初始化过相机
            if self.camera_handler is None:
                # 创建相机处理器
                self.camera_handler = CameraHandler()
                # 初始化相机
                self.camera_handler.initialize()
                self.addLogs('【INFO】摄像头初始化成功')
            else:
                self.addLogs('【INFO】摄像头已经启动')
                return

            # 3. 设置QLabel属性，让图像自动缩放填充
            # if hasattr(self, 'label_2'):
            #     # 设置自动缩放内容，填充整个QLabel
            #     self.label_2.setScaledContents(True)
            #         # 可选：设置对齐方式
            #     from PyQt5.QtCore import Qt
            #     self.label_2.setAlignment(Qt.AlignCenter)

            # 3. 创建并启动预览线程
            if self.camera_preview_thread is None or not self.camera_preview_thread.isRunning():
                self.camera_preview_thread = CameraPreviewThread(
                    self.camera_handler,
                    enable_detection=self.detection_enabled  # 传入检测开关
                )

                # 连接信号：当线程发送图像时，更新UI显示
                self.camera_preview_thread.signal_image.connect(self.updateCameraDisplay)
                self.camera_preview_thread.signal_error.connect(lambda msg: self.addLogs(f'【ERROR】{msg}'))
                # 连接检测结果信号
                self.camera_preview_thread.signal_detection_result.connect(self.onDetectionResult)
                # 启动线程
                self.camera_preview_thread.start()
                self.addLogs('【INFO】摄像头预览已启动')
                self.label_117.setStyleSheet(
                    "background-color: green; border-radius: 10px; min-height: 20px; max-height: 20px; min-width: 20px; max-width: 20px;")
            else:
                self.addLogs('【INFO】摄像头预览已在运行')

        except Exception as e:
            self.addLogs(f'【ERROR】启动摄像头失败: {str(e)}')
            # 确保出错时清理资源
            if self.camera_handler:
                try:
                    self.camera_handler.stop()
                except:
                    pass
                self.camera_handler = None

    def closeCameraBtnClicked(self):
        """关闭摄像头按钮点击事件"""
        try:
            # 1. 停止预览线程
            if self.camera_preview_thread is not None and self.camera_preview_thread.isRunning():
                self.camera_preview_thread.stop()
                self.camera_preview_thread.wait()  # 等待线程结束
                self.camera_preview_thread = None
            # 2. 停止相机
            if self.camera_handler is not None:
                self.camera_handler.stop()
                self.camera_handler = None

                # 清空显示
                if hasattr(self, 'label_2'):
                    self.label_2.clear()

                self.addLogs('【INFO】摄像头已关闭')
                self.label_117.setStyleSheet(
                    "background-color: red; border-radius: 10px; min-height: 20px; max-height: 20px; min-width: 20px; max-width: 20px;")
            else:
                self.addLogs('【WARNING】摄像头未启动')

        except Exception as e:
            self.addLogs(f'【ERROR】关闭摄像头失败: {str(e)}')

    def updateCameraDisplay(self, cv_image):
        """更新摄像头显示 - 保持宽高比，最大化显示"""
        try:
            import cv2
            from PyQt5.QtGui import QImage, QPixmap
            from PyQt5.QtCore import Qt

            # 转换BGR到RGB
            rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            height, width, channel = rgb_image.shape
            bytes_per_line = 3 * width

            # 创建QImage
            q_image = QImage(rgb_image.data, width, height, bytes_per_line, QImage.Format_RGB888)
            pixmap = QPixmap.fromImage(q_image)

            # 获取QLabel尺寸
            label_width = self.label_2.width()
            label_height = self.label_2.height()

            if label_width > 0 and label_height > 0:
                # 计算缩放比例，保持宽高比
                image_ratio = width / height
                label_ratio = label_width / label_height

                if image_ratio > label_ratio:
                    # 图像更宽，以宽度为准
                    scaled_width = label_width
                    scaled_height = int(label_width / image_ratio)
                else:
                    # 图像更高，以高度为准
                    scaled_width = int(label_height * image_ratio)
                    scaled_height = label_height

                # 缩放并显示
                scaled_pixmap = pixmap.scaled(
                    scaled_width,
                    scaled_height,
                    Qt.KeepAspectRatio,
                    Qt.SmoothTransformation
                )
                self.label_2.setPixmap(scaled_pixmap)
                self.label_2.setAlignment(Qt.AlignCenter)
            else:
                self.label_2.setPixmap(pixmap)

        except Exception as e:
            pass

    def targetObservationBtnClicked(self):
        """目标观测按钮点击事件 - 开启/关闭 ArUco 检测"""
        try:
            # 切换检测开关
            self.detection_enabled = not self.detection_enabled

            if self.detection_enabled:
                self.addLogs('【INFO】ArUco 目标检测已开启')
            else:
                self.addLogs('【INFO】ArUco 目标检测已关闭')

            # 如果预览线程正在运行，更新检测状态
            if self.camera_preview_thread is not None and self.camera_preview_thread.isRunning():
                self.camera_preview_thread.set_detection_enabled(self.detection_enabled)

        except Exception as e:
            self.addLogs(f'【ERROR】切换目标检测失败: {str(e)}')

    def onDetectionResult(self, corners, ids, center_point, tvec):
        """
        检测结果回调

        参数:
            corners: 角点坐标
            ids: 标记ID
            center_point: 中心点坐标
            tvec: 平移向量（位姿）
        """
        try:
            if center_point is not None:
                # 可以在UI上显示检测信息
                # 例如：在日志中显示
                info = f"检测到ArUco标记 - 中心点: ({center_point[0]:.1f}, {center_point[1]:.1f})"
                if tvec is not None and len(tvec) > 0:
                    distance = np.linalg.norm(tvec[0])
                    info += f" - 距离: {distance * 1000:.1f}mm"
                # self.addLogs(info)  # 可选：在日志中显示（可能会很频繁）

                # 可以在UI的输入框中显示位姿信息（如果有的话）
                # 例如：self.lineEdit_xxx.setText(f"{tvec[0][0]:.3f}")

        except Exception as e:
            pass  # 静默处理错误

        # ORU相关函数
    def on_prepare_passive_clicked(self):
        oru_actions.prepare_passive_side_grab(self.controller, self.addLogs)

    def on_grab_passive_clicked(self):
        """UI 按钮：抓取被动端（真正抓取位姿）"""
        oru_actions.grab_passive_side(self.controller, self.addLogs)

    def on_passive_insert_clicked(self):
        """UI 按钮：被动端插入"""
        oru_actions.passive_side_insert(self.controller, self.addLogs)
    def A_yelu_clicked(self):
        oru_actions.a_yelu(self.controller,self.addLogs)

    def yelu_dianlu_clicked(self):
        oru_actions.yelu_dianlu(self.controller,self.addLogs)

    def bei_pian_clicked(self):
        oru_actions.bei_pian(self.controller,self.addLogs)

    def bei_hui_clicked(self):
        oru_actions.bei_hui(self.controller, self.addLogs)

    def luosi_clicked(self):
        oru_actions.luosi(self.controller, self.addLogs)

    def luosiA_clicked(self):
        oru_actions.luosiA(self.controller, self.addLogs)

    def open_shijiao(self):
        self.controller.rtde_c.teachMode()
        self.label_119.setStyleSheet(
            "background-color: green; border-radius: 10px; min-height: 20px; max-height: 20px; min-width: 20px; max-width: 20px;")
        self.addLogs("示教模式已开启")

    def close_shijiao(self):
        self.controller.rtde_c.endTeachMode()
        self.label_119.setStyleSheet(
            "background-color: red; border-radius: 10px; min-height: 20px; max-height: 20px; min-width: 20px; max-width: 20px;")
        self.addLogs("示教模式已关闭")


if __name__ == '__main__':#Python的标准入口点检查
    # 创建Qt界面
    app = QApplication(sys.argv) #创建Qt应用程序案例 理解：创建一个应用程序的容器
    MainWindow = QMainWindow() #创建主窗口对象 理解 ：有一个框架 放置各种控件
    ui = MyUi()#调用函数 创建ui实例

    MainWindow.show() #显示主界面
    sys.exit(app.exec_()) #启动Qt事件