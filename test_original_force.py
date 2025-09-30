# -*- coding: utf-8 -*-
"""
测试原始六维力显示功能
模拟您原始代码中的 DrawFTCallback 和相关功能
"""
import sys
import time
import numpy as np
import random
from PyQt5.QtWidgets import (QApplication, QMainWindow, QVBoxLayout, QHBoxLayout, 
                            QPushButton, QLabel, QTextEdit, QWidget, QLineEdit)
from PyQt5 import QtCore
from PyQt5.QtChart import *
from PyQt5.QtCore import QMargins

class OriginalForceThread(QtCore.QThread):
    """模拟原始代码中的 ForceDataThread"""
    _signal_ft = QtCore.pyqtSignal(list)  # 使用原始信号名
    _signal_timestamp = QtCore.pyqtSignal(float)

    def __init__(self):
        super().__init__()
        self.running = True
        self.paused = False

    def stop(self):
        self.running = False
        self.quit()
        self.wait()

    def pause(self):
        self.paused = True

    def resume(self):
        self.paused = False

    def run(self):
        """生成随机六维力数据，模拟原始代码"""
        print("原始力数据线程启动")
        count = 0
        
        while self.running:
            if not self.paused:
                # 生成随机六维力数据
                ft = [
                    random.uniform(-50, 50),   # Fx
                    random.uniform(-50, 50),   # Fy  
                    random.uniform(-50, 50),   # Fz
                    random.uniform(-5, 5),     # Mx
                    random.uniform(-5, 5),     # My
                    random.uniform(-5, 5)      # Mz
                ]
                
                count += 1
                print(f"[{count}] 生成力数据: {ft}")
                
                # 发送数据信号（使用原始信号名）
                self._signal_timestamp.emit(time.time())
                self._signal_ft.emit(ft)
            
            time.sleep(0.2)  # 5Hz更新频率，模拟原始代码

class TestOriginalWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("原始六维力功能测试")
        self.setGeometry(100, 100, 1200, 800)
        
        # 创建中央widget
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        # 创建主布局
        main_layout = QVBoxLayout(central_widget)
        
        # 创建控制面板
        self.create_control_panel(main_layout)
        
        # 创建数据显示面板（模拟原始UI）
        self.create_original_data_panel(main_layout)
        
        # 创建图表面板（模拟原始图表）
        self.create_original_chart_panel(main_layout)
        
        # 创建日志面板
        self.create_log_panel(main_layout)
        
        # 初始化变量（模拟原始代码）
        self.force_thread = None
        self.controlTime = 0
        self.maxForce = 0
        self.maxTorque = 0
        
        self.addLogs("原始六维力功能测试初始化完成")

    def create_control_panel(self, parent_layout):
        """创建控制面板"""
        control_group = QWidget()
        control_layout = QVBoxLayout(control_group)
        
        # 标题
        title_label = QLabel("原始六维力功能测试")
        title_label.setStyleSheet("font-size: 16px; font-weight: bold; margin: 10px;")
        control_layout.addWidget(title_label)
        
        # 按钮布局
        button_layout = QHBoxLayout()
        
        self.start_btn = QPushButton("启动力数据线程")
        self.start_btn.setStyleSheet("QPushButton { background-color: #4CAF50; color: white; padding: 10px; }")
        self.start_btn.clicked.connect(self.start_force_data_thread)
        button_layout.addWidget(self.start_btn)
        
        self.stop_btn = QPushButton("停止力数据线程")
        self.stop_btn.setStyleSheet("QPushButton { background-color: #f44336; color: white; padding: 10px; }")
        self.stop_btn.clicked.connect(self.stop_force_data_thread)
        button_layout.addWidget(self.stop_btn)
        
        control_layout.addLayout(button_layout)
        
        # 状态显示
        self.status_label = QLabel("状态: 未启动")
        self.status_label.setStyleSheet("font-size: 14px; padding: 10px; background-color: #f0f0f0;")
        control_layout.addWidget(self.status_label)
        
        parent_layout.addWidget(control_group)

    def create_original_data_panel(self, parent_layout):
        """创建原始数据显示面板（模拟原始UI控件）"""
        data_group = QWidget()
        data_layout = QVBoxLayout(data_group)
        
        # 标题
        data_title = QLabel("六维力数据显示（模拟原始UI）")
        data_title.setStyleSheet("font-size: 14px; font-weight: bold; margin: 5px;")
        data_layout.addWidget(data_title)
        
        # 创建模拟原始代码中的 lineEdit 控件
        self.lineEdit_109 = QLineEdit("0.00")  # Fx
        self.lineEdit_110 = QLineEdit("0.00")  # Fy
        self.lineEdit_141 = QLineEdit("0.00")  # Fz
        self.lineEdit_142 = QLineEdit("0.00")  # Mx
        self.lineEdit_143 = QLineEdit("0.00")  # My
        self.lineEdit_144 = QLineEdit("0.00")  # Mz
        
        # 设置样式
        for lineEdit in [self.lineEdit_109, self.lineEdit_110, self.lineEdit_141, 
                        self.lineEdit_142, self.lineEdit_143, self.lineEdit_144]:
            lineEdit.setStyleSheet("border: 2px solid #ccc; padding: 5px; font-size: 12px;")
            lineEdit.setReadOnly(True)
        
        # 布局
        force_layout = QHBoxLayout()
        force_layout.addWidget(QLabel("Fx:"))
        force_layout.addWidget(self.lineEdit_109)
        force_layout.addWidget(QLabel("Fy:"))
        force_layout.addWidget(self.lineEdit_110)
        force_layout.addWidget(QLabel("Fz:"))
        force_layout.addWidget(self.lineEdit_141)
        
        torque_layout = QHBoxLayout()
        torque_layout.addWidget(QLabel("Mx:"))
        torque_layout.addWidget(self.lineEdit_142)
        torque_layout.addWidget(QLabel("My:"))
        torque_layout.addWidget(self.lineEdit_143)
        torque_layout.addWidget(QLabel("Mz:"))
        torque_layout.addWidget(self.lineEdit_144)
        
        data_layout.addLayout(force_layout)
        data_layout.addLayout(torque_layout)
        
        parent_layout.addWidget(data_group)

    def create_original_chart_panel(self, parent_layout):
        """创建原始图表面板（模拟原始图表）"""
        chart_group = QWidget()
        chart_layout = QVBoxLayout(chart_group)
        
        # 标题
        chart_title = QLabel("六维力实时图表（模拟原始图表）")
        chart_title.setStyleSheet("font-size: 14px; font-weight: bold; margin: 5px;")
        chart_layout.addWidget(chart_title)
        
        # 设置图表（模拟原始代码）
        self.setup_original_charts()
        
        # 创建图表视图
        self.force_chart_view = QChartView(self.forceChart)
        self.force_chart_view.setMinimumHeight(250)
        self.force_chart_view.setStyleSheet("border: 1px solid #ccc;")
        
        self.torque_chart_view = QChartView(self.torqueChart)
        self.torque_chart_view.setMinimumHeight(250)
        self.torque_chart_view.setStyleSheet("border: 1px solid #ccc;")
        
        chart_layout.addWidget(self.force_chart_view)
        chart_layout.addWidget(self.torque_chart_view)
        
        parent_layout.addWidget(chart_group)

    def create_log_panel(self, parent_layout):
        """创建日志面板"""
        log_group = QWidget()
        log_layout = QVBoxLayout(log_group)
        
        # 标题
        log_title = QLabel("运行日志")
        log_title.setStyleSheet("font-size: 14px; font-weight: bold; margin: 5px;")
        log_layout.addWidget(log_title)
        
        # 日志显示
        self.plainTextEdit = QTextEdit()
        self.plainTextEdit.setMaximumHeight(150)
        self.plainTextEdit.setStyleSheet("border: 1px solid #ccc; background-color: #f9f9f9;")
        log_layout.addWidget(self.plainTextEdit)
        
        parent_layout.addWidget(log_group)

    def setup_original_charts(self):
        """设置原始图表（完全模拟原始代码）"""
        # 创建力图表
        self.forceChart = QChart()
        self.forceChart.setTitle("六维力传感器数据")
        self.forceChart.setMargins(QMargins(0, 0, 0, 0))
        
        # 创建力矩图表
        self.torqueChart = QChart()
        self.torqueChart.setTitle("六维力矩传感器数据")
        self.torqueChart.setMargins(QMargins(0, 0, 0, 0))
        
        # 创建时间轴
        self.TimeAxis1 = QValueAxis()
        self.TimeAxis1.setTitleText("时间 (s)")
        self.TimeAxis1.setRange(0, 30)
        
        self.TimeAxis2 = QValueAxis()
        self.TimeAxis2.setTitleText("时间 (s)")
        self.TimeAxis2.setRange(0, 30)
        
        # 创建力轴
        self.ForceAxis = QValueAxis()
        self.ForceAxis.setTitleText("力 (N)")
        self.ForceAxis.setRange(-100, 100)
        
        # 创建力矩轴
        self.TorqueAxis = QValueAxis()
        self.TorqueAxis.setTitleText("力矩 (Nm)")
        self.TorqueAxis.setRange(-10, 10)
        
        # 创建数据系列（完全模拟原始代码）
        self.forceXSeries = QLineSeries()
        self.forceXSeries.setName("Fx")
        self.forceXSeries.setColor(QtCore.Qt.red)
        
        self.forceYSeries = QLineSeries()
        self.forceYSeries.setName("Fy")
        self.forceYSeries.setColor(QtCore.Qt.green)
        
        self.forceZSeries = QLineSeries()
        self.forceZSeries.setName("Fz")
        self.forceZSeries.setColor(QtCore.Qt.blue)
        
        self.torqueXSeries = QLineSeries()
        self.torqueXSeries.setName("Mx")
        self.torqueXSeries.setColor(QtCore.Qt.red)
        
        self.torqueYSeries = QLineSeries()
        self.torqueYSeries.setName("My")
        self.torqueYSeries.setColor(QtCore.Qt.green)
        
        self.torqueZSeries = QLineSeries()
        self.torqueZSeries.setName("Mz")
        self.torqueZSeries.setColor(QtCore.Qt.blue)
        
        # 添加系列到图表
        self.forceChart.addSeries(self.forceXSeries)
        self.forceChart.addSeries(self.forceYSeries)
        self.forceChart.addSeries(self.forceZSeries)
        
        self.torqueChart.addSeries(self.torqueXSeries)
        self.torqueChart.addSeries(self.torqueYSeries)
        self.torqueChart.addSeries(self.torqueZSeries)
        
        # 添加轴到图表
        self.forceChart.addAxis(self.TimeAxis1, QtCore.Qt.AlignBottom)
        self.forceChart.addAxis(self.ForceAxis, QtCore.Qt.AlignLeft)
        
        self.torqueChart.addAxis(self.TimeAxis2, QtCore.Qt.AlignBottom)
        self.torqueChart.addAxis(self.TorqueAxis, QtCore.Qt.AlignLeft)
        
        # 将系列附加到轴
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

    def start_force_data_thread(self):
        """启动力数据线程（模拟原始代码）"""
        if self.force_thread is None:
            self.force_thread = OriginalForceThread()
            
            # 连接信号（使用原始信号名）
            self.force_thread._signal_ft.connect(self.DrawFTCallback)
            self.force_thread._signal_timestamp.connect(self.RefreshTimeCallback)
            
            # 启动线程
            self.force_thread.start()
            
            self.status_label.setText("状态: 力数据线程运行中")
            self.status_label.setStyleSheet("font-size: 14px; padding: 10px; background-color: #e8f5e8; color: #2e7d32;")
            self.addLogs("【INFO】力传感器数据刷新线程已启动")
            print("力数据线程已启动")
        else:
            self.addLogs("【WARNING】力数据线程已在运行")

    def stop_force_data_thread(self):
        """停止力数据线程"""
        if self.force_thread is not None:
            self.force_thread.stop()
            self.force_thread = None
            self.status_label.setText("状态: 力数据线程已停止")
            self.status_label.setStyleSheet("font-size: 14px; padding: 10px; background-color: #ffebee; color: #c62828;")
            self.addLogs("【INFO】力传感器数据刷新线程已停止")
            print("力数据线程已停止")
        else:
            self.addLogs("【WARNING】力数据线程未运行")

    def DrawFTCallback(self, ft):
        """完全模拟原始代码中的 DrawFTCallback 方法"""
        print(f"DrawFTCallback 接收到数据: {ft}")
        self.addLogs(f"DrawFTCallback 接收到数据: {ft}")
        
        try:
            # 完全模拟原始代码的数据处理
            ft = np.array(ft).reshape(6, 1)  # 力数据转换为数组格式
            self.maxForce = max(self.maxForce, np.linalg.norm(ft[0:3]))  # 取最大值
            self.maxTorque = max(self.maxTorque, np.linalg.norm(ft[3:6]))
            
            # 更新图表数据（完全模拟原始代码）
            self.forceXSeries.append(self.controlTime, ft[0].item())
            self.forceYSeries.append(self.controlTime, ft[1].item())
            self.forceZSeries.append(self.controlTime, ft[2].item())
            self.torqueXSeries.append(self.controlTime, ft[3].item())
            self.torqueYSeries.append(self.controlTime, ft[4].item())
            self.torqueZSeries.append(self.controlTime, ft[5].item())
            
            # 更新文本界面（完全模拟原始代码）
            self.lineEdit_109.setText(str('%.2f' % ft[0].item()))
            self.lineEdit_110.setText(str('%.2f' % ft[1].item()))
            self.lineEdit_141.setText(str('%.2f' % ft[2].item()))
            self.lineEdit_142.setText(str('%.2f' % ft[3].item()))
            self.lineEdit_143.setText(str('%.2f' % ft[4].item()))
            self.lineEdit_144.setText(str('%.2f' % ft[5].item()))
            
            # 调整时间轴范围（完全模拟原始代码）
            if self.controlTime > 30:
                self.TimeAxis1.setRange(self.controlTime - 30, self.controlTime)
                self.TimeAxis2.setRange(self.controlTime - 30, self.controlTime)
            else:
                self.TimeAxis1.setRange(0, self.controlTime + 1)
                self.TimeAxis2.setRange(0, self.controlTime + 1)
            
            print(f"DrawFTCallback 处理完成，时间: {self.controlTime:.2f}s")
            self.addLogs(f"图表和文本更新成功，时间: {self.controlTime:.2f}s")
            
        except Exception as e:
            print(f"DrawFTCallback 错误: {e}")
            self.addLogs(f"【ERROR】DrawFTCallback 错误: {e}")
            import traceback
            traceback.print_exc()

    def RefreshTimeCallback(self, timestamp):
        """完全模拟原始代码中的 RefreshTimeCallback 方法"""
        if not hasattr(self, 'start_time'):
            self.start_time = timestamp
        
        self.controlTime = timestamp - self.start_time
        print(f"时间更新: {self.controlTime:.2f}s")

    def addLogs(self, *args, split=''):
        """完全模拟原始代码中的 addLogs 方法"""
        newLog = split.join(args)
        if not hasattr(self, 'logs'):
            self.logs = []
        self.logs.append(newLog)
        self.plainTextEdit.setPlainText("\n".join(self.logs))
        # 自动滚动到最新内容
        self.plainTextEdit.verticalScrollBar().setValue(
            self.plainTextEdit.verticalScrollBar().maximum()
        )

if __name__ == '__main__':
    print("开始启动原始六维力功能测试...")
    
    app = QApplication(sys.argv)
    print("QApplication 创建成功")
    
    window = TestOriginalWindow()
    print("测试窗口创建成功")
    
    window.show()
    print("测试窗口显示成功")
    
    print("=" * 60)
    print("原始六维力功能测试工具启动完成")
    print("=" * 60)
    print("测试步骤：")
    print("1. 点击 '启动力数据线程' 按钮")
    print("2. 观察上方的数值框是否在更新")
    print("3. 观察下方的图表是否有曲线绘制")
    print("4. 查看日志面板了解数据流情况")
    print("5. 如果这个测试正常，说明您的原始代码逻辑没问题")
    print("6. 如果这个测试有问题，说明是代码逻辑问题")
    print("=" * 60)
    
    sys.exit(app.exec_())
