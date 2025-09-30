# -*- coding: utf-8 -*-
"""
调试六维力信号连接问题
简化版本，专门用于测试信号连接和数据流
"""
import sys
import time
import numpy as np
import random
from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QHBoxLayout, QPushButton, QLabel, QTextEdit, QWidget
from PyQt5 import QtCore
from PyQt5.QtChart import *
from PyQt5.QtCore import QMargins

class DebugForceThread(QtCore.QThread):
    """调试用力传感器数据线程"""
    force_data_updated = QtCore.pyqtSignal(list)  # 使用标准信号名
    timestamp_updated = QtCore.pyqtSignal(float)

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
        """生成随机六维力数据"""
        print("调试力数据线程启动")
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
                print(f"[{count}] 生成随机力数据: {ft}")
                
                # 发送数据信号
                self.timestamp_updated.emit(time.time())
                self.force_data_updated.emit(ft)
            
            time.sleep(0.1)  # 10Hz更新频率

class DebugMainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("六维力信号调试工具")
        self.setGeometry(100, 100, 1200, 800)
        
        # 创建中央widget
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        # 创建布局
        layout = QVBoxLayout(central_widget)
        
        # 创建控制按钮
        button_layout = QHBoxLayout()
        
        self.start_btn = QPushButton("启动力数据")
        self.start_btn.clicked.connect(self.start_force_data)
        button_layout.addWidget(self.start_btn)
        
        self.stop_btn = QPushButton("停止力数据")
        self.stop_btn.clicked.connect(self.stop_force_data)
        button_layout.addWidget(self.stop_btn)
        
        self.pause_btn = QPushButton("暂停")
        self.pause_btn.clicked.connect(self.pause_force_data)
        button_layout.addWidget(self.pause_btn)
        
        self.resume_btn = QPushButton("恢复")
        self.resume_btn.clicked.connect(self.resume_force_data)
        button_layout.addWidget(self.resume_btn)
        
        layout.addLayout(button_layout)
        
        # 创建状态标签
        self.status_label = QLabel("状态: 未启动")
        layout.addWidget(self.status_label)
        
        # 创建力数据显示标签
        self.force_labels = {}
        force_names = ['Fx', 'Fy', 'Fz', 'Mx', 'My', 'Mz']
        force_layout = QHBoxLayout()
        
        for i, name in enumerate(force_names):
            label = QLabel(f"{name}: 0.00")
            label.setStyleSheet("border: 1px solid black; padding: 5px; margin: 2px;")
            self.force_labels[name] = label
            force_layout.addWidget(label)
        
        layout.addLayout(force_layout)
        
        # 创建图表
        self.setup_charts()
        layout.addWidget(self.force_chart_view)
        layout.addWidget(self.torque_chart_view)
        
        # 创建日志显示
        self.log_text = QTextEdit()
        self.log_text.setMaximumHeight(150)
        layout.addWidget(self.log_text)
        
        # 初始化变量
        self.force_thread = None
        self.control_time = 0
        self.start_time = None
        
        # 数据系列
        self.force_series = {}
        self.torque_series = {}
        
        self.add_log("调试工具初始化完成")

    def setup_charts(self):
        """设置图表"""
        # 创建力图表
        self.force_chart = QChart()
        self.force_chart.setTitle("六维力数据")
        
        # 创建力矩图表
        self.torque_chart = QChart()
        self.torque_chart.setTitle("六维力矩数据")
        
        # 创建时间轴
        self.time_axis = QValueAxis()
        self.time_axis.setTitleText("时间 (s)")
        self.time_axis.setRange(0, 30)
        
        # 创建力轴
        self.force_axis = QValueAxis()
        self.force_axis.setTitleText("力 (N)")
        self.force_axis.setRange(-100, 100)
        
        # 创建力矩轴
        self.torque_axis = QValueAxis()
        self.torque_axis.setTitleText("力矩 (Nm)")
        self.torque_axis.setRange(-10, 10)
        
        # 创建数据系列
        colors = [QtCore.Qt.red, QtCore.Qt.green, QtCore.Qt.blue]
        names = ['X', 'Y', 'Z']
        
        for i, (color, name) in enumerate(zip(colors, names)):
            # 力系列
            force_series = QLineSeries()
            force_series.setName(f"F{name}")
            force_series.setColor(color)
            self.force_series[f"F{name}"] = force_series
            self.force_chart.addSeries(force_series)
            
            # 力矩系列
            torque_series = QLineSeries()
            torque_series.setName(f"M{name}")
            torque_series.setColor(color)
            self.torque_series[f"M{name}"] = torque_series
            self.torque_chart.addSeries(torque_series)
        
        # 添加轴
        self.force_chart.addAxis(self.time_axis, QtCore.Qt.AlignBottom)
        self.force_chart.addAxis(self.force_axis, QtCore.Qt.AlignLeft)
        
        self.torque_chart.addAxis(self.time_axis, QtCore.Qt.AlignBottom)
        self.torque_chart.addAxis(self.torque_axis, QtCore.Qt.AlignLeft)
        
        # 将系列附加到轴
        for series in self.force_series.values():
            series.attachAxis(self.time_axis)
            series.attachAxis(self.force_axis)
        
        for series in self.torque_series.values():
            series.attachAxis(self.time_axis)
            series.attachAxis(self.torque_axis)
        
        # 创建图表视图
        self.force_chart_view = QChartView(self.force_chart)
        self.force_chart_view.setMinimumHeight(200)
        
        self.torque_chart_view = QChartView(self.torque_chart)
        self.torque_chart_view.setMinimumHeight(200)

    def start_force_data(self):
        """启动力数据线程"""
        if self.force_thread is None:
            self.force_thread = DebugForceThread()
            
            # 连接信号
            self.force_thread.force_data_updated.connect(self.on_force_data_received)
            self.force_thread.timestamp_updated.connect(self.on_timestamp_received)
            
            # 启动线程
            self.force_thread.start()
            
            self.status_label.setText("状态: 运行中")
            self.add_log("力数据线程已启动")
            print("力数据线程已启动")
        else:
            self.add_log("力数据线程已在运行")

    def stop_force_data(self):
        """停止力数据线程"""
        if self.force_thread is not None:
            self.force_thread.stop()
            self.force_thread = None
            self.status_label.setText("状态: 已停止")
            self.add_log("力数据线程已停止")
            print("力数据线程已停止")
        else:
            self.add_log("力数据线程未运行")

    def pause_force_data(self):
        """暂停力数据线程"""
        if self.force_thread is not None:
            self.force_thread.pause()
            self.status_label.setText("状态: 已暂停")
            self.add_log("力数据线程已暂停")

    def resume_force_data(self):
        """恢复力数据线程"""
        if self.force_thread is not None:
            self.force_thread.resume()
            self.status_label.setText("状态: 运行中")
            self.add_log("力数据线程已恢复")

    def on_force_data_received(self, ft):
        """接收力数据回调"""
        print(f"on_force_data_received 接收到数据: {ft}")
        self.add_log(f"接收到力数据: {ft}")
        
        try:
            # 更新标签显示
            force_names = ['Fx', 'Fy', 'Fz', 'Mx', 'My', 'Mz']
            for i, name in enumerate(force_names):
                if i < len(ft):
                    self.force_labels[name].setText(f"{name}: {ft[i]:.2f}")
            
            # 更新图表
            self.update_charts(ft)
            
        except Exception as e:
            print(f"处理力数据时出错: {e}")
            self.add_log(f"处理力数据时出错: {e}")

    def on_timestamp_received(self, timestamp):
        """接收时间戳回调"""
        if self.start_time is None:
            self.start_time = timestamp
        
        self.control_time = timestamp - self.start_time
        print(f"时间更新: {self.control_time:.2f}s")

    def update_charts(self, ft):
        """更新图表数据"""
        try:
            # 更新力图表
            self.force_series['FX'].append(self.control_time, ft[0])
            self.force_series['FY'].append(self.control_time, ft[1])
            self.force_series['FZ'].append(self.control_time, ft[2])
            
            # 更新力矩图表
            self.torque_series['MX'].append(self.control_time, ft[3])
            self.torque_series['MY'].append(self.control_time, ft[4])
            self.torque_series['MZ'].append(self.control_time, ft[5])
            
            # 调整时间轴范围
            if self.control_time > 30:
                self.time_axis.setRange(self.control_time - 30, self.control_time)
            
            print(f"图表更新成功，时间: {self.control_time:.2f}s")
            
        except Exception as e:
            print(f"更新图表时出错: {e}")
            self.add_log(f"更新图表时出错: {e}")

    def add_log(self, message):
        """添加日志"""
        timestamp = time.strftime("%H:%M:%S")
        log_message = f"[{timestamp}] {message}"
        self.log_text.append(log_message)
        print(log_message)

if __name__ == '__main__':
    app = QApplication(sys.argv)
    
    window = DebugMainWindow()
    window.show()
    
    print("调试工具启动完成")
    print("请点击'启动力数据'按钮开始测试")
    
    sys.exit(app.exec_())
