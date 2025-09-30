# -*- coding: utf-8 -*-
"""
测试六维力数据保存功能（无需实物连接）
"""
import sys
import numpy as np
import time
from datetime import datetime
from PyQt5.QtWidgets import QApplication, QMainWindow, QPushButton, QVBoxLayout, QWidget, QLabel
from PyQt5.QtCore import QTimer
from save_force_data import ForceDataSaver


class TestSaveWindow(QMainWindow):
    """测试保存功能的窗口"""
    
    def __init__(self):
        super().__init__()
        self.force_saver = ForceDataSaver()
        self.is_recording = False
        self.timer = QTimer()
        self.timer.timeout.connect(self.generate_test_data)
        
        self.init_ui()
    
    def init_ui(self):
        """初始化界面"""
        self.setWindowTitle("六维力数据保存功能测试")
        self.setGeometry(300, 300, 400, 300)
        
        # 创建中央窗口
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        # 创建布局
        layout = QVBoxLayout()
        central_widget.setLayout(layout)
        
        # 状态标签
        self.status_label = QLabel("状态: 未开始记录")
        layout.addWidget(self.status_label)
        
        # 数据信息标签
        self.data_label = QLabel("数据条数: 0")
        layout.addWidget(self.data_label)
        
        # 开始记录按钮
        self.start_button = QPushButton("开始记录")
        self.start_button.clicked.connect(self.start_recording)
        layout.addWidget(self.start_button)
        
        # 停止记录按钮
        self.stop_button = QPushButton("停止记录")
        self.stop_button.clicked.connect(self.stop_recording)
        self.stop_button.setEnabled(False)
        layout.addWidget(self.stop_button)
        
        # 保存数据按钮
        self.save_button = QPushButton("保存到Excel")
        self.save_button.clicked.connect(self.save_data)
        layout.addWidget(self.save_button)
        
        # 清除数据按钮
        self.clear_button = QPushButton("清除数据")
        self.clear_button.clicked.connect(self.clear_data)
        layout.addWidget(self.clear_button)
        
        # 显示当前力数据的标签
        self.force_label = QLabel("当前力数据: 无")
        layout.addWidget(self.force_label)
        
        # 显示当前时间的标签
        self.time_label = QLabel("当前时间: 无")
        layout.addWidget(self.time_label)
    
    def start_recording(self):
        """开始记录数据"""
        self.force_saver.start_recording()
        self.is_recording = True
        self.status_label.setText("状态: 正在记录...")
        self.start_button.setEnabled(False)
        self.stop_button.setEnabled(True)
        
        # 启动定时器，每200ms生成一次数据
        self.timer.start(200)
        print("开始记录测试数据...")
    
    def stop_recording(self):
        """停止记录数据"""
        self.timer.stop()
        self.is_recording = False
        self.status_label.setText("状态: 记录已停止")
        self.start_button.setEnabled(True)
        self.stop_button.setEnabled(False)
        print("停止记录测试数据")
    
    def generate_test_data(self):
        """生成测试数据"""
        if not self.is_recording:
            return
        
        # 生成模拟的六维力数据
        # 模拟一个周期性的力变化
        current_time = time.time()
        
        # 基础力值
        base_fx = 0.5 * np.sin(current_time * 0.5)  # 正弦变化的Fx
        base_fy = 0.3 * np.cos(current_time * 0.3)  # 余弦变化的Fy
        base_fz = 2.0 + 0.5 * np.sin(current_time * 0.2)  # 基础重力 + 变化
        base_tx = 0.1 * np.sin(current_time * 0.4)  # 小的扭矩变化
        base_ty = 0.05 * np.cos(current_time * 0.6)
        base_tz = 0.02 * np.sin(current_time * 0.8)
        
        # 添加随机噪声
        noise_level = 0.05
        ft_data = [
            base_fx + np.random.normal(0, noise_level),
            base_fy + np.random.normal(0, noise_level),
            base_fz + np.random.normal(0, noise_level),
            base_tx + np.random.normal(0, noise_level * 0.5),
            base_ty + np.random.normal(0, noise_level * 0.5),
            base_tz + np.random.normal(0, noise_level * 0.5)
        ]
        
        # 添加数据到保存器
        self.force_saver.add_force_data(ft_data)
        
        # 更新显示
        current_time_str = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
        self.data_label.setText(f"数据条数: {len(self.force_saver.force_data)}")
        self.force_label.setText(
            f"当前力数据: Fx={ft_data[0]:.3f}, Fy={ft_data[1]:.3f}, Fz={ft_data[2]:.3f}, "
            f"Tx={ft_data[3]:.3f}, Ty={ft_data[4]:.3f}, Tz={ft_data[5]:.3f}"
        )
        self.time_label.setText(f"当前时间: {current_time_str}")
    
    def save_data(self):
        """保存数据到Excel"""
        self.force_saver.save_to_excel(self)
    
    def clear_data(self):
        """清除所有数据"""
        self.force_saver.force_data = []
        self.force_saver.time_data = []
        self.force_saver.is_recording = False
        self.data_label.setText("数据条数: 0")
        self.force_label.setText("当前力数据: 无")
        self.time_label.setText("当前时间: 无")
        self.status_label.setText("状态: 数据已清除")
        print("数据已清除")


def test_save_function():
    """测试保存功能"""
    app = QApplication(sys.argv)
    
    # 创建测试窗口
    test_window = TestSaveWindow()
    test_window.show()
    
    print("测试窗口已打开！")
    print("使用说明:")
    print("1. 点击'开始记录'按钮开始生成测试数据")
    print("2. 观察实时生成的六维力数据")
    print("3. 点击'停止记录'按钮停止数据生成")
    print("4. 点击'保存到Excel'按钮保存数据")
    print("5. 点击'清除数据'按钮清除所有数据")
    
    return app.exec_()


if __name__ == "__main__":
    test_save_function()
