# -*- coding: utf-8 -*-
"""
简单的六维力测试工具
"""
import sys
import time
import random
from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QPushButton, QLabel, QWidget
from PyQt5 import QtCore

class SimpleTestWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("简单六维力测试")
        self.setGeometry(100, 100, 400, 300)
        
        # 创建中央widget
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        # 创建布局
        layout = QVBoxLayout(central_widget)
        
        # 创建按钮
        self.test_btn = QPushButton("测试按钮")
        self.test_btn.clicked.connect(self.test_clicked)
        layout.addWidget(self.test_btn)
        
        # 创建标签
        self.status_label = QLabel("状态: 未开始")
        layout.addWidget(self.status_label)
        
        # 创建数据显示标签
        self.data_label = QLabel("数据: 无")
        layout.addWidget(self.data_label)
        
        # 创建线程
        self.thread = None
        
        print("简单测试窗口初始化完成")

    def test_clicked(self):
        """测试按钮点击事件"""
        print("测试按钮被点击")
        self.status_label.setText("状态: 按钮被点击")
        
        if self.thread is None:
            self.thread = TestThread()
            self.thread.data_updated.connect(self.update_data)
            self.thread.start()
            self.status_label.setText("状态: 线程已启动")
        else:
            self.thread.stop()
            self.thread = None
            self.status_label.setText("状态: 线程已停止")

    def update_data(self, data):
        """更新数据显示"""
        print(f"接收到数据: {data}")
        self.data_label.setText(f"数据: {data}")

class TestThread(QtCore.QThread):
    """测试线程"""
    data_updated = QtCore.pyqtSignal(str)

    def __init__(self):
        super().__init__()
        self.running = True

    def stop(self):
        self.running = False
        self.quit()
        self.wait()

    def run(self):
        """线程主循环"""
        print("测试线程启动")
        count = 0
        
        while self.running:
            count += 1
            data = f"随机数据 {count}: {random.uniform(-10, 10):.2f}"
            self.data_updated.emit(data)
            time.sleep(0.5)  # 每0.5秒更新一次

if __name__ == '__main__':
    print("开始启动应用程序...")
    
    app = QApplication(sys.argv)
    print("QApplication 创建成功")
    
    window = SimpleTestWindow()
    print("窗口创建成功")
    
    window.show()
    print("窗口显示成功")
    
    print("应用程序启动完成，请查看窗口")
    
    sys.exit(app.exec_())
