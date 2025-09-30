# -*- coding: utf-8 -*-
"""
调试测试 - 逐步检查问题
"""
print("=== 开始调试测试 ===")

try:
    print("1. 导入 sys...")
    import sys
    print("   sys 导入成功")
    
    print("2. 导入 PyQt5.QtWidgets...")
    from PyQt5.QtWidgets import QApplication
    print("   PyQt5.QtWidgets 导入成功")
    
    print("3. 导入 PyQt5.QtCore...")
    from PyQt5 import QtCore
    print("   PyQt5.QtCore 导入成功")
    
    print("4. 导入 PyQt5.QtChart...")
    from PyQt5.QtChart import *
    print("   PyQt5.QtChart 导入成功")
    
    print("5. 导入 numpy...")
    import numpy as np
    print("   numpy 导入成功")
    
    print("6. 创建 QApplication...")
    app = QApplication(sys.argv)
    print("   QApplication 创建成功")
    
    print("7. 创建窗口...")
    from PyQt5.QtWidgets import QMainWindow, QPushButton, QVBoxLayout, QWidget
    window = QMainWindow()
    window.setWindowTitle("调试测试")
    window.setGeometry(100, 100, 400, 300)
    print("   窗口创建成功")
    
    print("8. 显示窗口...")
    window.show()
    print("   窗口显示成功")
    
    print("9. 启动事件循环...")
    print("   如果看到这个窗口，说明基本功能正常")
    sys.exit(app.exec_())
    
except ImportError as e:
    print(f"导入错误: {e}")
    print("请检查是否安装了 PyQt5 和 numpy")
    input("按回车键退出...")
    
except Exception as e:
    print(f"其他错误: {e}")
    import traceback
    traceback.print_exc()
    input("按回车键退出...")

print("=== 调试测试结束 ===")

