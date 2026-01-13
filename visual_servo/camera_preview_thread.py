# visual_servo/camera_preview_thread.py
import time
import cv2
import numpy as np
from PyQt5 import QtCore
from PyQt5.QtWidgets import QApplication


class CameraPreviewThread(QtCore.QThread):
    """相机预览线程 - 持续获取图像并发送信号"""

    # 定义信号：发送图像数据（OpenCV格式）
    signal_image = QtCore.pyqtSignal(object)
    signal_error = QtCore.pyqtSignal(str)

    def __init__(self, camera_handler):
        super().__init__()
        self.camera_handler = camera_handler
        self._isRunning = True
        self._isPaused = False
        self.condition = QtCore.QWaitCondition()
        self.mutex = QtCore.QMutex()

    def pause(self):
        """暂停预览"""
        self._isPaused = True

    def resume(self):
        """恢复预览"""
        self._isPaused = False
        self.condition.wakeAll()

    def stop(self):
        """停止预览"""
        self._isRunning = False
        self.condition.wakeAll()

    def run(self):
        """线程主循环 - 持续获取图像"""
        try:
            while self._isRunning:
                # 检查是否暂停
                self.mutex.lock()
                if self._isPaused:
                    self.condition.wait(self.mutex)
                    self.mutex.unlock()
                    continue
                self.mutex.unlock()

                # 获取图像
                try:
                    color_intrin, depth_intrin, img_color, img_depth, \
                        aligned_depth_frame, dist_coeffs, intr_matrix = \
                        self.camera_handler.get_aligned_images()

                    # 发送图像信号
                    self.signal_image.emit(img_color)

                except Exception as e:
                    self.signal_error.emit(f"获取图像失败: {str(e)}")

                # 处理Qt事件，保持响应
                QApplication.processEvents()

                # 控制帧率（约30fps）
                time.sleep(0.033)  # 约30fps

        except Exception as e:
            self.signal_error.emit(f"预览线程错误: {str(e)}")