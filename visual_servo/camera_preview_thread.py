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
    signal_detection_result = QtCore.pyqtSignal(object, object, object, object)  # corners, ids, center_point, tvec

    def __init__(self, camera_handler,enable_detection=False):
        super().__init__()
        self.camera_handler = camera_handler
        self.enable_detection = enable_detection  # 是否启用检测
        self._isRunning = True
        self._isPaused = False
        self.condition = QtCore.QWaitCondition()
        self.mutex = QtCore.QMutex()

    def set_detection_enabled(self, enabled):
        """设置是否启用检测"""
        self.enable_detection = enabled
        if enabled and not hasattr(self, 'aruco_detector'):
            from visual_servo.aruco_detector import ArUcoDetector
            self.aruco_detector = ArUcoDetector()

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

                    # 如果启用检测，进行 ArUco 检测
                    if self.enable_detection:
                        # 确保检测器已初始化（延迟初始化）
                        if not hasattr(self, 'aruco_detector'):
                            from visual_servo.aruco_detector import ArUcoDetector
                            self.aruco_detector = ArUcoDetector()

                        # 进行 ArUco 检测
                        corners, ids, rvec, tvec, center_point, detected_points = \
                            self.aruco_detector.detect_markers(
                                img_color, intr_matrix, dist_coeffs
                            )

                        # 如果检测到标记，在图像上绘制
                        if corners is not None:
                            # 绘制检测到的标记
                            cv2.aruco.drawDetectedMarkers(img_color, corners, ids)

                            # 绘制中心点
                            if center_point is not None:
                                cv2.circle(img_color,
                                           (int(center_point[0]), int(center_point[1])),
                                           5, (0, 255, 0), -1)

                            # 绘制坐标轴
                            if rvec is not None and tvec is not None:
                                cv2.drawFrameAxes(img_color, intr_matrix, dist_coeffs,
                                                  rvec[0], tvec[0], 0.05)

                            # 发送检测结果信号
                            self.signal_detection_result.emit(corners, ids, center_point, tvec)

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