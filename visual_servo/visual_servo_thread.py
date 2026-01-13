# visual_servo/visual_servo_thread.py
import time
import numpy as np
import cv2
from PyQt5 import QtCore
from PyQt5.QtWidgets import QApplication


class VisualServoThread(QtCore.QThread):
    """视觉伺服控制线程"""

    signal_error = QtCore.pyqtSignal(float)
    signal_image = QtCore.pyqtSignal(object)
    signal_finished = QtCore.pyqtSignal(bool, str)

    def __init__(self, controller=None, camera_handler=None,
                 aruco_detector=None, target_distance=0.45, lambda_gain=None):
        super().__init__()
        self._isPaused = False
        self._isRunning = True
        self.condition = QtCore.QWaitCondition()
        self.mutex = QtCore.QMutex()

        self.controller = controller
        self.camera_handler = camera_handler
        self.aruco_detector = aruco_detector
        self.target_distance = target_distance
        self.lambda_gain = lambda_gain

        # 导入视觉伺服函数
        from visual_servo.visual_servo_controller import servo
        from visual_servo.target_generator import generate_target_points_auto_scale
        self.servo_func = servo
        self.generate_target = generate_target_points_auto_scale

    def pause(self):
        self._isPaused = True

    def resume(self):
        self._isPaused = False
        self.condition.wakeAll()

    def stop(self):
        self._isRunning = False
        self.condition.wakeAll()

    def run(self):
        """视觉伺服主循环（从main.py的while True循环复制过来）"""
        try:
            while self._isRunning:
                # 检查暂停
                self.mutex.lock()
                if self._isPaused:
                    self.condition.wait(self.mutex)
                    self.mutex.unlock()
                    continue
                self.mutex.unlock()

                # 1. 获取图像（从main.py复制）
                color_intrin, depth_intrin, img_color, img_depth, \
                    aligned_depth_frame, intr_coeffs, intr_matrix = \
                    self.camera_handler.get_aligned_images()

                f = [color_intrin.fx, color_intrin.fy]
                resolution = [color_intrin.width, color_intrin.height]

                # 2. 检测ArUco标记（从main.py复制）
                corners, ids, rvec, tvec, center_point, detected_points = \
                    self.aruco_detector.detect_markers(img_color, intr_matrix, intr_coeffs)

                if corners is not None:
                    # 3. 生成目标点（从main.py复制）
                    target_points = self.generate_target(
                        current_corners=detected_points,
                        current_tvec=tvec[0][0],
                        target_distance=self.target_distance,
                        image_resolution=resolution
                    )

                    # 4. 调用视觉伺服控制（从main.py复制）
                    self.servo_func(
                        self.controller,  # 传入您的controller
                        detected_points,
                        img_depth,
                        target_points,
                        self.lambda_gain,
                        f,
                        resolution,
                        center_point
                    )

                    # 5. 绘制图像（用于显示）
                    img_display = img_color.copy()
                    cv2.aruco.drawDetectedMarkers(img_display, corners)
                    for point in target_points:
                        cv2.circle(img_display, tuple(point.astype(int)), 3, (255, 255, 255), -1)

                    # 发送图像信号
                    self.signal_image.emit(img_display)

                QApplication.processEvents()
                time.sleep(0.05)  # 控制频率

        except Exception as e:
            self.signal_finished.emit(False, f"视觉伺服出错: {str(e)}")