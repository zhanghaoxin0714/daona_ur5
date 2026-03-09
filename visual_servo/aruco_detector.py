# visual_servo/aruco_detector.py
import numpy as np
import cv2
import cv2.aruco as aruco


class ArUcoDetector:
    def __init__(self):
        # 兼容新旧版本的 OpenCV
        try:
            # 新版本 OpenCV (>= 4.7.0)
            self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
            self.parameters = cv2.aruco.DetectorParameters()
        except AttributeError:
            # 旧版本 OpenCV (< 4.7.0)
            self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_ARUCO_ORIGINAL)
            self.parameters = cv2.aruco.DetectorParameters_create()

    def detect_markers(self, img_color, camera_matrix, dist_coeffs):
        # 从main.py的ArUco检测代码复制过来
        corners, ids, rejected_img_points = aruco.detectMarkers(
            img_color, self.aruco_dict, parameters=self.parameters
        )

        if corners is not None and len(corners) > 0:
            rvec, tvec, markerPoints = aruco.estimatePoseSingleMarkers(
                corners, 0.094, camera_matrix, dist_coeffs
            )

            detected_points = corners[0][0]
            average_x = np.mean(detected_points[:, 0])
            average_y = np.mean(detected_points[:, 1])
            center_point = (average_x, average_y)

            return corners, ids, rvec, tvec, center_point, detected_points
        else:
            return None, None, None, None, None, None