import cv2
import numpy as np

# 获取字典
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)

# 生成 ID=0 的标记，大小 200x200 像素
marker_image = cv2.aruco.generateImageMarker(aruco_dict, 0, 200)

# 保存图片
cv2.imwrite('aruco_marker_0.png', marker_image)