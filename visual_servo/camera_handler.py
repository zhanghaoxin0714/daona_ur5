# visual_servo/camera_handler.py
import numpy as np
import pyrealsense2 as rs


class CameraHandler:
    def __init__(self):
        self.pipeline = None
        self.align = None

    def initialize(self):
        # 从main.py的相机初始化代码复制过来
        pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        pipeline.start(config)

        align_to = rs.stream.color
        align = rs.align(align_to)

        self.pipeline = pipeline
        self.align = align

    def get_aligned_images(self):  # 定义图像获取函数
        frames = self.pipeline.wait_for_frames()  # 等待获取一帧数据 wait_for_frames()`：阻塞等待，直到获取到新的帧数据 pipeline？
        aligned_frames = self.align.process(frames)  # 对齐深度图和彩色图
        aligned_depth_frame = aligned_frames.get_depth_frame()  # 提取深度帧
        aligned_color_frame = aligned_frames.get_color_frame()  # 提取彩色帧
        # 获取相机内参
        depth_intrin = aligned_depth_frame.profile.as_video_stream_profile().intrinsics
        color_intrin = aligned_color_frame.profile.as_video_stream_profile().intrinsics
        # 将图像数据转换为numpy数组
        img_color = np.asanyarray(aligned_color_frame.get_data())
        img_depth = np.asanyarray(aligned_depth_frame.get_data())
        # 构建相机内参矩阵K
        intr_matrix = np.array([
            [color_intrin.fx, 0, color_intrin.ppx], [0, color_intrin.fy, color_intrin.ppy], [0, 0, 1]
        ])
        return color_intrin, depth_intrin, img_color, img_depth, aligned_depth_frame, np.array(
            color_intrin.coeffs), intr_matrix

    def stop(self):
        if self.pipeline:
            self.pipeline.stop()