import copy
import math
import time

import cv2
import halcon as ha
import numpy as np

import urx


# 计算像素坐标系下的坐标   采用halcon的模板匹配
def locate(label, camera_num):
    ho_Image = ha.read_image("/home/ras/Desktop/wrs2021/capture0.png")  # 98153
    if label == 0:
        hv_ModelFile = "/home/ras/Desktop/biaoding_20210730/shm/test.shm"
    elif label == 1:
        hv_ModelFile = "/home/ras/Desktop/supplement0307/picture_and_shm/yueya.shm"
    elif label == 2:
        hv_ModelFile = "/home/ras/Desktop/biaoding_20210730/shm/small_circle0812.shm"  # 不带轴承的小滑轮
    elif label == 3:
        hv_ModelFile = "/home/ras/Desktop/biaoding_20210730/shm/small_bearing0804.shm"  # 带轴承 的小滑轮
    elif label == 4:
        hv_ModelFile = "/home/ras/Desktop/biaoding_20210730/shm/zhou0804.shm"  # 轴
    elif label == 5:
        # hv_ModelFile = "/home/ras/Desktop/biaoding_20210730/shm/bearing_base0804.shm"#轴承座
        hv_ModelFile = "/home/ras/Desktop/biaoding_20210730/shm/bearbase.shm"  # 轴承座bearbase.shm
    elif label == 6:
        hv_ModelFile = "/home/ras/Desktop/biaoding_20210730/shm/jiaju0810.shm"  # 夹具模板
    elif label == 7:
        hv_ModelFile = "/home/ras/Desktop/biaoding_20210730/shm/zhoucheng_part_weiduiqi.shm"  # 轴承座在固定板处
    elif label == 8:
        hv_ModelFile = "/home/ras/Desktop/biaoding_20210730/shm/zhoucheng_assemblyboard.shm"  # 装配板孔位
    elif label == 9:
        hv_ModelFile = "/home/ras/Desktop/biaoding_20210730/shm/belt0821.shm"  # 装配板孔位
    elif label == 10:
        hv_ModelFile = "/home/ras/Desktop/biaoding_20210730/shm/task2_board.shm"  # 底部大板位置
    elif label == 11:
        hv_ModelFile = "/home/ras/Desktop/biaoding_20210730/shm/board2_0823.shm"  # 底部板2位置
    elif label == 12:
        hv_ModelFile = "/home/ras/Desktop/biaoding_20210730/shm/board3_0823.shm"  # 底部板2位置
    elif label == 13:
        hv_ModelFile = "/home/ras/Desktop/biaoding_20210730/shm/dianji.shm"  # 电机模板
    elif label == 14:
        hv_ModelFile = "/home/ras/Desktop/biaoding_20210730/shm/2021_9_7_20:18:50.shm"

    model_id = ha.read_shape_model(hv_ModelFile)
    # print('model id:', model_id)

    model_contours = ha.get_shape_model_contours(model_id, 1)
    # print('model_contours:', model_contours)
    pointrow, pointcol = ha.get_shape_model_origin(model_id)
    # print('row col:',pointrow, pointcol)

    NumLevels, AngleStart, AngleExtent, AngleStep, ScaleMin, ScaleMax, ScaleStep, Metric, MinContrast = ha.get_shape_model_params(
        model_id)
    # print(NumLevels, AngleStart, AngleExtent, AngleStep, ScaleMin, ScaleMax, ScaleStep, Metric, MinContrast)

    if label == 6:
        RowCheck, ColCheck, AngleCheck, ScaleCheck, Score = ha.find_scaled_shape_model(ho_Image, model_id, -3.14, 3.14,
                                                                                       0.7, 2, 0, 3, 0.5,
                                                                                       "least_squares", 5, 0.8)  # 44567
    elif label == 7:
        RowCheck, ColCheck, AngleCheck, ScaleCheck, Score = ha.find_scaled_shape_model(ho_Image, model_id, 0, 1.57,
                                                                                       0.7, 2, 0, 1, 0.5,
                                                                                       "least_squares", 5, 0.8)  # 44567
    elif label == 10:
        RowCheck, ColCheck, AngleCheck, ScaleCheck, Score = ha.find_scaled_shape_model(ho_Image, model_id, -1.57, 1.57,
                                                                                       0.7, 2, 0.3, 1, 0.5,
                                                                                       "least_squares", 5, 0.8)  # 44567
    elif label == 14:
        RowCheck, ColCheck, AngleCheck, ScaleCheck, Score = ha.find_scaled_shape_model(ho_Image, model_id, -3.14, 3.14,
                                                                                       0.7, 2, 0.3, 10, 0.5,
                                                                                       "least_squares", 5, 0.8)  # 44567
    else:
        RowCheck, ColCheck, AngleCheck, ScaleCheck, Score = ha.find_scaled_shape_model(ho_Image, model_id, -3.14, 3.14,
                                                                                       0.7, 2, 0.3, 1, 0.5,
                                                                                       "least_squares", 5, 0.8)  # 44567

    print('Score:', Score)

    window_handle = ha.open_window(0, 0, 1624, 1234, 0, "invisible", "")  # 88799
    FoundContour = ha.gen_empty_obj()
    ha.disp_image(ho_Image, window_handle)
    for i in range(len(Score)):
        MovementOfObject = ha.vector_angle_to_rigid(pointrow, pointcol, 0, RowCheck[i], ColCheck[i], AngleCheck[i])
        MoveAndScaling = ha.hom_mat2d_scale(MovementOfObject, ScaleCheck[i], ScaleCheck[i], RowCheck[i],
                                            ColCheck[i])  # 74150
        ModelAtNewPosition = ha.affine_trans_contour_xld(model_contours, MoveAndScaling)
        FoundContour = ha.concat_obj(FoundContour, ModelAtNewPosition)

    ha.set_color(window_handle, "magenta")  # 111177
    ha.set_line_width(window_handle, 5)  # 112918

    ha.disp_xld(FoundContour, window_handle)  # 29972
    t = time.localtime()

    date = str(t.tm_year) + '年' + str(t.tm_mon) + '月' + str(t.tm_mday) + '日'
    second = str(t.tm_hour) + ':' + str(t.tm_min) + ':' + str(t.tm_sec)
    img_path = '/home/ras/Desktop/pic/' + date + second
    ha.dump_window(window_handle, "png", img_path)

    ha.close_window(window_handle)
    time.sleep(0.01)

    # fill in the camera params here!
    # UR3上的摄像头
    if camera_num == 2:
        # CamParam:内参    f        k        sx            sy        cx       cy
        hv_CamParam = (0.0101741, -883.681, 8.30602e-006, 8.3e-006, 367.998, 268.536, 720, 540)
        # 外参                   x           y        z        rx        ry       rz
        # hv_PoseModelPlane = (-0.0493143, 0.0317562, 0.338131, 0.64512, 20.1627, 4.3443, 0)  
        hv_PoseModelPlane = (0, 0, 0.26, 0, 0, 0, 0)
    # UR5 上的摄像头
    if camera_num == 1:
        # CamParam:内参    f        k        sx            sy        cx       cy
        hv_CamParam = (0.0100193, -749.426, 8.29677e-006, 8.3e-006, 370.333, 260.686, 720, 540)
        # 外参                   x           y        z        rx        ry       rz
        hv_PoseModelPlane = (0.0636048, -0.0117389, 0.618031, 0.276305, 359.259, 0.693089, 0)

    if camera_num == 3:
        # CamParam:内参    f        k        sx            sy        cx       cy
        hv_CamParam = (0.0100516, -416.099, 8.29252e-006, 8.3e-006, 368.498, 270.825, 720, 540)
        # 外参                   x           y        z        rx        ry       rz
        hv_PoseModelPlane = (0.0439549, -0.0302886, 0.493261, 356.524, 358.433, 359.008, 0)

    print("RowCheck ", RowCheck, ColCheck)

    if len(RowCheck):  # match success
        hv_X, hv_Y = ha.image_points_to_world_plane(hv_CamParam, hv_PoseModelPlane, RowCheck, ColCheck, "m")  # 76070

        AngleCheck = np.array(AngleCheck)
        hv_X = np.array(hv_X)
        hv_Y = np.array(hv_Y)

        hv_angle = AngleCheck * 180 / np.pi + 6
        if camera_num == 1:
            hv_x = 1000 * hv_X
            hv_y = 1000 * hv_Y
        elif camera_num == 2:
            hv_x = 1000 * hv_X
            hv_y = 1000 * hv_Y
        elif camera_num == 3:
            hv_x = 1000 * hv_X
            hv_y = 1000 * hv_Y

        return [hv_x, hv_y, hv_angle]
    else:

        print('匹配失败')
        return None


def MatRx(rx):
    matRx = np.zeros((3, 3))
    matRx[0][0] = 1
    matRx[0][1] = matRx[0][2] = matRx[1][0] = matRx[2][0] = 0
    matRx[1][1] = np.cos(rx)
    matRx[1][2] = -np.sin(rx)
    matRx[2][1] = np.sin(rx)
    matRx[2][2] = np.cos(rx)

    return matRx


def MatRy(ry):
    matRy = np.zeros((3, 3))
    matRy[1][1] = 1
    matRy[0][1] = matRy[1][0] = matRy[2][1] = matRy[1][2] = 0
    matRy[0][0] = np.cos(ry)
    matRy[2][0] = -np.sin(ry)
    matRy[0][2] = np.sin(ry)
    matRy[2][2] = np.cos(ry)

    return matRy


def MatRz(rz):
    matRz = np.zeros((3, 3))
    matRz[2][2] = 1
    matRz[0][2] = matRz[1][2] = matRz[2][0] = matRz[2][1] = 0
    matRz[0][0] = np.cos(rz)
    matRz[0][1] = -np.sin(rz)
    matRz[1][0] = np.sin(rz)
    matRz[1][1] = np.cos(rz)

    return matRz

    global rob3


def MatPose(pose):
    # pi = np.pi
    # px = pose[3] * pi / 180  #从角度转换成弧度
    # py = pose[4] * pi / 180
    # pz = pose[5] * pi / 180

    px = pose[3]
    py = pose[4]
    pz = pose[5]
    M = np.zeros((4, 4))
    matRx = np.zeros((3, 3))
    matRy = np.zeros((3, 3))
    matRz = np.zeros((3, 3))
    matR = np.zeros((3, 3))
    matRx = MatRx(px)
    matRy = MatRy(py)
    matRz = MatRz(pz)
    matR_middle = np.dot(matRz, matRy)
    matR = np.dot(matR_middle, matRx)  # 乘的顺序是matRz*matRy*matRx
    M[:3, :3] = copy.deepcopy(matR)  # 左上角3×3矩阵赋值过去
    M[0][3] = pose[0]  # 以下三行是平移量t，分别是xyz
    M[1][3] = pose[1]
    M[2][3] = pose[2]
    M[3][0] = M[3][1] = M[3][2] = 0
    M[3][3] = 1

    return M


def checkSolution(M, rx, ry, rz):
    ox = M[0][1]
    ax = M[0][2]
    oy = M[1][1]
    ay = M[1][2]
    ox2 = np.cos(rz) * np.sin(ry) * np.sin(rx) - np.sin(rz) * np.cos(rx)
    ax2 = np.cos(rz) * np.sin(ry) * np.cos(rx) + np.sin(rz) * np.sin(rx)
    oy2 = np.sin(rz) * np.sin(ry) * np.sin(rx) + np.cos(rz) * np.cos(rx)
    ay2 = np.sin(rz) * np.sin(ry) * np.cos(rx) - np.cos(rz) * np.sin(rx)

    if abs(ox - ox2) < 0.1 and abs(ax - ax2) < 0.1 and abs(oy - oy2) < 0.1 and abs(ay - ay2) < 0.1:  # 这个阈值可以再小点？
        return 1
    else:
        return 0


def PoseMat(M):
    p = [0, 0, 0, 0, 0, 0]
    p[0] = M[0][3]
    p[1] = M[1][3]
    p[2] = M[2][3]
    oz = M[2][1]
    az = M[2][2]
    nx = M[0][0]
    ny = M[1][0]
    nz = M[2][0]
    rx1 = np.arctan2(oz, az)  # arctan求绕x轴旋转角（弧度）,
    # arctan2可以计算四个向限的，第一个参数是y，第二个参数是x，例如arctan2(1,-1)*180/pi=135
    rz1 = np.arctan2(ny, nx)  # 绕z轴的旋转角
    rx2 = np.arctan2(-oz, -az)
    rz2 = np.arctan2(-ny, -nx)
    ry1 = np.arctan2(-nz, nx * np.cos(rz1) + ny * np.sin(rz1))  # 绕y轴旋转角
    ry2 = np.arctan2(-nz, nx * np.cos(rz2) + ny * np.sin(rz2))
    if checkSolution(M, rx1, ry1, rz1):
        p[3] = rx1
        p[4] = ry1
        p[5] = rz1
    elif checkSolution(M, rx2, ry2, rz2):
        p[3] = rx2
        p[4] = ry2
        p[5] = rz2

    return p


# 旋转向量转化为rpy
def convert_to_rpy(x):
    # print('begin tcp',x)
    # a=np.dot(x[3:6],-1)
    aa = x[3:6]

    # 旋轉矢量的角度  讀取的和示教盒上的存在一個正負號反向的問題
    b1 = x[0]
    b2 = x[1]
    b3 = x[2]
    aa = np.float32(aa)
    (R, R1) = cv2.Rodrigues(aa)
    r1 = R[0][0]
    r2 = R[0][1]
    r3 = R[0][2]
    r4 = R[1][0]
    r5 = R[1][1]
    r6 = R[1][2]
    r7 = R[2][0]
    r8 = R[2][1]
    r9 = R[2][2]
    R = np.zeros((3, 3))
    R[0][0] = r1
    R[1][0] = r2
    R[2][0] = r3
    R[0][1] = r4
    R[1][1] = r5
    R[2][1] = r6
    R[0][2] = r7
    R[1][2] = r8
    R[2][2] = r9
    w = 0.5 * np.sqrt(1 + R[0][0] + R[1][1] + R[2][2])
    # if w==0:
    #     w=0.01
    x = 0.25 * (R[2][1] - R[1][2]) / w
    y = 0.25 * (R[0][2] - R[2][0]) / w
    z = 0.25 * (R[1][0] - R[0][1]) / w
    # a = 180 / np.pi*np.arctan2(2 * (y * z + x * w), w * w + z * z - y * y - x * x)
    # b = 180 / np.pi*np.arcsin(-2 * (x * z - w * y))
    # c = 180 / np.pi*np.arctan2(2 * (x * y + w * z), w * w + x * x - y * y - z * z)
    a = np.arctan2(2 * (y * z + x * w), w * w + z * z - y * y - x * x)
    b = np.arcsin(-2 * (x * z - w * y))
    c = np.arctan2(2 * (x * y + w * z), w * w + x * x - y * y - z * z)

    # print("rpy value: ",a, b, c)
    rpy = [a, b, c]

    # x[3:7]=np.float64(rpy)
    k = np.zeros(6)
    # k[0:3]=np.float32(x[0:3])
    k[0] = b1
    k[1] = b2
    k[2] = b3
    k[3:6] = rpy
    return k


# 輸入和輸出弧度

def convert_to_rpy_rob(x):
    R = x.get_orientation()
    r1 = R[0][0]
    r2 = R[0][1]
    r3 = R[0][2]
    r4 = R[1][0]
    r5 = R[1][1]
    r6 = R[1][2]
    r7 = R[2][0]
    r8 = R[2][1]
    r9 = R[2][2]
    R = np.zeros((3, 3))
    R[0][0] = r1
    R[1][0] = r2
    R[2][0] = r3
    R[0][1] = r4
    R[1][1] = r5
    R[2][1] = r6
    R[0][2] = r7
    R[1][2] = r8
    R[2][2] = r9
    w = 0.5 * np.sqrt(1 + R[0][0] + R[1][1] + R[2][2])
    x = 0.25 * (R[2][1] - R[1][2]) / w
    y = 0.25 * (R[0][2] - R[2][0]) / w
    z = 0.25 * (R[1][0] - R[0][1]) / w
    a = 180 / np.pi * np.arctan2(2 * (y * z + x * w), w * w + z * z - y * y - x * x)
    b = 180 / np.pi * np.arcsin(-2 * (x * z - w * y))
    c = 180 / np.pi * np.arctan2(2 * (x * y + w * z), w * w + x * x - y * y - z * z)
    # print("rpy value: ",a, b, c)
    rpy = [a, b, c]
    return rpy


def eulerAngles2rotationMat(theta):
    # def eulerAngles2rotationMat(theta, format='degree'):

    """
    Calculates Rotation Matrix given euler angles.
    :param theta: 1-by-3 list [rx, ry, rz] angle in degree
    :return:
    RPY角，是ZYX欧拉角，依次 绕定轴XYZ转动[rx, ry, rz]
    """
    # if format is 'degree':
    #     theta = [i * math.pi / 180.0 for i in theta]

    R_x = np.array([[1, 0, 0],
                    [0, math.cos(theta[0]), -math.sin(theta[0])],
                    [0, math.sin(theta[0]), math.cos(theta[0])]
                    ])

    R_y = np.array([[math.cos(theta[1]), 0, math.sin(theta[1])],
                    [0, 1, 0],
                    [-math.sin(theta[1]), 0, math.cos(theta[1])]
                    ])

    R_z = np.array([[math.cos(theta[2]), -math.sin(theta[2]), 0],
                    [math.sin(theta[2]), math.cos(theta[2]), 0],
                    [0, 0, 1]
                    ])
    R = np.dot(R_z, np.dot(R_y, R_x))
    # print('R',R)
    return R


# 传进来的是弧度

def rpy_to_vec(x):
    a = x[3:6]
    b1 = x[0]
    b2 = x[1]
    b3 = x[2]
    a = np.float32(a)

    b = eulerAngles2rotationMat(a)  # b为旋转矩阵
    (c, d) = cv2.Rodrigues(b)
    # c=np.dot(c,-1)

    # print('vec',c)
    k = np.zeros(6)

    k[0] = b1
    k[1] = b2
    k[2] = b3
    k[3:6] = np.dot(c.ravel(), -1)

    return k


def PoseMat_deg(M):
    p = [0, 0, 0, 0, 0, 0]
    p[0] = M[0][3]
    p[1] = M[1][3]
    p[2] = M[2][3]
    oz = M[2][1]
    az = M[2][2]
    nx = M[0][0]
    ny = M[1][0]
    nz = M[2][0]
    rx1 = np.arctan2(oz, az)  # arctan求绕x轴旋转角（弧度）,
    # arctan2可以计算四个向限的，第一个参数是y，第二个参数是x，例如arctan2(1,-1)*180/pi=135
    rz1 = np.arctan2(ny, nx)  # 绕z轴的旋转角
    rx2 = np.arctan2(-oz, -az)
    rz2 = np.arctan2(-ny, -nx)
    ry1 = np.arctan2(-nz, nx * np.cos(rz1) + ny * np.sin(rz1))  # 绕y轴旋转角
    ry2 = np.arctan2(-nz, nx * np.cos(rz2) + ny * np.sin(rz2))
    if checkSolution(M, rx1, ry1, rz1):
        p[3] = rx1 * 180 / np.pi
        p[4] = ry1 * 180 / np.pi
        p[5] = rz1 * 180 / np.pi
    elif checkSolution(M, rx2, ry2, rz2):
        p[3] = rx2 * 180 / np.pi
        p[4] = ry2 * 180 / np.pi
        p[5] = rz2 * 180 / np.pi

    return p


def MatPose_deg(pose):
    pi = np.pi
    px = pose[3] * pi / 180  # 从角度转换成弧度
    py = pose[4] * pi / 180
    pz = pose[5] * pi / 180
    M = np.zeros((4, 4))
    matRx = np.zeros((3, 3))
    matRy = np.zeros((3, 3))
    matRz = np.zeros((3, 3))
    matR = np.zeros((3, 3))
    matRx = MatRx(px)
    matRy = MatRy(py)
    matRz = MatRz(pz)
    matR_middle = np.dot(matRz, matRy)
    matR = np.dot(matR_middle, matRx)  # 乘的顺序是matRz*matRy*matRx
    M[:3, :3] = copy.deepcopy(matR)  # 左上角3×3矩阵赋值过去
    M[0][3] = pose[0]  # 以下三行是平移量t，分别是xyz
    M[1][3] = pose[1]
    M[2][3] = pose[2]
    M[3][0] = M[3][1] = M[3][2] = 0
    M[3][3] = 1

    return M


def cal_world_location(match_x, match_y, match_angle, HOST, num=0):  # num 用来表示哪个一个位置的

    if HOST == "192.168.1.60":

        # cPt = [1.10282, 78.5487, -61.727, 359.32, 359.953, 153.436]  # ur3  2021.03.07 手眼标定参数，单位mm
        cPt = [-1.9374, 78.7988, -56.069, 0.0502194, 359.868, 333.154]
        # cPt =[37.4428, -69.3819, 56.0438, 0.0146986, 0.140244, 26.8462]
        mat_cPt = MatPose_deg(cPt)
        mat_tPc = np.linalg.pinv(mat_cPt)
        tPc = PoseMat_deg(mat_tPc)

        # poselocate 模板匹配定位出来的像素xy坐标和旋转角度angle，深度信息自己调节
        cPo = [0, 0, 0, 0, 0, 0]
        cPo[0] = match_x
        cPo[1] = match_y
        cPo[2] = 200.0  # 给定一个深度值,mm
        cPo[5] = match_angle
        angle = match_angle
        rob3 = urx.Robot("192.168.1.60")
        wPt = rob3.getl()

        for i in range(3):
            wPt[i] *= 1000
        # wPt = [105.14, 234.78, 453.1, 0, 0, 0]  #前三个是拍照时的x,y,z mm,后面三个下面会赋值所以这里写0
        # wPt = [0,0,0, 0, 0, 0]

        rpy = convert_to_rpy_rob(rob3)
        wPt[3:6] = rpy[0:3]
        # wPt[4] = rpy[1]
        # wPt[5] = rpy[2]
        print(wPt)
        P_go = PoseMat_deg(MatPose_deg(wPt) @ MatPose_deg(tPc) @ MatPose_deg(cPo))
        #  修正
        # P_go[0] = P_go[0]  
        # P_go[1] = P_go[1] 
        # P_go[2] = 0
        print("P_go at :", P_go)
        print('angle = ', angle)
        delta_joint6 = (angle - 4) * np.pi / 180

    if HOST == "192.168.1.63":

        # cPt = [-2.16918,45.7236,-113.2,358.908,359.798,0.104179]   #ur5  camera到tool transfer,手眼标定参数，单位mm
        # cPt =[-2.94454, 64.013, 76.5731, 0.646181, 2.00198, 178.623]
        # cPt =[-8.03544, 63.1237, 66.6766, 3.80625, 2.92681, 180.285]
        cPt = [-1.12306, 61.3039, 65.9019, 359.76, 358.368, 179.75]
        mat_cPt = MatPose_deg(cPt)
        mat_tPc = np.linalg.pinv(mat_cPt)
        tPc = PoseMat_deg(mat_tPc)

        # poselocate 模板匹配定位出来的像素xy坐标和旋转角度angle，深度信息自己调节
        cPo = [0, 0, 0, 0, 0, 0]
        cPo[0] = match_x
        cPo[1] = match_y
        if num == 0:
            cPo[2] = 611.82  # 给定一个深度值,mm
        elif num == 1:
            cPo[2] = 500
        cPo[5] = match_angle
        angle = match_angle
        rob3 = urx.Robot("192.168.1.63")
        # wPt = rob3.getl()
        # wPt = [361.68, -153.17, 611.82, 0, 0, 0]  #前三个是拍照时的x,y,z mm,后面三个下面会赋值所以这里写0
        # if num == 0:
        #     wPt = [432.80,-263.91,600.0,0,0,0]  #托盘上方拍照位置
        # elif num == 1:
        #     wPt = [311.35,84.5,780.78,0,0,0]  #夹具上方拍照位置

        wPt = rob3.getl()

        for i in range(3):
            wPt[i] *= 1000

        rpy = convert_to_rpy_rob(rob3)
        wPt[3:6] = rpy[0:3]
        # wPt[4] = rpy[1]
        # wPt[5] = rpy[2]

        P_go = PoseMat_deg(MatPose_deg(wPt) @ MatPose_deg(tPc) @ MatPose_deg(cPo))
        #  修正    
        # print("p_go",P_go)
        if num == 1:
            P_go[0] = P_go[0]
            P_go[1] = P_go[1]
        else:
            P_go[0] = P_go[0] - 56
            P_go[1] = P_go[1] - 16
        P_go[2] = 0
        print("P_go at :", P_go)
        print('angle = ', angle)
        delta_joint6 = (angle - 4) * np.pi / 180

    return P_go[0], P_go[1], delta_joint6


def locate_board(match_x, match_y, match_angle, HOST, num=0):  # num 用来表示哪个一个位置的

    if HOST == "192.168.1.60":

        # cPt = [1.10282, 78.5487, -61.727, 359.32, 359.953, 153.436]  # ur3  2021.03.07 手眼标定参数，单位mm
        cPt = [-1.9374, 78.7988, -56.069, 0.0502194, 359.868, 333.154]
        # cPt =[37.4428, -69.3819, 56.0438, 0.0146986, 0.140244, 26.8462]
        mat_cPt = MatPose_deg(cPt)
        mat_tPc = np.linalg.pinv(mat_cPt)
        tPc = PoseMat_deg(mat_tPc)

        # poselocate 模板匹配定位出来的像素xy坐标和旋转角度angle，深度信息自己调节
        cPo = [0, 0, 0, 0, 0, 0]
        cPo[0] = match_x
        cPo[1] = match_y
        cPo[2] = 260.0  # 给定一个深度值,mm
        cPo[5] = match_angle
        angle = match_angle
        rob3 = urx.Robot("192.168.1.60")
        wPt = rob3.getl()

        for i in range(3):
            wPt[i] *= 1000
        # wPt = [105.14, 234.78, 453.1, 0, 0, 0]  #前三个是拍照时的x,y,z mm,后面三个下面会赋值所以这里写0
        # wPt = [0,0,0, 0, 0, 0]

        rpy = convert_to_rpy_rob(rob3)
        wPt[3:6] = rpy[0:3]
        # print(rpy)
        # wPt[4] = rpy[1]
        # wPt[5] = rpy[2]
        print(wPt)
        P_go = PoseMat_deg(MatPose_deg(wPt) @ MatPose_deg(tPc) @ MatPose_deg(cPo))
        #  修正
        # P_go[0] = P_go[0]  
        # P_go[1] = P_go[1] 
        # P_go[2] = 0
        print("P_go at :", P_go)
        print('angle = ', angle)
        delta_joint6 = (angle - 4) * np.pi / 180
    return P_go
