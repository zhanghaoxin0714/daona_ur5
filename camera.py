import cv2
import numpy as np


def solve(frame):
    # image = frame
    # mat = image
    # mat = cv2.Mat(image.height(), image.width(), image.constBits(), image.bytesPerLine())
    orignal_image = frame
    gray_image = cv2.cvtColor(orignal_image, cv2.COLOR_BGR2GRAY)
    corners = cv2.goodFeaturesToTrack(gray_image, 4, 0.01, 10)
    if (corners is not None):
        for i in range(len(corners)):
            cv2.circle(orignal_image, (int(corners[i][0][0]), int(
                corners[i][0][1])), 10, (10, 255, 0), -1, 8, 0)
    else:
        return [0, 0, 0, 0, 0, 0]

    cameraMatrix = np.eye(3, dtype=np.float32)
    cameraMatrix[0][0] = 1.3781e+03
    cameraMatrix[1][1] = 1.3756e+03
    cameraMatrix[2][0] = 962.5793
    cameraMatrix[2][1] = 545.7304
    cameraMatrix[2][2] = 1

    distCoeffs = np.zeros((5, 1), dtype=np.float32)
    distCoeffs[0][0] = 0.173230511639020
    distCoeffs[1][0] = -0.645138161101467
    distCoeffs[2][0] = -0.00109294300160736
    distCoeffs[3][0] = -3.47866401740176e-06
    distCoeffs[4][0] = 0

    objP = np.array([[0., 0., 0.], [12.5, 2.5, 0], [
                    2.5, 8., 0], [-4.5, 5., 0]], dtype=np.float32)

    points = np.array([corners[0], corners[1], corners[2],
                      corners[3]], dtype=np.float32)

    rvecs = np.zeros((3, 1), dtype=np.float64)
    tvecs = np.zeros((3, 1), dtype=np.float64)

    cv2.solvePnP(objP, points, cameraMatrix, distCoeffs, rvecs, tvecs)

    rotM = cv2.Rodrigues(rvecs)[0]
    theta_x = np.arctan2(rotM[2][1], rotM[2][2]) * (180 / np.pi)
    theta_y = np.arctan2(-rotM[2][0], np.sqrt(rotM[2]
                         [1] ** 2 + rotM[2][2] ** 2)) * (180 / np.pi)
    theta_z: float = np.arctan2(rotM[1][0], rotM[0][0]) * (90 / np.pi)

    P = (rotM.T) * tvecs
    p1 = P[0][0]
    p2 = P[0][1]
    p3 = P[0][2]
    arr = [p1, p2, p3, theta_x, theta_y, theta_z]
    return arr


# cap = cv2.VideoCapture(0)
# flag = cap.isOpened()
# index = 1
# while(flag):
#     ret, frame = cap.read()
#     cv2.imshow("Cap", frame)
#     k = cv2.waitKey(1)
#     solve(frame)

# cap.release()
# cv2.destroyAllWindows()
