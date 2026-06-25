import cv2

cap = cv2.VideoCapture(2)
if not cap.isOpened():
    print("无法打开摄像头")
else:
    print("相机已打开")

while True:
    ret,frame = cap.read()
    if not ret:
        print("无法读取画面")
        break
    cv2.imshow("Camera",frame)
    key = cv2.waitKey(1)
    if key == 27:
        break
cap.release()
cv2.destroyAllWindows()  # ✅ 正确！注意是 All 和 Windows（复数）


