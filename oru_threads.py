from PyQt5 import QtCore
import numpy as np

class OruSequenceThread(QtCore.QThread):
    log = QtCore.pyqtSignal(str)
    finished = QtCore.pyqtSignal(bool, str)

    def __init__(self, controller, poses, speed=0.01, acc=0.003, desc=""):
        super().__init__()
        self.controller = controller
        self.poses = poses
        self.speed = speed
        self.acc = acc
        self.desc = desc

    def run(self):
        try:
            if self.controller is None:
                self.finished.emit(False, "请先连接机械臂")
                return

            for i, p in enumerate(self.poses, start=1):
                self.controller.moveL(np.array(p), asy=False, speed=self.speed, acc=self.acc)
                self.log.emit(f"【INFO】{self.desc}：第 {i}/{len(self.poses)} 点完成")

            self.finished.emit(True, f"{self.desc} 完成")
        except Exception as e:
            self.finished.emit(False, f"{self.desc} 出错：{e}")