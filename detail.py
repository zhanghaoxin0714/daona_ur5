# -*- coding: utf-8 -*-
# detail.py
"""
详细实现模块
包含所有UI事件处理的具体实现代码
"""

import numpy as np
from PyQt5.QtCore import QMargins, Qt


class Detail:
    """详细实现类 - 包含所有具体实现方法"""

    def __init__(self, ui_instance):
        """
        初始化

        参数:
            ui_instance: UI实例对象（MyUi的实例）
        """
        self.ui = ui_instance

    def init_ui_default_values(self):
        """初始化UI默认值"""
        # ==================== UI默认值设置 - IP地址 ====================
        self.ui.lineEdit_CrobotIP.setText('192.168.111.10')  # 机器人IP
        self.ui.lineEdit_CforceIP.setText('192.168.111.20')  # 力传感器IP

        # ==================== UI默认值设置 - 目标位姿 ====================
        self.ui.lineEdit_CtargetX.setText('-0.336')  # TargetX
        self.ui.lineEdit_CtargetY.setText('-0.514')  # TargetY
        self.ui.lineEdit_CtargetZ.setText('-0.117')  # TargetZ
        self.ui.lineEdit_CtargetRr.setText('1.712')  # TargetRr
        self.ui.lineEdit_CtargetRp.setText('2.626')  # TargetRp
        self.ui.lineEdit_CtargetRy.setText('0.07')  # TargetRy

        # ==================== UI默认值设置 - 导纳参数M矩阵 ====================
        # M矩阵参数 (0.2, 0.2, 0.05, 0.008, 0.008, 0.01)
        self.ui.lineEdit_adMx.setText('0.400')
        self.ui.lineEdit_adMy.setText('0.400')
        self.ui.lineEdit_adMz.setText('0.50')
        self.ui.lineEdit_adMRr.setText('0.05')
        self.ui.lineEdit_adMRp.setText('0.05')
        self.ui.lineEdit_adMRy.setText('0.05')

        # ==================== UI默认值设置 - 导纳参数B矩阵 ====================
        # B矩阵参数 (20, 20, 20, 20, 20, 20)
        self.ui.lineEdit_adBx.setText('100.000')
        self.ui.lineEdit_adBy.setText('100.000')
        self.ui.lineEdit_adBz.setText('80.000')
        self.ui.lineEdit_adBRr.setText('100.000')
        self.ui.lineEdit_adBRp.setText('100.000')
        self.ui.lineEdit_adBRy.setText('100.000')

        # ==================== UI默认值设置 - 导纳参数K矩阵 ====================
        # K矩阵参数 (50, 50, 100, 30, 30, 30)
        self.ui.lineEdit_adKx.setText('500.000')
        self.ui.lineEdit_adKy.setText('500.000')
        self.ui.lineEdit_adKz.setText('50.000')
        self.ui.lineEdit_adKRr.setText('500.000')
        self.ui.lineEdit_adKRp.setText('500.000')
        self.ui.lineEdit_adKRy.setText('500.000')

        # ==================== UI默认值设置 - 虚拟力参数 ====================
        self.ui.BHxnfx.setText('0')
        self.ui.BHxnfy.setText('0')
        self.ui.BHxnfz.setText('-50')
        self.ui.BHxntx.setText('0')
        self.ui.BHxnty.setText('0')
        self.ui.BHxntz.setText('0')

    def init_admittance_matrices(self):
        """初始化导纳控制矩阵"""
        # ==================== 导纳控制矩阵初始化 ====================
        # 创建导纳控制算法需要的数学矩阵
        self.ui.M = np.diag([0.2, 0.2, 0.05, 0.008, 0.008, 0.01])
        self.ui.B = np.diag([20, 20, 20, 20, 20, 20])
        self.ui.K = np.diag([50, 50, 100, 30, 30, 30])

    # 在 detail.py 的 Detail 类中添加

    def init_ft_chart(self):
        """初始化力传感器图表"""
        from PyQt5.QtChart import QChart, QLineSeries, QValueAxis
        from PyQt5.QtCore import QMargins
        from PyQt5.QtGui import QFont

        self.ui.maxForce = 50
        self.ui.maxTorque = 5
        self.ui.minForce = -50
        self.ui.minTorque = -5

        # 初始化图框
        self.ui.forceChart = QChart()
        self.ui.torqueChart = QChart()
        self.ui.forceChart.setBackgroundVisible(False)
        self.ui.torqueChart.setBackgroundVisible(False)
        self.ui.forceChart.setMargins(QMargins(0, 0, 0, 0))
        self.ui.torqueChart.setMargins(QMargins(0, 0, 0, 0))
        self.ui.forceChart.layout().setContentsMargins(0, 0, 0, 0)
        self.ui.torqueChart.layout().setContentsMargins(0, 0, 0, 0)
        self.ui.forceChart.setBackgroundRoundness(0)
        self.ui.torqueChart.setBackgroundRoundness(0)

        # 初始化曲线
        self.ui.forceXSeries = QLineSeries()
        self.ui.forceYSeries = QLineSeries()
        self.ui.forceZSeries = QLineSeries()
        self.ui.torqueXSeries = QLineSeries()
        self.ui.torqueYSeries = QLineSeries()
        self.ui.torqueZSeries = QLineSeries()

        # 设置曲线名称
        self.ui.forceXSeries.setName("Fx")
        self.ui.forceYSeries.setName("Fy")
        self.ui.forceZSeries.setName("Fz")
        self.ui.torqueXSeries.setName("Tx")
        self.ui.torqueYSeries.setName("Ty")
        self.ui.torqueZSeries.setName("Tz")

        # 将曲线添加到图框中
        self.ui.forceChart.addSeries(self.ui.forceXSeries)
        self.ui.forceChart.addSeries(self.ui.forceYSeries)
        self.ui.forceChart.addSeries(self.ui.forceZSeries)
        self.ui.torqueChart.addSeries(self.ui.torqueXSeries)
        self.ui.torqueChart.addSeries(self.ui.torqueYSeries)
        self.ui.torqueChart.addSeries(self.ui.torqueZSeries)

        # self.ui.forceChart.legend().hide()
        # self.ui.torqueChart.legend().hide()

        self.ui.forceChart.legend().setVisible(True)
        self.ui.forceChart.legend().setAlignment(Qt.AlignRight)
        self.ui.torqueChart.legend().setVisible(True)
        self.ui.torqueChart.legend().setAlignment(Qt.AlignRight)
        # 设置坐标轴
        self.ui.TimeAxis1 = QValueAxis()


        # 设置坐标轴
        self.ui.TimeAxis1 = QValueAxis()
        self.ui.TimeAxis2 = QValueAxis()
        self.ui.ForceAxis = QValueAxis()
        self.ui.TorqueAxis = QValueAxis()
        self.ui.TimeAxis1.setRange(0, 30)
        self.ui.TimeAxis2.setRange(0, 30)
        self.ui.ForceAxis.setRange(-60, 60)
        self.ui.TorqueAxis.setRange(-7, 7)

        # 设置坐标轴标签格式和字体
        small_font = QFont()
        small_font.setPointSize(8)

        # 设置时间轴标签格式
        self.ui.TimeAxis1.setLabelFormat("%.1f")
        self.ui.TimeAxis1.setTickCount(7)
        self.ui.TimeAxis1.setLabelsFont(small_font)
        self.ui.TimeAxis1.setLabelsVisible(True)

        self.ui.TimeAxis2.setLabelFormat("%.1f")
        self.ui.TimeAxis2.setTickCount(7)
        self.ui.TimeAxis2.setLabelsFont(small_font)
        self.ui.TimeAxis2.setLabelsVisible(True)

        # 设置力轴标签格式
        self.ui.ForceAxis.setLabelFormat("%.2f")
        self.ui.ForceAxis.setTickCount(3)
        self.ui.ForceAxis.setLabelsFont(small_font)
        self.ui.ForceAxis.setLabelsVisible(True)

        # 设置力矩轴标签格式
        self.ui.TorqueAxis.setLabelFormat("%.2f")
        self.ui.TorqueAxis.setTickCount(3)
        self.ui.TorqueAxis.setLabelsFont(small_font)
        self.ui.TorqueAxis.setLabelsVisible(True)

        self.ui.forceChart.setAxisX(self.ui.TimeAxis1)
        self.ui.forceChart.setAxisY(self.ui.ForceAxis)
        self.ui.torqueChart.setAxisX(self.ui.TimeAxis2)
        self.ui.torqueChart.setAxisY(self.ui.TorqueAxis)

        # 关联曲线
        self.ui.forceXSeries.attachAxis(self.ui.TimeAxis1)
        self.ui.forceXSeries.attachAxis(self.ui.ForceAxis)
        self.ui.forceYSeries.attachAxis(self.ui.TimeAxis1)
        self.ui.forceYSeries.attachAxis(self.ui.ForceAxis)
        self.ui.forceZSeries.attachAxis(self.ui.TimeAxis1)
        self.ui.forceZSeries.attachAxis(self.ui.ForceAxis)
        self.ui.torqueXSeries.attachAxis(self.ui.TimeAxis2)
        self.ui.torqueXSeries.attachAxis(self.ui.TorqueAxis)
        self.ui.torqueYSeries.attachAxis(self.ui.TimeAxis2)
        self.ui.torqueYSeries.attachAxis(self.ui.TorqueAxis)
        self.ui.torqueZSeries.attachAxis(self.ui.TimeAxis2)
        self.ui.torqueZSeries.attachAxis(self.ui.TorqueAxis)

        # 设置更新动画
        self.ui.forceChart.setAnimationOptions(QChart.AnimationOption.SeriesAnimations)
        self.ui.torqueChart.setAnimationOptions(QChart.AnimationOption.SeriesAnimations)

        # 将chart显示到界面
        self.ui.plotF.setChart(self.ui.forceChart)
        self.ui.plotT.setChart(self.ui.torqueChart)