# -*- coding: utf-8 -*-
"""
六维力数据保存功能模块
"""
import pandas as pd
import numpy as np
from datetime import datetime
from PyQt5.QtWidgets import QFileDialog, QMessageBox
import os


class ForceDataSaver:
    """六维力数据保存器"""
    
    def __init__(self):
        self.force_data = []  # 存储力传感器数据
        self.time_data = []   # 存储时间数据
        self.is_recording = False  # 是否正在记录
        self.start_time = None  # 记录开始时间
    
    def start_recording(self):
        """开始记录数据"""
        self.force_data = []
        self.time_data = []
        self.is_recording = True
        self.start_time = datetime.now()
        print("开始记录六维力数据...")
    
    def add_force_data(self, ft_data):
        """
        添加力传感器数据
        
        Args:
            ft_data: 六维力数据 [Fx, Fy, Fz, Tx, Ty, Tz]
        """
        if self.is_recording:
            current_time = datetime.now()
            
            # 使用具体的时间格式：年月日时分秒
            time_str = current_time.strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]  # 精确到毫秒
            
            self.time_data.append(time_str)
            self.force_data.append(ft_data)
    
    def save_to_excel(self, parent_widget=None):
        """
        保存数据到Excel文件
        
        Args:
            parent_widget: 父窗口组件，用于显示文件对话框
        """
        if not self.force_data:
            QMessageBox.warning(parent_widget, "警告", "没有可保存的数据！")
            return
        
        try:
            # 创建文件对话框
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            default_filename = f"六维力数据_{timestamp}.xlsx"
            
            file_path, _ = QFileDialog.getSaveFileName(
                parent_widget,
                "保存六维力数据",
                default_filename,
                "Excel文件 (*.xlsx);;所有文件 (*)"
            )
            
            if file_path:
                # 创建DataFrame
                df = pd.DataFrame({
                    '时间': self.time_data,
                    'Fx(N)': [data[0] for data in self.force_data],
                    'Fy(N)': [data[1] for data in self.force_data],
                    'Fz(N)': [data[2] for data in self.force_data],
                    'Tx(Nm)': [data[3] for data in self.force_data],
                    'Ty(Nm)': [data[4] for data in self.force_data],
                    'Tz(Nm)': [data[5] for data in self.force_data]
                })
                
                # 保存到Excel
                with pd.ExcelWriter(file_path, engine='openpyxl') as writer:
                    df.to_excel(writer, sheet_name='六维力数据', index=False)
                    
                    # 获取工作表对象
                    worksheet = writer.sheets['六维力数据']
                    
                    # 设置列宽
                    column_widths = [12, 10, 10, 10, 10, 10, 10]
                    for i, width in enumerate(column_widths, 1):
                        worksheet.column_dimensions[chr(64 + i)].width = width
                
                QMessageBox.information(
                    parent_widget, 
                    "成功", 
                    f"数据已保存到:\n{file_path}\n\n共保存 {len(self.force_data)} 条数据"
                )
                
                print(f"六维力数据已保存到: {file_path}")
                
        except Exception as e:
            QMessageBox.critical(parent_widget, "错误", f"保存失败: {str(e)}")
            print(f"保存失败: {e}")
    
    def get_data_summary(self):
        """获取数据摘要"""
        if not self.force_data:
            return "无数据"
        
        force_array = np.array(self.force_data)
        
        # 计算记录时长（从第一个时间到最后一个时间）
        if len(self.time_data) >= 2:
            start_time = datetime.strptime(self.time_data[0], "%Y-%m-%d %H:%M:%S.%f")
            end_time = datetime.strptime(self.time_data[-1], "%Y-%m-%d %H:%M:%S.%f")
            duration = (end_time - start_time).total_seconds()
            duration_str = f"{duration:.2f}秒"
        else:
            duration_str = "0.00秒"
        
        summary = {
            "数据条数": len(self.force_data),
            "记录时长": duration_str,
            "开始时间": self.time_data[0] if self.time_data else "无",
            "结束时间": self.time_data[-1] if self.time_data else "无",
            "Fx范围": f"{force_array[:, 0].min():.3f} ~ {force_array[:, 0].max():.3f} N",
            "Fy范围": f"{force_array[:, 1].min():.3f} ~ {force_array[:, 1].max():.3f} N",
            "Fz范围": f"{force_array[:, 2].min():.3f} ~ {force_array[:, 2].max():.3f} N",
            "Tx范围": f"{force_array[:, 3].min():.3f} ~ {force_array[:, 3].max():.3f} Nm",
            "Ty范围": f"{force_array[:, 4].min():.3f} ~ {force_array[:, 4].max():.3f} Nm",
            "Tz范围": f"{force_array[:, 5].min():.3f} ~ {force_array[:, 5].max():.3f} Nm"
        }
        return summary


# 使用示例函数
def create_save_force_button_handler(force_saver, parent_widget):
    """
    创建保存按钮的事件处理函数
    
    Args:
        force_saver: ForceDataSaver实例
        parent_widget: 父窗口组件
    
    Returns:
        事件处理函数
    """
    def save_button_clicked():
        """保存按钮点击事件"""
        force_saver.save_to_excel(parent_widget)
    
    return save_button_clicked


def create_start_recording_handler(force_saver):
    """
    创建开始记录的事件处理函数
    
    Args:
        force_saver: ForceDataSaver实例
    
    Returns:
        事件处理函数
    """
    def start_recording():
        """开始记录数据"""
        force_saver.start_recording()
    
    return start_recording


def create_add_data_handler(force_saver):
    """
    创建添加数据的事件处理函数
    
    Args:
        force_saver: ForceDataSaver实例
    
    Returns:
        事件处理函数
    """
    def add_force_data(ft_data):
        """添加力传感器数据"""
        force_saver.add_force_data(ft_data)
    
    return add_force_data


# 测试函数
def test_force_saver():
    """测试力数据保存功能"""
    import numpy as np
    
    # 创建保存器实例
    saver = ForceDataSaver()
    
    # 开始记录
    saver.start_recording()
    
    # 模拟一些数据
    for i in range(10):
        # 模拟六维力数据
        ft_data = [
            np.random.normal(0, 0.5),  # Fx
            np.random.normal(0, 0.5),  # Fy
            np.random.normal(2, 0.3),  # Fz
            np.random.normal(0, 0.1),  # Tx
            np.random.normal(0, 0.1),  # Ty
            np.random.normal(0, 0.1)   # Tz
        ]
        saver.add_force_data(ft_data)
    
    # 显示数据摘要
    summary = saver.get_data_summary()
    print("数据摘要:")
    for key, value in summary.items():
        print(f"  {key}: {value}")
    
    print("\n测试完成！")


if __name__ == "__main__":
    test_force_saver()
