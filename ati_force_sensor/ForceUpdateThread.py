import time
import xlsxwriter
from datetime import datetime
from ati_force_sensor.rpi_ati_net_ft import rpi_ati_net_ft
from ati_force_sensor.Force import Force


class ForceUpdateThread:
    def __init__(self):
        self.delta_x, self.delta_y, self.delta_z = 0, 0, 0
        self.current_force = Force([0, 0, 0, 0, 0, 0]) #初始化数据

    def calcu_delta_pose(self, current_force):
        if self.current_force.fx > 1.5:
            self.delta_y -= 0.0002
        elif self.current_force.fx < -1.5:
            self.delta_y += 0.0002

        if self.current_force.fy > 1.5:
            self.delta_x += 0.0002
        elif self.current_force.fy < -1.5:
            self.delta_x -= 0.0002

        if self.current_force.fz < -5:
            self.delta_z += 0.0005
        elif self.current_force.fz > 5:
            self.delta_z -= 0.0005

    def save_to_file(self, i, worksheet, clock, current_force):
        worksheet.write(i, 0, i)
        worksheet.write(i, 1, current_force.fx)
        worksheet.write(i, 2, current_force.fy)
        worksheet.write(i, 3, current_force.fz)
        worksheet.write(i, 4, current_force.tx)
        worksheet.write(i, 5, current_force.ty)
        worksheet.write(i, 6, current_force.tz)
        worksheet.write(i, 7, clock)

    def thread_job(self, force_host):
        host = force_host #获取力传感器IP
        netft = rpi_ati_net_ft.NET_FT(host) #连接力传感器 建立与力传感器的网络连接
        netft.set_tare_from_ft() #设置力传感器零点
        # print('netft.read_ft_http()', netft.read_ft_http())
        netft.start_streaming() #开始数据流传输 从力传感器获取实时数据
        netft.set_tare_from_ft() #再次设置零点
        self.workbook = xlsxwriter.Workbook("./01.xlsx")#创建数据文件
        worksheet = self.workbook.add_worksheet()
        worksheet.write(0, 0, "time")
        worksheet.write(0, 1, "fx")
        worksheet.write(0, 2, "fy")
        worksheet.write(0, 3, "fz")
        worksheet.write(0, 4, "tx")
        worksheet.write(0, 5, "ty")
        worksheet.write(0, 6, "tz")
        worksheet.write(0, 7, "clock")

        i = 1
        last_t = time.time()#记录开始获取数据的时间
        while True:#持续获取数据
            current_force = Force(netft.try_read_ft_streaming(0.01)[1])#从力传感器中读取数据 超时时间0.01s
            # print(current_force)
            self.calcu_delta_pose(current_force)#根据力数据计算位置偏移
            # time.sleep(0.019)
            self.current_force = current_force#更新当前力数据
            dt = datetime.now()  # 创建一个datetime类对象
            clock = dt.hour * 3600 + dt.minute * 60 + dt.second#将时间转化为当天0点开始的秒数
            delt_t = time.time() - last_t#计算距离上次记录数据的时间间隔
            if delt_t > 0.02:
                self.save_to_file(i, worksheet, clock, current_force)#事件间隔检查 并保存
                last_t += delt_t#更新时间戳
                i += 1
    
    def __del__(self):
        self.workbook.close()
