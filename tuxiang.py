import re
import os
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

excel_file = r'F:\ayanyi\buhuo - 1023\CE1.xlsx'
df = pd.read_excel(excel_file)

# 解析文本为 float（取最后一个数字，兼容 "[0. 0.0123]"）
float_pattern = re.compile(r'[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?')
def to_float(cell):
    if isinstance(cell, (float, int, np.floating, np.integer)):
        return float(cell)
    if pd.isna(cell):
        return np.nan
    nums = float_pattern.findall(str(cell))
    return float(nums[-1]) if nums else np.nan

# 列按序：时间、Fx、Fy、Fz、Tx、Ty、Tz
t = pd.to_datetime(df.iloc[:, 0], errors='coerce')
Fx = df.iloc[:, 1].apply(to_float)
Fy = df.iloc[:, 2].apply(to_float)
Fz = df.iloc[:, 3].apply(to_float)
Tx = df.iloc[:, 4].apply(to_float)
Ty = df.iloc[:, 5].apply(to_float)
Tz = df.iloc[:, 6].apply(to_float)

# 丢掉任一列为 NaN 的行，保持 x/y 对齐
mask_force = ~(t.isna() | Fx.isna() | Fy.isna() | Fz.isna())
mask_torque = ~(t.isna() | Tx.isna() | Ty.isna() | Tz.isna())

# 转 numpy，避免 Series 在 matplotlib 内部被 x[:, None] 触发错误
t_f = t[mask_force].to_numpy()
Fx = Fx[mask_force].to_numpy()
Fy = Fy[mask_force].to_numpy()
Fz = Fz[mask_force].to_numpy()

t_t = t[mask_torque].to_numpy()
Tx = Tx[mask_torque].to_numpy()
Ty = Ty[mask_torque].to_numpy()
Tz = Tz[mask_torque].to_numpy()

plt.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei']
plt.rcParams['axes.unicode_minus'] = False

# 力折线图
plt.figure(figsize=(12, 5))
plt.plot(t_f, Fx, label='Fx (N)', linewidth=1.2)
plt.plot(t_f, Fy, label='Fy (N)', linewidth=1.2)
plt.plot(t_f, Fz, label='Fz (N)', linewidth=1.2)
plt.xlabel('时间'); plt.ylabel('力 (N)'); plt.title('六维力 - 力折线图')
plt.legend(); plt.grid(True, alpha=0.3); plt.tight_layout()
forces_path = os.path.join(os.path.dirname(excel_file), 'forces.png')
plt.savefig(forces_path, dpi=300)
print('已保存:', forces_path)

# 力矩折线图
plt.figure(figsize=(12, 5))
plt.plot(t_t, Tx, label='Tx (Nm)', linewidth=1.2)
plt.plot(t_t, Ty, label='Ty (Nm)', linewidth=1.2)
plt.plot(t_t, Tz, label='Tz (Nm)', linewidth=1.2)
plt.xlabel('时间'); plt.ylabel('力矩 (Nm)'); plt.title('六维力 - 力矩折线图')
plt.legend(); plt.grid(True, alpha=0.3); plt.tight_layout()
torques_path = os.path.join(os.path.dirname(excel_file), 'torques.png')
plt.savefig(torques_path, dpi=300)
print('已保存:', torques_path)