import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import random
from joycon import Joycon
# 初始化数据
joy_L = Joycon()
titles = [
    {"rx":joy_L.gyro_theta['RX']},
    {"ry":joy_L.gyro_theta['RY']},
    {"rz":joy_L.gyro_theta['RZ']},
    {"ax":joy_L.accl_theta['X']},
    {"ay":joy_L.accl_theta['Y']},
    {"az":joy_L.accl_theta['Z']},
]

max_points = 100  # 每个子图最多显示100个点
rows=2
cols=3
x_data = np.arange(max_points)  # 初始化x轴数据
y_data = np.zeros((rows*cols, max_points))  # 初始化6个子图的y轴数据
y_data_max = np.full((rows * cols), 2) 
y_data_min = np.zeros((rows*cols))
line_width = 1
# 设置图表和子图
fig, axs = plt.subplots(rows, cols, figsize=(15, 10))  # 创建2×3的子图布局
lines = []  # 存储每个子图的线对象


for i in range(rows):
    for j in range(cols):
        if rows==1 or cols==1:
            line, = axs[i*cols + j].plot(x_data, y_data[i * cols + j], lw=line_width)  # 初始化线对象
            axs[i*cols + j].set_xlim(0, max_points)  # 设置x轴范围
            axs[i*cols + j].set_ylim(y_data_min[i * cols + j], y_data_max[i * cols + j])  # 设置y轴范围
            axs[i*cols + j].set_title(list(titles[i * cols + j].keys())[0])
        else:
            line, = axs[i, j].plot(x_data, y_data[i * cols + j], lw=line_width)  # 初始化线对象
            axs[i, j].set_xlim(0, max_points)  # 设置x轴范围
            axs[i, j].set_ylim(y_data_min[i * cols + j], y_data_max[i * cols + j])  # 设置y轴范围
            axs[i, j].set_title(list(titles[i * cols + j].keys())[0])
        lines.append(line)

# 更新函数
def update(frame):
    for i in range(rows):
        for j in range(cols):
            y_data[i*cols + j] = np.append(y_data[i*cols + j][1:], list(titles[i * cols + j].values())[0].value)  # 更新数据，移除第一个元素，添加新数据
            lines[i*cols + j].set_ydata(y_data[i*cols + j])  # 更新线的数据
            axs[i, j].set_ylim(np.min(y_data[i*cols + j]), np.max(y_data[i*cols + j]))  # 设置y轴范围
    return lines

# 创建动画
ani = FuncAnimation(fig, update, interval=5, blit=False)

# 显示图表
plt.show()