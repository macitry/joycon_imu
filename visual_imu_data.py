## 用于显示手柄原始IMU数据

from evdev import InputDevice, categorize, ecodes
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import time
import numpy as np


# 替换为你的 Joy-Con 设备路径，例如 /dev/input/event21
device_path = "/dev/input/event22"

# 打开设备
device = InputDevice(device_path)

print(f"Listening for events on {device.name}")
start = time.time()

# 定义一个字典来存储轴的名称
axis_names = {
    ecodes.ABS_RX: 'RX',
    ecodes.ABS_RY: 'RY',
    ecodes.ABS_RZ: 'RZ',
    ecodes.ABS_X: 'X',
    ecodes.ABS_Y: 'Y',
    ecodes.ABS_Z: 'Z'
}

# 初始化数据存储
data = {name: [] for name in axis_names.values()}
times = []

# 初始化 matplotlib 图表
fig, axes = plt.subplots(len(axis_names), 1, figsize=(10, 15), sharex=True)
lines = []

for ax, name in zip(axes, axis_names.values()):
    line, = ax.plot([], [], label=name)
    lines.append(line)
    ax.set_title(name)
    ax.set_ylabel('Value')
    ax.legend()

axes[-1].set_xlabel('Time (s)')
# TODO: 计算每个轴的基础偏置量，用于后续计算
theta_rx_zero=0
theta_delta_rx=0
count_rx=0
theta_ry_zero=0
theta_delta_ry=0
count_ry=0
theta_rz_zero=0
theta_delta_rz=0
count_rz=0
rx_bias=0.0058
ry_bias=-0.01061
rz_bias=-0.01031

def update(frame):
    global times, data, start
    global theta_rx_zero,theta_delta_rx,count_rx
    global theta_ry_zero,theta_delta_ry,count_ry
    global theta_rz_zero,theta_delta_rz,count_rz
    try:
        for event in device.read():
            if event.type == ecodes.EV_ABS:
                if event.code in axis_names:
                    axis_name = axis_names[event.code]
                    if axis_name in ['RX', 'RY', 'RZ']:
                        # 处理 RX, RY, RZ 轴的值
                        data[axis_name].append(event.value * (4000 / 65534000))
                        if len(data[axis_name]) > 1:
                            if axis_name == 'RX':
                                if count_rx==2000:
                                    print("rx_zero:",theta_rx_zero/2000)
                                    count_rx+=1
                                elif count_rx<2000:
                                    theta_delta_rx=(data[axis_name][-1] + data[axis_name][-2])*0.005/2
                                    theta_rx_zero=theta_rx_zero+theta_delta_rx-rx_bias
                                    # print(f"theta_delta_{axis_name}:{theta_delta_rx}||rx_w(dps):{data[axis_name][-1]}")
                                    print(f"theta_{axis_name}:{theta_rx_zero}")
                                    count_rx+=1
                            elif axis_name == 'RY':
                                if count_ry==2000:
                                    print("ry_zero:",theta_ry_zero/2000)
                                    count_ry+=1
                                elif count_ry<2000:
                                    theta_delta_ry=(data[axis_name][-1] + data[axis_name][-2])*0.005/2
                                    theta_ry_zero=theta_ry_zero+theta_delta_ry-ry_bias
                                    # print(f"theta_delta_{axis_name}:{theta_delta_ry}||ry_w(dps):{data[axis_name][-1]}")
                                    print(f"theta_{axis_name}:{theta_ry_zero}")
                                    count_ry+=1
                            elif axis_name == 'RZ':
                                if count_rz==2000:
                                    print("rz_zero:",theta_rz_zero/2000)
                                    count_rz+=1
                                elif count_rz<2000:
                                    theta_delta_rz=(data[axis_name][-1] + data[axis_name][-2])*0.005/2
                                    theta_rz_zero=theta_rz_zero+theta_delta_rz-rz_bias
                                    # print(f"theta_delta_{axis_name}:{theta_delta_rz}||rz_w(dps):{data[axis_name][-1]}")
                                    print(f"theta_{axis_name}:{theta_rz_zero}")
                                    count_rz+=1
                    else:
                        data[axis_name].append(event.value*  (16 / 65534))
                    times.append(time.time() - start)
    except BlockingIOError:
        pass
    
    # 确保数据长度一致
    min_length = min(len(times), *[len(data[name]) for name in axis_names.values()])
    times = times[:min_length]
    for name in axis_names.values():
        data[name] = data[name][:min_length]

    # 更新图表数据
    for line, name, ax in zip(lines, axis_names.values(), axes):
        line.set_xdata(times)
        line.set_ydata(data[name])
        line.axes.set_xlim(0, max(times, default=1))  # 动态调整 X 轴范围
        if len(data[name]) > 0:  # 确保数据不为空
            ax.set_ylim(min(data[name]), max(data[name]))  # 动态调整 Y 轴范围
        ax.relim()
        ax.autoscale_view()

    return lines

# 记录开始时间
start = time.time()

# 创建动画
ani = animation.FuncAnimation(fig, update, interval=100, blit=True)

# 显示图表
plt.show()