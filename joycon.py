from evdev import InputDevice, categorize, ecodes
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import time
import numpy as np
import os
import threading
from multiprocessing import Value, Array
import math
class Joycon:
    def __init__(self, device_path="/dev/input/event22"):
        device_path_IMU = device_path
        self.device = InputDevice(device_path_IMU)
        os.set_blocking(self.device.fd, True)
        # IMU 轴对应名称
        self.axis_names = {
            ecodes.ABS_RX: 'RX',
            ecodes.ABS_RY: 'RY',
            ecodes.ABS_RZ: 'RZ',
            ecodes.ABS_X: 'X',
            ecodes.ABS_Y: 'Y',
            ecodes.ABS_Z: 'Z'
        }
        # IMU 原始数据
        self.IMU_RAW = {name: [] for name in self.axis_names.values()}
        # 陀螺仪轴对应名称
        self.gyro_names = {
            ecodes.ABS_RX: 'RX',
            ecodes.ABS_RY: 'RY',
            ecodes.ABS_RZ: 'RZ',
        }
        # 角度微分偏置
        self.gyro_bias = {name: 0 for name in self.gyro_names.values()}
        self.gyro_bias['RX'] = 0.0058
        self.gyro_bias['RY'] = -0.01061
        self.gyro_bias['RZ'] = -0.01031
        # 角度微分
        self.theta_delta = {name: 0 for name in self.gyro_names.values()}
        # 角度
        self.gyro_theta = {name:Value('d', 0.0) for name in self.gyro_names.values()}

        #  加速度计对应名称
        self.accl_names = {
            ecodes.ABS_RX: 'X',
            ecodes.ABS_RY: 'Y',
            ecodes.ABS_RZ: 'Z',
        }
        # 解算角度
        self.accl_theta = {name:Value('d', 0.0) for name in self.accl_names.values()}

    def update_IMU(self):
        # 读取IMU数据
        print("received data")
        while True:
            try:
                count=0
                for event in self.device.read():
                    count=count+1
                    if event.type == ecodes.EV_MSC:
                        time.sleep(0.005)
                    elif event.type == ecodes.EV_ABS:
                        if event.code in self.axis_names:
                            axis_name = self.axis_names[event.code]
                            if axis_name in ['RX', 'RY', 'RZ']:
                                # 处理 RX, RY, RZ 轴的值  gryo数据 角速度计
                                self.IMU_RAW[axis_name].append(event.value * (4000 / 65534000))
                                if len(self.IMU_RAW[axis_name]) > 1:
                                    # 计算角度微分
                                    self.theta_delta[axis_name]=(self.IMU_RAW[axis_name][-1] + self.IMU_RAW[axis_name][-2])*0.005/2
                                    # 计算角度积分(减去偏置)
                                    self.gyro_theta[axis_name].value=self.gyro_theta[axis_name].value+self.theta_delta[axis_name]-self.gyro_bias[axis_name]
                            elif axis_name in 'X':
                                # 处理 X, Y, Z 轴的值   accel数据 加速度计
                                self.IMU_RAW[axis_name].append(event.value*  (16 / 65534))
                            elif axis_name in 'Y':
                                self.IMU_RAW[axis_name].append(event.value*  (16 / 65534))
                            elif axis_name in 'Z':
                                self.IMU_RAW[axis_name].append(event.value*  (16 / 65534))
                                if len(self.IMU_RAW['X'])and len(self.IMU_RAW['Y'])> 1:
                                    self.accl_theta['Y'].value=math.atan(self.IMU_RAW['X'][-1]/self.IMU_RAW['Z'][-1])
                                    self.accl_theta['X'].value=-math.atan(self.IMU_RAW['X'][-1]/math.sqrt(math.pow(self.IMU_RAW['Y'][-1],2)+math.pow(self.IMU_RAW['Z'][-1],2)))
                            # print("received data")
            except BlockingIOError:
                print("BlockingIOError")
joy_L = Joycon()
start_time = time.time()
print(f"start_time: {start_time}")
thread = threading.Thread(target=joy_L.update_IMU)

thread.start()
while True:
    end_time = time.time()
    time.sleep(0.005)
    print(f"RX: {joy_L.gyro_theta['RX'].value:.3f}, RY: {joy_L.gyro_theta['RY'].value:.3f}, RZ: {joy_L.gyro_theta['RZ'].value:.3f},X:{joy_L.accl_theta['X'].value:.3f},Y:{joy_L.accl_theta['Y'].value:.3f}======interval: {(end_time - start_time)*1000}ms")
    start_time=end_time
    # time.sleep(0.1)
thread.join()
print("Thread finished")