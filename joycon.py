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
        self.IMU_RAW_LIST = {name: [] for name in self.axis_names.values()}
        self.IMU_RAW_NEW = {name:Value('d', 0.0) for name in self.axis_names.values()}
        self.IMU_KF = {name:Value('d', 0.0) for name in self.axis_names.values()}
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
        self.gyro_bias['RZ'] = -0.01035
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
        self.thread_receive = threading.Thread(target=self.update_IMU)
        self.thread_receive.start()
        self.thread_state = threading.Thread(target=self.filter_KF)
        self.thread_state.start()
    # 更新IMU数据
    def update_IMU(self):
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
                                self.IMU_RAW_LIST[axis_name].append(event.value * (4000 / 65534000))
                                self.IMU_RAW_NEW[axis_name].value=self.IMU_RAW_LIST[axis_name][-1]
                                if len(self.IMU_RAW_LIST[axis_name]) > 1:
                                    # 计算角度微分
                                    self.theta_delta[axis_name]=(self.IMU_RAW_LIST[axis_name][-1] + self.IMU_RAW_LIST[axis_name][-2])*0.005/2
                                    # 计算角度积分(减去偏置)
                                    self.gyro_theta[axis_name].value=self.gyro_theta[axis_name].value+self.theta_delta[axis_name]-self.gyro_bias[axis_name]
                            elif axis_name in 'X':
                                # 处理 X, Y, Z 轴的值   accel数据 加速度计
                                self.IMU_RAW_LIST[axis_name].append(event.value*  (16 / 65534)*9.8)
                                self.IMU_RAW_NEW[axis_name].value=self.IMU_RAW_LIST[axis_name][-1]
                            elif axis_name in 'Y':
                                self.IMU_RAW_LIST[axis_name].append(event.value*  (16 / 65534)*9.8)
                                self.IMU_RAW_NEW[axis_name].value=self.IMU_RAW_LIST[axis_name][-1]
                            elif axis_name in 'Z':
                                self.IMU_RAW_LIST[axis_name].append(event.value*  (16 / 65534)*9.8)
                                self.IMU_RAW_NEW[axis_name].value=self.IMU_RAW_LIST[axis_name][-1]
                                if len(self.IMU_RAW_LIST['X'])and len(self.IMU_RAW_LIST['Y'])> 1:
                                    if self.IMU_RAW_LIST['Z'][-1]==0:
                                        self.IMU_RAW_LIST['Z'][-1]=0.0000001
                                    temp_ax=math.atan(self.IMU_RAW_LIST['Y'][-1]/self.IMU_RAW_LIST['Z'][-1])
                                    self.accl_theta['X'].value=(temp_ax/math.pi)*180
                                    temp_ry=-math.atan(self.IMU_RAW_LIST['X'][-1]/math.sqrt(math.pow(self.IMU_RAW_LIST['Y'][-1],2)+math.pow(self.IMU_RAW_LIST['Z'][-1],2)))
                                    self.accl_theta['Y'].value=(temp_ry/math.pi)*180
                            # print("received data")
            except BlockingIOError:
                print("BlockingIOError")
    def filter_KF(self):
        dt=0.005
        F=np.array([
            [1,-dt,0,0]
           ,[0,  1,0,0]
           ,[0,  0,1,-dt]
           ,[0,  0,0, 1]
        ])
        B=np.array([
            [dt,0]
           ,[0,0]
           ,[0,dt]
           ,[0,0]
        ])
        P_0=np.array([
            [0,0,0,0]
           ,[0,0,0,0]
           ,[0,0,0,0]
           ,[0,0,0,0]
        ])
        P_k=P_0
        Q_theta=0.001
        Q_omega_b=0.003
        Q=np.array([
            [Q_theta,0,0,0]
           ,[0,Q_omega_b,0,0]
           ,[0,0,Q_theta,0]
           ,[0,0,0,Q_omega_b]
        ])
        Q=Q*dt
        R_measure=0.06
        R=np.array([
            [R_measure,0],
            [0,R_measure]
        ])

        H=np.array([
            [1,0,0,0],
            [0,0,1,0]
        ])
        omega_b=0.0058
        X_0=np.array([
             [0]
            ,[omega_b]
            ,[0]
            ,[omega_b]
        ])
        X_k=X_0
        
        I=np.eye(4)
        K_0=np.array([
            [0.5,0],
            [0.5,0],
            [0,0.5],
            [0,0.5]
        ])
        K_k=K_0
        u_k=np.array([
            [0]
           ,[0]
        ])
        z_k=np.array([
            [0]
           ,[0]
        ])
        y_k=np.array([
            [0]
           ,[0]
        ])
        start_time=time.time()
        while (True):
            if u_k[0][0]!=self.IMU_RAW_NEW["RX"].value:

                u_k[0][0]=self.IMU_RAW_NEW["RX"].value
                u_k[1][0]=self.IMU_RAW_NEW["RY"].value
                z_k[0][0]= self.accl_theta['X'].value
                z_k[1][0]= self.accl_theta['Y'].value

                X_k=F@X_k+B@u_k
                self.IMU_KF["RX"].value=X_k[0][0]
                self.IMU_KF["RY"].value=X_k[2][0]
                P_k=F@P_k@F.T+Q
                K_k=P_k@H.T@np.linalg.inv(H@P_k@H.T+R)
                print("K_k",K_k)
                y_k=z_k-H@(X_k)
                print("H@X_k",H@X_k)
                X_k=X_k+K_k@y_k
                P_k=(I-K_k@H)@P_k
            time.sleep(0.001)


            
