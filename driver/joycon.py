from evdev import InputDevice, categorize, ecodes
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import time
import numpy as np
import os
import threading
from multiprocessing import Value, Array
import math
from scipy.spatial.transform import Rotation as R
class Joycon:
    def __init__(self, device_path="/dev/input/event22",filter_type='KF'):
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

        self.running = True
        self.thread_receive = threading.Thread(target=self.update_IMU)
        self.thread_receive.start()
        # 姿态估计求解器
        self.filter_type = filter_type
        if filter_type=="KF":
            self.thread_filter = threading.Thread(target=self.Filter_Kalman)
        elif filter_type=="CF":
            self.thread_filter = threading.Thread(target=self.Filter_Complementary)
        self.thread_filter.start()

    # 更新IMU数据
    def update_IMU(self):
        while self.running :
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
                            print("received data")
            except BlockingIOError:
                print("BlockingIOError")
            # except Exception as e:
            #     print(f"Error: {e}")
    def Filter_Kalman(self):
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
        while self.running :
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

                y_k=z_k-H@(X_k)
                
                X_k=X_k+K_k@y_k
                P_k=(I-K_k@H)@P_k
            time.sleep(0.001)
    def Filter_Complementary(self):
        #TODO: 初始化
        gyro_rx=0 # 弧度，角速度计获取
        gyro_ry=0
        gyro_rz=0
        accl_rx=0 # 弧度，加速度计获取,归一化处理
        accl_ry=0
        accl_rz=0
        norm=0  
        q0=1
        q1=0
        q2=0
        q3=0
        self.q_0=q0
        self.q_1=q1
        self.q_2=q2
        self.q_3=q3
        exInt=0
        eyInt=0
        ezInt=0
        Kp=0.75 # 2.0
        Ki=0.001 # 0.005
        dt=0.005 # 采样周期
        halfT = 0.5 * dt # 采样周期的一半
        #TODO:核心算法
        while self.running:
            if gyro_rx!=(self.IMU_RAW_NEW["RX"].value/180*math.pi):
                # 读取当前  陀螺仪数据
                gyro_rx=self.IMU_RAW_NEW["RX"].value/180*math.pi
                gyro_ry=self.IMU_RAW_NEW["RY"].value/180*math.pi
                gyro_rz=self.IMU_RAW_NEW["RZ"].value/180*math.pi
                # 读取当前  加速度计数据
                accl_rx=self.IMU_RAW_NEW['X'].value
                accl_ry=self.IMU_RAW_NEW['Y'].value
                accl_rz=self.IMU_RAW_NEW["Z"].value
                norm = math.sqrt(accl_rx * accl_rx + accl_ry * accl_ry + accl_rz * accl_rz)
                accl_rx=accl_rx/norm
                accl_ry=accl_ry/norm
                accl_rz=accl_rz/norm
                # 计算方向余弦矩阵的更新，仅计算方向余弦矩阵的第三列
                q0q0 = q0 * q0
                q0q1 = q0 * q1
                q0q2 = q0 * q2
                q1q1 = q1 * q1
                q1q3 = q1 * q3
                q2q2 = q2 * q2
                q2q3 = q2 * q3
                q3q3 = q3 * q3
                vx = 2 * (q1q3 - q0q2)
                vy = 2 * (q0q1 + q2q3)
                vz = q0q0 - q1q1 - q2q2 + q3q3
                # 计算误差向量
                ex = accl_ry * vz - accl_rz * vy
                ey = accl_rz * vx - accl_rx * vz
                ez = accl_rx * vy - accl_ry * vx
                # 计算积分项
                exInt += ex * Ki
                eyInt += ey * Ki
                ezInt += ez * Ki
                gyro_rx += Kp * ex + exInt
                gyro_ry += Kp * ey + eyInt
                gyro_rz += Kp * ez + ezInt
                # 更新四元数
                q0 += (-q1 * gyro_rx - q2 * gyro_ry - q3 * gyro_rz) * halfT
                q1 += (q0 * gyro_rx + q2 * gyro_rz - q3 * gyro_ry) * halfT
                q2 += (q0 * gyro_ry - q1 * gyro_rz + q3 * gyro_rx) * halfT
                q3 += (q0 * gyro_rz + q1 * gyro_ry - q2 * gyro_rx) * halfT
                # 归一化四元数
                norm = math.sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3)
                q0 /= norm
                q1 /= norm
                q2 /= norm
                q3 /= norm
                self.q_0 = q0
                self.q_1 = q1
                self.q_2 = q2
                self.q_3 = q3
            time.sleep(0.003)
    def get_Quaternion(self)-> list: #TODO: 获取四元数
            if self.filter_type == "CF":
                # 返回四元数
                return [self.q_0, self.q_1, self.q_2, self.q_3]
            elif self.filter_type == "KF":
                rx=self.IMU_KF["RX"].value
                ry=self.IMU_KF["RY"].value
                rz=self.gyro_theta["RZ"].value
                rotation = R.from_euler('zyx', [rx, ry, rz], degrees=True)
                # 获取四元数
                quaternion = rotation.as_quat()
                quaternion_list = [quaternion[3], quaternion[0], quaternion[1], quaternion[2]]
            
            return quaternion_list
    
    def __del__(self):
        # 设置线程运行标志为 False
        self.running = False
        # 等待线程结束
        self.thread_receive.join()
        self.thread_filter.join()
        print("Threads have been stopped.")


            
