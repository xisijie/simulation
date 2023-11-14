import sim
import sys
import time
import math
import numpy as np
import cv2
port = 19999
from tkinter import messagebox


def monitor(state, vilolation):
    obs = True
    start = True
    slow = True
    # choice = not p or not q
    if state == 1:
        if not obs:
            state = 1
        elif not start and slow:
            state = 0
        elif slow:
            state = 4
        elif not start and not obs:
            state = 0
        else:
            vilolation.append("违反")
            messagebox.showinfo("警告", "小于安全距离")
    if state == 2:
        if slow:
            state = 1
        elif start and slow:
            state = 4
        elif not obs:
            state = 3
    if state == 3:
        if not obs:
            state = 1
        elif not start and not obs:
            state = 0
        elif start and not obs:
            state = 4
        elif start and slow:
            state = 3
    if state == 4:
        if start and slow:
            state = 3
        elif not start and slow:
            state = 0
        elif slow:
            state = 2


if __name__ == '__main__':
    sim.simxFinish(-1)
    clientID = sim.simxStart('127.0.0.1', port, True, True, 5000, 5)
    if clientID != -1:  # check if client connection successful
        print("Connected to remote API server")
    else:
        print('Connection not successful')
        sys.exit('Could not connect')

    rearRightJoint = sim.simxGetObjectHandle(clientID, 'rearRightMotor', sim.simx_opmode_blocking)[1]
    rearLeftJoint = sim.simxGetObjectHandle(clientID, 'rearLeftMotor', sim.simx_opmode_blocking)[1]
    frontRightJoint = sim.simxGetObjectHandle(clientID, 'frontRightMotor', sim.simx_opmode_blocking)[1]
    frontLeftJoint = sim.simxGetObjectHandle(clientID, 'frontLeftMotor', sim.simx_opmode_blocking)[1]

    visionSensor = sim.simxGetObjectHandle(clientID, 'Vision_sensor', sim.simx_opmode_blocking)[1]
    print(visionSensor)

    car = sim.simxGetObjectHandle(clientID, 'vehicle', sim.simx_opmode_blocking)[1]
    redcar = sim.simxGetObjectHandle(clientID, 'example', sim.simx_opmode_blocking)[1]
# 定义轮子速度 这些代码使用了 simx_opmode_oneshot 模式，这意味着每个命令将被发送一次，然后等待命令完成，这是一种非阻塞模式，程序会继续往下执行而不会等待命令的执行结果。
    v = 50
    sim.simxSetJointTargetVelocity(clientID, rearRightJoint, v, sim.simx_opmode_oneshot)
    sim.simxSetJointTargetVelocity(clientID, frontLeftJoint, v, sim.simx_opmode_oneshot)
    sim.simxSetJointTargetVelocity(clientID, rearLeftJoint, v, sim.simx_opmode_oneshot)
    sim.simxSetJointTargetVelocity(clientID, frontRightJoint, v, sim.simx_opmode_oneshot)
    i = 0

# 循环遍历获取传感器数据和红色车的位置
    while(1):
        # 获取视觉传感器图像数据
        code, resolution, image = sim.simxGetVisionSensorImage(clientID, visionSensor, 0, sim.simx_opmode_streaming)
        # 获取小车位置
        result = sim.simxGetObjectPosition(clientID, redcar, car, sim.simx_opmode_streaming)
        print(i)
        i += 1
        # 判断是否违反安全距离，若是则停止小车并弹出警告窗口
        if (result[0] == 0):
            if (result[1][0] < 16):
                messagebox.showinfo("警告", "小于安全距离")
                sim.simxSetJointTargetVelocity(clientID, rearRightJoint, 0, sim.simx_opmode_oneshot)
                sim.simxSetJointTargetVelocity(clientID, rearLeftJoint, 0, sim.simx_opmode_oneshot)
                sim.simxSetJointTargetVelocity(clientID, frontRightJoint, 0, sim.simx_opmode_oneshot)
                sim.simxSetJointTargetVelocity(clientID, frontLeftJoint, 0, sim.simx_opmode_oneshot)
                break

# 处理传感器数据，将其转换为numpy数组
        if code == 0:
            sensorImage = np.array(image, dtype=np.uint8)
            sensorImage.resize([resolution[1], resolution[0], 3])  # 整理矩阵
            cv2.flip(sensorImage, 0, sensorImage)  # 颠倒矩阵

            cv2.imshow('1', sensorImage)  # 显示图像
            if cv2.waitKey(1) == 27:
                break
            # cv2.waitKey(0)
            # cv2.destroyAllWindows()



