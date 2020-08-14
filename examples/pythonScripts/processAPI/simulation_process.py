

import numpy as np
import math
import sim as vrep
import time


class VrepProcess:
    def __init__(self):
        self.m_baseName = []
        self.m_jointName = []

        self.m_baseHandle = []
        self.m_jointHandle = []

        self.m_tStep = 0
        self.m_N = 0

        self.m_clientID = -1

    def simConnect(self):
        vrep.simxFinish(-1)
        # 每隔0.2s检测一次，直到连接上V-rep
        while True:
            self.m_clientID = vrep.simxStart('127.0.0.1', 19997, True, True, 5000, 5)
            if self.m_clientID > -1:
                break
            else:
                time.sleep(0.2)
                print("Failed connecting to remote API server!")
        print("Connection success!")

    def simSetup(self, tStep, N):
        self.m_tStep = tStep
        self.m_N = N

        # 然后打开同步模式
        vrep.simxSynchronous(self.m_clientID, True) 
        # 设置仿真步长，为了保持API端与V-rep端相同步长
        vrep.simxSetFloatingParameter(self.m_clientID, vrep.sim_floatparam_simulation_time_step, tStep, vrep.simx_opmode_blocking)


    def simGetHandle(self, baseName, jointName):
        """
        只在单关节对象适用。考虑到对旧版本程序的兼容，不删除。
        获取vrep中的对象句柄
        """
        self.m_baseName.append(baseName)
        self.m_jointName.append(jointName)

        _, baseHandle = vrep.simxGetObjectHandle(self.m_clientID, baseName, vrep.simx_opmode_blocking)
        _, jointHandle = vrep.simxGetObjectHandle(self.m_clientID, jointName, vrep.simx_opmode_blocking)

        self.m_baseHandle.append(baseHandle)
        self.m_jointHandle.append(jointHandle)

        print('Handles available!')
        print(baseHandle)
        print(jointHandle)
        print(self.m_jointHandle)

    def simGetObjectHandle(self, objName):
        """
        扩展simGetHandle
        获取vrep中的对象句柄
        """
        self.m_baseName.append(objName)
        _, baseHandle = vrep.simxGetObjectHandle(self.m_clientID, objName, vrep.simx_opmode_blocking)
        self.m_baseHandle.append(baseHandle)

        print("available objecte handle added:")
        print(self.m_baseName)
        print(self.m_baseHandle)

        return baseHandle

    def simGetJointHandle(self, jointName):
        """
        扩展simGetHandle
        获取vrep中的关节句柄
        """
        self.m_jointName.append(jointName)
        _, jointHandle = vrep.simxGetObjectHandle(self.m_clientID, jointName, vrep.simx_opmode_blocking)
        self.m_jointHandle.append(jointHandle)

        print("available joint handle added:")
        print(self.m_jointName)
        print(self.m_jointHandle)

        return jointHandle


    def simStart(self):
        # 启动仿真
        vrep.simxStartSimulation(self.m_clientID, vrep.simx_opmode_oneshot)

        time.sleep(1)
        print('start simulation')

        baseHandle = self.m_baseHandle[0]
        jointHandle = self.m_jointHandle[0]
        print(baseHandle)
        print(jointHandle)

        #lastCmdTime=vrep.simxGetLastCmdTime(clientID)  # 记录当前时间
        # _, base_pos = vrep.simxGetObjectPosition(self.m_clientID, baseHandle, -1, vrep.simx_opmode_oneshot)
        # _, jointConfig = vrep.simxGetJointPosition(self.m_clientID, jointHandle, vrep.simx_opmode_oneshot)
        # vrep.simxSynchronousTrigger(self.m_clientID)  # 让仿真走一步

        # time.sleep(1)
        # print('trigger once')

    def simStepOnce(self):
        for joint_handle in self.m_jointHandle:
            _, jointConfig = vrep.simxGetJointPosition(self.m_clientID, joint_handle, vrep.simx_opmode_oneshot)
        vrep.simxSynchronousTrigger(self.m_clientID)  # 让仿真走一步
        time.sleep(1)
        print('trigger once')

    def simGetInfo(self):
        clientID = self.m_clientID
        baseHandle = self.m_baseHandle[0]
        jointHandle = self.m_jointHandle[0]

        currCmdTime=vrep.simxGetLastCmdTime(clientID)  # 记录当前时间

        _, base_pos = vrep.simxGetObjectPosition(clientID, baseHandle, -1, vrep.simx_opmode_oneshot)
        _, jointConfig = vrep.simxGetJointPosition(clientID, jointHandle, vrep.simx_opmode_oneshot)

        return currCmdTime, base_pos, jointConfig
    
    def simGetTime(self):
        clientID = self.m_clientID
        currCmdTime=vrep.simxGetLastCmdTime(clientID)  # 记录当前时间

        return currCmdTime

    def simGetJointInfo(self, jointHandle):
        clientID = self.m_clientID
        _, j_pos = vrep.simxGetJointPosition(clientID, jointHandle, vrep.simx_opmode_oneshot)
        _, j_vel = vrep.simxGetObjectFloatParameter(clientID, jointHandle, 2012, vrep.simx_opmode_oneshot)

        return j_pos, j_vel


    def simSetCmd(self, control_force):
        clientID = self.m_clientID
        jointHandle = self.m_jointHandle

        if np.sign(control_force) >= 0:
            set_force = control_force
            set_velocity = 9999
        else:
            set_force = -control_force
            set_velocity = -9999

            # 控制命令需要同时方式，故暂停通信，用于存储所有控制命令一起发送
            vrep.simxPauseCommunication(clientID, True)
            
            vrep.simxSetJointTargetVelocity(clientID, jointHandle, set_velocity, vrep.simx_opmode_oneshot)
            vrep.simxSetJointForce(clientID, jointHandle, set_force, vrep.simx_opmode_oneshot)

            vrep.simxPauseCommunication(clientID, False)
            
    def simSetJointCmd(self, jointHandle, control_force):
        clientID = self.m_clientID

        if control_force >= 0:
            set_force = control_force
            set_velocity = 9999
        else:
            set_force = -control_force
            set_velocity = -9999

        # 控制命令需要同时方式，故暂停通信，用于存储所有控制命令一起发送
        #vrep.simxPauseCommunication(clientID, True)
        
        vrep.simxSetJointTargetVelocity(clientID, jointHandle, set_velocity, vrep.simx_opmode_oneshot)
        vrep.simxSetJointForce(clientID, jointHandle, set_force, vrep.simx_opmode_oneshot)

        #vrep.simxPauseCommunication(clientID, False)

    def simTestCmd(self, jointHandle):
        clientID = self.m_clientID
        set_velocity = 9999
        set_force = 1
        vrep.simxSetJointTargetVelocity(clientID, jointHandle, set_velocity, vrep.simx_opmode_oneshot)
        vrep.simxSetJointForce(clientID, jointHandle, set_force, vrep.simx_opmode_oneshot)

    def simSetJointVelocity(self, jointHandle, velocity):
        clientID = self.m_clientID

        vrep.simxSetJointTargetVelocity(clientID, jointHandle, velocity, vrep.simx_opmode_oneshot)

    def simNextStep(self):
        vrep.simxSynchronousTrigger(self.m_clientID)  # 让仿真走一步
        vrep.simxGetPingTime(self.m_clientID)    # 使得该仿真步走完

    def simStop(self):
        # 关闭仿真
        vrep.simxStopSimulation(self.m_clientID, vrep.simx_opmode_oneshot)



