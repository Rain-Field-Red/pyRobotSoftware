
import sys
#sys.path.append('./examples/pythonScripts/processAPI')
sys.path.append('../processAPI')
sys.path.append('../controller')

from simulation_process import VrepProcess
from simple_pid import PID

from GaitScheduler import GaitType
from GaitScheduler import GaitScheduler
from CircularInterpolation import FootSwingTrajectory

from kinematics import kinematics

#from trackingPID import trackingPD

import matplotlib.pyplot as plt
import numpy as np
from math import fmod, sqrt, pi

def main():
    mainProcess = VrepProcess()

    mainProcess.simConnect()
    mainProcess.simSetup(0.001, 2000)

    # 加入本体对象
    baseName = 'Body'
    body_handle = mainProcess.simGetObjectHandle(baseName)

    # 加入左腿关节
    jointName = 'left_hip_joint'
    left_hip_joint_handle = mainProcess.simGetJointHandle(jointName)
    jointName = 'left_knee_joint'
    left_knee_joint_handle = mainProcess.simGetJointHandle(jointName)
    jointName = 'left_ankle_joint'
    left_ankle_joint_handle = mainProcess.simGetJointHandle(jointName)
    # 加入右腿关节
    jointName = 'right_hip_joint'
    right_hip_joint_handle = mainProcess.simGetJointHandle(jointName)
    jointName = 'right_knee_joint'
    right_knee_joint_handle = mainProcess.simGetJointHandle(jointName)
    jointName = 'right_ankle_joint'
    right_ankle_joint_handle = mainProcess.simGetJointHandle(jointName)

    mainProcess.simStart()
    mainProcess.simStepOnce()

    # 创建步态规划器和轨迹规划器
    gaitType = GaitType()
    gait = GaitScheduler(10*mainProcess.m_tStep)
    leftTraj = FootSwingTrajectory()
    rightTraj = FootSwingTrajectory()

    # 创建运动学计算模块
    kk = kinematics()


    # 创建左腿控制器
    left_hip_angle, left_hip_angle_dot = mainProcess.simGetJointInfo(left_hip_joint_handle)
    left_knee_angle, left_knee_angle_dot = mainProcess.simGetJointInfo(left_knee_joint_handle)
    left_ankle_angle, left_ankle_angle_dot = mainProcess.simGetJointInfo(left_ankle_joint_handle)
    pid_left_hip = PID(1000,0,200,(left_hip_angle))                      #期望设置到当前
    pid_left_knee = PID(700,0,150,(left_knee_angle))
    pid_left_ankle = PID(200,0,50,(left_ankle_angle))

    # 创建右腿控制器
    right_hip_angle, right_hip_angle_dot = mainProcess.simGetJointInfo(right_hip_joint_handle)
    right_knee_angle, right_knee_angle_dot = mainProcess.simGetJointInfo(right_knee_joint_handle)
    right_ankle_angle, right_ankle_angle_dot = mainProcess.simGetJointInfo(right_ankle_joint_handle)
    pid_right_hip = PID(1000,0,200,(right_hip_angle))                      #期望设置到当前
    pid_right_knee = PID(700,0,150,(right_knee_angle))
    pid_right_ankle = PID(200,0,50,(right_ankle_angle))

    # 记录关节轨迹
    time_hidtory = []
    left_hip_joint_history = []
    left_knee_joint_history = []
    left_ankle_joint_history = []
    right_hip_joint_history = []
    right_knee_joint_history = []
    right_ankle_joint_history = []

    count = 0                               # 用于规划的计数
    for idx in range(mainProcess.m_N):

        # 接收信息
        time = mainProcess.simGetTime()

        left_hip_angle, left_hip_angle_dot = mainProcess.simGetJointInfo(left_hip_joint_handle)
        left_knee_angle, left_knee_angle_dot = mainProcess.simGetJointInfo(left_knee_joint_handle)
        left_ankle_angle, left_ankle_angle_dot = mainProcess.simGetJointInfo(left_ankle_joint_handle)

        right_hip_angle, right_hip_angle_dot = mainProcess.simGetJointInfo(right_hip_joint_handle)
        right_knee_angle, right_knee_angle_dot = mainProcess.simGetJointInfo(right_knee_joint_handle)
        right_ankle_angle, right_ankle_angle_dot = mainProcess.simGetJointInfo(right_ankle_joint_handle)

        # 记录信息
        time_hidtory.append(0.001*time)

        left_hip_joint_history.append(left_hip_angle)
        left_knee_joint_history.append(left_knee_angle)
        left_ankle_joint_history.append(left_ankle_angle)

        right_hip_joint_history.append(right_hip_angle)
        right_knee_joint_history.append(right_knee_angle)
        right_ankle_joint_history.append(right_ankle_angle)

        """
        规划
        """
        if 0 == fmod(idx, 10):
            #print('fmod')
            tmp1 = np.zeros(3)
            tmp2 = np.zeros(3)
            pd = np.zeros(3)

            # 左腿的起始点和终点
            tmp1[0] = 0
            tmp1[1] = 0
            tmp1[2] = 0
            leftTraj.setInitialPosition(tmp1)

            tmp2[0] = 0
            tmp2[1] = 0
            tmp2[2] = 0
            leftTraj.setFinalPosition(tmp2)
            
            leftTraj.setHeight(0.2)

            # 右腿的起始点和终点
            tmp1[0] = 0
            tmp1[1] = 0
            tmp1[2] = 0
            rightTraj.setInitialPosition(tmp1)

            tmp2[0] = 0
            tmp2[1] = 0
            tmp2[2] = 0
            rightTraj.setFinalPosition(tmp2)
            
            rightTraj.setHeight(0.2)

            if 50 == count:
                gait.gaitData._nextGait = gaitType.STATIC_WALK
                gait.createGait()
            
            gait.step()
            leftPhase = gait.gaitData.phaseSwing[0]
            rightPhase = gait.gaitData.phaseSwing[1]

            # print(leftPhase)
            # print(rightPhase)

            leftTraj.computeSwingTrajectoryCircular(leftPhase)
            rightTraj.computeSwingTrajectoryCircular(rightPhase)

            # 将期望关节角度输入到pid控制器
            # 左腿
            pd[0] = leftTraj._p[0]
            pd[1] = leftTraj._p[2] - 0.4*sqrt(2)
            pd[2] = pi/2
            # qd = kk.inversekinematics(pd)
            qd = kk.inversekinematics2(pd)
            # print(pd)
            # print(qd)

            pid_left_hip.setTargetPoint(qd[0])
            pid_left_knee.setTargetPoint(qd[1])
            pid_left_ankle.setTargetPoint(qd[2])

            # 右腿
            pd[0] = rightTraj._p[0]
            pd[1] = rightTraj._p[2] - 0.4*sqrt(2)
            pd[2] = pi/2
            # qd = kk.inversekinematics(pd)
            qd = kk.inversekinematics2(pd)
            # print(pd)
            # print(qd)

            pid_right_hip.setTargetPoint(qd[0])
            pid_right_knee.setTargetPoint(qd[1])
            pid_right_ankle.setTargetPoint(qd[2])


            count += 1


        

        """
        控制
        """
        

        # 计算控制量
        u_left_hip = pid_left_hip(left_hip_angle)
        u_left_knee = pid_left_knee(left_knee_angle)
        u_left_ankle = pid_left_ankle(left_ankle_angle)

        u_right_hip = pid_right_hip(right_hip_angle)
        u_right_knee = pid_right_knee(right_knee_angle)
        u_right_ankle = pid_right_ankle(right_ankle_angle)


        # 输出控制量
        mainProcess.simSetJointCmd(left_hip_joint_handle, u_left_hip)
        mainProcess.simSetJointCmd(left_knee_joint_handle, u_left_knee)
        mainProcess.simSetJointCmd(left_ankle_joint_handle, u_left_ankle)

        mainProcess.simSetJointCmd(right_hip_joint_handle, u_right_hip)
        mainProcess.simSetJointCmd(right_knee_joint_handle, u_right_knee)
        mainProcess.simSetJointCmd(right_ankle_joint_handle, u_right_ankle)

        #print(currentTime)
        #print(basePos)
        #print(theta)
        #print(theta_dot)

        if idx%100 == 0:
            print('running')
            #print(u_left_hip)
        
        mainProcess.simNextStep()

    mainProcess.simStop()
    print('simulaiton stopped')

    plt.figure(1)
    plt.plot(time_hidtory,left_hip_joint_history,'b-')
    plt.plot(time_hidtory,left_knee_joint_history,'g-')
    plt.plot(time_hidtory,left_ankle_joint_history,'r-')
    plt.figure(2)
    plt.plot(time_hidtory,right_hip_joint_history,'b-')
    plt.plot(time_hidtory,right_knee_joint_history,'g-')
    plt.plot(time_hidtory,right_ankle_joint_history,'r-')
    plt.show()


if __name__ == '__main__':
    main()




