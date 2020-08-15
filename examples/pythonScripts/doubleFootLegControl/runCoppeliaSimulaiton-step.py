
import sys
#sys.path.append('./examples/pythonScripts/processAPI')
sys.path.append('../processAPI')
sys.path.append('../controller')

from simulation_process import VrepProcess
from simple_pid import PID

#from trackingPID import trackingPD

import matplotlib.pyplot as plt

def main():
    mainProcess = VrepProcess()

    mainProcess.simConnect()
    mainProcess.simSetup(0.001, 1000)

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

    """
    规划
    """

    """
    控制环
    """
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
            print(u_left_hip)
        
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




