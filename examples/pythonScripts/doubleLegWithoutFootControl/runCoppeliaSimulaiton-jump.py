
import sys
#sys.path.append('./examples/pythonScripts/processAPI')
sys.path.append('../processAPI')
sys.path.append('../controller')

from simulation_process import VrepProcess
from simple_pid import PID
from kinematics import kinematics

import matplotlib.pyplot as plt
import numpy as np
from math import fmod

def main():
    mainProcess = VrepProcess()

    mainProcess.simConnect()
    mainProcess.simSetup(0.001, 2000)

    baseName = 'Body'
    jointName = 'hip_joint'
    mainProcess.simGetHandle(baseName, jointName)
    jointName = 'knee_joint'
    mainProcess.simGetHandle(baseName, jointName)

    mainProcess.simStart()
    mainProcess.simStepOnce()

    kk = kinematics()

    """
    控制环
    """
    hip_joint_handle = mainProcess.m_jointHandle[0]
    knee_joint_handle = mainProcess.m_jointHandle[1]
    hip_angle, hip_angle_dot = mainProcess.simGetJointInfo(hip_joint_handle)
    knee_angle, knee_angle_dot = mainProcess.simGetJointInfo(knee_joint_handle)

    q1 = np.zeros(2)
    q2 = np.zeros(2)
    q1[0] = hip_angle
    q1[1] = knee_angle
    p_des = kk.forwardkinematics(q1)
    pid_x = PID(2000, 0, 500, p_des[0])
    pid_z = PID(2000, 0, 500, p_des[1])

    f_des = np.zeros(2)
    tao_des = np.zeros(2)

    # pid_hip = PID(500,0,200,(hip_angle-0.1))                      #期望设置到当前
    # pid_knee = PID(200,0,50,knee_angle)

    time_hidtory = []
    hip_joint_history = []
    knee_joint_history = []

    for idx in range(mainProcess.m_N):
        time = mainProcess.simGetTime()

        hip_angle, hip_angle_dot = mainProcess.simGetJointInfo(hip_joint_handle)
        knee_angle, knee_angle_dot = mainProcess.simGetJointInfo(knee_joint_handle)

        q2[0] = hip_angle
        q2[1] = knee_angle
        p_cur = kk.forwardkinematics(q2)
        J = kk.getJacobian(q2)

        # print(p)
        # print(J)

        if fmod(idx, 800) < 400:
            f_des[0] = pid_x(p_cur[0])
            # f_des[1] = pid_z(p_cur[1]) - 150            # 重力补偿
            f_des[1] = pid_z(p_cur[1])
        else:
            f_des[0] = pid_x(p_cur[0])
            if p_cur[1] > -0.6:
                f_des[1] = - 400
            else:
                f_des[1] = pid_z(p_cur[1])
        # print(f_des)

        tao = np.dot(J.T, f_des)
    

        time_hidtory.append(0.001*time)
        hip_joint_history.append(hip_angle)
        knee_joint_history.append(knee_angle)

        # u_hip = pid_hip(hip_angle)
        # u_knee = pid_knee(knee_angle)
        # mainProcess.simSetJointCmd(hip_joint_handle, u_hip)
        # mainProcess.simSetJointCmd(knee_joint_handle, u_knee)

        mainProcess.simSetJointCmd(hip_joint_handle, tao[0])
        mainProcess.simSetJointCmd(knee_joint_handle, tao[1])

        #print(currentTime)
        #print(basePos)
        #print(theta)
        #print(theta_dot)

        if idx%100 == 0:
            print('running')
            print(f_des[1])
            # print(u_hip, u_knee)
        
        mainProcess.simNextStep()

    mainProcess.simStop()
    print('simulaiton stopped')

    plt.figure()
    plt.plot(time_hidtory,hip_joint_history)
    plt.plot(time_hidtory,knee_joint_history)
    plt.show()


if __name__ == '__main__':
    main()




