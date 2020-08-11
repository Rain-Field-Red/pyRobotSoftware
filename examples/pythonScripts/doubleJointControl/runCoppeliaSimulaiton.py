
import sys
#sys.path.append('./examples/pythonScripts/processAPI')
sys.path.append('../processAPI')
sys.path.append('../controller')

from simulation_process import VrepProcess
from simple_pid import PID

import matplotlib.pyplot as plt

def main():
    mainProcess = VrepProcess()

    mainProcess.simConnect()
    mainProcess.simSetup(0.001, 1000)

    baseName = 'Body'
    jointName = 'hip_joint'
    mainProcess.simGetHandle(baseName, jointName)
    jointName = 'knee_joint'
    mainProcess.simGetHandle(baseName, jointName)

    mainProcess.simStart()
    mainProcess.simStepOnce()

    """
    控制环
    """
    hip_joint_handle = mainProcess.m_jointHandle[0]
    knee_joint_handle = mainProcess.m_jointHandle[1]
    hip_angle, hip_angle_dot = mainProcess.simGetJointInfo(hip_joint_handle)
    knee_angle, knee_angle_dot = mainProcess.simGetJointInfo(knee_joint_handle)
    pid_hip = PID(500,0,200,(hip_angle-0.1))                      #期望设置到当前
    pid_knee = PID(200,0,50,knee_angle)

    time_hidtory = []
    hip_joint_history = []
    knee_joint_history = []

    for idx in range(mainProcess.m_N):
        time = mainProcess.simGetTime()

        hip_angle, hip_angle_dot = mainProcess.simGetJointInfo(hip_joint_handle)
        knee_angle, knee_angle_dot = mainProcess.simGetJointInfo(knee_joint_handle)

        time_hidtory.append(0.001*time)
        hip_joint_history.append(hip_angle)
        knee_joint_history.append(knee_angle)

        u_hip = pid_hip(hip_angle)
        u_knee = pid_knee(knee_angle)
        mainProcess.simSetJointCmd(hip_joint_handle, u_hip)
        mainProcess.simSetJointCmd(knee_joint_handle, u_knee)

        #print(currentTime)
        #print(basePos)
        #print(theta)
        #print(theta_dot)

        if idx%100 == 0:
            print('running')
            #print(u)
        
        mainProcess.simNextStep()

    mainProcess.simStop()
    print('simulaiton stopped')

    plt.figure()
    plt.plot(time_hidtory,hip_joint_history)
    plt.plot(time_hidtory,knee_joint_history)
    plt.show()


if __name__ == '__main__':
    main()




