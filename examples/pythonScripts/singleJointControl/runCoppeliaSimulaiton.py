
import sys
#sys.path.append('./examples/pythonScripts/processAPI')
sys.path.append('../processAPI')

from simulation_process import VrepProcess

import matplotlib.pyplot as plt

def main():
    mainProcess = VrepProcess()

    mainProcess.simConnect()
    mainProcess.simSetup(0.01, 200)

    baseName = 'Body'
    jointName = 'hip_joint'
    mainProcess.simGetHandle(baseName, jointName)

    mainProcess.simStart()

    """
    控制环
    """
    time_hidtory = []
    theta_history = []
    theta_dot_history = []

    for idx in range(mainProcess.m_N):
        time = mainProcess.simGetTime()

        #currentTime, basePos, jointConfig = mainProcess.simGetInfo()
        hip_joint_handle = mainProcess.m_jointHandle[0]
        theta, theta_dot = mainProcess.simGetJointInfo(hip_joint_handle)

        time_hidtory.append(0.01*time)
        theta_history.append(theta)
        theta_dot_history.append(theta_dot)

        #print(currentTime)
        #print(basePos)
        print(theta)
        print(theta_dot)
        
        mainProcess.simNextStep()

    mainProcess.simStop()
    print('simulaiton stopped')

    plt.figure()
    plt.plot(time_hidtory,theta_history)

    plt.plot(time_hidtory,theta_dot_history)
    plt.show()


if __name__ == '__main__':
    main()




