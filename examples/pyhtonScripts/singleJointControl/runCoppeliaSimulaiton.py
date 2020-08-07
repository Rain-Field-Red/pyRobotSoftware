


from simulation_process import VrepProcess


def main():
    mainProcess = VrepProcess()

    mainProcess.simConnect()
    mainProcess.simSetup(0.01, 100)

    baseName = 'Body'
    jointName = 'hip_joint'
    mainProcess.simGetHandle(baseName, jointName)

    mainProcess.simStart()

    while True:
        currentTime, basePos, jointConfig = mainProcess.simGetInfo()

        print(currentTime)
        print(basePos)
        print(jointConfig)
        
        mainProcess.simNextStep()


if __name__ == '__main__':
    main()




