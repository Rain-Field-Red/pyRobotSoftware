


# by eric
# 2018-08-16
# inspired by cheetah software

import numpy as np
import math

class GaitType:
    def __init__(self):
        self.STAND = 0
        self.STAND_CYCLE = 1
        self.STATIC_WALK = 2

        self.TRANSITION_TO_STAND = 20


class GaitData:
    def __init__(self):
        self._currentGait = GaitType()
        self._nextGait = GaitType()

        self.gaitName = ''

        # 步态描述
        self.periodTimeNominal = 0.0
        self.initialPhase = 0.0
        self.switchingPhaseNominal = 0.0
        self.overrideable = 0       # 允许改写步态参数

        self.gaitEnabled = np.zeros(2)      # 使能

        # 基于时间的描述
        self.periodTime = np.zeros(2)             # 整个周期时长
        self.timeStance = np.zeros(2)             # 支撑时长
        self.timeSwing = np.zeros(2)              # 摆动时长
        self.timeStanceRemaining = np.zeros(2)    # 剩余支撑时间
        self.timeSwingRemaining = np.zeros(2)     # 剩余摆动时间

        # 基于相位的描述
        self.switchingPhase = np.zeros(2)         # 切换到摆动的相位
        self.phaseVariable = np.zeros(2)          # 整体相位
        self.phaseOffset = np.zeros(2)            # 标称的相位偏移
        self.phaseScale = np.zeros(2)             # 相对于相位变量的相位指示
        self.phaseStance = np.zeros(2)            # 支撑子相位
        self.phaseSwing = np.zeros(2)             # 摆动子相位

        # 规划的接触状态
        self.contactStateScheduled = np.zeros(2)  # 足端的接触状态
        self.contactStatePrev = np.zeros(2)       # 前一时刻的接触状态
        self.touchdownScheduled = np.zeros(2)     # 规划的接触事件标志
        self.liftoffScheduled = np.zeros(2)       # 规划的离地事件标志

        self.zero()

    def zero(self):
        self._nextGait = self._currentGait

        
class GaitScheduler:
    def __init__(self, _dt):
        self.gaitData = GaitData()

        # 自然步态修正
        self.period_time_natural = 0.5;
        self.switching_phase_natural = 0.5;
        self.swing_time_natural = 0.25;

        # 控制环步长变化
        self.dt = _dt

        # 每一步的相位变化
        self.dphase = 0.0

        # 信息打印的间隔
        self.printNum = 5
        # 从上次打印开始计数
        self.printIter = 0

        self.initialize()


    def initialize(self):
        print('[GAIT] Initialize Gait Scheduler')
        
        gaitType = GaitType()
        self.gaitData._currentGait = gaitType.STAND

        # 步态数据清零
        self.gaitData.zero()

        # 从标称初始创建步态
        self.createGait()

        self.period_time_natural = self.gaitData.periodTimeNominal
        self.switching_phase_natural = self.gaitData.switchingPhaseNominal

    def createGait(self):
        print('[GAIT]  Transitioning gait from', self.gaitData.gaitName)
        print('to')
        
        gaitType = GaitType()
        if gaitType.STAND == self.gaitData._nextGait:
            self.gaitData.gaitName = 'STAND'
            self.gaitData.gaitEnabled = np.ones(2)
            self.gaitData.periodTimeNominal = 10.0
            self.gaitData.initialPhase = 0.0
            self.gaitData.switchingPhaseNominal = 1.0
            self.gaitData.phaseOffset = 0.5 * np.ones(2)
            self.gaitData.phaseScale = np.ones(2)
            self.gaitData.overrideable = 0

        elif gaitType.STAND_CYCLE == self.gaitData._nextGait:
            self.gaitData.gaitName = 'STAND_CYCLE'
            self.gaitData.gaitEnabled = np.ones(2)
            self.gaitData.periodTimeNominal = 1.0
            self.gaitData.initialPhase = 0.0
            self.gaitData.switchingPhaseNominal = 1.0
            self.gaitData.phaseOffset = 0.5 * np.ones(2)
            self.gaitData.phaseScale = np.ones(2)
            self.gaitData.overrideable = 0

        elif gaitType.STATIC_WALK == self.gaitData._nextGait:
            self.gaitData.gaitName = 'STATIC_WALK'
            self.gaitData.gaitEnabled = np.ones(2)
            self.gaitData.periodTimeNominal = 1.25
            self.gaitData.initialPhase = 0.0
            self.gaitData.switchingPhaseNominal = 0.8
            tmp = np.ones(2)
            tmp[0] = 0.0
            tmp[1] = 0.5
            self.gaitData.phaseOffset = tmp
            self.gaitData.phaseScale = np.ones(2)
            self.gaitData.overrideable = 1

        elif gaitType.TRANSITION_TO_STAND == self.gaitData._nextGait:
            self.gaitData.gaitName = 'TRANSITION_TO_STAND'

        self.gaitData._currentGait = self.gaitData._nextGait
        print(self.gaitData.gaitName)

        # 计算其他的步态信息
        self.calcAuxiliaryGaitData()

    def modifyGait(self):
        print('modify nothing')
    
    def step(self):

        # 
        self.modifyGait()

        gaitType = GaitType()
        
        if gaitType.STAND != self.gaitData._currentGait:
            # 跟踪参考相位变量
            self.gaitData.initialPhase = \
                math.fmod((self.gaitData.initialPhase + (self.dt/self.gaitData.periodTimeNominal)), 1)
        # 对每条腿进行操作
        for foot in range(2):
            # 为下一步设置previous contact state
            self.gaitData.contactStatePrev[foot] = \
                self.gaitData.contactStateScheduled[foot]

            if 1 == self.gaitData.gaitEnabled[foot]:
                # 时间单调递增
                if gaitType.STAND == self.gaitData._currentGait:
                    # 站立时相位不增加
                    self.dphase = 0.0
                else:
                    self.dphase = \
                        self.gaitData.phaseScale[foot] * (self.dt/self.gaitData.periodTimeNominal)

                # 找到每条腿的当前相位
                self.gaitData.phaseVariable[foot] = \
                    math.fmod((self.gaitData.phaseVariable[foot] + self.dphase), 1)

                # 检查当前的接触状态
                if self.gaitData.phaseVariable[foot] <= self.gaitData.switchingPhase[foot]:
                    # 已经规划为接触
                    self.gaitData.contactStateScheduled[foot] = 1

                    # 支撑子相位计算
                    self.gaitData.phaseStance[foot] = \
                        self.gaitData.phaseVariable[foot] / self.gaitData.switchingPhase[foot]

                    # 接触时摆动相没有开始
                    self.gaitData.phaseSwing[foot] = 0.0

                    # 计算支撑剩余时间
                    self.gaitData.timeStanceRemaining[foot] = \
                        self.gaitData.periodTime[foot] * \
                            (self.gaitData.switchingPhase[foot] - self.gaitData.phaseVariable[foot])

                    # 接触时摆动剩余时间为0
                    self.gaitData.timeSwingRemaining[foot] = 0.0

                    # 首次接触以为着落足
                    if 0 == self.gaitData.contactStatePrev[foot]:
                        self.gaitData.touchdownScheduled[foot] = 1
                    else:
                        self.gaitData.touchdownScheduled[foot] = 0
                
                else:
                    # 没有规划为接触
                    self.gaitData.contactStateScheduled[foot] = 0

                    # 摆动时支撑相位完成
                    self.gaitData.phaseStance[foot] = 1.0

                    # 摆动子相位计算
                    self.gaitData.phaseSwing[foot] = \
                        (self.gaitData.phaseVariable[foot] - self.gaitData.switchingPhase[foot]) / \
                            (1.0 - self.gaitData.switchingPhase[foot])

                    # 摆动时支撑剩余时间为0
                    self.gaitData.timeStanceRemaining[foot] = 0.0
                    
                    # 计算摆动剩余时间
                    self.gaitData.timeSwingRemaining[foot] = \
                        self.gaitData.periodTime[foot] * (1 - self.gaitData.phaseVariable[foot])

                    # 首次不接触意味着抬腿
                    if 1 == self.gaitData.contactStatePrev[foot]:
                        self.gaitData.liftoffScheduled[foot] = 1
                    else:
                        self.gaitData.liftoffScheduled[foot] = 0
                
            else:
                # 设置抬腿标志为0
                self.gaitData.liftoffScheduled[foot] = 0


    def calcAuxiliaryGaitData(self):
        # 为每条腿设置步态参数
        for foot in range(2):
            if 1 == self.gaitData.gaitEnabled[foot]:
                # 每条腿的比例周期时间
                self.gaitData.periodTime[foot] = \
                    self.gaitData.periodTimeNominal / self.gaitData.phaseScale[foot]
                
                # 在哪个相位从支撑切换为摆动
                self.gaitData.switchingPhase[foot] = self.gaitData.switchingPhaseNominal

                # 根据偏移量初始化相位变量
                self.gaitData.phaseVariable[foot] = \
                    self.gaitData.initialPhase + self.gaitData.phaseOffset[foot]

                # 找到步态周期中的支撑时间
                self.gaitData.timeStance[foot] = \
                    self.gaitData.periodTime[foot] * self.gaitData.switchingPhase[foot]

                # 找到步态周期中的摆动时间
                self.gaitData.timeSwing[foot] = \
                    self.gaitData.periodTime[foot] * (1.0 - self.gaitData.switchingPhase[foot])

            else:
                # The scaled period time for each foot
                self.gaitData.periodTime[foot] = 0.0

                # Phase at which to switch the foot from stance to swing
                gaitData.switchingPhase[foot] = 0.0

                # Initialize the phase variables according to offset
                gaitData.phaseVariable[foot] = 0.0

                # Foot is never in stance
                gaitData.timeStance[foot] = 0.0

                # Foot is always in "swing"
                gaitData.timeSwing[foot] = 1000

    def printGaitInfo(self):
        self.printIter += 1

        if self.printIter == self.printNum:
            print('[GAIT SCHEDULER] Printing Gait Info...')
            print('Gait Type', self.gaitData.gaitName)
            print('---------------------------------------------------------')
            print('Enabled:', self.gaitData.gaitEnabled[0], 
                '|', self.gaitData.gaitEnabled[1])
            print('Period Time', self.gaitData.periodTime[0], 
                '|', self.gaitData.periodTime[1])
            print('Contact State:', self.gaitData.contactStateScheduled[0], 
                '|', self.gaitData.contactStateScheduled[1])
            print('Phase Variable:', self.gaitData.phaseVariable[0], 
                '|', self.gaitData.phaseVariable[1])
            print('Stance Time Remaining:', self.gaitData.timeStanceRemaining[0], 
                '|', self.gaitData.timeStanceRemaining[1])
            print('Swing Time Remaining:', self.gaitData.timeSwingRemaining[0], 
                '|', self.gaitData.timeSwingRemaining[1])
            print('nextGait', self.gaitData._nextGait)

            self.printIter = 0

def main():
    data = GaitData()

    print(data._currentGait)
    print(data.contactStateScheduled)

    gaitType = GaitType()

    sch = GaitScheduler(0.01)
    sch.printGaitInfo()

    sch.gaitData._nextGait = gaitType.STATIC_WALK
    sch.createGait()
    
    for idx in range(100):
        sch.step()
        sch.printGaitInfo()


if __name__ == '__main__':
    main()