
# by eric
# 2018-08-16
# inspired by cheetah software

import numpy as np
import matplotlib.pyplot as plt
from math import pi, sin

def circular(y0, yf, x):
    if (x<0) | (x>1):
        print('phase out of range')

    y = y0 + (yf - y0) * (sin((x - 0.5)*pi) + 1)/2

    return y


class FootSwingTrajectory:
    def __init__(self):
        self._p0 = np.zeros(3)
        self._pf = np.zeros(3)
        self._p = np.zeros(3)

        self._height = 0

    def setInitialPosition(self, p0):
        self._p0 = p0

    def setFinalPosition(self, pf):
        self._pf = pf

    def setHeight(self, h):
        self._height = h

    def computeSwingTrajectoryCircular(self, phase):
        
        # x轨迹
        self._p[0] = circular(self._p0[0], self._pf[0], phase)

        # y轨迹
        self._p[1] = circular(self._p0[1], self._pf[1], phase)

        # 高度轨迹
        if phase < 0.5:
            zp = circular(self._p0[2], (self._p0[2]+self._height), 2*phase)
        else:
            zp = circular((self._p0[2]+self._height), self._pf[2], (2*phase-1))
        
        self._p[2] = zp

        


def main():

    """
    test circular()
    """
    y0 = 0
    yf = 1

    t = []
    y = []
    for x in range(10):
        t.append(0.1*x)
        y.append(circular(y0,yf,(0.1*x)))

    plt.figure(1)
    plt.plot(t,y)
    plt.show()

    """
    test FootSwingTrajectory
    """
    fsTra = FootSwingTrajectory()
    tmp1 = np.zeros(3)
    tmp2 = np.zeros(3)

    tmp1[0] = 0
    tmp1[1] = 0.5
    tmp1[2] = 0
    fsTra.setInitialPosition(tmp1)
    print(fsTra._p0)

    tmp2[0] = 1
    tmp2[1] = 0.5
    tmp2[2] = -0.1
    fsTra.setFinalPosition(tmp2)
    
    fsTra.setHeight(0.2)

    print(fsTra._p0)
    print(fsTra._pf)
    print(fsTra._height)

    tt = []
    xx = []
    yy = []
    zz = []
    for idx in range(10):
        phase = 0.1*idx
        fsTra.computeSwingTrajectoryCircular(phase)

        tt.append(phase)
        xx.append(fsTra._p[0])
        yy.append(fsTra._p[1])
        zz.append(fsTra._p[2])
    
    plt.figure(2)
    plt.plot(tt,xx)
    plt.figure(3)
    plt.plot(tt,yy)
    plt.figure(4)
    plt.plot(tt,zz)
    plt.show()


if __name__ == '__main__':
    main()