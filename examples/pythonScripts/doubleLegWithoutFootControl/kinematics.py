
from math import *
import numpy as np

class kinematics:

    def __init__(self):
        self._l1 = 0.4
        self._l2 = 0.4

    def forwardkinematics(self, q):
        l1 = self._l1
        l2 = self._l2
        
        theta1 = q[0]
        theta2 = q[1]

        x_ankle = -l1*sin(theta1) - l2*sin(theta1+theta2)
        z_ankle = -l1*cos(theta1) - l2*cos(theta1+theta2)

        return x_ankle, z_ankle
    

    def inversekinematics(self, p):
        l1 = self._l1
        l2 = self._l2

        x_ankle = p[0]
        z_ankle = p[1]

        m = pow(x_ankle,2) + pow(z_ankle,2) - pow(l1,2) - pow(l2,2)
        n = 2*l1*l2
        theta2 = acos(m/n)
        
        m = -l1*x_ankle - l2*x_ankle*cos(theta2) + l2*z_ankle*sin(theta2)
        n = pow(x_ankle,2) + pow(z_ankle,2)
        theta1 = asin(m/n)

        return theta1, theta2

    def getJacobian(self, q):
        l1 = self._l1
        l2 = self._l2

        theta1 = q[0]
        theta2 = q[1]

        j11 = -l1*cos(theta1) - l2*cos(theta1 + theta2)
        j12 = -l2*cos(theta1 + theta2)
        j21 = l1*sin(theta1) + l2*sin(theta1 + theta2)
        j22 = l2*sin(theta1 + theta2)

        J = np.zeros((2,2))
        J[0,0] = j11
        J[0,1] = j12
        J[1,0] = j21
        J[1,1] = j22

        return J



def main():
    kk = kinematics()
    print('test forwardkinematics')
    q = [-pi/4, pi/2]
    p = kk.forwardkinematics(q)
    print('output is ', p)

    print('test inversekinematics')
    p = [0, -0.4*sqrt(2)]
    q = kk.inversekinematics(p)
    print('output is ', q)

    



if __name__ == '__main__':
    main()




