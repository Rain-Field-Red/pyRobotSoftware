
from sympy import symbols
from sympy import sin, cos, asin, acos
from sympy import simplify
from sympy import solve, Eq
from sympy import linsolve
from sympy import pi

from sympy import diff

from math import sqrt
#from math import pi

"""
运动学正解
"""
l1, l2 = symbols('l1 l2')
theta1, theta2 = symbols('theta1 theta2')

# 髋关节位置
x_hip = 0
z_hip = 0

p_hip = [x_hip, z_hip]
print('p_hip=', p_hip)

# 膝关节位置
x_knee = x_hip - l1*sin(theta1)
z_knee = z_hip - l1*cos(theta1)

p_knee = [x_knee, z_knee]
print('p_knee=', p_knee)

# 踝关节位置
x_ankle = x_knee - l2*sin(theta1 + theta2)
z_ankle = z_knee - l2*cos(theta1 + theta2)

p_ankle = [x_ankle, z_ankle]
print('p_ankle=', p_ankle)


"""
运动学逆解
"""
# 计算腿长
L = x_ankle**2 + z_ankle**2
print('L=', L)
LL = simplify(L)
print('LL=', LL)
print('leg length depends only on theta2')

# 已知期望位置求膝关节角度
xd, zd = symbols('xd zd')
th2 = solve(Eq((xd**2+zd**2), LL), theta2)

print('theta2=', th2)

# 代入l1,l2,xd,zd，计算th2[0]
tt = th2[0].subs([(l1,0.4), (l2,0.4), (xd,0), (zd, -0.4*sqrt(2))])
print('test first theta2 = ', tt)
tt = th2[1].subs([(l1,0.4), (l2,0.4), (xd,0), (zd, -0.4*sqrt(2))])
print('test second theta2 = ', tt)

# 
T = x_ankle*sin(theta1) + z_ankle*cos(theta1)
print('T=', T)
TT =simplify(T)
print('TT=', TT)
D = x_ankle*cos(theta1) - z_ankle*sin(theta1)
print('D=', D)
DD = simplify(D)
print('DD=', DD)
print('when theta2 is known, sin(theta1) and cos(theta1) can be calculated')

# 已知期望位置求髋关节角度
s1, c1 = symbols('s1 c1')
res = linsolve([xd*s1 + zd*c1 - TT, xd*c1 - zd*s1 - DD], (s1, c1))
res_list = list(res)
print('sin(theta1) and cos(theta1) is', res_list)

# 代入l1,l2,xd,zd，计算s1,c1
ss1 = res_list[0][0]
sss1 = ss1.subs([(l1,0.4), (l2,0.4), (xd,0), (zd, -0.4*sqrt(2))])
print('test sin(theta1) = ', sss1)
cc1 = res_list[0][1]
ccc1 = cc1.subs([(l1,0.4), (l2,0.4), (xd,0), (zd, -0.4*sqrt(2))])
print('test cos(theta1) = ', ccc1)

# 从多解中选择
print('select theta2 =', th2[1])
print('select theta1 =', asin(ss1))


"""
雅克比矩阵
"""
j11 = diff(x_ankle, theta1)
j12 = diff(x_ankle, theta2)
j21 = diff(z_ankle, theta1)
j22 = diff(z_ankle, theta2)

print('j11 = ', j11)
print('j12 = ', j12)
print('j21 = ', j21)
print('j22 = ', j22)



