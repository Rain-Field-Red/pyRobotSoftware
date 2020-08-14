
clear
clc

syms l1 l2 real
syms theta1 theta2 real

x_b = l1*cos(theta1)
z_b = -l1*sin(theta1)

x_c = x_b - l2*cos(theta2-theta1)
z_c = z_b - l2*sin(theta2-theta1)

% 腿长只与膝关节有关
f1 = x_c*x_c + z_c*z_c
f1 = simplify(f1)

syms xc zc real
ff = xc*xc + zc*zc - f1
th2 = solve(ff,theta2)

c2 = cos(th2)

% 简化并带入c2，求theta1
f2 = x_c*cos(theta1) - z_c*sin(theta1)
f2 = simplify(f2)

ff = xc*cos(theta1) - zc*sin(theta1) - l1 + l2*c2
th1 = solve(ff(1), theta1)


