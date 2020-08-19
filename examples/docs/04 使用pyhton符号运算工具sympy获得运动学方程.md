# 使用python符号运算工具sympy获得运动学方程



## 1、安装sympy

执行

```
conda install sympy
```



## 2、符号和表达式

```
from sympy import symbols

x, y = symbols('x y')
expr = x + 2*y
```



## 3、表达式简化

```
from sympy import simplify
from sympy import trigsimp, sin, cos
from sympy import powsimp

simplify(expr)
# 三角简化
trigsimp(expr)
# 指数简化
powsimp(expr)
```



## 4、方程求解

```
from sympy import solve, linsolve, Eq

# 对一个方程求解，使用solve
solve(Eq(2*x-1, 3), x)

# 对多个方程求解，使用linsolve
linsolve([x+2*y-5,2*x+y-1], (x,y))
```



## 5、赋值

```
expr.subs(x, y)
expr.subs[(x,1),(y,2)]
```



## 6、运算过程



定义腿部自然垂下为初始位形

![](.\images\kinematic.png)





（1）计算踝关节位置后，计算腿长L

（2）可以发现腿长仅与膝关节角度有关，因此当已知踝关节目标位置（xd，zd）可以计算出膝关节（theta2）

（3）计算踝关节位置的两种变换（x_ankle * sin(theta1) + z_ankle * cos(theta1)）和（x_ankle * cos(theta1) - z_ankle * sin(theta1)）

（4）上述两种变换与髋关节有关，确切的说，是于髋关节的正弦s1和余弦c1有关。

（5）当已知踝关节位置（xd，zd）和膝关节角度theta2后，可以计算出s1和c1

（6）根据前面的结果，可知有两组解，选择第一个膝关节的解和asin(s1)作为髋关节的解。

（7）使用sympy的diff函数，计算足端位置对两个角度的偏微分，构造雅克比矩阵



## 7、运算结果

（1）正运动学
$$
x = -l1*sin(\theta_1) - l2*sin(\theta_1+\theta_2)
\\
z = -l1*cos(\theta_1) - l2*cos(\theta_1+\theta_2)
$$


（2）逆运动学

选择膝关节角度大于0，髋关节角度接近0的一组解。
$$
\theta_2 = acos(\frac{xd^2+zd^2-l1^2-l2^2}{2*l1*l2} )
\\
\theta_1 = asin(\frac{-l1*xd-l2*xd*cos(\theta_2)+l2*zd*sin(\theta_2)}{xd^2+zd^2})
\\
$$



（3）雅克比矩阵
$$
J = \left[
\begin{matrix}

-l1*cos(\theta_1) - l2*cos(\theta_1 + \theta_2) & -l2*cos(\theta_1 + \theta_2)
\\ l1*sin(\theta_1) + l2*sin(\theta_1 + \theta_2)
& l2*sin(\theta_1 + \theta_2)

\end{matrix}
\right]
$$
