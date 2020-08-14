## 使用coppelia的api接口



### 接口文件

要使用coppelia的api接口，首先对应不用的系统将接口库拷贝到本地。

+++

对于windows系统来说，是remoteApi.dll、sim.py和simConst.py三个文件；

对于linux系统来说，是remoteApi.so、sim.py和simConst.py三个文件。

+++



### 接口编程

考虑到使用python编制每个仿真步长（例如0.01s）中的控制算法，通常运算时间就会超过仿真步长设定的时限，因此需要使用同步模式，使仿真服务器在每次仿真步骤之前等待控制算法计算完毕。



1、首先使用19997端口连接coppelia。



19997端口是coppelia软件默认打开的端口，只有连接此端口，才能远程启动和控制仿真。

```
# 每隔0.2s检测一次，直到连接上V-rep
while True:
    clientID = vrep.simxStart('127.0.0.1', 19997, True, True, 5000, 5)
    if clientID > -1:
        break
    else:
        time.sleep(0.2)
        print("Failed connecting to remote API server!")
print("Connection success!")
```



2、设置同步模式和仿真步长

为了让设置仿真步长的指令有效，需要首先在coppelia软件界面中选择定制步长

dt=**(custom)

在python指令运行后，将会发现其中的**变成了我们设定的步长。



执行的程序为

```
# 然后打开同步模式
vrep.simxSynchronous(clientID, True) 

# 设置仿真步长，为了保持API端与V-rep端相同步长
vrep.simxSetFloatingParameter(clientID, vrep.sim_floatparam_simulation_time_step, tstep, vrep.simx_opmode_blocking)
```



3、获取感兴趣的句柄

在仿真过程中，我们希望得到一些状态信息。这些信息通过对象句柄得到，因此在开启仿真之前，需要首先得到这些句柄。

```
baseName = 'Body'
jointName = 'hip_joint'

_, baseHandle = vrep.simxGetObjectHandle(clientID, baseName, vrep.simx_opmode_blocking)
_, jointHandle = vrep.simxGetObjectHandle(clientID, jointName, vrep.simx_opmode_blocking)

print('Handles available!')
```



4、启动仿真

```
# 启动仿真
vrep.simxStartSimulation(clientID, vrep.simx_opmode_oneshot)
```



5、执行控制循环

在一个while循环中，首先读取状态，然后执行计算，最后输出指令。

在输出指令之后，需要让仿真运行一步

```
vrep.simxSynchronousTrigger(clientID)  # 进行下一步
```



