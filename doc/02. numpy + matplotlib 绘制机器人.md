# 02 numpy + matplotlib 绘制机器人

## numpy

支持高阶大量的维度数组与矩阵运算，此外也针对数组运算提供大量的数学函数库

```python
import numpy as np
arr = np.array(...)
arr.shape
np.matmul()
基本的加减乘除
---
np.arange()
np.linspace()
```

## matplotlib

绘图库。它提供了一个面向对象的API，用于使用通用GUI工具包将绘图嵌入到应用程序中。设计与MATLAB非常类似

```python
import matplotlib.pyplot as plt
def plot_vehicle(x, y, theta, x_traj, y_traj):
    p1_i = np.array([0.5, 0, 1]).T
    p2_i = np.array([-0.5, 0.25, 1]).T
    p3_i = np.array([-0.5, -0.25, 1]).T

    T = transformation_matrix(x, y, theta)
    p1 = np.matmul(T, p1_i)
    p2 = np.matmul(T, p2_i)
    p3 = np.matmul(T, p3_i)

    plt.plot([p1[0], p2[0]], [p1[1], p2[1]], 'k-')
    plt.plot([p2[0], p3[0]], [p2[1], p3[1]], 'k-')
    plt.plot([p3[0], p1[0]], [p3[1], p1[1]], 'k-')

    plt.plot(x_traj, y_traj, 'b--')

    plt.xlim(0, 20)
    plt.ylim(0, 20)

    plt.pause(dt)
```

## json

一种轻量级资料交换格式。其内容由属性和值所组成，因此也有易于阅读和处理的优势。

```python
import json
## sender
vel = {
    "x":0.0, # m/s
    "w":0.0, # rad/s
}
data = json.dumps(vel)
print(data)
sock.sendto(data.encode('utf-8'), _ENDPOINT)

## receiver
data, addr = sock.recvfrom(65535)
data = data.decode()
vel = json.loads(data)
print(vel["x"],vel["w"])
```

## 其他

```python
if __name__ == "__main__":
    pass
```

```python
import threading
def f(a,b):
    count = 0
    while True:
        print(count)
        count += 1
t = threading.Thread(target=f)
t.start()
---
t.join()
```

```python
vel = ...
def f():
    global vel
    vel = ...
def g():
    print(vel)
```

---

## 参考架构

* sender：使用pynput直接监听键盘，回调函数中直接进行速度的udp下发
* receiver
  * 开一个线程进行udp接收，将接收的vel存储为全局变量（成员变量）
  * 主线程定时的调用matplotlib完成绘制
* matplotlib可能存在的问题
  * matplotlib必须在主线程中绘制，不能另开thread
  * 使用pause显示的窗口存在抢占focus的情况，退出比较麻烦，所以推荐没做快捷方式退出的情况下pause时间久一点，方便ctrl+c

---

变换矩阵

$R=\begin{bmatrix}cos(\theta) & -sin(\theta) & 0\\sin(\theta) & cos(\theta) & 0\\ 0&0&1\end{bmatrix}$

```python
import threading
import socket
vel = {
    "x": 0.0,
    "w": 0.0
}
pos = {
    "x": 0.0,
    "y": 0.0,
    "theta": 0.0
}
def receive():
    global vel
    while True:
        data, addr = sock.recvfrom(65535)
        data = data.decode()
        vel = json.loads(data)

if __name__ == "__main__":
    global vel
    t = threading.Thread(target=receive)
    t.start()
    while True:
        pos = calc(pos,vel,dt)
        draw(pos,traj,pause_t)
```