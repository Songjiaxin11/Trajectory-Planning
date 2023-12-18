# 四轮全向底盘轨迹规划

需要解决的问题：

1. 代码似乎有错误，没有约束到轨迹结束的速度与加速度
2. 轨迹计算使用实际的速度，需要修改车辆底盘代码，实现物理速度的跟踪
3. 如何在C++平台实现复杂的矩阵运算，或如何实现轨迹的存储

# 代码问题

- 从轨迹输出曲线看，轨迹结束速度与结束加速度并不是设置的 $v_t$ 与 $a_t$ 。

  原因出在了函数 ` polys_vals, polys_d_vals, polys_dd_vals` 的如下代码段：

  ```python
  t = np.arange(t1, t2, dt)
  ```

  上述插值代码，只能保证包含 `t1`， 不能保证包含 `t2`。这会导致当前代码存在一个问题：

  生成的速度或位置序列不是与控制频率严格对应的，在路径点前（也就是两端轨迹拼接处）输出的速度或位置序列的时间间隔是有可能小于控制间隔的。

- 从轨迹输出曲线看，轨迹速度不平滑，突变严重，如下图所示。

  ![Traj](F:\default\桌面\Traj.png)

在整段路径不变的情况下，减小路径点的个数，只保留关键路径点，得到的速度曲线将变得平滑，效果如下图。

![Traj_smooth](F:\default\桌面\Traj_smooth.png)

原因出在了如下代码段：

```python
    def arrangeT(self):
        # 计算每个路径点的时间戳

        dx = np.diff(self.path)

        distance = np.sum(np.sqrt(np.sum(dx ** 2, axis=0)))
        self.T = distance / self.arvSpeed

        self.ts = [0]
        for i in range(self.path.shape[1] - 1):
            self.ts.append(self.ts[i] + np.sqrt(np.sum(dx[:, i] ** 2)) / self.arvSpeed)

        # # debug
        # if self.ts[-1] == self.T:
        #     print('T is right')

        self.ts = np.array(self.ts)
        # np.savetxt('ts.txt', self.ts, fmt='%.4f', delimiter=',')
```

由于路径点的时间戳是以全段轨迹均为匀速直线运动设置，而实际上车辆运动要求在路径开始与结束阶段存在平滑的速度变化，所以相应的在前几个与最后几个路径点的时间戳不能使用速度直线运动的假设进行配置。

解决方法应该也不难，目前想到的方式和我们之前代码的三段式轨迹规划原理一样，我一开始求取时间戳时假设速度曲线是一个梯形，根据允许的加速度确定梯形斜率，从而求取出新的路径点的时间戳。

****

# 解决方案

## arrangeT函数

- 取起点 -> 终点，计算总路程，由输入的加速度限幅得到梯形 v-t 曲线，推导得到各路径点的时间。

## polys_vals函数

- 微调路径点时间，原来采样点时间为 t0, t1 (t1为分界点，即上一段多项式的结束点和上一段多项式的起始点), 原来 t1-t0 不保证结果为 dt, 即控制频率。故修正 t1 为 t0 + dt, 可以保证所有得到的控制点时间间隔均为 dt。
