# 第1章 制导系统与工程任务流程

> 本章从“软件每个周期拿到什么、算出什么、交给谁”讲起。示例用于一般飞行器轨迹跟踪、移动平台会合等教学任务。约定 SI 单位；内部角度统一用 rad。

## 1.1 三个模块不要混为一谈

- **导航/状态估计**：将传感器与外部信息整理为带时间戳的状态估计，例如位置、速度、姿态及其质量标志。
- **制导**：根据任务目标和估计状态生成期望运动量，例如加速度、速度方向或轨迹参考。
- **飞行控制**：把期望运动量变成舵面、推力或电机等执行机构命令，并处理执行机构动态与约束。

工程接口要写清参考系、单位、时间戳、有效标志和饱和状态。制导器不应把“无效/过期数据”当作正常测量继续计算。

## 1.2 离散周期中的信号流

数字计算机每隔采样周期 $T_s$ 执行一次。制导器输入的是时刻 $t_k$ 的状态估计，输出的是在接下来一段时间内保持或更新的指令。若周期抖动明显，不能一律用名义 $T_s$ 做差分；应使用实际时间戳差 $\Delta t=t_k-t_{k-1}$，并对异常 $\Delta t$ 进行拒收或降级。

一个通用闭环可写为

\[
\hat{x}_k=\mathcal{E}(y_{0:k}),\quad
u_k=\pi(\hat{x}_k,r_k),\quad
u^{\rm safe}_k=\operatorname{limit}(u_k),\quad
u^{\rm plant}_k=\mathcal{C}(u^{\rm safe}_k).
\]

这里 $y$ 是测量，$\hat{x}$ 是估计状态，$r$ 是任务参考，$\pi$ 是制导律，`limit` 是指令幅值/速率保护，$\mathcal C$ 是飞行控制。限幅不能掩盖不可达命令：应同时记录原始命令、限幅后命令、限幅标志与误差。

## 1.3 先定接口，再写算法

推荐每个周期使用明确的数据包：

| 字段 | 含义 | 必须约定 |
|---|---|---|
| `time_s` | 状态时间戳 | 秒；单调递增 |
| `pos_m[3]` | 位置 | 参考系、轴序、m |
| `vel_mps[3]` | 速度 | 同一参考系、m/s |
| `valid` | 数据有效 | 无效时采取何种降级 |
| `quality` | 估计质量 | 范围与阈值 |
| `cmd` | 期望加速度等 | 坐标系、单位、幅值和速率限制 |

初期建议以单一右手惯性直角系做教学模型。后续引入地球曲率、地球自转或高空环境时，再扩展地固/惯性变换，不能悄悄更换坐标意义。

## 1.4 C 实现：周期接口、数据有效性与命令限幅

下面示例可保存为 `ch1.c`，使用 `cc -std=c99 -Wall -Wextra -pedantic ch1.c -lm -o ch1` 编译运行。这里演示通用的期望加速度接口保护，不是某种具体飞行器控制律。

```c
#include <math.h>      /* isfinite、sqrt 等数学函数 */
#include <stdbool.h>   /* bool 类型 */
#include <stdio.h>     /* printf */

typedef struct {       /* 三维向量：本例统一采用同一右手坐标系 */
    double x, y, z;    /* 分量的单位由字段名或接口约定说明 */
} Vec3;

typedef struct {       /* 导航/估计输出的数据包 */
    double time_s;     /* 状态对应的时间戳，单位 s */
    Vec3 pos_m;        /* 位置，单位 m */
    Vec3 vel_mps;      /* 速度，单位 m/s */
    bool valid;        /* 上游明确置位的数据有效标志 */
} State;

typedef struct {       /* 制导模块输出及保护状态 */
    Vec3 accel_mps2;   /* 期望加速度，单位 m/s^2 */
    bool saturated;    /* 本周期是否发生幅值限幅 */
} GuidanceOutput;

static bool finite_vec(Vec3 v) { /* 拒绝 NaN 或无穷输入 */
    return isfinite(v.x) && isfinite(v.y) && isfinite(v.z);
}

static GuidanceOutput limit_accel(Vec3 a, double max_mps2) {
    GuidanceOutput out = {a, false};            /* 默认不改变输入 */
    double mag = hypot(hypot(a.x, a.y), a.z);     /* hypot 避免平方和溢出或下溢 */
    if (!isfinite(mag) || !isfinite(max_mps2) || max_mps2 <= 0.0) {
        out.accel_mps2 = (Vec3){0.0, 0.0, 0.0}; /* 参数非法时给安全默认值 */
        out.saturated = true;                    /* 同时报告保护触发 */
        return out;
    }
    if (mag > max_mps2) {                        /* 只在超过模长上限时缩放 */
        double scale = max_mps2 / mag;            /* 保持方向，缩小幅值 */
        out.accel_mps2.x *= scale;
        out.accel_mps2.y *= scale;
        out.accel_mps2.z *= scale;
        out.saturated = true;
    }
    return out;
}

int main(void) {
    State s = {1.0, {100.0, 0.0, 20.0}, {10.0, 1.0, 0.0}, true};
    Vec3 raw_cmd = {12.0, -6.0, 0.0};            /* 示例指令，单位 m/s^2 */
    if (!s.valid || !isfinite(s.time_s) || !finite_vec(s.pos_m) || !finite_vec(s.vel_mps)) {
        puts("状态无效：本周期不运行制导计算");
        return 1;
    }
    GuidanceOutput y = limit_accel(raw_cmd, 8.0); /* 对指令作幅值保护 */
    printf("a=(%.3f, %.3f, %.3f), saturated=%d\n",
           y.accel_mps2.x, y.accel_mps2.y, y.accel_mps2.z, y.saturated);
    return 0;
}
```

注意：真实模块还需处理状态过期、坐标系标识、命令速率、执行机构反馈和故障降级。上述限幅在向量模上限下保持三轴指令比例；若各轴具有不同边界，必须采用逐轴或约束分配方式，二者不是同一个限幅定义。

## 1.5 常见故障与排查步骤

| 现象 | 常见原因 | 按顺序排查 |
|---|---|---|
| 指令突然为零 | `valid=false`、NaN 检测触发、保护逻辑默认值 | 打印状态有效位、每个输入及保护原因码；定位第一个非有限数 |
| 命令长期贴着上限 | 任务不可达、单位错误、限幅过低或反馈方向错误 | 对照限幅前后命令；核实单位；在简单初值下检查误差符号 |
| 仿真每步结果不一样 | 未固定初始状态/随机种子，或使用未初始化内存 | 记录完整输入快照；清零状态；固定随机种子 |
| 首周期出现巨大导数 | 差分前没有有效上一时刻，或时间戳回退 | 初始化标志单独管理；检查 `dt>0` 且处于允许范围 |
| 离线仿真正常、实机异常 | 采样周期、延迟、坐标约定、数据有效语义不一致 | 记录时间戳和输入输出；重放原始日志；逐接口对照单位/坐标系 |

**最笨但有效的排错法**：每周期记录“时间戳、输入状态、原始制导命令、限幅后命令、保护标志、下游反馈”。先找到第一次异常的周期，再从该周期向前检查，不要只盯着最终偏差。

## 1.6 本章实验

加入可调采样周期、延迟、无效数据注入和加速度上限。分别观察数据无效时是否继续输出、限幅触发情况，以及采样变慢时闭环响应的变化。实验应展示保护标志和限幅前后曲线，而不只是轨迹图。

---
