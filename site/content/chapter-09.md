## 第9章 多主体、学习方法与 C 工程实现

> 本章把前面各章的制导/控制概念放到软件工程边界中：多个自主体如何交换有限信息，学习算法如何离线训练并安全接入运行时，以及一个周期任务如何把输入检查、单位转换、数值保护、限幅和日志组织起来。示例只用于一般移动体的安全航迹跟踪与协同，不代表特定飞行器设计或任务性能。

### 9.1 多主体协同：先定义信息与责任

“多主体”不是把多个单机控制器简单复制。实现前应明确每个节点知道什么、通过什么链路收到、信息有多旧，以及失联后采用什么行为。把通信图记作 \(\mathcal G=(\mathcal V,\mathcal E)\)，节点 \(i\) 的邻居集合为 \(\mathcal N_i\)。一个基础一致性量可写成

\[
\dot z_i = -k\sum_{j\in\mathcal N_i} a_{ij}(z_i-z_j), \quad k>0,
\]

其中 \(z_i\) 可以是某个共享的队形参考或时间基准，\(a_{ij}\ge0\) 表示通信权重。这个方程说明相邻节点通过“自己与邻居的差”调整状态；它不自动解决避碰、动力学可达性、通信延迟或安全约束。若网络不连通，所有节点也不一定收敛到同一值。

工程上常用分层：上层只发布经校验的参考量和有效期；单机层负责本地状态估计、约束检查与控制；通信层负责序号、时间戳、超时和完整性。禁止把“收到了队友数据”当成“数据仍然有效”。

### 9.2 学习制导：训练、验证、部署分开

强化学习策略可写成 \(u=\pi_\theta(o)\)，其中 \(o\) 是观测，\(u\) 是策略输出，\(\theta\) 是离线训练得到的参数。在线 C 运行时通常只执行固定网络的前向计算；数据采集、反向传播和参数更新应与硬实时周期解耦，除非系统经过专门的时序、安全和资源验证。

部署前固定观测顺序、归一化均值/方差、动作缩放、裁剪范围、网络层维度、权重精度和版本号。归一化参数属于模型的一部分，遗漏它们会导致“网络结构正确但行为错误”。学习策略输出只能视为候选命令：先做有限值检查、量纲转换、约束投影或安全监控，再交给下游模块。策略置信度低、输入越界、超时或模型校验失败时，应转入明确的保守模式。

### 9.3 C 周期任务的推荐骨架

将每次调用设计为明确的输入—状态—输出接口。下面示例实现一个通用的二维航迹速度指令模块：输入当前位置和目标点（m），输出期望速度分量（m/s）；程序包含输入有限性检查、死区、速度模长限制、固定周期状态和故障状态。它演示工程接口，不是飞行器动力学模型。

```c
/* gcc -std=c11 -Wall -Wextra -Wpedantic guidance_ch9.c -lm -o guidance_ch9 */
#include <math.h>      /* isfinite、sqrt、hypot */
#include <stdbool.h>   /* bool 类型 */
#include <stdio.h>     /* 示例 main 中的 printf */

/* 所有长度用 m，时间用 s，速度用 m/s；角度若出现统一使用 rad。 */
typedef struct {
    double x_m;        /* 当前东向位置，单位 m */
    double y_m;        /* 当前北向位置，单位 m */
    double target_x_m; /* 目标东向位置，单位 m */
    double target_y_m; /* 目标北向位置，单位 m */
} GuidanceInput;

typedef struct {
    double vx_mps;     /* 输出东向期望速度，单位 m/s */
    double vy_mps;     /* 输出北向期望速度，单位 m/s */
    bool valid;        /* false 表示输出不可用于下游控制 */
} GuidanceOutput;

typedef struct {
    double max_speed_mps; /* 速度模长上限，单位 m/s */
    double deadband_m;    /* 目标点附近不再追赶的半径，单位 m */
} GuidanceConfig;

/* 每周期调用一次。cfg 必须在启动时验证且运行中保持只读。 */
static GuidanceOutput guidance_step(const GuidanceInput *in,
                                    const GuidanceConfig *cfg)
{
    GuidanceOutput out = {0.0, 0.0, false}; /* 故障时默认输出零且无效 */

    /* 先检查指针，防止解引用空地址。 */
    if (in == NULL || cfg == NULL) {
        return out;
    }

    /* 拒绝 NaN/Inf；直接参与后续运算会污染整条控制链。 */
    if (!isfinite(in->x_m) || !isfinite(in->y_m) ||
        !isfinite(in->target_x_m) || !isfinite(in->target_y_m) ||
        !isfinite(cfg->max_speed_mps) || !isfinite(cfg->deadband_m)) {
        return out;
    }

    /* 配置必须是正的速度上限和非负死区。 */
    if (cfg->max_speed_mps <= 0.0 || cfg->deadband_m < 0.0) {
        return out;
    }

    /* 计算“目标减当前位置”，因此向量指向目标。 */
    const double dx_m = in->target_x_m - in->x_m;
    const double dy_m = in->target_y_m - in->y_m;
    const double distance_m = hypot(dx_m, dy_m); /* hypot 比 sqrt(dx*dx+dy*dy) 更稳健 */

    /* 如果已经到达死区内，输出零速度，避免在目标附近来回抖动。 */
    if (distance_m <= cfg->deadband_m) {
        out.valid = true;
        return out;
    }

    /* 归一化方向向量；distance 已大于死区。 */
    const double ux = dx_m / distance_m;
    const double uy = dy_m / distance_m;

    /* 先按配置速度生成期望值，再显式限制向量模长。 */
    out.vx_mps = cfg->max_speed_mps * ux;
    out.vy_mps = cfg->max_speed_mps * uy;
    const double speed_mps = hypot(out.vx_mps, out.vy_mps);
    if (!isfinite(speed_mps) || speed_mps <= 0.0) {
        out.vx_mps = 0.0;
        out.vy_mps = 0.0;
        return out;
    }
    if (speed_mps > cfg->max_speed_mps) {
        const double scale = cfg->max_speed_mps / speed_mps;
        out.vx_mps *= scale;
        out.vy_mps *= scale;
    }

    /* 只有全部计算通过后才置为有效。 */
    out.valid = isfinite(out.vx_mps) && isfinite(out.vy_mps);
    if (!out.valid) {
        out.vx_mps = 0.0;
        out.vy_mps = 0.0;
    }
    return out;
}

int main(void)
{
    const GuidanceInput input = {0.0, 0.0, 100.0, 50.0};
    const GuidanceConfig config = {12.0, 2.0};
    const GuidanceOutput output = guidance_step(&input, &config);
    printf("valid=%d vx=%.3f m/s vy=%.3f m/s\n",
           output.valid ? 1 : 0, output.vx_mps, output.vy_mps);
    return output.valid ? 0 : 1;
}
```

### 9.4 单位、采样与数值保护

- **内部单位统一**：长度 m、速度 m/s、加速度 m/s²、时间 s、角度 rad。输入输出边界处集中做换算，不要在公式中混用 degree 和 radian。
- **周期时间显式给出**：固定周期任务使用配置的 \(T_s\)，避免通过两次系统时钟相减得到不受控的采样间隔。若需要实测 \(\Delta t\)，必须检查其正值和上下界。
- **角度环绕要特殊处理**：角误差用 \(e_\psi=\operatorname{wrap}_{[-\pi,\pi)}(\psi_d-\psi)\)，不要直接相减后跨越 \(2\pi\) 边界。
- **防除零与越界**：除法前检查分母；反三角函数输入限制到有效区间；查表索引先夹紧或转入明确的表外策略；不能靠 NaN 传播后再期待下游修复。
- **饱和顺序要记录**：记录原始指令、滤波后指令、幅值限幅后指令、速率限制后指令和实际执行反馈，方便分辨问题出在哪一层。
- **确定性与可追溯**：日志中包含单调时钟时间戳、周期号、输入状态、模型版本、参数版本、限幅标志、有效性标志和故障码；不要只打印最终输出。

### 9.5 常见出错现象与排查流程

| 现象 | 常见原因 | 傻瓜式排查顺序 |
|---|---|---|
| 指令方向总是反的 | 相对位置定义反了；坐标轴正方向不一致 | 用“当前位置 (0,0)，目标在 (+10,0)”手算；检查 dx=目标−当前位置；再核对机体系到导航系变换 |
| 输出速度偶尔变成 NaN | 输入 NaN/Inf；除以零；无效查表输出 | 在模块入口打印并检查全部输入；定位第一个非有限值；给除法、平方根和表插值加边界检查 |
| 目标附近左右摆动 | 没有死区；反馈延迟；速度指令变化太猛 | 先关闭噪声和延迟；增大死区观察；再逐步恢复延迟/噪声；最后添加速率限制并核对响应时间 |
| 向量限幅后方向变了 | 分量分别裁剪而非按向量模长统一缩放 | 同时记录限幅前后向量及其模长；若要求保方向，用统一比例系数缩放分量 |
| 偶发超时或周期抖动 | 动态内存、阻塞 I/O、复杂查表/推理、日志输出过多 | 在每个阶段打时间戳；关闭同步打印；测最坏执行时间而非只看平均值；将慢任务移出实时循环 |
| 学习策略上线后表现完全不同 | 归一化、特征顺序、权重版本、动作缩放不一致 | 用同一组固定观测分别跑训练端和 C 端；逐层比对归一化值、层输出与最终动作；检查模型哈希/版本 |
| C 与 Python 仿真趋势不同 | 浮点精度、数组索引、矩阵转置、激活函数或边界处理不同 | 固定输入逐步比对中间量；先比输入张量，再比每层输出；最后比安全过滤后的动作 |
| 多节点出现不同步或队形漂移 | 时间戳基准不同、消息过期、丢包处理不一致 | 打印每条消息发送/接收时间、序号和有效期；注入延迟与丢包；验证超时后是否进入预定安全行为 |
| 目标点附近输出突然变零 | 死区设置过大或距离单位不一致 | 打印距离和死区原始数值及单位；把目标设置在已知 1 m 偏差处核对门限 |

推荐排查顺序始终是：**复现并冻结输入 → 检查单位和坐标 → 检查中间变量 → 检查限幅/滤波/延迟 → 检查数值边界 → 检查调度时间**。每次只改变一个因素，并保存参数、代码版本和日志，避免“调了以后好像好了”但无法复现。

### 9.6 验证清单：先小后大

1. **静态检查**：警告级别最高编译；检查未初始化变量、隐式类型转换、数组边界和浮点异常。
2. **函数级测试**：零误差、已知方向、最大量级、非法指针、NaN/Inf、负配置值及目标恰在死区边缘。
3. **边界测试**：输入刚好位于限值内/外；角度跨越 ±π；周期时间达到允许上下限；表格命中端点和表外点。
4. **差分测试**：相同输入在参考脚本与 C 实现之间逐步比较中间变量，容差需按数据类型和模型误差预先定义。
5. **故障注入**：人为加入消息延迟、丢包、传感器冻结、无效数值、执行机构饱和和计算超时；确认系统按设计报警并退化。
6. **闭环仿真**：加入动力学、测量噪声、执行机构带宽和采样延迟；检查峰值、稳态误差、限幅持续时间与故障恢复。
7. **目标处理器验证**：测量最坏执行时间、堆栈和内存；确认浮点库、编译选项和处理器上的数值行为与仿真一致。

### 9.7 交互实验建议

本章实验设置多个移动节点共享一个参考量，提供通信延迟、丢包率、节点速度上限和学习策略开关。观察一致性误差、消息新鲜度、限幅状态和每周期计算时间。学习策略应与一个固定的传统基线比较，训练过程不放进实时周期；实验面板应明确哪些量是仿真、哪些是通信假设、哪些是安全监控结果。

### 本章小结

算法能运行不等于软件能可靠工作。工程实现必须把单位、坐标、采样、异常输入、约束、模型版本、故障降级和可追溯日志都作为接口的一部分。学习策略可提供候选动作，但其运行边界、验证流程和失效处理仍需由系统设计明确规定。


## 综合作业：机动交通目标场景下的三维安全绕飞

> **任务性质：**民航/无人机交通避碰教学。目标是安全通过航路点并保持与另一架机动航空器的最小间隔；不以接触目标为成功条件。

### 作业目标

把本书的几何、制导、三自由度质点动力学、指令约束、单位处理和 C 实现串成一个闭环。你需要自己设计制导律，使自主飞行器到达指定航路点，同时预测并避免与机动交通目标发生近距冲突，然后平滑地回到航路。

本题不指定唯一制导律。可以比较纯航路点跟踪、基于最近会遇时间的冲突探测、速度障碍法或带安全约束的滚动规划；重点是说明你的方法何时有效、何时失效。

### 场景和约定

采用局部平直 ENU 坐标系：(x) 向东、(y) 向北、(z) 向上，长度 m、时间 s、速度 m/s、加速度 m/s²。所有角度内部使用 rad。两架航空器均用三自由度质点模型：

[
\dot{\mathbf r}=\mathbf v,\qquad
\dot{\mathbf v}=\mathbf a,
\]

其中 (mathbf r=[x,y,z]^T)，(mathbf v=[v_x,v_y,v_z]^T)，(mathbf a) 是惯性坐标系中的合加速度。这个简化模型不包含姿态、气动数据库、推力或真实飞控；加速度上限是教学约束，不能解释成具体机型性能。

| 项目 | 自主飞行器 | 机动交通目标 |
|---|---:|---:|
| 初始位置 | ((0,0,1000)) m | ((3500,-4000,1000)) m |
| 初始速度 | ((90,0,0)) m/s | ((60.62,35,0)) m/s |
| 加速度约束 | (|\mathbf a_c|le 2.5) m/s² | 按下述脚本机动 |
| 任务 | 通过航路点 ((12000,0,1000)) m | 按脚本飞行 |

仿真时域取 160 s，建议步长从 (Delta t=0.05) s 开始，并检查步长减半后结论是否明显变化。目标的横向机动采用已知、确定的测试脚本：(35le t<43) s 时沿水平速度左法向施加 (0.8) m/s²；(43le t<51) s 时施加 (-0.8) m/s²；(80le t<86) s 时施加 (-0.5) m/s²；(86le t<92) s 时施加 (+0.5) m/s²。其他时段加速度为零。左法向定义为 ([-v_y,v_x,0]^T/\sqrt{v_x^2+v_y^2})。若水平速度模过小，脚本应返回零加速度并置有效标志，不能除以零。

该脚本只是可重复的仿真输入，不是目标运动预测模型。自主飞行器可以读取带时间戳的当前位置和速度，但必须自己处理测量延迟、有限采样和状态误差。基础版本先用无噪声同步状态调通，再加入延迟与噪声。

### 要求你完成的工作

1. **建立基准闭环**：实现航路点跟踪，不考虑交通目标；记录位置、速度、指令加速度和航路点误差。
2. **加入冲突探测**：每周期预测一段有限时间窗内的相对运动，报告预测最近距离和最近会遇时间；说明恒速预测在目标机动后为何会失准。
3. **设计安全绕飞制导律**：在满足最小间隔的前提下生成平滑加速度指令；目标通过后重新捕获航路点。必须说明冲突解除条件，避免模式在“绕飞/回航”之间来回抖动。
4. **实现工程约束**：对加速度向量施加模长限制；再加变化率限制或一阶响应模型；检查速度、高度、数值有效性和仿真时域。若命令不可达，应报告饱和标志并在日志中保留限幅前后指令。
5. **完成对照实验**：至少比较“只跟航路点”和“加入安全绕飞”两种情况；再改变目标机动时刻、测量延迟和最小间隔，观察安全性与航路误差的权衡。

### 验收判据

- 全程最近间隔不得小于 (500) m；报告实际最小间隔及其发生时刻。
- 仿真结束前，自主飞行器必须进入航路点半径 (250) m 内，并保持速度 (75)–(105) m/s、航迹方向与东向夹角不超过 (15^circ)。
- 全程满足加速度模长上限；不得出现 NaN/Inf、负时间步或无效状态被当作正常值继续计算。
- 给出时间历程图：相对距离、预测最近距离、航路点误差、三轴加速度、加速度限幅标志。静态参数扫描图不得放时间播放控件。
- 说明在何种初始条件或延迟下任务会变为不可行；不能通过放宽安全间隔或隐去失败样本来宣称成功。

### C99 起步框架

下面程序包含状态定义、目标机动脚本、加速度限幅和三自由度积分。基准制导器只负责向航路点飞行，**故意没有实现避碰律**；你需要在标注的 TODO 处增加状态预测、冲突判别、安全绕飞和回航逻辑。初始版本可以运行，但未完成 TODO 前不满足安全验收条件。

```c
/* 编译：cc -std=c99 -Wall -Wextra -Wpedantic -O2 assignment.c -lm -o assignment */
#include <math.h>
#include <stdbool.h>
#include <stdio.h>

#define DT_S 0.05
#define T_END_S 160.0
#define A_MAX_MPS2 2.5
#define V_WAYPOINT_MPS 90.0

typedef struct { double x, y, z; } Vec3; /* ENU：m */
typedef struct { Vec3 r, v; } Body;      /* 位置 m、速度 m/s */
typedef struct { Vec3 a; bool valid; } AccelCommand; /* 加速度 m/s^2 */

static Vec3 add(Vec3 a, Vec3 b) { return (Vec3){a.x+b.x,a.y+b.y,a.z+b.z}; }
static Vec3 sub(Vec3 a, Vec3 b) { return (Vec3){a.x-b.x,a.y-b.y,a.z-b.z}; }
static Vec3 scale(Vec3 a, double k) { return (Vec3){k*a.x,k*a.y,k*a.z}; }
static double norm(Vec3 a) { return hypot(hypot(a.x,a.y),a.z); }

static Vec3 limit_accel(Vec3 a, double limit)
{
    const double m = norm(a);
    if (!isfinite(m) || !isfinite(limit) || limit <= 0.0) return (Vec3){0,0,0};
    if (m > limit) return scale(a, limit/m); /* 保持方向，限制向量模 */
    return a;
}

/* 已知机动目标脚本：返回确定的测试加速度，不是预测器。 */
static Vec3 traffic_accel(double t_s, Vec3 v)
{
    const double vh = hypot(v.x, v.y); /* 水平速度模，m/s */
    if (!isfinite(t_s) || !isfinite(vh) || vh < 1.0) return (Vec3){0,0,0};

    double lateral = 0.0; /* 左法向加速度，m/s^2 */
    if (t_s >= 35.0 && t_s < 43.0) lateral =  0.8;
    if (t_s >= 43.0 && t_s < 51.0) lateral = -0.8;
    if (t_s >= 80.0 && t_s < 86.0) lateral = -0.5;
    if (t_s >= 86.0 && t_s < 92.0) lateral =  0.5;

    const Vec3 left = {-v.y/vh, v.x/vh, 0.0};
    return scale(left, lateral);
}

/* 基准：只跟航路点，故意忽略交通目标。请在此函数补充安全逻辑。 */
static AccelCommand design_guidance(const Body *own, const Body *traffic,
                                    Vec3 waypoint)
{
    AccelCommand out;
    out.a = (Vec3){0.0,0.0,0.0};
    out.valid = false;
    if (own == NULL || traffic == NULL) return out;
    if (!isfinite(own->r.x) || !isfinite(own->r.y) || !isfinite(own->r.z) ||
        !isfinite(own->v.x) || !isfinite(own->v.y) || !isfinite(own->v.z) ||
        !isfinite(traffic->r.x) || !isfinite(traffic->r.y) ||
        !isfinite(traffic->r.z) || !isfinite(traffic->v.x) ||
        !isfinite(traffic->v.y) || !isfinite(traffic->v.z)) return out;

    /* 简单航路点基线：向航路点生成期望速度，再用比例项生成加速度。 */
    const Vec3 to_wp = sub(waypoint, own->r);
    const double d = norm(to_wp);
    Vec3 a_cmd = {0.0,0.0,0.0};
    if (isfinite(d) && d > 1.0) {
        const Vec3 v_des = scale(to_wp, V_WAYPOINT_MPS/d);
        a_cmd = scale(sub(v_des, own->v), 0.8); /* 教学基线，不含交通避让 */
    }

    /* TODO 1：基于相对位置/速度预测最近会遇距离和时间。 */
    /* TODO 2：若风险触发，生成带滞回的安全绕飞命令。 */
    /* TODO 3：冲突解除后平滑回到航路点跟踪，避免指令突变。 */

    out.a = limit_accel(a_cmd, A_MAX_MPS2); /* 最终施加加速度模长限制 */
    out.valid = isfinite(out.a.x) && isfinite(out.a.y) && isfinite(out.a.z);
    return out;
}

/* 三自由度质点：先更新速度，再用更新后的速度更新位置。 */
static void step_body(Body *b, Vec3 a, double dt_s)
{
    b->v = add(b->v, scale(a, dt_s));
    b->r = add(b->r, scale(b->v, dt_s));
}

int main(void)
{
    Body own;
    own.r = (Vec3){0.0,0.0,1000.0};
    own.v = (Vec3){90.0,0.0,0.0};
    Body traffic;
    traffic.r = (Vec3){3500.0,-4000.0,1000.0};
    traffic.v = (Vec3){60.62,35.0,0.0};
    const Vec3 waypoint = {12000.0,0.0,1000.0};
    double min_sep_m = INFINITY, min_sep_time_s = 0.0;

    for (double t=0.0; t<T_END_S; t+=DT_S) {
        const AccelCommand cmd = design_guidance(&own,&traffic,waypoint);
        if (!cmd.valid) { puts("无效制导输出：终止仿真并检查输入"); return 1; }

        step_body(&own, cmd.a, DT_S);
        step_body(&traffic, traffic_accel(t,traffic.v), DT_S);

        const double sep = norm(sub(own.r,traffic.r));
        if (sep < min_sep_m) { min_sep_m=sep; min_sep_time_s=t+DT_S; }

        if (!isfinite(sep) || !isfinite(own.r.x) || !isfinite(traffic.r.x)) {
            puts("数值异常：检查步长、输入和状态更新顺序");
            return 2;
        }
    }

    printf("最小间隔 %.1f m，发生于 %.2f s\n",min_sep_m,min_sep_time_s);
    printf("航路点剩余距离 %.1f m\n",norm(sub(waypoint,own.r)));
    puts("提示：此起步基线没有避碰逻辑，需完成 TODO 后再检查安全判据。");
    return 0;
}
```

**注意：**这段程序使用半隐式 Euler 积分，仅用于起步。你需要对比步长减半结果；若振荡或约束判定对步长敏感，改用 RK4 或明确处理事件穿越。还要补充速度/高度包线、加速度变化率限制、最小间隔预测及滞回退出；不能只在离散采样点检查距离，因为两步之间也可能发生最近接近。

### 提交内容

1. C99 源码、编译命令和运行说明。
2. 制导逻辑流程图或状态机，解释航路跟踪、冲突预测、绕飞、回航之间的切换条件。
3. 基准与避让方案的对照图和验收表，写出最小间隔、航路点误差、峰值加速度、饱和比例和计算步长。
4. 一页“失败案例”：至少展示一次参数或延迟导致的安全裕度不足，并按本章排错流程定位原因。
5. 简短说明：你选的方法的假设是什么，哪些情形下会失效，下一步如何提高模型可信度。

**评分建议（100 分）：**模型与单位约定 15；C 实现与数值健壮性 20；安全冲突预测和绕飞逻辑 25；平滑限幅与滞回切换 15；实验设计、图表和失败分析 15；复现说明 10。只让一次仿真“看起来绕开了”不算完成，须覆盖多组延迟、机动时刻和初始位置并逐一报告验收结果。
