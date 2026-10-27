# 第8章 改进制导律、最优制导与模型预测控制

本章把最优制导写成多目标泛函优化：既要压低终端脱靶量和落角误差，又要减小控制努力二次指标，还必须满足动压、过载、攻角和舵机约束。这里的 $\int u^2dt$ 不是燃料或机械能；只有加入推力、阻力、质量和速度动力学，才能讨论真实能耗及末端速度。模型预测控制（MPC）在每个采样时刻重复求解受约束的有限时域问题；增强型比例导引（APN）则可理解为在 PN 基础上加入目标机动前馈。

## 本章导航

**前置知识：**第3章的 PN 几何、第6章的执行机构约束和第7章的相对状态估计。读者只需先理解积分代表整段过程的累计代价。

**学习主线：**PN 抑制视线角速度；APN 加入目标机动前馈；ZEM 预测无后续控制时的终端偏差；落角约束制导同时规定终端位置和速度方向；MPC 把动力学、代价与约束放入统一滚动优化。目标是根据任务指标选择方法，而不是把复杂度等同于性能。

## 8.1 从目标函数到有限时域控制

以离散线性系统 \(x_{k+1}=Ax_k+Bu_k\) 为例，定义预测时域 \(N\)，代价
\[
J=\sum_{i=0}^{N-1}(x_i^TQx_i+u_i^TRu_i)+x_N^TQ_fx_N.
\]
\(Q,Q_f\succeq0\) 可惩罚相对位置、视线角速度和终端落角误差，\(R\succ0\) 惩罚控制能量。终端权重很大并不等价于严格满足终端约束；若落角必须满足，应显式加入终端等式/不等式约束并检查可行性。约束可写为 \(x_i\in\mathcal X, u_i\in\mathcal U\)。在当前时刻预测未来状态并求最小代价控制序列，只执行第一个控制量；下一采样时刻获取新状态后再重新优化。这一过程称滚动时域。

三维 PN 写为 $\mathbf a_{PN}=NV_c(\boldsymbol\omega_{LOS}\times\hat{\mathbf r})$。定义 LOS 法平面投影矩阵 $P_\perp=I-\hat{\mathbf r}\hat{\mathbf r}^T$，三维 APN 为 $\mathbf a_c=\mathbf a_{PN}+K_aP_\perp\hat{\mathbf a}_T$。$K_a$ 不是普适常数，它取决于剩余时间、动力学阶次和估计延迟。

在线性、无约束、固定权重条件下，最优控制可化成线性反馈；引入幅值/速率约束后，通常需要解二次规划或其他优化问题。MPC 的核心价值是显式处理约束，不是自动消除模型误差。

## 8.2 常用改进制导律：从 PN 到 ZEM

### 8.2.1 增强型比例导引 APN

PN 只利用闭合速度和视线角速度，对持续机动目标会留下系统性预测误差。APN 在 PN 项之外加入目标法向加速度前馈：

\[
\mathbf a_c=N V_c(\boldsymbol\omega_{LOS}\times\hat{\mathbf r})+K_aP_\perp\hat{\mathbf a}_T.
\]

其中 $\hat a_{T,n}$ 是目标加速度在视线法向上的估计。第一项消除正在形成的视线转动，第二项提前补偿目标机动。APN 的关键并非“多加一个加速度”，而是估计器带宽与前馈增益的匹配：若 $\hat a_{T,n}$ 噪声大或延迟明显，前馈会把噪声直接送入舵机。

### 8.2.2 零控脱靶量 ZEM 制导

三维实现必须先形成终端法平面内的脱靶向量，而不是只计算一个“横向误差”。令预测终端 LOS 为 $\hat{\boldsymbol\ell}_f$，定义

\[
P_{\perp,f}=I-\hat{\boldsymbol\ell}_f\hat{\boldsymbol\ell}_f^T,
\]

\[
\mathbf{ZEM}_\perp=P_{\perp,f}\left(\mathbf r+t_{go}\mathbf v_r+\frac12t_{go}^2\hat{\mathbf a}_r\right),
\qquad
\mathbf a_{ZEM}=-\frac{3}{t_{go}^2}\mathbf{ZEM}_\perp.
\]

该向量在 LOS 法平面内有两个独立分量。下面的标量双积分器只用于推导每个正交分量共同的系数 3；实际程序应在三维中构造投影矩阵并做向量模限幅。

零控脱靶量（zero-effort miss, ZEM）回答一个直接的工程问题：**若从当前时刻起不再施加修正控制，按当前模型传播到终端会偏离多少？** 三维向量式见上；为推导其中系数 3，在法平面任一正交基方向写误差动力学

\[
\dot e_y=e_v,\qquad \dot e_v=u,
\]

剩余飞行时间为 $t_{go}$。若零控制传播采用匀速模型，则

\[
ZEM=e_y+t_{go}e_v.
\]

最小化 $J=\tfrac12\int_t^{t_f}u^2(\tau)\,d\tau$，且只要求 $e_y(t_f)=0$，可得当前时刻的最小能量反馈

\[
u_{ZEM}=-\frac{3}{t_{go}^2}ZEM.
\]

令当前时刻为零、$T=t_{go}$。双积分器传播到终端时

\[
e_y(T)=e_y(0)+Te_v(0)+\int_0^T(T-\tau)u(\tau)d\tau.
\]

前两项正是 ZEM。以拉格朗日乘子最小化控制能量并满足 $e_y(T)=0$，最优控制必与约束核 $(T-\tau)$ 成正比：$u^*(\tau)=-c(T-\tau)$。代回约束可得 $c=3ZEM/T^3$，因此当前指令 $u^*(0)=-3ZEM/T^2$。系数 3 来自 $\int_0^T(T-\tau)^2d\tau=T^3/3$，不是经验参数。

若已估计到未补偿的相对横向加速度 $a_r$，零控传播量可扩展为

\[
ZEM_a=e_y+t_{go}e_v+\frac12a_r t_{go}^2,\qquad
u_{ZEM,a}=-\frac{3}{t_{go}^2}ZEM_a.
\]

ZEM 把“位置误差、速度误差、剩余时间”压缩成一个具有长度单位的预测量。在线性小角度、常速和准确 $t_{go}$ 条件下，它与经典 PN 可以建立等价关系；但加入目标机动、气动迟滞和饱和后，这种等价不再自动成立。

## 8.3 线性最优落角约束制导

### 8.3.1 终端角度怎样进入状态约束

三维落角不是一个平面角，而是期望终端速度方向 $\hat{\mathbf v}_f^*\in\mathbb S^2$。以

\[
P_f=I-\hat{\mathbf v}_f^*\hat{\mathbf v}_f^{*T}
\]

构造终端法平面，并定义 $\mathbf e_p=P_f(\mathbf r-\mathbf r_f^*)$、$\mathbf e_v=P_f(\mathbf v-\mathbf v_f^*)$。三维矢量律为

\[
\mathbf a_{IAC}=-\frac{6}{t_{go}^2}\mathbf e_p-\frac{4}{t_{go}}\mathbf e_v,
\qquad
\mathbf a_{TZC}=-\frac{12}{t_{go}^2}\mathbf e_p-\frac{6}{t_{go}}\mathbf e_v.
\]

以下标量推导是在 $P_f$ 的任一正交基方向上完成；两个方向使用同一 $t_{go}$ 和终端基，重新合成后才是实际三维指令。

在期望碰撞方向附近建立横向误差坐标。$e_y$ 表示相对期望碰撞线的横向位移，$e_v$ 表示相对期望终端横向速度的偏差。小角度近似下，指定终端落角可转化为指定终端横向速度，因此终端条件写成

\[
e_y(t_f)=0,\qquad e_v(t_f)=0.
\]

对双积分器 $\dot e_y=e_v,\ \dot e_v=u$，求解

\[
\min_u\;J=\frac12\int_t^{t_f}u^2(\tau)\,d\tau
\]

并严格满足上述两个终端条件，可得到当前时刻的线性最优角度约束制导律

\[
u_{IAC}=-\frac{6}{t_{go}^2}e_y-\frac{4}{t_{go}}e_v.
\]

终端速度和位置分别满足

\[
e_v(T)=e_v(0)+\int_0^T u(\tau)d\tau,
\]

\[
e_y(T)=e_y(0)+Te_v(0)+\int_0^T(T-\tau)u(\tau)d\tau.
\]

最小控制努力解属于两个约束核 $1$ 和 $(T-\tau)$ 的线性组合，所以 $u(\tau)=c_0+c_1\tau$。代入终端条件得到

\[
Tc_0+\frac{T^2}{2}c_1=-e_v(0),
\]

\[
\frac{T^2}{2}c_0+\frac{T^3}{6}c_1=-e_y(0)-Te_v(0).
\]

由第一式消去 $c_0$，得到 $c_1=12e_y(0)/T^3+6e_v(0)/T^2$，再得 $c_0=-6e_y(0)/T^2-4e_v(0)/T$。所以当前指令 $u(0)=c_0$。将二者回代两条终端方程，残差均为零，完成结果核对。

第一项主要消除预测位置偏差，第二项同时塑造终端速度方向。与只约束脱靶量的 ZEM 律相比，落角约束多消耗一个自由度，因而通常需要更早机动并付出更大的控制能量。

> **工程注记：**这里的 $e_v$ 不是原始横向速度，而是“当前横向速度减去期望终端横向速度”。若直接把 $v_y$ 填入而没有扣除落角对应的终端速度，最终角度会出现固定偏差。

### 8.3.2 末端零控制指令 TZC

有些任务不仅要求零脱靶和指定落角，还希望控制指令在终端趋于零，避免末端瞬时大舵偏。取最低阶二次控制剖面

\[
u(\tau)=c_0+c_1\tau+c_2\tau^2,
\]

并施加 $e_y(t_f)=0$、$e_v(t_f)=0$、$u(t_f)=0$，可解得当前指令

\[
u_{TZC}=-\frac{12}{t_{go}^2}e_y-\frac{6}{t_{go}}e_v.
\]

该式是满足三项终端条件的最低阶多项式反馈，不应与 ZEM 混为一谈：ZEM 的“零控制”描述的是预测过程，TZC 的“末端零控制”描述的是终端指令约束。TZC 往往把控制作用前移，因此更容易在前段触及过载上限；必须与可达性和饱和检查一起使用。

TZC 再增加 $u(T)=0$，控制函数至少需要二次多项式 $u(\tau)=c_0+c_1\tau+c_2\tau^2$，才有三个独立系数满足三项终端条件。将条件联立即得到当前指令中的系数 12 和 6。约束数量决定所需多项式自由度，这是理解这些数字的直接方法。

## 8.4 三维 C 实现：ZEM、落角约束与末端零控制

完整三维实现见 [`examples/guidance_3d.c`](https://github.com/PaoShou-China/guidance-handbook/blob/main/examples/guidance_3d.c)：它以向量投影构造两个法向自由度，并统一进行向量模限幅。下面程序只用于核对任一法平面基方向上的闭式系数，不是最终制导接口。

```c
#include <math.h>
#include <stdbool.h>
#include <stdio.h>

typedef struct {
    double ey_m;       /* 相对期望碰撞线的横向位移误差，m */
    double ev_mps;     /* 相对期望终端横向速度的误差，m/s */
    double tgo_s;      /* 剩余飞行时间估计，s */
    double ar_mps2;    /* 未补偿的相对横向加速度估计，m/s^2 */
} TerminalError;

typedef struct {
    double zem_m;          /* 加速度补偿后的零控脱靶量，m */
    double zem_cmd_mps2;   /* ZEM 最小能量修正指令，m/s^2 */
    double iac_cmd_mps2;   /* 线性最优落角约束指令，m/s^2 */
    double tzc_cmd_mps2;   /* 末端零控制指令，m/s^2 */
    bool valid;
} ImprovedGuidance;

static double clamp(double x, double limit) {
    if (x > limit) return limit;
    if (x < -limit) return -limit;
    return x;
}

static ImprovedGuidance improved_guidance(
    TerminalError e, double min_tgo_s, double accel_limit_mps2)
{
    ImprovedGuidance out = {0.0, 0.0, 0.0, 0.0, false};
    if (!isfinite(e.ey_m) || !isfinite(e.ev_mps) ||
        !isfinite(e.tgo_s) || !isfinite(e.ar_mps2) ||
        !isfinite(min_tgo_s) || !isfinite(accel_limit_mps2) ||
        min_tgo_s <= 0.0 || accel_limit_mps2 <= 0.0 ||
        e.tgo_s < min_tgo_s) {
        return out; /* 近终端奇异区或非法输入不得继续除以 tgo */
    }

    const double T = e.tgo_s;
    const double T2 = T * T;
    out.zem_m = e.ey_m + T * e.ev_mps + 0.5 * e.ar_mps2 * T2;
    out.zem_cmd_mps2 = clamp(-3.0 * out.zem_m / T2,
                             accel_limit_mps2);
    out.iac_cmd_mps2 = clamp(-6.0 * e.ey_m / T2
                             -4.0 * e.ev_mps / T,
                             accel_limit_mps2);
    out.tzc_cmd_mps2 = clamp(-12.0 * e.ey_m / T2
                             -6.0 * e.ev_mps / T,
                             accel_limit_mps2);
    out.valid = true;
    return out;
}

int main(void) {
    const TerminalError e = {120.0, 8.0, 6.0, 1.5};
    const ImprovedGuidance g =
        improved_guidance(e, 0.2, 30.0 * 9.80665);
    if (!g.valid) return 1;
    printf("ZEM=%.2f m, u_zem=%.2f, u_iac=%.2f, u_tzc=%.2f m/s^2\n",
           g.zem_m, g.zem_cmd_mps2,
           g.iac_cmd_mps2, g.tzc_cmd_mps2);
    return 0;
}
```

编译：`cc -std=c99 -Wall -Wextra -Wpedantic improved_guidance.c -lm -o improved_guidance`。三种指令都含 $1/t_{go}$ 或 $1/t_{go}^2$，因此不能一直计算到 $t_{go}=0$。工程实现应设置最小剩余时间、近距终止条件、指令幅值/速率限制，并记录限幅前后的命令。

## 8.5 如何选择这些制导律

| 制导律 | 主要反馈量 | 主要能力 | 关键风险 |
|---|---|---|---|
| PN | $V_c,\boldsymbol\omega_{LOS},\hat{\mathbf r}$ | 三维 LOS 法平面反馈 | 不直接约束落角；目标持续机动时有偏差 |
| APN | PN 项、$P_\perp\hat{\mathbf a}_T$ | 补偿三维目标法向机动 | 加速度估计噪声和延迟直接进入指令 |
| ZEM | $\mathbf{ZEM}_\perp,t_{go}$ | 压缩三维预测脱靶向量 | 强依赖传播模型和剩余时间 |
| 线性最优落角约束 | $\mathbf e_p,\mathbf e_v,t_{go}$ | 同时约束三维脱靶与终端方向 | 小角度假设；末段可能出现大指令 |
| TZC | $\mathbf e_p,\mathbf e_v,t_{go}$ | 三维零脱靶、方向约束且终端指令归零 | 控制前移，前段过载需求可能更高 |
| MPC | 完整预测状态与约束 | 显式处理多种状态/输入约束 | 计算时间、模型误差和不可行问题 |

推荐实现顺序是：先用 PN 建立符号正确的基线，再加入 ZEM 统一预测量，随后加入落角约束；只有在幅值、速率、视场或禁飞区等约束确实重要时，再使用 MPC。不要同时更换状态定义、$t_{go}$ 算法和制导律，否则很难定位性能变化来自哪一层。

## 8.6 剩余时间估计与适用边界

最简单的估计是 $t_{go}\approx R/V_c$，它只在 $V_c>0$、航迹弯曲不大和速度变化较小时合理。落角约束会主动弯曲弹道，实际路程通常大于当前直线距离，因此直接使用 $R/V_c$ 容易低估剩余时间。工程实现至少应：

1. 检查 $R>0$、$V_c>V_{c,min}$ 和时间戳有效性；
2. 对 $t_{go}$ 做上下界和变化率限制，避免估计跳变直接形成指令尖峰；
3. 用当前制导律滚动预测终端状态，并将预测飞行时间反馈给下一周期；
4. 在过载饱和时重新评估终端约束是否仍可达，而不是继续使用未饱和模型的 $t_{go}$。

上述闭式制导律均建立在线性小角度、理想加速度响应和准确状态估计之上。大初始航向误差、强气动非线性、一阶舵机迟滞或长时间饱和都会破坏推导条件；此时应采用分段制导、非线性最优方法或带动力学约束的 MPC。

延伸阅读：Ryoo、Cho 与 Tahk 给出了带终端落角约束的线性最优制导系统推导（*Journal of Guidance, Control, and Dynamics*, 2005, DOI: [10.2514/1.8392](https://doi.org/10.2514/1.8392)）；Lam 给出了加速度补偿 ZEM 与 PN/APN 关系的分析（*Journal of Guidance, Control, and Dynamics*, 2007, DOI: [10.2514/1.26948](https://doi.org/10.2514/1.26948)）。

## 8.7 一个可编译的受限 MPC 教学实现

下面使用一维双积分对象 \(x=[p,v]^T\)，输入为恒定加速度。程序枚举离散候选控制序列，保留代价最低者的首个输入。它是可运行的教学示例，不是高性能求解器；候选数和预测步数增加时，计算量呈 \(M^N\) 增长。

```c
#include <math.h>   // 提供 isfinite
#include <stdbool.h>// 提供 bool
#include <stdio.h>  // 提供 printf

#define N 5         // 预测步数
#define M 5         // 每步候选加速度数

static const double candidates[M] = {-2.0, -1.0, 0.0, 1.0, 2.0}; // m/s^2
static double best_cost; // 当前搜索到的最小代价
static double best_first_u; // 最优序列的第一个控制量

static void search(int depth, double p, double v, double cumulative_cost,
                   double first_u, bool has_first, double dt, double target_p) {
    if (depth == N) { // 到达预测末端，代价已累计完毕
        const double terminal = 8.0 * (p - target_p) * (p - target_p)
                              + 2.0 * v * v; // 终端位置/速度惩罚
        const double total_cost = cumulative_cost + terminal; // 阶段代价与终端代价相加
        if (total_cost < best_cost) { // 保留完整预测成本最低的序列
            best_cost = total_cost;
            best_first_u = first_u;
        }
        return; // 当前分支完成
    }
    for (int j = 0; j < M; ++j) { // 遍历本步有限个候选控制
        const double u = candidates[j]; // 本步加速度命令
        const double p_next = p + dt * v + 0.5 * dt * dt * u; // 精确常加速度位置更新
        const double v_next = v + dt * u; // 速度更新
        const double stage = (p_next - target_p) * (p_next - target_p)
                           + 0.2 * v_next * v_next + 0.1 * u * u; // 阶段成本
        /* 所有阶段代价非负；若已累计成本不小于当前完整最优值，后续不可能更优。 */
        const double next_cost = cumulative_cost + stage; // 累计本步阶段代价
        if (next_cost >= best_cost) continue;
        search(depth + 1, p_next, v_next, next_cost,
               has_first ? first_u : u, true, dt, target_p); // 将累计成本传入下一层
    }
}

int main(void) {
    const double dt = 0.1; // 控制周期，秒
    double p = 0.0, v = 0.0; // 当前状态，m 与 m/s
    const double target_p = 1.0; // 静止目标位置，m
    if (!isfinite(dt) || dt <= 0.0) return 1; // 检查周期合法
    for (int k = 0; k < 30; ++k) { // 闭环运行 30 个周期
        best_cost = INFINITY; // 每个周期重新初始化搜索上界
        best_first_u = 0.0; // 缺省控制量
        search(0, p, v, 0.0, 0.0, false, dt, target_p); // 初始累计成本为零
        const double u = best_first_u; // 只采用第一个控制量
        p += dt * v + 0.5 * dt * dt * u; // 教学对象按同一模型推进
        v += dt * u;
        printf("k=%d p=%.3f v=%.3f u=%.1f\n", k, p, v, u); // 输出闭环轨迹
    }
    return 0; // 正常结束
}
```

编译：`cc -std=c99 -O2 -Wall -Wextra -Wpedantic mpc_demo.c -lm -o mpc_demo`。本示例代码刻意保持短小，实际项目应把状态、权重、约束和求解状态放入显式结构体，避免全局变量；同时加入输入速率限制、超时检查、不可行检测以及安全备用控制。枚举器的所有阶段成本非负，因此剪枝使用“累计阶段成本不小于当前完整最优代价”这一安全下界；若引入负奖励项，就不能直接使用该剪枝条件。

> 注意：此处剪枝只展示搜索思路，成本下界并未构造，且 stage 和 terminal 权重的尺度需一致；生产 MPC 应使用经过验证的 QP/NLP 求解器，不能把此枚举器直接视作实时飞控实现。

## 8.8 约束、可行性与降级

约束至少应区分：状态安全范围、控制幅值范围、控制变化率范围、优化器运行时间。位置或加速度约束可在预测模型中显式表达。若没有可行解，应在设计时规定行为，例如沿用上周期可行解的安全部分、切换至限幅反馈、降低任务目标或报告故障；不能默认把失败求解返回的内存值当作指令。备用策略也必须在同一对象模型和约束下验证。

计算预算需包含最坏情形，而不是只看平均耗时。日志建议记录求解时间、迭代次数、退出码、约束最大违反量、首个控制量及 warm start 是否成功。采样周期小于求解最坏时间时，该 MPC 方案无法按计划闭环运行。

## 8.9 常见故障与排查

| 现象 | 可能原因 | 排查步骤 |
|---|---|---|
| 优化器总报告不可行 | 终端条件太严、约束互相冲突、状态单位错 | 暂时移除终端约束；逐组恢复约束；打印每条约束的单位和值 |
| 输入总顶在上限 | 目标太远/时域太短/输入权重太小 | 对照可达性上界；增大预测时域；检查权重数量级和输入单位 |
| 控制来回跳 | 缺少输入增量惩罚或速率限制、状态噪声大 | 绘制相邻输入差；加入 Δu 约束/代价；单独回放无噪声情况 |
| 仿真效果好、实物/高保真差 | 模型偏差、时延、执行机构动态没建模 | 记录实际输入与指令差；加入时延/一阶舵机模型；做参数扰动验证 |
| 偶发超时或错过周期 | 最坏计算量未评估、求解容差过严 | 统计最大执行时间而非均值；限制迭代和预算；准备确定性备用控制 |
| 解突然变差 | warm start 错误、数据结构未重置、上一轮状态过期 | 检查初值时间戳；验证移位后的控制序列；超时后丢弃陈旧解 |

排查顺序：先验证离散模型能否手算一步，再验证代价函数各项的单位与权重，然后单独启用约束，最后加入噪声、时延和执行机构模型。每次保留基准案例，避免多项同时改动后无法定位原因。

## 本章实现检查清单

- 状态、输入、参考量均有明确的单位和坐标系；角度内部统一用 rad。
- 明确实现的是 PN、APN、ZEM、落角约束还是 TZC，不能只把不同公式都笼统称为“最优制导”。
- $t_{go}$ 的定义、估计方法、上下界和近终端终止条件均已写入接口与测试。
- 落角约束中的 $e_v$ 已扣除期望终端横向速度，终端角度和坐标系正方向一致。
- 每次优化前确认状态估计有效且时间戳新鲜。
- 控制幅值与速率限制既写进优化问题，也在输出接口处做最后保护。
- 优化失败、超时、NaN、输入缺失均有显式状态码和预定降级路径。
- 对噪声、模型偏差、采样抖动、约束边界分别做回归案例。

## 8.10 算例：同一状态为何给出不同指令

设 \(e_y=120\ \mathrm m\)、\(e_v=8\ \mathrm{m/s}\)、\(t_{go}=6\ \mathrm s\)，暂不考虑相对加速度，则

\[
ZEM=120+6\times8=168\ \mathrm m.
\]

\[
u_{ZEM}=-14.0\ \mathrm{m/s^2},\qquad
u_{IAC}\approx-25.3\ \mathrm{m/s^2},\qquad
u_{TZC}=-48.0\ \mathrm{m/s^2}.
\]

差异来自终端要求不同：ZEM 只约束终端位置；IAC 还约束终端横向速度；TZC 又增加终端控制归零。约束越多，当前通常需要投入越多控制。若可用过载不足，应报告不可行或按优先级放宽软约束。

## 8.11 本章小结与基础自测

### 计算与编程练习

> **实验能力边界：** 标注为“网页可完成”的内容仅使用本页实际显示的控件；凡要求新增制导律、噪声、延迟或约束开关而页面未显示对应控件，均标注为“需自行编程”，不应寻找不存在的滑块。

- **基础题：**分别写出 PN、APN、ZEM、IAC、TZC 和 MPC 新增了什么信息或约束。
- **计算题：**复算 8.10 节三种闭式指令，并检查每项量纲。
- **编程题：**统一接口实现三种制导律，输出限幅前后命令和不可行标志。
- **实验步骤：**固定估计器、执行机构和终止条件，只切换制导律；随后分别扫描 $t_{go}$ 偏差和过载上限。比较脱靶量、落角、控制能量和饱和占空比。

1. 为什么 \(t_{go}\) 减半会使位置误差项产生约四倍指令？
2. APN 在目标加速度估计延迟较大时，为何可能比 PN 更差？
3. “终端权重很大”与“终端等式约束”有什么区别？
4. 落角约束制导长期饱和时，继续增大增益能恢复可达性吗？
5. MPC 超时时，不同降级指令各有什么风险？

本章核心不是记忆系数，而是从终端条件反推控制需求：先定义状态与约束，再确认可观测性和可达性，最后选择闭式律或在线优化。含 \(t_{go}^{-1}\) 的制导律必须与近端终止、状态质量和执行机构约束一起实现。

### 参考答案与判断依据

1. 位置误差项含 $1/t_{go}^2$，因此剩余时间减半时该项放大四倍；近端量测噪声也会被强烈放大。
2. 延迟或带噪的目标加速度经前馈直接进入命令，可能补偿错误时刻甚至错误方向的机动。
3. 大权重仍允许用有限代价违反目标；等式约束要求严格满足，若物理能力不足则问题不可行。
4. 不能。饱和意味着实际输入已不随理论增益增加，可达域没有扩大，反而可能带来抖动和 windup。
5. 陈旧序列可能与新状态不匹配，PN 基线不满足复杂终端约束，零指令可能直接丧失拦截能力；降级策略必须离线验证并带有效期。

### 贯穿算例：公平比较改进制导律

对统一三维场景使用同一估计器、同一 30g/速率约束和同一终止判据，分别运行三维 PN、APN、ZEM 与落角约束制导。每种方法都必须同时报告横向和垂向终端误差，并比较 $\int\lVert\mathbf a_c\rVert^2dt$、饱和占空比和剩余速度；不能只选择表现较好的平面分量。
