# 第8章 最优制导与模型预测控制

本章把“尽快到达目标”改写为可计算的优化问题，并说明模型预测控制（MPC）如何在每个采样时刻重复求解受约束的有限时域问题。优化方法只能在模型、约束和计算预算合理时发挥作用；求解器成功返回不等于真实对象一定安全。

## 8.1 从目标函数到有限时域控制

以离散线性系统 \(x_{k+1}=Ax_k+Bu_k\) 为例，定义预测时域 \(N\)，代价
\[
J=\sum_{i=0}^{N-1}(x_i^TQx_i+u_i^TRu_i)+x_N^TQ_fx_N.
\]
\(Q,Q_f\succeq0\) 惩罚状态偏差，\(R\succ0\) 惩罚控制用量。约束可写为 \(x_i\in\mathcal X, u_i\in\mathcal U\)。在当前时刻预测未来状态并求最小代价控制序列，只执行第一个控制量；下一采样时刻获取新状态后再重新优化。这一过程称滚动时域。

在线性、无约束、固定权重条件下，最优控制可化成线性反馈；引入幅值/速率约束后，通常需要解二次规划或其他优化问题。MPC 的核心价值是显式处理约束，不是自动消除模型误差。

## 8.2 一个可编译的受限 MPC 教学实现

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

## 8.3 约束、可行性与降级

约束至少应区分：状态安全范围、控制幅值范围、控制变化率范围、优化器运行时间。位置或加速度约束可在预测模型中显式表达。若没有可行解，应在设计时规定行为，例如沿用上周期可行解的安全部分、切换至限幅反馈、降低任务目标或报告故障；不能默认把失败求解返回的内存值当作指令。备用策略也必须在同一对象模型和约束下验证。

计算预算需包含最坏情形，而不是只看平均耗时。日志建议记录求解时间、迭代次数、退出码、约束最大违反量、首个控制量及 warm start 是否成功。采样周期小于求解最坏时间时，该 MPC 方案无法按计划闭环运行。

## 8.4 常见故障与排查

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
- 每次优化前确认状态估计有效且时间戳新鲜。
- 控制幅值与速率限制既写进优化问题，也在输出接口处做最后保护。
- 优化失败、超时、NaN、输入缺失均有显式状态码和预定降级路径。
- 对噪声、模型偏差、采样抖动、约束边界分别做回归案例。
