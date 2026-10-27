# 第5章 飞行动力学、气动模型与 BTT/STT 控制分配

> 本章把“期望法向加速度”转换成导弹能够跟踪的姿态与气动指令。BTT/STT 的选择不是术语差异，而是如何利用弹体气动效率的问题。所有公式都必须连同坐标系、正方向、单位和有效飞行包线一起使用。

## 本章导航

**前置知识：**牛顿第二定律、动压、升力/阻力系数以及第3章的加速度指令。
**学完后应能：**解释制导层为什么输出加速度而不是舵偏；说明动压怎样决定控制能力；比较 BTT 与 STT 的气动效率、响应速度和耦合；理解前馈反解、反馈跟踪和控制分配各自的职责。

从这里开始，手册从“希望弹道怎样弯”进入“弹体实际上能否这样弯”。这是制导与控制之间最容易被忽略、却最决定工程成败的一层。

## 5.1 从期望加速度到飞行控制量

制导层输出的通常是期望加速度向量 \(\mathbf a_c\)，而飞控层可直接调节的量可能是舵偏角、推力矢量、攻角 \(\alpha\)、侧滑角 \(\beta\) 和滚转角 \(\phi\)。中间需要一个**控制分配/指令生成层**：先把加速度变换到适当坐标系，再求满足气动、姿态和执行机构约束的指令。

必须区分三件事：

1. **前馈反解**：根据气动模型或查表，估计产生目标力所需的 \(\alpha,\beta,\phi\)。
2. **反馈跟踪**：飞控根据测量误差驱动舵面，使实际状态跟上指令。PID 是一种跟踪器，不是气动反解模型。
3. **可达性判断**：目标力可能超出当前动压、迎角、侧滑或舵面能力。此时必须报告不可达并按优先级降级，不能假设 PID 可以“补出来”。

## 5.2 气动力与气动数据库

在常见准定常模型中，动压为

\[
q=\tfrac12\rho V^2,
\]

参考面积为 \(S\) 时，升力、侧力、阻力系数关系可写成

\[
L=qS C_L(\alpha,\beta,M,Re,\delta,\ldots),\quad
Y=qS C_Y(\alpha,\beta,M,Re,\delta,\ldots),\quad
D=qS C_D(\alpha,\beta,M,Re,\delta,\ldots).
\]

这里的“……”可能包括构型、舵偏、推力状态等。数据表插值时至少要明确：每个轴的单位与顺序、角度是 deg 还是 rad、系数采用何种参考轴、数据来自何种构型、表外怎么处理。**绝不允许悄悄把表外点夹到边界后仍当成正常值**；应返回越界标志，并采取经设计确认的限制或降级策略。

动压接近零时，利用 \(L=qSC_L\) 反求系数会病态；不要直接除以 \(q\)。先做最小动压判断，并返回“当前模型/控制权不足”状态。气动表是模型输入，不是对真实导弹的自动保证。

## 5.3 BTT：倾斜升力矢量分配

BTT（bank-to-turn，倾斜转弯）的典型近似是假设侧滑较小，通过滚转把最大升力面转到所需机动平面。它能充分利用纵向升力能力并减小侧滑，通常适合高动压、高机动末段，但滚转通道响应会引入额外相位滞后和轴间耦合。定义速度坐标系中的法向加速度分量 \(a_z\) 和横向分量 \(a_y\)，并规定 \(\phi=0\) 时升力沿 \(+z\) 轴、正 \(\phi\) 将升力向 \(+y\) 倾斜，则理想关系为

\[
\begin{bmatrix}a_y\\a_z\end{bmatrix}
=\frac{L}{m}
\begin{bmatrix}\sin\phi\\\cos\phi\end{bmatrix}.
\]

于是非零目标加速度下，几何反解为

\[
\phi_c=\operatorname{atan2}(a_{y,c},a_{z,c}),\qquad
L_c=m\sqrt{a_{y,c}^2+a_{z,c}^2}.
\]

若忽略重力或已将重力补偿进指令，这个表达式才可直接使用。若坐标轴或正方向不同，符号也随之改变；用 `atan2` 而非 `atan(a_y/a_z)` 可正确处理象限并避免分母为零。之后通过 \(C_{L,c}=L_c/(qS)\) 和气动模型求 \(\alpha_c\)。在近似线性区 \(C_L\approx C_{L0}+C_{L\alpha}\alpha\) 时，

\[
\alpha_c\approx(C_{L,c}-C_{L0})/C_{L\alpha},
\]

但工程实现应优先对有效气动表做受约束反解，而不是无限外推线性斜率。

**BTT 的工程坑：** 倾侧角是周期角。对 \(+179^\circ\) 到 \(-179^\circ\) 直接做差会得到 \(-358^\circ\)，实际最短变化只有 \(+2^\circ\)。此外，目标加速度向量接近零时方向未定义，`atan2(0,0)` 的输出不能被当成有物理意义的滚转指令；应保持上次有效航向/倾侧参考或按任务逻辑退出转弯。

### 倾侧角反解为什么使用 `atan2`

在速度法平面选择正交方向 $\mathbf e_y,\mathbf e_z$，先投影 $a_y=\mathbf a_c^T\mathbf e_y$、$a_z=\mathbf a_c^T\mathbf e_z$。单一升力矢量滚转后满足

\[
a_y=a_n\sin\phi_c,\qquad a_z=a_n\cos\phi_c.
\]

平方相加得到 $a_n=\sqrt{a_y^2+a_z^2}$，两式相除并保留象限得到 $\phi_c=\operatorname{atan2}(a_y,a_z)$。普通 $\arctan(a_y/a_z)$ 会丢失象限且在 $a_z=0$ 时除零。该推导只完成几何分配，并未说明滚转需要多久，也未自动满足攻角和动压约束。

## 5.4 STT：通过侧滑产生侧向气动力

STT（skid-to-turn，侧滑转弯）不先滚转，而是直接利用侧滑和侧向舵面产生侧力，响应链更短，适合滚转受限构型或较低速段；代价是侧向气动效率通常较低，且偏航—滚转、侧滑—攻角交叉耦合更显著。它不是把 BTT 公式里的 \(\phi\) 换成 \(\beta\)：侧力与侧滑的关系由具体构型、马赫数、迎角、舵偏等决定。例如局部模型可能写作

\[
C_Y\approx C_{Y0}+C_{Y\beta}\beta+C_{Y\delta_r}\delta_r,
\quad a_y\approx qS C_Y/m,
\]

若当前局部斜率有效，才可近似反解

\[
\beta_c\approx\frac{m a_{y,c}/(qS)-C_{Y0}-C_{Y\delta_r}\delta_r}{C_{Y\beta}}.
\]

STT 的符号、可用侧力和与滚转/偏航通道的耦合均依赖气动数据库及控制构型，不存在跨构型通用的 \(a_y\to\beta\) 公式。应在查表有效域内求解目标侧力，检查解是否唯一、是否满足 \(|\beta|\) 和舵偏限制，并把不可达状态回报上层。

## 5.5 查表反解与反馈的组合

建议流程：

1. 将指令、状态和气动表统一到规定坐标系及单位。
2. 根据任务要求补偿已知重力/惯性项，得到所需气动力。
3. 通过查表或数值求解寻找 \((\alpha,\beta,\phi,\delta)\) 候选解。
4. 检查表域、包线、舵偏、速率和多解歧义；选取与当前状态连续且代价合适的解。
5. 将指令交给飞控闭环跟踪，监控实际加速度及跟踪误差。
6. 当不可达或模型无效时，记录原因并按优先级限幅/降级，而不是扩大 PID 积分。

查表前馈解决“理论上需要什么气动量”，反馈解决“实际量如何跟上”。反馈误差大时应同时检查传感器时延、符号、坐标变换、执行机构饱和和模型失配。单纯提高 PID 增益可能导致振荡、舵机饱和和噪声放大。

## 5.6 C 示例：BTT 几何分配与安全状态

下面的独立程序使用 SI 单位，输入是已经补偿重力后的速度系法向加速度。它展示几何分配、角度限幅和低动压/不可达检查；不替代气动表求逆与飞控。

```c
#include <math.h>       // 提供 atan2、sqrt、fmod 等数学函数
#include <stdbool.h>    // 提供 bool、true、false
#include <stdio.h>      // 提供 printf

#define PI 3.14159265358979323846  // 定义高精度圆周率常量

typedef struct {                    // 定义分配结果结构体
    double bank_rad;                // 倾侧角指令，单位 rad
    double lift_n;                  // 需求升力，单位 N
    double ay_achievable;           // 限幅后模型可实现的横向加速度，单位 m/s^2
    double az_achievable;           // 限幅后模型可实现的竖直法向加速度，单位 m/s^2
    bool valid;                     // 本次分配是否有效
    bool limited;                   // 是否触发了限幅/不可达
} BttResult;

static double clamp(double x, double lo, double hi) { // 将数值限制在闭区间内
    if (x < lo) return lo;            // 低于下界时返回下界
    if (x > hi) return hi;            // 高于上界时返回上界
    return x;                         // 区间内则原样返回
}

static BttResult btt_allocate(double ay, double az, double mass,
                              double q, double area,
                              double lift_coeff_max, double bank_limit_rad,
                              double previous_bank_rad) {
    BttResult out = {0.0, 0.0, 0.0, 0.0, false, false}; // 默认安全无效输出
    if (!isfinite(ay) || !isfinite(az) || !isfinite(mass) || // 检查输入是否为有限数
        !isfinite(q) || !isfinite(area) || !isfinite(lift_coeff_max) ||
        !isfinite(bank_limit_rad) || !isfinite(previous_bank_rad) ||
        mass <= 0.0 || q <= 1.0 || area <= 0.0 || lift_coeff_max <= 0.0 ||
        bank_limit_rad < 0.0 || bank_limit_rad > PI) {
        return out;                     // 数值异常、质量/动压/面积无效时拒绝计算
    }
    double a_mag = hypot(ay, az);       // 计算目标法向加速度模，避免直接平方溢出
    if (a_mag < 1e-9) {                 // 目标向量几乎为零，方向没有定义
        out.bank_rad = previous_bank_rad; // 小指令时保持上一有效倾侧角，避免方向跳变
        out.ay_achievable = 0.0;          // 本例将零指令对应的可实现加速度记为零
        out.az_achievable = 0.0;          // 含重力模型需另外定义零指令保持逻辑
        out.valid = true;               // 零升力需求是有效的
        out.limited = false;            // 没有发生限幅
        return out;                     // bank_rad 保持零；真实系统可保持上次有效参考
    }
    double bank = atan2(ay, az);        // 根据明确的坐标约定求倾侧角，正确处理象限
    double lift = mass * a_mag;         // 由 F=m*a 得到理想升力需求，单位 N
    double cl_required = lift / (q * area); // 计算所需升力系数，无量纲
    double cl_used = fmin(cl_required, lift_coeff_max); // 限制到正升力教学模型的有效上限
    if (cl_used < cl_required) out.limited = true; // 记录升力能力不足
    double lift_used = q * area * cl_used; // 计算限制后可用的升力
    double bank_limited = clamp(bank, -bank_limit_rad, bank_limit_rad); // 对倾侧角施加幅值限制
    if (fabs(bank_limited - bank) > 1e-12) out.limited = true; // 记录倾侧角触限
    /* 倾侧角限幅后，按受限角度重算可实现加速度，不能继续报告原目标已实现。 */
    out.ay_achievable = (lift_used / mass) * sin(bank_limited); // 可实现横向分量，未计横向重力
    out.az_achievable = (lift_used / mass) * cos(bank_limited); // 可实现升力法向分量，未计重力
    out.bank_rad = bank_limited;                // 写入倾侧角指令
    out.lift_n = lift_used;             // 写入受能力约束的升力请求
    out.valid = true;                   // 输入有效，输出可用于后续控制分配
    return out;                         // 返回结果及状态标志
}

int main(void) {                        // 独立演示程序入口
    const double mass = 180.0;          // 教学用导弹质量，kg；非型号参数
    const double q = 15000.0;           // 动压，Pa
    const double area = 16.0;           // 参考面积，m^2
    const double cl_max = 1.2;          // 教学用有效升力系数上限
    const double bank_max = 70.0 * PI / 180.0; // 将 70 度转换为内部弧度
    BttResult r = btt_allocate(4.0, 6.0, mass, q, area, cl_max, bank_max, 0.0); // 请求 ay=4, az=6 m/s^2；首次调用以 0 rad 初始化
    if (!r.valid) {                      // 检查函数是否拒绝了输入
        puts("分配失败：检查输入、动压和质量"); // 给出可操作的诊断提示
        return 1;                        // 用非零状态码表示失败
    }
    printf("bank=%.2f deg, lift=%.1f N, ay_real=%.2f, az_real=%.2f, limited=%d\n", // 输出受限后的估计可达量
           r.bank_rad * 180.0 / PI, r.lift_n, r.ay_achievable, r.az_achievable, (int)r.limited); // 显示角度时转为度
    return 0;                            // 正常结束
}
```

编译命令：`cc -std=c99 -Wall -Wextra -pedantic btt_demo.c -lm -o btt_demo`。本例把目标法向量解释为已补偿后的速度系升力需求，使用正升力局部近似。若倾侧角或升力系数限幅，程序会基于限幅后的倾侧角与升力重算可实现分量；通常不再等于原始目标。代码没有建模重力、姿态动力学、攻角反解和舵机响应，实际系统必须明确哪些项已由上层补偿，避免重力重复补偿或漏补偿。示例中的 CLmax=1.2、动压、面积质量比和倾侧角上限仅用于演示，绝非通用推荐值；应依据具体构型、飞行状态和验证过的气动数据确定，并同时检查攻角包线、变化率和载荷约束。

## 5.7 常见错误与排查

| 现象 | 常见原因 | 逐步排查 | 建议修正 |
|---|---|---|---|
| 横向指令方向反了 | 坐标轴正方向、视线定义或 `atan2` 参数顺序不一致 | 用一个“向右转”的已知矢量手算；打印各坐标系下 ay/az；检查变换矩阵是否正交 | 在接口文档中固定轴向；为变换写单元测试 |
| 倾侧角在 ±180° 附近猛跳 | 直接做角度差，没有 wrap | 打印当前角、目标角和差值；测试 179°→−179° | 用最短角差 `atan2(sin(d),cos(d))` 后再限速 |
| 攻角需求突然很大 | 低动压除法、查表越界、斜率接近零或单位错误 | 记录 q、CL_req、alpha_req、气动表边界和插值标志 | 最小动压保护；标记表外；对逆解范围和斜率设检查 |
| STT 方向与预期相反 | CYβ 符号/侧滑正向与代码约定相反 | 在固定 α、M、δ 下对 β 做小幅正负扰动，核实 CY 变化方向 | 从气动数据自动核验局部导数符号；显式配置侧滑约定 |
| PID 越调越大但加速度上不去 | 请求不可达、执行机构已饱和，积分 windup | 同时画请求/限幅后/实际加速度、舵偏和积分状态 | 把可达性标志传到跟踪器；暂停或回算积分 |
| 插值结果突然折线/跳变 | 表轴顺序错、非均匀网格按均匀网格插值、重复/缺失数据 | 对每个轴做边界点、网格点和中点测试；查表数据版本 | 校验网格单调性和维度；离线生成数据摘要及测试 |
| 高速时结果偏差很大 | 只使用低速线性气动斜率，忽略马赫数/构型变化 | 对照表格输入实际 M、Re 和舵偏；确认插值维数 | 在适用包线内用多维表；超域返回状态而非静默外推 |

## 5.8 三自由度与六自由度模型的边界

三自由度质点模型只传播位置和速度，把法向加速度看作可直接实现的输入，适合制导律原理验证。六自由度模型还传播姿态和角速度，并通过气动力矩、舵机和推力矢量产生运动：

\[
\text{制导加速度}\xrightarrow{\text{反解/分配}}
(\alpha_c,\beta_c,\phi_c)
\xrightarrow{\text{姿态控制}}
\delta
\xrightarrow{\text{六自由度动力学}}
\mathbf a_{actual}.
\]

若三自由度仿真假定命令加速度瞬时实现，它只能证明几何制导趋势，不能证明真实弹体可实现。至少应加入一阶加速度响应、幅值和速率限制，再讨论制导律性能。

## 5.9 自测与小结

### 分层练习与实验操作卡

- **基础题：**解释 BTT 为何需要滚转，STT 为何会引入侧滑。
- **计算题：**对 $a_y=20,a_z=20\ \mathrm{m/s^2}$ 计算 $a_n$ 和 $\phi_c$。
- **编程题：**实现零向量保持、角度环绕、倾侧角速率限制和气动表越界标志。
- **实验步骤：**固定加速度方向扫描动压，再固定动压扫描加速度方向。分别观察攻角需求和滚转需求，不能把几何反解结果当作真实姿态响应。

1. 相同舵偏在低动压和高动压下为什么产生不同加速度？
2. 为什么 BTT 通常气动效率高，却不一定比 STT 响应更快？
3. 前馈气动反解已经很准确时，为什么仍需要反馈控制？
4. 指令长期不可达时，正确做法为什么不是继续增大 PID 积分？

**本章结论：**制导律给出“需要的轨迹曲率”，控制系统负责把它变成真实气动力。BTT/STT、气动包线和执行机构共同决定这条命令是否可达。

### 参考答案与判断依据

1. 气动力近似与动压 $q=\tfrac12\rho V^2$ 成正比，因此相同舵偏在不同高度和速度下产生的力矩、升力与加速度不同。
2. BTT 要先滚转，使最大升力面转到所需方向；若滚转通道带宽有限，方向建立可能比直接侧滑更慢。
3. 反解依赖不精确的气动模型，反馈用于消除参数误差、外扰和未建模动态。
4. 积分只会继续累积不可实现的需求并造成 windup。应限幅、抗饱和、报告不可达，并重新评估制导目标。

### 贯穿算例：从加速度到 BTT 指令

把第3章 $\mathbf a_c$ 投影到速度法平面，得到两个正交法向分量。BTT 可用 $\phi_c=\operatorname{atan2}(a_{c,2},a_{c,1})$ 决定升力面方向，再由 $n_c=\lVert\mathbf a_c\rVert/g$ 得到所需过载。若 $n_c>30$，必须先承认命令不可达，再限幅；不能通过角度反解“绕过”过载上限。

---
