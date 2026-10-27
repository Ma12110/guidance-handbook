# 第5章 飞行动力学、气动模型与 BTT/STT 控制分配

> 本章把“期望加速度”转换成飞行器能够跟踪的姿态/气动指令。所有公式都必须连同坐标系、正方向、单位和有效飞行包线一起使用。这里的模型是通用教学模型，不代表某一具体飞行器。

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

动压接近零时，利用 \(L=qSC_L\) 反求系数会病态；不要直接除以 \(q\)。先做最小动压判断，并返回“当前模型/控制权不足”状态。气动表是模型输入，不是对真实飞行器的自动保证。

## 5.3 BTT：倾斜升力矢量分配

BTT（bank-to-turn，倾侧转弯）的典型近似是假设侧滑较小，侧向机动主要靠滚转倾斜升力矢量实现。定义速度坐标系中的法向加速度分量 \(a_z\) 和横向分量 \(a_y\)，并规定 \(\phi=0\) 时升力沿 \(+z\) 轴、正 \(\phi\) 将升力向 \(+y\) 倾斜，则理想关系为

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

## 5.4 STT：通过侧滑产生侧向气动力

STT（sideslip-to-turn，侧滑转弯）通过产生侧滑角，利用侧力获得横向加速度。它不是把 BTT 公式里的 \(\phi\) 换成 \(\beta\)：侧力与侧滑的关系由具体构型、马赫数、迎角、舵偏等决定。例如局部模型可能写作

\[
C_Y\approx C_{Y0}+C_{Y\beta}\beta+C_{Y\delta_r}\delta_r,
\quad a_y\approx qS C_Y/m,
\]

若当前局部斜率有效，才可近似反解

\[
\beta_c\approx\frac{m a_{y,c}/(qS)-C_{Y0}-C_{Y\delta_r}\delta_r}{C_{Y\beta}}.
\]

STT 的符号、可用侧力和与滚转/偏航通道的耦合均依赖气动数据库及控制构型，不存在跨飞行器通用的 \(a_y\to\beta\) 公式。应在查表有效域内求解目标侧力，检查解是否唯一、是否满足 \(|\beta|\) 和舵偏限制，并把不可达状态回报上层。

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
                              double lift_coeff_max, double bank_limit_rad) {
    BttResult out = {0.0, 0.0, false, false}; // 默认安全无效输出
    if (!isfinite(ay) || !isfinite(az) || !isfinite(mass) || // 检查输入是否为有限数
        !isfinite(q) || !isfinite(area) || mass <= 0.0 || q <= 1.0 || area <= 0.0) {
        return out;                     // 数值异常、质量/动压/面积无效时拒绝计算
    }
    double a_mag = hypot(ay, az);       // 计算目标法向加速度模，避免直接平方溢出
    if (a_mag < 1e-9) {                 // 目标向量几乎为零，方向没有定义
        out.valid = true;               // 零升力需求是有效的
        out.limited = false;            // 没有发生限幅
        return out;                     // bank_rad 保持零；真实系统可保持上次有效参考
    }
    double bank = atan2(ay, az);        // 根据明确的坐标约定求倾侧角，正确处理象限
    double lift = mass * a_mag;         // 由 F=m*a 得到理想升力需求，单位 N
    double cl_required = lift / (q * area); // 计算所需升力系数，无量纲
    double cl_used = fmin(cl_required, lift_coeff_max); // 将系数需求限制在模型有效上限
    if (cl_used < cl_required) out.limited = true; // 记录升力能力不足
    double lift_used = q * area * cl_used; // 计算限制后可用的升力
    bank = clamp(bank, -bank_limit_rad, bank_limit_rad); // 对倾侧角施加幅值限制
    if (fabs(bank) >= bank_limit_rad) out.limited = true; // 记录倾侧角触限
    out.bank_rad = bank;                // 写入倾侧角指令
    out.lift_n = lift_used;             // 写入受能力约束的升力请求
    out.valid = true;                   // 输入有效，输出可用于后续控制分配
    return out;                         // 返回结果及状态标志
}

int main(void) {                        // 独立演示程序入口
    const double mass = 1200.0;         // 飞行器质量，kg
    const double q = 15000.0;           // 动压，Pa
    const double area = 16.0;           // 参考面积，m^2
    const double cl_max = 1.2;          // 教学用有效升力系数上限
    const double bank_max = 70.0 * PI / 180.0; // 将 70 度转换为内部弧度
    BttResult r = btt_allocate(4.0, 6.0, mass, q, area, cl_max, bank_max); // 请求 ay=4, az=6 m/s^2
    if (!r.valid) {                      // 检查函数是否拒绝了输入
        puts("分配失败：检查输入、动压和质量"); // 给出可操作的诊断提示
        return 1;                        // 用非零状态码表示失败
    }
    printf("bank=%.2f deg, lift=%.1f N, limited=%d\n", // 仅在显示时转换为度
           r.bank_rad * 180.0 / PI, r.lift_n, (int)r.limited); // 打印结果
    return 0;                            // 正常结束
}
```

编译命令：`cc -std=c11 -Wall -Wextra -pedantic btt_demo.c -lm -o btt_demo`。本例把目标加速度模直接视为升力加速度，省略了重力、姿态动力学、攻角反解和舵机响应；实际系统必须明确哪些项已由制导层补偿，避免重力重复补偿或漏补偿。

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

---
