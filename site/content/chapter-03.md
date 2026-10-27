# 第3章 经典制导律：从相对几何到受限指令

本章把追踪法、前置碰撞航向和比例导引放在统一的弹目相对运动框架里。比例导引不是“追着目标尾巴跑”，而是通过负反馈压低视线角速度，维持碰撞三角形：若闭合速度保持为正且 $\dot\lambda\rightarrow0$，弹目视线方向趋于固定，双方沿各自速度矢量走向同一未来点。计算结果是期望横向加速度，必须经过第5、6章的动力学反解、控制分配和指令整形，不能直接当成舵偏角。

## 本章导航

**前置知识：**第2章的相对位置、闭合速度和 LOS 角速度。
**学完后应能：**区分追踪法、前置法和 PN；从碰撞三角形解释 PN；说明导航系数 $N$、闭合速度与 LOS 角速度分别起什么作用；知道二维公式怎样扩展为三维向量形式。

## 3.1 先固定符号和单位

在二维惯性平面内，导弹位置和速度为 \(\mathbf r_M,\mathbf v_M\)，目标位置和速度为 \(\mathbf r_T,\mathbf v_T\)。定义“目标减导弹”的相对量：

\[
\mathbf r=\mathbf r_T-\mathbf r_M=[x,y]^T,\quad
\mathbf v_r=\mathbf v_T-\mathbf v_M=[v_x,v_y]^T,\quad R=\|\mathbf r\|.
\]

视线角 \(\lambda=\operatorname{atan2}(y,x)\)，闭合速度定义为 \(V_c=-\dot R\)，所以正在接近时 \(V_c>0\)。所有计算角均用 rad，角速度用 rad/s，速度 m/s，加速度 m/s²，时间 s。代码内部一律 SI；转换到度只用于显示。

对 \(R>0\)，对距离平方求导：

\[
R^2=x^2+y^2,\quad 2R\dot R=2xv_x+2yv_y,
\quad \dot R=\frac{xv_x+yv_y}{R}.
\]

再对 atan2 的几何关系求导：

\[
\dot\lambda=\frac{xv_y-yv_x}{x^2+y^2}.
\]

分子是二维叉积的标量形式。若状态量采用相反的相对方向“导弹减目标”，两式的方向符号也会改变；不要靠试改一个负号来修补结果，先把相对向量定义写在代码注释和单元测试里。

## 3.2 几何方法各自回答什么

- **纯追踪**：速度方向指向目标当前方位。实现直接，但移动目标下容易绕行或尾随；它不等价于预测会合。
- **固定前置方向**：速度方向相对当前视线提前一个预设角。对固定场景可作示范，不适合把单一前置角当通用解。
- **比例导引（PN）**：用视线旋转量生成横向加速度。本文给出一种二维速度法向约定：

\[
a_n=N V_c\dot\lambda,
\]

其中 \(N\) 无量纲。上式是制导命令律，不是由运动学恒等式自动推出的稳定性保证。不同资料中的 PPN/TPN 定义、加速度方向和速度尺度可能不同，工程文档必须写全约定，不能只写“PN”。

离散系统每个采样周期读取一致时间戳的相对位置与速度，计算几何量，形成命令并送入限幅/控制分配。目标状态延迟、速度估计噪声、飞行控制响应和采样周期都会改变闭环表现。必要时需加死区、滤波或指令整形；滤波器会引入相位滞后，不能不经验证地无限加大滤波强度。

### 追踪法、前置法和 PN 的根本区别

纯追踪法始终把速度方向指向目标当前位置。目标只要存在横向速度，导弹就会不断改变航向，弹道常呈持续弯曲。它容易理解，但通常消耗更多速度和控制能量，末段还可能出现较大转弯需求。

前置法瞄准预测相遇点而非目标当前位置。若目标匀速、导弹速度近似恒定，可通过

\[
\left\|\mathbf r+\mathbf v_T t_i\right\|=V_M t_i
\]

求正的拦截时间 $t_i$，再把期望速度方向指向 $\mathbf r+\mathbf v_Tt_i$。其弱点是依赖目标运动预测；目标机动后必须滚动重算。

PN 不直接追逐位置，也不必显式解出远处的相遇点，而是反馈当前碰撞几何是否正在被破坏。若 LOS 持续旋转，说明当前速度三角形不能在同一点闭合。PN 产生法向加速度使 $\dot\lambda$ 衰减。$V_c$ 把角速率转换为加速度尺度；$N$ 决定反馈强度。教学分析常取 $N$ 在 3～5 附近，但它不是脱离动力学、延迟和噪声后仍普适的“最佳常数”。

三维情况下可利用 LOS 角速度向量写成

\[
\mathbf a_c=N V_c
\left(\boldsymbol\omega_{LOS}\times\hat{\mathbf r}\right).
\]

具体正负号取决于相对量与加速度坐标定义。可靠做法不是凭经验改符号，而是构造手算案例，验证该指令能否减小下一时刻的 LOS 角速度。

### PN 从碰撞几何到反馈公式

二维、小角度、速度近似恒定时，设横向误差为 $y$、纵向距离为 $x\approx V_ct_{go}$，则 $\lambda\approx y/x$。理想碰撞三角形要求 LOS 方向趋于固定，也就是 $\dot\lambda\to0$。因此命令结构应对 $\dot\lambda$ 做负反馈；为了得到加速度量纲，还需乘一个速度尺度，于是形成

\[
a_n=NV_c\dot\lambda.
\]

$N$ 是无量纲导航比，不是“导弹转角等于目标转角的 $N$ 倍”。在理想、无延迟、非机动目标模型中，常见分析要求 $N>2$ 才有足够终端收敛趋势；实际系统还受噪声、气动迟滞和饱和制约，所以不能无限增大。

三维时，$\boldsymbol\omega_{LOS}$ 给出 LOS 的空间转轴，因而

\[
\mathbf a_c=NV_c(\boldsymbol\omega_{LOS}\times\hat{\mathbf r}).
\]

必须同时通过两个自检：量纲为 $\mathrm{m/s^2}$，并且 $\mathbf a_c^T\hat{\mathbf r}\approx0$。前者确认它是加速度，后者确认它位于 LOS 法平面。

## 3.3 C 示例：二维视线几何与受限命令

以下程序是可独立编译的**几何/软件接口演示**。它计算距离、闭合速度、视线角速度，检查退化输入，再生成带闭合门限和幅值限制的横向加速度命令。它没有实现弹体动力学、姿态环或完整制导性能验证。

```c
#include <math.h>      // hypot、atan2、isfinite
#include <stdbool.h>   // bool 类型
#include <stdio.h>     // printf

#define EPS_R 1.0e-6   // 距离过小时的数值保护阈值，单位 m
#define PI 3.14159265358979323846

typedef struct {
    double rx, ry;     // 相对位置：目标减导弹，m
    double rvx, rvy;   // 相对速度：目标减导弹，m/s
} RelativeState;

typedef struct {
    double range_m;       // 相对距离，m
    double closing_mps;   // 闭合速度，接近为正，m/s
    double los_rate_rps;  // 惯性视线角速度，rad/s
    double accel_cmd;     // 有符号横向加速度命令，m/s^2
    bool valid;           // 输入和计算是否有效
} GuidanceOutput;

static double clamp(double x, double lo, double hi)
{
    if (x < lo) return lo;  // 低于下限时钳位
    if (x > hi) return hi;  // 高于上限时钳位
    return x;               // 范围内原样通过
}

GuidanceOutput guidance_step(RelativeState s,
                             double nav_gain,
                             double accel_limit,
                             double min_closing_speed)
{
    GuidanceOutput o = {0};  // 先清零，避免错误路径留下未初始化输出

    // 防止 NaN/无穷大进入 atan2、平方和及控制器。
    if (!isfinite(s.rx) || !isfinite(s.ry) ||
        !isfinite(s.rvx) || !isfinite(s.rvy) ||
        !isfinite(nav_gain) || !isfinite(accel_limit)) {
        return o;            // valid 保持 false，调用者应进入安全处理
    }
    if (nav_gain < 0.0 || accel_limit <= 0.0) return o;

    const double r2 = s.rx*s.rx + s.ry*s.ry; // 距离平方，m^2
    if (!isfinite(r2) || r2 <= EPS_R*EPS_R) return o;

    o.range_m = sqrt(r2); // 距离，m
    const double range_rate = (s.rx*s.rvx + s.ry*s.rvy) / o.range_m;
    o.closing_mps = -range_rate; // 统一规定“接近”为正
    o.los_rate_rps = (s.rx*s.rvy - s.ry*s.rvx) / r2;

    // 目标未接近或接近速度太小，避免噪声引起的大指令。
    if (o.closing_mps > min_closing_speed) {
        const double raw = nav_gain * o.closing_mps * o.los_rate_rps;
        if (!isfinite(raw)) return (GuidanceOutput){0};
        o.accel_cmd = clamp(raw, -accel_limit, accel_limit);
    } else {
        o.accel_cmd = 0.0;  // 这只是教学策略；真实系统需定义状态机/重规划
    }
    o.valid = true;
    return o;
}

int main(void)
{
    // 示例相对状态：位置 m，速度 m/s；此处数据仅用于演示程序接口。
    const RelativeState s = {1000.0, 120.0, -35.0, -2.0};
    const GuidanceOutput o = guidance_step(s, 3.0, 20.0, 0.1);
    if (!o.valid) {
        puts("输入无效：检查坐标、单位、距离和有限数值");
        return 1;
    }
    printf("R=%.2f m, Vc=%.2f m/s, lambda_dot=%.6f rad/s, a_cmd=%.2f m/s^2\n",
           o.range_m, o.closing_mps, o.los_rate_rps, o.accel_cmd);
    printf("视线角仅供显示：%.2f deg\n", atan2(s.ry, s.rx) * 180.0 / PI);
    return 0;
}
```

用 GCC/Clang 编译：`cc -std=c99 -Wall -Wextra -Wpedantic -O2 guidance_demo.c -lm -o guidance_demo`。`-lm` 用于链接数学库。启用警告并消除警告后再集成，不要用关闭警告的方式掩盖类型或初始化问题。

### 代码接入时必须补上的工程环节

1. **验证时间戳**：`RelativeState` 四个数必须来自同一时刻；不同步数据要先传播到统一时刻。
2. **确认参考系**：所有位置和速度必须在同一惯性/导航系中。若速度来自机体系，先正确旋转。
3. **命令解释**：`accel_cmd` 只是带符号横向加速度；调用者还需定义横向正方向、坐标系与飞控接口。
4. **限幅和速率**：本例只演示幅值限幅。真实接口还需按控制周期施加变化率限制，并监视长期饱和。
5. **状态退出**：`valid=false` 不应被当成零命令悄悄继续运行；上层要产生明确故障码、记录输入并切换安全策略。
6. **大数值保护**：有限数值检查之外，还应按任务范围限制输入；否则极大数值平方会溢出。

## 3.4 常见错误与排查顺序

| 现象 | 常见原因 | 排查方法 |
|---|---|---|
| 接近时 `Vc` 为负 | 相对向量方向与闭合速度定义不一致，或速度顺序颠倒 | 手算一个“距离正在减小”的例子，确认 `Rdot<0` 且 `Vc=-Rdot>0` |
| 视线角速度符号相反 | 坐标系左右手约定、相对位置/速度方向不一致 | 用目标沿 +y 运动、视线沿 +x 的小例子核对二维叉积符号 |
| 加速度命令总贴着上限 | 单位错、`Vc` 过大、角速度噪声、采样不同步或确实超出可用能力 | 逐项打印原始 `Vc`、`lambda_dot`、未限幅命令和限制原因 |
| 轨迹左右来回摆 | 符号错、测量噪声、更新延迟、转弯能力/控制响应不够 | 先关闭噪声与延迟，再检查符号；之后逐项加回动态因素 |
| 命令出现 NaN | 距离接近零、输入已有 NaN、平方溢出、未初始化内存 | 在每个模块边界记录输入范围；启用浮点异常和 Address/Undefined sanitizers |
| 度数输入后命令大约差 57 倍 | 将 deg/s 当 rad/s 使用 | 在接口声明单位；只在输入/显示边界做显式转换 |
| 仿真稳定，实机/实时回放不稳定 | 未模拟传感器延迟、任务调度抖动或控制执行器动态 | 记录采样时间戳和周期抖动；加入延迟、一阶响应及限幅回放 |

建议按这个顺序定位问题：**(1) 单位和数据有效性；(2) 相对向量方向；(3) 坐标系与角度符号；(4) 几何中间量；(5) 未限幅命令；(6) 限幅/变化率；(7) 飞控响应；(8) 噪声和延迟**。一次只改变一项。

## 3.5 实验与验收建议

交互实验应让读者同时看到相对轨迹、视线方向、距离、闭合速度、视线角速度、原始命令和限幅后命令。至少提供“符号检查”场景和“噪声/延迟”开关。验收时，手算的直线相对运动应与页面输出一致；当闭合速度低于门限时，程序应按文档进入零命令或显式状态切换，而不是无限放大噪声。

## 3.6 自测与小结

### 分层练习与实验操作卡

- **基础题：**画出纯追踪、固定前置和 PN 各自使用的几何信息。
- **计算题：**给定 $N=3,V_c=300\ \mathrm{m/s},\dot\lambda=0.01\ \mathrm{rad/s}$，计算指令及其 g 值。
- **编程题：**实现三维 PN，并用点积自动验证命令位于 LOS 法平面。
- **实验步骤：**先在无噪声、无限幅条件下改变 $N$；再逐项加入 LOS 噪声和过载限制。观察收敛、指令峰值与饱和时间，不能只用最终脱靶量评价。

1. 为什么“始终指向目标”不等于“最短时间到达目标”？
2. 当 $\dot\lambda=0$ 但 $V_c<0$ 时，是否仍形成未来碰撞？
3. 增大 $N$ 为什么既可能加快 LOS 收敛，也可能放大噪声并造成饱和？
4. PN 输出为什么不能直接解释为舵偏角？

**本章结论：**追踪法反馈目标方位，前置法预测相遇点，PN 反馈碰撞三角形的旋转。理解这一差别，比记住公式本身更重要。

### 参考答案与判断依据

1. 纯追踪把速度始终指向目标当前位置，目标运动时会形成弯曲的尾追轨迹；前置制导则朝未来相遇点建立碰撞三角形。
2. 不能。$V_c<0$ 表示距离正在增大，即使 LOS 不转，也可能是沿同一直线远离。
3. 较大 $N$ 提高横向纠偏强度，但量测噪声也经同一增益放大，并更容易触及过载和舵机速率上限。
4. PN 输出是所需惯性加速度。舵偏还取决于速度、动压、气动导数、姿态和控制分配，二者不能直接等同。

### 贯穿算例：形成 PN 指令

取导航比 $N=3$，使用第2章算得的 $V_c$ 与 $\boldsymbol\omega_{LOS}$，按三维矢量形式计算 $\mathbf a_c=N V_c(\boldsymbol\omega_{LOS}\times\hat{\mathbf r})$。先验证 $\mathbf a_c^T\hat{\mathbf r}\approx0$：PN 指令应主要位于 LOS 法平面。若点积明显不为零，应优先检查叉积次序和相对位置定义。

---
