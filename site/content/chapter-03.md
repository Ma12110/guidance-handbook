# 第3章 经典制导律：从相对几何到受限指令

本章把追踪、前置方向和比例导引放在统一的相对运动框架里。所有示例服务于抽象航迹跟踪、无人机航点会合或移动机器人相对运动教学，不代表特定飞行器设计。计算结果是期望横向加速度；它必须经过本书第5、6章的飞行动力学与控制分配环节，不能直接当成舵偏角。

## 3.1 先固定符号和单位

在二维惯性平面内，受控体位置和速度为 \(\mathbf r_M,\mathbf v_M\)，参考目标或移动航点为 \(\mathbf r_T,\mathbf v_T\)。定义“目标减受控体”的相对量：

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

分子是二维叉积的标量形式。若状态量采用相反的相对方向“受控体减目标”，两式的方向符号也会改变；不要靠试改一个负号来修补结果，先把相对向量定义写在代码注释和单元测试里。

## 3.2 几何方法各自回答什么

- **纯追踪**：速度方向指向目标当前方位。实现直接，但移动目标下容易绕行或尾随；它不等价于预测会合。
- **固定前置方向**：速度方向相对当前视线提前一个预设角。对固定场景可作示范，不适合把单一前置角当通用解。
- **比例导引（PN）**：用视线旋转量生成横向加速度。本文给出一种二维速度法向约定：

\[
a_n=N V_c\dot\lambda,
\]

其中 \(N\) 无量纲。上式是制导命令律，不是由运动学恒等式自动推出的稳定性保证。不同资料中的 PPN/TPN 定义、加速度方向和速度尺度可能不同，工程文档必须写全约定，不能只写“PN”。

离散系统每个采样周期读取一致时间戳的相对位置与速度，计算几何量，形成命令并送入限幅/控制分配。目标状态延迟、速度估计噪声、飞行控制响应和采样周期都会改变闭环表现。必要时需加死区、滤波或指令整形；滤波器会引入相位滞后，不能不经验证地无限加大滤波强度。

## 3.3 C 示例：二维视线几何与受限命令

以下程序是可独立编译的**几何/软件接口演示**。它计算距离、闭合速度、视线角速度，检查退化输入，再生成带闭合门限和幅值限制的横向加速度命令。它没有实现飞行器动力学、姿态环或完整制导性能验证。

```c
#include <math.h>      // hypot、atan2、isfinite
#include <stdbool.h>   // bool 类型
#include <stdio.h>     // printf

#define EPS_R 1.0e-6   // 距离过小时的数值保护阈值，单位 m
#define PI 3.14159265358979323846

typedef struct {
    double rx, ry;     // 相对位置：目标减受控体，m
    double rvx, rvy;   // 相对速度：目标减受控体，m/s
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

用 GCC/Clang 编译：`cc -std=c11 -Wall -Wextra -Wpedantic -O2 guidance_demo.c -lm -o guidance_demo`。`-lm` 用于链接数学库。启用警告并消除警告后再集成，不要用关闭警告的方式掩盖类型或初始化问题。

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

---
