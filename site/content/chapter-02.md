# 第2章 坐标系、角度与三维相对运动

> 三维高速拦截看似包含姿态、气动和地球坐标变换，真正决定能否相撞的核心却是弹目相对运动。先用视线（LOS）把几何关系抽离出来，才能避免在复杂动力学中迷失。示例采用同一右手惯性坐标系；内部角度用弧度，展示时才换算成度。

## 本章导航

**前置知识：**三维向量、点积、叉积、矩阵乘法。
**学完后应能：**从弹目位置和速度计算距离、闭合速度、视线方向与 LOS 角速度；解释纯径向速度为什么不改变 LOS；用基准向量发现 ENU/NED、主动/被动旋转和叉乘符号错误。

本章只做几何，不急于引入制导律。原因是制导律的正确性首先取决于几何量是否正确：若 LOS 角速度符号错了，再先进的控制器也会朝错误方向加速。

## 数学补给：点积、叉积与坐标变换

点积 $\mathbf a^T\mathbf b=\lVert\mathbf a\rVert\lVert\mathbf b\rVert\cos\theta$ 提取两个向量在同一方向上的程度。相对速度在 LOS 上的投影决定距离变化，所以闭合速度使用点积。叉积 $\mathbf a\times\mathbf b$ 垂直于两向量张成的平面，其模长为 $\lVert\mathbf a\rVert\lVert\mathbf b\rVert\sin\theta$，正好提取引起 LOS 转动的横向运动。

坐标变换只改变同一物理向量的分量表示。若 $C_A^B$ 表示从 A 系分量转换到 B 系，则 $\mathbf v^B=C_A^B\mathbf v^A$。只在正交旋转矩阵条件下才有 $(C_A^B)^{-1}=(C_A^B)^T$。

## 从二维到三维：先看懂平面 LOS

二维相对位置为 $\mathbf r=[x,y]^T$，LOS 角 $\lambda=\operatorname{atan2}(y,x)$。求导得到

\[
\dot\lambda=\frac{x\dot y-y\dot x}{x^2+y^2}.
\]

分子是二维叉积的有符号结果，分母是 $R^2$。推广到三维后，标量角速度变为旋转轴向量：

\[
\boldsymbol\omega_{LOS}=\frac{\mathbf r\times\mathbf v_r}{R^2}.
\]

三维公式只是把“LOS 在纸面内转多快”推广为“LOS 绕空间中的哪根轴转多快”。平面运动时，它自然退化回二维公式。

## 全书符号速查

| 符号 | 定义 | 单位与约定 |
|---|---|---|
| $\mathbf r_M,\mathbf r_T$ | 导弹、目标惯性位置 | m，同一导航系 |
| $\mathbf r=\mathbf r_T-\mathbf r_M$ | 导弹指向目标的相对位置 | m，定义不可单独反转 |
| $\mathbf v_r=\mathbf v_T-\mathbf v_M$ | 相对速度 | m/s |
| $R=\lVert\mathbf r\rVert$ | 弹目距离 | m |
| $V_c=-\dot R$ | 闭合速度 | m/s，接近为正 |
| $\hat{\mathbf r}$ | LOS 单位向量 | 无量纲 |
| $\boldsymbol\omega_{LOS}$ | LOS 角速度 | rad/s |
| $\mathbf a_c$ | 制导所需惯性加速度 | m/s²，不是舵偏 |
| $t_{go}$ | 剩余时间估计 | s，必须有下界 |
| $P,Q,R_m$ | 状态、过程、量测协方差 | $R_m$ 与距离 $R$ 区分 |

## 2.1 坐标系转换的核心规则

坐标系是表达矢量的方式，不是矢量本身。全书统一用 $C_A^B$ 表示“输入 A 系分量、输出 B 系分量”的旋转矩阵，则

\[
\mathbf v^B=C_A^B\mathbf v^A,\qquad (C_A^B)^{-1}=(C_A^B)^T.
\]

使用前要确认矩阵下标的方向定义、主动/被动旋转约定、旋转顺序，以及角度正方向。不同教材可能采用不同欧拉角顺序；复制公式时必须连同约定复制。固定测试为：$\mathbf e_1^A=[1,0,0]^T$ 经变换后必须等于 $C_A^B$ 的第一列；若得到第一行，通常是行/列或方向写反。

ENU 与 NED 的分量关系为

\[
\begin{bmatrix}v_N\\v_E\\v_D\end{bmatrix}
=\begin{bmatrix}0&1&0\\1&0&0\\0&0&-1\end{bmatrix}
\begin{bmatrix}v_E\\v_N\\v_U\end{bmatrix}.
\]

即交换水平前两轴并反转垂向轴。位置、速度、加速度要使用同一变换；只变位置、不变速度会造成隐蔽错误。

## 2.2 相对位置、相对速度与距离变化率

令导弹位置为 $\mathbf r_M$，目标位置为 $\mathbf r_T$。统一定义“相对量从导弹指向目标”：

\[
\mathbf r=\mathbf r_T-\mathbf r_M,\quad
\mathbf v_r=\dot{\mathbf r}=\mathbf v_T-\mathbf v_M,\quad
R=\|\mathbf r\|.
\]

对 $R^2=\mathbf r^T\mathbf r$ 求导：$2R\dot R=2\mathbf r^T\mathbf v_r$，所以

\[
\dot R=\frac{\mathbf r\cdot\mathbf v_r}{R}.
\]

按以上定义，$\dot R<0$ 表示距离正在缩短，$\dot R>0$ 表示距离正在增加。若代码采用相反的相对位置方向，符号也会相反；务必把定义写在接口注释和测试中。

## 2.3 视线角速度的向量形式

视线单位向量为 $\hat{\mathbf r}=\mathbf r/R$。求导并整理：

\[
\dot{\hat{\mathbf r}}=\frac{1}{R}(I-\hat{\mathbf r}\hat{\mathbf r}^{T})\mathbf v_r
=\frac{\mathbf r\times(\mathbf v_r\times\mathbf r)}{R^3}.
\]

视线方向变化只取决于相对速度在视线法平面内的分量；沿视线方向的闭合速度不会改变视线方向。常用的视线角速度向量可写成

\[
\boldsymbol\omega_{LOS}=\frac{\mathbf r\times\mathbf v_r}{R^2}.
\]

该角速度的具体方向取决于叉乘顺序，交换次序会反号。它给出视线转动的瞬时轴与角速度，是三维比例导引的几何输入；其符号必须与速度系法向加速度约定一致。

## 2.4 方位角、仰角：不要让 atan2 搞反

以 ENU 坐标 $(E,N,U)$ 为例，一种常见定义是

\[
\mathrm{az}=\operatorname{atan2}(E,N),\qquad
\mathrm{el}=\operatorname{atan2}\left(U,\sqrt{E^2+N^2}\right).
\]

这里方位角从北向东为正。若使用 `atan2(N,E)`，得到的是另一种参考方向。`atan2(y,x)` 的参数顺序容易写反；应使用已知向量做单元测试：北向 `(0,1,0)` 方位为 0，东向 `(1,0,0)` 方位为 $\pi/2$，天向 `(0,0,1)` 仰角为 $\pi/2$。

## 2.5 C 语言实现：相对运动、视线角与单位安全

保存为 `ch2.c`，使用 `cc -std=c99 -Wall -Wextra -pedantic ch2.c -lm -o ch2`。函数约定位置单位 m、速度 m/s、时间 s，返回角度/角速度单位 rad、rad/s。

```c
#include <math.h>       /* sqrt、atan2、isfinite */
#include <stdbool.h>    /* bool */
#include <stdio.h>      /* printf */

typedef struct { double x, y, z; } Vec3; /* 三分量向量，分量坐标系必须一致 */

typedef struct {       /* 相对运动输出 */
    double range_m;    /* 距离 R，单位 m */
    double range_rate_mps; /* 距离变化率；负值代表接近，单位 m/s */
    double az_rad;     /* ENU 方位角，北起向东为正，单位 rad */
    double el_rad;     /* 仰角，水平面向上为正，单位 rad */
    Vec3 los_rate_radps; /* 视线角速度向量，单位 rad/s */
} RelState;

static Vec3 sub(Vec3 a, Vec3 b) { /* 向量相减 a-b */
    return (Vec3){a.x-b.x, a.y-b.y, a.z-b.z};
}
static double dot(Vec3 a, Vec3 b) { /* 点积 */
    return a.x*b.x + a.y*b.y + a.z*b.z;
}
static Vec3 cross(Vec3 a, Vec3 b) { /* 右手叉乘 a×b */
    return (Vec3){a.y*b.z-a.z*b.y, a.z*b.x-a.x*b.z, a.x*b.y-a.y*b.x};
}
static bool finite3(Vec3 a) { /* 过滤 NaN 和无穷值 */
    return isfinite(a.x) && isfinite(a.y) && isfinite(a.z);
}

static bool relative_state(Vec3 own_pos, Vec3 own_vel,
                           Vec3 tgt_pos, Vec3 tgt_vel,
                           RelState *out) {
    if (out == NULL || !finite3(own_pos) || !finite3(own_vel) ||
        !finite3(tgt_pos) || !finite3(tgt_vel)) return false; /* 输入校验 */
    Vec3 r = sub(tgt_pos, own_pos); /* 导弹指向目标：r = r_T - r_M */
    Vec3 vr = sub(tgt_vel, own_vel); /* 相对速度：v_r = v_T - v_M */
    double r2 = dot(r, r); /* 距离平方，避免过早开方 */
    if (!isfinite(r2) || r2 < 1.0e-12) return false; /* 零距离附近角度无定义 */
    double range = sqrt(r2); /* 距离，m */
    out->range_m = range;
    out->range_rate_mps = dot(r, vr) / range; /* R 点乘 v_r / R */
    out->az_rad = atan2(r.x, r.y); /* ENU 方位：东分量作 y，北分量作 x */
    out->el_rad = atan2(r.z, hypot(r.x, r.y)); /* 仰角，输出 rad */
    Vec3 c = cross(r, vr); /* r × v_r；叉乘顺序决定符号 */
    out->los_rate_radps = (Vec3){c.x/r2, c.y/r2, c.z/r2}; /* omega = r×v_r/R² */
    return isfinite(out->range_rate_mps) && isfinite(out->az_rad) &&
           isfinite(out->el_rad) && finite3(out->los_rate_radps); /* 输出校验 */
}

int main(void) {
    Vec3 own_p = {0.0, 0.0, 0.0}; /* 导弹位置，ENU m */
    Vec3 own_v = {300.0, 0.0, 0.0}; /* 导弹速度，ENU m/s */
    Vec3 tgt_p = {10000.0, 0.0, 1000.0}; /* 目标位置，ENU m */
    Vec3 tgt_v = {0.0, 0.0, 0.0}; /* 目标速度，ENU m/s */
    RelState q; /* 接收几何计算结果 */
    if (!relative_state(own_p, own_v, tgt_p, tgt_v, &q)) {
        puts("输入无效或距离过小，不能计算 LOS 角");
        return 1;
    }
    const double RAD_TO_DEG = 180.0 / 3.14159265358979323846; /* 只为显示换算 */
    printf("R=%.3f m, Rdot=%.3f m/s, az=%.3f deg, el=%.3f deg\n",
           q.range_m, q.range_rate_mps,
           q.az_rad*RAD_TO_DEG, q.el_rad*RAD_TO_DEG);
    return 0;
}
```

生产代码还应根据数据来源检查时间戳是否对齐。如果目标与导弹状态不是同一时刻，先做时间同步/外推，不能直接相减。极坐标角度跨越 $-\pi$ 与 $+\pi$ 时，角度差应先归一化到 $[-\pi,\pi)$，否则会出现接近 $2\pi$ 的假跳变。

## 2.6 角度单位与数值保护清单

- C 标准库 `sin/cos/atan2` 输入输出均按弧度；把度直接传给 `sin` 是常见量级错误。
- 角度只在输入/显示边界换算；算法内部不要一会儿度、一会儿弧度。
- 对 `atan2` 的零向量、距离很小、非有限输入进行处理；零距离时视线角没有物理定义。
- 用 `hypot(x,y)` 代替 `sqrt(x*x+y*y)`，可减轻极大/极小数值下的溢出或下溢。
- 角度差通过 `atan2(sin(a-b),cos(a-b))` 归一化，或用经过测试的 wrap 函数。
- 分清 `range_rate` 是距离的一阶导数（标量）还是视线角速度（向量），变量名不要都叫 `rate`。

## 2.7 常见错误与定位方法

| 错误写法或做法 | 直接后果 | 最小检查案例 |
|---|---|---|
| $\mathbf r=\mathbf r_M-\mathbf r_T$，其余公式仍按目标减导弹 | LOS 与命令方向不一致 | 导弹在原点、目标在 $+x$ 轴 |
| 写成 $\mathbf v_r\times\mathbf r$ | LOS 角速度符号反转 | $\mathbf r=[1,0,0]^T,\mathbf v_r=[0,1,0]^T$ |
| 定义 $V_c=\dot R$ 却认为接近为正 | PN 符号错误 | 目标沿 LOS 靠近时检查正负 |
| degree 直接送入 `sin/cos` | 数值失去物理意义 | $90^\circ$ 必须先转为 $\pi/2$ |
| 主动旋转与被动换系混用 | 方向或矩阵转置错误 | 单位轴旋转 $90^\circ$ 手算 |
| $R\to0$ 时继续归一化 | 除零、NaN 或角度跳变 | 设置近距终止门限 |

| 现象 | 常见原因 | 定位方法 |
|---|---|---|
| 接近时 `range_rate` 为正 | 相对位置/速度减法方向不一致 | 手算一维例子：目标在前方静止，本机向前运动，结果必须为负 |
| 方位角整体差 90° 或反号 | `atan2` 参数顺序、方位参考轴或正方向不同 | 用北/东/南/西四个基准向量逐一测试 |
| 视线角速度方向反了 | `cross(r,vr)` 写成 `cross(vr,r)` | 用一个简单平面运动手算叉乘方向 |
| 距离很大时角度乱跳 | 目标/导弹状态不同步；坐标系一个 ENU 一个 NED | 打印来源时间戳、坐标系标签；先统一转换再计算 |
| 角度突然跳约 360° | 对包裹角直接差分 | 先 wrap 角度差，输出连续角而非裸 `atan2` 结果 |
| 只有某些姿态下方向错误 | 混淆机体系、速度系和导航系分量 | 每个向量变量命名带坐标系后缀，如 `pos_enu`、`vel_body` |

### 一步一步排查坐标 bug

1. 暂停复杂仿真，造一个手算得出的几何状态。
2. 检查相对位置定义：目标减导弹还是导弹减目标。
3. 检查位置与速度使用同一坐标系、同一时刻、同一轴序。
4. 单独验证距离、距离变化率，再验证方位/仰角，最后检查叉乘量。
5. 每个转换矩阵测试正交性 $C^TC=I$ 和行列式 $\det(C)=+1$。
6. 编写北东天、向前、纯径向运动等基准测试；对角度跳变做边界测试。

## 2.8 本章交互实验

提供 ENU/NED 切换、相对位置/速度输入和三维视线显示；同步绘制距离、距离变化率、方位/仰角与 LOS 角速度。设置“纯径向运动”“纯横向运动”“坐标系误切换”“角度跨界”预设，让读者观察典型错误的特征。所有角度控件明确标出单位，内部计算结果以 rad 保存。

## 2.9 从几何量形成物理直觉

- $R$ 回答“还相距多远”，但不能单独判断是否会相撞。
- $V_c=-\dot R$ 回答“沿 LOS 方向接近得多快”，$V_c>0$ 只是正在接近，不保证脱靶量为零。
- $\boldsymbol\omega_{LOS}$ 回答“目标在视场中转得多快”。若 $V_c>0$ 且 LOS 保持不转，当前速度关系形成碰撞三角形。
- 最近接近距离由相对位置和相对速度共同决定。匀速假设下，

\[
t_{ca}=-\frac{\mathbf r^T\mathbf v_r}{\mathbf v_r^T\mathbf v_r},
\qquad
d_{ca}=\left\|\mathbf r+t_{ca}\mathbf v_r\right\|,
\]

其中只有 $t_{ca}>0$ 才代表未来最近接近。该结果是中段拦截预测的最简模型，也是理解零控脱靶量的入口。

## 2.10 本章小结与基础自测

### 计算与编程练习

> **实验能力边界：** 标注为“网页可完成”的内容仅使用本页实际显示的控件；凡要求新增制导律、噪声、延迟或约束开关而页面未显示对应控件，均标注为“需自行编程”，不应寻找不存在的滑块。

- **基础题：**说明点积为何对应径向接近，叉积为何对应 LOS 转动。
- **计算题：**对 $\mathbf r=[3,4,0]^T$、$\mathbf v_r=[-2,1,0]^T$ 手算 $R,V_c,\boldsymbol\omega_{LOS}$。
- **编程题：**为相对几何函数加入 $R<R_{min}$、非有限输入和坐标系枚举检查。
- **实验步骤：**先固定相对位置，只改变径向速度；再只改变横向速度。分别观察 $V_c$ 与 LOS 角速度，避免同时改变多个变量。

1. 目标在导弹正前方且双方同向同速，$R$、$V_c$ 和 LOS 角速度分别怎样变化？
2. $V_c>0$ 但 LOS 角速度不为零时，为什么可能从目标旁边掠过？
3. 把 $\mathbf r\times\mathbf v_r$ 写成 $\mathbf v_r\times\mathbf r$ 会造成什么后果？

**本章结论：**三维制导首先是相对运动几何。距离描述尺度，闭合速度描述径向趋势，LOS 角速度描述横向趋势；三者缺一不可。

### 参考答案与判断依据

1. 若初始距离非零，则 $R$ 保持不变，$V_c=0$，LOS 角速度也为零。这只是相对静止，不是正在形成碰撞。
2. $V_c>0$ 仅说明距离当前在减小；LOS 仍转动表示横向误差尚未消除，最近接近距离可能大于零。
3. 叉积次序交换会使结果取反，LOS 角速度方向随之反转，最终使制导指令方向错误。

### 贯穿算例：先手算相对几何

由第1章统一状态可得 $\mathbf r=\mathbf r_T-\mathbf r_M=[6000,1200,200]^T\ \mathrm m$、$\mathbf v_r=\mathbf v_T-\mathbf v_M=[-400,0,0]^T\ \mathrm{m/s}$。先自行计算，再用以下结果核对：$R\approx6122.1\ \mathrm m$、$V_c\approx392.0\ \mathrm{m/s}$、$\boldsymbol\omega_{LOS}\approx[0,-0.00213,0.01281]^T\ \mathrm{rad/s}$。若数量级或符号不同，应回查相对量定义和叉积次序。后续各章均使用这组相对状态。
