# 第2章 坐标系、角度与三维相对运动

> 本章所有几何示例采用同一右手惯性坐标系；示例图使用 ENU（东、北、天），但公式对任意一致的笛卡尔坐标系成立。内部角度用弧度，展示时才换算成度。

## 2.1 坐标系转换的核心规则

坐标系是表达矢量的方式，不是矢量本身。若 $C^A_B$ 表示“把 B 系坐标分量变换为 A 系分量”的旋转矩阵，则

\[
\mathbf v^A=C^A_B\mathbf v^B,\qquad (C^A_B)^{-1}=(C^A_B)^T.
\]

使用前要确认矩阵下标的方向定义、主动/被动旋转约定、旋转顺序，以及角度正方向。不同教材可能采用不同欧拉角顺序；复制公式时必须连同约定复制。

ENU 与 NED 的分量关系为

\[
\begin{bmatrix}v_N\\v_E\\v_D\end{bmatrix}
=\begin{bmatrix}0&1&0\\1&0&0\\0&0&-1\end{bmatrix}
\begin{bmatrix}v_E\\v_N\\v_U\end{bmatrix}.
\]

即交换水平前两轴并反转垂向轴。位置、速度、加速度要使用同一变换；只变位置、不变速度会造成隐蔽错误。

## 2.2 相对位置、相对速度与距离变化率

令本机（或追踪体）位置为 $\mathbf r_M$，目标/参考体位置为 $\mathbf r_T$。统一定义“相对量从本机指向目标”：

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

该角速度垂直于视线平面法向的具体方向取决于叉乘顺序。交换叉乘次序会反号。此处仅讲运动几何和软件实现，不据此构造具体拦截或伤害应用。

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
    Vec3 r = sub(tgt_pos, own_pos); /* 本机指向目标：r = r_T - r_M */
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
    Vec3 own_p = {0.0, 0.0, 0.0}; /* 本机位置，ENU m */
    Vec3 own_v = {10.0, 0.0, 0.0}; /* 本机速度，ENU m/s */
    Vec3 tgt_p = {1000.0, 0.0, 100.0}; /* 目标位置，ENU m */
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

生产代码还应根据数据来源检查时间戳是否对齐。如果目标与本机状态不是同一时刻，先做时间同步/外推，不能直接相减。极坐标角度跨越 $-\pi$ 与 $+\pi$ 时，角度差应先归一化到 $[-\pi,\pi)$，否则会出现接近 $2\pi$ 的假跳变。

## 2.6 角度单位与数值保护清单

- C 标准库 `sin/cos/atan2` 输入输出均按弧度；把度直接传给 `sin` 是常见量级错误。
- 角度只在输入/显示边界换算；算法内部不要一会儿度、一会儿弧度。
- 对 `atan2` 的零向量、距离很小、非有限输入进行处理；零距离时视线角没有物理定义。
- 用 `hypot(x,y)` 代替 `sqrt(x*x+y*y)`，可减轻极大/极小数值下的溢出或下溢。
- 角度差通过 `atan2(sin(a-b),cos(a-b))` 归一化，或用经过测试的 wrap 函数。
- 分清 `range_rate` 是距离的一阶导数（标量）还是视线角速度（向量），变量名不要都叫 `rate`。

## 2.7 常见错误与定位方法

| 现象 | 常见原因 | 定位方法 |
|---|---|---|
| 接近时 `range_rate` 为正 | 相对位置/速度减法方向不一致 | 手算一维例子：目标在前方静止，本机向前运动，结果必须为负 |
| 方位角整体差 90° 或反号 | `atan2` 参数顺序、方位参考轴或正方向不同 | 用北/东/南/西四个基准向量逐一测试 |
| 视线角速度方向反了 | `cross(r,vr)` 写成 `cross(vr,r)` | 用一个简单平面运动手算叉乘方向 |
| 距离很大时角度乱跳 | 目标/本机状态不同步；坐标系一个 ENU 一个 NED | 打印来源时间戳、坐标系标签；先统一转换再计算 |
| 角度突然跳约 360° | 对包裹角直接差分 | 先 wrap 角度差，输出连续角而非裸 `atan2` 结果 |
| 只有某些姿态下方向错误 | 混淆机体系、速度系和导航系分量 | 每个向量变量命名带坐标系后缀，如 `pos_enu`、`vel_body` |

### 一步一步排查坐标 bug

1. 暂停复杂仿真，造一个手算得出的几何状态。
2. 检查相对位置定义：目标减本机还是本机减目标。
3. 检查位置与速度使用同一坐标系、同一时刻、同一轴序。
4. 单独验证距离、距离变化率，再验证方位/仰角，最后检查叉乘量。
5. 每个转换矩阵测试正交性 $C^TC=I$ 和行列式 $\det(C)=+1$。
6. 编写北东天、向前、纯径向运动等基准测试；对角度跳变做边界测试。

## 2.8 本章交互实验

提供 ENU/NED 切换、相对位置/速度输入和三维视线显示；同步绘制距离、距离变化率、方位/仰角与 LOS 角速度。设置“纯径向运动”“纯横向运动”“坐标系误切换”“角度跨界”预设，让读者观察典型错误的特征。所有角度控件明确标出单位，内部计算结果以 rad 保存。
