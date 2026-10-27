# 第4章 中制导、预测与阶段交接

中制导的任务是利用惯导/卫星组合导航与数据链更新，在导引头尚未稳定截获前，把导弹送入末制导可捕获域。中末制导交接本质上是**多源信息融合与控制权的无缝移交**：状态定义、量测噪声、更新频率乃至坐标系都可能在交接瞬间改变。若直接硬切换，状态跳变、协方差重置错误和数据链延迟会被末段高闭合速度放大，最终表现为指令尖峰或脱靶量发散。

## 本章导航

**前置知识：**第1章的 GNC 信息流、第2章的相对运动。
**学完后应能：**说明中制导为什么以预测和更新为主；区分数据生成时刻、接收时刻和当前计算时刻；列出导引头交接所需的几何、质量和可达性条件；解释状态与协方差为什么必须一起转换。

中制导不追求每一时刻都把导弹指向目标，而是塑造一个“末制导容易完成”的交接状态：目标位于导引头视场内、相对状态足够准确、闭合速度合理、剩余时间和过载余量充足。

## 4.1 延迟状态传播

外部观测在时间戳 \(t_s\) 给出位置 \(\hat p_s\)、速度 \(\hat v_s\) 和可选加速度 \(\hat a_s\)。当前时刻为 \(t=t_s+\tau\)。恒加速度近似来自运动方程积分：

\[
\hat p(t)=\hat p_s+\tau\hat v_s+\tfrac12\tau^2\hat a_s,\qquad
\hat v(t)=\hat v_s+\tau\hat a_s.
\]

若不使用加速度项，就隐含假设观测间隔内加速度为零。加速度未知时，传播误差会随 \(\tau\) 增大；所以必须同时保留观测时间戳、传播时长和不确定性，不能把旧数据伪装成“当前状态”。数据未来时间戳、过期、乱序或重复都要显式处理。

## 4.2 预测点与滚动更新

恒速模型下，未来时域 \(T\) 的位置预测为 \(p(T)=p_0+v_0T\)。如果参考运动体加速度不可忽略，可使用恒加速度预测。预测只对选定模型和时域成立；扩大时域并不自动提高准确度。

滚动规划的基本做法是：读取带时间戳状态，传播到当前时刻，更新预测和可行性，只执行短时间内的规划结果，再接收新数据并重算。预测模块应返回“可行/不可行/数据过期”等状态，而不只是返回一个位置向量。

## 4.3 交接不是一个距离阈值：状态与协方差必须连续

可将交接判定写成多个条件的联合逻辑：相对距离处于导引头作用范围、目标落入视场且连续跟踪、闭合速度满足末制导可达性、状态协方差低于允许值、剩余过载余量足够，并连续若干周期均满足条件。进入阈值和退出阈值应不同，形成滞回区，降低边界附近反复切换。

交接时不能只复制位置和速度。若中制导滤波器状态为 $x_m$、协方差为 $P_m$，末制导状态采用变换 $x_t=f(x_m)$，一阶协方差应按 $P_t\approx J_fP_mJ_f^T+P_{align}$ 转换，其中 $P_{align}$ 表示传感器安装误差与时间对准误差。把导引头首帧当成“绝对真值”并将协方差清零，会造成虚假高置信度和过激指令。

例如对归一化误差量 \(q\)，进入条件可设 \(q<q_{in}\)，退出条件设 \(q>q_{out}\)，其中 \(q_{in}<q_{out}\)。还可要求连续 `N_on` 个有效采样满足进入条件；无效数据则清零计数或进入明确的降级状态。门限应由传感器、动力学和控制可达性验证确定，不能只为让仿真好看而调参。

## 4.4 C 示例：带时间戳的传播与滞回交接状态机

示例用一维位置/速度展示接口原则；它不是轨迹规划器。输入单位 m、m/s、m/s²、s。采样时间由单调时钟计算，不使用可能跳变的日历时钟。代码包含 stale 数据拒绝和连续周期进入判定。

```c
#include <stdbool.h>
#include <stddef.h>  /* NULL */
#include <math.h>
#include <stdio.h>   /* printf */

typedef struct {
    double pos_m;       // 测量时刻的位置，m
    double vel_mps;     // 测量时刻的速度，m/s
    double acc_mps2;    // 区间加速度估计，m/s^2
    double stamp_s;     // 测量时间戳，单调时钟秒
    bool has_acc;       // 加速度估计是否可信可用
    bool valid;         // 上游数据质量标志
} StateSample;

typedef enum {
    MODE_MIDCOURSE = 0, // 中制导/远程跟踪阶段
    MODE_HANDOFF,       // 正在交接，等待新模式确认
    MODE_TERMINAL       // 导引头闭环末制导阶段
} GuidanceMode;

typedef struct {
    GuidanceMode mode;
    unsigned int enter_count; // 连续满足进入条件的计数
} HandoffState;

static bool propagate(const StateSample *s, double now_s,
                      double max_age_s, double *pos_now,
                      double *vel_now)
{
    if (s == NULL || pos_now == NULL || vel_now == NULL) return false;
    if (!isfinite(max_age_s) || max_age_s < 0.0) return false;
    if (!s->valid || !isfinite(now_s) || !isfinite(s->stamp_s) ||
        !isfinite(s->pos_m) || !isfinite(s->vel_mps)) return false;

    const double dt = now_s - s->stamp_s; // 数据年龄，s
    if (dt < 0.0 || dt > max_age_s) return false; // 拒绝未来/过期数据

    double a = 0.0; // 默认恒速传播，明确表达加速度假设
    if (s->has_acc) {
        if (!isfinite(s->acc_mps2)) return false;
        a = s->acc_mps2;
    }
    *pos_now = s->pos_m + s->vel_mps*dt + 0.5*a*dt*dt;
    *vel_now = s->vel_mps + a*dt;
    return isfinite(*pos_now) && isfinite(*vel_now);
}

void update_handoff(HandoffState *h, bool data_ok,
                    bool inside_entry_region,
                    bool outside_exit_region,
                    bool handoff_confirmed,
                    unsigned int required_samples)
{
    if (h == NULL || required_samples == 0U) return;

    if (!data_ok) {
        h->enter_count = 0U; // 无效观测不能累积“连续满足”次数
        if (h->mode == MODE_HANDOFF) h->mode = MODE_MIDCOURSE;
        return;
    }

    if (h->mode == MODE_MIDCOURSE) {
        if (inside_entry_region) {
            if (h->enter_count < required_samples) h->enter_count++;
            if (h->enter_count >= required_samples) h->mode = MODE_HANDOFF;
        } else {
            h->enter_count = 0U;
        }
    } else if (h->mode == MODE_HANDOFF) {
        // 交接中若离开较宽的退出区域，回退并重新规划。
        if (outside_exit_region) {
            h->mode = MODE_MIDCOURSE;
            h->enter_count = 0U;
        } else if (handoff_confirmed) {
            // 接收端确认局部传感器/状态有效后，才完成模式切换。
            h->mode = MODE_TERMINAL;
            h->enter_count = 0U;
        }
    } else { // MODE_TERMINAL
        if (outside_exit_region) {
            h->mode = MODE_MIDCOURSE; // 明确的退回逻辑，避免失锁后卡死
            h->enter_count = 0U;
        }
    }
}

int main(void)
{
    StateSample sample = {100.0, 20.0, 0.0, 1.0, false, true};
    HandoffState handoff = {MODE_MIDCOURSE, 0U};
    double pos_now = 0.0;
    double vel_now = 0.0;
    bool handoff_confirmed = false; // 演示：此样例尚未收到接收端确认
    bool ok = propagate(&sample, 1.1, 0.5, &pos_now, &vel_now);
    update_handoff(&handoff, ok, true, false, handoff_confirmed, 3U);
    printf("valid=%d pos=%.3f m vel=%.3f m/s mode=%d\n",
           ok ? 1 : 0, pos_now, vel_now, (int)handoff.mode);
    return ok ? 0 : 1;
}
```

这段代码需要由调用方补充：1) 单调时钟来源；2) 时间戳单位和数据同步；3) `inside_entry_region`、`outside_exit_region` 的物理定义；4) 只有接收端确认局部传感器/状态有效时才将 `handoff_confirmed` 置真；5) 状态迁移日志和故障码。不能把“进入交接态”误当成“新传感器已经捕获”。

## 4.5 常见错误与傻瓜式排查

| 现象 | 可能原因 | 排查步骤 |
|---|---|---|
| 预测位置跳来跳去 | 数据乱序、时戳回退、速度噪声大 | 日志同时打印源时戳、接收时戳、传播年龄；按源时戳排序检查 |
| 延迟补偿后误差反而更大 | 加速度估计偏差、单位错、传播时间过长 | 先设 `has_acc=false` 与真值恒速对比，再逐步引入加速度和延迟 |
| 交接标志频繁开关 | 只用单一阈值、没有滞回或连续样本判据 | 画出误差曲线与进入/退出阈值；验证两阈值之间不会切换 |
| 数据过期却仍产生正常指令 | 上层忽略 `propagate=false` | 注入过期、未来、NaN 样本，确认输出故障码且控制状态进入定义好的降级路径 |
| 交接后局部导航立即失锁 | 只检查了距离，没检查视场、协方差或接收端确认 | 分别记录各个交接条件；确认局部传感器数据有效后才完成状态迁移 |
| 进入阶段后无法退出 | 状态机缺少超时/退出边；异常处理只覆盖正常路径 | 为每个状态列出所有输入事件和下一状态；对断链、超时、传感器失效做注入测试 |
| 静态预测曲线被误当时间动画 | 横轴是预测时域而非真实仿真时间 | 图标题明确写“预测时域”；静态参数扫描不要播放时间轴 |

推荐调试顺序：**检查时间戳 → 检查单位 → 对比手算传播 → 验证数据过期分支 → 单独测试状态机 → 联调传感器确认 → 最后加入噪声和丢包**。每次只注入一种故障，并保留可复现日志。

## 4.6 验收清单与交互实验

交互实验至少能调节数据延迟、速度误差、加速度误差、观测丢包和进入/退出阈值；显示真值与传播值、预测不确定性、数据年龄、状态机状态及切换原因。实验必须区分“估计传播正确”与“交接条件设计合理”。

C 示例验收应覆盖：零延迟、恒速、恒加速度、超时数据、未来时间戳、无效浮点数、阈值边界连续抖动、局部传感器未确认。每种用例检查返回值和状态迁移，不只看程序是否崩溃。

## 两章共用 C 工程小贴士

- **弧度/角度**：三角函数接口使用 rad。输入数据源若给 deg，应在边界处转换一次并标注；不要变量名只写 `angle`，建议 `angle_rad`、`angle_deg`。
- **编译检查**：统一按 C99 编译：`cc -std=c99 -Wall -Wextra -Wpedantic`；开发构建可使用 AddressSanitizer/UndefinedBehaviorSanitizer。嵌入式目标若无完整 libc，应替换 `isfinite` 等接口并验证其语义。
- **浮点容差**：几何退化判断要有单位一致的阈值；不要拿一个“万能 epsilon”同时比较米、秒和角度。
- **日志**：每条控制周期至少记录时间戳、输入有效标志、参考系/模式、关键中间量、原始命令、限幅后命令和故障码。日志不要只保存最终输出。
- **模块接口**：建议纯函数计算制导几何，状态机单独管理模式，限幅器独立管理约束，驱动层负责传感器/执行器。避免把解析、全局变量、滤波和控制计算塞进一个巨大函数。
- **测试顺序**：先单元测试几何恒等式和符号，再测状态机与边界输入，再做闭环仿真，最后做带延迟、丢包和受限执行器的回放。数值仿真通过不等同于实时系统验证。

## 4.7 本章小结与基础自测

### 计算与编程练习

> **实验能力边界：** 标注为“网页可完成”的内容仅使用本页实际显示的控件；凡要求新增制导律、噪声、延迟或约束开关而页面未显示对应控件，均标注为“需自行编程”，不应寻找不存在的滑块。

- **基础题：**区分量测时间、接收时间、处理时间和控制生效时间。
- **计算题：**目标速度 200 m/s、通信延迟 0.15 s 时，匀速传播修正量是多少？
- **编程题：**实现带连续有效帧计数和进入/退出滞回的交接状态机。
- **实验步骤：**先只扫描距离门限，再加入视场和协方差条件，最后注入丢帧。统计误交接和反复切换次数，不要只观察一次成功轨迹。

1. 为什么目标状态的接收时间不能替代量测时间？
2. 只用距离门限触发末制导，可能在哪些情况下错误交接？
3. 为什么把导引头首帧协方差直接置零会产生过激控制？
4. 进入和退出阈值为什么应构成滞回，而不是使用同一个数？

**本章结论：**中制导的核心是带着不确定性做滚动预测，交接的核心是保证信息、状态和控制能力连续。交接不是一句“切到末制导”，而是一套可验证的状态机。

### 参考答案与判断依据

1. 接收时间只说明数据何时到达，不能说明状态对应何时；两者之差正是通信与处理延迟。
2. 目标可能在作用距离内但位于视场外、信噪比不足、状态未收敛或只有偶发检测，此时距离满足而信息质量不满足。
3. 协方差置零等价于宣称首帧绝对准确，会使滤波器和制导器过度相信可能带偏差的初始量测，从而产生指令跳变。
4. 滞回可防止量测在门限附近波动时反复切换模式；进入条件应严格，退出条件则保留合理裕量。

### 贯穿算例：设计交接状态机

统一场景规定：距离小于 3000 m、目标连续有效 5 帧、LOS 角位于导引头视场内且协方差迹低于门限后，才由数据链中制导转入导引头末制导；任一质量条件连续失效 3 帧则退出。交接时不要重置相对位置，而应以旧估计为先验，用首批导引头量测逐步更新，并在 0.2 s 内混合两侧指令。
