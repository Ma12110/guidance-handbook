# 第4章 中制导、预测与阶段交接

中制导的教学重点是**带着不确定性和通信延迟，把系统引导到可交接的状态区域**。这里以无人机编队会合、航天器相对导航等民用抽象任务为例。交接是状态估计、传感器可用性和控制能力的综合判定，不应简单等同于“到达某个距离”。

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

## 4.3 交接不是一个距离阈值

可将交接判定写成多个条件的联合逻辑：相对距离处于可观测范围、视线角在传感器范围内、相对速度满足后续控制能力、状态协方差低于允许值、连续若干周期均满足条件。进入阈值和退出阈值应不同，形成滞回区，降低边界附近反复切换。

例如对归一化误差量 \(q\)，进入条件可设 \(q<q_{in}\)，退出条件设 \(q>q_{out}\)，其中 \(q_{in}<q_{out}\)。还可要求连续 `N_on` 个有效采样满足进入条件；无效数据则清零计数或进入明确的降级状态。门限应由传感器、动力学和控制可达性验证确定，不能只为让仿真好看而调参。

## 4.4 C 示例：带时间戳的传播与滞回交接状态机

示例用一维位置/速度展示接口原则；它不是轨迹规划器。输入单位 m、m/s、m/s²、s。采样时间由单调时钟计算，不使用可能跳变的日历时钟。代码包含 stale 数据拒绝和连续周期进入判定。

```c
#include <stdbool.h>
#include <stddef.h>  /* NULL */
#include <math.h>

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
    MODE_LOCAL          // 局部相对导航/跟踪阶段
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
        }
        // 只有接收端确认局部传感器/状态有效后，外层才置 MODE_LOCAL。
    } else { // MODE_LOCAL
        if (outside_exit_region) {
            h->mode = MODE_MIDCOURSE; // 明确的退回逻辑，避免失锁后卡死
            h->enter_count = 0U;
        }
    }
}
```

这段代码需要由调用方补充：1) 单调时钟来源；2) 时间戳单位和数据同步；3) `inside_entry_region`、`outside_exit_region` 的物理定义；4) 交接接收端确认协议；5) 状态迁移日志和故障码。不能把“进入交接态”误当成“新传感器已经捕获”。

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
- **编译检查**：`-Wall -Wextra -Wpedantic`；开发构建可使用 AddressSanitizer/UndefinedBehaviorSanitizer。嵌入式目标若无完整 libc，应替换 `isfinite` 等接口并验证其语义。
- **浮点容差**：几何退化判断要有单位一致的阈值；不要拿一个“万能 epsilon”同时比较米、秒和角度。
- **日志**：每条控制周期至少记录时间戳、输入有效标志、参考系/模式、关键中间量、原始命令、限幅后命令和故障码。日志不要只保存最终输出。
- **模块接口**：建议纯函数计算制导几何，状态机单独管理模式，限幅器独立管理约束，驱动层负责传感器/执行器。避免把解析、全局变量、滤波和控制计算塞进一个巨大函数。
- **测试顺序**：先单元测试几何恒等式和符号，再测状态机与边界输入，再做闭环仿真，最后做带延迟、丢包和受限执行器的回放。数值仿真通过不等同于实时系统验证。
