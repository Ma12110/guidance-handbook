# 第五篇：状态估计与鲁棒制导

本篇说明：测量设备给出的通常是含噪声的位置、角度或距离，制导算法需要先从这些数据中估计运动状态；当动力学模型存在误差或外界扰动时，还需要分析制导误差是否仍能保持有界。以下例子采用通用点质量运动体和移动目标的教学模型，用于理解估计与控制概念。

## 第17章 目标状态估计

### 17.1 为什么制导前需要估计

制导律常需要相对位置、相对速度，有时还需要目标加速度。实际传感器只能提供带噪声的观测。例如，雷达可能给出距离和方位角，视觉传感器可能给出目标像面位置；这些观测并不等同于完整的三维位置和速度。状态估计器根据运动模型与观测历史，综合得到当前状态估计，并同时给出估计不确定度。

对教学仿真，可将目标在惯性坐标系中的状态写成

\[
\mathbf{x}_k=\begin{bmatrix}\mathbf{r}_{T,k}\\\mathbf{v}_{T,k}\end{bmatrix},
\qquad
\mathbf{x}_{k+1}=\mathbf{F}_k\mathbf{x}_k+\mathbf{G}_k\mathbf{a}_{T,k}+\mathbf{w}_k,
\qquad
\mathbf{z}_k=\mathbf{H}_k\mathbf{x}_k+\mathbf{v}_k.
\]

其中，\(\mathbf{r}_T\) 和 \(\mathbf{v}_T\) 分别为位置与速度，\(\mathbf{a}_T\) 是目标加速度输入或未知机动，\(\mathbf{w}_k\) 表示过程模型误差，\(\mathbf{v}_k\) 表示测量噪声。这里采用离散时间记号，采样间隔为 \(\Delta t\)。

### 17.2 匀速模型与 α–β 滤波

一维匀速状态模型为

\[
\begin{bmatrix}p_{k+1}\\v_{k+1}\end{bmatrix}
=\begin{bmatrix}1&\Delta t\\0&1\end{bmatrix}
\begin{bmatrix}p_k\\v_k\end{bmatrix}+\mathbf{w}_k,
\qquad z_k=p_k+\nu_k.
\]

α–β滤波器先按匀速模型预测，再用新测量修正位置与速度：

\[
\hat p^-_k=\hat p_{k-1}+\Delta t\hat v_{k-1},\quad
e_k=z_k-\hat p^-_k,
\]
\[
\hat p_k=\hat p^-_k+\alpha e_k,\qquad
\hat v_k=\hat v_{k-1}+\frac{\beta}{\Delta t}e_k.
\]

\(\alpha\) 与 \(\beta\) 决定对新观测的信任程度。增大增益通常能更快跟随机动，却会把更多测量噪声带入速度估计；减小增益会使曲线平滑，但可能产生滞后。α–β滤波器容易实现，适合做概念演示；它不直接维护协方差，因此不适合用来表达严格的统计置信区间。

### 17.3 卡尔曼滤波

对于线性系统，标准卡尔曼滤波假定过程噪声与测量噪声为零均值白噪声，两者互不相关，且与初始估计误差不相关；协方差分别为 \(\mathbf Q\) 和 \(\mathbf R\)。高斯条件下它给出条件均值，通常仅凭二阶统计假设则称最优线性估计。预测步骤为

\[
\hat{\mathbf{x}}^-_k=\mathbf{F}_{k-1}\hat{\mathbf{x}}_{k-1},\qquad
\mathbf{P}^-_k=\mathbf{F}_{k-1}\mathbf{P}_{k-1}\mathbf{F}_{k-1}^{\mathsf T}+\mathbf Q.
\]

测量到达后，计算创新和增益：

\[
\mathbf{y}_k=\mathbf{z}_k-\mathbf H_k\hat{\mathbf{x}}^-_k,
\quad
\mathbf S_k=\mathbf H_k\mathbf P^-_k\mathbf H_k^{\mathsf T}+\mathbf R,
\]
\[
\mathbf K_k=\mathbf P^-_k\mathbf H_k^{\mathsf T}\mathbf S_k^{-1}.
\]

状态和协方差更新为

\[
\hat{\mathbf{x}}_k=\hat{\mathbf{x}}^-_k+\mathbf K_k\mathbf y_k,
\quad
\mathbf P_k=(\mathbf I-\mathbf K_k\mathbf H_k)\mathbf P^-_k.
\]

实现时常用 Joseph 形式更新协方差，以改善有限精度下的对称性和半正定性：

\[
\mathbf P_k=(\mathbf I-\mathbf K_k\mathbf H_k)\mathbf P^-_k(\mathbf I-\mathbf K_k\mathbf H_k)^{\mathsf T}
+\mathbf K_k\mathbf R\mathbf K_k^{\mathsf T}.
\]

过程噪声协方差不是随意的“调参旋钮”：它表达未建模机动和模型误差的量级；测量噪声协方差表达传感器精度。两者应结合采样频率、传感器说明或仿真设定确定，再通过创新序列检查是否与假设相符。

### 17.3.1 增益公式怎样从最小均方误差得到

令先验误差为 \(\tilde{\mathbf x}^-=\mathbf x-\hat{\mathbf x}^-\)，并选择线性更新 \(\hat{\mathbf x}^+=\hat{\mathbf x}^-+K(\mathbf z-H\hat{\mathbf x}^-)\)。代入观测 \(\mathbf z=H\mathbf x+\boldsymbol\nu\)，得

\[
\tilde{\mathbf x}^+=(I-KH)\tilde{\mathbf x}^- -K\boldsymbol\nu.
\]

先验误差与当前测量噪声不相关时，取外积期望立即得到 Joseph 形式。将它展开，记 \(S=HP^-H^{\mathsf T}+R\)：

\[
P^+=P^- -KH P^- -P^-H^{\mathsf T}K^{\mathsf T}+KSK^{\mathsf T}.
\]

最小化 \(J(K)=\operatorname{tr}P^+\)，矩阵求导为 \(\partial J/\partial K=-2P^-H^{\mathsf T}+2KS\)。若 \(S\) 正定，令导数为零便得到 \(K=P^-H^{\mathsf T}S^{-1}\)。只有代入这个最优增益后，Joseph 式才在精确算术下化简为 \((I-KH)P^-\)；任意人为增益不能直接使用简式。实现应求解线性方程而非显式求逆。

位置和速度具有不同单位，\(\operatorname{tr}P\) 的写法严格而言还需要指定状态尺度或加权损失。更一般的目标为 \(E[\tilde x^{\mathsf T}W\tilde x]\)，取正定权重 \(W\) 后仍得到同一 Kalman 增益；量纲可由 \(W\) 或状态归一化统一。

### 17.3.2 过程噪声协方差为什么含有三次方

若连续模型为 \(\dot p=v,\ \dot v=w_a\)，白加速度噪声满足 \(E[w_a(t)w_a(s)]=q_a\delta(t-s)\)，则 \(F(\Delta t)=[[1,\Delta t],[0,1]]\)，输入向量 \(G=[0,1]^{\mathsf T}\)。离散协方差由积分给出：

\[
Q_d=\int_0^{\Delta t}e^{A\tau}Gq_aG^{\mathsf T}e^{A^{\mathsf T}\tau}d\tau
=q_a\begin{bmatrix}\Delta t^3/3&\Delta t^2/2\\\Delta t^2/2&\Delta t\end{bmatrix}.
\]

这里 \(q_a\) 是连续噪声谱密度，单位 m²/s³。若改用“每个采样区间内保持常值的随机加速度，方差 \(\sigma_a^2\)”模型，则 \(Q_d=\sigma_a^2[\Delta t^2/2,\Delta t]^{\mathsf T}[\Delta t^2/2,\Delta t]\)，出现四次方项。两种模型的参数单位和数值不同，不能把同一个数值在两式之间直接替换。

### 17.4 非线性观测与扩展卡尔曼滤波

若传感器输出距离和角度，观测关系通常是非线性的：

\[
\mathbf z_k=h(\mathbf x_k)+\mathbf v_k.
\]

扩展卡尔曼滤波（EKF）在当前状态估计处对状态转移函数 \(f\) 和观测函数 \(h\) 作一阶线性化，使用雅可比矩阵

\[
\mathbf F_{k-1}=\left.\frac{\partial f_{k-1}}{\partial\mathbf x}\right|_{\hat{\mathbf x}_{k-1}},\qquad
\mathbf H_k=\left.\frac{\partial h}{\partial\mathbf x}\right|_{\hat{\mathbf x}^-_k}
\]

这里离散映射 \(f_{k-1}\) 将 \(k-1\) 时刻状态传播到 \(k\) 时刻。把这些雅可比矩阵代入卡尔曼滤波协方差预测与更新，状态预测本身仍使用非线性映射。EKF依赖局部线性近似；强非线性、角度跨越 \(\pm\pi\)、初值偏差过大时，估计可能失准。角度创新需按观测定义环绕到如 \((-\pi,\pi]\) 的主值区间，不能直接对角度数值作普通减法。

### 17.5 目标加速度估计与机动模型

若使用恒加速度模型，可令状态为 \([p,v,a]^{\mathsf T}\)，并采用

\[
\mathbf F=\begin{bmatrix}1&\Delta t&\tfrac12\Delta t^2\\0&1&\Delta t\\0&0&1\end{bmatrix}.
\]

该模型适用于短时间内加速度近似缓变的情况。若目标会在匀速、转弯等模式间切换，可使用多个运动模型并按观测似然更新各模型概率；交互演示中可先采用两个简单模型，展示“单模型响应平滑但转弯滞后，多模型切换较快但模型概率波动”的权衡。

### 17.6 估计误差如何影响制导

设真实相对位置和速度为 \(\mathbf r,\mathbf v_r\)，估计值为 \(\hat{\mathbf r},\hat{\mathbf v}_r\)。误差分别为

\[
\tilde{\mathbf r}=\mathbf r-\hat{\mathbf r},\qquad
\tilde{\mathbf v}_r=\mathbf v_r-\hat{\mathbf v}_r.
\]

视线角速度由相对状态计算，因此位置和速度误差会共同影响制导指令。若预测时间为 \(t_g\)，匀速预测的位置误差近似包含

\[
\tilde{\mathbf r}_{\mathrm{pred}}\approx
\tilde{\mathbf r}+t_g\tilde{\mathbf v}_r,
\]

所以较远距离的预测对速度误差尤其敏感。合理做法是同时显示估计值和不确定度，不把单条估计轨迹误认为真实状态；必要时缩短预测时间并随新观测滚动修正。

### 17.7 已实现实验与范围

本页已实现一维位置/速度状态的卡尔曼滤波，采用Joseph协方差更新。可调测量噪声标准差和假设加速度噪声强度，比较位置真值、观测、估计与±2σ协方差尺度，记录RMSE和最终估计速度。

测量噪声采用固定种子的均匀分布；±2σ不是严格的高斯95%置信区间。当前没有二维转弯、α–β/EKF/多模型对比、机动时刻、采样周期或创新曲线控件。

### 17.8 本章小结

状态估计把离散、有噪声的观测转化为连续可用的运动状态。滤波效果取决于运动模型、噪声设定和观测质量；模型失配时，平滑的曲线也可能是有偏的估计。制导仿真应把测量、估计和真值分开展示，并通过误差和创新检查估计器。

## 第18章 鲁棒与滑模制导

### 18.1 不确定性从哪里来

制导模型与真实运动之间可能存在未建模气动力、风扰、质量或参数偏差、传感器误差和执行器滞后。分析时应区分：随机测量噪声、可界定的外部扰动、参数不确定性，以及未建模动态。滑模控制主要针对匹配且有界的扰动；它不会自动消除测量噪声，也不能补偿任意方向、任意频率的模型误差。

为解释基本思想，考虑一个标量误差系统：

\[
\dot e=d(t)+b u,
\]

其中 \(e\) 是教学航迹误差，\(d(t)\) 是未知扰动，且满足 \(|d(t)|\le d_{\max}\)；控制效能满足 \(b\ge b_{\min}>0\)。控制输入 \(u\) 与扰动作用在同一误差通道，这就是匹配扰动假设。

### 18.2 滑模面与到达条件

对二阶误差系统

\[
\ddot e=f(e,\dot e,t)+b u+d(t),
\]

选取滑模变量

\[
s=\dot e+\lambda e,\qquad \lambda>0.
\]

当 \(s=0\) 时，误差满足 \(\dot e+\lambda e=0\)，因此理想滑模面上的误差按指数规律趋于零。控制律可写为

\[
u=u_{\mathrm{eq}}-k\,\operatorname{sgn}(s),
\]

对已归一化的 \(b=1\)，应取 \(u_{\mathrm{eq}}=-f-\lambda\dot e\)，同时抵消名义项和滑模变量求导产生的 \(\lambda\dot e\)。若 \(b\) 已知且非零，则相应除以 \(b\)。切换项用于克服有界不确定性。若控制效能已归一化，且名义项准确，则取 \(k>d_{\max}\) 可使

\[
\dot V=s\dot s\le -(k-d_{\max})|s|<0,\qquad V=\tfrac12s^2,
\]

从而在理想执行器、连续时间及 Filippov 解的解释下，有限时间到达滑模面；到达时间满足 \(T_r\le |s(0)|/(k-d_{\max})\)。到达后误差按指数规律收敛，不等于误差有限时间归零。若 \(b\) 未归一化，则增益必须依据 \(b_{\min}\) 重新界定；不能直接照搬 \(k>d_{\max}\)。

### 18.2.1 逐步验证 Lyapunov 不等式

以 \(b=1\)、名义 \(f\) 已知、扰动匹配且 \(|d|\le d_{\max}\) 为例。先求导 \(\dot s=\ddot e+\lambda\dot e=f+u+d+\lambda\dot e\)，代入 \(u=-f-\lambda\dot e-k\operatorname{sgn}s\)，得到 \(\dot s=d-k\operatorname{sgn}s\)。因此

\[
\dot V=s d-k|s|\le |s|d_{\max}-k|s|
=-(k-d_{\max})|s|.
\]

对 \(s\ne0\)，可直接写成 \(d|s|/dt\le-(k-d_{\max})\)，积分得到前面的到达时间上界。到达面后解 \(\dot e=-\lambda e\)，得 \(e(t)=e(T_r)e^{-\lambda(t-T_r)}\)。这里证明的是有限时间到达滑模面、随后指数误差收敛，二者须区别。若采用估计的名义项，剩余建模误差也要计入扰动界。

### 18.3 边界层与抖振

符号函数在 \(s=0\) 两侧瞬时切换。数字采样、执行器带宽有限和测量噪声会使控制量快速来回切换，形成抖振。常见的连续化方式是用饱和函数：

\[
\operatorname{sat}(s/\phi)=
\begin{cases}
1,&s>\phi,\\
s/\phi,&|s|\le\phi,\\
-1,&s<-\phi,
\end{cases}
\qquad
u=u_{\mathrm{eq}}-k\operatorname{sat}(s/\phi).
\]

边界层厚度 \(\phi\) 越大，控制越平滑，但误差通常只收敛到边界层附近；越小则更接近理想滑模，却更容易放大噪声和高频动作。增益 \(k\) 也受执行器幅值限制，因此设计时应先检查可用控制幅值是否足以覆盖已知名义项与扰动上界。

### 18.3.1 边界层内究竟留下多大误差

在 \(|s|\le\phi\) 时，\(\dot s=-(k/\phi)s+d\)。对恒定扰动 \(d_0\)，稳态为 \(s_\infty=\phi d_0/k\)。对任意满足界的扰动，稳定线性系统的输入界给出 \(\limsup|s|\le\phi d_{\max}/k\)。又由 \(\dot e=-\lambda e+s\) 的积分解，得

\[
\limsup_{t\to\infty}|e(t)|\le
\frac{\phi d_{\max}}{k\lambda}.
\]

该结论要求理想连续时间模型、正确名义补偿和足够控制能力；执行器饱和、时延和测量噪声会引入额外误差项。它说明边界层宽度不是仅改变画面光滑程度，而会改变可证明的最终误差界。

### 18.4 与比例导引等方法的比较

比例导引直接利用视线变化构造横向指令，结构简单，适用于其运动学假设成立的场景。滑模设计通常围绕选定误差和不确定性界构造鲁棒项，可用于说明扰动抑制机制，但需要明确控制效能、扰动通道和幅值边界。二者并非在所有条件下谁都更优：若扰动很小且观测质量良好，简单方法可能已经足够；若扰动超出假设范围，滑模的理论保证也不再适用。

### 18.5 鲁棒性边界与工程限制

滑模稳定性结论需要逐项核对：扰动是否有界，是否匹配，控制方向是否已知，控制效能是否存在正下界，采样与执行器是否能跟上指令，状态是否可测或估计误差是否有界。若 \(u\) 饱和，实际输入可能不再满足推导所需的增益条件；若测量噪声进入 \(s\)，高增益切换可能造成明显抖振。存在执行器动态时，应把执行器状态纳入模型，或在结论中明确采用理想执行器假设。

对更一般的系统

\[
\dot{\mathbf x}=\mathbf f(\mathbf x,t)+\mathbf B(\mathbf x,t)\mathbf u+\mathbf d(\mathbf x,t),
\]

需要说明扰动能否写成 \(\mathbf B\mathbf d_u\) 的匹配形式。如果存在不匹配扰动，简单滑模切换项不能保证完全抵消，应采用适合该模型的分析方法并通过仿真验证，而不能声称“对任意扰动鲁棒”。

### 18.6 已实现实验与范围

本页已实现一维二阶对象的连续边界层滑模跟踪。可调切换增益、边界层宽度和有界匹配扰动，观察位置误差、滑模变量、控制量与扰动上界关系。

当前没有比例控制/符号滑模对比、执行器滞后、采样周期或测量噪声控件。实验采用理想控制输入，不能由这条轨迹推断实际飞行控制系统的鲁棒性。若进一步加入一阶执行器，应明确模型为 \(\tau\dot a+a=a_c\)，并重新分析增广闭环条件。

### 18.7 本章小结

鲁棒制导的结论必须与不确定性的数学描述相匹配。滑模方法的基本保证依赖匹配、有界扰动和足够控制能力；边界层能减轻抖振，却引入有限稳态误差。仿真展示性能时，应同时报告误差、控制幅值、变化率和假设是否满足。

## 符号约定

| 符号 | 含义 |
| --- | --- |
| \(\mathbf r_T,\mathbf v_T\) | 目标位置与速度 |
| \(\mathbf x_k,\hat{\mathbf x}_k\) | 离散状态与状态估计 |
| \(\mathbf P_k\) | 状态估计误差协方差 |
| \(\mathbf Q,\mathbf R\) | 过程噪声和测量噪声协方差 |
| \(e,\tilde e\) | 标量误差及误差估计量（按上下文说明） |
| \(s\) | 滑模变量 |
| \(d_{\max}\) | 扰动绝对值上界 |
| \(k,\lambda,\phi\) | 切换增益、滑模面系数、边界层厚度 |

