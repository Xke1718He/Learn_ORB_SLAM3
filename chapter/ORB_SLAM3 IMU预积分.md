# 误差
## 1.确定性误差
* Bias：偏置
* Scale：实际数值和传感器输出值之间的比值。
* Misalignment：非正交误差。
* 标定的方法：`六面法`标定`加速度`。
## 2.随机误差
* Allan方差
* 随机游走
# IMU器件测量模型
## 1.角标符号说明
* `b`：body坐标系
* `a`：加速计(acc)
* `g`：陀螺仪(gyro)
* `w`：世界坐标系
* `d`：离散(discrete)
## 2.假设
* 假设：地球是静止的，忽略自转，运行平面为水平面，重力指向固定，模值恒定。
	1. 运行场景小；
	2. 运行时间短；
	3. 精度低(mems)；
## 3.测量模型(gyro/acc)
* 陀螺仪
$$
\tilde{\omega } ^{b}_{wb}\left ( t \right ) = \omega ^{b}_{wb}\left ( t \right ) +b_g\left (  t\right ) +\eta _g\left ( t \right ) 
$$
* 加速计

$$
a^b\left ( t \right ) = {\color{Red} R_{b}^{wT}} \left ( a^{w}-g^{w} \right )  +b_a\left (  t\right ) +\eta _a \left( t \right ) 
$$
* $\widetilde{\omega}, a^b\left ( t \right )$：测量值
* $\omega,a^{w}$：真实值
* `偏置项`和`噪声项`都位于t时刻的`载体坐标系(b)`
* $R_{b}^{w}$：从`载体坐标系(b)`到`世界坐标系(w)`的旋转
* $g^{w}$：`世界坐标系下`的重力加速度

# 预积分
估计的状态：[$R_b^w(t), p^{w}(t), v^{w}(t)$]
* $p^{w}(t)$和$v^{w}(t)$为世界坐标系下IMU的位置和速度
* [$R_b^w(t), p^{w}(t)$]将帧从`B`映射到`W`

## 1.运动方程
使用**欧拉积分**，可以得到**运动方程**的**离散**形式为：
$$
\begin{array}{l}
\mathbf{R}_{b(t+\Delta t)}^{w}=\mathbf{R}_{b(t)}^{w} {\color{Red}\operatorname{Exp} } \left({\color{Red} \boldsymbol{\omega}_{w b}^{b}(t)}  \cdot \Delta t\right) \\
\mathbf{v}^{w}(t+\Delta t)=\mathbf{v}^{w}(t)+{\color{Red} \mathbf{a}^{w}(t)}  \cdot \Delta t \\
\mathbf{p}^{w}(t+\Delta t)=\mathbf{p}^{w}(t)+\mathbf{v}^{w}(t) \cdot \Delta t+{\color{Red} \frac{1}{2} \mathbf{a}^{w}(t)}  \cdot \Delta t^{2}
\end{array}
$$
其中， $w_{wb}^b(t)$表示t时刻`角速度矢量`在b系下的坐标，$w_{wb}^b(t)\cdot \Delta t$表示`旋转矢量`在b系下的坐标，${\color{Red}\operatorname{Exp} } \left({\color{Red} \boldsymbol{\omega}_{w b}^{b}(t)}  \cdot \Delta t\right)$表示在b系下从$t+\Delta t$时刻到$t$时刻的旋转变换($R_{b \left (t + \Delta t \right)}^{b \left ( t \right)}$)。

在采样频率不变，也就是$\Delta t$不变，将**测量模型**代入**离散运动方程**为：
$$
\begin{array}{l}
\mathbf{R}_{k+1}=\mathbf{R}_{k} \cdot \operatorname{Exp}\left(\left(\tilde{\boldsymbol{\omega}}_{k}-\mathbf{b}_{k}^{g}-\boldsymbol{\eta}_{k}^{g d}\right) \cdot \Delta t\right) \\
\mathbf{v}_{k+1}=\mathbf{v}_{k}+\mathbf{R}_{k} \cdot\left(\tilde{\mathbf{f}}_{k}-\mathbf{b}_{k}^{a}-\boldsymbol{\eta}_{k}^{a d}\right) \cdot \Delta t+{\color{Red}\mathbf{g} \cdot \Delta t} \\
\mathbf{p}_{k+1}=\mathbf{p}_{k}+\mathbf{v}_{k} \cdot \Delta t+\frac{1}{2} \mathbf{g} \cdot \Delta t^{2}+\frac{1}{2} \mathbf{R}_{k} \cdot\left(\tilde{\mathbf{f}}_{k}-\mathbf{b}_{k}^{a}-\mathbf{\eta}_{k}^{a d}\right) \cdot \Delta t^{2}
\end{array}
$$
其中，
$$
\mathbf{R}(t) \doteq \mathbf{R}_{b(t)}^{w} ; \boldsymbol{\omega}(t) \doteq \boldsymbol{\omega}_{w b}^{b}(t) ; \mathbf{f}(t)=\mathbf{f}^{b}(t) ; \mathbf{v}(t) \doteq \mathbf{v}^{w}(t) ; \mathbf{p}(t) \doteq \mathbf{p}^{w}(t) ; \mathbf{g} \doteq \mathbf{g}^{w}
$$
## 2.预积分
根据`欧拉积分`，可以利用$i$时刻到$j-1$时刻的**所有IMU测量**，其中$j$时刻的$R_j, v_j, p_j$(`世界坐标系`)可以直接由$i$时刻的$R_i, v_i,p_i$(`世界坐标系`)更新得到。
* $R_j$
$$
R_{j} =R_{i} \prod_{k=i}^{j-1} Exp\left ( \left ( \widetilde{\omega}_{k} -b_k^g-\eta _{k}^{gd}   \right ) \cdot \bigtriangleup t \right ) 
$$

* $v_j$
$$
v_{j} =v_{i} +{\color{Purple} g\cdot \Delta t_{ij}} +\sum_{k=i}^{j-1} {\color{Red} R_k} \cdot \left ( \widetilde{f_k}-b_k^a-\eta _{k}^{ad}  \right )\cdot \Delta  t 
$$
其中，$\left ( \widetilde{f_k}-b_k^a-\eta _{k}^{ad}  \right )\cdot \Delta  t$相对于的是IMU坐标系，`需要转换到世界坐标系`。

* $p_j$
$$
\begin{align}
p_j & = p_i+\sum_{k  = i}^{j-1} v_k\cdot \Delta t+{\color{Purple} \frac{j-i}{2} g\cdot \Delta t^2} +\frac{1}{2} \sum_{k  = i}^{j-1}{\color{Red} R_k} \cdot \left ( \widetilde{f_k}-b_k^a-\eta _k^{ad}  \right )\cdot \Delta t^2 
\\ & = p_i+\sum_{k=i}^{j-1}\left [ v_k\cdot \Delta t+\frac{1}{2} g\cdot \Delta t^2+\frac{1}{2} {\color{Red} R_k} \cdot \left ( \widetilde{f_k}-b_k^a-\eta _k^{ad}  \right )\cdot \Delta t^2 \right ]  
\end{align}
 $$
 其中，$\Delta t_{ij}=\sum_{k=i}^{j-1} \Delta t=\left ( j-i \right ) \Delta t$


 为了避免每次更新初始的$R_i, v_i, p_i$都需要重新计算$R_j, v_j, p_j$，引出**预积分项**：
$$
\begin{aligned}
\Delta \mathbf{R}_{i j} & \triangleq \mathbf{R}_{i}^{T} \mathbf{R}_{j} \\
&=\prod_{k=i}^{j-1} \operatorname{Exp}\left(\left(\tilde{\mathbf{\omega}}_{k}-\mathbf{b}_{k}^{g}-\eta_{k}^{g d}\right) \cdot \Delta t\right) \\
\Delta \mathbf{v}_{i j} & \triangleq \mathbf{R}_{i}^{T}\left(\mathbf{v}_{j}-\mathbf{v}_{i}-\mathbf{g} \cdot \Delta t_{i j}\right) \\
&=\sum_{k=i}^{j-1} \Delta \mathbf{R}_{i k} \cdot\left(\tilde{\mathbf{f}}_{k}-\mathbf{b}_{k}^{a}-\eta_{k}^{a d}\right) \cdot \Delta t \\
\Delta \mathbf{p}_{i j} & \triangleq \mathbf{R}_{i}^{T}\left(\mathbf{p}_{j}-\mathbf{p}_{i}-\mathbf{v}_{i} \cdot \Delta t_{i j}-\frac{1}{2} \mathbf{g} \cdot \Delta t_{i j}^{2}\right) \\
&=\sum_{k=i}^{j-1}\left[\Delta \mathbf{v}_{i k} \cdot \Delta t+\frac{1}{2} \Delta \mathbf{R}_{i k} \cdot\left(\tilde{\mathbf{f}}_{k}-\mathbf{b}_{k}^{a}-\mathbf{\eta}_{k}^{a d}\right) \cdot \Delta t^{2}\right]
\end{aligned}
$$
### 预积分测量值和测量噪声
假设预积分计算区间内的`bias`相等，即$b_i^g = b_{i+1}^g =...=b_j^{g}$以及$b_i^a = b_{i+1}^a =...=b_j^a$
1. $\Delta R_{ij}$项
根据性质“当$\delta \overrightarrow{\phi }$ ”是`小量`时，有
$$
\operatorname{Exp}(\vec{\phi}+\delta \vec{\phi}) \approx \operatorname{Exp}(\vec{\phi}) \cdot \operatorname{Exp}\left(\mathbf{J}_{r}(\vec{\phi}) \cdot \delta \vec{\phi}\right)
$$
指数映射的Adjoint性质：
$$
\begin{array}{l}
\mathbf{R} \cdot \operatorname{Exp}(\vec{\phi}) \cdot \mathbf{R}^{T} = \exp \left(\mathbf{R} \vec{\phi}^{\wedge} \mathbf{R}^{T}\right) = \operatorname{Exp}(\mathbf{R} \vec{\phi}) \\
\Leftrightarrow \operatorname{Exp}(\vec{\phi}) \cdot \mathbf{R} = \mathbf{R} \cdot \operatorname{Exp}\left(\mathbf{R}^{T} \vec{\phi}\right)
\end{array}
$$
$$
\begin{aligned}
\Delta \mathbf{R}_{i j} &=\prod_{k=i}^{j-1} \operatorname{Exp}\left(\underbrace{\left(\tilde{\boldsymbol{\omega}}_{k}-\mathbf{b}_{i}^{g}\right) \Delta t}_{\overrightarrow{\phi } }\underbrace{-\boldsymbol{\eta}_{k}^{g d} \Delta t}_{\delta \overrightarrow{\phi } } \right) \\
& \stackrel{(1)}{\approx}  \prod_{k=i}^{j-1}\left\{\operatorname{Exp}\left(\left(\tilde{\boldsymbol{\omega}}_{k}-\mathbf{b}_{i}^{g}\right) \Delta t\right) \cdot \operatorname{Exp}\left(-\mathbf{J}_{r}\left(\left(\tilde{\boldsymbol{\omega}}_{k}-\mathbf{b}_{i}^{g}\right) \Delta t\right) \cdot \boldsymbol{\eta}_{k}^{g d} \Delta t\right)\right\} \\
& \stackrel{(2)}{=} \Delta \tilde{\mathbf{R}}_{i j} \cdot \prod_{k=i}^{j-1} \operatorname{Exp}\left(-\Delta \tilde{\mathbf{R}}_{k+1 j}^{T} \cdot \mathbf{J}_{r}^{k} \cdot \boldsymbol{\eta}_{k}^{g d} \Delta t\right)
\end{aligned}
$$
其中，令：
$$
\begin{array}{l} 
\mathbf{J}_{r}^{k}=\mathbf{J}_{r}\left(\left(\tilde{\boldsymbol{\omega}}_{k}-\mathbf{b}_{i}^{g}\right) \Delta t\right)\\
\Delta \tilde{\mathbf{R}}_{i j}=\prod_{k=i}^{j-1} \operatorname{Exp}\left(\left(\tilde{\boldsymbol{\omega}}_{k}-\mathbf{b}_{i}^{g}\right) \Delta t\right) \\
\operatorname{Exp}\left(-\delta \vec{\phi}_{i j}\right)=\prod_{k=i}^{j-1} \operatorname{Exp}\left(-\Delta \tilde{\mathbf{R}}_{k+1 j}^{T} \cdot \mathbf{J}_{r}^{k} \cdot \boldsymbol{\eta}_{k}^{g d} \Delta t\right)
\end{array}
$$
其中，$\Delta \widetilde{R}_{jj}=I$，则**可以得到**：
$$
\Delta \mathbf{R}_{i j} \triangleq \Delta \tilde{\mathbf{R}}_{i j} \cdot \operatorname{Exp}\left(-\delta \vec{\phi}_{i j}\right)
$$
$\Delta\widetilde{R}_{ij}$即`旋转量预积分测量值`，它由`陀螺仪测量值`和`对陀螺仪bias的估计`得到，$\delta\overrightarrow{\phi }_{ij}$为`测量的噪声`。

2. $\Delta v_{ij}$项：
将$\Delta \mathbf{R}_{i j} \triangleq \Delta \tilde{\mathbf{R}}_{i j} \cdot \operatorname{Exp}\left(-\delta \vec{\phi}_{i j}\right)$代入到$\Delta v_{ij}$中
$$
\begin{aligned}
\Delta \mathbf{v}_{i j} &=\sum_{k=i}^{j-1} \Delta \mathbf{R}_{i k} \cdot\left(\tilde{\mathbf{f}}_{k}-\mathbf{b}_{i}^{a}-\mathbf{\eta}_{k}^{a d}\right) \cdot \Delta t \\
& \approx \sum_{k=i}^{j-1} \Delta \tilde{\mathbf{R}}_{i k} \cdot \underbrace{{\color{Red} \operatorname{Exp}\left(-\delta \vec{\phi}_{i k}\right)}}_{\mathbf{I}-\delta \vec{\phi}_{i k}^{\wedge}}  \cdot\left(\tilde{\mathbf{f}}_{k}-\mathbf{b}_{i}^{a}-\boldsymbol{\eta}_{k}^{a d}\right) \cdot \Delta t \\
& \stackrel{(1)}{\approx} \sum_{k=i}^{j-1} \Delta \tilde{\mathbf{R}}_{i k} \cdot\left(\mathbf{I}-\delta \vec{\phi}_{i k}^{\wedge}\right) \cdot\left(\tilde{\mathbf{f}}_{k}-\mathbf{b}_{i}^{a}-\boldsymbol{\eta}_{k}^{a d}\right) \cdot \Delta t \\
& \stackrel{(2)}{\approx} \sum_{k=i}^{j-1}\left[\underbrace{\Delta \tilde{\mathbf{R}}_{i k} \cdot\left(\mathbf{I}-\delta \vec{\phi}_{i k}^{\wedge}\right) \cdot\left(\tilde{\mathbf{f}}_{k}-\mathbf{b}_{i}^{a}\right) \cdot \Delta t}_{{\color{Red} a^{\wedge } \cdot b = -b^{\wedge }\cdot a} } -\Delta \tilde{\mathbf{R}}_{i k} \boldsymbol{\eta}_{k}^{a d} \Delta t +\underline{\Delta\widetilde{R}_{ik}\delta \overrightarrow{\phi} _{ij}^{\wedge }\eta _{k}^{ad}} \right] \\
& \stackrel{(3)}{=} \sum_{k=i}^{j-1}\left[\Delta \tilde{\mathbf{R}}_{i k} \cdot\left(\tilde{\mathbf{f}}_{k}-\mathbf{b}_{i}^{a}\right) \cdot \Delta t+\Delta \tilde{\mathbf{R}}_{i k} \cdot\left(\tilde{\mathbf{f}}_{k}-\mathbf{b}_{i}^{a}\right)^{\wedge} \cdot \delta \vec{\phi}_{i k} \cdot \Delta t-\Delta \tilde{\mathbf{R}}_{i k} \boldsymbol{\eta}_{k}^{a d} \Delta t\right] \\
&=\underbrace{\sum_{k=i}^{j-1}\left[\Delta \tilde{\mathbf{R}}_{i k} \cdot\left(\tilde{\mathbf{f}}_{k}-\mathbf{b}_{i}^{a}\right) \cdot \Delta t\right]}_{\Delta \widetilde{v}_{ij} } +\underbrace{\sum_{k=i}^{j-1}\left[\Delta \tilde{\mathbf{R}}_{i k} \cdot\left(\tilde{\mathbf{f}}_{k}-\mathbf{b}_{i}^{a}\right)^{\wedge} \cdot \delta \vec{\phi}_{i k} \cdot \Delta t-\Delta \tilde{\mathbf{R}}_{i k} \boldsymbol{\eta}_{k}^{a d} \Delta t\right]}_{-\delta v_{ij}} 
\end{aligned}
$$
**可以得到**：
$$
\Delta v_{ij} \triangleq \Delta \widetilde{v}_{ij}-\delta v_{ij}
$$
$\Delta \widetilde{v}_{ij}$为速度增量预积分测量值，它由IMU的测量值和对bias的估计得到，$\delta v_{ij}$为测量噪声。
3. $\Delta p_{ij}$项
将$\Delta \mathbf{R}_{i j} \triangleq \Delta \tilde{\mathbf{R}}_{i j} \cdot \operatorname{Exp}\left(-\delta \vec{\phi}_{i j}\right)$和$\Delta v_{ij} \triangleq \Delta \widetilde{v}_{ij}-\delta v_{ij}$代入到$\Delta p_{ij}$中
$$
\begin{array}{l}
\Delta \mathbf{p}_{i j}=\sum_{k=i}^{j-1}\left[\Delta \mathbf{v}_{i k} \cdot \Delta t+\frac{1}{2} \Delta \mathbf{R}_{i k} \cdot\left(\tilde{\mathbf{f}}_{k}-\mathbf{b}_{i}^{a}-\boldsymbol{\eta}_{k}^{a d}\right) \cdot \Delta t^{2}\right] \\
\approx \sum_{k=i}^{j-1}\left[{\color{Red} \left(\Delta \tilde{\mathbf{v}}_{i k}-\delta \mathbf{v}_{i k}\right)}  \cdot \Delta t+\frac{1}{2} {\color{Red} \Delta \tilde{\mathbf{R}}_{i k}  \cdot \operatorname{Exp}\left(-\delta \vec{\phi}_{i k}\right)} \cdot\left(\tilde{\mathbf{f}}_{k}-\mathbf{b}_{i}^{a}-\boldsymbol{\eta}_{k}^{a d}\right) \cdot \Delta t^{2}\right] \\
\stackrel{(1)}{\approx} \sum_{k=i}^{j-1}\left[\left(\Delta \tilde{\mathbf{v}}_{i k}-\delta \mathbf{v}_{i k}\right) \cdot \Delta t+\frac{1}{2} \underbrace{\Delta \tilde{\mathbf{R}}_{i k} \cdot\left(\mathbf{I}-\delta \vec{\phi}_{i k}^{\wedge}\right)}_{\operatorname{Exp}\left (\vec{ \phi} \right)=I+\vec{\phi}^{\wedge}} \cdot\left(\tilde{\mathbf{f}}_{k}-\mathbf{b}_{i}^{a}-\mathbf{\eta}_{k}^{a d}\right) \cdot \Delta t^{2}\right] \\
\stackrel{(2)}{\approx} \sum_{k=i}^{j-1}\left[\left(\Delta \tilde{\mathbf{V}}_{i k}-\delta \mathbf{v}_{i k}\right) \cdot \Delta t+\frac{1}{2} \Delta \tilde{\mathbf{R}}_{i k} \cdot\left(\mathbf{I}-\delta \vec{\phi}_{i k}^{\wedge}\right) \cdot\left(\tilde{\mathbf{f}}_{k}-\mathbf{b}_{i}^{a}\right) \cdot \Delta t^{2}-\frac{1}{2} \Delta \tilde{\mathbf{R}}_{i k} \mathbf{\eta}_{k}^{a d} \Delta t^{2} + \underline{\frac{1}{2} \Delta \tilde{\mathbf{R}}_{i k} \delta \vec{\phi}_{i k}^{\wedge} \mathbf{\eta}_{k}^{a d} \Delta t^{2}} \right] \\
\stackrel{(3)}{=} \sum_{k=i}^{j-1}\left[\underbrace{\Delta \tilde{\mathbf{V}}_{i k} \Delta t+\frac{1}{2} \Delta \tilde{\mathbf{R}}_{i k} \cdot\left(\tilde{\mathbf{f}}_{k}-\mathbf{b}_{i}^{a}\right) \Delta t^{2}}_{\Delta \widetilde{p} _{ij}} +\underbrace{\underbrace{\frac{1}{2} \Delta \tilde{\mathbf{R}}_{i k} \cdot\left(\tilde{\mathbf{f}}_{k}-\mathbf{b}_{i}^{a}\right)^{\wedge} \delta \vec{\phi}_{i k} \Delta t^{2}}_{a^{\wedge }\cdot b=-b^{\wedge }\cdot a} -\frac{1}{2} \Delta \tilde{\mathbf{R}}_{i k} \mathbf{\eta}_{k}^{a d} \Delta t^{2}-\delta \mathbf{v}_{i k} \Delta t} _{\delta p_{ij}}\right] \\
\end{array}
$$

**可以得到**：
$$
\Delta p_{ij} \triangleq \Delta \widetilde{p}_{ij}-\delta p_{ij}
$$
$\Delta \widetilde{p}_{ij}$为速度增量预积分测量值，它由IMU的测量值和对bias的估计得到，$\delta p_{ij}$为测量噪声。
**预积分理想值**：
$$
\begin{array}{l}
\Delta \mathbf{R}_{i j} \triangleq \mathbf{R}_{i}^{T} \mathbf{R}_{j}\\
\Delta \mathbf{v}_{i j} \triangleq \mathbf{R}_{i}^{T}\left(\mathbf{v}_{j}-\mathbf{v}_{i}-\mathbf{g} \cdot \Delta t_{i j}\right)\\
 \Delta \mathbf{p}_{i j} \triangleq \mathbf{R}_{i}^{T}\left(\mathbf{p}_{j}-\mathbf{p}_{i}-\mathbf{v}_{i} \cdot \Delta t_{i j}-\frac{1}{2} \mathbf{g} \cdot \Delta t_{i j}^{2}\right)
\end{array}
$$
将预积分理想值代入：**测量值 = 理想值 + 噪声**
$$
\begin{array}{l}
\Delta \tilde{\mathbf{R}}_{i j} \approx \Delta \mathbf{R}_{i j} \operatorname{Exp}\left(\delta \vec{\phi}_{i j}\right)=\mathbf{R}_{i}^{T} \mathbf{R}_{j} \operatorname{Exp}\left(\delta \vec{\phi}_{i j}\right) \\
\Delta \tilde{\mathbf{v}}_{i j} \approx \Delta \mathbf{v}_{i j}+\delta \mathbf{v}_{i j}=\mathbf{R}_{i}^{T}\left(\mathbf{v}_{j}-\mathbf{v}_{i}-\mathbf{g} \cdot \Delta t_{i j}\right)+\delta \mathbf{v}_{i j} \\
\Delta \tilde{\mathbf{p}}_{i j} \approx \Delta \mathbf{p}_{i j}+\delta \mathbf{p}_{i j}=\mathbf{R}_{i}^{T}\left(\mathbf{p}_{j}-\mathbf{p}_{i}-\mathbf{v}_{i} \cdot \Delta t_{i j}-\frac{1}{2} \mathbf{g} \cdot \Delta t_{i j}^{2}\right)+\delta \mathbf{p}_{i j}
\end{array}
$$
## 噪声分析
1. $\delta \vec{\phi}_{i j}$：
$$
\operatorname{Exp}\left(-\delta \vec{\phi}_{i j}\right)=\prod_{k=i}^{j-1} \operatorname{Exp}\left(-\Delta \tilde{\mathbf{R}}_{k+1 j}^{T} \cdot \mathbf{J}_{r}^{k} \cdot \boldsymbol{\eta}_{k}^{g d} \Delta t\right)
$$
其中，$\mathbf{J}_{r}^{k}=\mathbf{J}_{r}\left(\left(\tilde{\boldsymbol{\omega}}_{k}-\mathbf{b}_{i}^{g}\right) \Delta t\right)$

对上式两边取对数：
$$
\delta \vec{\phi}_{i j}=-\log \left(\prod_{k=i}^{j-1} \operatorname{Exp}\left(-\Delta \tilde{\mathbf{R}}_{k+1 j}^{T} \cdot \mathbf{J}_{r}^{k} \cdot \boldsymbol{\eta}_{k}^{g d} \Delta t\right)\right)
$$
利用**性质**：当$\delta \vec{\phi}$是小量时，$\log{\left(\operatorname{Exp}\left(\vec{\phi} \right) \cdot \operatorname{Exp}\left(\delta\vec{\phi}\right)\right)}=\vec{\phi }+\operatorname{J}^{-1}_{r}\left (\vec{\phi } \right ) \cdot  \delta \vec{\phi }$,  其中$\boldsymbol{\eta}_{k}^{g d}$是小量，$\xi =\Delta \tilde{\mathbf{R}}_{k+1 j}^{T} \cdot \mathbf{J}_{r}^{k} \cdot \boldsymbol{\eta}_{k}^{g d} \Delta t$是小量，于是$\operatorname{J}^{-1}_{r}\left (\xi_{k} \right )\approx I$

$$
\begin{aligned}
\delta \vec{\phi}_{i j} & =-\log \left(\prod_{k=i}^{j-1} \operatorname{Exp}\left(-\xi_{k}\right)\right) \\
& =-\log \left(\operatorname{Exp}\left(-\xi_{i}\right) \prod_{k=i+1}^{j-1} \operatorname{Exp}\left(-\xi_{k}\right)\right) \\
& \approx-\left(-\xi_{i}+\mathbf{I} \cdot \log \left(\prod_{k=i+1}^{j-1} \operatorname{Exp}\left(-\xi_{k}\right)\right)\right)=\xi_{i}-\log \left(\prod_{k=i+1}^{j-1} \operatorname{Exp}\left(-\xi_{k}\right)\right) \\
& =\xi_{i}-\log \left(\operatorname{Exp}\left(-\xi_{i+1}\right) \prod_{k=i+2}^{j-1} \operatorname{Exp}\left(-\xi_{k}\right)\right) \\
& \approx \xi_{i}+\xi_{i+1}-\log \left(\prod_{k=i+2}^{j-1} \operatorname{Exp}\left(-\xi_{k}\right)\right) \\
& \approx \cdots \\
& \approx \sum_{k=i}^{j-1} \xi_{k}
\end{aligned}
$$
即：
$$
\delta\vec{\phi}_{ij}\approx \sum_{k=i}^{j-1} {\Delta\tilde{R}_{k+1 j}^{T}\operatorname{J}_{r}^{k}\eta_{k}^{gd}\Delta t   }
$$
由于$\Delta\tilde{R}_{k+1 j}^{T}$、$\operatorname{J}_{r}^{k}$、$\Delta t$都是已知量，而$\eta_{k}^{gd}$为零均值高斯噪声，因此$\delta \vec{\phi}_{i j}$也为零均值高斯噪声
2. $\delta v_{ij}$
$$
\delta \mathbf{v}_{i j}=\sum_{k=i}^{j-1}\left[\Delta \tilde{\mathbf{R}}_{i k} \mathbf{\eta}_{k}^{a d} \Delta t-\Delta \tilde{\mathbf{R}}_{i k} \cdot\left(\tilde{\mathbf{f}}_{k}-\mathbf{b}_{i}^{a}\right)^{\wedge} \cdot \delta \vec{\phi}_{i k} \cdot \Delta t\right]
$$
3. $\delta p_{ij}$
$$
\delta \mathbf{p}_{i j}=\sum_{k=i}^{j-1}\left[\delta \mathbf{v}_{i k} \Delta t-\frac{1}{2} \Delta \tilde{\mathbf{R}}_{i k} \cdot\left(\tilde{\mathbf{f}}_{k}-\mathbf{b}_{i}^{a}\right)^{\wedge} \delta \vec{\phi}_{i k} \Delta t^{2}+\frac{1}{2} \Delta \tilde{\mathbf{R}}_{i k} \mathbf{\eta}_{k}^{a d} \Delta t^{2}\right]
$$
## 噪声更新
1. $\delta\overrightarrow{\phi}_{ij-1}\to  \delta\overrightarrow{\phi}_{ij}$
$$
\begin{aligned}
\delta \vec{\phi}_{i j} & =\sum_{k=i}^{j-1} \Delta \tilde{\mathbf{R}}_{k+1 j}^{T} \mathbf{J}_{r}^{k} \boldsymbol{\eta}_{k}^{g d} \Delta t \\
& =\sum_{k=i}^{j-2} {\color{Red} \Delta \tilde{\mathbf{R}}_{k+1 j}^{T}}  \mathbf{J}_{r}^{k} \boldsymbol{\eta}_{k}^{g d} \Delta t+\underbrace{\Delta \tilde{\mathbf{R}}_{j j}^{T}}_{I}  \mathbf{J}_{r}^{j-1} \boldsymbol{\eta}_{j-1}^{g d} \Delta t \\
&= \sum_{j-2}^{k=i}{\color{Red}  \left(\Delta \tilde{\mathbf{R}}_{k+1 j-1} \Delta \tilde{\mathbf{R}}_{j-1 j}\right)^{T}}  \mathbf{J}_{r}^{k} \boldsymbol{\eta}_{k}^{g d} \Delta t+\mathbf{J}_{r}^{j-1} \boldsymbol{\eta}_{j-1}^{g d} \Delta t \\
& =\Delta \tilde{\mathbf{R}}_{j j-1} \sum_{k=i}^{j-2} \Delta \tilde{\mathbf{R}}_{k+1 j-1}^{T} \mathbf{J}_{r}^{k} \boldsymbol{\eta}_{k}^{g d} \Delta t+\mathbf{J}_{r}^{j-1} \boldsymbol{\eta}_{j-1}^{g d} \Delta t \\
& =\Delta \tilde{\mathbf{R}}_{j j-1} \delta \vec{\phi}_{i j-1}+\mathbf{J}_{r}^{j-1} \boldsymbol{\eta}_{j-1}^{g d} \Delta t
\end{aligned}
$$
2. $\delta v_{ij-1} \to \delta v_{ij}$
$$
\begin{aligned}
\delta \mathbf{v}_{i j}= & \sum_{k=i}^{j-1}\left[\Delta \tilde{\mathbf{R}}_{i k} \mathbf{\eta}_{k}^{a d} \Delta t-\Delta \tilde{\mathbf{R}}_{i k} \cdot\left(\tilde{\mathbf{f}}_{k}-\mathbf{b}_{i}^{a}\right)^{\wedge} \cdot \delta \vec{\phi}_{i k} \cdot \Delta t\right] \\
= & \sum_{k=i}^{j-2}\left[\Delta \tilde{\mathbf{R}}_{i k} \boldsymbol{\eta}_{k}^{a d} \Delta t-\Delta \tilde{\mathbf{R}}_{i k} \cdot\left(\tilde{\mathbf{f}}_{k}-\mathbf{b}_{i}^{a}\right)^{\wedge} \cdot \delta \vec{\phi}_{i k} \cdot \Delta t\right] \ldots \\
& +\Delta \tilde{\mathbf{R}}_{i j-1} \boldsymbol{\eta}_{j-1}^{a d} \Delta t-\Delta \tilde{\mathbf{R}}_{i j-1} \cdot\left(\tilde{\mathbf{f}}_{j-1}-\mathbf{b}_{i}^{a}\right)^{\wedge} \cdot \delta \vec{\phi}_{i j-1} \cdot \Delta t \\
= & \delta \mathbf{v}_{i j-1}+\Delta \tilde{\mathbf{R}}_{i j-1} \boldsymbol{\eta}_{j-1}^{a d} \Delta t-\Delta \tilde{\mathbf{R}}_{i j-1} \cdot\left(\tilde{\mathbf{f}}_{j-1}-\mathbf{b}_{i}^{a}\right)^{\wedge} \cdot \delta \vec{\phi}_{i j-1} \cdot \Delta t
\end{aligned}
$$
3. $\delta p_{ij-1} \to \delta p_{ij}$
$$
\begin{aligned}
\delta \mathbf{p}_{i j} & =\sum_{k=i}^{j-1}\left[\delta \mathbf{v}_{i k} \Delta t-\frac{1}{2} \Delta \tilde{\mathbf{R}}_{i k} \cdot\left(\tilde{\mathbf{f}}_{k}-\mathbf{b}_{i}^{a}\right)^{\wedge} \delta \vec{\phi}_{i k} \Delta t^{2}+\frac{1}{2} \Delta \tilde{\mathbf{R}}_{i k} \boldsymbol{\eta}_{k}^{a d} \Delta t^{2}\right] \\
& =\delta \mathbf{p}_{i j-1}+\delta \mathbf{v}_{i j-1} \Delta t-\frac{1}{2} \Delta \tilde{\mathbf{R}}_{i j-1} \cdot\left(\tilde{\mathbf{f}}_{j-1}-\mathbf{b}_{i}^{a}\right)^{\wedge} \delta \vec{\phi}_{i j-1} \Delta t^{2}+\frac{1}{2} \Delta \tilde{\mathbf{R}}_{i j-1} \boldsymbol{\eta}_{j-1}^{a d} \Delta t^{2}
\end{aligned}
$$
**矩阵C**:
$$
\begin{aligned}
\chi_{i j} & =\left[\eta_{i j}^{T}, \gamma_{i j}^{T}\right]^{T} \sim N\left(0_{15 \times 1}, C_{i j}\right) \\
\eta_{i j} & =\left[\delta \phi_{i j}^{T}, \delta v_{i j}^{T}, \delta p_{i j}^{T}\right]^{T} \sim N\left(0_{9 \times 1}, \Sigma_{i j}^{\eta}\right) \\
\gamma_{i j} & =\left[\delta b_{i j}^{a T}, \delta b_{i j}^{g T}\right]^{T} \sim N\left(0_{6 \times 1}, \Sigma_{i j}^{\gamma}\right)
\end{aligned}
$$
根据$\delta \phi$、$\delta v$、$\delta p$、$\delta b_{a}$、$\delta b_{g}$，可以得到：
$$
C_{i j}=\left(\begin{array}{cc}
\Sigma_{i j}^{\eta} & 0_{9 \times 6} \\
0_{6 \times 9} & \Sigma_{i j}^{\gamma}
\end{array}\right)_{15 \times 15}
$$


$\boldsymbol{\eta}_{i j}^{\Delta} \triangleq\left[\begin{array}{lll}\delta \vec{\phi}_{i j}^{T} & \delta \mathbf{v}_{i j}^{T} & \delta \mathbf{p}_{i j}^{T}\end{array}\right]^{T}$的**递推形式**如下
$$
\begin{aligned}
\boldsymbol{\eta}_{i j}^{\Delta}= &{\underbrace{ {\left[\begin{array}{ccc}
\Delta \tilde{\mathbf{R}}_{j j-1} & \mathbf{0} & \mathbf{0} \\
-\Delta \tilde{\mathbf{R}}_{i j-1} \cdot\left(\tilde{\mathbf{f}}_{j-1}-\mathbf{b}_{i}^{a}\right)^{\wedge} \Delta t & \mathbf{I} & \mathbf{0} \\
-\frac{1}{2} \Delta \tilde{\mathbf{R}}_{i j-1} \cdot\left(\tilde{\mathbf{f}}_{j-1}-\mathbf{b}_{i}^{a}\right)^{\wedge} \Delta t^{2} & \Delta t \mathbf{I} & \mathbf{I}
\end{array}\right]}}  _{A_{j-1} }\boldsymbol{\eta}_{i j-1}^{\Delta} \cdots } \\
& +\underbrace{\left[\begin{array}{cc}
\mathbf{J}_{r}^{j-1} \Delta t & \mathbf{0} \\
\mathbf{0} & \Delta \tilde{\mathbf{R}}_{i j-1} \Delta t \\
\mathbf{0} & \frac{1}{2} \Delta \tilde{\mathbf{R}}_{i j-1} \Delta t^{2}
\end{array}\right]}_{B_{j-1}}\underbrace{ \begin{bmatrix}
 \left(\eta_{j-1}^{gd}\right)^T\\
\left(\eta_{j-1}^{ad}\right)^T
\end{bmatrix} } _{\boldsymbol{\eta}_{j-1}^{d}}
\end{aligned}
$$
$\Sigma_{ij}^{\eta}$的**递推形式**如下：
$$
\boldsymbol{\Sigma}_{i j}^{\eta}=\mathbf{A}_{j-1} \boldsymbol{\Sigma}_{i j-1}^{\eta} \mathbf{A}_{j-1}^{T}+\mathbf{B}_{j-1} \boldsymbol{\Sigma}_{\boldsymbol{\eta}} \mathbf{B}_{j-1}^{T}
$$
其中：$\Sigma_{\eta}$为IMU**噪声**的协方差矩阵


$\gamma_{i j} =\left[\delta b_{i j}^{a T}, \delta b_{i j}^{g T}\right]^{T}$的**递推形式**如下
$$
\gamma_{i j}=\gamma_{i j-1}+\sigma_{j-1}^{d}
$$
其中：
$$
 \sigma_{k}^{d}=\left[\left(\sigma_{k}^{b g d}\right)^{T} \left(\sigma_{k}^{b a d}\right)^{T}\right]^{T} 
$$
$\Sigma_{i j}^{\gamma}$的**递推形式**如下：
$$
\begin{array}{l}
 \Sigma_{i j}^{\gamma}=\Sigma_{i j-1}^{\gamma}+\Sigma_{\gamma}
\end{array}
$$
## 预积分测量值更新
当bias不发生变化时
* $\Delta \tilde{R} _{ij-1}\to \Delta \tilde{R}_{ij}$
$$
\begin{array}{l}
\Delta \tilde{\mathbf{R}}_{i j}=\prod_{k=i}^{j-1} \operatorname{Exp}\left(\left(\tilde{\boldsymbol{\omega}}_{k}-\mathbf{b}_{i}^{g}\right) \Delta t\right) \\
=\prod_{k=i}^{j-2} \operatorname{Exp}\left(\left(\tilde{\boldsymbol{\omega}}_{k}-\mathbf{b}_{i}^{g}\right) \Delta t\right)\cdot \operatorname{Exp}\left(\left(\tilde{\boldsymbol{\omega}}_{j-1}-\mathbf{b}_{i}^{g}\right) \Delta t\right) \\
=\Delta \tilde{\mathbf{R}}_{i j-1}\cdot\Delta \tilde{\mathbf{R}}_{j-1 j}
\end{array}
$$
* $\Delta \tilde{v}_{ij-1} \to \Delta \tilde{v}_{ij}$
$$
\begin{array}{l}
\Delta \tilde{\mathbf{v}}_{i j} \triangleq \sum_{k=i}^{j-1}\left[\Delta \tilde{\mathbf{R}}_{i k} \cdot\left(\tilde{\mathbf{f}}_{k}-\mathbf{b}_{i}^{a}\right) \cdot \Delta t\right] \\
\triangleq \sum_{k=i}^{j-2}\left[\Delta \tilde{\mathbf{R}}_{i k} \cdot\left(\tilde{\mathbf{f}}_{k}-\mathbf{b}_{i}^{a}\right) \cdot \Delta t\right] + \Delta \tilde{\mathbf{R}}_{i j-1} \cdot\left(\tilde{\mathbf{f}}_{j-1}-\mathbf{b}_{i}^{a}\right) \cdot \Delta t\\
\triangleq \Delta \tilde{\mathbf{v}}_{i j-1}+ \Delta \tilde{\mathbf{R}}_{i j-1} \cdot\left(\tilde{\mathbf{f}}_{j-1}-\mathbf{b}_{i}^{a}\right) \cdot \Delta t
\end{array}
$$

* $\Delta \tilde{p}_{ij-1} \to \Delta \tilde{p}_{ij}$
$$
\begin{array}{l}
\Delta \tilde{\mathbf{p}}_{i j} \triangleq \sum_{k = i}^{j-1}\left[\Delta \tilde{\mathbf{v}}_{i k} \Delta t+\frac{1}{2} \Delta \tilde{\mathbf{R}}_{i k} \cdot\left(\tilde{\mathbf{f}}_{k}-\mathbf{b}_{i}^{a}\right) \Delta t^{2}\right] \\
\triangleq \sum_{k = i}^{j-2}\left[\Delta \tilde{\mathbf{v}}_{i k} \Delta t+\frac{1}{2} \Delta \tilde{\mathbf{R}}_{i k} \cdot\left(\tilde{\mathbf{f}}_{k}-\mathbf{b}_{i}^{a}\right) \Delta t^{2}\right] + \Delta \tilde{\mathbf{v}}_{i j-1} \Delta t+\frac{1}{2} \Delta \tilde{\mathbf{R}}_{i j-1} \cdot\left(\tilde{\mathbf{f}}_{j-1}-\mathbf{b}_{i}^{a}\right) \Delta t^{2} \\
\triangleq \Delta \tilde{\mathbf{p}}_{i j-1}  + \Delta \tilde{\mathbf{v}}_{i j-1} \Delta t+\frac{1}{2} \Delta \tilde{\mathbf{R}}_{i j-1} \cdot\left(\tilde{\mathbf{f}}_{j-1}-\mathbf{b}_{i}^{a}\right) \Delta t^{2}
\end{array}
$$
## 预积分测量值更新
当bias发生变化时，利用**线性化**来进行bias变化时预积分项的**一阶近似更新**
* bias更新
	* $\bar{b}$：旧的bias
	* $\hat{b}$：新的bias
	* $\delta b$：bias更新量
$$
\begin{array}{c}
\hat{b} _{i}^{g}\gets \bar{b}  _{i}^{g}+\delta b_{i}^{g} \\
\hat{b} _{i}^{a}\gets \bar{b}  _{i}^{a}+\delta b_{i}^{a}
\end{array}
$$
* 一阶近似更新：
$$
\begin{array}{l}
\Delta \tilde{\mathbf{R}}_{i j}\left(\hat{\mathbf{b}}_{i}^{g}\right) \approx \Delta \tilde{\mathbf{R}}_{i j}\left(\overline{\mathbf{b}}_{i}^{g}\right) \cdot \operatorname{Exp}\left(\frac{\partial \Delta \overline{\mathbf{R}}_{i j}}{\partial \overline{\mathbf{b}}^{g}} \delta \mathbf{b}_{i}^{g}\right) \\
\Delta \tilde{\mathbf{v}}_{i j}\left(\hat{\mathbf{b}}_{i}^{g}, \hat{\mathbf{b}}_{i}^{a}\right) \approx \Delta \tilde{\mathbf{v}}_{i j}\left(\overline{\mathbf{b}}_{i}^{g}, \overline{\mathbf{b}}_{i}^{a}\right)+\frac{\partial \Delta \overline{\mathbf{v}}_{i j}}{\partial \overline{\mathbf{b}}^{g}} \delta \mathbf{b}_{i}^{g}+\frac{\partial \Delta \overline{\mathbf{v}}_{i j}}{\partial \overline{\mathbf{b}}^{a}} \delta \mathbf{b}_{i}^{a} \\
\Delta \tilde{\mathbf{p}}_{i j}\left(\hat{\mathbf{b}}_{i}^{g}, \hat{\mathbf{b}}_{i}^{a}\right) \approx \Delta \tilde{\mathbf{p}}_{i j}\left(\overline{\mathbf{b}}_{i}^{g}, \overline{\mathbf{b}}_{i}^{a}\right)+\frac{\partial \Delta \overline{\mathbf{p}}_{i j}}{\partial \overline{\mathbf{b}}^{g}} \delta \mathbf{b}_{i}^{g}+\frac{\partial \Delta \overline{\mathbf{p}}_{i j}}{\partial \overline{\mathbf{b}}^{a}} \delta \mathbf{b}_{i}^{a}
\end{array}
$$
* 符号简化：
$$
\begin{array}{l}
\Delta \hat{\mathbf{R}}_{i j} \doteq \Delta \tilde{\mathbf{R}}_{i j}\left(\hat{\mathbf{b}}_{i}^{g}\right), \Delta \overline{\mathbf{R}}_{i j} \doteq \Delta \tilde{\mathbf{R}}_{i j}\left(\overline{\mathbf{b}}_{i}^{g}\right) \\
\Delta \hat{\mathbf{v}}_{i j} \doteq \Delta \tilde{\mathbf{v}}_{i j}\left(\hat{\mathbf{b}}_{i}^{g}, \hat{\mathbf{b}}_{i}^{a}\right), \Delta \overline{\mathbf{v}}_{i j} \doteq \Delta \tilde{\mathbf{v}}_{i j}\left(\overline{\mathbf{b}}_{i}^{g}, \overline{\mathbf{b}}_{i}^{a}\right) \\
\Delta \hat{\mathbf{p}}_{i j} \doteq \Delta \tilde{\mathbf{p}}_{i j}\left(\hat{\mathbf{b}}_{i}^{g}, \hat{\mathbf{b}}_{i}^{a}\right), \Delta \overline{\mathbf{p}}_{i j} \doteq \Delta \tilde{\mathbf{p}}_{i j}\left(\overline{\mathbf{b}}_{i}^{g}, \overline{\mathbf{b}}_{i}^{a}\right)
\end{array}
$$
* 更新公式可以简化为：
$$
\begin{array}{l}
\Delta \hat{\mathbf{R}}_{i j} \approx \Delta \overline{\mathbf{R}}_{i j} \cdot \operatorname{Exp}\left(\frac{\partial \Delta \overline{\mathbf{R}}_{i j}}{\partial \overline{\mathbf{b}}^{g}} \delta \mathbf{b}_{i}^{g}\right) \\
\Delta \hat{\mathbf{v}}_{i j} \approx \Delta \overline{\mathbf{v}}_{i j}+\frac{\partial \Delta \overline{\mathbf{v}}_{i j}}{\partial \overline{\mathbf{b}}^{g}} \delta \mathbf{b}_{i}^{g}+\frac{\partial \Delta \overline{\mathbf{v}}_{i j}}{\partial \overline{\mathbf{b}}^{a}} \delta \mathbf{b}_{i}^{a} \\
\Delta \hat{\mathbf{p}}_{i j} \approx \Delta \overline{\mathbf{p}}_{i j}+\frac{\partial \Delta \overline{\mathbf{p}}_{i j}}{\partial \overline{\mathbf{b}}^{g}} \delta \mathbf{b}_{i}^{g}+\frac{\partial \Delta \overline{\mathbf{p}}_{i j}}{\partial \overline{\mathbf{b}}^{a}} \delta \mathbf{b}_{i}^{a}
\end{array}
$$
* $\frac{\partial \Delta \overline{\mathbf{R}}_{i j}}{\partial \overline{\mathbf{b}}^{g}}$

$$
\begin{aligned}
\Delta \hat{\mathbf{R}}_{i j} & =\Delta \tilde{\mathbf{R}}_{i j}\left(\hat{\mathbf{b}}_{i}^{g}\right) \\
& =\prod_{k=i}^{j-1} \operatorname{Exp}\left(\left(\tilde{\mathbf{\omega}}_{k}-\hat{\mathbf{b}}_{i}^{g}\right) \Delta t\right) \\
& =\prod_{k=i}^{j-1} \operatorname{Exp}\left(\left(\tilde{\mathbf{\omega}}_{k}-\left(\overline{\mathbf{b}}_{i}^{g}+\delta \mathbf{b}_{i}^{g}\right)\right) \Delta t\right) \\
& =\prod_{k=i}^{j-1} \operatorname{Exp}\left(\left(\tilde{\mathbf{\omega}}_{k}-\overline{\mathbf{b}}_{i}^{g}\right) \Delta t-\delta \mathbf{b}_{i}^{g} \Delta t\right) \\
& \stackrel{(1)}{\approx}\underbrace{ \prod_{k=i}^{j-1}\left(\operatorname{Exp}\left(\left(\tilde{\mathbf{\omega}}_{k}-\overline{\mathbf{b}}_{i}^{g}\right) \Delta t\right) \cdot \operatorname{Exp}\left(-\mathbf{J}_{r}^{k} \delta \mathbf{b}_{i}^{g} \Delta t\right)\right)}_{\operatorname{Exp}(\vec{\phi}+\delta \vec{\phi}) \approx \operatorname{Exp}(\vec{\phi}) \cdot \operatorname{Exp}\left(\mathbf{J}_{r}(\vec{\phi}) \cdot \delta \vec{\phi}\right) and \operatorname{Exp}(\vec{\phi}) \cdot \mathbf{R} = \mathbf{R} \cdot \operatorname{Exp}\left(\mathbf{R}^{T} \vec{\phi}\right)} \\ 
& =\Delta \overline{\mathbf{R}}_{i j} \prod_{k=i}^{j-1} \operatorname{Exp}\left(-\Delta \overline{\mathbf{R}}_{k+1 j}^{T} \mathbf{J}_{r}^{k} \delta \mathbf{b}_{i}^{g} \Delta t\right)
\end{aligned}
$$
可以得到，
$$
\frac{\partial \Delta \overline{\mathbf{R}}_{i j}}{\partial \overline{\mathbf{b}}^{g}}=\sum_{k=i}^{j-1}\left(-\Delta \overline{\mathbf{R}}_{k+1 j}^{T} \mathbf{J}_{r}^{k} \Delta t\right)
$$
其中，
$$
\mathbf{J}_{r}^{k}=\mathbf{J}_{r}\left(\left(\tilde{\boldsymbol{\omega}}_{k}-\mathbf{b}_{i}^{g}\right) \Delta t\right)
$$
* $\frac{\partial \Delta \overline{\mathbf{v}}_{i j}}{\partial \overline{\mathbf{b}}^{g}}$和$\frac{\partial \Delta \overline{\mathbf{v}}_{i j}}{\partial \overline{\mathbf{b}}^{a}}$
将$\Delta \hat{\mathbf{R}}_{i j}=\Delta \overline{\mathbf{R}}_{i j} \operatorname{Exp}\left(\sum_{k=i}^{j-1}\left(-\Delta \overline{\mathbf{R}}_{k+1 j}^{T} \mathbf{J}_{r}^{k} \delta \mathbf{b}_{i}^{g} \Delta t\right)\right)$代入
$$
\begin{array}{l}
\Delta \hat{\mathbf{v}}_{i j}=\Delta \tilde{\mathbf{v}}_{i j}\left(\hat{\mathbf{b}}_{i}^{g}, \hat{\mathbf{b}}_{i}^{a}\right) \\
=\sum_{k=i}^{j-1}\left[\Delta \tilde{\mathbf{R}}_{i k}\left(\hat{\mathbf{b}}_{i}^{g}\right) \cdot\left(\tilde{\mathbf{f}}_{k}-\hat{\mathbf{b}}_{i}^{a}\right) \Delta t\right] \\
\approx \sum_{k=i}^{j-1}\left[\Delta \overline{\mathbf{R}}_{i k} \cdot \operatorname{Exp}\left(\frac{\partial \Delta \overline{\mathbf{R}}_{i k}}{\partial \overline{\mathbf{b}}^{g}} \delta \mathbf{b}_{i}^{g}\right) \cdot\left(\tilde{\mathbf{f}}_{k}-\overline{\mathbf{b}}_{i}^{a}-\delta \mathbf{b}_{i}^{a}\right) \Delta t\right] \\
\stackrel{(1)}{\approx} \sum_{k=i}^{j-1}\left[\Delta \overline{\mathbf{R}}_{i k} \cdot\left(\mathbf{I}+\left(\frac{\partial \Delta \overline{\mathbf{R}}_{i k}}{\partial \overline{\mathbf{b}}^{g}} \delta \mathbf{b}_{i}^{g}\right)^{\wedge}\right) \cdot\left(\tilde{\mathbf{f}}_{k}-\overline{\mathbf{b}}_{i}^{a}-\delta \mathbf{b}_{i}^{a}\right) \Delta t\right] \\
=\sum_{k=i}^{j-1}\left[\Delta \overline{\mathbf{R}}_{i k} \cdot\left(\tilde{\mathbf{f}}_{k}-\overline{\mathbf{b}}_{i}^{a}\right) \Delta t-\Delta \overline{\mathbf{R}}_{i k} \delta \mathbf{b}_{i}^{a} \Delta t+\Delta \overline{\mathbf{R}}_{i k} \cdot\left(\frac{\partial \Delta \overline{\mathbf{R}}_{i k}}{\partial \overline{\mathbf{b}}^{g}} \delta \mathbf{b}_{i}^{g}\right)^{\wedge}\left(\tilde{\mathbf{f}}_{k}-\overline{\mathbf{b}}_{i}^{a}\right) \Delta t-\Delta \overline{\mathbf{R}}_{i k} \cdot\left(\frac{\partial \Delta \overline{\mathbf{R}}_{i k}}{\partial \overline{\mathbf{b}}^{g}} \delta \mathbf{b}_{i}^{g}\right)^{\wedge} \delta \mathbf{b}_{i}^{a} \Delta t\right] \\
\stackrel{(2)}{\approx} \Delta \overline{\mathbf{v}}_{i j}+\sum_{k=i}^{j-1}\left\{-\left[\Delta \overline{\mathbf{R}}_{i k} \Delta t\right] \delta \mathbf{b}_{i}^{a}-\left[\Delta \overline{\mathbf{R}}_{i k} \cdot\left(\tilde{\mathbf{f}}_{k}-\overline{\mathbf{b}}_{i}^{a}\right)^{\wedge} \frac{\partial \Delta \overline{\mathbf{R}}_{i k}}{\partial \overline{\mathbf{b}}^{g}} \Delta t\right] \delta \mathbf{b}_{i}^{g}\right\} \\
\end{array}
$$
**可以得到**：
$$
\begin{aligned}
\frac{\partial \Delta \overline{\mathbf{v}}_{i j}}{\partial \overline{\mathbf{b}}^{g}} & =-\sum_{k=i}^{j-1}\left(\Delta \overline{\mathbf{R}}_{i k} \cdot\left(\tilde{\mathbf{f}}_{k}-\overline{\mathbf{b}}_{i}^{a}\right)^{\wedge} \frac{\partial \Delta \overline{\mathbf{R}}_{i k}}{\partial \overline{\mathbf{b}}^{g}} \Delta t\right) \\
\frac{\partial \Delta \overline{\mathbf{v}}_{i j}}{\partial \overline{\mathbf{b}}^{a}} & =-\sum_{k=i}^{j-1}\left(\Delta \overline{\mathbf{R}}_{i k} \Delta t\right)
\end{aligned}
$$
* $\frac{\partial \Delta \overline{\mathbf{p}}_{i j}}{\partial \overline{\mathbf{b}}^{g}}$和 $\frac{\partial \Delta \overline{\mathbf{p}}_{i j}}{\partial \overline{\mathbf{b}}^{a}}$
$$
\begin{aligned}
\Delta \hat{\mathbf{p}}_{i j} & =\Delta \tilde{\mathbf{p}}_{i j}\left(\hat{\mathbf{b}}_{i}^{g}, \hat{\mathbf{b}}_{i}^{a}\right) \\
& =\sum_{k=i}^{j-1}\left[\Delta \tilde{\mathbf{v}}_{i k}\left(\hat{\mathbf{b}}_{i}^{g}, \hat{\mathbf{b}}_{i}^{a}\right) \Delta t+\frac{1}{2} \Delta \tilde{\mathbf{R}}_{i k}\left(\hat{\mathbf{b}}_{i}^{g}\right) \cdot\left(\tilde{\mathbf{f}}_{k}-\hat{\mathbf{b}}_{i}^{a}\right) \Delta t^{2}\right] \\
& =\underbrace{\sum_{k=i}^{j-1}\left[\Delta \tilde{\mathbf{v}}_{i k}\left(\hat{\mathbf{b}}_{i}^{g}, \hat{\mathbf{b}}_{i}^{a}\right) \Delta t\right]}_{1}+\underbrace{\frac{1}{2} \sum_{k=i}^{j-1}\left[\Delta \tilde{\mathbf{R}}_{i k}\left(\hat{\mathbf{b}}_{i}^{g}\right) \cdot\left(\tilde{\mathbf{f}}_{k}-\hat{\mathbf{b}}_{i}^{a}\right) \Delta t^{2}\right]}_{2}
\end{aligned}
$$
对于**1**和**2**分别推导：
$$
\begin{aligned}
(1) & =\sum_{k=i}^{j-1}\left[\left(\Delta \overline{\mathbf{v}}_{i k}+\frac{\partial \Delta \overline{\mathbf{v}}_{i k}}{\partial \overline{\mathbf{b}}^{g}} \delta \mathbf{b}_{i}^{g}+\frac{\partial \Delta \overline{\mathbf{v}}_{i k}}{\partial \overline{\mathbf{b}}^{a}} \delta \mathbf{b}_{i}^{a}\right) \Delta t\right] \\
& =\sum_{k=i}^{j-1}\left[\Delta \overline{\mathbf{v}}_{i k} \Delta t+\left(\frac{\partial \Delta \overline{\mathbf{v}}_{i k}}{\partial \overline{\mathbf{b}}^{g}} \Delta t\right) \delta \mathbf{b}_{i}^{g}+\left(\frac{\partial \Delta \overline{\mathbf{v}}_{i k}}{\partial \overline{\mathbf{b}}^{a}} \Delta t\right) \delta \mathbf{b}_{i}^{a}\right]
\end{aligned}
$$
$$
\begin{aligned}
(2) & \approx \frac{\Delta t^{2}}{2} \sum_{k=i}^{j-1}\left[\Delta \tilde{\mathbf{R}}_{i k}\left(\hat{\mathbf{b}}_{i}^{g}\right) \cdot\left(\tilde{\mathbf{f}}_{k}-\hat{\mathbf{b}}_{i}^{a}\right)\right] \\
& =\frac{\Delta t^{2}}{2} \sum_{k=i}^{j-1}\left[\Delta \overline{\mathbf{R}}_{i k} \cdot \operatorname{Exp}\left(\frac{\partial \Delta \overline{\mathbf{R}}_{i k}}{\partial \overline{\mathbf{b}}^{g}} \delta \mathbf{b}_{i}^{g}\right) \cdot\left(\tilde{\mathbf{f}}_{k}-\overline{\mathbf{b}}_{i}^{a}-\delta \mathbf{b}_{i}^{a}\right)\right] \\
& \approx \frac{\Delta t^{2}}{2} \sum_{k=i}^{j-1}\left[\Delta \overline{\mathbf{R}}_{i k} \cdot\left(\mathbf{I}+\left(\frac{\partial \Delta \overline{\mathbf{R}}_{i k}}{\partial \overline{\mathbf{b}}^{g}} \delta \mathbf{b}_{i}^{g}\right)^{\wedge}\right) \cdot\left(\tilde{\mathbf{f}}_{k}-\overline{\mathbf{b}}_{i}^{a}-\delta \mathbf{b}_{i}^{a}\right)\right] \\
& \approx \frac{\Delta t^{2}}{2} \sum_{k=i}^{j-1}\left[\Delta \overline{\mathbf{R}}_{i k} \cdot\left(\tilde{\mathbf{f}}_{k}-\overline{\mathbf{b}}_{i}^{a}\right)-\Delta \overline{\mathbf{R}}_{i k} \delta \mathbf{b}_{i}^{a}-\Delta \overline{\mathbf{R}}_{i k} \cdot\left(\tilde{\mathbf{f}}_{k}-\overline{\mathbf{b}}_{i}^{a}\right)^{\wedge} \frac{\partial \Delta \overline{\mathbf{R}}_{i k}}{\partial \overline{\mathbf{b}}^{g}} \delta \mathbf{b}_{i}^{g}\right]
\end{aligned}
$$
将**1**与**2**组合：
$$
\begin{array}{l}
\Delta \hat{\mathbf{p}}_{i j} =\sum_{k=i}^{j-1}\left\{\left[\Delta \overline{\mathbf{v}}_{i k} \Delta t+\frac{1}{2} \Delta \overline{\mathbf{R}}_{i k} \cdot\left(\tilde{\mathbf{f}}_{k}-\overline{\mathbf{b}}_{i}^{a}\right) \Delta t^{2}\right]+\left[\frac{\partial \Delta \overline{\mathbf{v}}_{i k}}{\partial \overline{\mathbf{b}}^{g}} \Delta t-\frac{1}{2} \Delta \overline{\mathbf{R}}_{i k} \cdot\left(\tilde{\mathbf{f}}_{k}-\overline{\mathbf{b}}_{i}^{a}\right)^{\wedge} \frac{\partial \Delta \overline{\mathbf{R}}_{i k}}{\partial \overline{\mathbf{b}}^{g}} \Delta t^{2}\right] \delta \mathbf{b}_{i}^{g}+\left[\frac{\partial \Delta \overline{\mathbf{v}}_{i k}}{\partial \overline{\mathbf{b}}^{a}} \Delta t-\frac{1}{2} \Delta \overline{\mathbf{R}}_{i k} \Delta t^{2}\right] \delta \mathbf{b}_{i}^{a}\right\} \\
=\Delta \overline{\mathbf{p}}_{i j}+\left\{\sum_{k=i}^{j-1}\left[\frac{\partial \Delta \overline{\mathbf{v}}_{i k}}{\partial \overline{\mathbf{b}}^{g}} \Delta t-\frac{1}{2} \Delta \overline{\mathbf{R}}_{i k} \cdot\left(\tilde{\mathbf{f}}_{k}-\overline{\mathbf{b}}_{i}^{a}\right)^{\wedge} \frac{\partial \Delta \overline{\mathbf{R}}_{i k}}{\partial \overline{\mathbf{b}}^{g}} \Delta t^{2}\right]\right\} \delta \mathbf{b}_{i}^{g}+\left\{\sum_{k=i}^{j-1}\left[\frac{\partial \Delta \overline{\mathbf{v}}_{i k}}{\partial \overline{\mathbf{b}}^{a}} \Delta t-\frac{1}{2} \Delta \overline{\mathbf{R}}_{i k} \Delta t^{2}\right]\right\} \delta \mathbf{b}_{i}^{a}
\end{array}
$$
**得到**：
$$
\begin{aligned}
\frac{\partial \Delta \overline{\mathbf{p}}_{i j}}{\partial \overline{\mathbf{b}}^{g}} & =\sum_{k=i}^{j-1}\left[\frac{\partial \Delta \overline{\mathbf{v}}_{i k}}{\partial \overline{\mathbf{b}}^{g}} \Delta t-\frac{1}{2} \Delta \overline{\mathbf{R}}_{i k} \cdot\left(\tilde{\mathbf{f}}_{k}-\overline{\mathbf{b}}_{i}^{a}\right)^{\wedge} \frac{\partial \Delta \overline{\mathbf{R}}_{i k}}{\partial \overline{\mathbf{b}}^{g}} \Delta t^{2}\right] \\
\frac{\partial \Delta \overline{\mathbf{p}}_{i j}}{\partial \overline{\mathbf{b}}^{a}} & =\sum_{k=i}^{j-1}\left[\frac{\partial \Delta \overline{\mathbf{v}}_{i k}}{\partial \overline{\mathbf{b}}^{a}} \Delta t-\frac{1}{2} \Delta \overline{\mathbf{R}}_{i k} \Delta t^{2}\right]
\end{aligned}
$$
## Jacobian更新
* $\frac{\partial \Delta \overline{\mathbf{v}}_{i j-1}}{\partial \overline{\mathbf{b}}^{g}} \to \frac{\partial \Delta \overline{\mathbf{v}}_{i j}}{\partial \overline{\mathbf{b}}^{g}}$
 $$
\begin{array}{l}
 \frac{\partial \Delta \overline{\mathbf{v}}_{i j}}{\partial \overline{\mathbf{b}}^{g}} =-\sum_{k=i}^{j-1}\left(\Delta \overline{\mathbf{R}}_{i k} \cdot\left(\tilde{\mathbf{f}}_{k}-\overline{\mathbf{b}}_{i}^{a}\right)^{\wedge} \frac{\partial \Delta \overline{\mathbf{R}}_{i k}}{\partial \overline{\mathbf{b}}^{g}} \Delta t\right) \\
=-\sum_{k=i}^{j-2}\left(\Delta \overline{\mathbf{R}}_{i k} \cdot\left(\tilde{\mathbf{f}}_{k}-\overline{\mathbf{b}}_{i}^{a}\right)^{\wedge} \frac{\partial \Delta \overline{\mathbf{R}}_{i k}}{\partial \overline{\mathbf{b}}^{g}} \Delta t\right) - \Delta \overline{\mathbf{R}}_{i j-1} \cdot\left(\tilde{\mathbf{f}}_{j-1}-\overline{\mathbf{b}}_{i}^{a}\right)^{\wedge} \frac{\partial \Delta \overline{\mathbf{R}}_{i j-1}}{\partial \overline{\mathbf{b}}^{g}} \Delta t \\
= \frac{\partial \Delta \overline{\mathbf{v}}_{i j-1}}{\partial \overline{\mathbf{b}}^{g}}-{\color{Green}  \Delta \overline{\mathbf{R}}_{i j-1}}  \cdot\left(\tilde{\mathbf{f}}_{j-1}-\overline{\mathbf{b}}_{i}^{a}\right)^{\wedge} {\color{Yellow} \frac{\partial \Delta \overline{\mathbf{R}}_{i j-1}}{\partial \overline{\mathbf{b}}^{g}}}  \Delta t
\end{array}
 $$
* $\frac{\partial \Delta \overline{\mathbf{v}}_{i j-1}}{\partial \overline{\mathbf{b}}^{a}} \to \frac{\partial \Delta \overline{\mathbf{v}}_{i j}}{\partial \overline{\mathbf{b}}^{a}}$ 

$$
\begin{array}{l}
\frac{\partial \Delta \overline{\mathbf{v}}_{i j}}{\partial \overline{\mathbf{b}}^{a}} =-\sum_{k=i}^{j-1}\left(\Delta \overline{\mathbf{R}}_{i k} \Delta t\right) \\
 =-\sum_{k=i}^{j-2}\left(\Delta \overline{\mathbf{R}}_{i k} \Delta t\right)-\Delta \overline{\mathbf{R}}_{i j-1} \Delta t \\
=\frac{\partial \Delta \overline{\mathbf{v}}_{i j-1}}{\partial \overline{\mathbf{b}}^{a}} -{\color{Green} \Delta \overline{\mathbf{R}}_{i j-1} } \Delta t 
\end{array}
$$
* $\frac{\partial \Delta \overline{\mathbf{p}}_{i j-1}}{\partial \overline{\mathbf{b}}^{g}} \to \frac{\partial \Delta \overline{\mathbf{p}}_{i j}}{\partial \overline{\mathbf{b}}^{g}}$

$$
\begin{array}{l}
\frac{\partial \Delta \overline{\mathbf{p}}_{i j}}{\partial \overline{\mathbf{b}}^{g}}  =\sum_{k=i}^{j-1}\left[\frac{\partial \Delta \overline{\mathbf{v}}_{i k}}{\partial \overline{\mathbf{b}}^{g}} \Delta t-\frac{1}{2} \Delta \overline{\mathbf{R}}_{i k} \cdot\left(\tilde{\mathbf{f}}_{k}-\overline{\mathbf{b}}_{i}^{a}\right)^{\wedge} \frac{\partial \Delta \overline{\mathbf{R}}_{i k}}{\partial \overline{\mathbf{b}}^{g}} \Delta t^{2}\right] \\
=\sum_{k=i}^{j-2}\left[\frac{\partial \Delta \overline{\mathbf{v}}_{i k}}{\partial \overline{\mathbf{b}}^{g}} \Delta t-\frac{1}{2} \Delta \overline{\mathbf{R}}_{i k} \cdot\left(\tilde{\mathbf{f}}_{k}-\overline{\mathbf{b}}_{i}^{a}\right)^{\wedge} \frac{\partial \Delta \overline{\mathbf{R}}_{i k}}{\partial \overline{\mathbf{b}}^{g}} \Delta t^{2}\right] + {\color{Red} \frac{\partial \Delta \overline{\mathbf{v}}_{i j-1}}{\partial \overline{\mathbf{b}}^{g}}}  \Delta t-\frac{1}{2} {\color{Green} \Delta \overline{\mathbf{R}}_{i j-1}}  \cdot\left(\tilde{\mathbf{f}}_{j-1}-\overline{\mathbf{b}}_{i}^{a}\right)^{\wedge} {\color{Yellow} \frac{\partial \Delta \overline{\mathbf{R}}_{i j-1}}{\partial \overline{\mathbf{b}}^{g}} } \Delta t^{2} \\
=\frac{\partial \Delta \overline{\mathbf{p}}_{i j-1}}{\partial \overline{\mathbf{b}}^{g}} + {\color{Red} \frac{\partial \Delta \overline{\mathbf{v}}_{i j-1}}{\partial \overline{\mathbf{b}}^{g}}}  \Delta t-\frac{1}{2} {\color{Green} \Delta \overline{\mathbf{R}}_{i j-1}}  \cdot\left(\tilde{\mathbf{f}}_{j-1}-\overline{\mathbf{b}}_{i}^{a}\right)^{\wedge} {\color{Yellow} \frac{\partial \Delta \overline{\mathbf{R}}_{i j-1}}{\partial \overline{\mathbf{b}}^{g}} } \Delta t^{2}
\end{array}
$$

* $\frac{\partial \Delta \overline{\mathbf{p}}_{i j-1}}{\partial \overline{\mathbf{b}}^{a}} \to \frac{\partial \Delta \overline{\mathbf{p}}_{i j}}{\partial \overline{\mathbf{b}}^{a}}$

$$
\begin{array}{l}
\frac{\partial \Delta \overline{\mathbf{p}}_{i j}}{\partial \overline{\mathbf{b}}^{a}}  =\sum_{k=i}^{j-1}\left[\frac{\partial \Delta \overline{\mathbf{v}}_{i k}}{\partial \overline{\mathbf{b}}^{a}} \Delta t-\frac{1}{2} \Delta \overline{\mathbf{R}}_{i k} \Delta t^{2}\right] \\
 =\sum_{k=i}^{j-2}\left[\frac{\partial \Delta \overline{\mathbf{v}}_{i k}}{\partial \overline{\mathbf{b}}^{a}} \Delta t-\frac{1}{2} \Delta \overline{\mathbf{R}}_{i k} \Delta t^{2}\right] + {\color{Red} \frac{\partial \Delta \overline{\mathbf{v}}_{i j-1}}{\partial \overline{\mathbf{b}}^{a}}}  \Delta t-\frac{1}{2} {\color{Green} \Delta \overline{\mathbf{R}}_{i j-1} } \Delta t^{2} \\
=\frac{\partial \Delta \overline{\mathbf{p}}_{i j-1}}{\partial \overline{\mathbf{b}}^{a}} + {\color{Red} \frac{\partial \Delta \overline{\mathbf{v}}_{i j-1}}{\partial \overline{\mathbf{b}}^{a}}}  \Delta t-\frac{1}{2} {\color{Green} \Delta \overline{\mathbf{R}}_{i j-1} } \Delta t^{2} 
\end{array}
$$
* $\frac{\partial \Delta \overline{\mathbf{R}}_{i j-1}}{\partial \overline{\mathbf{b}}^{g}} \to \frac{\partial \Delta \overline{\mathbf{R}}_{i j}}{\partial \overline{\mathbf{b}}^{g}}$

$$
\begin{array}{l}
\frac{\partial \Delta \overline{\mathbf{R}}_{i j}}{\partial \overline{\mathbf{b}}^{g}}=\sum_{k=i}^{j-1}\left(-\Delta \overline{\mathbf{R}}_{k+1 j}^{T} \mathbf{J}_{r}^{k} \Delta t\right) \\
=\sum_{k=i}^{j-2}\left(-\Delta \overline{\mathbf{R}}_{k+1 j}^{T} \mathbf{J}_{r}^{k} \Delta t\right)
-\Delta \overline{\mathbf{R}}_{j j}^{T} \mathbf{J}_{r}^{k} \Delta t \\
=\sum_{k=i}^{j-2}\left(-\left(\Delta \overline{\mathbf{R}}_{k+1 j-1}\Delta \overline{\mathbf{R}}_{j-1 j} \right)^{T}\mathbf{J}_{r}^{k} \Delta t\right)
-\Delta \overline{\mathbf{R}}_{j j}^{T} \mathbf{J}_{r}^{k} \Delta t \\
=-\Delta \overline{\mathbf{R}}_{ j-1 j}^{T}\cdot \sum_{k=i}^{j-2}\left(\Delta \overline{\mathbf{R}}_{k+1 j-1}^{T} \mathbf{J}_{r}^{k} \Delta t\right)
-\Delta \overline{\mathbf{R}}_{j j}^{T} \mathbf{J}_{r}^{k} \Delta t \\
= -\Delta \overline{\mathbf{R}}_{ j-1 j}^{T}\cdot \frac{\partial \Delta \overline{\mathbf{R}}_{i j-1}}{\partial \overline{\mathbf{b}}^{g}}
-\underbrace{\Delta \overline{\mathbf{R}}_{j j}^{T}}_{I}  \mathbf{J}_{r}^{k} \Delta t
\end{array}
$$
## 残差
* 预积分的测量值采用**一阶近似**修正，免去了积分重新运算，降低计算量
* 状态包含：$\mathbf{R}_{i}, \mathbf{p}_{i}, \mathbf{v}_{i}, \mathbf{R}_{j}, \mathbf{p}_{j}, \mathbf{v}_{j}, \delta \mathbf{b}_{i}^{g}, \delta \mathbf{b}_{i}^{a}$，其中关于bias的是**bias的偏差**
$$
\begin{aligned}
\mathbf{r}_{\Delta \mathbf{R}_{i j}} & \triangleq \log \left\{\left[\Delta \tilde{\mathbf{R}}_{i j}\left(\overline{\mathbf{b}}_{i}^{g}\right) \cdot \operatorname{Exp}\left(\frac{\partial \Delta \overline{\mathbf{R}}_{i j}}{\partial \overline{\mathbf{b}}^{g}} \delta \mathbf{b}_{i}^{g}\right)\right]^{T} \cdot \mathbf{R}_{i}^{T} \mathbf{R}_{j}\right\} \\
& \triangleq \log \left[\left(\Delta \hat{\mathbf{R}}_{i j}\right)^{T} \Delta \mathbf{R}_{i j}\right] \\
\mathbf{r}_{\Delta \mathbf{v}_{i j}} & \triangleq \mathbf{R}_{i}^{T}\left(\mathbf{v}_{j}-\mathbf{v}_{i}-\mathbf{g} \cdot \Delta t_{i j}\right)-\left[\Delta \tilde{\mathbf{v}}_{i j}\left(\overline{\mathbf{b}}_{i}^{g}, \overline{\mathbf{b}}_{i}^{a}\right)+\frac{\partial \Delta \overline{\mathbf{v}}_{i j}}{\partial \overline{\mathbf{b}}^{g}} \delta \mathbf{b}_{i}^{g}+\frac{\partial \Delta \overline{\mathbf{v}}_{i j}}{\partial \overline{\mathbf{b}}^{a}} \delta \mathbf{b}_{i}^{a}\right] \\
& \triangleq \Delta \mathbf{v}_{i j}-\Delta \hat{\mathbf{v}}_{i j} \\
\mathbf{r}_{\Delta \mathbf{p}_{i j}} & \triangleq \mathbf{R}_{i}^{T}\left(\mathbf{p}_{j}-\mathbf{p}_{i}-\mathbf{v}_{i} \cdot \Delta t_{i j}-\frac{1}{2} \mathbf{g} \cdot \Delta t_{i j}^{2}\right)-\left[\Delta \tilde{\mathbf{p}}_{i j}\left(\overline{\mathbf{b}}_{i}^{g}, \overline{\mathbf{b}}_{i}^{a}\right)+\frac{\partial \Delta \overline{\mathbf{p}}_{i j}}{\partial \overline{\mathbf{b}}^{g}} \delta \mathbf{b}_{i}^{g}+\frac{\partial \Delta \overline{\mathbf{p}}_{i j}}{\partial \overline{\mathbf{b}}^{a}} \delta \mathbf{b}_{i}^{a}\right] \\
& \triangleq \Delta \mathbf{p}_{i j}-\Delta \hat{\mathbf{p}}_{i j}
\end{aligned}
$$
### $\mathbf{r}_{\Delta \mathbf{R}_{i j}}$
* $\mathbf{r}_{\Delta \mathbf{R}_{i j}}$关于$\delta \vec{\phi}_{i}$的jacobian
$$
\begin{aligned}
\mathbf{r}_{\Delta \mathbf{R}_{i j}}\left(\mathbf{R}_{i} \operatorname{Exp}\left(\delta \vec{\phi}_{i}\right)\right) & =\log \left[\left(\Delta \hat{\mathbf{R}}_{i j}\right)^{T}\left(\mathbf{R}_{i} \operatorname{Exp}\left(\delta \vec{\phi}_{i}\right)\right)^{T} \mathbf{R}_{j}\right] \\
& \stackrel{(1)}{=} \log \left[\left(\Delta \hat{\mathbf{R}}_{i j}\right)^{T} \operatorname{Exp}\left(-\delta \vec{\phi}_{i}\right) \mathbf{R}_{i}^{T} \mathbf{R}_{j}\right] \\
& \stackrel{2}{=} \log \left[\left(\Delta \hat{\mathbf{R}}_{i j}\right)^{T} \mathbf{R}_{i}^{T} \mathbf{R}_{j} \operatorname{Exp}\left(-\mathbf{R}_{j}^{T} \mathbf{R}_{i} \delta \vec{\phi}_{i}\right)\right] \\
& =\log \left\{\operatorname{Exp}\left[\log \left(\left(\Delta \hat{\mathbf{R}}_{i j}\right)^{T} \mathbf{R}_{i}^{T} \mathbf{R}_{j}\right)\right] \cdot \operatorname{Exp}\left(-\mathbf{R}_{j}^{T} \mathbf{R}_{i} \delta \vec{\phi}\right)\right\} \\
& =\log \left[\operatorname{Exp}\left(\mathbf{r}_{\Delta \mathbf{R}_{i j}}\left(\mathbf{R}_{i}\right)\right) \cdot \operatorname{Exp}\left(-\mathbf{R}_{j}^{T} \mathbf{R}_{i} \delta \vec{\phi}_{i}\right)\right] \\
& \stackrel{3}{\approx} \mathbf{r}_{\Delta \mathbf{R}_{i j}}\left(\mathbf{R}_{i}\right)-\mathbf{J}_{r}^{-1}\left(\mathbf{r}_{\Delta \mathbf{R}_{i j}}\left(\mathbf{R}_{i}\right)\right) \mathbf{R}_{j}^{T} \mathbf{R}_{i} \delta \vec{\phi}_{i} \\
& \stackrel{4}{=} \mathbf{r}_{\Delta \mathbf{R}_{i j}}-\mathbf{J}_{r}^{-1}\left(\mathbf{r}_{\Delta \mathbf{R}_{i j}}\right) \mathbf{R}_{j}^{T} \mathbf{R}_{i} \delta \vec{\phi}_{i}
\end{aligned}
$$
可以得到：
$$
\frac{\partial \mathbf{r}_{\Delta \mathbf{R}_{i j}}}{\partial \delta \vec{\phi}_{i}}=-\mathbf{J}_{r}^{-1}\left(\mathbf{r}_{\Delta \mathbf{R}_{i j}}\right) \mathbf{R}_{j}^{T} \mathbf{R}_{i}
$$
* $\mathbf{r}_{\Delta \mathbf{R}_{i j}}$关于$\delta \vec{\phi}_{j}$的jacobian
$$
\begin{aligned}
\mathbf{r}_{\Delta \mathbf{R}_{i j}}\left(\mathbf{R}_{j} \operatorname{Exp}\left(\delta \vec{\phi}_{j}\right)\right) & =\log \left[\left(\Delta \hat{\mathbf{R}}_{i j}\right)^{T} \mathbf{R}_{i}^{T} \mathbf{R}_{j} \operatorname{Exp}\left(\delta \vec{\phi}_{j}\right)\right] \\
& =\log \left\{\operatorname{Exp}\left[\log \left(\left(\Delta \hat{\mathbf{R}}_{i j}\right)^{T} \mathbf{R}_{i}^{T} \mathbf{R}_{j}\right)\right] \cdot \operatorname{Exp}\left(\delta \vec{\phi}_{j}\right)\right\} \\
& =\log \left\{\operatorname{Exp}\left(\mathbf{r}_{\Delta \mathbf{R}_{i j}}\left(\mathbf{R}_{j}\right)\right) \cdot \operatorname{Exp}\left(\delta \vec{\phi}_{j}\right)\right\} \\
& \stackrel{(1)}{\approx} \mathbf{r}_{\Delta \mathbf{R}_{i j}}\left(\mathbf{R}_{j}\right)+\mathbf{J}_{r}^{-1}\left(\mathbf{r}_{\Delta \mathbf{R}_{i j}}\left(\mathbf{R}_{j}\right)\right) \delta \vec{\phi}_{j} \\
& \stackrel{2}{=} \mathbf{r}_{\Delta \mathbf{R}_{i j}}+\mathbf{J}_{r}^{-1}\left(\mathbf{r}_{\Delta \mathbf{R}_{i j}}\right) \delta \vec{\phi}_{j}
\end{aligned}
$$
可以得到：
$$
\frac{\partial \mathbf{r}_{\Delta \mathbf{R}_{i j}}}{\partial \delta \vec{\phi}_{j}}=\mathbf{J}_{r}^{-1}\left(\mathbf{r}_{\Delta \mathbf{R}_{i j}}\right)
$$
* $\mathbf{r}_{\Delta \mathbf{R}_{i j}}$关于$\delta p_{i}$的jacobian
$$
\frac{\partial \mathbf{r}_{\Delta \mathbf{R}_{i j}}}{\partial \delta \mathbf{p}_{i}}=\frac{\partial \mathbf{r}_{\Delta \mathbf{R}_{i j}}}{\partial \mathbf{p}_{i}}=\mathbf{0}
$$
* $\mathbf{r}_{\Delta \mathbf{R}_{i j}}$关于$\delta p_{j}$的jacobian
$$
\frac{\partial \mathbf{r}_{\Delta \mathbf{R}_{i j}}}{\partial \delta \mathbf{p}_{j}}=\frac{\partial \mathbf{r}_{\Delta \mathbf{R}_{i j}}}{\partial \mathbf{p}_{j}}=\mathbf{0}
$$
* $\mathbf{r}_{\Delta \mathbf{R}_{i j}}$关于$\delta v_{i}$的jacobian
$$
\frac{\partial \mathbf{r}_{\Delta \mathbf{R}_{i j}}}{\partial \delta \mathbf{v}_{i}}=\frac{\partial \mathbf{r}_{\Delta \mathbf{R}_{i j}}}{\partial \mathbf{v}_{i}}=\mathbf{0}
$$

* $\mathbf{r}_{\Delta \mathbf{R}_{i j}}$关于$\delta v_{j}$的jacobian
$$
\frac{\partial \mathbf{r}_{\Delta \mathbf{R}_{i j}}}{\partial \delta \mathbf{v}_{j}}=\frac{\partial \mathbf{r}_{\Delta \mathbf{R}_{i j}}}{\partial \mathbf{v}_{j}}=\mathbf{0}
$$
* $\mathbf{r}_{\Delta \mathbf{R}_{i j}}$关于$\delta \mathbf{b}_{i}^{a}$的jacobian
$$
\frac{\partial \mathbf{r}_{\Delta \mathbf{R}_{i j}}}{\partial \delta \mathbf{b}_{i}^{a}}=\frac{\partial \mathbf{r}_{\Delta \mathbf{R}_{i j}}}{\partial \delta \mathbf{b}_{i}^{a}}=\mathbf{0}
$$
* $\mathbf{r}_{\Delta \mathbf{R}_{i j}}$关于$\delta \mathbf{b}_{i}^{g}$的jacobian
$$
\begin{array}{l}
\mathbf{r}_{\Delta \mathbf{R}_{i j}}\left(\delta \mathbf{b}_{i}^{g}+\widetilde{\delta \mathbf{b}_{i}^{g}}\right)=\log \left\{\left[\Delta \tilde{\mathbf{R}}_{i j}\left(\overline{\mathbf{b}}_{i}^{g}\right) \operatorname{Exp}\left(\frac{\partial \Delta \overline{\mathbf{R}}_{i j}}{\partial \overline{\mathbf{b}}^{g}}\left(\delta \mathbf{b}_{i}^{g}+\widetilde{\delta \mathbf{b}_{i}^{g}}\right)\right)\right]^{T} \mathbf{R}_{i}^{T} \mathbf{R}_{j}\right\} \\
\stackrel{(1)}{\approx} \log \left\{\left[\Delta \tilde{\mathbf{R}}_{i j}\left(\overline{\mathbf{b}}_{i}^{g}\right) \operatorname{Exp}\left(\frac{\partial \Delta \overline{\mathbf{R}}_{i j}}{\partial \overline{\mathbf{b}}^{g}} \delta \mathbf{b}_{i}^{g}\right) \operatorname{Exp}\left(\mathbf{J}_{r}\left(\frac{\partial \Delta \overline{\mathbf{R}}_{i j}}{\partial \overline{\mathbf{b}}^{g}} \delta \mathbf{b}_{i}^{g}\right) \frac{\partial \Delta \overline{\mathbf{R}}_{i j}}{\partial \overline{\mathbf{b}}^{g}} \widetilde{\delta \mathbf{b}_{i}^{g}}\right)\right]^{T} \Delta \mathbf{R}_{i j}\right\} \\
\stackrel{(2)}{=} \log \left\{\left[\Delta \hat{\mathbf{R}}_{i j} \cdot \operatorname{Exp}\left(\boldsymbol{\varepsilon} \cdot \frac{\partial \Delta \overline{\mathbf{R}}_{i j}}{\partial \overline{\mathbf{b}}^{g}} \widehat{\delta \mathbf{b}_{i}^{g}}\right)\right]^{T} \Delta \mathbf{R}_{i j}\right\} \\
\stackrel{(3)}{=} \log \left[\operatorname{Exp}\left(-\boldsymbol{\varepsilon} \cdot \frac{\partial \Delta \overline{\mathbf{R}}_{i j}}{\partial \overline{\mathbf{b}}^{g}} \widetilde{\delta \mathbf{b}_{i}^{g}}\right) \Delta \hat{\mathbf{R}}_{i j}^{T} \Delta \mathbf{R}_{i j}\right] \\
=\log \left[\operatorname{Exp}\left(-\boldsymbol{\varepsilon} \cdot \frac{\partial \Delta \overline{\mathbf{R}}_{i j}}{\partial \overline{\mathbf{b}}^{g}} \widetilde{\delta \mathbf{b}_{i}^{g}}\right) \operatorname{Exp}\left(\log \left(\Delta \hat{\mathbf{R}}_{i j}^{T} \Delta \mathbf{R}_{i j}\right)\right)\right] \\
=\log \left[\operatorname{Exp}\left(-\boldsymbol{\varepsilon} \cdot \frac{\partial \Delta \overline{\mathbf{R}}_{i j} \widetilde{\delta \overline{\mathbf{b}}_{i}^{g}}}{\partial \overline{\mathbf{b}}^{g}}\right) \operatorname{Exp}\left(\mathbf{r}_{\Delta \mathbb{R}_{i j}}\left(\delta \mathbf{b}_{i}^{g}\right)\right)\right] \\
\stackrel{(4)}{=} \log \left\{\operatorname{Exp}\left(\mathbf{r}_{\Delta \mathbf{R}_{i j}}\left(\delta \mathbf{b}_{i}^{g}\right)\right) \operatorname{Exp}\left[-\operatorname{Exp}\left(-\mathbf{r}_{\Delta \mathbf{R}_{i j}}\left(\delta \mathbf{b}_{i}^{g}\right)\right) \cdot \varepsilon \cdot \frac{\partial \Delta \overline{\mathbf{R}}_{i j}}{\partial \overline{\mathbf{b}}^{g}} \widetilde{\delta \mathbf{b}_{i}^{g}}\right]\right\} \\
\stackrel{(5)}{\approx} \mathbf{r}_{\Delta \mathbf{R}_{i j}}\left(\delta \mathbf{b}_{i}^{g}\right)-\mathbf{J}_{r}^{-1}\left(\mathbf{r}_{\Delta \mathbf{R}_{i j}}\left(\delta \mathbf{b}_{i}^{g}\right)\right) \cdot \operatorname{Exp}\left(-\mathbf{r}_{\Delta \mathbf{R}_{i j}}\left(\delta \mathbf{b}_{i}^{g}\right)\right) \cdot \boldsymbol{\varepsilon} \cdot \frac{\partial \Delta \overline{\mathbf{R}}_{i j}}{\partial \overline{\mathbf{b}}^{g}} \widetilde{\delta \mathbf{b}_{i}^{g}} \\
\stackrel{(6)}{=} \mathbf{r}_{\Delta \mathbf{R}_{i j}}-\mathbf{J}_{r}^{-1}\left(\mathbf{r}_{\Delta \mathbb{R}_{i j}}\right) \cdot \operatorname{Exp}\left(-\mathbf{r}_{\Delta \mathbf{R}_{i j}}\right) \cdot \mathbf{J}_{r}\left(\frac{\partial \Delta \overline{\mathbf{R}}_{i j}}{\partial \overline{\mathbf{b}}^{g}} \delta \mathbf{b}_{i}^{g}\right) \cdot \frac{\partial \Delta \overline{\mathbf{R}}_{i j}}{\partial \overline{\mathbf{b}}^{g}} \cdot \widetilde{\delta \mathbf{b}_{i}^{g}} \\
\end{array}
$$
可以得到：
$$
\frac{\partial \mathbf{r}_{\Delta \mathbf{R}_{i j}}}{\partial \widetilde{\delta \mathbf{b}_{i}^{g}}}=\frac{\partial \mathbf{r}_{\Delta \mathbf{R}_{i j}}}{\partial \delta \mathbf{b}_{i}^{g}}=-\mathbf{J}_{r}^{-1}\left(\mathbf{r}_{\Delta \mathbf{R}_{i j}}\right) \cdot \operatorname{Exp}\left(-\mathbf{r}_{\Delta \mathbf{R}_{i j}}\right) \cdot \mathbf{J}_{r}\left(\frac{\partial \Delta \overline{\mathbf{R}}_{i j}}{\partial \overline{\mathbf{b}}^{g}} \delta \mathbf{b}_{i}^{g}\right) \cdot \frac{\partial \Delta \overline{\mathbf{R}}_{i j}}{\partial \overline{\mathbf{b}}^{g}}
$$
### $\mathbf{r}_{\Delta \mathbf{v}_{i j}}$
*  $\mathbf{r}_{\Delta \mathbf{v}_{i j}}$关于$\delta \vec{\phi}_{i}$的jacobian
$$
\begin{aligned}
\mathbf{r}_{\Delta \mathbf{v}_{i j}}\left(\mathbf{R}_{i} \operatorname{Exp}\left(\delta \vec{\phi}_{i}\right)\right) & =\left(\mathbf{R}_{i} \operatorname{Exp}\left(\delta \vec{\phi}_{i}\right)\right)^{T}\left(\mathbf{v}_{j}-\mathbf{v}_{i}-\mathbf{g} \cdot \Delta t_{i j}\right)-\Delta \hat{\mathbf{v}}_{i j} \\
& \stackrel{(1)}{=} \operatorname{Exp}\left(-\delta \overrightarrow{\phi_{i}}\right) \cdot \mathbf{R}_{i}^{T} \cdot\left(\mathbf{v}_{j}-\mathbf{v}_{i}-\mathbf{g} \cdot \Delta t_{i j}\right)-\Delta \hat{\mathbf{v}}_{i j} \\
& \stackrel{(2)}{\approx}\left(\mathbf{I}-\left(\delta \vec{\phi}_{i}\right)^{\wedge}\right) \cdot \mathbf{R}_{i}^{T} \cdot\left(\mathbf{v}_{j}-\mathbf{v}_{i}-\mathbf{g} \cdot \Delta t_{i j}\right)-\Delta \hat{\mathbf{v}}_{i j} \\
& =\mathbf{R}_{i}^{T} \cdot\left(\mathbf{v}_{j}-\mathbf{v}_{i}-\mathbf{g} \cdot \Delta t_{i j}\right)-\Delta \hat{\mathbf{v}}_{i j}-\left(\delta \vec{\phi}_{i}\right)^{\wedge} \cdot \mathbf{R}_{i}^{T} \cdot\left(\mathbf{v}_{j}-\mathbf{v}_{i}-\mathbf{g} \cdot \Delta t_{i j}\right) \\
& \stackrel{(3)}{=} \mathbf{r}_{\Delta \mathbf{v}_{i j}}\left(\mathbf{R}_{i}\right)+\left[\mathbf{R}_{i}^{T} \cdot\left(\mathbf{v}_{j}-\mathbf{v}_{i}-\mathbf{g} \cdot \Delta t_{i j}\right)\right]^{\wedge} \cdot \delta \vec{\phi}_{i} \\
& \stackrel{(4)}{=} \mathbf{r}_{\Delta \mathbf{v}_{i j}}+\left[\mathbf{R}_{i}^{T} \cdot\left(\mathbf{v}_{j}-\mathbf{v}_{i}-\mathbf{g} \cdot \Delta t_{i j}\right)\right]^{\wedge} \cdot \delta \vec{\phi}_{i}
\end{aligned}
$$
可以得到：
$$
\frac{\partial \mathbf{r}_{\Delta \mathbf{v}_{i j}}}{\partial \delta \vec{\phi}_{i}}=\left[\mathbf{R}_{i}^{T} \cdot\left(\mathbf{v}_{j}-\mathbf{v}_{i}-\mathbf{g} \cdot \Delta t_{i j}\right)\right]^{\wedge}
$$
*  $\mathbf{r}_{\Delta \mathbf{v}_{i j}}$关于$\delta \vec{\phi}_{j}$的jacobian
$$
\frac{\partial \mathbf{r}_{\Delta v_{ij}}}{\partial \delta \vec{\phi}_{j}}=\mathbf{0}
$$
*  $\mathbf{r}_{\Delta \mathbf{v}_{i j}}$关于$\delta p_{i}$的jacobian
$$
\frac{\partial \mathbf{r}_{\Delta \mathbf{v}_{i j}}}{\partial \delta \mathbf{p}_{i}}=\frac{\partial \mathbf{r}_{\Delta \mathbf{v}_{i j}}}{\partial \mathbf{p}_{i}}=\mathbf{0}
$$
*  $\mathbf{r}_{\Delta \mathbf{v}_{i j}}$关于$\delta p_{j}$的jacobian
$$
\frac{\partial \mathbf{r}_{\Delta \mathbf{v}_{i j}}}{\partial \delta \mathbf{p}_{j}}=\frac{\partial \mathbf{r}_{\Delta \mathbf{v}_{i j}}}{\partial \mathbf{p}_{j}}=\mathbf{0}
$$
*  $\mathbf{r}_{\Delta \mathbf{v}_{i j}}$关于$\delta v_{i}$的jacobian
$$
\begin{aligned}
\mathbf{r}_{\Delta \mathbf{v}_{i j}}\left(\mathbf{v}_{i}+\delta \mathbf{v}_{i}\right) & =\mathbf{R}_{i}^{T} \cdot\left(\mathbf{v}_{j}-\mathbf{v}_{i}-\delta \mathbf{v}_{i}-\mathbf{g} \cdot \Delta t_{i j}\right)-\Delta \hat{\mathbf{v}}_{i j} \\
& =\mathbf{R}_{i}^{T} \cdot\left(\mathbf{v}_{j}-\mathbf{v}_{i}-\mathbf{g} \cdot \Delta t_{i j}\right)-\Delta \hat{\mathbf{v}}_{i j}-\mathbf{R}_{i}^{T} \delta \mathbf{v}_{i} \\
& =\mathbf{r}_{\Delta \mathbf{v}_{i j}}\left(\mathbf{v}_{i}\right)-\mathbf{R}_{i}^{T} \delta \mathbf{v}_{i} \\
& \stackrel{11}{ }=\mathbf{r}_{\Delta \mathbf{v}_{i j}}-\mathbf{R}_{i}^{T} \delta \mathbf{v}_{i}
\end{aligned}
$$
可以得到：
$$
\frac{\partial \mathbf{r}_{\Delta \mathbf{v}_{i j}}}{\partial \delta \mathbf{v}_{i}}=\frac{\partial \mathbf{r}_{\Delta \mathbf{v}_{i j}}}{\partial \mathbf{v}_{i}}=-\mathbf{R}_{i}^{T}
$$
*   $\mathbf{r}_{\Delta \mathbf{v}_{i j}}$关于$\delta v_{j}$的jacobian
$$
\begin{aligned}
\mathbf{r}_{\Delta \mathbf{v}_{i j}}\left(\mathbf{v}_{j}+\delta \mathbf{v}_{j}\right) & =\mathbf{R}_{i}^{T} \cdot\left(\mathbf{v}_{j}+\delta \mathbf{v}_{j}-\mathbf{v}_{i}-\mathbf{g} \cdot \Delta t_{i j}\right)-\Delta \hat{\mathbf{v}}_{i j} \\
& =\mathbf{R}_{i}^{T} \cdot\left(\mathbf{v}_{j}-\mathbf{v}_{i}-\mathbf{g} \cdot \Delta t_{i j}\right)-\Delta \hat{\mathbf{v}}_{i j}+\mathbf{R}_{i}^{T} \delta \mathbf{v}_{j} \\
& =\mathbf{r}_{\Delta \mathbf{v}_{i j}}\left(\mathbf{v}_{j}\right)+\mathbf{R}_{i}^{T} \delta \mathbf{v}_{j} \\
& =\mathbf{r}_{\Delta \mathbf{v}_{i j}}+\mathbf{R}_{i}^{T} \delta \mathbf{v}_{j}
\end{aligned}
$$
可以得到：
$$
\frac{\partial \mathbf{r}_{\Delta \mathbf{v}_{i j}}}{\partial \delta \mathbf{v}_{j}}=\frac{\partial \mathbf{r}_{\Delta \mathbf{v}_{i j}}}{\partial \mathbf{v}_{j}}=\mathbf{R}_{i}^{T}
$$
*  $\mathbf{r}_{\Delta \mathbf{v}_{i j}}$关于$\delta \mathbf{b}_{i}^{a}$的jacobian
$$
\frac{\partial \mathbf{r}_{\Delta v_{i j}}}{\partial \widetilde{\delta \mathbf{b}_{i}^{\mathrm{g}}}}=\frac{\partial \mathbf{r}_{\Delta v_{i j}}}{\partial \delta \mathbf{b}_{i}^{\mathrm{g}}}=-\frac{\partial \Delta \overline{\mathbf{v}}_{i j}}{\partial \mathbf{b}^{g}}
$$
*  $\mathbf{r}_{\Delta \mathbf{v}_{i j}}$关于$\delta \mathbf{b}_{i}^{g}$的jacobian
$$
\frac{\partial \mathbf{r}_{\Delta \mathbf{v}_{i j}}}{\partial \widetilde{\delta \mathbf{b}_{i}^{\mathrm{a}}}}=\frac{\partial \mathbf{r}_{\Delta \mathbf{v}_{i j}}}{\partial \delta \mathbf{b}_{i}^{\mathrm{a}}}=-\frac{\partial \Delta \overline{\mathbf{v}}_{i j}}{\partial \mathbf{b}^{a}}
$$

### $\mathbf{r}_{\Delta \mathbf{p}_{i j}}$
*   $\mathbf{r}_{\Delta \mathbf{p}_{i j}}$关于$\delta \vec{\phi}_{i}$的jacobian
$$
\begin{aligned}
\mathbf{r}_{\Delta \mathbf{p}_{\mathrm{p}}}\left(\mathbf{R}_{i} \operatorname{Exp}\left(\delta \overrightarrow{\phi_{i}}\right)\right) & =\left(\mathbf{R}_{i} \operatorname{Exp}\left(\delta \vec{\phi}_{i}\right)\right)^{T}\left(\mathbf{p}_{j}-\mathbf{p}_{i}-\mathbf{v}_{i} \cdot \Delta t_{i j}-\frac{1}{2} \mathbf{g} \cdot \Delta t_{i j}^{2}\right)-\Delta \hat{\mathbf{p}}_{i j} \\
& \stackrel{(1)}{=} \operatorname{Exp}\left(-\delta \vec{\phi}_{i}\right) \cdot \mathbf{R}_{i}^{T} \cdot\left(\mathbf{p}_{j}-\mathbf{p}_{i}-\mathbf{v}_{i} \cdot \Delta t_{i j}-\frac{1}{2} \mathbf{g} \cdot \Delta t_{i j}^{2}\right)-\Delta \hat{\mathbf{p}}_{i j} \\
& \stackrel{(2)}{ }\left(\mathbf{I}-\left(\delta \vec{\phi}_{i}\right)^{\wedge}\right) \cdot \mathbf{R}_{i}^{T} \cdot\left(\mathbf{p}_{j}-\mathbf{p}_{i}-\mathbf{v}_{i} \cdot \Delta t_{i j}-\frac{1}{2} \mathbf{g} \cdot \Delta t_{i j}^{2}\right)-\Delta \hat{\mathbf{p}}_{i j} \\
& =\mathbf{R}_{i}^{T} \cdot\left(\mathbf{p}_{j}-\mathbf{p}_{i}-\mathbf{v}_{i} \cdot \Delta t_{i j}-\frac{1}{2} \mathbf{g} \cdot \Delta t_{i j}^{2}\right)-\Delta \hat{\mathbf{p}}_{i j}-\left(\delta \vec{\phi}_{i}\right)^{\wedge} \mathbf{R}_{i}^{T} \cdot\left(\mathbf{p}_{j}-\mathbf{p}_{i}-\mathbf{v}_{i} \cdot \Delta t_{i j}-\frac{1}{2} \mathbf{g} \cdot \Delta t_{i j}^{2}\right) \\
& \stackrel{(3)}{=} \mathbf{r}_{\Delta \mathbf{p}_{y j}}\left(\mathbf{R}_{i}\right)+\left[\mathbf{R}_{i}^{T} \cdot\left(\mathbf{p}_{j}-\mathbf{p}_{i}-\mathbf{v}_{i} \cdot \Delta t_{i j}-\frac{1}{2} \mathbf{g} \cdot \Delta t_{i j}^{2}\right)\right]^{\wedge} \cdot \delta \vec{\phi}_{i} \\
& \stackrel{(4)}{=} \mathbf{r}_{\Delta \mathbf{p}_{y}}+\left[\mathbf{R}_{i}^{T} \cdot\left(\mathbf{p}_{j}-\mathbf{p}_{i}-\mathbf{v}_{i} \cdot \Delta t_{i j}-\frac{1}{2} \mathbf{g} \cdot \Delta t_{i j}^{2}\right)\right] \cdot \delta \vec{\phi}_{i}
\end{aligned}
$$
可以得到：
$$
\frac{\partial \mathbf{r}_{\Delta \mathbf{p}_{i j}}}{\partial \delta \overrightarrow{\phi_{i}}}=\left[\mathbf{R}_{i}^{T} \cdot\left(\mathbf{p}_{j}-\mathbf{p}_{i}-\mathbf{v}_{i} \cdot \Delta t_{i j}-\frac{1}{2} \mathbf{g} \cdot \Delta t_{i j}^{2}\right)\right]^{\wedge}
$$
*  $\mathbf{r}_{\Delta \mathbf{p}_{i j}}$关于$\delta \vec{\phi}_{j}$的jacobian
$$
\frac{\partial \mathbf{r}_{\Delta \mathbf{p}_{i j}}}{\partial \delta \vec{\phi}_{j}}=\mathbf{0}
$$
*  $\mathbf{r}_{\Delta \mathbf{p}_{i j}}$关于$\delta p_{i}$的jacobian
$$
\begin{aligned}
\mathbf{r}_{\Delta \mathbf{p}_{i j}}\left(\mathbf{p}_{i}+\mathbf{R}_{i} \cdot \delta \mathbf{p}_{i}\right) & =\mathbf{R}_{i}^{T}\left(\mathbf{p}_{j}-\mathbf{p}_{i}-\mathbf{R}_{i} \cdot \delta \mathbf{p}_{i}-\mathbf{v}_{i} \cdot \Delta t_{i j}-\frac{1}{2} \mathbf{g} \cdot \Delta t_{i j}^{2}\right)-\Delta \hat{\mathbf{p}}_{i j} \\
& =\mathbf{R}_{i}^{T}\left(\mathbf{p}_{j}-\mathbf{p}_{i}-\mathbf{v}_{i} \cdot \Delta t_{i j}-\frac{1}{2} \mathbf{g} \cdot \Delta t_{i j}^{2}\right)-\Delta \hat{\mathbf{p}}_{i j}-\mathbf{I} \cdot \delta \mathbf{p}_{i} \\
& =\mathbf{r}_{\Delta \mathbf{p}_{i j}}\left(\mathbf{p}_{i}\right)-\mathbf{I} \cdot \delta \mathbf{p}_{i} \\
& \stackrel{1}{ } \\
& =\mathbf{r}_{\Delta \mathbf{p}_{i j}}-\mathbf{I} \cdot \delta \mathbf{p}_{i}
\end{aligned}
$$
*  $\mathbf{r}_{\Delta \mathbf{p}_{i j}}$关于$\delta p_{j}$的jacobian
$$
\begin{aligned}
\mathbf{r}_{\Delta \mathbf{p}_{i j}}\left(\mathbf{p}_{j}+\mathbf{R}_{j} \cdot \delta \mathbf{p}_{j}\right) & =\mathbf{R}_{i}^{T}\left(\mathbf{p}_{j}+\mathbf{R}_{j} \cdot \delta \mathbf{p}_{j}-\mathbf{p}_{i}-\mathbf{v}_{i} \cdot \Delta t_{i j}-\frac{1}{2} \mathbf{g} \cdot \Delta t_{i j}^{2}\right)-\Delta \hat{\mathbf{p}}_{i j} \\
& =\mathbf{R}_{i}^{T}\left(\mathbf{p}_{j}-\mathbf{p}_{i}-\mathbf{v}_{i} \cdot \Delta t_{i j}-\frac{1}{2} \mathbf{g} \cdot \Delta t_{i j}^{2}\right)-\Delta \hat{\mathbf{p}}_{i j}+\mathbf{R}_{i}^{T} \mathbf{R}_{j} \cdot \delta \mathbf{p}_{j} \\
& =\mathbf{r}_{\Delta \mathbf{p}_{i j}}\left(\mathbf{p}_{j}\right)+\mathbf{R}_{i}^{T} \mathbf{R}_{j} \cdot \delta \mathbf{p}_{j} \\
& \stackrel{(1)}{=} \mathbf{r}_{\Delta \mathbf{p}_{i j}}+\mathbf{R}_{i}^{T} \mathbf{R}_{j} \cdot \delta \mathbf{p}_{j}
\end{aligned}
$$
可以得到：
$$
\frac{\partial \mathbf{r}_{\Delta \mathbf{p}_{i j}}}{\partial \delta \mathbf{p}_{j}}=\frac{\partial \mathbf{r}_{\Delta \mathbf{p}_{i j}}}{\partial \mathbf{p}_{j}}=\mathbf{R}_{i}^{T} \mathbf{R}_{j}
$$
*  $\mathbf{r}_{\Delta \mathbf{p}_{i j}}$关于$\delta v_{i}$的jacobian
$$
\begin{aligned}
\mathbf{r}_{\Delta \mathbf{p}_{i j}}\left(\mathbf{v}_{i}+\delta \mathbf{v}_{i}\right) & =\mathbf{R}_{i}^{T}\left(\mathbf{p}_{j}-\mathbf{p}_{i}-\mathbf{v}_{i} \cdot \Delta t_{i j}-\delta \mathbf{v}_{i} \cdot \Delta t_{i j}-\frac{1}{2} \mathbf{g} \cdot \Delta t_{i j}^{2}\right)-\Delta \hat{\mathbf{p}}_{i j} \\
& =\mathbf{R}_{i}^{T}\left(\mathbf{p}_{j}-\mathbf{p}_{i}-\mathbf{v}_{i} \cdot \Delta t_{i j}-\frac{1}{2} \mathbf{g} \cdot \Delta t_{i j}^{2}\right)-\Delta \hat{\mathbf{p}}_{i j}-\mathbf{R}_{i}^{T} \Delta t_{i j} \cdot \delta \mathbf{v}_{i} \\
& =\mathbf{r}_{\Delta \mathbf{p}_{i j}}\left(\mathbf{v}_{i}\right)-\mathbf{R}_{i}^{T} \Delta t_{i j} \cdot \delta \mathbf{v}_{i} \\
& \stackrel{(1)}{ } \\
& =\mathbf{r}_{\Delta \mathbf{p}_{i j}}-\mathbf{R}_{i}^{T} \Delta t_{i j} \cdot \delta \mathbf{v}_{i}
\end{aligned}
$$
可以得到：
$$
\frac{\partial \mathbf{r}_{\Delta \mathbf{p}_{i j}}}{\partial \delta \mathbf{v}_{i}}=\frac{\partial \mathbf{r}_{\Delta \mathbf{p}_{i j}}}{\partial \mathbf{v}_{i}}=-\mathbf{R}_{i}^{T} \Delta t_{i j}
$$
*  $\mathbf{r}_{\Delta \mathbf{p}_{i j}}$关于$\delta v_{j}$的jacobian

$$
\frac{\partial \mathbf{r}_{\Delta \mathbf{p}_{i j}}}{\partial \delta \mathbf{v}_{j}}=\frac{\partial \mathbf{r}_{\Delta \mathbf{p}_{i j}}}{\partial \mathbf{v}_{j}}=\mathbf{0}
$$
* $\mathbf{r}_{\Delta \mathbf{p}_{i j}}$关于$\delta \mathbf{b}_{i}^{a}$的jacobian

$$
\frac{\partial \mathbf{r}_{\Delta \mathbf{p}_{i j}}}{\partial \widehat{\delta \mathbf{b}_{i}^{g}}}=\frac{\partial \mathbf{r}_{\Delta \mathbf{p}_{i j}}}{\partial \delta \mathbf{b}_{i}^{g}}=-\frac{\partial \Delta \overline{\mathbf{p}}_{i j}}{\partial \mathbf{b}^{g}}
$$
* $\mathbf{r}_{\Delta \mathbf{p}_{i j}}$关于$\delta \mathbf{b}_{i}^{g}$的jacobian

$$
\frac{\partial \mathbf{r}_{\Delta \mathbf{p}_{i j}}}{\partial \delta \mathbf{b}_{i}^{a}}=-\frac{\partial \Delta \overline{\mathbf{p}}_{i j}}{\partial \mathbf{b}^{a}}
$$
