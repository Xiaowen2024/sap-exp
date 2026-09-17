# MuJoCo vs SAP Warp：Box Stack 核心结果

## 实验设置

- Box：40 × 40 × 40 mm，质量 1 kg。
- 每个 box 只允许沿 z 轴移动；没有旋转、横向移动和摩擦。
- 每个平面界面理论上有 4 个接触点。
- 重力：9.81 m/s²。
- 主实验 timestep：0.5 ms。
- Box 数量：1、3、5。
- Nominal pair stiffness：$10^4$、$10^5$、$10^6$ N/m。
- MuJoCo：Newton、`implicitfast`、`condim=1`、`margin=0`、`gap=0`。
- SAP Warp：`drake` preset、`rigid_gap=0`、shape `margin=0`。

## 三个主要指标

### 1. 平均静力误差 $E_F$

理论上，第 $i$ 个界面应承担它上方所有 box 的重量。定义：

$$
E_{F,i}=\frac{|\bar F_{n,i}-F_{n,i}^{static}|}{F_{n,i}^{static}}.
$$

$E_F$ 越接近 0，说明最后 1 秒的平均接触力越符合静力平衡。

注意：这只是 **tail 平均静力平衡**。它不能证明每一帧都满足 dynamic force balance。一个系统可以一边振动，一边仍然具有正确的平均支撑力。

### 2. 几何刚度比 $R_i$

四个刚度均为 $k$ 的接触点并联时，理论压缩量为 $F_{n,i}/(4k)$。定义：

$$
R_i^{geom}=\frac{\bar\delta_i^{geom}}{F_{n,i}/(4k)}.
$$

- $R=1$：几何压缩量正确。
- $R>1$：压缩过多，界面比目标更软。
- $R<1$：压缩过少，界面比目标更硬。
- 系统没有静止时，$R$ 只能作为观测值，不能解释成静态刚度。

这里统一使用由 box 位置和尺寸计算的 **geometric penetration**。SAP 内部的 $-\phi_0$ 只作为 solver 诊断，不放进主表。

### 3. 平均有效接触点数 $\bar C_i$

- $C=4$：flat interface 保持完整四点接触。
- $C<4$：接触点在消失或切换。
- 非整数平均值，例如 1.46，表示最后 1 秒内接触点数随时间变化。

## 核心结果表

以下全部使用 dt = 0.5 ms。`内部`表示所有 box–box 界面的最小值到最大值。

| 引擎 | Box 数 N | k (N/m) | 最大平均力误差 $E_F$ | $R_{ground}$ | $R_{internal}$ | $C_{ground}$ | $C_{internal}$ | 状态 |
|---|---:|---:|---:|---:|---:|---:|---:|---|
| MuJoCo | 1 | 1e4 | 0.0000% | 1.000 | — | 4.00 | — | 稳定 |
| MuJoCo | 1 | 1e5 | 0.0000% | 1.000 | — | 4.00 | — | 稳定 |
| MuJoCo | 1 | 1e6 | 0.0000% | 1.000 | — | 4.00 | — | 稳定 |
| MuJoCo | 3 | 1e4 | 0.0000% | 1.000 | 2.000–2.000 | 4.00 | 4.00–4.00 | 稳定 |
| MuJoCo | 3 | 1e5 | 0.0000% | 1.000 | 2.000–2.000 | 4.00 | 4.00–4.00 | 稳定 |
| MuJoCo | 3 | 1e6 | 0.0000% | 1.000 | 2.000–2.000 | 4.00 | 4.00–4.00 | 稳定 |
| MuJoCo | 5 | 1e4 | 0.0000% | 1.000 | 2.000–2.000 | 4.00 | 4.00–4.00 | 稳定 |
| MuJoCo | 5 | 1e5 | 0.0000% | 1.000 | 2.000–2.000 | 4.00 | 4.00–4.00 | 稳定 |
| MuJoCo | 5 | 1e6 | 0.0000% | 1.000 | 2.000–2.000 | 4.00 | 4.00–4.00 | 稳定 |
| SAP | 1 | 1e4 | 0.0000% | 1.000 | — | 4.00 | — | 稳定 |
| SAP | 1 | 1e5 | 0.0000% | 1.000 | — | 4.00 | — | 稳定 |
| SAP | 1 | 1e6 | 0.0000% | 1.000 | — | 4.00 | — | 稳定 |
| SAP | 3 | 1e4 | 0.0000% | 1.000 | 0.592–0.796 | 4.00 | 4.00–4.00 | 稳定，但有几何偏移 |
| SAP | 3 | 1e5 | 0.0659% | 1.000 | 0.000–0.011 | 4.00 | 1.54–2.46 | 接触切换 |
| SAP | 3 | 1e6 | 0.0700% | 1.000 | 0.000–0.000 | 4.00 | 0.85–0.85 | 接触切换 |
| SAP | 5 | 1e4 | 0.0000% | 1.000 | 0.592–0.898 | 4.00 | 4.00–4.00 | 稳定，但有几何偏移 |
| SAP | 5 | 1e5 | 0.0656% | 1.000 | 0.000–0.078 | 4.00 | 1.46–3.80 | 接触切换 |
| SAP | 5 | 1e6 | 0.2631% | 1.000 | 0.000–0.000 | 4.00 | 0.68–1.67 | 持续抖动、接触切换 |

## 如何读这张表

### 平均 force balance

- MuJoCo 所有 case 的 $E_F$ 都接近机器精度。
- SAP 的最大误差也不超过 0.27%。
- 因此两边的 **平均支撑力基本一致**。
- 但 SAP 高刚度 case 仍在运动，所以不能说它满足逐帧 dynamic force balance。

### $R$：实际几何压缩

- 单箱时，两边均有 $R=1$，说明 one-box–plane calibration 成功。
- MuJoCo 多层内部界面始终为 $R=2$：box–box 界面压缩是目标的两倍，实际 aggregate stiffness 是目标的一半。
- SAP 在 $k=10^4$ 时保持四点接触，但内部 geometric $R<1$。原因是 SAP 的 solver residual 与几何表面距离存在约 0.1 mm 差值。
- SAP 在 $k=10^5$ 和 $10^6$ 时内部 $R$ 接近零，但此时接触点正在切换，不能把它解释为稳定的“无限刚”接触。

### 接触点数

- MuJoCo 所有 case 始终保持每界面 4 点。
- SAP 单箱和 $k=10^4$ stack 保持 4 点。
- SAP 多层 $k\ge10^5$ 时内部接触点明显减少并随时间变化。

## Timestep 检查：五层、k = 1e6 N/m

| 引擎 | dt (ms) | 最大平均力误差 $E_F$ | $R_{ground}$ | $R_{internal}$ | $C_{internal}$ |
|---|---:|---:|---:|---:|---:|
| MuJoCo | 0.25 | 0.0000% | 1.000 | 2.000–2.000 | 4.00–4.00 |
| MuJoCo | 0.50 | 0.0000% | 1.000 | 2.000–2.000 | 4.00–4.00 |
| MuJoCo | 1.00 | 0.0000% | 1.000 | 2.000–2.000 | 4.00–4.00 |
| SAP | 0.25 | 0.1141% | 1.000 | 0.000–0.000 | 0.45–1.19 |
| SAP | 0.50 | 0.2631% | 1.000 | 0.000–0.000 | 0.68–1.67 |
| SAP | 1.00 | 0.0000% | 1.000 | 0.000–1.209 | 1.20–2.40 |

SAP 减小 dt 后 gap 波动和速度抖动有所下降，但没有恢复稳定四点接触。MuJoCo 的三个指标在该 dt 范围内基本不变。

## 当前结论

1. 两边在单箱 ground interface 上都完成了 stiffness calibration。
2. MuJoCo 多层结果稳定，但 scalar ground calibration 没有自动迁移到 dynamic box–box：内部界面实际软 2 倍。
3. SAP 在低刚度下保持完整接触，但 solver residual 与几何 penetration 相差约 0.1 mm。
4. SAP 当前 high-stiffness 多层结果没有达到可信静态状态；平均力平衡正确不能抵消这一问题。
5. 下一次严格 matched-interface 对比，应先将 MuJoCo box–box `solref stiffness` 调为当前值的 2 倍，再重新验证 $R_{internal}\approx1$。

## 尚未记录的量

- 全部 case 的逐帧 dynamic force residual；
- 全部 SAP case 的 tail force fluctuation；
- 每层完整轨迹的最大 penetration $\delta_{i,max}$。

因此本报告不声称已经比较 dynamic force accuracy 或 transient maximum penetration。

## 数据文件

- MuJoCo：`exp-results/layered-stack-stiffness/mujoco-full/results.csv`
- SAP 主实验：`exp-results/layered-stack-stiffness/sap/results.csv`
- SAP timestep：`exp-results/layered-stack-stiffness/sap-dt/results.csv`
