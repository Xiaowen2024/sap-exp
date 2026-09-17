# SAP vs. MuJoCo：完全弹性 bouncing benchmark

## 结论

在本机当前 SAP Warp 实现中，重复的近完全弹性碰撞是一个明显更具挑战性的场景。将 SAP 的原生耗散时间尺度设为零、并对 `dt`、`ke` 与 contact preset 扫描后，最优已测 SAP 配置的第一次反弹已接近解析解，但完整 20 s 内仍损失 16.6% 的机械能。

相对地，现代 MuJoCo 的直接接触格式 `solref=[-stiffness, -damping]` 在 `damping=0` 时可逼近完全弹性反弹；相同 `dt=1 ms`、同一刚度数量级下，其长期机械能漂移显著更小。

这支持一个受限但有意义的结论：**对该版本 SAP Warp 的原生接触模型和已测 preset 而言，重复无摩擦弹性碰撞的能量守恒明显弱于 MuJoCo 的零阻尼直接接触配置。**这不是关于所有 SAP 实现、所有物理任务或原始 SAP 理论的普遍性结论。

## 1. 任务与解析基准

场景严格复刻 [leggedrobotics/SimBenchmark bouncing test](https://leggedrobotics.github.io/SimBenchmark/bouncing/index.html)：

- 静态平面；
- 7×7 个独立球，共 49 个；
- 每球质量 10 kg、半径 0.1 m；
- 初始球心高度 5 m，球间横向距离 2 m，因而没有球–球碰撞；
- 重力 \(g=9.81\ \mathrm{m/s^2}\)，无摩擦；
- 仿真时长 20 s；
- 理想目标为恢复系数 \(e=1\)。

解析不变量是总机械能：

\[
E(t)=E_0=NmgH=49\times10\times9.81\times5=24034.5\ \mathrm{J}.
\]

原始 benchmark 的误差定义为逐仿真步采样后的

\[
\operatorname{MSE}_E=\frac1N\sum_i(E(t_i)-E_0)^2.
\]

除该严格沿用原网页的 MSE 外，本实验也记录：

- 第一次反弹最高高度（解析值：5 m）；
- 20 s 最终能量比 \(E(T)/E_0\)。

后两者避免只以机械能 MSE 判断柔顺接触：接触压缩时，一部分能量会暂时储存在数值接触的隐式/显式弹簧中，而原网页的度量没有计入该部分。

## 2. 接触模型与公平性边界

两端完全对齐的内容是几何、质量、重力、初始状态、无摩擦设定、采样方式和解析指标。

两端**不强行对齐**的内容是接触律本身：

- MuJoCo 使用 native direct `solref=[-k,-b]`，零 `b`；
- SAP 使用 native `ke` 与 `tau`，固定 `tau=0` 以测试其无显式耗散极限；
- SAP 测试 `approx32`、`approx64`、`drake` 三种 contact preset。

因此比较应解释为“每个引擎在同一解析任务、各自接触接口内的已测表现”，而不是声称 `ke` 与 MuJoCo `solref` 存在一一数值映射。

所有 SAP 点的实际 raw contact 数均为 49，低于 `max_rigid_contact=64`，没有发生接触容量截断。

## 3. MuJoCo 参照结果

现代 MuJoCo 3.12.0 使用 Newton、elliptic cone、direct `solref` 和零 damping。

在直接可比的 `dt=1 ms, stiffness=1e4` 点：

- 第一次反弹：5.0075 m（相对解析 +0.15%）；
- 20 s 最终能量比：1.00238（+0.238%）；
- 能量 MSE：\(3.41\times10^6\ \mathrm{J^2}\)。

进一步的 MuJoCo 扫描中，按原网页 MSE 最低的已测点为 `dt=0.1 ms, stiffness=1e6, damping=0`，MSE \(3.34\times10^5\ \mathrm{J^2}\)，但其首次反弹误差（−0.086%）略高于低刚度、较小步长的反弹高度最优点。这说明“最低原始 MSE”与“最接近反弹高度”也是两个不同目标。

## 4. SAP baseline：Drake preset

首先测试 rolling 实验中较好的 `tau=4 ms`，它不适合此完全弹性任务：

- `drake, dt=1 ms, ke=1e4, tau=4 ms`；
- 第一次反弹：2.676 m；
- 20 s 最终能量比：0.0161；
- 能量 MSE：\(4.73\times10^8\ \mathrm{J^2}\)。

将 `tau` 降为零显著改善：

- `drake, dt=1 ms, ke=1e4, tau=0`；
- 第一次反弹：4.649 m（−7.02%）；
- 20 s 最终能量比：0.4495；
- 能量 MSE：\(8.61\times10^7\ \mathrm{J^2}\)。

这表明前一个失败不能仅归因于 SAP 的核心离散化；`tau` 的耗散作用非常显著。不过 `tau=0` 后仍未达到解析弹性守恒。

## 5. SAP 参数扫描

短时筛选运行 2.4 s，覆盖第一次落地及第一轮反弹。全点固定 `tau=0`，扫描：

- preset：`approx32`、`approx64`、`drake`；
- `dt`：1 ms、0.5 ms；
- `ke`：\(10^4\)、\(10^5\)。

结果摘要（指标为第一次反弹高度）：

| preset | dt | ke | 第一次反弹 | 相对 5 m |
|---|---:|---:|---:|---:|
| approx32 | 1 ms | 1e4 | 4.823 m | −3.54% |
| approx32 | 1 ms | 1e5 | 4.499 m | −10.02% |
| approx32 | 0.5 ms | 1e4 | **4.910 m** | **−1.81%** |
| approx32 | 0.5 ms | 1e5 | 4.732 m | −5.36% |
| approx64 | 1 ms | 1e4 | 4.823 m | −3.54% |
| approx64 | 1 ms | 1e5 | 4.499 m | −10.02% |
| approx64 | 0.5 ms | 1e4 | **4.910 m** | **−1.81%** |
| approx64 | 0.5 ms | 1e5 | 4.732 m | −5.36% |
| drake | 1 ms | 1e4 | 4.649 m | −7.02% |
| drake | 1 ms | 1e5 | 4.010 m | −19.81% |
| drake | 0.5 ms | 1e4 | 4.822 m | −3.59% |
| drake | 0.5 ms | 1e5 | 4.481 m | −10.38% |

观察：

1. 从 1 ms 降至 0.5 ms 一贯改善反弹；
2. 由 \(10^4\) 提升至 \(10^5\) 一贯使反弹变差；更硬不等于更弹；
3. `approx32` 和 `approx64` 在本场景、所报告精度下数值一致；
4. 以相同 `dt`、`ke` 对比，Drake 的反弹更低。

## 6. 最优 SAP 候选的完整 20 s 验证

从短时扫描选取：

```text
preset = approx32
dt     = 0.5 ms
ke     = 1e4
tau    = 0
```

完整时域结果：

- 第一次反弹：4.9097 m（−1.81%）；
- 20 s 最终能量比：0.83403；
- 长期机械能损失：16.60%；
- 能量 MSE：\(2.365\times10^7\ \mathrm{J^2}\)；
- 实际接触数：49；
- CPU instrumentation loop：482.1 s，约 83.0 step/s。

这里的 482 s 包含每步 Python/Warp 状态读取和全系统能量采样，不能与 MuJoCo 的纯 stepping 速度直接作为 engine throughput 对比；它反映的是当前单 world、CPU、仪器化 SAP runner 的端到端成本。

## 7. 解释与结论范围

这项测试隔离了 normal impact：没有摩擦、没有滚动摩擦、没有复杂几何、没有球–球耦合。因此结果并不是 rolling 中摩擦接触分配问题的重复，而是一个独立证据。

已测配置下，SAP 的首次反弹可通过选择 `approx32/64`、小 `dt`、中等 `ke` 改善到 −1.81%，但重复撞击仍造成 16.6% 长期机械能损失。MuJoCo 的相近点则在 +0.238% 长期能量漂移附近。

可作出的结论：

> 对当前 SAP Warp 的实现与已测 native contact preset，长时、近完全弹性的刚体碰撞会发生显著数值耗散；MuJoCo 的 direct zero-damping contact 在此解析任务中更接近能量守恒。

不可作出的结论：

- “SAP 在所有接触任务都更差”；
- “SAP 不可能模拟弹性碰撞”；
- “本报告的 CPU 端到端计时就是各求解器内核速度差”。

## 8. 可复现材料

- SAP runner：`exp-results/bouncing/scripts/run_sap_bouncing_benchmark.py`
- SAP 短时扫描：`exp-results/bouncing/sap-sweep-screen/results.csv`
- SAP 最优候选完整验证：`exp-results/bouncing/sap-sweep-full-approx32/results.csv`
- SAP baseline trace（Drake, `tau=4 ms`）：`exp-results/bouncing/sap-cpu/energy-trace-tau-4ms-dt-1ms.csv`
- MuJoCo runner：`exp-results/bouncing/scripts/run_mujoco_bouncing_benchmark.py`
- MuJoCo 扫描：`exp-results/bouncing/mujoco/results.csv`

