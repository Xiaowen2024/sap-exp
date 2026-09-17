# MuJoCo vs. SAP/Warp：25 球滚动接触实验报告

**状态：** 已完成 MuJoCo 参数 sweep（648 组）与 SAP Drake/CPU 单因素 A/B/C sweep（11 组）。

## 结论摘要

- MuJoCo 的标准 SimBenchmark 配置（Newton + Euler）得到 **MSE = 0.00692691 (m/s)^2**，与原始 SimBenchmark 发布值 `0.00692691` 一致。
- 在已测试的 SAP/Warp Drake 配置中，最好的 25 球结果为 **MSE = 0.34721228 (m/s)^2**：`dt=1 ms`、`ke=1e4`、`tau=4 ms`。
- 因而，在这个有解析纯滚动解的特定任务上，已测试 SAP 配置相对 MuJoCo 标准配置的 MSE 高约 **50.1 倍**。这支持“该 SAP 接触配置在此纯滚动基准下更不准确”的结论；不支持“SAP solver 在所有任务上更差”的泛化结论。
- SAP 的 `tau` 和 `ke` 会改变接触模型本身，而不仅是计算精度；因此 MuJoCo 与 SAP 不能通过逐参数硬映射来得到同一接触模型。比较应按冻结的 native configuration 与预先规定的调参预算进行。

## 1. 任务定义与解析参考

任务来自 [leggedrobotics/SimBenchmark rolling benchmark](https://github.com/leggedrobotics/SimBenchmark)。场景为：

- 静止平面；
- 一个 10 kg、`20 × 20 × 1 m` 的自由平板；
- 平板上 25 个实心球，5×5 排列，每个半径 0.5 m、质量 1 kg；
- 对平板持续施加 150 N 的水平力，方向为 60°；
- 重力 9.81 m/s²，模拟 4 s。

目标有效 pair friction 为：

```text
box–ground = 0.4
ball–box   = 0.8
```

### 误差定义

每个积分前时刻 `t_i = i * dt`，记录平板和左下角球的**世界坐标系线速度**。报告：

\[
\mathrm{MSE}=\frac{1}{N}\sum_{i=0}^{N-1}
\left(
\|v_{box}(t_i)-v^*_{box}(t_i)\|^2+
\|v_{ball,0}(t_i)-v^*_{ball}(t_i)\|^2
\right).
\]

单位为 `(m/s)^2`。解析解假设 25 个球保持对称、始终在平板上、无滑动纯滚动，并采用原项目的实心球滚动惯性系数 `3.5`。原始实现见 [`RollingBenchmark.hpp`](https://github.com/leggedrobotics/SimBenchmark/blob/master/benchmark/include/RollingBenchmark.hpp)。

> 重要修正：早期 MuJoCo 原型误用了 ball 的局部 spatial velocity，曾产生无效的 `0.422655` MSE。当前实现读取 free joint 的世界线速度前三项，已由官方 Newton/Euler 数值验证。

## 2. 引擎配置

### MuJoCo 3.12.0

MuJoCo 的 geom friction 按逐项 `max` 混合：ground 和 box 设为 `0.4`，ball 设为 `0.8`，因此得到目标 pair friction。

标准复现配置：

```text
solver      Newton
integrator  Euler
cone        elliptic
dt          1 ms
iterations  1000
tolerance   1e-30
contact     MuJoCo 默认 solref=[0.02, 1],
            solimp=[0.9, 0.95, 0.001, 0.5, 2]
```

完整 sweep：3 solvers × 4 integrators × 2 cones × 3 iteration limits × 3 timesteps × 3 contact-softness settings = **648 组**。

### SAP/Warp 1.16.0（CPU）

SAP 使用 Drake preset，`armijo_decay` line search。SAP 的材质摩擦按调和规则混合；为实现目标 pair friction，设定：

```text
ground mu = 0.25
box    mu = 1.00
ball   mu = 0.6667
```

有效 pair friction 仍为 0.4 和 0.8。接触容量设为 64，实测峰值活跃接触数为 29，因此没有容量截断。

SAP A/B/C 单因素实验：

```text
A: dt  = 0.5, 1, 2, 5 ms        (ke=1e4, tau=30 ms)
B: tau = 4, 10, 30, 60 ms       (dt=1 ms, ke=1e4)
C: ke  = 1e3, 1e4, 1e5          (dt=1 ms, tau=30 ms)
```

## 3. MuJoCo 结果

### 3.1 标准 SimBenchmark 复现

| 配置 | MSE `(m/s)^2` | 测得步率 |
| --- | ---: | ---: |
| Newton + Euler + elliptic, 1 ms, 1000 iter | **0.0069269084** | 7280.2 Hz |
| 原始 SimBenchmark CSV：Newton + Euler | **0.00692691** | 历史硬件：1.67194 s / rollout |

该数值匹配确认了场景、解析参考、采样时刻和速度坐标约定。

### 3.2 648 组 sweep 的主要规律

下表是仅限 elliptic cone 的 MSE 中位数；它描述参数的边际趋势，不是逐配置最优值。

| 参数 | 设置 | MSE 中位数 `(m/s)^2` |
| --- | --- | ---: |
| integrator | Euler / RK4 / Implicit / ImplicitFast | 0.006927 / **0.003099** / 0.006927 / 0.006927 |
| dt | 0.5 / 1 / 2 ms | **0.003951** / 0.006436 / 0.011368 |
| iterations | 20 / 100 / 500 | 0.005705 / 0.005563 / 0.005563 |
| softness | default / soft-40ms / stiff-4ms | 0.005592 / **0.005084** / 0.007516 |
| cone | elliptic / pyramidal | **0.005563** / 35.421516 |

观察：

- 椭圆摩擦锥是本任务的关键选择；pyramidal cone 在这个 benchmark 上产生非常大的误差。
- RK4 与更小 dt 通常显著降低误差。
- 从 100 到 500 次迭代的边际收益很小；实际迭代会在达到收敛条件后提前停止。
- contact softness 会改变结果；它不应被误解为纯粹的“无物理影响的求解器开关”。

### 3.3 MuJoCo 最佳已测试配置

| 配置 | MSE `(m/s)^2` | 4 s rollout wall time |
| --- | ---: | ---: |
| PGS + RK4 + elliptic, `dt=0.5 ms`, 100 iter, stiff-4ms | **0.0011458890** | 5.218 s |

此条目是 648 组中的最低 MSE。它是“本次 MuJoCo sweep 的最优数值点”，不是与 SAP 直接公平的 engine-best 对比：MuJoCo sweep 的组合空间远大于 SAP 的 11 组单因素 sweep。

### 3.4 与 SAP A/B/C 结构对齐的 MuJoCo contact sweep

为避免只比较 MuJoCo 的大 sweep 与 SAP 的小 sweep，额外完成了相同结构的 11 组 MuJoCo A/B/C。固定 `Newton + Euler + elliptic`、1000 iter、`tolerance=1e-30`：

- A：`dt = 0.5, 1, 2, 5 ms`；
- B：`solref` time constant = `4, 10, 30, 60 ms`；这是 SAP `tau` 最接近的 MuJoCo 参数；
- C：`solimp = (dmin, dmax)` = `(0.70,0.75)`, `(0.90,0.95)`, `(0.98,0.99)`；它调节 MuJoCo 接触阻抗，是 SAP `ke` 的近似对照维度，不是一一映射。

| Sweep | 参数 | MSE `(m/s)^2` | wall time | steps/s |
| --- | --- | ---: | ---: | ---: |
| A: dt | 0.5 ms | **0.003951** | 1.137 s | 7038.4 |
| A: dt | 1 ms | 0.006927 | 0.592 s | 6756.3 |
| A: dt | 2 ms | 0.012594 | 0.360 s | 5553.9 |
| A: dt | 5 ms | 0.034363 | 0.203 s | 3948.7 |
| B: solref | 4 ms | 0.007516 | 1.608 s | 2487.7 |
| B: solref | 10 ms | 0.005941 | 1.404 s | 2849.2 |
| B: solref | 30 ms | 0.006248 | 0.573 s | 6980.2 |
| B: solref | 60 ms | **0.005654** | 0.740 s | 5408.7 |
| C: solimp | (0.70, 0.75) | **0.003700** | 0.622 s | 6431.4 |
| C: solimp | (0.90, 0.95) default | 0.006927 | 0.616 s | 6498.3 |
| C: solimp | (0.98, 0.99) | 0.009462 | 2.535 s | 1578.0 |

MuJoCo 在对应 A/B/C 范围内的最低值为 `0.003700`（C 的较低接触阻抗）。这仍显著低于 SAP 已测试最优 `0.347212`，但两者的 softness 参数不能作为相同物理量直接比较。

## 4. SAP Drake/CPU 结果（25 球）

| Sweep | 参数 | MSE `(m/s)^2` | wall time | steps/s |
| --- | --- | ---: | ---: | ---: |
| A: dt | 0.5 ms | 0.360258 | 1032.90 s | 7.75 |
| A: dt | 1 ms | 0.360058 | 627.18 s | 6.38 |
| A: dt | 2 ms | 0.359808 | 335.16 s | 5.97 |
| A: dt | 5 ms | 0.358023 | 133.45 s | 5.99 |
| B: tau | 4 ms | **0.347212** | 42.86 s | 93.33 |
| B: tau | 10 ms | 0.363305 | 52.67 s | 75.94 |
| B: tau | 30 ms | 0.360058 | 613.09 s | 6.52 |
| B: tau | 60 ms | 0.360185 | 667.02 s | 6.00 |
| C: ke | 1e3 | 0.508143 | 48.33 s | 82.77 |
| C: ke | 1e4 | **0.360058** | 598.81 s | 6.68 |
| C: ke | 1e5 | 0.399693 | 421.43 s | 9.49 |

### SAP 已测试最佳配置

```text
Drake preset
dt  = 1 ms
ke  = 1e4
tau = 4 ms
MSE = 0.34721228 (m/s)^2
```

该配置相对于 SAP `tau=30 ms` baseline 将 MSE 从 `0.360058` 降至 `0.347212`（约 3.6%）。`tau=4 ms` 同时使当前端到端 instrumented CPU runner 更快；这不是 SAP solver 内核吞吐的公平比较，因为 runner 每步包含 Python/Warp 调度、碰撞、状态读取和诊断开销。

## 5. MuJoCo–SAP 比较与解释

### 什么是公平的

本实验在以下任务级条件上对齐：场景、质量、外力、重力、时长、dt（每个对应 case）、有效 pair friction、误差公式、采样时刻，以及目标解析解。MuJoCo 的官方数值复现也支持任务和 metric 的正确性。

因此可以严谨地说：

> 在这个理想刚体、纯滚动解析 benchmark 中，已测试的 SAP Drake compliant-contact 配置比 MuJoCo 标准配置更偏离解析 reference。

### 为什么相同 mu 仍会不同

`mu` 只给出切向接触冲量的 Coulomb 上限；它不强制球保持无滑动纯滚动。SAP 中实际接触冲量还由以下因素决定：

- `ke` 与 `tau` 定义法向接触柔顺性、恢复时间尺度和阻尼；
- 每颗球的法向力不同，因此其摩擦上限 `mu*N` 不同；
- 多接触优化与正则化决定 25 颗球之间的切向冲量分配；
- 有限 dt 下的 stick/slip 转换与数值耗散会沿轨迹累积。

解析解假设每颗球得到理想对称的纯滚动接触力。当前 SAP 配置的结果表现为平台速度偏低、球速度偏高：外力冲量在 box 与球之间的分配不同于解析纯滚动模型。

### 结论边界

不应由本实验得出以下结论：

- “SAP 在所有接触任务上都比 MuJoCo 差”；
- “SAP 的核心 solver 必然更差”；
- “当前 wall time 可代表两者的 solver 内核速度差”。

原因是 SAP 的 `ke/tau` 本身是接触物理模型的一部分，MuJoCo 的 `solref/solimp` 也是；二者没有逐参数一一映射。当前 CPU SAP runner 也包含大量非 solver 的 Python/Warp/诊断开销。

## 6. 推荐的后续实验

1. 以 `dt=1 ms, ke=1e4, tau=4 ms` 作为 SAP 当前候选配置，验证重复运行一致性，并在其他解析任务（滑动、弹跳、斜面）测试泛化。
2. 为 MuJoCo 与 SAP 预先定义相同的调参预算，分别选择各自 native-best configuration，再在保留任务上评估，避免按测试结果事后调参。
3. 将性能分为：单场景 latency、去除 host-device 读取后的 pure-step runtime、批量多环境 throughput；不要用当前 instrumented single-world wall time 代表 GPU/SAP 的上限性能。
4. 在 Panda grasp、pinch、stack、impact 等无解析解任务中，以稳定性、滑移、穿透、能量行为、任务成功率和 dt 收敛替代单一 MSE。

## 7. 可复现产物

- MuJoCo runner：`exp-results/rolling/scripts/run_mujoco_rolling_benchmark.py`
- MuJoCo sweep：`exp-results/rolling/scripts/sweep_mujoco_rolling_settings.py`
- MuJoCo 对标 A/B/C sweep：`exp-results/rolling/scripts/sweep_mujoco_rolling_matched.py`
- SAP runner：`exp-results/rolling/scripts/run_sap_rolling_benchmark.py`
- SAP A/B/C sweep：`exp-results/rolling/scripts/sweep_sap_rolling_drake.py`
- MuJoCo 结果：`exp-results/rolling/mujoco/metrics.json`、`exp-results/rolling/mujoco-sweep/results.csv`
- MuJoCo 对标 A/B/C 结果：`exp-results/rolling/mujoco-matched-contact-sweep/results.csv`
- SAP 结果：`exp-results/rolling/sap-cpu-25ball-drake-sweep/results.csv`
