# Box-stack high-stiffness sweep：SAP Warp vs MuJoCo

## 1. 这个实验测什么？

这是较早的 **native-parameter sweep**，不是 matched-compliance benchmark。两边各自使用原生接触参数，因此不能把 MuJoCo 的 `timeconst` 和 SAP 的 `ke` 当作相同物理刚度。

- 1、3、5、10 个 40 mm 立方体竖直堆叠。
- 每个 box 密度 1000 kg/m³，因此质量约 0.064 kg。
- 初始 ground gap 为 5 mm，层间 gap 为 1 mm，随后自由下落并碰撞。
- Box 使用 free joint，可以平移和旋转。
- 摩擦系数为 0.8，重力为 9.81 m/s²。
- 每条轨迹模拟 3 s，tail 指最后 0.5 s。

| 设置 | MuJoCo | SAP Warp |
|---|---|---|
| Solver | Newton，100 iteration budget | SAP，`drake` preset，100 iteration budget |
| Integrator | `implicitfast` | SAP Euler path |
| dt | 0.5、1、2、5 ms | 0.5、1、2、5 ms |
| 接触软硬度 | positive `solref timeconst` = 1、2、4、10 ms | shape `ke` = 1e3、1e4、1e5、1e6 N/m |
| Damping | `dampratio=1` | shape `tau=3 ms` |
| Gap | geom margin/gap 使用默认零 | `rigid_gap=0.1 mm` |
| 计算设备 | CPU | RTX GPU，single world |

MuJoCo 只保留 `timeconst >= 2 dt` 的有效组合，所以有 40 个 case；SAP 有 64 个 case。

重要修正：SAP CSV 中的 `ke_N_m` 和 `tau_ms` 是 **shape 参数**。相同材料接触时，当前组合规则给出 `pair ke = shape ke / 2`、`pair tau = 6 ms`，不是 CSV 表面看起来的 `pair ke = ke`、`pair tau = 3 ms`。

## 2. 记录了哪些 metric？

| Metric | 含义 | 越小越好吗？ | 注意事项 |
|---|---|---|---|
| Peak penetration | 整段 3 s 内最大的接触约束 penetration | 是 | MuJoCo 使用 `-contact.dist`；SAP 使用 `-phi0`，两者不是完全相同的几何量 |
| Final COM error | 最终 stack COM 与零间隙理想刚体 COM 的绝对误差 | 通常是 | SAP 设置了 0.1 mm rigid gap，因此会把有意保留的层间正间隙计入 error |
| Orientation drift | 整段轨迹最大姿态偏移 | 是 | 本实验完全对齐，不能强烈激发倾倒模式 |
| Settling time | 最后一次速度超过 1e-3 后的时间 | 是 | 达到 3 s 表示在模拟结束前未满足阈值 |
| Tail COM jitter | 最后 0.5 s 内 COM peak-to-peak 波动 | 是 | 只看 COM，可能掩盖不同层反向运动 |
| Max contacts | 整段轨迹最大同时接触候选数 | 不是 accuracy | 这是最大总数，不是每层平均接触点数 |
| Mean iterations | MuJoCo 每步 solver iteration 平均值 | 仅表示计算量 | SAP 此版本返回 -1，表示 telemetry 缺失，不是负迭代 |
| Wall time | 3 s trajectory 的 stepping 时间 | 是 | CPU MuJoCo 与 single-world GPU SAP 不能作为硬件归一化引擎速度排名 |
| Failure | 非有限状态、顺序错误、逃逸/倾倒或 contact truncation | False 最好 | 所有 104 个 case 均为 False |

这个旧实验 **没有记录接触力、force-balance error、每层 penetration 或每层平均接触点数**。因此不能从这份 CSV 计算新版 calibrated report 中的 `E_F`、`R_i` 和 `C_i`。

## 3. 每个 N、dt 下的最佳结果

这里的“最佳”只表示：在该引擎自己的参数中选择 final COM error 最小者；若相同，再比较 peak penetration。它不是两边 stiffness 已匹配后的公平 accuracy 排名。

### MuJoCo

|   N |   dt ms |   tc ms |   peak pen mm |   COM err mm |   tilt deg |   settle s |   tail jitter mm |   max contacts |   mean iters |   wall s |
|----:|--------:|--------:|--------------:|-------------:|-----------:|-----------:|-----------------:|---------------:|-------------:|---------:|
|   1 |     0.5 |       1 |        0.1012 |    0.0001434 |  0         |     0.0475 |        0         |              4 |       0.732  |  0.06449 |
|   1 |     1   |       2 |        0.1797 |    0.0005736 |  0         |     0.042  |        0         |              4 |       0.7294 |  0.03212 |
|   1 |     2   |       4 |        0.3366 |    0.002294  |  0         |     0.06   |        7.98e-14  |              4 |       0.7395 |  0.01561 |
|   1 |     5   |      10 |        0.1503 |    0.01434   |  0         |     0.06   |        4.788e-13 |              4 |       0.7604 |  0.00636 |
|   3 |     0.5 |       1 |        0.1752 |    0.0009082 |  0         |     0.056  |        6.064e-05 |             14 |       0.8485 |  0.1892  |
|   3 |     1   |       2 |        0.1982 |    0.003633  |  0         |     0.055  |        0         |             12 |       0.8124 |  0.08831 |
|   3 |     2   |       4 |        0.8656 |    0.01453   |  0.0006111 |     2.286  |        5.759e-13 |             14 |       0.8008 |  0.04481 |
|   3 |     5   |      10 |        1.212  |    0.0874    |  0.005924  |     0.125  |        9.885e-09 |             16 |       0.8918 |  0.0195  |
|   5 |     0.5 |       1 |        0.1752 |    0.002438  |  0         |     0.0545 |        7.648e-05 |             22 |       0.8034 |  0.3096  |
|   5 |     1   |       2 |        0.3208 |    0.009751  |  0.0001352 |     0.405  |        2.359e-13 |             22 |       0.7751 |  0.1546  |
|   5 |     2   |       4 |        0.8656 |    0.039     |  0.0007619 |     1.32   |        6.939e-14 |             22 |       0.8248 |  0.07761 |
|   5 |     5   |      10 |        2.183  |    0.2414    |  0.02675   |     0.225  |        9.957e-11 |             21 |       0.8735 |  0.03114 |
|  10 |     0.5 |       1 |        0.226  |    0.009608  |  0.0006444 |     0.4025 |        2.776e-14 |             42 |       0.8342 |  0.7093  |
|  10 |     1   |       2 |        0.4374 |    0.03443   |  0.01725   |     2.408  |        1.197e-05 |             46 |       1.011  |  0.3521  |
|  10 |     2   |       4 |        0.8656 |    0.1494    |  0.00708   |     3      |        0.004592  |             44 |       0.9993 |  0.1642  |
|  10 |     5   |      10 |        5.331  |    0.9184    |  0.06586   |     2.915  |        0.001283  |             44 |       1.057  |  0.06609 |

### SAP Warp

|   N |   dt ms |   shape ke N/m |   peak pen mm |   COM err mm |   tilt deg |   settle s |   tail jitter mm |   max contacts |   mean iters |   wall s |
|----:|--------:|---------------:|--------------:|-------------:|-----------:|-----------:|-----------------:|---------------:|-------------:|---------:|
|   1 |     0.5 |          1e+06 |      0.000516 |    0.0005164 |          0 |     0.0455 |         0        |              4 |           -1 |   26.1   |
|   1 |     1   |          1e+06 |      0.001131 |    0.001131  |          0 |     0.051  |         0        |              4 |           -1 |   12.67  |
|   1 |     2   |          1e+06 |      0.3366   |    0.002601  |          0 |     0.062  |         0        |              4 |           -1 |    6.759 |
|   1 |     5   |     100000     |      0.1503   |    0.008932  |          0 |     0.06   |         0        |              4 |           -1 |    2.723 |
|   3 |     0.5 |      10000     |      0.2893   |    0.0465    |          0 |     0.068  |         0        |             12 |           -1 |   22.06  |
|   3 |     1   |      10000     |      0.3008   |    0.0465    |          0 |     0.064  |         0        |             12 |           -1 |   11.12  |
|   3 |     2   |      10000     |      0.6837   |    0.0465    |          0 |     0.076  |         0        |             12 |           -1 |    5.581 |
|   3 |     5   |     100000     |      1.391    |    0.04568   |          0 |     0.09   |         0        |             12 |           -1 |    2.355 |
|   5 |     0.5 |      10000     |      0.4385   |    0.1453    |          0 |     0.113  |         0        |             20 |           -1 |   22.95  |
|   5 |     1   |      10000     |      0.4284   |    0.1453    |          0 |     0.114  |         0        |             20 |           -1 |   11.49  |
|   5 |     2   |      10000     |      0.8958   |    0.1453    |          0 |     0.118  |         0        |             20 |           -1 |    5.773 |
|   5 |     5   |     100000     |      2.683    |    0.05629   |          0 |     0.14   |         0        |             20 |           -1 |    2.446 |
|  10 |     0.5 |     100000     |      0.1281   |    0.3291    |          0 |     0.075  |         0        |             40 |           -1 |   22.43  |
|  10 |     1   |     100000     |      0.3575   |    0.3291    |          0 |     0.078  |         0        |             40 |           -1 |   11.8   |
|  10 |     2   |     100000     |      0.6089   |    0.2817    |          0 |     0.092  |         0        |             40 |           -1 |    5.778 |
|  10 |     5   |     100000     |      7.244    |    0.1095    |          0 |     0.365  |         2.98e-06 |             40 |           -1 |    4.22  |

## 4. Stiffness / softness 趋势

固定最重场景 `N=10, dt=0.5 ms`。

### MuJoCo：改变 timeconst

MuJoCo positive-format `timeconst` 越大，接触越软。

|   tc ms |   peak pen mm |   COM err mm |   tilt deg |   settle s |   tail jitter mm |   max contacts |   mean iters |   wall s |
|--------:|--------------:|-------------:|-----------:|-----------:|-----------------:|---------------:|-------------:|---------:|
|       1 |        0.226  |     0.009608 |  0.0006444 |     0.4025 |        2.776e-14 |             42 |       0.8342 |   0.7093 |
|       2 |        0.4442 |     0.03843  |  0         |     0.1035 |        0         |             40 |       0.8792 |   0.6286 |
|       4 |        0.9049 |     0.1535   |  0         |     0.58   |        0.007966  |             44 |       0.9968 |   0.6544 |
|      10 |        1.971  |     0.9184   |  0.0015    |     0.6665 |        2.587e-10 |             42 |       1.015  |   0.6728 |

明显趋势：

- `tc: 1 -> 10 ms` 时，peak penetration 从 0.226 增至 1.971 mm。
- Final COM error 从 0.00961 增至 0.918 mm，约增加 96 倍。
- Mean iterations 只从 0.834 增至 1.015；该小系统通常一两个 Newton iteration 就收敛。
- Settling time 非单调，因此不能只用 softness 预测 transient settling。

### SAP：改变 ke

SAP shape `ke` 越大，请求的接触越硬；相同两侧材料的实际 pair stiffness 为表中数值的一半。

|   shape ke N/m |   peak pen mm |   COM err mm |   tilt deg |   settle s |   tail jitter mm |   max contacts |   mean iters |   wall s |
|---------------:|--------------:|-------------:|-----------:|-----------:|-----------------:|---------------:|-------------:|---------:|
|       1000     |        6.328  |      11.66   |          0 |      2.841 |           0.1074 |             40 |           -1 |    38.89 |
|      10000     |        0.7599 |       0.7586 |          0 |      0.296 |           0      |             40 |           -1 |    25.13 |
|     100000     |        0.1281 |       0.3291 |          0 |      0.075 |           0      |             40 |           -1 |    22.43 |
|          1e+06 |        0.1036 |       0.4168 |          0 |      0.076 |           0      |             40 |           -1 |    22.55 |

明显趋势：

- `ke: 1e3 -> 1e5 N/m` 时，peak penetration 从 6.328 降到 0.128 mm，COM error 从 11.657 降到 0.329 mm。
- `1e5 -> 1e6 N/m` 的收益明显减弱：peak 只从 0.128 降到 0.104 mm，COM error反而从 0.329 增至 0.417 mm。
- 这说明当前配置进入 diminishing-return 区域；结合后续诊断，更合理的解释是 0.1 mm gap/几何参考开始主导，而不是仅凭这张表断言 solver stiffness saturation。

## 5. Stack size N 的趋势

固定 `dt=0.5 ms`。MuJoCo 固定 `tc=1 ms`；SAP 固定 `shape ke=1e5 N/m`。

### MuJoCo

|   N |   peak pen mm |   COM err mm |   tilt deg |   settle s |   tail jitter mm |   max contacts |   mean iters |   wall s |
|----:|--------------:|-------------:|-----------:|-----------:|-----------------:|---------------:|-------------:|---------:|
|   1 |        0.1012 |    0.0001434 |  0         |     0.0475 |        0         |              4 |       0.732  |  0.06449 |
|   3 |        0.1752 |    0.0009082 |  0         |     0.056  |        6.064e-05 |             14 |       0.8485 |  0.1892  |
|   5 |        0.1752 |    0.002438  |  0         |     0.0545 |        7.648e-05 |             22 |       0.8034 |  0.3096  |
|  10 |        0.226  |    0.009608  |  0.0006444 |     0.4025 |        2.776e-14 |             42 |       0.8342 |  0.7093  |

### SAP Warp

|   N |   peak pen mm |   COM err mm |   tilt deg |   settle s |   tail jitter mm |   max contacts |   mean iters |   wall s |
|----:|--------------:|-------------:|-----------:|-----------:|-----------------:|---------------:|-------------:|---------:|
|   1 |      0.003127 |     0.003128 |          0 |      0.044 |                0 |              4 |           -1 |    26.71 |
|   3 |      0.05676  |     0.08533  |          0 |      0.053 |                0 |             12 |           -1 |    21.98 |
|   5 |      0.05788  |     0.1654   |          0 |      0.064 |                0 |             20 |           -1 |    22.21 |
|  10 |      0.1281   |     0.3291   |          0 |      0.075 |                0 |             40 |           -1 |    22.43 |

明显趋势：

- 随 N 增大，两边的 peak penetration 和 COM error 总体增大，因为底层承担更多载荷，同时接触数量增加。
- MuJoCo COM error 从 N=1 的 0.000143 mm 增至 N=10 的 0.00961 mm。
- SAP COM error 从 0.00313 mm 增至 0.329 mm。这里同时包含 compliant compression 和每层 0.1 mm rigid-gap reference bias。
- Max contacts 大致随 N 线性增加。SAP 为 4N；MuJoCo transient 中达到 4、14、22、42，偶尔高于理想静态 4N。
- MuJoCo wall time 随 N 从 0.0645 s 增至 0.709 s。SAP single-world wall time约 22–27 s，没有呈现简单的 N 线性关系，GPU launch/solver overhead 占主导。

## 6. Timestep dt 的趋势

固定 `N=10`。MuJoCo 使用在全部 dt 下都有效的 `tc=10 ms`；SAP 使用 `shape ke=1e5 N/m`。

### MuJoCo

|   dt ms |   peak pen mm |   COM err mm |   tilt deg |   settle s |   tail jitter mm |   max contacts |   mean iters |   wall s |
|--------:|--------------:|-------------:|-----------:|-----------:|-----------------:|---------------:|-------------:|---------:|
|     0.5 |         1.971 |       0.9184 |   0.0015   |     0.6665 |        2.587e-10 |             42 |        1.015 |  0.6728  |
|     1   |         2.112 |       0.9184 |   0.002588 |     1.565  |        4.269e-10 |             42 |        1.021 |  0.3234  |
|     2   |         2.393 |       0.9184 |   0.003322 |     0.21   |        7.575e-09 |             40 |        1.015 |  0.1606  |
|     5   |         5.331 |       0.9184 |   0.06586  |     2.915  |        0.001283  |             44 |        1.057 |  0.06609 |

### SAP Warp

|   dt ms |   peak pen mm |   COM err mm |   tilt deg |   settle s |   tail jitter mm |   max contacts |   mean iters |   wall s |
|--------:|--------------:|-------------:|-----------:|-----------:|-----------------:|---------------:|-------------:|---------:|
|     0.5 |        0.1281 |       0.3291 |          0 |      0.075 |         0        |             40 |           -1 |   22.43  |
|     1   |        0.3575 |       0.3291 |          0 |      0.078 |         0        |             40 |           -1 |   11.8   |
|     2   |        0.6089 |       0.2817 |          0 |      0.092 |         0        |             40 |           -1 |    5.778 |
|     5   |        7.244  |       0.1095 |          0 |      0.365 |         2.98e-06 |             40 |           -1 |    4.22  |

明显趋势：

- 两边 wall time 都大致随 timestep 增大而下降，因为 step 数减少。
- MuJoCo 的 final COM error 在固定 `tc=10 ms` 下约为 0.918 mm，几乎不随 dt 变化；但 peak penetration 从 1.97 增至 5.33 mm。
- SAP peak penetration 对 dt 更敏感：从 0.128 mm 增至 7.244 mm。
- SAP final COM error 在粗 dt 下反而下降，不能解释为 accuracy 提高；因为 peak penetration 同时恶化约 57 倍，而且 zero-gap COM reference 与 0.1 mm rigid gap 并不一致。
- 5 ms 时两边 transient penetration 和 tilt/jitter 都更差，说明 coarse timestep 主要伤害碰撞瞬态。

## 7. 其他 metric 的总体结论

- **Failure：** MuJoCo 40/40、SAP 64/64 均未触发 failure flag，也没有 contact truncation。
- **Orientation drift：** SAP 全部为 0；MuJoCo 最大仅 0.0659 deg。这个对齐场景无法有效区分姿态稳定能力。
- **Tail jitter：** 大部分 case 接近零；最大值为 MuJoCo 0.00797 mm、SAP 0.107 mm。由于只测 COM，这个 metric 对层间局部振动不够敏感。
- **Settling：** MuJoCo 范围 0.042–3.0 s；SAP 0.036–2.841 s。它受接触软硬度和 dt 共同影响，整体不是单调函数。
- **Runtime：** MuJoCo 为 0.006–0.709 s；SAP 为 2.30–38.89 s。这里只能描述这两个脚本的 single-trajectory wall time。

## 8. 可以从旧实验得出什么？

1. 增大 N 会增加载荷累积、COM error、接触数量和 MuJoCo 计算时间。
2. MuJoCo 增大 positive `timeconst` 会系统性增加静态 compliance error。
3. SAP 从 `ke=1e3` 提升至 `1e5 N/m` 有明显收益，继续提升至 `1e6` 收益很小，并受到 rigid-gap/几何参考影响。
4. 增大 dt 会显著恶化 peak penetration，尤其是 SAP 的 10-box transient。
5. 该实验没有 force telemetry，也没有匹配两边 effective compliance，因此不能用来断言哪一个 solver 的物理刚度更准确；新版 calibrated stack 才负责回答那个问题。

## 9. 数据与复现

- MuJoCo runner：`mujoco-sim/learn_mujoco/stacking_experiments/run_box_stack_sweep.py`
- SAP runner：`sap-sim/scripts/run_box_stack_sweep.py`
- MuJoCo CSV：`exp-results/box-stacking/mujoco/results.csv`
- SAP CSV：`exp-results/box-stacking/sap/results.csv`
- SAP high-stiffness diagnostic：`exp-results/box-stacking/sap-high-stiffness-diagnostic/results.csv`
