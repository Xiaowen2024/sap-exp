# GAUGE 七类刚体任务：MuJoCo vs SAP/Warp 独立重建（首轮）

日期：2026-10-17。**这不是 GAUGE 官方仿真流水线的原样复现，也不是完整材料 × 子任务矩阵。** 本轮每个有公开真实轨迹的刚体任务族取一个代表条件，使用相同公开 OBJ、实测质量/密度、几何初态、180 Hz 物理步长和 30 Hz 评分采样；比较 standalone MuJoCo 3.12.0 与本项目 SAP/Warp 1.16.0（CPU）。GAUGE 论文中的 Newton 1.3 MuJoCo backend、释放时刻对齐和评分脚本未在公开仓库中给出，不能把本报告数值当成论文表格的直接复现。

GAUGE 论文列出 **八** 类刚体任务；公开数据树中 wall breaking 缺少可评分 JSON，本轮覆盖其余 **七** 类。论文称实测物理参数输入引擎，其余设置维持默认；我们也区分“已直接匹配”与“没有等价输入因而未匹配”。[论文](https://arxiv.org/html/2608.05948) · [公开数据](https://huggingface.co/datasets/InternRobotics/GAUGE-Dataset) · [公开仓库](https://github.com/InternRobotics/GAUGE)

## 读结果前先看这三点

1. 主表 SAP 使用 **字面默认** shape `ke=2.5×10³ N/m`、`tau=10 ms`、`approx32` preset 和默认 `monotone_decay` line search；MuJoCo 使用默认 Newton、Euler、`solref/solimp`。为了不让 SAP 默认的 `rigid_gap=0.1 m` 提前激活接触，我们唯一固定了几何激活 `rigid_gap=0`，MuJoCo `margin=0`。SAP 的 `drake` preset 与较硬 `ke` 是另一个设置，不能叫默认。
2. 实测质量/密度和摩擦系数已尽量匹配，但 **实测恢复系数没有得到等价的物理实现**：SAP/Warp scene 已写入 metadata 的 `restitution` 字段，但当前 SAP 接触求解路径没有消费该字段；MuJoCo 原生软接触也没有一个可直接写入且保证 `e_eff=e_metadata` 的标量。弹跳球与牛顿摆的数字只能比较这两套默认接触响应，不能归因于“已匹配材料下哪个引擎更准”。
3. 表内误差不是 GAUGE 论文的官方 RMSE/DTW。我们对每个真实 trial 用该 trial 的第一帧位置重启仿真，取相同时间采样的 3D 位置，算 `RMSE_i = sqrt(mean_t ||p_sim,i(t)-p_real,i(t)||²)`，再对 trial 取均值；不优化释放时刻或时间偏移。单摆与牛顿摆使用相同 metadata 初始铰链角、零角速度，因此一条确定性仿真轨迹对 20 个真实 trial 评分。`real spread` 是真实 trial 对其他真实 trial 均值的 leave-one-out RMSE，只提供量级参照；其值接近仿真误差并不意味着已满足官方 benchmark。

## 首轮结果

数值为毫米，越低越好；每行评分窗口不同，**只可同行比较**。除注明的牛顿摆外，误差为被记录物体的 3D 位置 RMSE。主列使用 SAP 字面默认且已核实加载质量；真实 spread 定义见上文。

| 代表条件 | trial × 帧 | 真实 spread | MuJoCo 默认 | SAP/Warp 字面默认 | 直接观察 |
| --- | ---: | ---: | ---: | ---: | --- |
| 斜面接触，木制四面体 | 20 × 20 | 11.02 | 39.54 | 31.64 | 此重建与评分下 SAP 更接近真实轨迹；两者仍远高于真实间离散度。 |
| 非光滑接触，木制金字塔–方底 | 20 × 11 | 13.16 | 42.15 | 42.77 | 近似持平，差 0.62 mm，不宜排名。 |
| 固定斜板滑块，木块，30° | 20 × 19 | 18.94 | 20.39 | 20.98 | 近似持平；差异小于 trial 间变化。 |
| 转台，木块，task-1 | 20 × 186 | 2.16 | 8.24 | 6.66 | 使用同一真实转盘 yaw 轨迹作外部驱动；此设置下 SAP 小幅更接近实测轨迹。 |
| 弹跳球，软球 | 22 × 18 | 6.30 | 117.73 | 119.11 | 两边默认均严重低估反弹；**恢复系数未匹配**。 |
| 牛顿摆，末端球 | 20 × 150 | 34.98 | 78.20 | 81.43 | 两边默认均低于真实动量传递幅度；**恢复系数未匹配**。 |
| 单摆，金属球 | 20 × 300 | 134.12 | 275.57 | 323.73 | 两边周期偏长；SAP 默认 `midpoint` 积分比 MuJoCo/Euler 更长。 |

牛顿摆的 `real spread` 和误差均取被撞出的末端球；主动球误差另见原始 JSON。弹跳球公开目录有 22 个 JSON，论文写 20 次试验；本报告没有暗中删掉 2 个。

**如何概括这七行：**按当前默认与代表条件，SAP 在斜面接触和转台更接近实测轨迹；固定斜板滑块、非光滑接触、弹跳球和牛顿摆之间的数值差距很小或被未匹配的碰撞参数主导；MuJoCo 在单摆周期/长轨迹上更接近真实。没有一个覆盖七类的单向胜负。尤其不能拿弹跳球两者约 `1 mm` 的 RMSE 差来宣称接触模型优劣——两者相对真实都错约 `0.12 m`。

## 对齐论文 Table 3 材料与子任务的第二轮

上面的首轮表不是论文 Table 3 的逐行条件。论文四个非球接触条件是 slope contact **plastic/10 帧**、nonsmooth contact **plastic/task-3/20 帧**、slope slider **metal/task-3/60 帧**、turntable **metal/task-3/95 帧**。通过下载公开 20 条真实 trial、计算与论文相同的“各 trial 相对 trial 均值”离散度，后面三个的公开 baseline 分别为 `17.6708`、`38.0087`、`13.2558 mm`，与论文 `17.67`、`38.01`、`13.26 mm` 对上；子任务编号不再是猜测。[论文 Table 3](https://arxiv.org/html/2608.05948) · [候选任务审计 JSON](/Users/xiaowenyuan/Documents/SIM-EXP/exp-results/gauge-rigid-reproduction/paper_case_audit.json)

| 论文对应条件 | 本地真实 baseline (mm) | MuJoCo (mm) | SAP/Warp (mm) | 解释边界 |
| --- | ---: | ---: | ---: | --- |
| slope contact plastic，10 帧 | 9.25 | 22.20 | 19.39 | SAP 此配置略更接近公开轨迹；但公开 wood/plastic baseline 与论文表中的材料标签恰好交叉互换，见下。 |
| nonsmooth plastic，task-3，20 帧 | 17.67 | 150.52 | 152.52 | 两者都远超真实离散度，差距相对误差量级很小。 |
| slope slider metal，task-3，60 帧 | 38.01 | 49.00 | 507.27 | 两边重放相同实测斜板 6-DoF 变化；SAP 字面默认在动态倾斜板上反向滑移，参数/接触表示敏感，不能推广到所有 SAP 设置。 |
| turntable metal，task-3，95 帧 | 13.26 | 14.64 | 11.63 | 两边重放相同实测转盘 yaw，SAP 此配置小幅更接近轨迹。 |

第二轮实际输入的运动物体质量依次为 `0.0529 / 0.1463 / 0.3397 / 0.3397 kg`，SAP 加载质量分别为 `0.0528997 / 0.1463000 / 0.3396800 / 0.3396800 kg`。按明确写入脚本的 harmonic pair-mixing 假设，有效 `μ` 依次为 `0.182460 / 0.274344 / 0.262758 / 0.324373`，两个引擎均使用这些有效值（包含地面接触）。metadata 的 `e` 分别为 `0.307058 / 0.389192 / 0.333623 / 0.225054`，**仍未能等效映射到两边原生接触模型**；这些任务可称质量与摩擦输入对齐，不能称所有接触材料性质已完全对齐。

这些 RMSE 仍是**本项目的一 trial 一仿真、每条仿真对其对应真实 trial**，与论文的“仿真轨迹对真实均值”及其轨迹对齐方式不是同一评分协议，不能拿数值直接和论文 Isaac Sim/Genesis/Newton 列相减。对 slope contact，公开 wood 前 10 帧 baseline 是 `12.3126 mm`，plastic 是 `9.2543 mm`，而论文 Table 3/5 分别把 `9.25` 写在 wood、`12.31` 写在 plastic。数值精确交叉互换，**很像表格材料标签与公开文件不一致**；在确认作者数据版本/映射前，我们按公开目录的 material 命名报告，另外也跑了 wood/10 帧（MuJoCo `20.05`、SAP `20.45 mm`），没有擅自交换标签。修正了 MuJoCo 地面初版遗留的默认摩擦后，表内所有 MuJoCo 两体结果已重跑；地面与其他接触 geom 现都使用同一目标有效 `μ`。

斜板 case 的第一版曾把 board 固定成水平，导致两边 cube 几乎不动、误差都约 `134 mm`；这组**无效结果已排除**。公开 board quaternion 显示它在 trial 内明显倾转，我们现在逐步重放实测 board 平移/旋转相对于首帧的变化。首 trial 的驱动姿态审计：MuJoCo 板位置/姿态误差为 `0`，SAP 板位置误差最大约 `2.0×10⁻⁸ m`、姿态误差为 `0 rad`，排除了“板没被驱动到目标姿态”的解释。SAP `ke=2500 N/m, tau=10 ms, approx32` 下平均 `507.27 mm`；只在首个 trial 改用较硬的 `ke=1e5 N/m, tau=3 ms, drake` 后，SAP 从反向滑移转为正向，首 trial RMSE 为 `32.61 mm`。这说明**该默认材料设置不适合此高速驱动接触**，而不是已证明 SAP 无法模拟滑动。首 trial 的调参结果不能与 20-trial 默认结果充作同预算排名。

### 论文长窗口的撞击与无接触运动

- **Newton’s cradle，1027 帧：**论文用首撞动量传递效率 MTE 而非短窗轨迹 RMSE。按论文的高度比定义，我们在前 2 s 取首次外摆峰值，真实 20 trial 均值约 `0.929`；MuJoCo `0.278`，SAP 字面默认 `0.345`。两者都传递不足，SAP 此设置略高。该数值是明确标为 *first-swing proxy* 的本地实现；论文没有公开完整事件分割代码，不能把它称为官方 MTE。20-trial 的 1027 帧末球位置 RMSE 分别为 `45.84` 和 `84.89 mm`，说明单独看首撞效率与长轨迹误差可能给出不同排序。两者仍未匹配实测 `e=0.202387`。
- **Newton’s cradle 的 LSD 尚不能按论文数字复现：**论文给阈值 `0.5 mm` 的竖直位置范围和 `5 mm/s` 的静止速度，但未在公开代码中指定跟踪哪颗球、30 Hz 速度如何差分/滤波、是否排除首次撞击前静止段。我们在 20 条真实轨迹上试了主动/被动球、3D/竖直速度、中心/后向差分及起始排除；所得均值从约 `0.04 s` 到 `10.70 s`，没有无歧义地产生论文的 `0.38 s`。所以**没有**给 MuJoCo/SAP 的 LSD 排名，也没有挑一个恰好接近论文的定义冒充官方实现。[LSD 定义审计](/Users/xiaowenyuan/Documents/SIM-EXP/exp-results/gauge-rigid-reproduction/cradle_lsd_audit.json)。
- **Pendulum，3000 帧：**论文还看峰值势能近似的能量损失。我们使用同一 OBJ、质量、铰链和 `dt`，比较最后一个转折峰与释放峰：MuJoCo Euler `EL=0.00320`；SAP `approx32/midpoint` `EL≈−1.000`，出现明显数值能量增益；SAP `drake/sap_euler` `EL=0.00332`。进一步**只把 SAP `approx32` 的位置积分从 `midpoint` 改为 `sap_euler`**，不改精度和接触 preset 其余项，得到 `EL=0.00329`。因此反常的长时能量增益可归于此设置的积分离散化，而不是 SAP 接触求解器——该任务根本没有接触。MuJoCo 与 `sap_euler` 的相近数值也不意味着两者所有积分机制等价。

### 补充物理量，比一个 RMSE 更容易解释

- **单摆周期：**真实约 `1.167 s`；MuJoCo 约 `1.253 s`；SAP 字面默认 `approx32/midpoint` 约 `1.376 s`。同一场景改成 `approx64/midpoint` 仍约 `1.376 s`，而 `approx64/sap_euler` 回到约 `1.253 s`：差异主要对应 preset 的**位置积分方案**，不是 fp32/64 精度本身。SAP `drake/sap_euler` 也约 `1.253 s`。没有铰链阻尼或杆惯量的实测资料，不能把周期误差全归因于 solver。
- **弹跳球：**真实 trial 1 的反弹顶点约 `0.254 m`；本轮默认 MuJoCo 的后碰峰值均值约 `0.043 m`，SAP 字面默认约 `0.046 m`。两者的绝对轨迹误差都约 `0.12 m`，主要是默认接触耗散和未匹配的 `e=0.576733`，而不是两者之间 1–2 mm 的差别。
- **牛顿摆：**前 5 s 末端球的最大 `y`，真实均值约 `0.591 m`，MuJoCo 约 `0.509 m`，SAP 字面默认约 `0.521 m`。这比两边 RMSE 的数毫米差异更直观：两者都传递得不足。真实首帧主动球与 metadata 几何有约 2 mm 差异；当前按 metadata 角度释放，未做反推初角。
- **转台：**JSON 文件里确有 `plate.qw/qx/qy/qz`，虽然 metadata 只把 cube 列为评分对象。我们用每次记录的 plate yaw 增量驱动两个引擎相同的铰链，30 Hz 分段线性插值到 180 Hz；转盘 marker 的微小平移/倾斜没有施加到碰撞 mesh。SAP 直接移动 `kinematic` body 的第一版几乎不向方块传递切向运动，属于**当前 SAP/Warp 接口建模路径不等价的无效对照**；改成受控 revolute joint 并校验实际方块质量后，20 次均值是 SAP `6.66 mm`、MuJoCo `8.24 mm`。这只支持当前木块 task-1 的小幅差异，不支持所有转台工况的总排名。

## 弹跳球的释放对齐与恢复系数诊断（补充实验）

公开真实轨迹的前碰段（第 4–9 帧）按 `g=9.81 m/s²` 拟合，推得零初速释放时刻平均约 `0.0603 s`（标准差 `0.0047 s`），而不是评分起始的第 0 帧。仅将两个引擎改为从实测第 2 帧位置及前碰段拟合速度启动，其余仍用默认接触，22-trial 轨迹 RMSE 从 MuJoCo `117.73→109.21 mm`、SAP `119.11→108.23 mm`。这解释了小部分误差，但主要反弹误差仍在。

球 metadata 标注的 restitution 是 `0.576733`；然而从真实首碰前高度、地面高度与反弹顶点估计的**有效球–地恢复比**约为 `0.732±0.003`（22 次的均值±标准差）。这是按简化的垂直弹道和球半径推算的量，不应直接说 metadata 错误：真实平台的接触表面、释放/撞击速度和作者的参数测法可能不同。只按 metadata 数字调引擎，不保证真实弹跳轨迹正确。

我们还在首个 trial 上做了探索性参数匹配，再将参数冻结用于其余 21 个 trial：MuJoCo `solref=(12 ms, 0.41)` 得到 `64.74 mm`，SAP shape `ke=2500 N/m, tau=0.68 ms` 得到 `37.83 mm`。**这不是可采信的引擎精度排名**：首个 trial 的离散采样峰值不能充当可靠的逐碰 `e_eff`，且 SAP 此处为 `approx32`，下面的独立标定器使用 `drake` preset。它只能说明默认耗散不是唯一可行设置，不能说恢复行为已物理匹配。

为了检查参数能否迁移，另做无重力、无摩擦、单球–平面撞击：质量 `0.121 kg`，半径 `30 mm`，入射速度 `2.8 m/s`，从离地 `1–15 mm` 启动；从接触结束后的速度测 `e_eff=v^+/|v^-|`。同一组所谓“首 trial 匹配”参数，在 **180 Hz** 下仅改变初始离地间隙，MuJoCo 的 `e_eff` 就在 `0.02–1.23` 之间，SAP 在 `0.37–0.50` 之间；MuJoCo 出现 `e_eff>1`，说明这种粗步长/软接触组合可产生非物理能量增益。改为 **900 Hz**，取 `1 mm` 与 `15 mm` 两个间隙，MuJoCo 分别为 `0.348/0.311`，SAP 为 `0.699/0.689`：相位敏感性明显缩小，但两者并未被校准到相同的 `e_eff`。SAP 这里使用 `drake` preset，所以此诊断只说明参数迁移和离散化风险，不是默认 `approx32` 的完整消融。

**当前正确解读：**弹跳任务尚未完成稳健的等恢复系数、等接触时间标定；首轮七任务表中的弹跳行只比较各引擎默认响应。下一步应固定统一的单次撞击定义，同时匹配 `e_eff` 和接触时间，检验多个撞击相位/速度后冻结参数，再在 22 条真实轨迹上比较。标定代码与逐 case JSON 在 [`gauge_impact_calibration.py`](/Users/xiaowenyuan/Documents/SIM-EXP/exp-results/gauge-rigid-reproduction/gauge_impact_calibration.py) 和 [`impact_calibration`](/Users/xiaowenyuan/Documents/SIM-EXP/exp-results/gauge-rigid-reproduction/impact_calibration)。

## 参数与几何对齐审计

下面的数值是本轮**实际选中的七个代表条件**，不是 GAUGE 全材料/子任务的参数全集。质量取公开 metadata；括号中的 SAP 运行时加载质量用于核查 OBJ 密度换算。`μ` 是本轮两引擎共同使用的有效接触摩擦系数，不代表论文指定了该 pair-mixing 规则。

| 条件 | metadata 质量 → SAP 加载质量 (kg) | 本轮有效 `μ` | metadata `e` | `e` 是否实际匹配 |
| --- | ---: | ---: | ---: | --- |
| 斜面接触，木制四面体 | 0.083900 → 0.083900 | 0.388428 | 0.370309 | 否 |
| 非光滑接触，木制金字塔 | 0.043300 → 0.043300 | 0.273673 | 0.329297（task-3） | 否 |
| 固定斜板滑块，木块 | 0.090300 → 0.090295 | 0.268104 | 0.223583 | 否 |
| 转台，木块 | 0.090300 → 0.090295 | 0.348243 | 0.285485 | 否 |
| 弹跳球，软球 | 0.121000 → 0.120879 | 0.808698 | 0.576733 | 否；对结论影响极大 |
| 牛顿摆，每球 | 0.033000 → 0.032967 | 0.010000 | 0.202387 | 否；对结论影响极大 |
| 单摆，末端球 | 0.317500 → 0.317182 | 不涉及主要接触 | 未给出 | 不适用 |

所有条件用 `dt=1/180 s`、重力 `9.81 m/s²`。MuJoCo 使用 Euler 积分、默认 Newton solver、默认 `solref/solimp`；SAP/Warp 的主表使用 `approx32`、shape `ke=2500 N/m`、`tau=10 ms`、`rigid_gap=0`。除单摆等无接触段外，SAP 对两 shape 合成后的实际 pair 刚度为各自 shape 刚度的调和组合；因此表中 `2500 N/m` **不是** pair 刚度。两个相同 shape 接触时约为 `1250 N/m`，pair `tau` 为 `20 ms`。这些是引擎原生默认/组合语义，不是与 MuJoCo `solref` 对齐后的等效材料刚度。

- **匹配：**各任务的公开 OBJ；metadata 质量或密度（SAP mesh 按密度算质量，运行时检查与目标差 <2%）；每条 trial 的首帧运动物体位置与 metadata 初始 mesh 姿态；重力 `9.81 m/s²`；180 Hz 物理步长；30 Hz 采样；各任务同一试验记录。单摆铰链长 `0.278 m`、牛顿摆铰链长 `0.13263 m`。转台从 JSON 重放每个 trial 的外生 yaw；第二轮 metal slope slider 重放 board 的平移与旋转变化。
- **摩擦：**公开 JSON 给每个物体各自的 `mu`，但没有指定跨引擎 pair-mixing 公式。本报告先以 SAP 当前 harmonic combination `mu_pair=2 mu_A mu_B/(mu_A+mu_B)` 作为明确假设，然后把两边几何的摩擦都设成该有效 pair 值，避免 MuJoCo 的不同混合规则悄悄改变目标。它是**相同有效 pair 摩擦**，不等于已经证明 GAUGE 作者采用同一 mixing。
- **尚未匹配：**测得的恢复系数；真实释放帧与首速度；单摆/牛顿摆的杆和轴承损耗；转台 marker-to-collision-mesh 的完整变换；真实弹跳装置木板/支架（本轮是原球 OBJ + 刚性平面）。MuJoCo 用凸 mesh collider，SAP 用同一 OBJ 的 `convex_mesh`，接触点生成和惯量处理仍非完全同一实现。
- **接触容量：**修正质量后 SAP 场景的最大候选接触数远小于所配容量；没有把容量截断当作有效结果。
- **数据目录编号：**非光滑接触的 metadata 将金字塔–方底标为 `task-3`，公开轨迹目录将同一首个保留工况标为 `wood/task-1`；脚本显式作 `3→1` 映射，并核对首帧位置与 metadata 基本一致。

## 重要的已纠正错误

首轮脚本曾在 SAP `convex_mesh` 的 shape `cfg` 写 `mass`，但该 loader 实际按 `density × mesh volume` 构建刚体质量。例如斜面接触运动物体目标是 `0.0839 kg`，那版实际加载约 `0.1667 kg`。因此文件名不含 `mass-corrected` 的旧 SAP 两体结果，以及早期转台 trial 结果，**不得用于任何 MuJoCo–SAP 排名**。当前两体脚本改用 GAUGE 密度并在运行时断言实际质量；修正后该物体为 `0.0838996 kg`。弹跳球、牛顿摆、单摆原本已使用正确密度；它们加载质量分别约 `0.120879`、`0.032967`、`0.317182 kg`，与目标相差约 0.1%。

## 复跑入口与原始数据

结果和公开数据在 [`exp-results/gauge-rigid-reproduction`](/Users/xiaowenyuan/Documents/SIM-EXP/exp-results/gauge-rigid-reproduction)。主要脚本：[`two_body_compare.py`](/Users/xiaowenyuan/Documents/SIM-EXP/exp-results/gauge-rigid-reproduction/two_body_compare.py)、[`bouncing_ball_compare.py`](/Users/xiaowenyuan/Documents/SIM-EXP/exp-results/gauge-rigid-reproduction/bouncing_ball_compare.py)、[`newtons_cradle_compare.py`](/Users/xiaowenyuan/Documents/SIM-EXP/exp-results/gauge-rigid-reproduction/newtons_cradle_compare.py)、[`turntable_compare.py`](/Users/xiaowenyuan/Documents/SIM-EXP/exp-results/gauge-rigid-reproduction/turntable_compare.py)、[`sap_pendulum.py`](/Users/xiaowenyuan/Documents/SIM-EXP/exp-results/gauge-rigid-reproduction/sap_pendulum.py)、[`real_baselines.py`](/Users/xiaowenyuan/Documents/SIM-EXP/exp-results/gauge-rigid-reproduction/real_baselines.py)。完整 SAP 字面默认两体命令示例：

```bash
GAUGE_SAP_KE=2500 GAUGE_SAP_TAU=0.01 GAUGE_SAP_PRESET=approx32 GAUGE_RUN_TAG=mass-corrected-literal sap-sim/.sap-venv/bin/python exp-results/gauge-rigid-reproduction/two_body_compare.py sap slope-contact-wood
```

对应 MuJoCo：

```bash
mujoco-sim/.venv/bin/python exp-results/gauge-rigid-reproduction/two_body_compare.py mujoco slope-contact-wood
```

**下一步不是直接宣称引擎胜负。** 需要对恢复系数做同一参考撞击的行为校准并冻结参数，在弹跳球和牛顿摆上测 transfer；再扩展每个任务族的剩余材料与子任务，最后与 GAUGE 官方评分窗口/释放对齐规则比较。当前最稳健的初步事实是：在这七类代表条件中，差异具有强烈任务与 preset 依赖性；“真实 baseline 与论文一致”只验证了数据读取，不能验证仿真复刻。
