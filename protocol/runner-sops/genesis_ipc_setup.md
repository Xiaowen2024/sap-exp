# Genesis / IPC 环境与任务安排

2026-11-14 已验证：**Genesis 1.4.6 + pyuipc 0.0.29**（当日核对的最新发布版），Python 3.12.15、Torch 2.8.0+cu128、Quadrants 1.4.0。服务器使用 `/root/autodl-tmp/genesis-runtime/bin/python-genesis` 启动。

**Rigid** = Genesis 原生 RigidSolver；**两者** = 分别运行 Rigid 和 Genesis + IPC，各自选 B=5。两条路径的落地、初速度和恒力测试共 6 项已通过；**下表是任务安排，完整任务的 B=1/B=5/Generalizability 尚未运行**。

| 任务 | 运行方式 | Setup 与正式运行前待验证项 |
| --- | --- | --- |
| Three-box push | 两者 | 三盒堆叠、外力推盒；核对 IPC 外力、接触力和实际摩擦系数。 |
| Simple pinch | 两者 | 保留夹爪滑动关节、夹紧力、抬升控制及方块外载；验证关节控制和预载力读数。 |
| Three-plate stack | 两者 | 三圆板叠放，统一 μ=0.8；检查薄板初始接触、IPC 接触距离和层间载荷。 |
| GAUGE ball bounce | 两者 | 真实球网格、相同释放状态和地面；验证释放速度，沿用真实轨迹与 apex metric。 |
| ContactNets cube toss | 两者 | 同一方块、地面、质量和惯量；验证线速度、角速度输入，保留真实轨迹划分。 |
| Cube-toss resting tail | 两者 | 从原始状态释放；用 IPC 实际运动和支撑力判断静止，不使用被桥接层清零的速度。 |
| GAUGE slope slider | 两者 | 方块与静止／运动斜板；验证斜板轨迹输入、跟踪误差和摩擦配对。 |
| MIT Push | 两者 | 保留桌面、物体和推杆轨迹；分别核对桌面／物体与推杆／物体摩擦及推杆力。 |
| Round socket / peg-in-hole | 两者 | 保留已声明版本的真实孔形和间隙；验证初始几何与 IPC 接触距离，不自动采用 v6 草稿。 |
| GLIDE-inspired transfer | 两者，仅现有简化任务 | 两个运动支撑、刚性盘和三个刚性球；验证支撑轨迹与盘的碰撞几何，不改为软体番茄。 |
| Newton’s cradle | 两者，IPC 需先验证关节 | 保留五个固定锚点和铰链；解决无质量、无碰撞锚点的 IPC 表示，不能用自由球替代摆。 |
| Pendulum | Rigid | 无接触，使用原生关节动力学；保留 GAUGE setup 和已批准的真实数据 metric。 |
| Cassie | 先 Rigid，IPC 暂缓 | 先验证浮动基座、弹簧、状态与力矩映射；当前 IPC 固定基座关节接口不直接适用。 |
| DENSE/EZGripper | 先 Rigid，IPC 暂缓 | 先验证腱驱动与欠驱动关节，补齐物体／指令／参考数据；独立驱动手指不能替代原机构。 |

共同要求：沿用同一 manifest、选择格、留出格和 R=20 扰动；对齐几何、质量／惯量、实际接触对摩擦、输入与 dt。两条路径分别保留 literal-default B=1，再各自冻结 B=5；不改变原有 metric 或成功阈值。

IPC 注意：使用专用接口输入速度和力；角速度、运动轨迹、关节驱动和接触力读数仍待逐任务验证。`friction` 与 `coup_friction` 要分别设置并核对实际配对值；质量／几何变化需重建 IPC 场景。初始穿透不能通过偷偷挪动物体消除。默认配置 `d_hat=10 mm`，本次落地测试最终表面间隙约 8.62 mm；薄板和小间隙任务必须记录实际配置，不能在 B=1 前悄悄调参。

完整设置与 manifest 链接：[任务清单 JSON](genesis_ipc_task_setup.json)。版本、测试与运行入口：[环境记录](../../exp-results/genesis-ipc-setup-20261114/environment.json)、[依赖锁定](../../exp-results/genesis-ipc-setup-20261114/requirements-lock.txt)、[启动脚本](../../exp-results/genesis-ipc-setup-20261114/python-genesis.sh)。实现依据：[Genesis 1.4.6](https://github.com/Genesis-Embodied-AI/genesis-world/releases/tag/v1.4.6)、[libuipc 0.0.29](https://github.com/spiriMirror/libuipc/releases/tag/v0.0.29)、[IPC 桥接源码](https://github.com/Genesis-Embodied-AI/genesis-world/blob/v1.4.6/genesis/engine/couplers/ipc_coupler/coupler.py)。

清理记录：已释放磁盘 2.36 GB、共享内存 4.70 GB，结果／scripts／输入数据／现有环境均保留；详见[首次清理](../../exp-results/cleanup-20261114/server_cleanup.json)和[空闲后清理](../../exp-results/cleanup-20261114/deferred_cleanup.json)。
