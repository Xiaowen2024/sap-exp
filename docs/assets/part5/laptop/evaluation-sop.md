# 闭合笔记本：按叠碗 SOP 评估完整 run

沿用[用户提供的叠碗评估 SOP](../stack-bowls/sop.md)，把评价对象替换为一台已关闭的矩形笔记本。只重新整理已经完成的完整 run，不启动模拟、不重新调参。机器人轨迹是输入，物体运动是评价对象。

## 1. Physical Fidelity

Object-trajectory error：可靠可见帧上的物体中心 RMSE 与可辨识的姿态误差，保持原始时间对齐。矩形笔记本不自动套用碗的轴对称豁免。参考只能作为 estimated reference；报告有效物体/帧覆盖率、方法与不确定度。初始场景拟合、夹爪跟踪、COM 在夹爪系的变化和两引擎之间的差值不能替代该误差。

Final-pose error：在相同释放后源视频窗口内，用可靠的多帧/多视角估计比较位置与姿态。本次共同源窗口为 21.50–21.96 s（回放 12.50–12.96 s）。没有接纳的参考时留空，不能把追加静止画面当作参考。误差差异小于参考不确定度时不判优。

## 2. Simulation Consistency

Geometric validity：按物体对报告共同几何的最大采样穿透、发生时间、采样间隔和覆盖；结合局部壁厚、几何近似与穿墙检查解释。本次已有 10 ms 共同凸几何观察，1447 帧，覆盖回放 0–14.46 s。数值是采样最大凸组件对穿透的上下界，不是精确并集穿透；宽区间不强行取上界排名。局部壁厚/穿墙审核未完成即留空。

Contact continuity：仅用预先声明的持续抓持/支撑窗口，逐物体对报告承载 duty、最长无承载时间和接触恢复次数。候选接触与承载接触分开。阈值 0.01 N，另查 0.001/0.1 N；接触按 1 ms 记录。左指对 [3.2,7.6] s，右指对 [8.3,11.6] s，架子支撑 [12.3,14.46] s。架子窗口包含追加观察段，必须注明。正常接触切换不自动判为错误，双指合并缺失也不能直接判脱手。

Post-release settling：主结果使用源视频内相同的释放后窗口：源时间 [21.5,21.96] s，对应回放 [12.5,12.96] s，每引擎 461 个 1 ms 状态样本；报告物体 COM 线速度 RMS、世界角速度 RMS、峰值速度和净漂移。RMS 为 sqrt(integral(||v||² dt)/duration)。路径长度和倾角波动没有现成有效结果时留空。[13.96,14.46] s 的追加尾窗仅作补充。稳定不等于末态正确。

## 3. Task Outcome

逐 episode 判断：笔记本是否插入竖架、夹爪完全释放后是否保持。参考和模拟分别记录 observed outcome。明显穿透或持续托举不计有效成功；必要有效性审核缺失时 valid_success=null。只在有效标签能确定时填写严格 outcome agreement、false success/false failure。单条不报成功率。

## 4. 输入检查与报告

夹爪位置/姿态跟踪、抓持过程中 COM 相对变化、最大抬升均作为独立诊断，不进入物体复刻质量主表。输入检查未通过会限制因果解释，但不能自动替代物体任务结果。所有缺失项记 null / 未评估，不记零。

本次离线整理命令：

```sh
python3 docs/assets/part5/laptop/summarize-existing.py docs/assets/part5/laptop
```

输出 `full-sop-summary.json`，引用原始 `full-metrics.json`、`full-stages.json` 和参考拟合尝试的 SHA-256。只提供 Markdown/网页，不创建 Canvas。
