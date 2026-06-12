# 按摩机器人手法学习与实验评价论文调研

调研日期：2026-05-31  
关注问题：按摩机器人如何在实验中定量分析“按摩手法学习效果”，采用了哪些学习模型，以及如何评价模型效果。

## 结论摘要

1. 按摩机器人论文里，“临床疗效评估”和“手法学习效果评估”是两条不同线索。系统综述显示，临床研究常用 VAS 疼痛评分、ODI、EMG、唾液分泌、皮温、超声肌肉特征、舒适度问卷等指标，但很多研究并没有真正评价学习算法本身。
2. 真正面向手法学习的论文，大多把按摩手法拆成可测的运动和力信号，再用轨迹误差、力跟踪误差、频域特征、收敛迭代次数、跨受试者泛化效果等代理指标评价学习效果。
3. 目前常见模型路线有三类：  
   示教学习：DMP、GMM/GMR、DTW、动力系统 DS、SVR 表面建模。  
   强化学习：模型化 RL、残差 RL、DQN、PPO/ICM、SAC，与阻抗/导纳控制结合。  
   感知和规划学习：深度视觉识别穴位/身体区域、RGB-D 点云、ICP 轨迹更新、B 样条规划，通常不直接学习手法，但支撑自动按摩。
4. 最扎实的定量指标目前集中在“力控稳定性”：目标力通常设为 4 N、5 N、5.5 N 或 6 N，优秀结果多报告在线力误差约在 `±0.2 N` 或 `0.1 N` 以内。轨迹规划类工作则报告毫米级定位误差，例如 VBRM 框架的 3D 轨迹规划精度为 `4.3 ± 2.6 mm`。
5. 现有不足：很多论文只展示曲线或截图，缺少统计检验、训练/测试拆分、跨人体泛化验证、舒适度和临床疗效联合评价。后续实验最好同时报告“算法层指标、手法层指标、用户/临床层指标”。

## 核心论文对照

| 论文 | 任务和手法 | 学习/控制模型 | 实验量化指标 | 关键结果 | 局限 |
| --- | --- | --- | --- | --- | --- |
| Li et al., 2020, Frontiers, *An Enhanced Robot Massage System...* | KUKA LBR iiwa 通过示教学习背部/肩部按摩路径 | FD 分段，DMP 表示运动，DTW 对齐多次示教，GMM/GMR 综合轨迹，混合力/位置控制 | 正弦轨迹示教再现，关节角曲线，末端 XYZ 接触力，不同参与者身体厚度适应 | 5 次有缺陷示教可生成更平滑轨迹；一次按摩示教可在不同参与者复现；Z 向接触力随人体厚度不同约为 `[-3.5,-5] N` 与 `[-5,-6.5] N` | 学习效果主要用曲线和定性描述，没有报告轨迹 RMSE/MAE |
| Khoramshahi et al., 2020, ICRA, *Toward compliant robotic massage* | KUKA iiwa + Allegro 手，在假人手臂上复现拇指圆周按摩和法向力模式 | 专家示教的运动-力模式用极坐标动力系统 DS 编码，LWR/RBF 学习相位到期望力，SVR 学习非平面表面距离和法向，阻抗控制执行 | OptiTrack 120 Hz 和 FingerTPS 40 Hz 采集专家运动/压力；低通滤波 2 Hz；验证到达表面、沿表面运动、按相位施力 | 能在非平面表面上保持姿态、沿手臂移动并输出随相位变化的按摩力 | 作者明确指出力和速度跟踪误差仍受 Allegro 手感知/执行精度限制，缺少数字化误差表 |
| Xiao et al., 2023, *Intelligent Service Robotics* | 未知皮肤环境下机器人按摩力控制 | 阻抗控制建立初始接触模型，皮肤力学模型估计边界，BP 神经网络建立状态转移模型，交叉熵法搜索控制参数，模型化强化学习 | 与 PID 对比，在线实验接触力误差 | 力比传统 PID 更平滑；在线力误差基本在 `±0.2 N` 内 | 摘要层面公开指标有限，完整实验细节需读全文 |
| Xiao et al., 2024, IEEE Access, *Residual Reinforcement Learning* | 机器人按摩恒力控制 | 阻抗控制作为初始策略，强化学习学习残差补偿位移；神经网络拟合接触动态模型，离线训练，输出残差平滑 | 离线收敛迭代次数，在线力误差 | 约 `80` 次离线迭代后收敛；在线力误差基本在 `±0.2 N` 内 | 主要评价力控，没有评价真实疗效和复杂手法相似度 |
| Xiao/Zhang et al., 2024, Frontiers, *GMM/GMR fusing compensation strategies* | 机器人沿手臂皮肤滑动时保持 5 N 接触力 | 阻抗控制基础上，DQN 学习位移补偿；BP 和 LSTM 分别构建环境动态模型；Hunt-Crossley 皮肤力学模型在线补偿；GMM/GMR 融合三种补偿策略 | 50 Hz 控制，2 mm/s 滑动，目标力 5 N；指标含 `|e|max`、MAE、误差标准差；与阻抗控制和模型化 RL 对比 | DQN 在约 `40-50` 次迭代收敛；融合策略力误差稳定在 `±0.2 N`；四组实验中 MAE 相比阻抗控制降低约 `45.7%-87.5%`，相比模型化 RL 降低约 `35.7%-74.4%` | 只测两名志愿者手臂和双向直线路径，临床/舒适度评价不足 |
| Li et al., 2024, IEEE SMC Magazine, *Learning Variable Impedance Control...* | Baxter 搭载按摩工具，学习身体表面柔顺运动和力控 | 深度强化学习 + 可变阻抗控制；二级资料归纳为 PPO + ICM，输入接触力和位置误差，输出运动轨迹与时变阻抗参数 | 轨迹对齐、力跟踪、舒适度；参考力约 5.5 N | 二级综述称 PPO+ICM 比 PPO 单独使用更平滑，力更贴近 5.5 N，固定阻抗相比舒适度低约 18% | 原文为 IEEE Magazine，公开摘要有限；具体实验设置需以原文为准 |
| Wang et al., 2024, JPCS, *Compliance control... dynamic scenes* | 动态场景中康复按摩机器人恒力控制 | Soft Actor-Critic (SAC) 深度强化学习，构建背部按摩仿真环境和奖励函数，实现恒力控制 | 仿真中轨迹跟踪和目标力跟踪误差 | 摘要报告实际接触力与目标力控制在 `0.1 N` 内 | 主要为仿真实验，真实人体泛化和安全评价仍需补足 |
| Zhang et al., 2024, *Industrial Robot*, autonomous path planning and force control | 未知人体组织环境下自动规划背部按摩路径并稳定接触力 | 深度学习进行背部区域提取和穴位识别，3D 重建规划路径，在线自适应力跟踪控制 | 穴位识别准确率，在线力误差 | 摘要报告改进网络可高准确率识别穴位，在线力误差基本在 `±0.2 N` 内 | 学习部分主要是感知识别，不是完整手法策略学习 |
| Xu et al., 2024, Complex & Intelligent Systems, VBRM | 操作者在 RGB 图像上画 2D 轨迹，机器人自动生成 3D 按摩轨迹并力控执行 | RGB-D 点云、2D 到 3D 投影、B 样条平滑、ICP 动态更新、PBVS 视觉伺服、PID 法向力控制 | 3D 轨迹规划误差、ICP RMSE、目标力跟踪曲线 | 轨迹规划精度 `4.3 ± 2.6 mm`；去掉两个深度异常点后为 `2.78 ± 1.19 mm`；平移/旋转 ICP inlier RMSE 分别为 `0.006` 和 `0.004`；目标力 6 N | 不是学习手法模型，但给出了自动按摩系统中可复用的轨迹/力控评价范式 |
| Xu et al., 2024, arXiv, *Digital Modeling of Massage Techniques...* | 数字化建模中医按摩的 beat、press、push、vibrate 四类手法 | OptiTrack + 按摩力测量仪采集专家手法；自适应导纳控制复现；根据手法设计轨迹和力函数 | 专家和机器人力信号的最大/最小值、均值、标准差、偏度、峰度、主频 | 例如专家/机器人主频：beat `2.79/1.00 Hz`，press `0.14/0.19 Hz`，push `0.36/0.88 Hz`，vibrate `7.49/7.33 Hz`；vibrate 频率最接近专家 | 预印本，模型更多是数字化规则复现，不是统计意义上的学习模型 |

## 定量分析按摩手法学习效果的常用指标

### 1. 力控指标

最常见，也是最适合先做的指标。

- 力误差：`e_f(t)=F_actual(t)-F_ref(t)`。
- 最大绝对误差：`max |e_f|`，用于安全边界。
- 平均绝对误差 MAE：衡量整体贴近目标力的能力。
- RMSE：对突发大误差更敏感，适合动态人体或曲面跟踪。
- 标准差：衡量力输出是否稳定。
- 超调量和恢复时间：评价接触瞬间是否容易顶压人体。
- 频域指标：主频、频谱能量，用于 tapping、vibration、kneading 等周期手法。

代表性论文中，Xiao 系列工作基本都以 `±0.2 N` 作为在线力控是否达标的量化标准；SAC 仿真工作报告 `0.1 N` 内的目标力误差。

### 2. 轨迹和姿态指标

用于评价机器人是否学会了专家的手法路径，尤其是沿背部、肩颈、手臂曲面运动。

- 末端位置误差：欧氏距离，单位 mm。
- 姿态误差：法向夹角、旋转向量误差。
- 轨迹 RMSE/MAE：机器人轨迹与专家示教轨迹或规划轨迹的差异。
- DTW 距离：适合不同速度的示教轨迹对齐。
- 平滑性：速度、加速度、jerk，避免按摩动作生硬。
- 在线更新误差：如 ICP inlier RMSE，用于人体移动时的轨迹重定位。

VBRM 工作给出了较清晰的轨迹评价范式：先用 AR 标记验证 2D 到 3D 规划误差，再在动态假体上用 ICP RMSE 和力控曲线验证实时更新。

### 3. 手法特征相似度指标

适合回答“机器人是否学到了某类按摩手法的风格”。

- 时间域：峰值、谷值、均值、标准差、偏度、峰度。
- 周期域：主频、周期稳定性、相位偏差。
- 力-位移相位关系：如按压时下压位移和法向力是否同步。
- 轨迹形状：圆周、往复、敲击、振动等几何形态。
- 与专家信号的相关系数、DTW 距离、频谱相似度。

arXiv 2024 的数字化手法建模论文虽然不是严格学习算法，但给出了一个可直接借鉴的手法量化模板：对专家和机器人都计算 `Max/Min/Mean/Std/Skew/Kurt/Frequency`，再逐项比较。

### 4. 学习算法指标

用于比较不同模型的学习效率和泛化能力。

- 收敛迭代次数：如 DQN 约 `40-50` 次，残差 RL 约 `80` 次。
- 累计回报曲线：强化学习常用。
- 训练样本量和真实交互次数：按摩涉及人体安全，越少越好。
- 跨受试者泛化：在不同皮肤刚度、体型、曲率下是否维持误差。
- 消融实验：去掉皮肤模型、去掉 LSTM、去掉 GMM/GMR 融合、固定阻抗 vs 可变阻抗。
- 成功率：完成整条路径且未超过安全力阈值的比例。

### 5. 用户和临床指标

如果目标是证明按摩有效，必须在算法指标之外加上人体反馈。

- 主观疼痛：VAS。
- 功能障碍：ODI，常用于腰痛。
- 肌肉疲劳和放松：sEMG 电活动 EA、median frequency。
- 肌肉硬度/厚度：超声、硬度指数。
- 自主神经和放松：心率、HRV、皮温、唾液淀粉酶。
- 舒适度和偏好：Likert/VAS 问卷。
- 安全：不良事件、最大接触力、急停次数。

系统综述 Yang et al. 2024 纳入 17 篇机器人按摩研究、841 名成人，其中只有 1 篇随机对照试验。它说明临床疗效证据仍偏早期，因此做学习算法论文时，应避免只用临床主观结果替代手法学习评价。

## 学习模型分类和适用场景

### 示教学习：DMP、GMM/GMR、DTW、DS

适合有治疗师示教、希望机器人复现专家轨迹的场景。

- DMP 适合把单个按摩动作建成可缩放的运动基元。
- DTW 适合对齐多次示教的时间差异。
- GMM/GMR 适合从多次示教中生成平均、平滑、抗噪的轨迹。
- DS 适合周期或半周期动作，如揉、振、敲击。

建议评价：专家-机器人轨迹 RMSE、DTW 距离、频率误差、力曲线相关系数、跨身体部位泛化。

### 强化学习：模型化 RL、残差 RL、DQN、PPO、SAC

适合皮肤刚度未知、人体会移动、接触力需要自适应的场景。

- 模型化 RL 用神经网络学习皮肤接触状态转移，减少真实人体试错。
- 残差 RL 在传统阻抗控制上学习补偿量，比从零学习更安全。
- DQN 适合离散补偿位移选择；可以与 BP/LSTM 环境模型结合。
- PPO/ICM 或 SAC 适合连续动作空间和可变阻抗参数学习。

建议评价：力误差 MAE/RMSE、最大误差、收敛迭代、真实交互次数、不同志愿者/不同路径的泛化、与固定阻抗/导纳控制对比。

### 感知驱动规划：深度视觉、点云、ICP、B 样条

适合自动寻找按摩区域、穴位、背部曲面路径。

- 深度学习可用于身体区域和穴位识别。
- RGB-D 点云可用于生成曲面轨迹。
- ICP 可用于人体移动时在线更新轨迹。
- B 样条适合把采样轨迹变成平滑路径。

建议评价：定位误差 mm、识别准确率、动态跟踪 RMSE、完成时间、力控安全性。

## 对后续实验设计的建议

### 推荐的最小实验闭环

1. 选择 3-4 个标准手法：按压、推、揉/圆周、振动或叩击。
2. 采集专家数据：末端位姿、速度、法向力、切向力、接触面积或压力阵列，采样频率建议至少 50 Hz，周期动作建议更高。
3. 建立两个基线：固定阻抗/导纳控制、示教轨迹直接回放。
4. 加入学习模型：优先考虑 `DMP/GMM-GMR` 做轨迹学习，`残差 RL 或 DQN` 做力补偿。
5. 在软体假体上调参，再做人体志愿者小样本实验。

### 建议报告的核心指标

| 层级 | 必报指标 | 可选增强指标 |
| --- | --- | --- |
| 轨迹学习 | 位置 RMSE/MAE，姿态误差，DTW 距离 | jerk，路径完成率，动态人体下 ICP RMSE |
| 力控学习 | `max |e|`，MAE，RMSE，标准差，超调量 | 频谱相似度，力-位移相位差，安全阈值触发次数 |
| 手法相似度 | Max/Min/Mean/Std/Skew/Kurt/Frequency | 与专家信号相关系数，周期稳定性，接触面积 |
| 强化学习 | 回报曲线，收敛迭代，真实交互次数 | 消融实验，跨人泛化，失败案例分析 |
| 用户评价 | 舒适度、疼痛 VAS、安全不良事件 | EMG、肌肉硬度、皮温、临床量表 |

### 推荐对比组

- 固定位置轨迹回放。
- 固定阻抗/导纳控制。
- PID 或传统阻抗力控。
- 示教学习模型：DMP/GMR。
- 学习补偿模型：残差 RL、DQN、PPO/SAC。
- 消融：无皮肤模型、无 LSTM、无 GMM/GMR 融合、无 ICM。

### 对按摩机器人论文写作最有价值的结果形式

- 每种手法给出专家和机器人力曲线叠图。
- 每种手法给出频谱图和主频误差。
- 每个受试者给出力误差箱线图。
- 给出不同皮肤刚度/不同体型的泛化表。
- 给出学习收敛曲线和真实交互次数。
- 给出安全统计：最大力、超过阈值次数、急停次数、不良事件。

## 参考文献和链接

1. Yang J. et al. 2024. *Robotics in Massage: A Systematic Review*. Health Services Research and Managerial Epidemiology. DOI: https://doi.org/10.1177/23333928241230948
2. Li C., Fahmy A., Li S., Sienz J. 2020. *An Enhanced Robot Massage System in Smart Homes Using Force Sensing and a Dynamic Movement Primitive*. Frontiers in Neurorobotics. DOI: https://doi.org/10.3389/fnbot.2020.00030
3. Khoramshahi M. et al. 2020. *Arm-hand motion-force coordination for physical interactions with non-flat surfaces using dynamical systems: Toward compliant robotic massage*. ICRA. DOI: https://doi.org/10.1109/ICRA40945.2020.9196593
4. Xiao M. et al. 2023. *Study on force control for robot massage with a model-based reinforcement learning algorithm*. Intelligent Service Robotics. DOI: https://doi.org/10.1007/s11370-023-00474-6
5. Xiao M., Zhang T., Zou Y., Chen S., Wu W. 2024. *Research on Robot Massage Force Control Based on Residual Reinforcement Learning*. IEEE Access. DOI: https://doi.org/10.1109/ACCESS.2023.3347416
6. Xiao M. et al. 2024. *A study on robot force control based on the GMM/GMR algorithm fusing different compensation strategies*. Frontiers in Neurorobotics. DOI: https://doi.org/10.3389/fnbot.2024.1290853
7. Li Z. et al. 2024. *Learning Variable Impedance Control for Robotic Massage With Deep Reinforcement Learning: A Novel Learning Framework*. IEEE Systems, Man, and Cybernetics Magazine. DOI: https://doi.org/10.1109/MSMC.2022.3231416
8. Wang F. et al. 2024. *Compliance control of a rehabilitation massage robot in dynamic scenes*. Journal of Physics: Conference Series. DOI: https://doi.org/10.1088/1742-6596/2816/1/012103
9. Zhang X. et al. 2024. *Autonomous path planning and stabilizing force interaction control for robotic massage in unknown environment*. Industrial Robot. DOI: https://doi.org/10.1108/IR-11-2023-0292
10. Xu Q. et al. 2024. *Toward automatic robotic massage based on interactive trajectory planning and control*. Complex & Intelligent Systems. DOI: https://doi.org/10.1007/s40747-024-01384-5
11. Xu Y., Huang K., Guo W., Du L. 2024. *Digital Modeling of Massage Techniques and Reproduction by Robotic Arms*. arXiv. DOI: https://doi.org/10.48550/arXiv.2412.05940
12. Chen W. et al. 2026. *Feeding, grooming, dressing, and body repositioning: categorizing four pillars of learning-based manipulation for robotic caregiving*. Artificial Intelligence Review. DOI: https://doi.org/10.1007/s10462-026-11524-7
