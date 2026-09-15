# FAST-LIO 固定先验地图定位系统

<p align="center">
  <a href="README.md">简体中文</a> |
  <a href="README_EN.md">English</a>
</p>

![License](https://img.shields.io/badge/license-GPL--3.0-blue)
![ROS2](https://img.shields.io/badge/ROS2-Humble-blue)
![C++](https://img.shields.io/badge/C%2B%2B-17-orange)

> **项目定位**：本项目是一个面向**已知先验地图**的 LiDAR 定位系统，设计为可在**仿真环境与公开数据集**上验证。
> **项目背景**：本课题为**机器人学课程设计**，在**深蓝学院「多传感器融合」课程**所学 LiDAR-惯性里程计与状态估计方法的基础上完成实现与扩展。
> **说明：本仓库当前未附带实验数据或评估指标结果**——[§8](#8-实验设计计划) 给出的是实验**设计（计划）**，而非已完成的实验结论。[§12](#12-推荐实机测试配置) 给出推荐的实机测试配置。

---

## 目录

1. [研究背景与问题定义](#1-研究背景与问题定义)
2. [方法论](#2-方法论)
3. [系统架构](#3-系统架构)
4. [与上游 FAST-LIO 的差异（ROS 2 版本）](#4-与上游-fast-lio-的差异ros-2-版本)
5. [状态估计原理](#5-状态估计原理)
6. [定位状态机](#6-定位状态机)
7. [鲁棒定位机制](#7-鲁棒定位机制)
8. [实验设计（计划）](#8-实验设计计划)
9. [快速开始](#9-快速开始)
10. [参数配置](#10-参数配置)
11. [话题接口](#11-话题接口)
12. [推荐实机测试配置](#12-推荐实机测试配置)
13. [复现步骤（计划）](#13-复现步骤计划)
14. [工程方法论沉淀](#14-工程方法论沉淀)
15. [目录结构](#15-目录结构)
16. [许可与致谢](#16-许可与致谢)

---

## 1. 研究背景与问题定义

### 1.1 问题背景

经典 LiDAR-惯性里程计（LIO）以**在线增量建图**为核心，面向未知环境探索。但在巡检、仓储、无人机定点作业等场景中，环境地图往往**事先已获得**，需求从"增量建图"转为**在已知地图中的稳定定位**。

直接复用 LIO 建图框架会引入两类问题：

1. **地图污染** —— 在线增删破坏先验地图一致性；
2. **失效无感知** —— 定位退化（特征缺失、遮挡、绑架）时系统仍持续输出错误位姿，且无回退机制。

### 1.2 问题定义

给定先验点云地图 $\mathcal{M}$ 与实时 LiDAR/IMU 观测序列，估计传感器位姿 $\mathbf{T}_k \in SE(3)$，并同时输出**定位置信状态** $s_k$，使得：

- 正常时位姿误差有界；
- 退化时系统能**检测失效**并**回退到最后一个可靠位姿**，而非输出发散结果。

形式化地，设 $\hat{\mathbf{T}}_k$ 为估计位姿，$\mathbf{T}_k^{*}$ 为真值，则期望满足：

$$
\left\| \log\!\left( \mathbf{T}_k^{*\,\top} \hat{\mathbf{T}}_k \right) \right\| \le \varepsilon \quad \text{当 } s_k \in \{\text{TRACKING}, \text{LOCKED}\}
$$

且在 $s_k = \text{LOST}$ 时输出冻结于最后可靠帧。

### 1.3 设计目标

| 编号 | 目标 | 对应机制 | 源码位置 |
| :-- | :-- | :-- | :-- |
| G1 | 固定地图定位，禁止在线增删 | 先验地图一次性构建 ikd-Tree | `init_localization_map_from_prior()` |
| G2 | 复用成熟估计主干 | scan-to-map + IESKF | `h_share_model()` / `process_one_frame()` |
| G3 | 失效可检测 | 匹配质量评估 + 四状态状态机 | `state_machine.hpp` |
| G4 | 失效可恢复 | 位姿门控 + 最后一帧可靠位姿回退 | `pose_gate.hpp` / `process_one_frame()` |
| G5 | 结果可复现 | 提供仿真/数据集配置与单元测试 | `test/test_state_machine.cpp`、`config/marsim.yaml` |

---

## 2. 方法论

### 2.1 核心思想：估计与可信度解耦

系统采用**分层解耦**设计：将"位姿怎么算"与"位姿可不可信"拆成两条独立链路。

| 层次 | 职责 | 是否可单测 | 关键文件 |
| :-- | :-- | :-- | :-- |
| 估计层 | 位姿/速度/零偏/外参/重力估计 | 依赖数据，不易单测 | `IMU_Processing.hpp`、`fastlio_node.cpp` |
| 鲁棒层 | 质量判定、状态迁移、门控回退 | **纯逻辑，可单测** | `localization/state_machine.hpp`、`localization/pose_gate.hpp` |

**为什么这样拆？** 鲁棒策略的调参会频繁变更，若与估计核心耦合，每次调参都要回归测试整个 EKF。拆开后，鲁棒层作为"纯函数式"逻辑（输入 `MatchQuality`，输出状态），可以脱离 ROS 与点云独立测试——这正是 `test_state_machine.cpp` 能覆盖状态转移边界的原因。

### 2.2 设计权衡

| 决策点 | 采用方案 | 备选 | 理由 |
| :-- | :-- | :-- | :-- |
| 先验地图结构 | ikd-Tree | 普通 kd-tree / 体素哈希 | 支持高效最近邻，库成熟 |
| 定位估计器 | 复用 IESKF 作主估计 | 用 NDT 替换 IESKF | 复用成熟主干；NDT 仅作**失位恢复后端**（见 §4.6），不替换估计器 |
| 失效处理 | 状态机 + 门控 | 仅阈值告警 | 提供可执行的回退动作 |
| 鲁棒层形态 | 纯逻辑头文件 | 并入主节点 | 可独立单测，便于调参 |
| 门控语义 | 与几何质量**解耦** | 混入同一评分 | 避免误判（详见 §6.3） |

### 2.3 关键工程加固

以下加固均来自代码实现，用于消除数值/并发/内存隐患。

| 类别 | 问题 | 处理 | 位置 |
| :-- | :-- | :-- | :-- |
| 数值安全 | `acos` 输入因浮点误差越界 | 钳制到 $[-1,1]$ | `so3_math.h` / `Exp_mat.h` |
| 数值安全 | 平面法向量零范数归一化 | 范数下界 `1e-12` 保护 | `common_lib.h::esti_normvector` |
| 数值安全 | 退化邻点角余弦误判 | 零范数直接 `continue` 跳过 | `preprocess.cpp` |
| 并发安全 | `std::vector<bool>` 位压缩竞争 | 改 `std::vector<uint8_t>` | `fastlio_node.cpp` |
| 内存安全 | PCL 容器 `clear()` 后下标写入 | 改为按输入尺寸 `resize` | `fastlio_node.cpp::h_share_model` |
| 时间语义 | ROS 2 时间戳分解不规范 | 规范化进位，`nanosec ∈ [0,1e9)` | `common_lib.h::sec_to_stamp` |
| 运行稳定 | 路径无界增长 | 最小距离抽稀 + 最大长度截断 | `publish_path()` |
| 运行稳定 | 每帧日志刷屏 | `RCLCPP_INFO_THROTTLE` | `preprocess.cpp` |

---

## 3. 系统架构

![系统总体架构](doc/figures/architecture.svg)

**图 1** 五层结构：输入层、预处理层、惯导处理层、估计核心层、定位鲁棒层。估计与鲁棒判定分离是整体设计的主线。

### 3.1 数据流与话题接口

![数据流与话题](doc/figures/topic_graph.svg)

**图 8** 展示节点与 ROS 话题的输入输出关系。输入为 LiDAR/IMU，输出包含里程计、轨迹、配准点云、状态可视化与先验地图（latched）。

---

## 4. 与上游 FAST-LIO 的差异（ROS 2 版本）

> 本节仅列**代码与结构层面可核实的事实**。**不包含量化性能对比**——本仓库未附实验数据（见 [§8](#8-实验设计计划)），因此不主张精度提升幅度。
> **本仓库是 ROS 2（Humble）版本**：上游 `hku-mars/FAST_LIO` 是 ROS 1 包，本项目为其 **ROS 2 移植 + 功能扩展**。

### 4.1 平台移植：ROS 1 → ROS 2

| 维度 | 上游 FAST-LIO | 本项目（ROS 2 版本） |
| :-- | :-- | :-- |
| 中间件 | ROS 1（`roscpp`，Melodic+） | **ROS 2 Humble（`rclcpp`）** |
| 构建系统 | `catkin_make` | **`ament_cmake` + `colcon`** |
| 节点结构 | 单文件 `laserMapping.cpp` + 全局变量 | **模块化 `FastLioNode` 类**（`src/fastlio_node.cpp`），无全局变量 |
| 参数机制 | ROS 1 参数服务器 | **ROS 2 `declare_parameter` + YAML**（`/**/ros__parameters`） |
| 雷达驱动 | `livox_ros_driver` | **`livox_ros_driver2`**（`CustomMsg`） |
| 状态可视化 | `jsk_rviz_plugins` OverlayText | 标准 **`visualization_msgs/Marker`**（rviz2 原生可用） |
| 时间类型 | `ros::Time` | `builtin_interfaces` + **规范化 `sec_to_stamp`** |
| launch | `*.launch`（XML） | **`*.launch.py`**（Python） |
| 消息生成 | `message_generation` | **`rosidl_default_generators`** |
| 单元测试 | 无 | **`test/test_state_machine.cpp`**（GTest，脱离 ROS） |

### 4.2 功能新增：固定先验地图重定位（上游无）

上游仅提供**在线增量建图**。本项目增加 `MODE_LOCALIZATION`（`run_mode: 1`）：

| 能力 | 上游 | 本项目 | 源码 |
| :-- | :--: | :--: | :-- |
| 先验 PCD 地图加载 | ✗ | ✅ | `load_prior_map_from_pcd()` |
| 先验地图 voxel 下采样 | ✗ | ✅ | `init_localization_map_from_prior()` |
| 一次性构建 ikd-Tree，禁止在线增删 | ✗ | ✅ | `init_localization_map_from_prior()` |
| 先验地图 latched 发布 | ✗ | ✅ | `/Laser_map`（`transient_local`） |
| 双运行模式（建图 / 定位） | ✗ | ✅ | `RunMode` / `run_mode` |

### 4.3 功能新增：定位鲁棒机制（上游无）

上游定位退化时**持续输出位姿**，无失效处理。本项目新增独立鲁棒层：

| 机制 | 上游 | 本项目 | 源码 |
| :-- | :--: | :--: | :-- |
| 匹配质量评估（点数 + 残差） | ✗ | ✅ | `MatchQuality` / `is_match_*` |
| 四状态状态机 | ✗ | ✅ | `TrackingStateMachine` |
| 位姿门控 + 状态/协方差回滚 | ✗ | ✅ | `PoseGate` |
| LOST 冻结 / 回退 | ✗ | ✅ | `process_one_frame()` |
| 可选 LOST 恢复（软恢复） | ✗ | ✅（默认关） | `lost_recovery_en` |
| 可选外部 NDT 重定位恢复 | ✗ | ✅（默认关） | `NdtRelocalizer` / `relocalization.*` |
| 可选自适应门控 | ✗ | ✅（默认关） | `adaptive_gating_en` |

> 新机制均**默认关闭或保持上游行为**，避免改变原语义（见 [§14.4](#144-默认向后兼容)）。

### 4.4 工程质量加固（相对上游的修复）

| 类别 | 上游问题 | 本项目处理 |
| :-- | :-- | :-- |
| 并发 | `std::vector<bool>` 位压缩在 OpenMP 下竞争 | 改 `std::vector<uint8_t>` |
| 内存 | PCL 容器 `clear()` 后下标写入（UB） | 按输入尺寸 `resize` |
| 数值 | `esti_normvector` 无点数量边界 / 零范数保护 | 加边界与范数下界 |
| 数值 | `acos` 输入越界未钳制 | 钳制到 `[-1,1]` |
| 数值 | IMU 异常小范数致加速度暴涨 | 加范数保护 |
| 运行 | 每帧日志刷屏 | `RCLCPP_INFO_THROTTLE` |
| 运行 | 轨迹无界增长 | 最小距离抽稀 + 最大长度截断 |
| 构建 | OpenMP `QUIET` 静默降级 | `REQUIRED` + 现代 CMake target |

### 4.5 继承自上游的能力（非本项目新增）

以下能力**沿用上游**，本项目未改动其核心：多雷达支持（Velodyne / Ouster / Livox / MARSIM）、ikd-Tree 增量地图、IKFoM 流形 Kalman、MARSIM 仿真配置、PCD 保存。这些是上游的贡献，本项目继承而非自创。

### 4.6 功能新增：NDT 重定位后端（上游无）

上游没有任何"失位后重新定位"的后端。本项目新增独立的 **NDT 重定位后端** `NdtRelocalizer`（`src/ndt_relocalizer.{hpp,cpp}`，基于 PCL 原生 `pcl::NormalDistributionsTransform`），提供两条恢复路径：

| 路径 | 触发 | 行为 | 涉及参数 |
| :-- | :-- | :-- | :-- |
| **A. LOST 自动恢复** | 处于 `TRACKING_LOST` 且恢复开关开启，按帧间隔触发 | 拼接近期扫描为**源子图**（按帧间位姿统一到最新帧 body 系）→ 从先验地图裁剪**局部目标** → NDT 对齐 → 接受门校验 → 注入 EKF 并回到 `TRACKING` | `relocalization.*` + `localization.lost_recovery_en` |
| **B. 启动位姿初值** | 提供 `relocalization.initial_pose` 时，启动首帧注入一次 | 把外部给的 `T_WI` 作为滤波器起始位姿（**启动阶段不跑 NDT**，粗对齐来自 assume-given 的初值） | `relocalization.initial_pose` |

**关键设计：**

- **坐标系统一**：源点云为 LiDAR 系；NDT 源子图各历史帧按帧间相对位姿统一到"最新帧 body 系"拼接（消除运动重影）；NDT 输出 `T_WL` 经外参换算为 `T_WI` 后写入 EKF。
- **与软恢复共预算**：外部 NDT 恢复与 `lost_recovery_en` 的软恢复**共用** `recovery_attempts_` / 冷却预算，且**尝试失败也计入一次**，避免无限重试；受 `lost_recovery_en` 总开关约束（关闭时 LOST 永久冻结）。
- **接受门**：`hasConverged()` + 有效 fitness（限定匹配距离）+ 相对初值的平移/旋转校正门限 + 源/目标点数下限；任一不满足即拒绝。
- **注入策略**：仅覆盖 `pos`/`rot`，清零 `vel`（LOST 期间不可信），保留 bias/外参/重力；协方差仅膨胀 pos/rot 子块。
- **默认关闭**：`relocalization.enable=false`，保持既有行为（见 [§14.4](#144-默认向后兼容)）。
- **不做**无初值的裸 NDT 全局搜索（启动全局重定位需外部初值约束）。

详见 [§6.5 恢复语义](#65-恢复语义)、[§10.5 relocalization 参数](#105-ndt-重定位后端relocalization)。

---

## 5. 状态估计原理

### 5.1 状态量定义

系统在流形上维护 8 块状态（对应 `state_ikfom`）：

![误差状态与流形](doc/figures/state_manifold.svg)

**图 7** 状态量与流形定义。误差状态共 **23 维**。

$$
\mathbf{x} = \left[\, {}^G\mathbf{p}_I,\; {}^G\mathbf{R}_I,\; {}^I\mathbf{R}_L,\; {}^I\mathbf{t}_L,\; {}^G\mathbf{v}_I,\; \mathbf{b}_\omega,\; \mathbf{b}_a,\; {}^G\mathbf{g} \,\right]
$$

| 块 | 记号 | 流形 | 维度 | 含义 |
| :-- | :-- | :-- | :-- | :-- |
| 位置 | ${}^G\mathbf{p}_I$ | $\mathbb{R}^3$ | 3 | IMU 在世界系位置 |
| 姿态 | ${}^G\mathbf{R}_I$ | $SO(3)$ | 3 | IMU→世界旋转 |
| 外参旋转 | ${}^I\mathbf{R}_L$ | $SO(3)$ | 3 | LiDAR→IMU 旋转 |
| 外参平移 | ${}^I\mathbf{t}_L$ | $\mathbb{R}^3$ | 3 | LiDAR→IMU 平移 |
| 速度 | ${}^G\mathbf{v}_I$ | $\mathbb{R}^3$ | 3 | IMU 世界系速度 |
| 陀螺零偏 | $\mathbf{b}_\omega$ | $\mathbb{R}^3$ | 3 | 角速度零偏 |
| 加计零偏 | $\mathbf{b}_a$ | $\mathbb{R}^3$ | 3 | 加速度零偏 |
| 重力 | ${}^G\mathbf{g}$ | $S^2$ | 2 | 重力方向（单位球） |
| **合计** | | | **23** | |

> 误差状态维度：$3+3+3+3+3+3+3+2 = 23$。过程噪声 12 维（`ng, na, nbg, nba`），对应 `esekf<state_ikfom, 12, input_ikfom>`。

### 5.2 坐标系与外参

![坐标系与外参](doc/figures/frames.svg)

**图 5** 坐标系定义。世界系 $W$ 由 `world_frame` 指定（定位模式默认 `map`，建图默认 `camera_init`）。点到世界系的变换：

$$
\mathbf{p}_W = {}^G\mathbf{R}_I \left( {}^I\mathbf{R}_L\, \mathbf{p}_L + {}^I\mathbf{t}_L \right) + {}^G\mathbf{p}_I
$$

其中 $\mathbf{p}_L$ 为 LiDAR 系下的点坐标。该式即 `h_share_model()` 中 `p_global = s.rot * (s.offset_R_L_I * p_body + s.offset_T_L_I) + s.pos` 的直接对应。

### 5.3 预测与更新

![IESKF 时序](doc/figures/ieskf_timeline.svg)

**图 4** 预测–更新时序。IMU 高频传播名义状态与协方差，LiDAR 帧到达时执行迭代更新。

**预测**（`get_f` / `df_dx` / `df_dw`）：

$$
\mathbf{P}_{k|k-1} = \mathbf{F}\,\mathbf{P}_{k-1}\,\mathbf{F}^\top + \mathbf{Q}, \qquad \mathbf{F} = \mathbf{I} + \mathbf{A}\,\Delta t
$$

过程噪声对角（`process_noise_cov`）为非零块：$\sigma_{ng}^2=\sigma_{na}^2=10^{-4}$，$\sigma_{nbg}^2=\sigma_{nba}^2=10^{-5}$。

**更新**（点到平面残差）：对每个有效特征点，以最近邻拟合的局部平面法向量 $\mathbf{u}_i$ 构造观测

$$
\mathbf{z}_i = \mathbf{u}_i^\top \left( {}^G\mathbf{R}_I ({}^I\mathbf{R}_L \mathbf{p}_i^{L} + {}^I\mathbf{t}_L) + {}^G\mathbf{p}_I \right) + d_i
$$

迭代修正（`update_iterated_dyn_share_modified`）：

$$
\delta\mathbf{x} = \left( \mathbf{H}^\top \mathbf{R}^{-1} \mathbf{H} + \mathbf{P}^{-1} \right)^{-1} \mathbf{H}^\top \mathbf{R}^{-1} \mathbf{r}
$$

| 常量 | 值 | 含义 | 源码 |
| :-- | :-- | :-- | :-- |
| `NUM_MATCH_POINTS` | 5 | 最近邻点数 | `common_lib.h` |
| `LASER_POINT_COV` | 0.001 | 观测噪声方差 | `fastlio_node.cpp` |
| `max_iteration` | 4（参数，默认） | 迭代上限 | `fastlio_node.cpp` |
| 平面内点阈值 | 0.1 | `esti_plane` 残差门限 | `fastlio_node.cpp` |
| 残差接受阈值 | `s_score > 0.9` | 点被选为有效特征 | `fastlio_node.cpp` |
| 近邻距离平方门限 | 5 | 超过则丢弃该点 | `fastlio_node.cpp` |

### 5.4 重力对齐

重力对齐是本项目相对上游的重要扩展（`IMU_Processing.hpp::IMU_init`）：

- 累计静止段 IMU 加速度均值 $\bar{\mathbf{a}}$；
- 构造把 $\bar{\mathbf{a}}$ 旋到 $\hat{\mathbf{Z}}$ 的对齐旋转
  $\mathbf{q}_{\text{align}} = \text{FromTwoVectors}(\bar{\mathbf{a}}, \hat{\mathbf{Z}})$；
- 在虚拟水平系下固定重力 ${}^G\mathbf{g} = (0,0,-g)$，滤波器姿态从单位阵起步。

| 模式 | 重力初始化 | 效果 |
| :-- | :-- | :-- |
| `gravity_align_en: true` | $\mathbf{g}=(0,0,-g)$ + 水平对齐 | 倾斜安装也可直接输出正平面点云 |
| `gravity_align_en: false` | $\mathbf{g}=-g\,\bar{\mathbf{a}}/\|\bar{\mathbf{a}}\|$ | 保留上游原始行为 |

---

## 6. 定位状态机

![定位状态机](doc/figures/state_machine.svg)

**图 2** 四状态状态机（实现位于 `localization/state_machine.hpp`）。

### 6.1 状态定义

| 状态 | 枚举 | 含义 | 输出行为 |
| :-- | :-- | :-- | :-- |
| UNLOCKED | `TRACKING_UNLOCKED` | 初始化，未建立稳定跟踪 | 正常输出，标记不可信 |
| TRACKING | `TRACKING_TRACKING` | 可用但不稳定 | 正常输出 |
| LOCKED | `TRACKING_LOCKED` | 稳定可靠 | 正常输出，启用位姿门控 |
| LOST | `TRACKING_LOST` | 检测到失效 | 冻结于最后可靠位姿 |

### 6.2 质量判定（纯函数）

匹配质量由有效特征点数与平均残差判定，**只看几何对应质量**：

$$
\text{acceptable} := \text{corr} \wedge (n_{\text{eff}} \ge n_{\text{trk}}) \wedge (\bar{r} \le r_{\text{trk}})
$$

$$
\text{good} := \text{corr} \wedge (n_{\text{eff}} \ge n_{\text{good}}) \wedge (\bar{r} \le r_{\text{good}})
$$

| 参数 | 默认 | 作用 |
| :-- | :-- | :-- |
| `min_effective_points_for_tracking` | 15 | acceptable 最小有效点数 |
| `max_residual_for_tracking` | 0.40 | acceptable 最大平均残差 |
| `min_effective_points_for_good` | 30 | good 最小有效点数 |
| `max_residual_for_good` | 0.20 | good 最大平均残差 |

> **配置自洽约束**（构造函数中强制）：`min_points_good ≥ min_points_tracking`，`max_residual_good ≤ max_residual_tracking`。这保证 good 是 acceptable 的子集，避免状态定义出现矛盾。

### 6.3 状态转移

| 起始 | 目标 | 触发条件（源码） |
| :-- | :-- | :-- |
| UNLOCKED | TRACKING | `match_acceptable && acceptable_streak ≥ unlock_to_tracking_streak` |
| TRACKING | LOCKED | `elapsed ≥ min_time_before_lock_sec && match_good && good_streak ≥ good_match_streak_to_lock` |
| TRACKING | LOST | `!match_acceptable && bad_streak ≥ bad_match_streak_to_lost` |
| LOCKED | TRACKING | `!match_good`（质量回落，立即回退，并清零 good streak） |
| LOST | TRACKING | 恢复开启且满足恢复条件（见 §5.5） |

> 注意：**LOST 只能从 TRACKING 进入**。LOCKED 状态一旦 `!match_good` 先回到 TRACKING，再由后续帧在 TRACKING 中累积 `bad_streak` 才会进入 LOST。因此 `LOCKED → LOST` 不是单帧直接转移。

### 6.4 转移矩阵

| 当前 \ 下一 | UNLOCKED | TRACKING | LOCKED | LOST |
| :--: | :--: | :--: | :--: | :--: |
| **UNLOCKED** | ○ | ✓ | – | – |
| **TRACKING** | – | ○ | ✓ | ✓ |
| **LOCKED** | – | ✓ | ○ | – |
| **LOST** | – | ✓\* | – | ○ |

> ✓ 允许；○ 自环（维持）；– 不允许；✓\* 仅在 `lost_recovery_en: true` 时允许。
> LOCKED 一列中不存在到 LOST 的直接转移（见 §5.3 说明）。

### 6.5 恢复语义

LOST → TRACKING 的恢复需**同时**满足（`state_machine.hpp`）：

1. `lost_recovery_en == true`；
2. `match_acceptable && acceptable_streak ≥ lost_recovery_streak`；
3. `recovery_attempts < lost_recovery_max_attempts`；
4. 不在冷却期内（`now ≥ recovery_cooldown_until`）。

恢复成功时：`recovery_attempts++`，设置冷却终点 `now + cooldown`，**重置** `acceptable_streak` 与 `bad_streak`，置 `just_recovered_flag_`，进入 `on_enter_tracking()`（`has_last_tracking = true`，`good_streak = 0`）。

| 参数 | 默认 | 说明 |
| :-- | :-- | :-- |
| `lost_recovery_en` | false | 恢复总开关（默认关闭以保持原行为） |
| `lost_recovery_streak` | 10 | 触发恢复所需连续可接受帧数 |
| `lost_recovery_max_attempts` | 3 | 最大恢复尝试次数 |
| `lost_recovery_cooldown_sec` | 5.0 | 两次恢复的最短间隔 |

> **外部 NDT 恢复（第二条恢复路径）**：除上述"软恢复"外，LOST 状态下还可由 **NDT 重定位后端**恢复（见 [§4.6](#46-功能新增ndt-重定位后端上游无)）。它**共用**上述 `recovery_attempts_` / `lost_recovery_max_attempts` / `lost_recovery_cooldown_sec` 预算与冷却，同样受 `lost_recovery_en` 约束；不同点是：软恢复靠"连续可接受帧"，外部恢复靠"NDT 对齐成功 + 接受门通过"，成功后直接把重定位位姿注入 EKF。

---

## 7. 鲁棒定位机制

### 7.1 机制总览

![鲁棒决策流程](doc/figures/robust_decision.svg)

**图 6** 单帧鲁棒决策：EKF 更新后先过门控，再做质量评估与状态机更新，必要时回滚或冻结。

### 7.2 门控与回滚

位姿门控 `PoseGate` 仅在 `LOCKED` 状态生效；其余状态直接放行（与原版一致）。

$$
\text{allow} \iff \left\| \Delta\mathbf{p} \right\| \le \tau, \qquad \Delta\mathbf{p} = \mathbf{p}_{\text{after}} - \mathbf{p}_{\text{before}}
$$

自适应门控（`adaptive_gating_en: true`）按残差比率放宽阈值：

$$
\tau_{\text{eff}} = \tau \cdot \left( 1 + \min\!\left( \frac{\bar{r}}{r_{\text{good}}},\; s_{\max} - 1 \right) \right)
$$

| 参数 | 默认 | 说明 |
| :-- | :-- | :-- |
| `max_position_jump_for_update_locked` | 1.0 | 基础跳变阈值（米） |
| `adaptive_gating_en` | false | 自适应门控开关 |
| `adaptive_scale_max` | 3.0 | 最大放大倍数 |

### 7.3 语义解耦（关键设计）

`MatchQuality` 同时携带两个**互相独立**的事实：

| 字段 | 含义 | 来源 |
| :-- | :-- | :-- |
| `correspondence_valid` | 几何对应是否有效 | `effct_feat_num ≥ 1` |
| `gate_rejected` | 门控是否拒绝了本次更新 | `PoseGate::allow` 取反 |
| `effct_feat_num` | 有效特征点数 | `h_share_model` |
| `res_mean_last` | 平均残差 | `total_residual / effct_feat_num` |

**为什么必须解耦？** 若把门控拒绝混入质量判定，则"几何匹配良好但位姿跳变大"的帧会被错误地判为几何劣化，进而 `bad_streak` 累加、误触 LOST。项目将二者分离，使状态机只依据几何质量迁移，门控只影响是否提交该帧位姿。

### 7.4 回滚对象

| 触发 | 回滚目标 | 源码 |
| :-- | :-- | :-- |
| 门控拒绝 | 本帧更新前 `state_before_update` + `P_before_update` | `process_one_frame()` |
| 进入 LOST（首帧） | `last_locked_state_` + `last_locked_covariance_` | `process_one_frame()` |
| 已在 LOST | `frame_entry_state` / `last_locked_state_` | `process_one_frame()` 早退分支 |

---

## 8. 实验设计（计划）

> **重要说明**：本节描述的是**计划采用的评价指标与实验方法**，**本仓库当前未附带任何实验数据或评估结果**。下文的"预期"均为**待验证的假设**，不代表已经取得的结论。

### 8.1 计划采用的评价指标

| 指标 | 缩写 | 定义 | 用途 |
| :-- | :-- | :-- | :-- |
| 绝对轨迹误差 | ATE | 估计轨迹与真值逐点误差的 RMSE | 全局精度 |
| 相对位姿误差 | RPE | 固定间隔相对位姿误差 | 局部漂移 |
| 定位有效率 | – | LOCKED 帧占比 | 稳定性 |
| 失效检测延迟 | – | 真实失效到进入 LOST 的帧数 | 响应速度 |
| 恢复成功率 | – | LOST 后成功回到 TRACKING 的比例 | 恢复能力 |

### 8.2 计划使用的数据来源

| 类型 | 说明 | 用途 |
| :-- | :-- | :-- |
| 仿真环境 | 提供带真值的 LiDAR/IMU 数据流（`config/marsim.yaml`，`lidar_type: 4`） | 精度与失效注入实验 |
| 公开数据集 | 结构化场景序列 | 泛化性验证 |

> 上面两个数据来源是**可选的实验路径**；仓库中仅包含对应的**参数配置**（如 `config/marsim.yaml`），**不包含数据本体与结果**。

### 8.3 实验设计（计划）

| 实验 | 目的 | 变量 | 观测量 |
| :-- | :-- | :-- | :-- |
| E1 精度 | 定位精度 | 先验地图开/关、门控开/关 | ATE / RPE |
| E2 失效注入 | 检测与冻结 | 遮挡 / 绑架 / 长走廊退化 | 检测延迟、冻结正确性 |
| E3 消融 | 各机制贡献 | 门控 off / 状态机 off | ATE、有效率 |
| E4 恢复 | 恢复能力 | `lost_recovery_en` on/off | 恢复成功率、尝试次数 |

### 8.4 消融对照（待验证假设）

| 配置 | 状态机 | 门控 | 恢复 | 待验证的预期（非结论） |
| :-- | :--: | :--: | :--: | :-- |
| 基线 | ✓ | ✓ | ✗ | 预期稳定但失效后不恢复 |
| 无门控 | ✓ | ✗ | ✗ | 预期大跳变被接受，ATE 恶化 |
| 无状态机 | ✗ | ✓ | ✗ | 预期无 LOST 感知，失效时继续输出 |
| 全功能 | ✓ | ✓ | ✓ | 预期可检测且可恢复 |

---

## 9. 快速开始

![单帧数据处理流程](doc/figures/pipeline.svg)

**图 3** 单帧数据从到达到输出的完整流程。

### 9.1 依赖

| 组件 | 版本 / 说明 |
| :-- | :-- |
| 操作系统 | Ubuntu 22.04 |
| 中间件 | ROS 2 Humble |
| 编译 | `ament_cmake` + C++17 |
| 点云库 | PCL（`common` / `io` / `filters`） |
| 线性代数 | Eigen3 |
| 雷达驱动 | `livox_ros_driver2`（Livox `CustomMsg`） |
| 并行 | OpenMP |

### 9.2 编译

```bash
cd ~/ros2_ws/src
git clone <this-repo>
cd ..
rosdep install --from-paths src --ignore-src -y
colcon build --packages-select fast_lio
source install/setup.bash
```

### 9.3 运行（仿真 / 数据集）

```bash
# 建图模式：构建先验地图，pcd_save_en: true 时保存 PCD
ros2 launch fast_lio mapping_mid360.launch.py

# 定位模式：加载先验地图重定位
# 地图路径可写入 yaml，也可通过 launch 参数覆盖：
ros2 launch fast_lio relocalization_mid360.launch.py map_file_path:=/abs/path/map.pcd
```

### 9.4 单元测试

```bash
colcon test --packages-select fast_lio
colcon test-result --verbose
```

覆盖状态机状态转移（含可选 LOST 恢复边界）、匹配质量阈值与位姿门控边界。

---

## 10. 参数配置

### 10.1 运行模式

| 参数 | 类型 | 默认 | 说明 |
| :-- | :-- | :-- | :-- |
| `run_mode` | int | 0 | 0 建图 / 1 定位 |
| `map_file_path` | string | `""` | 先验 PCD 地图路径（定位模式必填） |
| `map_voxel_size` | double | 0.5 | 先验地图下采样体素（下界 1e-3） |
| `world_frame` | string | 依模式 | 世界系 frame id（定位 `map` / 建图 `camera_init`） |

### 10.2 定位状态机与门控

| 参数 | 默认 | 说明 |
| :-- | :-- | :-- |
| `localization.min_effective_points_for_tracking` | 15 | 可接受最小有效点数 |
| `localization.max_residual_for_tracking` | 0.40 | 可接受最大平均残差 |
| `localization.min_effective_points_for_good` | 30 | 良好最小有效点数 |
| `localization.max_residual_for_good` | 0.20 | 良好最大平均残差 |
| `localization.unlock_to_tracking_streak` | 3 | 进入 TRACKING 连续帧数 |
| `localization.good_match_streak_to_lock` | 5 | 进入 LOCKED 连续帧数 |
| `localization.bad_match_streak_to_lost` | 5 | 进入 LOST 连续劣化帧数 |
| `localization.min_time_before_lock_sec` | 2.0 | 锁定前最短时间 |
| `localization.max_position_jump_for_update_locked` | 1.0 | LOCKED 最大位置跳变 |
| `localization.adaptive_gating_en` | false | 自适应门控 |
| `localization.lost_recovery_en` | false | LOST 自动恢复 |
| `localization.lost_recovery_streak` | 10 | 恢复所需连续可接受帧数 |
| `localization.lost_recovery_max_attempts` | 3 | 最大恢复尝试次数 |
| `localization.lost_recovery_cooldown_sec` | 5.0 | 恢复冷却时间 |
| `localization.prior_map_pub_interval` | 50 | 先验地图周期重发间隔 |

### 10.5 NDT 重定位后端（relocalization）

见 [§4.6](#46-功能新增ndt-重定位后端上游无)。**默认全部关闭**。

| 参数 | 默认 | 说明 |
| :-- | :-- | :-- |
| `relocalization.enable` | false | NDT 重定位总开关（需配合 `localization.lost_recovery_en`） |
| `relocalization.method` | `"ndt"` | 方法名（当前仅 NDT） |
| `relocalization.trigger_interval_frames` | 5 | LOST 下每 N 帧尝试一次 |
| `relocalization.local_submap_frames` | 5 | 源子图帧数（应使点数 ≥ `min_source_points`） |
| `relocalization.local_map_radius` | 30.0 | 局部目标地图裁剪半径（米） |
| `relocalization.cov_inflation` | 10.0 | 注入后 pos/rot 协方差膨胀因子 |
| `relocalization.source_voxel_size` | 0.5 | 源点云下采样体素 |
| `relocalization.target_voxel_size` | 0.5 | 目标点云下采样体素 |
| `relocalization.ndt_resolution` | 1.0 | NDT 体素分辨率（米） |
| `relocalization.ndt_step_size` | 0.1 | NDT 线搜索步长 |
| `relocalization.ndt_trans_eps` | 0.01 | NDT 收敛阈值 |
| `relocalization.ndt_max_iter` | 30 | NDT 最大迭代次数 |
| `relocalization.ndt_num_threads` | 0 | NDT 线程数（0 = auto，PCL OpenMP） |
| `relocalization.max_translation_delta` | 8.0 | 接受门：相对初值的平移校正上限（米） |
| `relocalization.max_rotation_delta_deg` | 20.0 | 接受门：相对初值的旋转校正上限（度） |
| `relocalization.max_fitness_score` | 1.0 | 接受门：适应度上限 |
| `relocalization.min_source_points` | 200 | 接受门：源点云最小点数 |
| `relocalization.min_target_points` | 1000 | 接受门：目标点云最小点数 |
| `relocalization.initial_pose` | `[]` | 启动位姿初值 `[x,y,z,roll,pitch,yaw]`（米/度，语义 `T_WI`）；留空则启动不注入 |

### 10.6 预处理与估计

| 参数 | 默认 | 说明 |
| :-- | :-- | :-- |
| `preprocess.lidar_type` | 1 | 1 Livox / 2 Velodyne / 3 Ouster / 4 MARSIM |
| `preprocess.scan_line` | 6 | 线数（钳制 1–128） |
| `preprocess.blind` | 0.01 | 盲区半径 |
| `preprocess.timestamp_unit` | 2 | 时间戳单位（0 Sec / 1 ms / 2 us / 3 ns） |
| `preprocess.scan_rate` | 10 | 扫描频率（下界 1） |
| `point_filter_num` | 1 | 点抽稀间隔（下界 1） |
| `feature_extract_enable` | false | 特征提取开关 |
| `max_iteration` | 4 | IESKF 最大迭代次数 |
| `filter_size_corner` | 0.5 | 角点滤波尺寸（下界 1e-3） |
| `filter_size_surf` | 0.5 | 面点滤波尺寸（下界 1e-3） |
| `filter_size_map` | 0.5 | 地图体素尺寸（下界 1e-3） |
| `cube_side_length` | 200.0 | 局部地图边长（下界 1e-3） |
| `mapping.gravity_align_en` | true | 重力对齐开关 |
| `mapping.det_range` | 300.0 | 有效探测距离 |
| `mapping.fov_degree` | 180.0 | 视场角 |
| `mapping.extrinsic_est_en` | true | 外参在线估计（代码默认；示例 yaml 中设为 false） |
| `mapping.gyr_cov` / `acc_cov` | 0.1 | 陀螺/加计噪声 |
| `mapping.b_gyr_cov` / `b_acc_cov` | 0.0001 | 零偏噪声 |

### 10.7 发布与保存

| 参数 | 默认 | 说明 |
| :-- | :-- | :-- |
| `publish.path_en` | true | 发布轨迹 |
| `publish.path_min_distance` | 0.05 | 轨迹最小抽稀距离 |
| `publish.path_max_length` | 10000 | 轨迹最大长度 |
| `publish.scan_publish_en` | true | 发布配准点云 |
| `publish.dense_publish_en` | true | 发布稠密点云 |
| `publish.scan_bodyframe_pub_en` | true | 发布 IMU 体系点云 |
| `pcd_save.pcd_save_en` | false | 保存 PCD |
| `pcd_save.interval` | -1 | 单文件帧数，-1 为全部 |

---

## 11. 话题接口

| 方向 | 话题 | 类型 | QoS / 队列 |
| :-- | :-- | :-- | :-- |
| 订阅 | `/livox/lidar` | `livox_ros_driver2/CustomMsg`（或 `PointCloud2`） | 队列 200000 |
| 订阅 | `/livox/imu` | `sensor_msgs/Imu` | 队列 200000 |
| 发布 | `/cloud_registered` | `sensor_msgs/PointCloud2` | 队列 100000 |
| 发布 | `/cloud_registered_body` | `sensor_msgs/PointCloud2` | 队列 100000 |
| 发布 | `/Laser_map` | `sensor_msgs/PointCloud2` | `QoS(1).transient_local()` |
| 发布 | `/Odometry` | `nav_msgs/Odometry` | 队列 100000 |
| 发布 | `/path` | `nav_msgs/Path` | 队列 100000 |
| 发布 | `/localization_status_marker` | `visualization_msgs/Marker` | 队列 10 |

话题名由 `common.lid_topic` / `common.imu_topic` 参数配置，默认 `/livox/lidar`、`/livox/imu`。

---

## 12. 推荐实机测试配置

> 本节为**有条件时的建议配置**，非本项目结论来源。仿真与数据集是**推荐的验证路径**（本项目尚未附上相应结果）。

| 项目 | 推荐 | 说明 |
| :-- | :-- | :-- |
| 雷达 | Livox Mid-360 / AVIA | 非重复扫描，适合 LIO |
| IMU | 内置或独立 200 Hz+ 工业级 IMU | 与雷达刚性固连 |
| 算力 | 6 核以上 x86 CPU | 计算量集中在最近邻与滤波 |
| 安装 | 雷达斜装亦可 | 依赖重力对齐 |
| 时间同步 | 硬同步优先（PPS/PTP） | 无硬同步时用 `time_sync_en` |
| 外参标定 | 建议 LI-Init 等工具标定 | 写入 `extrinsic_T` / `extrinsic_R` |
| 初始位姿 | 起始点靠近地图原点，yaw 偏移要小 | 先验地图需与建图坐标系一致 |
| 初始静止 | 上电后保持静止约 1–2 秒 | 供 IMU 初始化与重力对齐 |

**跑通检查清单**

- [ ] `ros2 topic hz /livox/lidar` 与 `/livox/imu` 频率正常
- [ ] `map_file_path` 指向有效 PCD 且坐标系与建图一致
- [ ] 静止时 `/Odometry` 漂移在可接受范围
- [ ] `/localization_status_marker` 能进入 `LOCKED`
- [ ] 遮挡 / 绑架时能进入 `LOST` 并冻结

---

## 13. 复现步骤（计划）

1. **环境**：按 §9.1 安装 Ubuntu 22.04 + ROS 2 Humble + PCL + Eigen + `livox_ros_driver2`。
2. **编译**：执行 §9.2 命令，确保 `colcon build` 成功。
3. **建图**：以建图模式运行，采集/回放仿真或数据集序列，`pcd_save_en: true` 导出先验地图。
4. **定位**：切换 `run_mode: 1`，设置 `map_file_path`，运行定位模式。
5. **指标**：记录 `/Odometry` 与 `/path`，按 §8.1 计算 ATE/RPE。
6. **失效实验**：按 §8.3 注入失效，观察状态机与冻结行为。
7. **单元测试**：执行 §9.4，确认状态机与门控逻辑通过。

---

## 14. 工程方法论沉淀

本节记录开发过程中的方法论，供后续维护与扩展参考。

### 14.1 分层解耦优先

鲁棒策略（状态机、门控）被抽为**纯逻辑头文件**，不依赖 ROS 与点云。收益：

- 可脱离传感器数据做单元测试；
- 调参不影响估计核心；
- 状态迁移边界可穷举验证。

### 14.2 语义单一职责

一次"匹配"被拆成**几何质量**与**门控结果**两个正交事实，避免一个字段承载两种含义导致的误判（§6.3）。

### 14.3 边界与退化先行

数值代码优先处理边界：零范数、越界索引、`acos` 输入域、除零。加固集中在 `common_lib.h`、`so3_math.h`、`preprocess.cpp`，并以"失败即返回"替代"静默产生 NaN"。

### 14.4 默认向后兼容

新增能力（`lost_recovery_en`、`adaptive_gating_en`）默认**关闭**，保证不改变既有行为；需要时显式开启。降低回归风险。

### 14.5 配置自洽校验

状态机构造函数强制参数间约束（`good ⊂ acceptable`），把"非法组合"挡在运行之前，而非在运行中出现矛盾行为。

### 14.6 时间语义规范化

ROS 2 时间戳必须满足 `nanosec ∈ [0, 1e9)`。`sec_to_stamp` 用 `floor + llround + 进位归一化`，并正确处理负时间。

### 14.7 运行期可观测与可控

- 日志节流（`RCLCPP_INFO_THROTTLE`）避免刷屏；
- 路径抽稀 + 长度上限控制内存；
- 先验地图 latched 发布保证后加入的订阅者可见。

### 14.8 可复现性

仿真/数据集配置 + 单元测试使方法验证不依赖特定硬件。

---

## 15. 目录结构

```
fast_lio/
├── config/                  # ROS 2 参数配置（mid360 / 仿真 / 多雷达）
├── doc/
│   └── figures/             # 本文档示意图（SVG）
├── include/
│   ├── ikd-Tree/            # 增量 kd-tree
│   ├── IKFoM_toolkit/       # 流形 Kalman 工具
│   └── common_lib.h, so3_math.h, use-ikfom.hpp, ...
├── launch/                  # 建图 / 定位 launch
├── msg/                     # 自定义消息（Pose6D）
├── rviz_cfg/                # RViz 配置
├── src/
│   ├── localization/        # 状态机与门控（纯逻辑，可单测）
│   ├── fastlio_node.cpp     # 主节点
│   ├── ndt_relocalizer.*    # NDT 重定位后端（LOST 恢复 / 启动位姿初值）
│   ├── preprocess.*         # 点云预处理
│   └── IMU_Processing.hpp   # 惯导处理与去畸变
└── test/                    # 单元测试
```

| 目录 | 职责 | 关键文件 |
| :-- | :-- | :-- |
| `src/localization/` | 鲁棒层纯逻辑 | `state_machine.hpp`、`pose_gate.hpp` |
| `src/` | 估计与主流程 | `fastlio_node.cpp`、`IMU_Processing.hpp`、`preprocess.cpp` |
| `src/ndt_relocalizer.*` | NDT 重定位后端 | `ndt_relocalizer.hpp`、`ndt_relocalizer.cpp` |
| `include/ikd-Tree/` | 先验地图索引 | `ikd_Tree.cpp` |
| `config/` | 运行时参数 | `mid360_mapping_ros2.yaml`、`mid360_relocalization_ros2.yaml`、`marsim.yaml` |

---

## 16. 许可与致谢

### 16.1 许可

本项目以 **GPL-3.0** 许可发布，详见 [LICENSE](LICENSE)。

### 16.2 致谢

本项目的算法主干基于 **FAST-LIO / FAST-LIO2**（作者：Wei Xu、Fu Zhang 等，香港大学 HKU-MARS 实验室）。其版权归原作者所有：

- 上游仓库：<https://github.com/hku-mars/FAST_LIO>
- 上游论文：W. Xu and F. Zhang, *FAST-LIO: A Fast, Robust LiDAR-inertial Odometry Package by Tightly-Coupled Iterated Kalman Filter*, IEEE RA-L, 2021.

本项目在上游基础上进行了模块化重构，并扩展了固定地图定位、定位状态机与鲁棒门控机制。开发过程中参考并受益于开源社区的 LiDAR-惯性里程计工作，谨向上游作者及相关开源项目致谢。

本课题为**机器人学课程设计**，其方法学习得益于**深蓝学院「多传感器融合」课程**，谨此致谢。

> 关于上游许可：上游仓库的 `package.xml` 标注为 BSD，但其 `LICENSE` 文件为 GPL-2.0，两者存在不一致，使用前请以上游实际授权文件为准。本项目在此保留上游署名与来源。

---

## 商业使用

本项目仅限用于学术研究或非商业行为。商业使用需联系作者授权。
