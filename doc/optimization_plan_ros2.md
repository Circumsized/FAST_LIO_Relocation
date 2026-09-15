# FAST\_LIO\_Relocation 系统性优化与 ROS2 迁移计划

> 基准代码：ROS 1 版 FAST-LIO2 代码库（main 分支）
> 本地工作目录：`d:\重定位`（存放源码）
> 已确认决策：本地克隆优化 | 四维度全覆盖（质量/架构/鲁棒/性能） | 验收=可编译+静态检查为主 | **ROS2 Humble + Ubuntu 22.04 + livox\_ros\_driver2 功能等价移植**

***

## 1. 摘要 (Summary)

FAST\_LIO\_Relocation 是 FAST-LIO2 的 fork，把增量建图改造为**基于先验 PCD 地图的固定地图定位**，
并加入了 `UNLOCKED→TRACKING→LOCKED→LOST` 鲁棒定位状态机、匹配质量评估、位姿门控与 LOST 回滚。

本计划基于对仓库全部核心源码（`laserMap.cpp` 全文 64.6KB、`IMU_Processing.hpp`、`preprocess.cpp/h`、
`use-ikfom.hpp`、`common_lib.h`、`CMakeLists.txt`、`config/*.yaml`）的逐行阅读，以及 FAST-LIO2 论文、
ikd-Tree 论文、IESKF、先验地图定位/DOP 置信度/Scan Context 重定位等专业资料的交叉验证，进行：

1. **ROS2 迁移**：ROS1 Noetic/catkin/C++14 → ROS2 Humble/ament\_cmake/C++17，功能等价、可编译运行；
2. **模块化重构**：拆分巨型单文件 `laserMap.cpp` 为职责清晰的模块；
3. **代码现代化**：消除全局变量/魔法数/硬编码/裸指针，接入 clang-format/clang-tidy；
4. **鲁棒性增强**：状态机完备化（LOST 恢复边）、退化检测、自适应门控（全部开关化、默认保持原行为）；
5. **性能优化**：去固定大数组、去冗余拷贝、热点降载、固定地图下 ikd-tree 无效操作剔除。

验收基线：`colcon build` 通过 + `clang-tidy` 高优先级清零 + 状态机/匹配评估 gtest 通过。

***

## 2. 当前状态分析（基于源码实证）

### 2.1 仓库结构（实测，commit 3d5062b）

| 文件                              | 规模           | 职责                                                                                                                                      |
| ------------------------------- | ------------ | --------------------------------------------------------------------------------------------------------------------------------------- |
| `src/laserMap.cpp`              | 64,605 B，单文件 | ROS 回调、数据同步、IESKF 主循环、状态机、先验地图、发布、日志、main —— **全部逻辑**                                                                                   |
| `src/IMU_Processing.hpp`        | \~15 KB      | `ImuProcess` 类：IMU 初始化、前向传播、`UndistortPcl` 反向传播去畸变、虚拟水平重力对齐（`q_align_`）                                                                 |
| `src/preprocess.cpp/h`          | \~25 KB      | avia/velodyne/ouster64/marsim 四种点云接入 + 可选特征提取 `give_feature`                                                                            |
| `include/use-ikfom.hpp`         | \~4 KB       | IESKF 流形状态 `state_ikfom`（pos/rot/offset\_R\_L\_I/offset\_T\_L\_I/vel/bg/ba/grav(S2)，dof 23）、`get_f`/`df_dx`/`df_dw`、`process_noise_cov` |
| `include/common_lib.h`          | \~8 KB       | `MeasureGroup`、`StatesGroup`（18 维，FAST-LIO1 遗留）、`esti_plane`/`esti_normvector`、宏集                                                       |
| `include/ikd-Tree/`             | vendored     | 增量 k-d 树（已去 submodule）                                                                                                                  |
| `include/IKFoM_toolkit/esekfom` | vendored     | 迭代误差状态卡尔曼滤波                                                                                                                             |
| `config/*.yaml`                 | 7 个          | `run_mode`、`map_file_path`、`map_voxel_size`、`localization/*` 阈值、`mapping/*` 噪声外参                                                        |
| `msg/Pose6D.msg`                | 1 个          | IMU 传播位姿记录消息                                                                                                                            |
| `CMakeLists.txt`                | 2.2 KB       | catkin，硬编码 Debug、`-std=c++14` 重复 5 次、链接 PythonLibs + matplotlib                                                                         |

### 2.2 核心机制（逐行实证）

**（a）双模式与先验地图**（`laserMap.cpp`）

- `enum RunMode { MODE_MAPPING=0, MODE_LOCALIZATION=1 }`，参数 `run_mode`；非法值回退 MAPPING。

- 定位模式启动时 `init_localization_map_from_prior()`：`pcl::io::loadPCDFile` → `pcl::VoxelGrid`(leaf=`map_voxel_size`) → `ikdtree.set_downsample_param` + `ikdtree.Build(prior_map_ds->points)` 一次性建树。

- `allow_map_update()`/`lasermap_fov_segment()`/`map_incremental()` 仅在建图模式执行；定位模式禁用地图增删。

- 先验地图通过 latched topic `/Laser_map`（queue=1, latch=true）发布，`prior_map_pub_interval` 周期重发。

**（b）状态机**（`update_tracking_state_machine()`）

- 四态枚举 `LocalizationTrackingState { TRACKING_UNLOCKED, TRACKING_TRACKING, TRACKING_LOCKED, TRACKING_LOST }`。

- 转移（实证）：

  - UNLOCKED --(acceptable 且 streak≥3)→ TRACKING（**注意：阈值 3 为硬编码，未走参数**）

  - TRACKING --(time\_since\_start≥`min_time_before_lock_sec` 且 good 且 `good_match_streak`≥`good_match_streak_to_lock`)→ LOCKED（保存 `last_locked_state`）

  - TRACKING --(!acceptable 且 `bad_match_streak`≥`bad_match_streak_to_lost`)→ LOST

  - LOCKED --(!good)→ 降级 TRACKING（`good_match_streak=0`）

  - **LOST 无任何出边**（`return;`），进入后只能通过主循环分支回滚冻结，**无法自动恢复**——这是鲁棒性最大缺口。

- `last_locked_state`/`last_tracking_state` 为 `state_ikfom` 快照，LOST 时每帧 `kf.change_x(last_locked_state)` 冻结输出。

**（c）匹配质量评估**（`is_match_acceptable()`/`is_match_good()`）

- 指标：`effct_feat_num`（`h_share_model` 中满足平面拟合+s>0.9 的有效点数）、`res_mean_last = total_residual/effct_feat_num`、`accept_lidar_update`。

- acceptable 阈值：`min_effective_points_for_tracking`(15)、`max_residual_for_tracking`(0.40)；
  good 阈值：`min_effective_points_for_good`(30)、`max_residual_for_good`(0.20) —— 均参数化（yaml `localization/*`）。

**（d）位姿门控**（`check_pose_update_reasonable()`）

- 仅 LOCKED 生效：`delta_pos = (state_after.pos - state_before.pos).norm()` > `max_position_jump_for_update_locked`(默认 1.0m，yaml 0.5) 时拒绝整帧更新，`kf.change_x/change_P` 回滚，`accept_lidar_update=false`。**固定阈值，无自适应**。

**（e）IESKF 更新**（`h_share_model`）

- 世界系变换 → `ikdtree.Nearest_Search`(NUM\_MATCH\_POINTS=5) → 最远邻距离>5 剔除 → `esti_plane`(0.1f) → `s = 1 - 0.9*|pd2|/sqrt(p_body.norm())` >0.9 保留 → 组装 `h_x`(effct×12) 与残差 `h`；`kf.update_iterated_dyn_share_modified(LASER_POINT_COV=0.001, solve_H_time)`。

- OpenMP 并行最近邻（`MP_EN`/`MP_PROC_NUM`）。

**（f）IMU 处理**（`IMU_Processing.hpp`）

- `IMU_init`：`MAX_INI_COUNT=10` 帧静止累计均值/协方差初始化 bias 与重力；`gravity_align_en` 时用 `Quaterniond::FromTwoVectors(acc0, UnitZ)` 构建虚拟水平系 `q_align_`，把 IMU 量测与去畸变点都旋入水平系，重力固定 (0,0,-G)。

- `UndistortPcl`：IMU 前向传播（中值积分）→ 反向按点曲率时间补偿。

- **注意 bug 级问题**：`Process()` 中 IMU 初始化分支 `imu_need_init_ = true;`（应为 false 之前保持 true 直到初始化完成——实际逻辑虽能工作但写法易误）；`ROS_ASSERT(meas.lidar != nullptr)` 在 release 下失效。

**（g）识别的问题清单（全部有源码定位）**

| #   | 类别      | 问题                                                                                                                                                                                 | 位置                                 |
| --- | ------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ---------------------------------- |
| P1  | 架构      | 64.6KB 单文件承载全部职责，\~80 个全局变量，不可测                                                                                                                                                    | `laserMap.cpp` 全文                  |
| P2  | 资源      | 固定大数组：`T1/s_plot*[MAXN=72000]`、`res_last[100000]`、`point_selected_surf[100000]`、`laserCloudOri/normvec/corr_normvect` 各预分配 10 万点                                                   | 文件头部                               |
| P3  | 依赖      | 无用的 `#include <Python.h>` + matplotlibcpp（只在注释掉的绘图中用到），拖慢编译、引入 PythonLibs 依赖                                                                                                       | `laserMap.cpp` 头部、`CMakeLists.txt` |
| P4  | 鲁棒      | LOST 无恢复边、无全局重定位（无 Scan Context/描述子接口）                                                                                                                                             | `update_tracking_state_machine`    |
| P5  | 鲁棒      | 门控阈值固定，未随 `res_mean_last`/协方差自适应；无几何退化检测                                                                                                                                           | `check_pose_update_reasonable`     |
| P6  | 一致性     | 建图用 frame `camera_init`、定位用 `map` 字符串散落 7 处；`path.header.frame_id` 初始化仍写死 `camera_init`                                                                                            | 各 publish\_\*                      |
| P7  | 硬编码     | UNLOCKED→TRACKING 的 streak≥3 硬编码；`MOV_THRESHOLD=1.5f`、`HALF_FOV_COS` 等                                                                                                             | 状态机、fov\_segment                   |
| P8  | 现代C++   | 裸 `new`/原始 FILE\*、`memset` 对非 POD 风险写法、`ros::Time().fromSec` 已废弃 API                                                                                                               | 全文                                 |
| P9  | 构建      | Debug 硬编码、`-std=c++14` 重复 5 次、`-std=c++0x` 混用、cmake\_minimum 2.8.3                                                                                                                 | `CMakeLists.txt`                   |
| P10 | ROS2 障碍 | ros/ros.h、NodeHandle、ros::Time、ROS\_INFO/WARN/ERROR(\_STREAM/\_THROTTLE)、tf(1)、message\_generation、jsk\_rviz\_plugins::OverlayText、livox\_ros\_driver(CustomMsg)、catkin、launch xml | 全文                                 |
| P11 | 并发      | `mtx_buffer` 手动 lock/unlock（异常路径易死锁）、回调与主循环共享全局无原子保护                                                                                                                               | 各 \*\_cbk                          |
| P12 | 性能      | `publish_frame_world`/`map_incremental` 中逐点 RGB 变换深拷贝；`sync_packages` 中 deque 频繁 pop\_front；发布在关键路径同步执行                                                                            | publish\_\*/主循环                    |

### 2.3 参考资料（已检索）

- FAST-LIO2 论文（arXiv:2107.06829）：ikd-Tree 增量更新/并行重建、点到面直接配准、IESKF。

- ikd-Tree 仓库与文档：盒式删除、树上下采样、动态再平衡。

- 先验地图定位实践（FAST\_LIO\_LOCALIZATION、better\_fastlio2 FAST\_LIO\_SAM）：DOP 匹配置信度、Scan Context 在线重定位 + GTSAM 后端。

- FAST-LIO2 源码解读（CSDN 系列）：`h_share_model`、`effct_feat_num`/`res_mean_last` 语义交叉验证。

***

## 3. 目标与非目标

### 3.1 目标

- 迁移 **ROS2 Humble（rclcpp，C++17）+ Ubuntu 22.04 + livox\_ros\_driver2**，功能等价、可编译运行。

- 四维度优化落地，保持双模式与状态机**行为语义等价**（默认配置下输出一致）。

- 状态机与匹配评估模块可脱离 ROS 单元测试（gtest）。

### 3.2 非目标（本轮不做）

- 不做 NDT 分支、不做 GTSAM/PoseGraph 回环后端、不做多会话拼图。

- 不改 IESKF/点到面/ikd-Tree 的数学正确性。

- 不做真实机器人实测；不接除 livox\_ros\_driver2 之外的新驱动。

- Scan Context 全局重定位仅**预留接口**（阶段 4 选项），默认关闭不实现完整描述子。

***

## 4. 拟议变更 (Proposed Changes)

### 阶段 0：环境准备（本地）

1. 在 `d:\重定位\FAST_LIO_Relocation` 建分支 `ros2-upgrade`，在本地展开 ROS2 迁移工作。
2. 目录重排为 ROS2 包结构（保留上游路径以便 diff）：

   - `src/`（节点与模块实现）、`include/fast_lio/`（公开头）、`msg/`、`launch/`、`config/`、`rviz_cfg/`、`test/`。

### 阶段 1：ROS2 迁移（功能等价优先）——解决 P9/P10/P3

1. **构建**：`CMakeLists.txt` 重写为 ament\_cmake（`cmake_minimum_required 3.8`、`set(CMAKE_CXX_STANDARD 17)`、`ament_target_dependencies`：rclcpp、sensor\_msgs、nav\_msgs、geometry\_msgs、visualization\_msgs、tf2、tf2\_ros、tf2\_eigen、pcl\_conversions、livox\_ros\_driver2）；删除 PythonLibs/matplotlibcpp（P3）；`package.xml` 转 format 3。
2. **消息**：`Pose6D.msg` 走 `rosidl_generate_interfaces()`。
3. **节点**：`ros::NodeHandle`→`rclcpp::Node`（建议 `fastlio::LaserMappingNode : public rclcpp::Node`）；
   `nh.param`→`declare_parameter<T>(name, default)` + `get_parameter`；订阅用 `create_subscription<...>`（Livox CustomMsg 用 `livox_ros_driver2::msg::CustomMsg`）；发布 `create_publisher<...>(topic, qos)`，`/Laser_map` latch → `rclcpp::QoS(1).transient_local()`。
4. **时间/日志/tf**：`ros::Time().fromSec`→`rclcpp::Time(seconds)`（注意 sec→nanosec）；`ROS_*`→`RCLCPP_*`（THROTTLE 用 `RCLCPP_*_THROTTLE` 配 clock）；`tf::TransformBroadcaster`→`tf2_ros::TransformBroadcaster`；`eigen_conversions`→`tf2_eigen`。
5. **Overlay**：`jsk_rviz_plugins::OverlayText` 在 ROS2 不可用 → 发布 `visualization_msgs::msg::Marker`(TEXT\_VIEW\_FACING) 到 `/localization_status_marker`，rviz2 直接显示；保留状态文本内容不变。
6. **launch**：`launch/*.launch`→`*.launch.py`（DeclareLaunchArgument + parameters=yaml）；`config/*.yaml` 调整为 ROS2 参数层级（`/**: ros__parameters:`）。
7. **IMU\_Processing/preprocess**：`sensor_msgs::ImuConstPtr`→`sensor_msgs::msg::Imu::ConstSharedPtr`；`livox_ros_driver::CustomMsg`→`livox_ros_driver2::msg::CustomMsg`（字段名一致：line/tag/offset\_time/reflectivity，直接替换命名空间即可，风险低）。
8. 里程碑 M1：`colcon build` 通过，节点能启动并打印参数。

### 阶段 2：模块化重构——解决 P1/P6/P11

将 `laserMap.cpp` 拆为（命名空间 `fast_lio`，各模块构造注入参数，主节点只做装配）：

| 新文件                                          | 承载（从 laserMap.cpp 迁移）                                                                        | 公开接口要点                                                                               |
| -------------------------------------------- | -------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------ |
| `include/fast_lio/types.h`                   | RunMode、LocalizationTrackingState 枚举、配置 struct `LocalizationConfig`（聚合全部 localization/\* 参数） | POD + from\_parameters                                                               |
| `src/localization/state_machine.{hpp,cpp}`   | `update_tracking_state_machine`、`is_match_acceptable/good`、`save_last_*_state`               | `TrackingStateMachine::update(MatchQuality, time_since_start) -> Transition`；纯逻辑、可单测 |
| `src/localization/match_evaluator.{hpp,cpp}` | effct/res\_mean/accept 判定                                                                    | `MatchEvaluator::evaluate(...) -> MatchQuality`                                      |
| `src/localization/pose_gate.{hpp,cpp}`       | `check_pose_update_reasonable`                                                               | `PoseGate::allow(before, after, state) -> bool`                                      |
| `src/mapping/prior_map_manager.{hpp,cpp}`    | `load_prior_map_from_pcd`、`init_localization_map_from_prior`、`init_map_from_first_scan`      | `PriorMapManager::load(pcd_path, voxel) / build_tree(ikdtree)`                       |
| `src/estimation/laser_pipeline.{hpp,cpp}`    | `h_share_model` 中的 NN/平面/残差组装（保留为自由函数回调，签名不变以兼容 esekfom）                                     | 依赖注入 ikdtree/参数                                                                      |
| `src/io/publishers.{hpp,cpp}`                | publish\_frame\_world/body/effect/map/prior\_map/odometry/path/status\_overlay               | `Publishers` 持有 rclcpp publisher + frame\_id 集中管理（P6）                                |
| `src/fastlio_node.cpp`                       | main、参数声明、回调、主循环 sync\_packages                                                              | 装配层                                                                                  |

- **P11 修复**：`mtx_buffer` 改 `std::mutex` + `std::lock_guard`；回调缓冲封装为 `SyncBuffer` 类（deque + 互斥 + cv），消除手动 lock/unlock。

- **P6 修复**：frame\_id 集中为参数 `world_frame`（默认按模式自动 `map`/`camera_init`，可覆盖），删除 7 处散字串。

- 里程碑 M2：行为等价（同配置同数据，话题字段/频率一致），模块可独立编译。

### 阶段 3：代码现代化——解决 P2/P7/P8

- P2：`res_last`/`point_selected_surf` 改 `std::vector` 按需 resize；删除 `T1/s_plot*` MAXN 数组与 Python 计时日志（改 `std::vector<double>` + 简单 CSV 写出，或改为运行时开关默认关）。

- P7：`unlock_to_tracking_streak`（当前硬编码 3）加入 `localization/` 参数；`MOV_THRESHOLD` 等提升为具名常量 `constexpr`。

- P8：`new` → `std::make_unique/make_shared`；`FILE*` → `std::ofstream` RAII；删除 `memset` 初始化（改容器赋值）；裸 `printf/cout` 统一走 RCLCPP 日志宏。

- 接入 `.clang-format`（Google 风格）与 `.clang-tidy`（modernize-*, readability-*, performance-*, bugprone-*），`-Wall -Wextra` 零新增告警。

- 里程碑 M3：clang-tidy 高优先级清零。

### 阶段 4：鲁棒性增强（全部开关化、默认关=原行为）——解决 P4/P5

- **LOST 恢复边**：新增参数 `localization/lost_recovery_en`(默认 false)、`lost_recovery_max_attempts`、`lost_recovery_cooldown_sec`；实现 LOST --(acceptable 连续 N 帧)→ TRACKING 的恢复转移，并记录恢复事件日志。状态机单测覆盖。

- **自适应门控**：`localization/adaptive_gating_en`(默认 false)；`max_position_jump_for_update_locked` 按 `res_mean_last/max_residual_for_good` 与 `trace(P_pos)` 缩放。

- **退化检测**：`localization/degeneracy_check_en`(默认 false)；基于有效点数占比与法向量分布（协方差特征值比）输出退化标志话题 + 门控收紧。

- **（预留接口）** `GlobalRelocalizer` 抽象类 + 空实现/简单实现位，后续可挂 Scan Context；本轮不实现描述子。

- 里程碑 M4：新增行为均有参数开关与 gtest 用例；默认配置下与原行为一致。

### 阶段 5：性能优化——解决 P12 及性能项

- `publish_frame_world`/`map_incremental` 点变换循环改 `pcl::transformPointCloud`（内部 SSE/并行）或 `std::execution::par_unseq`；避免逐点函数调用开销。

- 去畸变输出 `feats_undistort` 复用缓冲（预留 capacity）替代每帧 `pcl_out = *(meas.lidar)` 全量深拷贝（保留正确性前提下评估 move）。

- 先验 PCD 加载与 `ikdtree.Build` 放入独立 std::thread（节点启动不阻塞订阅），加载完成前状态机停留 UNLOCKED 并提示。

- `/Laser_map` 周期重发改为 transient\_local QoS 单次发布 + 参数控制是否周期重发（默认关闭周期重发，减少带宽尖峰）。

- 定位模式下彻底跳过 `lasermap_fov_segment`/`points_cache_collect` 已确认（现状已跳过，重构后确保无残留调用）。

- 里程碑 M5：同等数据下处理耗时不高于基线（以 runtime 日志对比），内存峰值下降（去除 MAXN 数组）。

### 阶段 6：配置/文档标准化

- `config/mid360_relocalization.yaml` 等整理为带注释的 ROS2 参数文件，新增全部新参数默认值与说明。

- README 增补：ROS2 Humble 安装/构建（colcon）/运行、架构图（模块）、状态机转移图、参数表。

- 里程碑 M6：文档与配置齐备。

***

## 5. 假设与决策 (Assumptions & Decisions)

| 决策点          | 结论                                                                         |
| ------------ | -------------------------------------------------------------------------- |
| 交付方式         | 克隆到本地 `d:\重定位\FAST_LIO_Relocation`，分支 `ros2-upgrade`                       |
| 优化覆盖         | 质量现代化 + 架构模块化 + 鲁棒/算法 + 性能/实时性（全选）                                         |
| 验收           | `colcon build` + clang-tidy + gtest（状态机/匹配评估）；示例数据冒烟为辅                     |
| 目标环境         | **ROS2 Humble + Ubuntu 22.04**，C++17                                       |
| 迁移深度         | 功能等价可运行的完整移植（节点/参数/消息/tf/launch/rviz 全覆盖）                                  |
| Livox 驱动     | `livox_ros_driver2`（CustomMsg 字段同名，低风险替换）                                  |
| Overlay      | jsk\_rviz\_plugins 不可用 → `visualization_msgs/Marker` TEXT\_VIEW\_FACING 替代 |
| 行为兼容         | 鲁棒新特性默认关闭；默认配置与原行为等价                                                       |
| 数学算法         | 不改 IESKF/点到面/ikd-Tree 正确性                                                  |
| Scan Context | 仅预留接口，不实现                                                                  |
| 编译验证环境       | 用户侧需 Ubuntu 22.04 + ROS2 Humble（Windows 本地仅编码与静态检查）                        |

***

## 6. 实施步骤 (Implementation Steps)

1. **M0**：克隆仓库、建分支；目录重排；基线 ROS1 逻辑走读对照表（完成）。
2. **M1**：阶段 1 ROS2 迁移 → `colcon build` 通过、节点可启动。
3. **M2**：阶段 2 模块化拆分 + SyncBuffer/frame\_id 集中 → 行为等价。
4. **M3**：阶段 3 现代化（容器化、RAII、参数化硬编码、clang-tidy/format 接入并清零）。
5. **M4**：阶段 4 鲁棒增强（LOST 恢复边、自适应门控、退化检测）+ gtest。
6. **M5**：阶段 5 性能优化 + 耗时对照。
7. **M6**：阶段 6 配置/文档/README。
8. **收尾**：全量 `colcon build` + `colcon test` + clang-tidy；输出变更说明（每文件 what/why）。

***

## 7. 验证 (Verification)

| 层级   | 内容                                                            | 通过标准                       |
| ---- | ------------------------------------------------------------- | -------------------------- |
| 编译   | `colcon build --packages-select fast_lio`（Ubuntu22.04/Humble） | 0 错误；`-Wall -Wextra` 无新增告警 |
| 静态检查 | clang-tidy（modernize/readability/performance/bugprone）        | 高优先级 0                     |
| 单元测试 | gtest：状态机全转移（含 LOST 恢复边）、匹配评估阈值、位姿门控                          | 全部通过                       |
| 行为回归 | 默认参数下与基线同数据对比：话题集合、frame\_id、状态机转移序列、odometry 频率              | 一致                         |
| 冒烟   | 示例 PCD + mid360 数据包（用户提供或公开包）定位模式启动                           | 进入 TRACKING/LOCKED，无崩溃     |
| 性能   | runtime 日志平均帧耗时、内存峰值                                          | 不劣于基线，内存下降                 |

***

## 8. 风险与缓解 (Risks & Mitigations)

| 风险                                | 缓解                                                                |
| --------------------------------- | ----------------------------------------------------------------- |
| ROS2 迁移面大、一次改动过多                  | 阶段化 M1→M6，每阶段可独立编译/回退；先等价后优化                                      |
| livox\_ros\_driver2 字段/时间戳差异      | 接入层封装；若字段不符仅改 preprocess 适配点                                      |
| jsk Overlay 丢失可视化                 | Marker TEXT\_VIEW\_FACING 替代；状态同时输出到 `/localization_status` 字符串话题 |
| 鲁棒新特性引入行为变化                       | 全部参数开关、默认关；行为回归验证默认配置                                             |
| esekfom/ikd-Tree 为 vendored C++14 | C++17 下兼容性验证；不改其内部逻辑                                              |
| 本地 Windows 无法编译 ROS2              | 明确标注：编译与测试在 Ubuntu22.04/Humble 执行；本机仅编码+格式检查                      |

***

## 9. 附：laserMap.cpp 结构地图（重构依据）

```
头部 include(含无用 Python.h/matplotlib) → 宏(INIT_TIME/LASER_POINT_COV/MAXN/PUBFRAME_PERIOD)
→ 全局计时/日志数组(MAXN) → 全局状态(run_mode/状态机/阈值/streak/last_*_state)
→ 全局缓冲(mtx_buffer/sig_buffer/lidar_buffer/imu_buffer/time_buffer)
→ 全局点云对象与 ikdtree、kf(esekf)、Measures、publish 消息缓存
→ SigHandle / dump_lio_state_to_log
→ pointBodyToWorld 系列(4 个重载+RGB 版)
→ points_cache_collect / lasermap_fov_segment(仅建图)
→ standard_pcl_cbk / livox_pcl_cbk / imu_cbk / sync_packages
→ allow_map_update / is_mapping_mode / is_localization_mode / get_tracking_state_name
→ save_last_tracking/locked_state / is_match_acceptable / is_match_good
→ update_tracking_state_machine / check_pose_update_reasonable
→ load_prior_map_from_pcd / init_localization_map_from_prior / init_map_from_first_scan / map_incremental
→ publish_frame_world / publish_frame_body / publish_effect_world / publish_map
→ publish_prior_map / publish_localization_status_overlay
→ refresh_pose_cache_from_state / set_posestamp / publish_odometry / publish_path
→ h_share_model(NN+esti_plane+Jacobian 组装)
→ main(参数读取 → 订阅/发布 → LOST 分支 → p_imu->Process → fov_segment(建图) → 降采样
        → kf.update_iterated_dyn_share_modified → 门控回滚 → 状态机 → LOST 冻结分支
        → map_incremental(建图) → 发布 → 调试日志)
```

