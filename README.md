# ucar_localization

基于 ICP 的激光 scan-to-map 定位节点，为移动小车提供 `map` 坐标系下的位姿估计，并发布 `map → odom` 变换。

不依赖 amcl 的粒子滤波，直接用地图点云与单帧激光做配准；通过里程计给出预测初值，用 ICP 求解修正量。适合结构特征明显、需要平滑连续位姿的室内场景。

## 工作原理

节点启动后拉取 `map` 话题的栅格地图，转为点云并建立 KD-Tree。每收到一帧激光：

1. **去畸变** — 按 `use_scan_midpoint_time` / `enable_scan_deskew`，结合里程计把一帧内各点校正到同一时刻
2. **预处理** — 统计离群点滤波（`enable_outlier_removal`）→ 体素降采样 → 剔除与地图点云距离过远的障碍点（`ObstacleRemoval_Distance_Max`）
3. **预测初值** — 由 `odom` 队列推算相邻帧间运动量，得到预测位姿
4. **位姿搜索** — 可选地在平移 / 偏航角邻域内枚举候选位姿，用打分选出最优初值（`enable_yaw_search`、`search_*`）
5. **ICP 精配准** — 以候选位姿为初值做点到面 ICP，得到最终位姿
6. **平滑与发布** — 按运动状态选择平滑系数，发布定位结果与 `map → odom`

针对两类易失败场景有专门机制：

- **启动重定位**（`enable_startup_relocalization`）— 初始若干帧在小范围内穷举搜索，避免起步时落入局部最优
- **全局偏航角搜索**（`enable_initial_global_yaw_search`）— 首帧在 ±180° 内按步长搜索，解决初始朝向未知的问题

静止时会锁定位姿抑制抖动（`suppress_static_jitter`、`static_*` 系列阈值）。

## 依赖

| 类型 | 依赖 |
|---|---|
| ROS | Melodic / Noetic，catkin 构建 |
| ROS 库 | `roscpp` `tf` `tf2` `tf2_ros` `tf2_geometry_msgs` `sensor_msgs` `nav_msgs` `geometry_msgs` `diagnostic_msgs` |
| 第三方 | [PCL](https://pointclouds.org/)（`common` `kdtree` `registration` `filters`）、Eigen3 |

编译需要 PCL 开发包：

```bash
sudo apt install libpcl-dev
```

## 编译

```bash
cd ~/ucar_ws
catkin_make          # 或 catkin build
source devel/setup.bash
```

## 运行

```bash
roslaunch ucar_localization scan_to_map_location.launch
```

该 launch 会加载 `config/icp_params.yaml` 到节点私有命名空间。运行前需要：

- 有节点发布 `map` 话题（通常是 `map_server`）
- 有雷达发布 `/scan`
- 有底盘发布 `odom`

在 `ucar_navigation` 中已由 `navigation.launch` 自动拉起。

手动指定初始位姿（未启用启动重定位时使用）：

```bash
rostopic pub /initialpose geometry_msgs/PoseWithCovarianceStamped ...
```

或在 RViz 中用 "2D Pose Estimate" 工具。

## 接口

### 订阅

| 话题 | 类型 | 说明 |
|---|---|---|
| `/scan` | `sensor_msgs/LaserScan` | 激光数据 |
| `map` | `nav_msgs/OccupancyGrid` | 栅格地图，转为点云后作为配准目标 |
| `odom` | `nav_msgs/Odometry` | 里程计，用于预测初值与去畸变 |
| `initialpose` | `geometry_msgs/PoseWithCovarianceStamped` | 手动初始位姿 |

### 发布

| 话题 | 类型 | 说明 |
|---|---|---|
| `location_match` | `PoseWithCovarianceStamped` | **定位结果**，含协方差 |
| `map → odom` | TF | 供导航栈使用 |
| `localization_state` | `std_msgs/String` | 定位状态，latched |
| `localization_diagnostics` | `diagnostic_msgs/DiagnosticArray` | 诊断信息，latched |
| `removal_pointcloud` | `PointCloud2` | 剔除的障碍点 |
| `debug_predicted_pose` / `debug_final_pose` | `PoseStamped` | 预测 / 最终位姿 |
| `debug_predicted_scan` / `debug_scan_used` / `debug_final_scan` | `PointCloud2` | 各阶段点云 |

调试话题默认关闭，需将 `if_debug` / `debug_publish_poses` / `debug_publish_clouds` 置 `true`。

## 参数

全部参数位于 `config/icp_params.yaml`（共 75 项），分以下几组：

### 坐标系

| 参数 | 默认 | 说明 |
|---|---|---|
| `map_frame` | `map` | 地图坐标系 |
| `odom_frame` | `odom` | 里程计坐标系 |
| `base_frame` | `base_link` | 车体坐标系 |
| `lidar_frame` | `laser_frame` | 雷达坐标系 |

### ICP 收敛判据

| 参数 | 说明 |
|---|---|
| `SCORE_THRESHOLD_MAX` | 迭代结束代价仍高于此值判为不收敛 |
| `Maximum_Iterations` | 最大迭代次数 |
| `ANGLE_THRESHOLD` / `DIST_THRESHOLD` | 最小变换量（提前终止条件） |
| `ANGLE_UPPER_THRESHOLD` | 最大允许角度变换 |
| `Point_Quantity_THRESHOLD` | 有效点数下限 |
| `ANGLE_SPEED_THRESHOLD` | 角速度超过此值时不发布定位（防转弯时跳变） |

### 点云预处理

| 参数 | 默认 | 说明 |
|---|---|---|
| `ObstacleRemoval_Distance_Max` | 1.3 | 与地图最近点距离超过此值视为障碍点剔除 |
| `VoxelGridRemoval_LeafSize` | 0.02 | 体素滤波边长 |
| `enable_outlier_removal` | true | 统计离群点滤波开关 |
| `outlier_mean_k` | 16 | 统计邻域点数，过大会吃掉墙角 |
| `outlier_stddev_mul` | 1.5 | 离群判定阈值，越小越严格 |

### 位姿搜索

`enable_yaw_search` 控制是否在邻域枚举候选；`search_translation_range` / `search_translation_step` / `yaw_search_range_deg` / `yaw_search_step_deg` 定义搜索空间；`search_accept_score_threshold` 及两个 `improvement_*` 决定是否采纳搜索结果。`local_map_radius` 控制参与匹配的局部地图半径（默认 7.5 m）。

### 启动重定位与全局搜索

`enable_startup_relocalization` 与 `enable_initial_global_yaw_search` 两项，配合 `initial_global_yaw_search_range_deg`（默认 180°）等参数，解决启动时初始位姿未知的问题。代价是启动阶段耗时增加。

### map→odom 发布

| 参数 | 默认 | 说明 |
|---|---|---|
| `publish_map_to_odom_tf` | true | 是否发布 TF |
| `map_to_odom_publish_rate` | 40.0 | 发布频率 |
| `smooth_map_to_odom` | true | 平滑开关 |
| `map_to_odom_stationary_alpha` | 0.12 | 静止时的平滑系数（越小越稳） |
| `map_to_odom_moving_alpha` | 0.85 | 运动时的平滑系数（越大跟随越紧） |
| `map_to_odom_moving_linear_threshold` | 0.08 | 判定为运动的线速度阈值 |
| `map_to_odom_moving_angular_threshold` | 0.15 | 判定为运动的角速度阈值 |

## 调参建议

- **转弯时定位跳变** — 调低 `ANGLE_SPEED_THRESHOLD`，或增大 `map_to_odom_moving_alpha` 之外的平滑力度
- **静止时位姿抖动** — 开 `suppress_static_jitter`，调小 `static_pose_translation_epsilon` / `static_pose_yaw_epsilon`
- **起步定位错误** — 确认 `enable_startup_relocalization` 与 `enable_initial_global_yaw_search` 为 `true`，必要时增大 `startup_relocalization_frames`
- **动态障碍物误剔除** — 增大 `ObstacleRemoval_Distance_Max`
- **墙面点云被滤掉** — 减小 `outlier_mean_k` 或增大 `outlier_stddev_mul`

## 已知问题

1. **10 个参数未在 `icp_params.yaml` 中暴露** — 代码读取但配置文件里没有，只能用代码内置默认值：

   | 参数 | 代码默认 | 说明 |
   |---|---|---|
   | `Scan_Range_Max` | 20 | 雷达数据最大距离 |
   | `Scan_Range_Min` | 0.3 | 雷达数据最小距离 |
   | `AGE_THRESHOLD` | 1 | scan 与匹配结果的最大时间间隔 |
   | `odom_queue_length` | 300 | 里程计队列长度 |
   | `adaptive_threshold_initial` | 0.20 | 自适应阈值初值 |
   | `adaptive_threshold_min_motion` | 0.05 | 触发阈值更新的最小运动量 |
   | `registration_translation_epsilon` | 1e-3 | 配准平移收敛阈值 |
   | `registration_rotation_epsilon` | 1e-3 | 配准旋转收敛阈值 |
   | `publish_localization_state` | true | 是否发布状态话题 |
   | `publish_localization_diagnostics` | true | 是否发布诊断话题 |

   其中 `Scan_Range_Min` / `Scan_Range_Max` 和 `AGE_THRESHOLD` 是实际调参时常用到的，建议补进 yaml。

2. **`package.xml` 的 `<license>` 为 `TODO`** — 公开仓库建议补上明确的许可证。

3. **`debug_odom_jump_*` 与 `debug_pose_jump_threshold`** — 名字带 `debug_` 前缀，但实际参与运行时的跳变判断，不只是调试输出，参数命名有误导性。

## License

待补充。
