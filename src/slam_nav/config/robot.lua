include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",              -- 地图坐标系
  tracking_frame = "base_link",   -- 机器人基座坐标系，IMU 和激光雷达位于此框架
  published_frame = "base_link",  -- 发布的位姿坐标系，与基座一致
  odom_frame = "odom",            -- 里程计坐标系（未使用，因 provide_odom_frame = false）
  provide_odom_frame = false,     -- 不提供 odom 坐标系，依赖 Cartographer 的内部位姿
  publish_frame_projected_to_2d = false, -- 不限制为 2D 位姿
  use_odometry = false,           -- 不使用外部里程计数据
  use_nav_sat = false,            -- 不使用 GPS 数据
  use_landmarks = false,          -- 不使用地标数据
  num_laser_scans = 1,            -- 使用单个激光雷达，订阅 /scan 主题
  num_multi_echo_laser_scans = 0, -- 不使用多回波激光雷达
  num_subdivisions_per_laser_scan = 1, -- 每个激光扫描不细分
  num_point_clouds = 0,           -- 不使用点云数据
  lookup_transform_timeout_sec = 0.2, -- TF 变换查询超时（秒）
  submap_publish_period_sec = 0.3, -- 子地图发布间隔（秒）
  pose_publish_period_sec = 0.1,  -- 位姿发布间隔（秒，10Hz）
  publish_to_tf = true,           -- 启用 TF 变换发布
  publish_tracked_pose = false,   -- 不发布 /tracked_pose 主题（可选）
  trajectory_publish_period_sec = 0.03, -- 轨迹标记发布间隔（秒，约 33Hz）
  rangefinder_sampling_ratio = 1.0, -- 测距仪消息采样比例（1.0 表示全采样）
  odometry_sampling_ratio = 1.0,  -- 里程计消息采样比例（未使用，设为默认）
  fixed_frame_pose_sampling_ratio = 1., -- 固定帧位姿采样比例
  imu_sampling_ratio = 1.0,       -- IMU 消息采样比例（全采样）
  landmarks_sampling_ratio = 1.0, -- 地标消息采样比例（未使用，设为默认）
}

MAP_BUILDER.use_trajectory_builder_2d = true

TRAJECTORY_BUILDER_2D.use_imu_data = true      -- 启用 IMU 数据
TRAJECTORY_BUILDER_2D.min_range = 0.1          -- 激光雷达最小距离（米）
TRAJECTORY_BUILDER_2D.max_range = 20.0         -- 激光雷达最大距离（米）
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 5.0 -- 缺失数据射线长度
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1 -- 累积范围数据数量

return options