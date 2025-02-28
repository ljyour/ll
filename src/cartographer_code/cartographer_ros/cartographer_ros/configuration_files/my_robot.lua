include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,                -- map_builder.lua的配置信息
  trajectory_builder = TRAJECTORY_BUILDER,  -- trajectory_builder.lua的配置信息
  
  map_frame = "map",                        -- 地图坐标系的名字
  tracking_frame = "base_link",              -- 将所有传感器数据转换到这个坐标系下
  published_frame = "base_footprint",       			-- tf: map -> base_footprint
  odom_frame = "odom",                      -- 里程计的坐标系名字
  provide_odom_frame = true,               -- 是否提供odom的tf, 如果为true,则tf树为map->odom->base_footprint
                                            -- 如果为false tf树为map->base_footprint
  publish_frame_projected_to_2d = false,    -- 是否将坐标系投影到平面上
  --use_pose_extrapolator = false,          -- 发布tf时是使用pose_extrapolator的位姿还是前端计算出来的位姿

  use_odometry = false,                      -- 是否使用里程计,如果使用要求一定要有odom的tf
  use_nav_sat = false,                      -- 是否使用gps
  use_landmarks = false,                    -- 是否使用landmark
  num_laser_scans = 1,                      -- 是否使用单线激光数据
  num_multi_echo_laser_scans = 0,           -- 是否使用multi_echo_laser_scans数据
  num_subdivisions_per_laser_scan = 1,      -- 1帧数据被分成几次处理,一般为1
  num_point_clouds = 0,                     -- 是否使用多线点云数据
  
  lookup_transform_timeout_sec = 0.2,       -- 查找tf时的超时时间
  submap_publish_period_sec = 0.3,          -- 发布数据的时间间隔
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,

  rangefinder_sampling_ratio = 1.,          -- 传感器数据的采样频率
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = true
TRAJECTORY_BUILDER_2D.use_imu_data = true
TRAJECTORY_BUILDER_2D.min_range = 0.
TRAJECTORY_BUILDER_2D.max_range = 12.   --default 30.   减小(减小需要处理的点数, 在雷达数据远距离的点不准时一定要减小这个值)


--修改以下参数,减少全局及本地SLAM(前端和后端)延迟
--TRAJECTORY_BUILDER_2D.voxel_filter_size = 0.025        --default 0.025 增大(增大体素滤波的立方体的边长,相当于减小了需要处理的点数)
--TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_length = 0.5          --default 0.5   增大(增大自适应体素滤波立方体边长,相当于减少需要处理的点数)
--TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.min_num_points = 200    --default 200   减小(减小降采样后最小点云数量)
--TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_range = 50.         --default 50.    减小(减小自适应体素滤波器的最大范围)
--TRAJECTORY_BUILDER_2D.submaps.resolution = 0.05   --default 0.05  增大(增大了submap分辨率，相当于减小了匹配时的搜索量)
--TRAJECTORY_BUILDER_2D.submaps.num_range_data = 90 --default 90    减小(减小插入submap的激光雷达点数)
--修改以下参数,减少全局SLAM(后端)延迟
POSE_GRAPH.optimize_every_n_nodes = 160   --default 90 减小(多少数量的轨迹节点插入到地图中，则执行一次优化,值越高，执行后端优化的频率越低)
--POSE_GRAPH.max_num_final_iterations = 200 --default 200 减小(最大迭代次数)
POSE_GRAPH.global_sampling_ratio = 0.002                           --default 0.003  减小(全局采样率)  
--POSE_GRAPH.global_constraint_search_after_n_seconds = 10.          --default 10.    提高(n秒后全局约束的搜索)  
-- ceres匹配的一些配置参数 
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = false  -- 是否使用 real_time_correlative_scan_matcher 为ceres提供先验信息
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 10.   --default 10.ceres扫描匹配的平移权重
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 1.      --default 1. ceres扫描匹配的旋转权重



-- 修改以下参数,降低纯定位时错误重定位的概率
POSE_GRAPH.constraint_builder.sampling_ratio = 0.2                 --default 0.3    减小(对局部子图进行回环检测时的计算频率, 数值越大, 计算次数越多)  
POSE_GRAPH.constraint_builder.max_constraint_distance = 15.        --default 15.    减小(对局部子图进行回环检测时能成为约束的最大距离)  
POSE_GRAPH.constraint_builder.min_score = 0.55                     --default 0.55   提高(对局部子图进行回环检测时的最低分数阈值)  
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.6  --default 0.6    提高(减小计算约束的数量，提高约束正确的概率)  


return options
