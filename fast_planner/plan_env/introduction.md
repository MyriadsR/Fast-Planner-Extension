<!--
 * @Author: xzr && 1841953204@qq.com
 * @Date: 2025-10-22 11:44:26
 * @LastEditors: xzr && 1841953204@qq.com
 * @LastEditTime: 2025-10-22 11:44:39
 * @FilePath: /Fast_Planner_ws/src/Fast-Planner-Extension/fast_planner/plan_env/introduction.md
 * @Description: 
 * 
 * Copyright (c) 2025 by 1841953204@qq.com, All Rights Reserved. 
-->
• 下面基于 plan_env 下的实现详细解释 Fast-Planner 地图（占据+ESDF）的构建流程与数据流。关键文件：

  - 地图核心：sdf_map.h / sdf_map.cpp
  - EDT 查询封装：edt_environment.h / edt_environment.cpp
  - 辅助模块：raycast.h/.cpp（栅格穿越）、obj_predictor/obj_generator（与动态物体相关，不是静态地图的主流程）

  总体目标

  - 将深度图或点云融入占据栅格（概率 log-odds），局部膨胀后构建三维 ESDF（欧氏距离场），并提供查询接口（距离/梯度）。ESDF 支撑路径碰撞检测、优化梯度
    与可视化。

  一、SDFMap 初始化与参数

  - 函数：SDFMap::initMap(ros::NodeHandle& nh)，路径 src/.../plan_env/src/sdf_map.cpp:17
  - 读取参数（命名空间 sdf_map/）：
      - 分辨率与地图尺寸：resolution、map_size_{x,y,z}、map_origin、ground_height
      - 局部更新范围与边界膨胀：local_update_range_{x,y,z}、local_bound_inflate、local_map_margin
      - 占据概率模型：p_hit/p_miss/p_min/p_max/p_occ 及其 logit（转 log-odds）
      - 深度相机内参：fx, fy, cx, cy；深度预处理：use_depth_filter、阈值与下采样 skip_pixel
      - 射线长度范围：min_ray_length/max_ray_length
      - 可视化高度裁剪、虚拟天花板：visualization_truncate_height、virtual_ceil_height
  - 分配栅格缓冲：
      - occupancy_buffer_（log-odds）、occupancy_buffer_inflate_（二值占据+膨胀）
      - distance_buffer_/distance_buffer_neg_/distance_buffer_all_（正负距与组合）
      - 各类临时缓存与标记（count_hit、flag_rayend/flag_traverse 等）
  - ROS 订阅/发布与定时器：
      - 输入：/sdf_map/depth 与 pose/odom 同步；也可直接用 /sdf_map/cloud 与 /sdf_map/odom
      - 定时器：updateOccupancyCallback（0.05s）、updateESDFCallback（0.05s）、visCallback（0.05s）
      - 可视化发布：occupancy、occupancy_inflate、esdf、update_range、unknown、depth_cloud

  二、占据融合（updateOccupancyCallback）

  - 触发条件：有新深度图/点云且相机位姿有效时，置 occ_need_update_ 为 true。
  - 步骤：
      1. projectDepthImage()
          - 将深度图投到世界坐标，得到稀疏点云 md_.proj_points_（可选深度滤波与下采样）。
      2. raycastProcess()
          - 对每个投影点，进行从相机位置到该点的栅格射线穿越，沿线将体素更新为自由（0），终点更新为击中（1），同时记录更新范围的包围盒作为后续“局部更
            新边界”。
          - 对越界点，先投回地图边界或截断射线到 max_ray_length。
          - 使用队列缓存需要更新 log-odds 的体素，按“命中/未命中计数”结合 p_hit/p_miss 在局部范围内融合；局部范围之外的体素被清空为最小 log-odds。
      3. clearAndInflateLocalMap()
          - 在局部边界内，将 occupancy_buffer_ 高于阈值的体素膨胀（obstacles_inflation_ → inf_step 体素），写入 occupancy_buffer_inflate_。
          - 可选添加虚拟天花板限制飞行高度。
  - 结果：得到局部区域更新后的二值占据（含膨胀），为 ESDF 更新做准备；同时设置 esdf_need_update_。

  三、ESDF 更新（updateESDFCallback → updateESDF3d）

  - 算法：Felzenszwalb 一维距离变换的 3 次串联，三维正/负距离分别计算，最后组合。
  - 正距离（到障碍物）：
      - 将 occupancy_buffer_inflate_ 中障碍体素作为 0，空闲体素为 +inf，沿 z、y、x 三个维度依次做 1D DT，得到 distance_buffer_（乘以分辨率）。
  - 负距离（到自由空间）：
      - 将 free/occ 取反生成 occupancy_buffer_neg_，重复上述 1D DT，得到 distance_buffer_neg_。
  - 组合得到 distance_buffer_all_：
      - 对占据体素，用负距（带符号）；对空闲体素，用正距。
  - 该局部边界由 raycastProcess 确定，减少计算量。可选时间统计与可视化发布。

  四、EDTEnvironment 封装查询

  - 类：EDTEnvironment（src/.../plan_env/src/edt_environment.cpp）
      - setMap(shared_ptr<SDFMap>) 关联地图
      - evaluateCoarseEDT(pos, time): 返回 pos 的距离（粗略）
      - evaluateEDTWithGrad(pos, time, dist, grad): 三线性插值从 distance_buffer_all_ 取值与梯度（用于优化）
      - 其他辅助：minDistToAllBox 等
  - 规划器通过 edt_environment_ 查询距离与梯度：
      - 碰撞检测，如 findCollisionRange 内 compare evaluateCoarseEDT 与 clearance
      - 代价与梯度构造（BsplineOptimizer NORMAL_PHASE）

  五、数据流总结

  - 输入：深度图/点云 + 相机位姿
  - 投影与射线：生成自由与占据观测，并更新局部 log-odds
  - 膨胀：把障碍进行半径膨胀，贴近现实安全距离
  - ESDF：在局部边界内计算正/负距离并组合，提供连续距离场
  - 查询：EDTEnvironment 对外提供距离/梯度接口，给 A*、优化器、可视化使用

  六、常见配置与调试建议

  - 分辨率/尺寸：resolution、map_size_* 与 local_update_range_* 要与飞行/感知尺度匹配
  - 概率参数：p_hit/p_miss/p_min/p_max/p_occ 与传感器噪声、视野遮挡相关，过于保守会导致地图膨胀过多
  - 深度滤波与下采样：use_depth_filter、skip_pixel、深度范围等影响投影点质量与速度
  - 膨胀半径：obstacles_inflation_ 直接影响安全裕度与路径的可行性
  - ESDF 更新范围：local_bound_inflate 与局部包围盒设置可兼顾效率与完整性
  - 可视化：开启 publishMap/ESDF/unknown/depth_cloud 方便验证感知-融合-距离链路

  如果你希望，我可以添加一个“地图健康检查”工具函数，快速打印当前 ESDF 更新频率、局部边界、占据体素比例、最近障碍距离等，帮助在线诊断映射质量与规划失
  败原因。