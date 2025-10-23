# 调试指南 - 动态立方体障碍物

## ✅ 编译成功检查表

- [x] 编译成功
- [x] 可执行文件生成: `/home/zr/Fast_Planner_ws/devel/lib/dynamic_obstacles/dynamic_cube_obstacles`
- [x] 包可被找到: `rospack find dynamic_obstacles`
- [x] 已集成到 `topo_replan.launch`

## 🚀 测试步骤

### 方法1：独立测试（推荐先测试）

#### 终端1：启动 roscore
```bash
roscore
```

#### 终端2：启动动态障碍物节点
```bash
cd ~/Fast_Planner_ws
source devel/setup.bash
roslaunch dynamic_obstacles dynamic_cubes.launch
```

**预期输出**：
```
=== Dynamic Cube Obstacles Publisher ===
  Number of obstacles: 5
  Max speed: 1.00 m/s
  Update frequency: 10.0 Hz
  Point cloud resolution: 0.20 m
  Map boundary: [-20.0, -10.0, 0.0] to [20.0, 10.0, 5.0]
✓ Initialized 5 cube obstacles
```

#### 终端3：查看话题
```bash
source ~/Fast_Planner_ws/devel/setup.bash
rostopic list | grep dynamic
```

**预期输出**：
```
/dynamic_obstacles/cloud
/dynamic_obstacles/vis
```

#### 终端4（可选）：查看点云频率
```bash
rostopic hz /dynamic_obstacles/cloud
```

**预期输出**：约 10 Hz

#### 终端5：启动 RViz 可视化
```bash
rviz
```

**RViz 配置**：
1. **Fixed Frame**: 设置为 `world`
2. **添加 PointCloud2**:
   - Topic: `/dynamic_obstacles/cloud`
   - Color Transformer: 选择 `Intensity` 或 `AxisColor`
   - Size: 0.05
3. **添加 MarkerArray**:
   - Topic: `/dynamic_obstacles/vis`

**预期效果**：
- 看到橙色半透明立方体在地图中移动
- 绿色箭头显示速度方向
- 白色点云填充立方体内部

---

### 方法2：集成测试（与 topo_replan 一起运行）

```bash
cd ~/Fast_Planner_ws
source devel/setup.bash
roslaunch plan_manage topo_replan.launch
```

**自定义参数**：
```bash
# 禁用动态障碍物
roslaunch plan_manage topo_replan.launch enable_dynamic_obs:=false

# 修改障碍物数量
roslaunch plan_manage topo_replan.launch dynamic_obs_num:=10

# 修改速度
roslaunch plan_manage topo_replan.launch dynamic_obs_speed:=0.5

# 组合参数
roslaunch plan_manage topo_replan.launch \
  dynamic_obs_num:=8 \
  dynamic_obs_speed:=1.5
```

---

## 🐛 常见问题排查

### 问题1：节点启动失败

**症状**：
```
[dynamic_cube_obstacles-X] process has died
```

**检查步骤**：
```bash
# 1. 检查包是否存在
rospack find dynamic_obstacles

# 2. 检查可执行文件
ls -lh /home/zr/Fast_Planner_ws/devel/lib/dynamic_obstacles/

# 3. 尝试直接运行
rosrun dynamic_obstacles dynamic_cube_obstacles

# 4. 查看详细错误
roslaunch dynamic_obstacles dynamic_cubes.launch --screen
```

**解决方案**：
- 确保已经 `source devel/setup.bash`
- 重新编译：`catkin_make`

---

### 问题2：没有点云输出

**检查步骤**：
```bash
# 查看话题列表
rostopic list | grep dynamic

# 查看话题信息
rostopic info /dynamic_obstacles/cloud

# 尝试接收消息
rostopic echo /dynamic_obstacles/cloud --noarr
```

**如果没有任何输出**：
- 检查节点是否运行：`rosnode list | grep dynamic`
- 查看节点日志：`rosnode info /dynamic_cube_obstacles`

**如果点云为空**：
- 检查分辨率参数是否过大
- 检查障碍物数量是否为 0

---

### 问题3：RViz 中看不到障碍物

**检查 PointCloud2**：
1. Fixed Frame 必须是 `world`
2. Topic 正确：`/dynamic_obstacles/cloud`
3. Size 不要太小：建议 0.05-0.1

**检查 MarkerArray**：
1. Topic 正确：`/dynamic_obstacles/vis`
2. 确保节点正在发布：`rostopic hz /dynamic_obstacles/vis`

**调试命令**：
```bash
# 查看点云数量
rostopic echo /dynamic_obstacles/cloud/width

# 查看标记数量
rostopic echo /dynamic_obstacles/vis/markers/[0]/ns
```

---

### 问题4：障碍物不移动或移动不连贯

**检查更新频率**：
```bash
rostopic hz /dynamic_obstacles/cloud
```

**如果频率低于 10 Hz**：
- 检查 CPU 负载
- 降低点云分辨率：
  ```xml
  <param name="resolution" value="0.3"/>  <!-- 默认 0.2 -->
  ```
- 减少障碍物数量

**如果完全不移动**：
- 检查速度参数：`obs_max_speed` 不应为 0
- 查看节点日志是否有错误

---

### 问题5：障碍物穿墙或消失

**症状**：障碍物移出地图边界

**检查地图参数**：
```cpp
// 在代码中的地图边界应该与 topo_replan.launch 一致
map_min_ = Vector3d(-20.0, -10.0, 0.0);
map_max_ = Vector3d(20.0, 10.0, 5.0);
```

**验证**：
```bash
# 查看节点启动日志中的地图边界信息
roslaunch dynamic_obstacles dynamic_cubes.launch | grep "Map boundary"
```

---

### 问题6：与 ESDF 集成失败

**当前状态**：
- 动态障碍物点云发布到: `/dynamic_obstacles/cloud`
- SDFMap 订阅: `/sdf_map/cloud` 或 `/pcl_render_node/cloud`

**集成方案A：话题重映射**（简单）

修改 `topo_replan.launch`：
```xml
<!-- 将动态障碍物点云合并到静态地图话题 -->
<remap from="/dynamic_obstacles/cloud" to="/map_generator/global_cloud"/>
```

⚠️ **注意**：这会覆盖静态地图的点云！

**集成方案B：修改 SDFMap**（推荐）

在 `sdf_map.cpp` 的 `initMap()` 函数中添加：
```cpp
// 订阅动态障碍物点云
dynamic_cloud_sub_ = node_.subscribe<sensor_msgs::PointCloud2>(
    "/dynamic_obstacles/cloud", 10, &SDFMap::cloudCallback, this);
```

并在头文件中添加成员变量：
```cpp
ros::Subscriber dynamic_cloud_sub_;
```

**集成方案C：点云合并节点**（待实现）

使用 `pcl_ros` 合并多个点云源。

---

## 📊 性能优化

### CPU 使用率过高

**优化参数**：
```xml
<!-- 降低更新频率 -->
<param name="freq" value="5.0"/>  <!-- 从 10.0 降到 5.0 -->

<!-- 降低点云密度 -->
<param name="resolution" value="0.3"/>  <!-- 从 0.2 增加到 0.3 -->

<!-- 减少障碍物数量 -->
<arg name="dynamic_obs_num" value="3"/>  <!-- 从 5 降到 3 -->
```

### 内存使用优化

**当前内存占用估算**：
- 每个点: ~12 bytes (PCL PointXYZ)
- 单个立方体（1.5m x 1.5m x 2.0m @ 0.2m 分辨率）: ~1000 点
- 5个障碍物: ~5000 点 ≈ 60 KB

**如果需要更多障碍物**：
- 增大 `resolution` 减少点数
- 或者使用空心立方体（只渲染表面）

---

## 🔍 调试命令速查表

```bash
# ===== 基本检查 =====
rospack find dynamic_obstacles
rosnode list | grep dynamic
rostopic list | grep dynamic

# ===== 话题监控 =====
rostopic hz /dynamic_obstacles/cloud    # 频率
rostopic bw /dynamic_obstacles/cloud    # 带宽
rostopic echo /dynamic_obstacles/cloud --noarr  # 查看头信息
rostopic echo /dynamic_obstacles/vis/markers/[0]/pose  # 查看位置

# ===== 节点信息 =====
rosnode info /dynamic_cube_obstacles
rosrun rqt_graph rqt_graph   # 可视化节点图

# ===== 性能分析 =====
top -p $(pgrep dynamic_cube)  # CPU 使用率
ps aux | grep dynamic_cube    # 内存使用

# ===== 录制与回放 =====
rosbag record /dynamic_obstacles/cloud /dynamic_obstacles/vis -O test.bag
rosbag play test.bag
rosbag info test.bag
```

---

## 📝 参数配置快速参考

| 参数 | 默认值 | 说明 | 建议范围 |
|------|--------|------|----------|
| `obs_num` | 5 | 障碍物数量 | 1-20 |
| `obs_max_speed` | 1.0 m/s | 最大速度 | 0.1-3.0 |
| `freq` | 10.0 Hz | 更新频率 | 5-30 |
| `resolution` | 0.2 m | 点云分辨率 | 0.1-0.5 |
| `size_min_x/y/z` | 0.5 m | 最小尺寸 | 0.3-2.0 |
| `size_max_x/y/z` | 1.5 m | 最大尺寸 | 0.5-3.0 |

---

## ✅ 功能验证清单

- [ ] 节点正常启动
- [ ] 点云话题正常发布（10 Hz）
- [ ] 可视化标记正常发布
- [ ] RViz 中能看到橙色立方体
- [ ] RViz 中能看到绿色速度箭头
- [ ] RViz 中能看到白色点云
- [ ] 障碍物在地图内移动
- [ ] 碰到边界会反弹
- [ ] 可以调整参数（数量、速度）
- [ ] 可以禁用动态障碍物（`enable_dynamic_obs:=false`）

---

## 📞 需要帮助？

如果遇到未解决的问题：

1. 检查 ROS 日志：
   ```bash
   roscd log
   cat latest/*.log | grep -i error
   ```

2. 查看源码注释：`src/dynamic_cube_obstacles.cpp`

3. 查看完整文档：`README.md` 和 `QUICKSTART.md`

4. 提供以下信息：
   - 错误日志
   - `rostopic list` 输出
   - `rosnode list` 输出
   - 参数配置

---

**最后更新**: 2025-10-19
