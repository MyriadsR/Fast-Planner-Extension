# 快速入门指南

## 1. 编译

```bash
cd ~/Fast_Planner_ws
catkin build dynamic_obstacles
source devel/setup.bash
```

## 2. 测试动态障碍物（独立运行）

### 启动节点

```bash
roslaunch dynamic_obstacles dynamic_cubes.launch
```

### 查看话题

```bash
# 查看点云
rostopic echo /dynamic_obstacles/cloud

# 查看点云频率
rostopic hz /dynamic_obstacles/cloud

# 查看可视化标记
rostopic list | grep dynamic
```

### RViz 可视化

```bash
# 启动 RViz
rviz

# 添加显示项：
# 1. Fixed Frame: world
# 2. Add -> PointCloud2 -> Topic: /dynamic_obstacles/cloud
# 3. Add -> MarkerArray -> Topic: /dynamic_obstacles/vis
```

## 3. 集成到 topo_replan

### 方式1：使用集成launch文件

```bash
roslaunch dynamic_obstacles topo_replan_with_dynamic_obs.launch
```

### 方式2：修改原有的 topo_replan.launch

在 `topo_replan.launch` 文件末尾添加：

```xml
  <!-- 动态立方体障碍物 -->
  <include file="$(find dynamic_obstacles)/launch/dynamic_cubes.launch">
    <arg name="obs_num" value="5"/>
    <arg name="obs_max_speed" value="1.0"/>
  </include>
</launch>
```

## 4. 连接到 ESDF Map

### 理解数据流

```
动态障碍物        静态障碍物
    ↓               ↓
点云话题         点云话题
    ↓               ↓
[需要合并或分别订阅]
    ↓
SDFMap (cloudCallback)
    ↓
ESDF 距离场
    ↓
路径规划算法
```

### 方法A：重映射到同一话题（简单但可能冲突）

在 `dynamic_cubes.launch` 中修改：

```xml
<remap from="~cloud" to="/map_generator/global_cloud"/>
```

**注意**：这会覆盖静态地图的点云！

### 方法B：修改 SDFMap 订阅多个话题（推荐）

修改 `sdf_map.cpp`，添加第二个点云订阅器：

```cpp
// 订阅静态地图
indep_cloud_sub_ = node_.subscribe<sensor_msgs::PointCloud2>(
    "/sdf_map/cloud", 10, &SDFMap::cloudCallback, this);

// 订阅动态障碍物
dynamic_cloud_sub_ = node_.subscribe<sensor_msgs::PointCloud2>(
    "/dynamic_obstacles/cloud", 10, &SDFMap::cloudCallback, this);
```

### 方法C：使用点云合并节点

```bash
# 安装点云工具
sudo apt-get install ros-noetic-pcl-ros

# 使用 nodelet 合并点云（需要额外配置）
```

## 5. 参数调优

### 增加障碍物密度

```bash
roslaunch dynamic_obstacles dynamic_cubes.launch \
  obs_num:=10 \
  resolution:=0.1
```

### 减小障碍物速度（适合调试）

```bash
roslaunch dynamic_obstacles dynamic_cubes.launch \
  obs_max_speed:=0.3
```

### 自定义障碍物尺寸

```bash
roslaunch dynamic_obstacles dynamic_cubes.launch \
  size_min_x:=1.0 size_max_x:=2.0 \
  size_min_y:=1.0 size_max_y:=2.0 \
  size_min_z:=1.0 size_max_z:=3.0
```

## 6. 常见问题

### Q1: 编译错误 "找不到 PCL"

```bash
sudo apt-get install libpcl-dev ros-noetic-pcl-ros ros-noetic-pcl-conversions
```

### Q2: 点云不显示

检查话题：
```bash
rostopic info /dynamic_obstacles/cloud
```

检查节点：
```bash
rosnode list | grep dynamic
rosnode info /dynamic_cube_obstacles
```

### Q3: ESDF 中看不到动态障碍物

确认点云话题连接：
```bash
rostopic info /sdf_map/cloud
```

需要将 `/dynamic_obstacles/cloud` 重映射或合并到 SDFMap 订阅的话题。

### Q4: 障碍物运动不连贯

增加更新频率：
```xml
<param name="freq" value="30.0"/>  <!-- 默认 10.0 -->
```

### Q5: 计算负载过高

降低点云分辨率：
```xml
<param name="resolution" value="0.3"/>  <!-- 默认 0.2 -->
```

## 7. 高级用法

### 录制 rosbag

```bash
rosbag record /dynamic_obstacles/cloud /dynamic_obstacles/vis /tf -O dynamic_obs.bag
```

### 回放 rosbag

```bash
rosbag play dynamic_obs.bag
```

### 性能监控

```bash
# CPU 使用率
top -p $(pgrep dynamic_cube)

# 话题带宽
rostopic bw /dynamic_obstacles/cloud

# 话题延迟
rostopic delay /dynamic_obstacles/cloud
```

## 8. 下一步

- [ ] 调整障碍物参数适配你的场景
- [ ] 集成到完整的规划系统
- [ ] 添加自定义的障碍物运动模式
- [ ] 优化点云分辨率和更新频率

## 需要帮助？

查看完整文档：`README.md`

检查源码注释：`src/dynamic_cube_obstacles.cpp`
