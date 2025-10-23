# Dynamic Cube Obstacles for Fast Planner

这个包为 Fast Planner 提供动态立方体障碍物，通过生成点云的方式，自动转换为 ESDF Map。

## 功能特性

- ✅ 生成动态移动的立方体障碍物
- ✅ 自动边界碰撞检测和反弹
- ✅ 点云输出，兼容 ESDF Map 转换
- ✅ RViz 可视化支持
- ✅ 与 topo_replan.launch 地图参数自动对齐

## 工作原理

### 地图生成流程（与 topo_replan.launch 一致）

```
动态障碍物点云
    ↓
[cloudCallback] 接收点云
    ↓
[setOccupied] 标记占用体素
    ↓
[updateESDF3d] 计算欧氏距离场
    ↓
ESDF Map (用于规划)
```

**核心流程**：
1. **本节点** 生成动态立方体的点云（填充内部点）
2. **SDFMap** (`sdf_map.cpp`) 订阅点云话题
3. **点云自动转换为 ESDF**，供规划算法使用

### 与静态障碍物的区别

| 特性 | 静态障碍物 (random_forest) | 动态障碍物 (本包) |
|------|---------------------------|-------------------|
| 生成方式 | 圆柱体 + 圆环点云 | 立方体点云 |
| 运动状态 | 静止 | 动态移动 + 边界反弹 |
| 更新频率 | 一次性生成 | 持续更新（10Hz） |
| ESDF转换 | ✅ 支持 | ✅ 支持 |

## 编译

```bash
cd ~/Fast_Planner_ws
catkin build dynamic_obstacles
source devel/setup.bash
```

## 使用方法

### 方法 1：独立运行

```bash
roslaunch dynamic_obstacles dynamic_cubes.launch obs_num:=5 obs_max_speed:=1.0
```

**参数说明**：
- `obs_num`: 障碍物数量（默认5个）
- `obs_max_speed`: 最大速度 (m/s，默认1.0)
- `freq`: 更新频率 (Hz，默认10)
- `resolution`: 点云分辨率 (m，默认0.2)
- `size_min_x/y/z`: 立方体最小尺寸 (m)
- `size_max_x/y/z`: 立方体最大尺寸 (m)

### 方法 2：集成到 topo_replan.launch

在 `topo_replan.launch` 中添加：

```xml
<!-- 添加动态障碍物 -->
<include file="$(find dynamic_obstacles)/launch/dynamic_cubes.launch">
  <arg name="obs_num" value="5"/>
  <arg name="obs_max_speed" value="1.0"/>
</include>
```

### 方法 3：点云话题重映射（连接到 ESDF Map）

如果需要将动态障碍物点云合并到 ESDF Map：

```xml
<node pkg="dynamic_obstacles" type="dynamic_cube_obstacles" name="dynamic_cube_obstacles">
  <!-- 重映射到 SDFMap 订阅的话题 -->
  <remap from="/dynamic_obstacles/cloud" to="/sdf_map/cloud"/>
</node>
```

## 可视化

### RViz 话题订阅

在 RViz 中添加以下话题：

1. **点云可视化**：
   - Topic: `/dynamic_obstacles/cloud`
   - Type: `PointCloud2`
   - Color: 高度图或强度

2. **立方体标记**：
   - Topic: `/dynamic_obstacles/vis`
   - Type: `MarkerArray`
   - 显示半透明立方体和速度箭头

### 预期效果

- 橙色半透明立方体：障碍物本体
- 绿色箭头：速度向量
- 白色点云：填充的点云（用于 ESDF 转换）

## 参数配置示例

### 高密度小障碍物

```xml
<arg name="obs_num" value="10"/>
<arg name="resolution" value="0.1"/>
<arg name="size_min_x" value="0.3"/>
<arg name="size_max_x" value="0.8"/>
```

### 大型慢速障碍物

```xml
<arg name="obs_num" value="3"/>
<arg name="obs_max_speed" value="0.5"/>
<arg name="size_min_x" value="1.5"/>
<arg name="size_max_x" value="2.5"/>
```

## 技术细节

### 点云生成策略

立方体内部按分辨率（resolution）均匀填充点：

```cpp
for (x in [-half_size, +half_size] step resolution)
  for (y in [-half_size, +half_size] step resolution)
    for (z in [-half_size, +half_size] step resolution)
      添加点 (center + [x, y, z])
```

**优点**：
- 点云密度可控（通过 `resolution` 参数）
- ESDF 转换准确
- 计算效率高

### 边界碰撞处理

```cpp
if (position - half_size < map_min)
  position = map_min + half_size
  velocity = -velocity  // 完美弹性碰撞
```

### 地图边界（与 topo_replan.launch 对齐）

```
X: [-20, +20] m  (40m 宽)
Y: [-10, +10] m  (20m 深)
Z: [  0,  +5] m  ( 5m 高)
```

## 故障排除

### 问题1：点云不显示

**检查**：
```bash
rostopic echo /dynamic_obstacles/cloud
```

**解决**：确保节点正常运行，检查 ROS 话题连接

### 问题2：ESDF 不更新

**原因**：点云话题没有连接到 SDFMap

**解决**：
```xml
<remap from="/dynamic_obstacles/cloud" to="/sdf_map/cloud"/>
```

或者在 `sdf_map.cpp` 中订阅 `/dynamic_obstacles/cloud`

### 问题3：障碍物穿墙

**原因**：边界参数与地图不匹配

**解决**：检查 `map_size_x/y/z` 与 `topo_replan.launch` 一致

## 依赖项

- ROS (Noetic/Melodic)
- PCL (Point Cloud Library)
- Eigen3
- Fast Planner (plan_env 模块)

## 文件结构

```
dynamic_obstacles/
├── CMakeLists.txt
├── package.xml
├── README.md
├── src/
│   └── dynamic_cube_obstacles.cpp
└── launch/
    └── dynamic_cubes.launch
```

## 许可证

BSD License

## 作者

Created for Fast Planner Extension
