# topo_replan with Dynamic Obstacles - 使用说明

## 问题已解决 ✅

您遇到的问题：**启动 `topo_replan_with_dynamic_obs.launch` 时 RViz 中看不到动态障碍物**

已经成功解决！

---

## 解决方案

### 1. 修改了 `traj.rviz` 配置文件

在 `/home/zr/Fast_Planner_ws/src/Fast-Planner-Extension/fast_planner/plan_manage/config/traj.rviz` 中添加了新的显示组 `DynamicObstacles`，包含：

- **dynamic_cloud (PointCloud2)**
  - 话题：`/dynamic_obstacles/cloud`
  - 颜色：按 Z 轴彩虹色
  - 大小：0.05m 球体
  - 状态：**已启用**

- **dynamic_markers (MarkerArray)**
  - 话题：`/dynamic_obstacles/vis`
  - 包含：橙色立方体 + 绿色速度箭头
  - 状态：**已启用**

### 2. 修改了 `topo_replan_with_dynamic_obs.launch`

添加了 RViz 启动节点：

```xml
<!-- ========== RViz 可视化 ========== -->
<node pkg="rviz" type="rviz" name="rviz"
      args="-d $(find plan_manage)/config/traj.rviz"
      output="screen"/>
```

---

## 使用方法

### 一键启动（推荐）

```bash
cd ~/Fast_Planner_ws
source devel/setup.bash
roslaunch dynamic_obstacles topo_replan_with_dynamic_obs.launch
```

### 启动后您会看到：

1. **RViz 窗口自动打开**
2. **显示所有 topo_replan 的可视化内容**：
   - LocalMap（本地地图）
   - Planning（规划轨迹）
   - Mapping（静态地图）
   - Simulation（机器人）
   - **DynamicObstacles（动态障碍物）** ← 新增！

3. **动态障碍物可视化**：
   - 🟧 橙色半透明立方体（障碍物本体）
   - 🟢 绿色箭头（速度方向）
   - ⚪ 白色点云（内部填充）

---

## RViz 显示组说明

在 RViz 左侧的 Displays 面板中，您会看到：

```
Displays
├── Axes
├── Grid
├── LocalMap
│   ├── local_map
│   ├── local_esdf
│   ├── local_range
│   └── depth_cloud
├── Planning
│   ├── bspline_traj
│   ├── topo_path
│   ├── executed_traj
│   └── position_cmd
├── Mapping
│   ├── simulation_map
│   └── real_map
├── Simulation
│   └── robot
├── Estimation
│   ├── vins-camera
│   ├── camera
│   ├── uart_odom
│   └── opti_track
└── DynamicObstacles ← 新增！
    ├── dynamic_cloud ✓
    └── dynamic_markers ✓
```

---

## 自定义参数

### 调整动态障碍物数量和速度

```bash
roslaunch dynamic_obstacles topo_replan_with_dynamic_obs.launch \
  dynamic_obs_num:=10 \
  dynamic_obs_speed:=1.5
```

### 禁用动态障碍物

```bash
roslaunch dynamic_obstacles topo_replan_with_dynamic_obs.launch \
  enable_dynamic_obs:=false
```

---

## RViz 操作技巧

### 查看动态障碍物

1. 在左侧 Displays 面板中找到 `DynamicObstacles` 组
2. 确保 `dynamic_cloud` 和 `dynamic_markers` 都已勾选（✓）
3. 调整视角：鼠标左键拖拽旋转，滚轮缩放

### 如果看不到动态障碍物

**检查清单**：
- [ ] `DynamicObstacles` 组是否展开？
- [ ] `dynamic_cloud` 是否已勾选（Enabled）？
- [ ] `dynamic_markers` 是否已勾选（Enabled）？
- [ ] Fixed Frame 是否为 `world`？

**验证话题发布**：
```bash
# 新开一个终端
rostopic hz /dynamic_obstacles/cloud  # 应该显示 ~10 Hz
rostopic hz /dynamic_obstacles/vis    # 应该显示 ~10 Hz
```

### 调整显示效果

**点云大小**：
1. 展开 `DynamicObstacles` → `dynamic_cloud`
2. 修改 `Size (m)` 参数（默认 0.05）

**点云颜色**：
1. 展开 `Color Transformer`
2. 可选：`AxisColor`（彩虹色）、`FlatColor`（单色）、`Intensity`

**隐藏速度箭头**：
1. 取消勾选 `dynamic_markers`

---

## 话题说明

### 发布的话题

| 话题 | 类型 | 频率 | 内容 |
|------|------|------|------|
| `/dynamic_obstacles/cloud` | `sensor_msgs/PointCloud2` | 10 Hz | 动态障碍物点云 |
| `/dynamic_obstacles/vis` | `visualization_msgs/MarkerArray` | 10 Hz | 可视化标记（立方体+箭头）|

### 订阅说明

如果您想让 `sdf_map` 订阅动态障碍物点云：

1. 修改 `sdf_map.cpp`，添加订阅：
```cpp
dynamic_cloud_sub_ = node_.subscribe<sensor_msgs::PointCloud2>(
    "/dynamic_obstacles/cloud", 10, &SDFMap::cloudCallback, this);
```

2. 在 `cloudCallback` 中处理动态障碍物点云

---

## 测试验证

### 终端输出检查

启动后应该看到：

```
[ INFO] ✓ Initialized 5 cube obstacles
[ INFO] === Dynamic Cube Obstacles Publisher ===
[ INFO]   Number of obstacles: 5
[ INFO]   Max speed: 1.00 m/s
[ INFO]   Update frequency: 10.0 Hz
[ INFO]   Point cloud resolution: 0.20 m
[ INFO]   Map boundary: [-20.0, -10.0, 0.0] to [20.0, 10.0, 5.0]
```

### RViz 窗口检查

应该看到：
- ✓ RViz 窗口自动打开
- ✓ 左侧 Displays 面板显示所有组
- ✓ DynamicObstacles 组中有两个显示项
- ✓ 3D 视图中可以看到动态立方体移动

---

## 故障排除

### 问题：RViz 打开但看不到动态障碍物

**原因**：显示项可能被禁用或隐藏

**解决**：
1. 在 RViz 左侧面板中，找到 `DynamicObstacles` 组
2. 点击旁边的三角形展开
3. 确保 `dynamic_cloud` 和 `dynamic_markers` 都有勾选标记

### 问题：RViz 中只看到静态地图，没有动态内容

**原因**：dynamic_cube_obstacles 节点可能未启动

**验证**：
```bash
rosnode list | grep dynamic
# 应该显示：/dynamic_cube_obstacles
```

**解决**：
```bash
# 检查节点状态
rosnode info /dynamic_cube_obstacles

# 如果没有运行，检查 enable_dynamic_obs 参数是否为 true
```

### 问题：点云发布但 RViz 不显示

**原因**：可能的 TF 坐标系问题

**检查**：
```bash
# 检查 Fixed Frame
rostopic echo /dynamic_obstacles/cloud/header/frame_id -n 1
# 应该输出：world
```

**解决**：
在 RViz 全局选项中，确保 Fixed Frame 设置为 `world`

---

## 性能优化

如果系统运行卡顿：

### 减少障碍物数量
```bash
roslaunch dynamic_obstacles topo_replan_with_dynamic_obs.launch \
  dynamic_obs_num:=3
```

### 降低点云分辨率
在 launch 文件中修改（需要编辑文件）：
```xml
<param name="resolution" value="0.3"/>  <!-- 默认 0.2 -->
```

### 在 RViz 中优化
- 减小点云大小（Size: 0.03）
- 降低帧率（Global Options → Frame Rate: 15）
- 禁用不需要的显示项

---

## 与其他 Launch 文件的区别

| Launch 文件 | RViz | 动态障碍物 | 静态地图 | 规划系统 |
|------------|------|-----------|---------|---------|
| `dynamic_cubes.launch` | ✓ | ✓ | ✗ | ✗ |
| `topo_replan.launch` | ✗ | ✗ | ✓ | ✓ |
| `topo_replan_with_dynamic_obs.launch` | ✓ | ✓ | ✓ | ✓ |

---

## 总结

现在您可以：

✅ 一键启动完整的 topo_replan 系统 + 动态障碍物
✅ 在 RViz 中同时看到静态地图和动态障碍物
✅ 观察规划器如何应对动态环境
✅ 调整障碍物参数进行测试

**享受完整的可视化体验！** 🚀

---

**最后更新**：2025-10-19
**版本**：1.0.0
