# Test PRM with Dynamic Obstacles

本目录包含用于测试 PRM（Probabilistic Roadmap）算法的工具，支持动态和静态障碍物环境。

## 功能特性

- ✅ 动态圆球障碍物生成和可视化
- ✅ 静态障碍物支持
- ✅ 基于 UTVD（Unobstructed Time-Varying Domain）的时空拓扑检测
- ✅ RViz 实时可视化
- ✅ 完整的 PRM 图生成和路径规划

## 文件结构

```
test_prm/
├── src/
│   ├── test_prm_graph.cpp              # PRM 图测试节点
│   └── publish_dynamic_obstacles.cpp   # 动态障碍物发布节点
├── launch/
│   ├── test_graph.launch              # 主启动文件
│   ├── prm_params.yaml                # PRM 参数配置
│   ├── dynamic_obs_params.yaml        # 动态障碍物参数
│   └── prm_test.rviz                  # RViz 配置
└── CMakeLists.txt
```

## 编译

```bash
cd ~/Fast_Planner_ws
catkin_make --pkg test_prm
source devel/setup.bash
```

## 运行

### 方法 1：使用 launch 文件（推荐）

启动完整系统（包括静态/动态障碍物、PRM 图测试和 RViz）：

```bash
roslaunch test_prm test_graph.launch
```

### 方法 2：分步启动

1. 启动静态障碍物发布器：
```bash
roslaunch static_obs fake_static_obs_sim.launch
```

2. 启动动态障碍物发布器：
```bash
rosrun test_prm dynamic_obstacle_publisher __ns:=test_prm
```

3. 启动 PRM 图测试节点：
```bash
rosrun test_prm test_prm_graph_node __ns:=test_prm
```

4. 启动 RViz：
```bash
rviz -d $(rospack find test_prm)/launch/prm_test.rviz
```

## 使用方法

1. **启动系统**：运行 `roslaunch test_prm test_graph.launch`

2. **查看动态障碍物**：
   - 在 RViz 中查看 `/dyn_obstacles_vis` 话题
   - 橙色球体表示障碍物
   - 绿色箭头表示速度方向

3. **设置目标点**：
   - 在 RViz 中使用 "2D Nav Goal" 工具点击设置目标点
   - 系统会自动创建 PRM 图并可视化

4. **查看 PRM 图**：
   - 蓝色大球：起点（固定在 [0, 0, 1]）
   - 紫色大球：终点（通过 2D Nav Goal 设置）
   - 蓝色小球：Guard 节点
   - 绿色小球：Connector 节点
   - 黄色线段：图的边

## 参数配置

### PRM 参数 (`prm_params.yaml`)

```yaml
robot_speed: 1.0           # 机器人速度 (m/s)
max_sample_time: 1.0       # 最大采样时间 (秒)
max_sample_num: 2000       # 最大采样数量
sample_inflate_x: [3.0, 3.0, 1.5]  # 采样区域膨胀 [x, y, z]
safety_margin: 0.3         # 安全裕度 (m)
```

### 动态障碍物参数 (`dynamic_obs_params.yaml`)

```yaml
obs_num: 8                 # 障碍物数量
obs_max_speed: 1.5         # 最大速度 (m/s)
freq: 30                   # 发布频率 (Hz)
gbbox_min: [-10.0, -10.0, 0.5]      # 边界框最小点
gbbox_size: [20.0, 20.0, 2.5]       # 边界框尺寸
size_min: [0.4, 0.4, 0.4]           # 最小直径 (m)
size_max: [1.0, 1.0, 1.0]           # 最大直径 (m)
```

## ROS 话题

### 发布话题

| 话题 | 类型 | 描述 |
|------|------|------|
| `/obj_states` | `obj_state_msgs::ObjectsStates` | 动态障碍物状态（位置、速度、尺寸） |
| `/dyn_obstacles_vis` | `visualization_msgs::MarkerArray` | 动态障碍物可视化 |
| `/static_obj_states` | `obj_state_msgs::ObjectsStates` | 静态障碍物状态 |
| `/prm_graph` | `visualization_msgs::MarkerArray` | PRM 图边可视化 |
| `/prm_nodes` | `visualization_msgs::MarkerArray` | PRM 图节点可视化 |

### 订阅话题

| 话题 | 类型 | 描述 |
|------|------|------|
| `/move_base_simple/goal` | `geometry_msgs::PoseStamped` | 目标点设置 |

## 核心算法

### UTVD 拓扑等价性检测

系统使用 UTVD（Unobstructed Time-Varying Domain）理论来判断两条路径在时空域中是否拓扑等价：

1. **时间窗口计算**：计算每条边的安全通行时间窗口
2. **时间走廊交集**：计算多条边组成路径的时间走廊交集
3. **UTVD 检查**：
   - 粗检查：12×6 采样点
   - 精细检查：48×24 采样点
4. **碰撞检测**：在时空域中检查路径是否与动态障碍物碰撞

### 动态障碍物运动模型

- 恒速直线运动
- 边界反弹
- 碰撞时间区间计算：使用二次方程求解球体轨迹与位置点的碰撞时间

## 调试技巧

### 查看日志信息

```bash
# 查看 PRM 图创建信息
rostopic echo /rosout | grep "test_prm_graph"

# 查看动态障碍物数量
rostopic echo /rosout | grep "dynamic_obstacle"
```

### 检查话题发布频率

```bash
# 检查动态障碍物发布频率
rostopic hz /obj_states

# 检查可视化发布频率
rostopic hz /dyn_obstacles_vis
```

### 常见问题

1. **没有显示动态障碍物**
   - 检查话题是否发布：`rostopic list | grep obj_states`
   - 检查 RViz 中是否添加了 MarkerArray 显示

2. **PRM 图不生成**
   - 确保已设置目标点（使用 2D Nav Goal）
   - 检查静态障碍物是否发布

3. **编译错误**
   - 确保安装了所有依赖：`rosdep install --from-paths src --ignore-src -r -y`
   - 清理并重新编译：`catkin_make clean && catkin_make`

## 扩展功能

### 修改障碍物运动模式

编辑 `src/publish_dynamic_obstacles.cpp` 中的 `updateObstacles()` 函数，可以实现：
- 圆周运动
- 正弦波运动
- 随机游走
- 自定义轨迹

### 调整采样策略

编辑 `prm_params.yaml` 调整：
- `max_sample_num`：增加采样数量可获得更密集的图
- `sample_inflate_x`：调整采样区域大小
- `robot_speed`：影响时间窗口计算

## 参考资料

- [T-PRM 论文](https://arxiv.org/abs/2204.02560)
- [Fast-Planner 项目](https://github.com/HKUST-Aerial-Robotics/Fast-Planner)
- UTVD 理论：Unobstructed Time-Varying Domain for trajectory planning

## 作者

基于 Fast-Planner 和 T-PRM 开发

---

**提示**：建议在 RViz 中添加以下显示项以获得最佳可视化效果：
- MarkerArray: `/dyn_obstacles_vis`（动态障碍物）
- MarkerArray: `/prm_graph`（PRM 图边）
- MarkerArray: `/prm_nodes`（PRM 图节点）
- MarkerArray: `/static_obstacles_vis`（静态障碍物）
