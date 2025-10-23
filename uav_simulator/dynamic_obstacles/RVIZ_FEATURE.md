# RViz 自动启动功能 - 更新说明

## ✨ 新功能

现在 `dynamic_cubes.launch` 已升级，支持**一键启动自动打开 RViz**！

---

## 🎯 快速使用

### 方法1：自动启动 RViz（推荐）

```bash
cd ~/Fast_Planner_ws
source devel/setup.bash
roslaunch dynamic_obstacles dynamic_cubes.launch
```

**自动执行**:
- ✅ 启动动态障碍物节点
- ✅ 自动打开 RViz
- ✅ 加载预配置（点云+标记已配置好）
- ✅ 开始可视化

---

### 方法2：仅启动节点（不要 RViz）

```bash
roslaunch dynamic_obstacles dynamic_cubes.launch use_rviz:=false
```

---

## 📋 新增参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `use_rviz` | `true` | 是否自动启动 RViz |

---

## 🎨 RViz 预配置内容

自动加载的配置包括：

### 1. 显示项

- **Grid（网格）**: 50x50，帮助定位
- **PointCloud2（点云）**:
  - Topic: `/dynamic_obstacles/cloud`
  - 颜色: 按 Z 轴高度（彩虹色）
  - 大小: 0.05m
  - 样式: 球体
- **MarkerArray（标记）**:
  - Topic: `/dynamic_obstacles/vis`
  - 橙色立方体（障碍物）
  - 绿色箭头（速度）

### 2. 视角设置

- Fixed Frame: `world`
- 距离: 35m
- 俯仰角: ~45度
- 焦点: (0, 0, 2.5)

---

## 🔧 修改的文件

### 1. 新增文件

```
rviz/dynamic_obstacles.rviz    # RViz 配置文件
```

### 2. 修改文件

`launch/dynamic_cubes.launch` 新增：

```xml
<!-- 可视化选项 -->
<arg name="use_rviz" default="true"/>

<!-- RViz 可视化（可选） -->
<group if="$(arg use_rviz)">
  <node pkg="rviz" type="rviz" name="rviz"
        args="-d $(find dynamic_obstacles)/rviz/dynamic_obstacles.rviz"
        output="screen"/>
</group>
```

---

## 📊 使用示例

### 示例1：默认启动

```bash
roslaunch dynamic_obstacles dynamic_cubes.launch
```

**结果**: 5个障碍物 + RViz 自动打开

---

### 示例2：10个障碍物 + RViz

```bash
roslaunch dynamic_obstacles dynamic_cubes.launch obs_num:=10
```

---

### 示例3：仅后台节点（调试用）

```bash
roslaunch dynamic_obstacles dynamic_cubes.launch use_rviz:=false
```

然后手动打开 RViz:
```bash
rviz -d $(rospack find dynamic_obstacles)/rviz/dynamic_obstacles.rviz
```

---

### 示例4：低速大障碍物 + RViz

```bash
roslaunch dynamic_obstacles dynamic_cubes.launch \
  obs_num:=3 \
  obs_max_speed:=0.5 \
  size_max_x:=2.5 \
  size_max_z:=3.0
```

---

## 🎓 RViz 操作技巧

### 视角控制

- **旋转**: 鼠标左键拖拽
- **缩放**: 鼠标滚轮
- **平移**: Shift + 左键 或 鼠标中键

### 显示调整

- **隐藏某项**: 左侧 Displays → 取消勾选
- **调整颜色**: 点击显示项 → 修改 Color Transformer
- **改变大小**: PointCloud2 → Size (m)

### 保存配置

如果你调整了显示设置：

1. `File` → `Save Config As...`
2. 保存到 `rviz/my_config.rviz`
3. 修改 launch 文件指向新配置

---

## 🐛 故障排除

### RViz 不自动打开

**检查**:
```bash
# 1. 确认 RViz 已安装
rospack find rviz

# 2. 确认配置文件存在
ls $(rospack find dynamic_obstacles)/rviz/dynamic_obstacles.rviz

# 3. 手动测试 RViz
rviz
```

**解决**: 如果 rviz 未安装
```bash
sudo apt-get install ros-noetic-rviz
```

---

### RViz 中看不到障碍物

**检查清单**:
- [ ] Fixed Frame 是否为 `world`
- [ ] PointCloud2 是否启用（勾选）
- [ ] 话题名称是否正确
- [ ] 节点是否正在发布数据

**验证发布**:
```bash
rostopic hz /dynamic_obstacles/cloud  # 应该 ~10 Hz
rostopic echo /dynamic_obstacles/cloud/width  # 应该有点数
```

---

### RViz 启动后立即崩溃

**查看日志**:
```bash
cat ~/.ros/log/latest/rviz-*.log
```

**常见原因**:
- 配置文件损坏
- OpenGL 驱动问题

**临时解决**:
```bash
# 使用默认配置
roslaunch dynamic_obstacles dynamic_cubes.launch use_rviz:=false
rviz  # 不加载配置
```

---

## 🎯 与旧版本的区别

| 特性 | 旧版本 | 新版本（当前）|
|------|--------|-------------|
| 启动方式 | 需要手动开 RViz | 自动打开 ✨ |
| RViz 配置 | 手动添加显示项 | 预配置好 ✨ |
| 可选性 | - | `use_rviz` 参数 ✨ |
| 视角 | 随机 | 预设最佳角度 ✨ |

---

## 📝 向后兼容

**旧的使用方式仍然有效**:

```bash
# 方式1：仍可手动启动
roslaunch dynamic_obstacles dynamic_cubes.launch use_rviz:=false
rviz

# 方式2：在 topo_replan 中集成时不影响
roslaunch plan_manage topo_replan.launch
```

---

## 🎉 总结

现在只需一条命令：

```bash
roslaunch dynamic_obstacles dynamic_cubes.launch
```

就能获得：
- ✅ 动态障碍物节点运行
- ✅ RViz 自动打开
- ✅ 完美配置的可视化
- ✅ 即刻查看效果

**享受一键启动的便利！** 🚀

---

**更新日期**: 2025-10-19
**版本**: 1.1.0
