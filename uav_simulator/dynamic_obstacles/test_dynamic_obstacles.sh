#!/bin/bash

# 动态障碍物测试脚本
# 使用方法：./test_dynamic_obstacles.sh

echo "======================================"
echo "动态立方体障碍物测试脚本"
echo "======================================"

# 设置工作空间
WS_PATH="/home/zr/Fast_Planner_ws"

# Source 环境
echo "1. 设置 ROS 环境..."
source /opt/ros/noetic/setup.bash
source ${WS_PATH}/devel/setup.bash

# 检查 roscore 是否运行
echo ""
echo "2. 检查 roscore..."
if ! pgrep -x roscore > /dev/null; then
    echo "⚠️  roscore 未运行！"
    echo "请在另一个终端运行: roscore"
    echo ""
    read -p "roscore 启动后按 Enter 继续..."
fi

# 显示包信息
echo ""
echo "3. 包信息:"
rospack find dynamic_obstacles
echo ""

# 显示launch文件
echo "4. 可用的 launch 文件:"
ls -lh ${WS_PATH}/src/Fast-Planner-Extension/uav_simulator/dynamic_obstacles/launch/
echo ""

# 询问测试模式
echo "======================================"
echo "选择测试模式:"
echo "  1) 独立测试 (只启动动态障碍物)"
echo "  2) 查看话题列表"
echo "  3) 查看节点信息"
echo "  4) 退出"
echo "======================================"
read -p "请选择 (1-4): " choice

case $choice in
    1)
        echo ""
        echo "启动动态障碍物节点..."
        echo "提示：在 RViz 中添加以下显示项："
        echo "  - PointCloud2: /dynamic_obstacles/cloud"
        echo "  - MarkerArray: /dynamic_obstacles/vis"
        echo ""
        roslaunch dynamic_obstacles dynamic_cubes.launch
        ;;
    2)
        echo ""
        echo "等待节点启动..."
        sleep 2
        echo "当前话题列表:"
        rostopic list | grep -E "dynamic|obs"
        ;;
    3)
        echo ""
        rosnode list | grep dynamic
        ;;
    4)
        echo "退出测试"
        exit 0
        ;;
    *)
        echo "无效选择"
        exit 1
        ;;
esac
