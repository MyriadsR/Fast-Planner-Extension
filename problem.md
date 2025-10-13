<!--
 * @Author: xzr && 1841953204@qq.com
 * @Date: 2025-10-13 20:25:55
 * @LastEditors: xzr && 1841953204@qq.com
 * @LastEditTime: 2025-10-13 22:06:27
 * @FilePath: /Fast_Planner_ws/src/Fast-Planner-Extension/problem.md
 * @Description: problem record
 * 
 * Copyright (c) 2025 by 1841953204@qq.com, All Rights Reserved. 
-->
# 目前存在问题
1. 动态障碍物的碰撞检测是假设障碍物匀速运动，实际中障碍物可能会有加速度，`isColliding`函数
2. 采样的参数应该如何设置是最佳的，包括采样点、采样范围等
3. 边的安全时间窗口计算时，考虑无人机是沿着该边匀速运动，实际肯定不是匀速，这会导致计算出的安全时间窗口不准确，即 `computeObstacleCollisionWindow`函数
4. 如何加速边的安全时间窗口计算