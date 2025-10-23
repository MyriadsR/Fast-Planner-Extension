<!--
 * @Author: xzr && 1841953204@qq.com
 * @Date: 2025-10-23 16:36:16
 * @LastEditors: xzr && 1841953204@qq.com
 * @LastEditTime: 2025-10-23 16:36:34
 * @FilePath: /Fast_Planner_ws/src/Fast-Planner-Extension/fast_planner/bspline/doc/reallocateTime.md
 * @Description: 
 * 
 * Copyright (c) 2025 by 1841953204@qq.com, All Rights Reserved. 
-->
# 示例：速度超限
  - 设定：三次样条 p=3，结间隔 interval=0.5，控制点数 N=6（P0..P5），速度上限 limit_vel=2.0。
  - 初始结向量（开放均匀）：u = [-1.5,-1.0,-0.5,0.0,0.5,1.0,1.5,2.0,2.5,3.0]（src/Fast-Planner-Extension/fast_planner/bspline/src/
    non_uniform_bspline.cpp:38）。
  - 假设在 x 轴上 P1 - P0 = 2.5（很大），其余差值较小。速度段 i=0：
      - time_ori = u(i+p+1) - u(i+1) = u(4)-u(1) = 0.5 - (-1.0) = 1.5
      - vel_0 = p * (P1 - P0) / time_ori = 3 * 2.5 / 1.5 = 5.0 > 2.0
      - 放大比例（截断到单次上限）：ratio = min( max_vel/limit_vel + 1e-4, limit_ratio ) = min(2.5+1e-4, 1.1) = 1.1
      - 新时间与增量：time_new = 1.5 * 1.1 = 1.65，delta_t = 0.15，t_inc = delta_t / p = 0.05（src/Fast-Planner-Extension/fast_planner/bspline/src/
        non_uniform_bspline.cpp:245）
  - 结向量更新（两步，保持单调与局部均匀伸长，src/.../non_uniform_bspline.cpp:253,260）：
      - 局部内部结（均匀分摊）：j= i+2..i+p+1 = 2..4
          - u(2) += (2-1)*0.05 = +0.05 → -0.5 → -0.45
          - u(3) += (3-1)*0.05 = +0.10 → 0.0 → 0.10
          - u(4) += (4-1)*0.05 = +0.15 → 0.5 → 0.65
      - 后续全部结（整体平移）：j= i+p+2..end = 5..9，每个 +0.15
          - u(5): 1.0 → 1.15，u(6): 1.5 → 1.65，…，u(9): 3.0 → 3.15
  - 结果：time_ori 被放大到 time_new，该段速度按比例降低（近似除以 ratio），其后的时间轴整体后移保持顺序。

  示例：加速度超限

  - 设定：同上 p=3, interval=0.5, N=6，加速度上限 limit_acc=4.0。用独立例子（不叠加上一例更新）便于演示。
  - 控制点在 x 轴上设为：P2=1.0, P3=3.0, P4=3.2（使相邻差分变化剧烈），考虑段 i=2：
      - 差分与时间差（均为 1.5）：(P4-P3)/(u(7)-u(4)) = 0.2/1.5；(P3-P2)/(u(6)-u(3)) = 2.0/1.5
      - “中间跨度”：u(i+p+1)-u(i+2) = u(6)-u(4) = 1.0
      - 加速度估计（src/Fast-Planner-Extension/fast_planner/bspline/src/non_uniform_bspline.cpp:269）：
          - acc_2 = p(p-1)[term1 - term2]/(u(6)-u(4)) = 3*2*(0.2/1.5 - 2.0/1.5)/1.0 ≈ 6*(-1.8/1.5) ≈ -7.2
          - 模长 |acc_2| = 7.2 > 4.0
      - 放大比例（平方根关系，截断到上限）：ratio = min( sqrt(max_acc/limit_acc) + 1e-4, limit_ratio ) = min( sqrt(7.2/4)+1e-4, 1.1 ) ≈ 1.1
      - 新时间与增量：time_new = 1.0 * 1.1 = 1.1，delta_t = 0.1，t_inc = delta_t/(p-1) = 0.05（src/.../non_uniform_bspline.cpp:285,289）
  - 结向量更新（特例分配，靠近起点时采用固定索引，src/.../non_uniform_bspline.cpp:294）：
      - 因为 i==2 命中特例分支，更新：
          - 局部内部结：j=2..5，u(j) += (j-1)*t_inc
              - u(2): -0.5 → -0.45，u(3): 0.0 → 0.10，u(4): 0.5 → 0.65，u(5): 1.0 → 1.20
          - 后续全部结：j>=6，u(j) += 4*t_inc = +0.20
              - u(6): 1.5 → 1.70，u(7): 2.0 → 2.20，…，u(9): 3.0 → 3.20
  - 结果：相关差分的时间尺度被按 ratio 放大，加速度近似按 1/ratio^2 降低；用固定索引避免破坏开放均匀样条的端部性质。

  要点总结

  - 速度用“一阶差分/时间差”，加速度用“二阶差分/时间差组合”，分别与时间尺度成 1/ratio 与 1/ratio^2 的关系。
  - 更新策略：在目标区段的内部结线性分摊增量，让该区段整体变宽；其后的结统一平移，保持结向量非递减。
  - 单次调整幅度受 limit_ratio 限制（默认 1.1），推荐循环：先 checkFeasibility()，若不满足则 reallocateTime()，再检查，直至满足或达迭代上限。