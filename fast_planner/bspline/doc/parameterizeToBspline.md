<!--
 * @Author: xzr && 1841953204@qq.com
 * @Date: 2025-10-23 11:16:30
 * @LastEditors: xzr && 1841953204@qq.com
 * @LastEditTime: 2025-10-23 11:17:35
 * @FilePath: /Fast_Planner_ws/src/Fast-Planner-Extension/fast_planner/bspline/guide.md
 * @Description: 
 * 
 * Copyright (c) 2025 by 1841953204@qq.com, All Rights Reserved. 
-->
# B 样条参数化示例
 - 采样点数 K=3（三个位置采样），时间步长 ts=0.5。
  - 采样点：S0=(0,0,0), S1=(1,2,0), S2=(2,3,0)。
  - 边界导数（起/止速度与加速度）：v0=vend=a0=aend=(0,0,0)。
  - 未知控制点向量（以 x 轴为例）：x = [P0, P1, P2, P3, P4]^T，长度 K+2=5。
  - 系数矩阵 A 尺寸 (K+4) × (K+2) = 7 × 5；右端向量 b_x 长度 7。

  A 矩阵（数值展开）

  - 位置约束权重 (1/6)[1,4,1] = [0.1667, 0.6667, 0.1667]
  - 速度约束权重 (1/(2ts))[-1,0,1] = [-1, 0, 1]（因为 ts=0.5）
  - 加速度约束权重 (1/ts^2)[1,-2,1] = [4, -8, 4]

  A 的 7 行按规则填充如下（每行 5 列，未列出者为 0）：

  - 第 0 行（位置 S0）：[0.1667, 0.6667, 0.1667, 0,      0     ]
  - 第 1 行（位置 S1）：[0,      0.1667, 0.6667, 0.1667, 0     ]
  - 第 2 行（位置 S2）：[0,      0,      0.1667, 0.6667, 0.1667]
  - 第 3 行（起始速度 v0）：[-1,     0,      1,      0,      0     ]
  - 第 4 行（结束速度 vend）：[0,      0,     -1,      0,      1     ]
  - 第 5 行（起始加速度 a0）：[4,     -8,      4,      0,      0     ]
  - 第 6 行（结束加速度 aend）：[0,      0,      4,     -8,      4     ]

  对应代码填充参考：

  - 位置：src/Fast-Planner-Extension/fast_planner/bspline/src/non_uniform_bspline.cpp:360
  - 速度：src/Fast-Planner-Extension/fast_planner/bspline/src/non_uniform_bspline.cpp:364, src/.../non_uniform_bspline.cpp:365
  - 加速度：src/Fast-Planner-Extension/fast_planner/bspline/src/non_uniform_bspline.cpp:367, src/.../non_uniform_bspline.cpp:368

  b 向量与未知 x

  - x 轴右端向量 b_x = [S0.x, S1.x, S2.x, v0.x, vend.x, a0.x, aend.x]^T = [0, 1, 2, 0, 0, 0, 0]^T
  - y 轴右端向量 b_y = [0, 2, 3, 0, 0, 0, 0]^T
  - z 轴右端向量 b_z = [0, 0, 0, 0, 0, 0, 0]^T
  - 未知控制点向量（逐轴）：
      - x = [P0.x, P1.x, P2.x, P3.x, P4.x]^T
      - y = [P0.y, P1.y, P2.y, P3.y, P4.y]^T
      - z = [P0.z, P1.z, P2.z, P3.z, P4.z]^T

  线性系统与求解

  - 每个轴独立解：A * px = b_x, A * py = b_y, A * pz = b_z
  - 使用带列主元的 QR 分解求最小二乘解：
      - px = A.colPivHouseholderQr().solve(b_x)
      - py = A.colPivHouseholderQr().solve(b_y)
      - pz = A.colPivHouseholderQr().solve(b_z)
  - 合并得到控制点矩阵 ctrl_pts（形状 (K+2)×3 = 5×3）：
      - 第 i 行为控制点 Pi = (px[i], py[i], pz[i])

  一般化说明

  - 对任意 K 和 ts，A 的前三 K 行是位置约束，以 [i, i+1, i+2] 三列填 (1/6)[1,4,1]。
  - 速度行索引为 K 和 K+1，分别填充列 [0..2] 与 [K-1..K+1] 的 (1/(2ts))[-1,0,1]。
  - 加速度行索引为 K+2 和 K+3，分别填充列 [0..2] 与 [K-1..K+1] 的 (1/ts^2)[1,-2,1]。
  - 同一个 A 同时用于 x/y/z 三个轴，b 由对应轴的采样与边界导数组成。