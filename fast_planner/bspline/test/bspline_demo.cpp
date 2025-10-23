#include <ros/ros.h>
#include <Eigen/Dense>
#include <vector>
#include <iostream>
#include "bspline/non_uniform_bspline.h"

using fast_planner::NonUniformBspline;

int main(int argc, char** argv) {
  ros::init(argc, argv, "bspline_demo");

  // 1) 输入采样点与边界导数约束
  std::vector<Eigen::Vector3d> point_set;
  point_set.push_back(Eigen::Vector3d(0.0, 0.0, 0.0));
  point_set.push_back(Eigen::Vector3d(1.0, 0.5, 0.0));
  point_set.push_back(Eigen::Vector3d(2.0, 1.0, 0.0));
  point_set.push_back(Eigen::Vector3d(3.0, 1.0, 0.0));
  point_set.push_back(Eigen::Vector3d(4.0, 0.7, 0.0));

  // 起/止一阶、二阶导数：(v0, vend, a0, aend)
  std::vector<Eigen::Vector3d> start_end_derivative(4, Eigen::Vector3d::Zero());

  double ts = 0.2;  // 采样时间步，用于参数化与初始均匀结间隔

  // 2) 将采样点参数化成三次均匀B样条控制点
  Eigen::MatrixXd ctrl_pts;
  NonUniformBspline::parameterizeToBspline(ts, point_set, start_end_derivative, ctrl_pts);

  // 3) 构造样条并设置物理限值
  NonUniformBspline traj(ctrl_pts, 3, ts);
  traj.setPhysicalLimits(/*vel*/ 2.0, /*acc*/ 4.0);

  // 4) 可行性检查与时间重分配（最多迭代3次）
  bool feasible = traj.checkFeasibility(true);
  int it = 0;
  while (!feasible && it < 3) {
    traj.reallocateTime(true);
    feasible = traj.checkFeasibility(true);
    ++it;
  }
  double ratio = traj.checkRatio();
  std::cout << "[Check] ratio=" << ratio << " feasible=" << feasible << " iters=" << it << std::endl;

  // 5) 统计长度、平均/最大速度与加速度、跃度
  double length = traj.getLength(0.05);
  double mean_v = 0.0, max_v = 0.0;
  double mean_a = 0.0, max_a = 0.0;
  traj.getMeanAndMaxVel(mean_v, max_v);
  traj.getMeanAndMaxAcc(mean_a, max_a);
  double jerk = traj.getJerk();

  std::cout << "[Stats] length=" << length
            << " mean_v=" << mean_v << " max_v=" << max_v
            << " mean_a=" << mean_a << " max_a=" << max_a
            << " jerk=" << jerk << std::endl;

  // 6) 采样输出轨迹点（相对起点时间）
  double t0, t1;
  traj.getTimeSpan(t0, t1);
  std::cout << "[Span] t0=" << t0 << " t1=" << t1 << " total=" << (t1 - t0) << std::endl;

  for (double t = 0.0; t <= (t1 - t0) + 1e-6; t += 0.2) {
    Eigen::VectorXd p = traj.evaluateDeBoorT(t);
    std::cout << "t=" << t << " p=(" << p(0) << ", " << p(1) << ", " << p(2) << ")" << std::endl;
  }

  return 0;
}

