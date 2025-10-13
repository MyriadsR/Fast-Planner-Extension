/*
 * @Author: Test PRM Dynamic Scene
 * @Description: 创建简单的动态场景，测试createGraph对动态障碍物和时间同伦拓扑的处理
 */

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tprm/obstacle_impl.h>

#include <iostream>
#include <vector>
#include <list>
#include <memory>
#include <random>
#include <Eigen/Eigen>

/* ---------- GraphNode definition ---------- */
class GraphNode {
public:
  enum NODE_TYPE { Guard = 1, Connector = 2 };
  enum NODE_STATE { NEW = 1, CLOSE = 2, OPEN = 3 };

  GraphNode() {}
  GraphNode(Eigen::Vector3d pos, NODE_TYPE type, int id) {
    pos_ = pos;
    type_ = type;
    state_ = NEW;
    id_ = id;
  }
  ~GraphNode() {}

  std::vector<std::shared_ptr<GraphNode>> neighbors_;
  Eigen::Vector3d pos_;
  NODE_TYPE type_;
  NODE_STATE state_;
  int id_;

  typedef std::shared_ptr<GraphNode> Ptr;
};

/* ---------- 动态场景测试类 ---------- */
class DynamicSceneTester {
private:
  ros::NodeHandle nh_;
  ros::Publisher graph_vis_pub_;
  ros::Publisher node_vis_pub_;
  ros::Publisher obs_vis_pub_;

  // PRM参数
  std::default_random_engine eng_;
  std::random_device rd_;
  std::uniform_real_distribution<double> rand_pos_;

  Eigen::Vector3d start_pos_;
  Eigen::Vector3d goal_pos_;
  Eigen::Vector3d sample_r_;
  Eigen::Vector3d translation_;
  Eigen::Matrix3d rotation_;
  Eigen::Vector3d sample_inflate_;

  double line_step_;
  double robot_speed_;
  double max_sample_time_;
  int max_sample_num_;
  double safety_margin_;
  double max_prediction_time_;

  ros::Time start_time_;  // 记录开始时间，用于动态障碍物动画

  std::list<GraphNode::Ptr> graph_;
  std::vector<std::shared_ptr<tprm::StaticSphereObstacle>> sta_obstacles_;
  std::vector<std::shared_ptr<tprm::DynamicSphereObstacle>> dyn_obstacles_;

  // 存储节点的安全时间区间
  std::map<int, std::vector<std::pair<double, double>>> safety_data_;
  std::vector<std::pair<double, double>> safe_edge_windows_1_;
  std::vector<std::pair<double, double>> safe_edge_windows_2_;
  std::vector<std::pair<double, double>> safe_edge_windows_3_;
  std::vector<std::pair<double, double>> safe_edge_windows_4_;

public:
  DynamicSceneTester(ros::NodeHandle& nh) : nh_(nh) {
    // 初始化发布器
    graph_vis_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/prm_graph", 10);
    node_vis_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/prm_nodes", 10);
    obs_vis_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/dynamic_obstacles_viz", 10);

    // 初始化随机数生成器
    eng_ = std::default_random_engine(rd_());
    rand_pos_ = std::uniform_real_distribution<double>(-1.0, 1.0);

    // 读取参数
    nh_.param("robot_speed", robot_speed_, 1.0);
    nh_.param("max_sample_time", max_sample_time_, 1.0);
    nh_.param("max_sample_num", max_sample_num_, 2000);
    nh_.param("safety_margin", safety_margin_, 0.3);

    std::vector<double> inflate;
    nh_.param("sample_inflate_x", inflate, std::vector<double>());
    if (inflate.size() >= 3) {
      sample_inflate_ = Eigen::Vector3d(inflate[0], inflate[1], inflate[2]);
    } else {
      sample_inflate_ = Eigen::Vector3d(2.0, 2.0, 1.0);
    }

    line_step_ = 0.5 * robot_speed_;
    max_prediction_time_ = 5.0;

    // 设置起点和终点
    start_pos_ = Eigen::Vector3d(-5.0, 0.0, 1.0);
    goal_pos_ = Eigen::Vector3d(5.0, 0.0, 1.0);

    ROS_INFO("Dynamic Scene Tester initialized!");
  }

  /**
   * @brief 创建简单的动态场景
   * 场景包含：
   * 1. 两个静态障碍物（作为环境边界）
   * 2. 两个动态障碍物，在起点和终点之间交叉移动，形成时变拓扑
   */
  void createSimpleDynamicScene() {
    ROS_INFO("\n=== Creating Simple Dynamic Scene ===");

    // 场景1：交叉路径的动态障碍物
    // 障碍物1：Y 轴速度
    Eigen::Vector3d dyn_pos_1(0.0, 3.0, 1.0);
    Eigen::Vector3d dyn_vel_1(0.0, -0.8, 0.0);  
    double dyn_radius_1 = 0.5 + safety_margin_;

    auto dyn_obs_1 = std::make_shared<tprm::DynamicSphereObstacle>(dyn_pos_1, dyn_vel_1, dyn_radius_1);
    dyn_obstacles_.push_back(dyn_obs_1);

    // 障碍物2：Y 轴速度
    Eigen::Vector3d dyn_pos_2(-2.0, -3.0, 1.0);
    Eigen::Vector3d dyn_vel_2(0.0, 0.6, 0.0);   
    double dyn_radius_2 = 0.5 + safety_margin_;

    auto dyn_obs_2 = std::make_shared<tprm::DynamicSphereObstacle>(dyn_pos_2, dyn_vel_2, dyn_radius_2);
    dyn_obstacles_.push_back(dyn_obs_2);

    // 障碍物3：X 轴速度
    Eigen::Vector3d dyn_pos_3(-3.0, 1.5, 1.0);
    Eigen::Vector3d dyn_vel_3(0.5, 0.0, 0.0);
    double dyn_radius_3 = 0.4 + safety_margin_;

    auto dyn_obs_3 = std::make_shared<tprm::DynamicSphereObstacle>(dyn_pos_3, dyn_vel_3, dyn_radius_3);
    dyn_obstacles_.push_back(dyn_obs_3);

    // 障碍物4：Y 轴速度
    Eigen::Vector3d dyn_pos_4(2.0, -3.0, 1.0);
    Eigen::Vector3d dyn_vel_4(0.0, 0.6, 0.0);   
    double dyn_radius_4 = 0.5 + safety_margin_;

    auto dyn_obs_4 = std::make_shared<tprm::DynamicSphereObstacle>(dyn_pos_4, dyn_vel_4, dyn_radius_4);
    dyn_obstacles_.push_back(dyn_obs_4);
    
    // // 障碍物5：Z 轴速度
    // Eigen::Vector3d dyn_pos_5(0.0, 0.0, 1.0);
    // Eigen::Vector3d dyn_vel_5(0.0, 0.0, 0.6);   
    // double dyn_radius_5 = 0.5 + safety_margin_;

    // auto dyn_obs_5 = std::make_shared<tprm::DynamicSphereObstacle>(dyn_pos_5, dyn_vel_5, dyn_radius_5);
    // dyn_obstacles_.push_back(dyn_obs_5);

    // 添加少量静态障碍物（可选）
    Eigen::Vector3d sta_pos_1(2.0, 2.5, 1.0);
    auto sta_obs_1 = std::make_shared<tprm::StaticSphereObstacle>(sta_pos_1, 0.4);
    sta_obstacles_.push_back(sta_obs_1);

    Eigen::Vector3d sta_pos_2(-2.0, -2.5, 1.0);
    auto sta_obs_2 = std::make_shared<tprm::StaticSphereObstacle>(sta_pos_2, 0.4);
    sta_obstacles_.push_back(sta_obs_2);

    Eigen::Vector3d sta_pos_3(0.0, 0.0, 1.0);
    auto sta_obs_3 = std::make_shared<tprm::StaticSphereObstacle>(sta_pos_3, 0.4);
    sta_obstacles_.push_back(sta_obs_3);

    Eigen::Vector3d sta_pos_4(2.0, 0.0, 1.0);
    auto sta_obs_4 = std::make_shared<tprm::StaticSphereObstacle>(sta_pos_4, 0.4);
    sta_obstacles_.push_back(sta_obs_4);

    Eigen::Vector3d sta_pos_5(4.0, 0.0, 1.0);
    auto sta_obs_5 = std::make_shared<tprm::StaticSphereObstacle>(sta_pos_5, 0.4);
    sta_obstacles_.push_back(sta_obs_5);

    // Eigen::Vector3d sta_pos_6(-2.0, 0.0, 1.0);
    // auto sta_obs_6 = std::make_shared<tprm::StaticSphereObstacle>(sta_pos_6, 0.4);
    // sta_obstacles_.push_back(sta_obs_6);

    ROS_INFO("Created %lu static obstacles", sta_obstacles_.size());
    ROS_INFO("Created %lu dynamic obstacles", dyn_obstacles_.size());

    // 打印动态障碍物信息
    for (size_t i = 0; i < dyn_obstacles_.size(); i++) {
      Eigen::Vector3d pos_t0 = dyn_obstacles_[i]->getCOM(0.0);
      Eigen::Vector3d vel = dyn_obstacles_[i]->getVelocity();
      ROS_INFO("  Dyn obs %zu: pos(t=0)=[%.2f, %.2f, %.2f], vel=[%.2f, %.2f, %.2f]",
               i, pos_t0(0), pos_t0(1), pos_t0(2), vel(0), vel(1), vel(2));

      // 预测未来位置
      Eigen::Vector3d pos_t5 = dyn_obstacles_[i]->getCOM(5.0);
      ROS_INFO("           pos(t=5)=[%.2f, %.2f, %.2f]",
               pos_t5(0), pos_t5(1), pos_t5(2));
    }

    ROS_INFO("=== Scene Creation Complete ===\n");
  }

  /**
   * @brief 可视化动态障碍物轨迹
   */
  void visualizeDynamicObstacles() {
    visualization_msgs::MarkerArray markers;
    int marker_id = 0;

    // 计算当前时间（相对于开始时间），并让时间循环
    double current_time = (ros::Time::now() - start_time_).toSec();
    // 让时间在 0 到 10 秒之间循环，这样障碍物会重复移动
    current_time = fmod(current_time, 10.0);

    // 可视化每个动态障碍物在不同时间的位置
    for (size_t i = 0; i < dyn_obstacles_.size(); i++) {
      const auto& obs = dyn_obstacles_[i];

      // 绘制轨迹线（显示从当前时间开始的未来轨迹）
      visualization_msgs::Marker traj_marker;
      traj_marker.header.frame_id = "map";
      traj_marker.header.stamp = ros::Time::now();
      traj_marker.ns = "dyn_trajectories";
      traj_marker.id = marker_id++;
      traj_marker.type = visualization_msgs::Marker::LINE_STRIP;
      traj_marker.action = visualization_msgs::Marker::ADD;
      traj_marker.scale.x = 0.05;
      traj_marker.color.r = 1.0;
      traj_marker.color.g = 0.5;
      traj_marker.color.b = 0.0;
      traj_marker.color.a = 0.8;

      // 采样轨迹点（从当前时间开始）
      for (double t = current_time; t <= current_time + max_prediction_time_; t += 0.1) {
        Eigen::Vector3d pos = obs->getCOM(t);
        geometry_msgs::Point p;
        p.x = pos(0);
        p.y = pos(1);
        p.z = pos(2);
        traj_marker.points.push_back(p);
      }
      markers.markers.push_back(traj_marker);

      // 绘制当前时刻的球体位置（这样球体会移动）
      visualization_msgs::Marker sphere_marker;
      sphere_marker.header.frame_id = "map";
      sphere_marker.header.stamp = ros::Time::now();
      sphere_marker.ns = "dyn_spheres";
      sphere_marker.id = marker_id++;
      sphere_marker.type = visualization_msgs::Marker::SPHERE;
      sphere_marker.action = visualization_msgs::Marker::ADD;

      // 使用当前时间计算障碍物位置（关键修改！）
      Eigen::Vector3d pos_current = obs->getCOM(current_time);
      sphere_marker.pose.position.x = pos_current(0);
      sphere_marker.pose.position.y = pos_current(1);
      sphere_marker.pose.position.z = pos_current(2);
      sphere_marker.pose.orientation.w = 1.0;

      double radius = obs->getRadius();
      sphere_marker.scale.x = sphere_marker.scale.y = sphere_marker.scale.z = 2.0 * radius;
      sphere_marker.color.r = 1.0;
      sphere_marker.color.g = 0.0;
      sphere_marker.color.b = 0.0;
      sphere_marker.color.a = 0.5;

      markers.markers.push_back(sphere_marker);
    }

    // 可视化静态障碍物
    for (size_t i = 0; i < sta_obstacles_.size(); i++) {
      visualization_msgs::Marker marker;
      marker.header.frame_id = "map";
      marker.header.stamp = ros::Time::now();
      marker.ns = "static_obs";
      marker.id = marker_id++;
      marker.type = visualization_msgs::Marker::SPHERE;
      marker.action = visualization_msgs::Marker::ADD;

      Eigen::Vector3d pos = sta_obstacles_[i]->getCOM();
      marker.pose.position.x = pos(0);
      marker.pose.position.y = pos(1);
      marker.pose.position.z = pos(2);
      marker.pose.orientation.w = 1.0;

      double radius = sta_obstacles_[i]->getRadius();
      marker.scale.x = marker.scale.y = marker.scale.z = 2.0 * radius;
      marker.color.r = 0.5;
      marker.color.g = 0.5;
      marker.color.b = 0.5;
      marker.color.a = 0.8;

      markers.markers.push_back(marker);
    }

    obs_vis_pub_.publish(markers);
    ROS_INFO("Dynamic obstacles visualization published!");
  }

  // ========== 以下是从test_prm_graph.cpp复制的辅助函数 ==========

  Eigen::Vector3d getSample() {
    Eigen::Vector3d pt;
    pt(0) = rand_pos_(eng_) * sample_r_(0);
    pt(1) = rand_pos_(eng_) * sample_r_(1);
    pt(2) = rand_pos_(eng_) * sample_r_(2);

    pt = rotation_ * pt + translation_;
    return pt;
  }

  bool lineVisibStatic(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2,
                       double base_step, Eigen::Vector3d& pc) {
    Eigen::Vector3d dir = p2 - p1;
    double total_len = dir.norm();
    if (total_len < 1e-6) return true;
    dir.normalize();

    double s = 0.0;
    while (s <= total_len) {
      Eigen::Vector3d pt = p1 + dir * s;

      for (const auto& obstacle : sta_obstacles_) {
        if (obstacle->isColliding(pt)) {
          pc = pt;
          return false;
        }
      }
      s += base_step;
    }

    // 检查终点
    for (const auto& obstacle : sta_obstacles_) {
      if (obstacle->isColliding(p2)) {
        pc = p2;
        return false;
      }
    }

    return true;
  }

  std::vector<GraphNode::Ptr> findVisibGuard(Eigen::Vector3d pt) {
    std::vector<GraphNode::Ptr> visib_guards;
    Eigen::Vector3d pc;
    int visib_num = 0;

    for (auto iter = graph_.begin(); iter != graph_.end(); ++iter) {
      if ((*iter)->type_ == GraphNode::Connector) continue;

      if (lineVisibStatic(pt, (*iter)->pos_, line_step_, pc)) {
        visib_guards.push_back(*iter);
        ++visib_num;
        if (visib_num > 2) break;
      }
    }

    return visib_guards;
  }

  double pathLength(const std::vector<Eigen::Vector3d>& path) {
    double length = 0.0;
    if (path.size() < 2) return length;

    for (size_t i = 0; i < path.size() - 1; ++i) {
      length += (path[i + 1] - path[i]).norm();
    }
    return length;
  }

  std::vector<Eigen::Vector3d> discretizePath(const std::vector<Eigen::Vector3d>& path, int pt_num) {
    std::vector<double> len_list;
    len_list.push_back(0.0);

    for (size_t i = 0; i < path.size() - 1; ++i) {
      double inc_l = (path[i + 1] - path[i]).norm();
      len_list.push_back(inc_l + len_list[i]);
    }

    double len_total = len_list.back();
    double dl = len_total / double(pt_num - 1);

    std::vector<Eigen::Vector3d> dis_path;
    for (int i = 0; i < pt_num; ++i) {
      double cur_l = double(i) * dl;

      int idx = -1;
      for (size_t j = 0; j < len_list.size() - 1; ++j) {
        if (cur_l >= len_list[j] - 1e-4 && cur_l <= len_list[j + 1] + 1e-4) {
          idx = j;
          break;
        }
      }

      double lambda = (cur_l - len_list[idx]) / (len_list[idx + 1] - len_list[idx]);
      Eigen::Vector3d inter_pt = (1 - lambda) * path[idx] + lambda * path[idx + 1];
      dis_path.push_back(inter_pt);
    }

    return dis_path;
  }

  bool sameTopoPath(const std::vector<Eigen::Vector3d>& path1,
                    const std::vector<Eigen::Vector3d>& path2) {
    double len1 = pathLength(path1);
    double len2 = pathLength(path2);
    double max_len = std::max(len1, len2);
    int pt_num = ceil(max_len / line_step_);

    std::vector<Eigen::Vector3d> pts1 = discretizePath(path1, pt_num);
    std::vector<Eigen::Vector3d> pts2 = discretizePath(path2, pt_num);

    Eigen::Vector3d pc;
    for (int i = 0; i < pt_num; ++i) {
      if (!lineVisibStatic(pts1[i], pts2[i], line_step_, pc)) {
        return false;
      }
    }

    return true;
  }

  std::vector<std::pair<double, double>> mergeIntervals(
      std::vector<std::pair<double, double>> intervals) {
    if (intervals.empty())
      return intervals;

    std::sort(intervals.begin(), intervals.end());
    std::vector<std::pair<double, double>> merged;
    merged.push_back(intervals[0]);

    for (size_t i = 1; i < intervals.size(); ++i) {
      if (intervals[i].first <= merged.back().second) {
        merged.back().second = std::max(merged.back().second, intervals[i].second);
      } else {
        merged.push_back(intervals[i]);
      }
    }

    return merged;
  }

  std::vector<std::pair<double, double>> computeSafeIntervals(
      const Eigen::Vector3d& position,
      const std::vector<std::shared_ptr<tprm::DynamicSphereObstacle>>& obstacles,
      double robot_radius = 0.3) {
    std::vector<std::pair<double, double>> safe_intervals;
    std::vector<std::pair<double, double>> collision_intervals;

    // 收集所有障碍物的碰撞时间间隔
    for (const auto& obstacle : obstacles) {
      double hit_time_from, hit_time_to;
      if (obstacle->isColliding(position, hit_time_from, hit_time_to)) {
        // 只考虑max_prediction_time内的碰撞
        if (hit_time_from < max_prediction_time_) {
          hit_time_to = std::min(hit_time_to, max_prediction_time_);
          collision_intervals.emplace_back(hit_time_from, hit_time_to);
        }
      }
    }

    // 合并重叠的碰撞间隔
    collision_intervals = mergeIntervals(collision_intervals);

    // 从碰撞间隔推导安全间隔
    if (collision_intervals.empty()) {
      safe_intervals.emplace_back(0.0, max_prediction_time_);
    } else {
      if (collision_intervals[0].first > 0) {
        safe_intervals.emplace_back(0.0, collision_intervals[0].first);
      }

      for (size_t i = 0; i < collision_intervals.size() - 1; ++i) {
        double safe_start = collision_intervals[i].second;
        double safe_end = collision_intervals[i + 1].first;
        if (safe_start < safe_end) {
          safe_intervals.emplace_back(safe_start, safe_end);
        }
      }

      if (collision_intervals.back().second < max_prediction_time_) {
        safe_intervals.emplace_back(collision_intervals.back().second, max_prediction_time_);
      }
    }

    return safe_intervals;
  }

  std::pair<double, double> computeObstacleCollisionWindow(
      const Eigen::Vector3d& from,
      const Eigen::Vector3d& to,
      const std::shared_ptr<tprm::DynamicSphereObstacle>& obstacle,
      double robot_speed) {
    double distance = (to - from).norm();
    double travel_time = distance / robot_speed;

    double min_collision_time = std::numeric_limits<double>::infinity();
    double max_collision_time = 0.0;

    const int samples = 10;
    for (int i = 0; i <= samples; ++i) {
      double t = static_cast<double>(i) / samples;
      Eigen::Vector3d point = from + t * (to - from);
      double segment_time = t * travel_time;

      double hit_from, hit_to;
      if (obstacle->isColliding(point, hit_from, hit_to)) {
        double adjusted_from = std::max(0.0, hit_from - segment_time);
        double adjusted_to = std::max(0.0, hit_to - segment_time);

        min_collision_time = std::min(min_collision_time, adjusted_from);
        max_collision_time = std::max(max_collision_time, adjusted_to);
      }
    }

    if (min_collision_time < std::numeric_limits<double>::infinity()) {
      return {min_collision_time, max_collision_time};
    }

    return {std::numeric_limits<double>::infinity(), 0.0};
  }

  std::vector<std::pair<double, double>> computeEdgeSafeWindow(
      const Eigen::Vector3d& from,
      const Eigen::Vector3d& to,
      const std::vector<std::shared_ptr<tprm::DynamicSphereObstacle>>& obstacles,
      double robot_speed) {
    std::vector<std::pair<double, double>> safe_windows;
    std::vector<std::pair<double, double>> collision_windows;

    for (const auto& obstacle : obstacles) {
      auto window = computeObstacleCollisionWindow(from, to, obstacle, robot_speed);
      if (window.first < window.second) {
        // 限制碰撞窗口的结束时间
        if (window.first < max_prediction_time_) {
          window.second = std::min(window.second, max_prediction_time_);
          collision_windows.push_back(window);
        }
      }
    }

    collision_windows = mergeIntervals(collision_windows);

    if (collision_windows.empty()) {
      safe_windows.emplace_back(0.0, max_prediction_time_);
    } else {
      if (collision_windows[0].first > 0) {
        safe_windows.emplace_back(0.0, collision_windows[0].first);
      }

      for (size_t i = 0; i < collision_windows.size() - 1; ++i) {
        double safe_start = collision_windows[i].second;
        double safe_end = collision_windows[i + 1].first;
        if (safe_start < safe_end) {
          safe_windows.emplace_back(safe_start, safe_end);
        }
      }

      if (collision_windows.back().second < max_prediction_time_) {
        safe_windows.emplace_back(collision_windows.back().second, max_prediction_time_);
      }
    }

    return safe_windows;
  }

  std::vector<std::pair<double,double>> intersectIntervals(
      const std::vector<std::pair<double,double>>& A,
      const std::vector<std::pair<double,double>>& B) {
    std::vector<std::pair<double,double>> R;
    size_t ia = 0, ib = 0;
    while(ia < A.size() && ib < B.size()){
      double s = std::max(A[ia].first, B[ib].first);
      double e = std::min(A[ia].second, B[ib].second);
      if (e >= s) R.emplace_back(s,e);
      if (A[ia].second < B[ib].second) ++ia; else ++ib;
    }
    return R;
  }

  bool isObstacleCollidingAtTime(
      const std::shared_ptr<tprm::DynamicSphereObstacle>& obstacle,
      const Eigen::Vector3d& position,
      double time,
      double eps_dist = 1e-3) const {
    double hit_time_from, hit_time_to;
    if (!obstacle->isColliding(position, hit_time_from, hit_time_to)) {
      return false;
    }

    return (time >= hit_time_from - eps_dist && time <= hit_time_to + eps_dist);
  }

  bool sameTopoPathUTVD(const GraphNode::Ptr guard1, const GraphNode::Ptr guard2,
                        const GraphNode::Ptr connector, const Eigen::Vector3d pt) {
    // 构造路径
    std::array<Eigen::Vector3d, 3> path_new = {guard1->pos_, pt, guard2->pos_};
    std::array<Eigen::Vector3d, 3> path_exist = {guard1->pos_, connector->pos_, guard2->pos_};

    // 计算边的安全时间窗口
    safe_edge_windows_3_ = computeEdgeSafeWindow(guard1->pos_, connector->pos_, dyn_obstacles_, robot_speed_);
    safe_edge_windows_4_ = computeEdgeSafeWindow(connector->pos_, guard2->pos_, dyn_obstacles_, robot_speed_);

    // 计算两个边的安全时间窗口的交集
    auto corridors1 = intersectIntervals(safe_edge_windows_1_, safe_edge_windows_2_);
    auto corridors2 = intersectIntervals(safe_edge_windows_3_, safe_edge_windows_4_);

    // 如果没有重叠，则返回false
    if (corridors1.empty() || corridors2.empty()) return false;

    // 构造 PathAdapter，映射 s ∈ [0,1] → 空间 & 时间
    struct PathAdapter {
      std::function<Eigen::Vector3d(double)> posAt;
      std::function<double(double)> timeAt;
    };

    auto makePathAdapter = [&](const std::array<Eigen::Vector3d, 3> &path, const std::pair<double, double> &corr) {
      double t_start = corr.first;
      double t_end = corr.second;
      double T = t_end - t_start;
      PathAdapter P;
      P.posAt = [=](double s) -> Eigen::Vector3d {
        if (s <= 0.5) {
          double u = s / 0.5;
          return (1.0 - u) * path[0] + u * path[1];
        } else {
          double u = (s - 0.5) / 0.5;
          return (1.0 - u) * path[1] + u * path[2];
        }
      };
      P.timeAt = [=](double s) -> double {
        return t_start + s * T;
      };
      return P;
    };

    // 遍历两路径的所有时间走廊组合
    const int Ns_coarse = 12;
    const int N_lambda_coarse = 6;
    const int Ns_fine = 48;
    const int N_lambda_fine = 24;
    const double eps_dist = 1e-3;
    const double eps_time = 1e-6;

    for (const auto &c1 : corridors1) {
      for (const auto &c2 : corridors2) {
        double T1 = c1.second - c1.first;
        double T2 = c2.second - c2.first;
        if (T1 <= eps_time || T2 <= eps_time)
          continue;

        double alpha = T1 / T2;
        double theta = (c1.first - c2.first) / T2;

        if (theta < -eps_time || alpha + theta > 1.0 + eps_time)
          continue;

        auto P1 = makePathAdapter(path_new, c1);
        auto P2 = makePathAdapter(path_exist, c2);

        // UTVD 检查函数（coarse + fine）
        auto checkUTVD = [&](int Ns, int N_lambda) -> bool {
          for (int i = 0; i < Ns; ++i) {
            double s = double(i) / (Ns - 1);
            double s2 = alpha * s + theta;
            if (s2 < 0.0 - 1e-9 || s2 > 1.0 + 1e-9)
              return false;

            Eigen::Vector3d p1 = P1.posAt(s);
            Eigen::Vector3d p2 = P2.posAt(s2);
            double t1 = P1.timeAt(s);
            double t2 = P2.timeAt(s2);

            for (int j = 0; j < N_lambda; ++j) {
              double lambda = double(j) / (N_lambda - 1);
              Eigen::Vector3d x = (1.0 - lambda) * p1 + lambda * p2;
              double tau = (1.0 - lambda) * t1 + lambda * t2;

              for (const auto &obs : dyn_obstacles_) {
                if (isObstacleCollidingAtTime(obs, x, tau, eps_dist)) {
                  return false;
                }
              }
            }
          }
          return true;
        };

        bool coarse_pass = checkUTVD(Ns_coarse, N_lambda_coarse);
        if (!coarse_pass)
          continue;
        bool fine_pass = checkUTVD(Ns_fine, N_lambda_fine);

        if (fine_pass) {
          return true;
        }
      }
    }

    return false;
  }

  bool needConnection(GraphNode::Ptr g1, GraphNode::Ptr g2, Eigen::Vector3d pt) {
    std::vector<Eigen::Vector3d> path1(3), path2(3);
    GraphNode::Ptr connector;
    path1[0] = g1->pos_;
    path1[1] = pt;
    path1[2] = g2->pos_;

    path2[0] = g1->pos_;
    path2[2] = g2->pos_;

    for (size_t i = 0; i < g1->neighbors_.size(); ++i) {
      for (size_t j = 0; j < g2->neighbors_.size(); ++j) {
        if (g1->neighbors_[i]->id_ == g2->neighbors_[j]->id_) {
          path2[1] = g1->neighbors_[i]->pos_;
          connector = g1->neighbors_[i];
          bool same_topo = sameTopoPathUTVD(g1, g2, connector, pt);
          if (same_topo) {
            if (pathLength(path1) < pathLength(path2)) {
              g1->neighbors_[i]->pos_ = pt;
            }
            return false;
          }
        }
      }
    }
    return true;
  }

  void pruneGraph() {
    if (graph_.size() > 2) {
      for (auto iter1 = graph_.begin(); iter1 != graph_.end() && graph_.size() > 2; ++iter1) {
        if ((*iter1)->id_ <= 1) continue;

        if ((*iter1)->neighbors_.size() <= 1) {
          for (auto iter2 = graph_.begin(); iter2 != graph_.end(); ++iter2) {
            for (auto it_nb = (*iter2)->neighbors_.begin(); it_nb != (*iter2)->neighbors_.end(); ++it_nb) {
              if ((*it_nb)->id_ == (*iter1)->id_) {
                (*iter2)->neighbors_.erase(it_nb);
                break;
              }
            }
          }

          graph_.erase(iter1);
          iter1 = graph_.begin();
        }
      }
    }
  }

  /**
   * @brief 创建PRM图，处理动态障碍物和时间同伦拓扑
   * 这是核心函数，验证动态障碍物的时间窗口计算和UTVD同伦检查
   */
  std::list<GraphNode::Ptr> createGraph() {
    ROS_INFO("\n=== Starting PRM Graph Creation with Dynamic Obstacles ===");

    graph_.clear();

    // 初始化步长
    line_step_ = 0.5 * robot_speed_;

    // 初始化起点和终点节点
    GraphNode::Ptr start_node = GraphNode::Ptr(new GraphNode(start_pos_, GraphNode::Guard, 0));
    GraphNode::Ptr end_node = GraphNode::Ptr(new GraphNode(goal_pos_, GraphNode::Guard, 1));

    graph_.push_back(start_node);
    graph_.push_back(end_node);

    // 设置采样区域
    sample_r_(0) = 0.5 * (goal_pos_ - start_pos_).norm() + sample_inflate_(0);
    sample_r_(1) = sample_inflate_(1);
    sample_r_(2) = sample_inflate_(2);

    // 设置变换
    translation_ = 0.5 * (start_pos_ + goal_pos_);

    Eigen::Vector3d xtf, ytf, ztf, downward(0, 0, -1);
    xtf = (goal_pos_ - translation_).normalized();
    ytf = xtf.cross(downward).normalized();
    ztf = xtf.cross(ytf);

    rotation_.col(0) = xtf;
    rotation_.col(1) = ytf;
    rotation_.col(2) = ztf;

    int node_id = 1;
    int sample_num = 0;
    double sample_time = 0.0;

    // 统计计数器
    int rejected_by_time_window = 0;
    int rejected_by_topology = 0;
    int accepted_connections = 0;

    ROS_INFO("Sample region: [%.2f, %.2f, %.2f]", sample_r_(0), sample_r_(1), sample_r_(2));
    ROS_INFO("Translation: [%.2f, %.2f, %.2f]", translation_(0), translation_(1), translation_(2));
    ROS_INFO("Max prediction time: %.2f s", max_prediction_time_);

    // 主采样循环
    ros::Time t1, t2;
    while (sample_time < max_sample_time_ && sample_num < max_sample_num_) {
      t1 = ros::Time::now();

      Eigen::Vector3d pt = getSample();
      ++sample_num;

      // 检查采样点是否在静态障碍物内
      bool is_blocked = false;
      for (const auto& obstacle : sta_obstacles_) {
        if (obstacle->isColliding(pt)) {
          is_blocked = true;
          break;
        }
      }

      if (is_blocked) {
        sample_time += (ros::Time::now() - t1).toSec();
        continue;
      }

      // 查找可见的guard节点
      std::vector<GraphNode::Ptr> visib_guards = findVisibGuard(pt);

      if (visib_guards.size() == 0) {
        // 创建新的guard节点
        GraphNode::Ptr guard = GraphNode::Ptr(new GraphNode(pt, GraphNode::Guard, ++node_id));
        graph_.push_back(guard);
      } else if (visib_guards.size() == 2) {
        // **关键部分：考虑动态障碍物**
        auto visib_guard_1 = visib_guards[0];
        auto visib_guard_2 = visib_guards[1];

        // 计算安全时间区间
        auto safe_intervals_1 = computeSafeIntervals(visib_guard_1->pos_, dyn_obstacles_);
        auto safe_intervals_2 = computeSafeIntervals(visib_guard_2->pos_, dyn_obstacles_);
        auto safe_intervals_pt = computeSafeIntervals(pt, dyn_obstacles_);

        // 检查边的安全时间窗口
        safe_edge_windows_1_ = computeEdgeSafeWindow(
            visib_guard_1->pos_, pt, dyn_obstacles_, robot_speed_);
        safe_edge_windows_2_ = computeEdgeSafeWindow(
            pt, visib_guard_2->pos_, dyn_obstacles_, robot_speed_);

        // 检查时间窗口重叠
        auto intervalsOverlap = [](const std::vector<std::pair<double, double>>& a,
                                   const std::vector<std::pair<double, double>>& b,
                                   double eps = 1e-6) -> bool {
          if (a.empty() || b.empty()) return false;
          size_t ia = 0, ib = 0;
          while (ia < a.size() && ib < b.size()) {
            double a_start = a[ia].first, a_end = a[ia].second;
            double b_start = b[ib].first, b_end = b[ib].second;

            if (a_start <= b_end + eps && b_start <= a_end + eps) {
              double overlap_start = std::max(a_start, b_start);
              double overlap_end = std::min(a_end, b_end);
              if (overlap_end + eps >= overlap_start) return true;
            }

            if (a_end < b_end) ++ia; else ++ib;
          }
          return false;
        };

        // 如果没有时间窗口重叠，则拒绝连接
        if (!intervalsOverlap(safe_edge_windows_1_, safe_edge_windows_2_)) {
          rejected_by_time_window++;
          sample_time += (ros::Time::now() - t1).toSec();
          continue;
        }

        // **关键部分：时间同伦拓扑检查（UTVD）**
        bool need_connect = needConnection(visib_guards[0], visib_guards[1], pt);
        if (!need_connect) {
          rejected_by_topology++;
          sample_time += (ros::Time::now() - t1).toSec();
          continue;
        }

        // 创建新的connector节点
        GraphNode::Ptr connector = GraphNode::Ptr(new GraphNode(pt, GraphNode::Connector, ++node_id));
        graph_.push_back(connector);
        accepted_connections++;

        // 连接guards
        visib_guards[0]->neighbors_.push_back(connector);
        visib_guards[1]->neighbors_.push_back(connector);
        connector->neighbors_.push_back(visib_guards[0]);
        connector->neighbors_.push_back(visib_guards[1]);
      }

      sample_time += (ros::Time::now() - t1).toSec();
    }

    // 输出统计信息
    ROS_INFO("\n=== Graph Creation Statistics ===");
    ROS_INFO("  Total samples: %d", sample_num);
    ROS_INFO("  Sample time: %.3f s", sample_time);
    ROS_INFO("  Rejected by time window: %d", rejected_by_time_window);
    ROS_INFO("  Rejected by topology check (UTVD): %d", rejected_by_topology);
    ROS_INFO("  Accepted connections: %d", accepted_connections);

    int total_2guard_samples = rejected_by_time_window + rejected_by_topology + accepted_connections;
    if (total_2guard_samples > 0) {
      ROS_INFO("  Time window rejection rate: %.1f%%",
               100.0 * rejected_by_time_window / total_2guard_samples);
      ROS_INFO("  Topology rejection rate: %.1f%%",
               100.0 * rejected_by_topology / total_2guard_samples);
      ROS_INFO("  Acceptance rate: %.1f%%",
               100.0 * accepted_connections / total_2guard_samples);
    }

    // 剪枝
    pruneGraph();

    ROS_INFO("Final graph nodes: %lu", graph_.size());
    ROS_INFO("=== Graph Creation Complete ===\n");

    return graph_;
  }

  void visualizeGraph() {
    visualization_msgs::MarkerArray node_markers;
    visualization_msgs::MarkerArray edge_markers;

    int marker_id = 0;

    // 可视化节点
    for (const auto& node : graph_) {
      visualization_msgs::Marker marker;
      marker.header.frame_id = "map";
      marker.header.stamp = ros::Time::now();
      marker.ns = "prm_nodes";
      marker.id = marker_id++;
      marker.type = visualization_msgs::Marker::SPHERE;
      marker.action = visualization_msgs::Marker::ADD;

      marker.pose.position.x = node->pos_(0);
      marker.pose.position.y = node->pos_(1);
      marker.pose.position.z = node->pos_(2);
      marker.pose.orientation.w = 1.0;

      // Guard节点（蓝色）和Connector节点（绿色）
      if (node->type_ == GraphNode::Guard) {
        if (node->id_ == 0) {
          // 起点（深蓝色，大一些）
          marker.scale.x = marker.scale.y = marker.scale.z = 0.3;
          marker.color.r = 0.0;
          marker.color.g = 0.0;
          marker.color.b = 1.0;
          marker.color.a = 1.0;
        } else if (node->id_ == 1) {
          // 终点（紫色，大一些）
          marker.scale.x = marker.scale.y = marker.scale.z = 0.3;
          marker.color.r = 1.0;
          marker.color.g = 0.0;
          marker.color.b = 1.0;
          marker.color.a = 1.0;
        } else {
          // 普通guard节点（蓝色）
          marker.scale.x = marker.scale.y = marker.scale.z = 0.15;
          marker.color.r = 0.2;
          marker.color.g = 0.2;
          marker.color.b = 1.0;
          marker.color.a = 0.8;
        }
      } else {
        // Connector节点（绿色）
        marker.scale.x = marker.scale.y = marker.scale.z = 0.12;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.color.a = 0.8;
      }

      node_markers.markers.push_back(marker);
    }

    // 可视化边
    for (const auto& node : graph_) {
      for (const auto& neighbor : node->neighbors_) {
        // 避免重复绘制边
        if (node->id_ < neighbor->id_) {
          visualization_msgs::Marker marker;
          marker.header.frame_id = "map";
          marker.header.stamp = ros::Time::now();
          marker.ns = "prm_edges";
          marker.id = marker_id++;
          marker.type = visualization_msgs::Marker::LINE_STRIP;
          marker.action = visualization_msgs::Marker::ADD;

          geometry_msgs::Point p1, p2;
          p1.x = node->pos_(0);
          p1.y = node->pos_(1);
          p1.z = node->pos_(2);
          p2.x = neighbor->pos_(0);
          p2.y = neighbor->pos_(1);
          p2.z = neighbor->pos_(2);

          marker.points.push_back(p1);
          marker.points.push_back(p2);

          marker.scale.x = 0.05;
          marker.color.r = 1.0;
          marker.color.g = 1.0;
          marker.color.b = 0.0;
          marker.color.a = 0.6;

          edge_markers.markers.push_back(marker);
        }
      }
    }

    node_vis_pub_.publish(node_markers);
    graph_vis_pub_.publish(edge_markers);

    ROS_INFO("Graph visualization published!");
  }

  void run() {
    ros::Rate rate(10.0);  // 10 Hz - 更流畅的动画效果

    // 初始化开始时间（用于动态障碍物动画）
    start_time_ = ros::Time::now();

    // 等待订阅者连接
    ros::Duration(0.5).sleep();

    // 创建场景
    createSimpleDynamicScene();

    // 可视化障碍物
    visualizeDynamicObstacles();

    ros::Duration(0.5).sleep();

    // 创建并可视化图
    ROS_INFO("\nCreating PRM graph...");
    // 计时：记录 createGraph 开始时间（使用墙钟，不受 /use_sim_time 影响）
    ros::WallTime _create_start = ros::WallTime::now();
    createGraph();
    // 计时：计算并打印耗时
    ros::WallDuration _create_dur = ros::WallTime::now() - _create_start;
    ROS_WARN("createGraph elapsed: %.6f s (%.3f ms)", _create_dur.toSec(), _create_dur.toSec() * 1000.0);

    visualizeGraph();

    ROS_INFO("\n=== Test Complete ===");
    ROS_INFO("Check RViz to see:");
    ROS_INFO("  - Dynamic obstacle trajectories (orange lines)");
    ROS_INFO("  - Dynamic obstacles (red spheres)");
    ROS_INFO("  - Static obstacles (gray spheres)");
    ROS_INFO("  - PRM graph nodes (blue=guards, green=connectors)");
    ROS_INFO("  - PRM graph edges (yellow lines)");
    ROS_INFO("\nThe graph should avoid both static and dynamic obstacles,");
    ROS_INFO("and demonstrate time-homotopic topology classes.\n");

    // 保持运行以持续发布可视化
    while (ros::ok()) {
      visualizeDynamicObstacles();
      visualizeGraph();
      ros::spinOnce();
      rate.sleep();
    }
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "test_prm_dynamic_simple");
  ros::NodeHandle nh("~");

  DynamicSceneTester tester(nh);

  ROS_INFO("=================================================");
  ROS_INFO("  PRM Dynamic Scene Tester");
  ROS_INFO("=================================================");
  ROS_INFO("This test creates a simple dynamic scene with:");
  ROS_INFO("  - Moving obstacles that create time-varying topology");
  ROS_INFO("  - Static obstacles");
  ROS_INFO("  - PRM graph creation with UTVD homotopy checking");
  ROS_INFO("=================================================\n");

  tester.run();

  return 0;
}
