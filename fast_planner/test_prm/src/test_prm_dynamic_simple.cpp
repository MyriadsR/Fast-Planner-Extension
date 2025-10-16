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

using namespace std;

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
  ros::Publisher robot_vis_pub_;

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
  double robot_radius_;  // 机器人半径
  double max_sample_time_;
  int max_sample_num_;
  double safety_margin_;
  double max_prediction_time_;
  int max_raw_path_, max_raw_path2_;
  int reserve_num_;
  double ratio_to_short_;

  ros::Time start_time_;  // 记录开始时间，用于动态障碍物动画

  std::list<GraphNode::Ptr> graph_;
  vector<vector<Eigen::Vector3d>> raw_paths_;
  vector<vector<Eigen::Vector3d>> filtered_paths_;
  vector<vector<Eigen::Vector3d>> final_paths_;
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
    robot_vis_pub_ = nh_.advertise<visualization_msgs::Marker>("/robot_sphere", 10);

    // 初始化随机数生成器
    eng_ = std::default_random_engine(rd_());
    rand_pos_ = std::uniform_real_distribution<double>(-1.0, 1.0);

    // 读取参数
    nh_.param("robot_speed", robot_speed_, 0.0);
    nh_.param("robot_radius", robot_radius_, 0.3);  // 默认半径0.3米
    nh_.param("max_sample_time", max_sample_time_, 0.0);
    nh_.param("max_sample_num", max_sample_num_, -1);
    nh_.param("safety_margin", safety_margin_, 0.0);
    nh_.param("max_raw_path", max_raw_path_, -1);
    nh_.param("max_raw_path2", max_raw_path2_, -1);
    nh_.param("reserve_num", reserve_num_, 0);
    nh_.param("ratio_to_short", ratio_to_short_, 0.0);
    

    std::vector<double> inflate;
    nh_.param("sample_inflate_x", inflate, std::vector<double>());
    if (inflate.size() >= 3) {
      sample_inflate_ = Eigen::Vector3d(inflate[0], inflate[1], inflate[2]);
    } else {
      sample_inflate_ = Eigen::Vector3d(2.0, 2.0, 1.0);
    }

    line_step_ = 0.1 * robot_speed_;
    max_prediction_time_ = 10.0;

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
    
    // 障碍物5：Z 轴速度
    Eigen::Vector3d dyn_pos_5(0.0, 0.0, 1.0);
    Eigen::Vector3d dyn_vel_5(0.0, 0.0, 0.6);   
    double dyn_radius_5 = 0.5 + safety_margin_;

    auto dyn_obs_5 = std::make_shared<tprm::DynamicSphereObstacle>(dyn_pos_5, dyn_vel_5, dyn_radius_5);
    dyn_obstacles_.push_back(dyn_obs_5);

    // 添加少量静态障碍物（可选）
    Eigen::Vector3d sta_pos_1(3.0, 0.5, 1.0);
    auto sta_obs_1 = std::make_shared<tprm::StaticSphereObstacle>(sta_pos_1, 0.4);
    sta_obstacles_.push_back(sta_obs_1);

    Eigen::Vector3d sta_pos_2(-2.0, -0.5, 1.0);
    auto sta_obs_2 = std::make_shared<tprm::StaticSphereObstacle>(sta_pos_2, 0.4);
    sta_obstacles_.push_back(sta_obs_2);

    Eigen::Vector3d sta_pos_3(0.0, 0.0, 1.0);
    auto sta_obs_3 = std::make_shared<tprm::StaticSphereObstacle>(sta_pos_3, 0.4);
    sta_obstacles_.push_back(sta_obs_3);

    Eigen::Vector3d sta_pos_4(2.0, 1.0, 1.0);
    auto sta_obs_4 = std::make_shared<tprm::StaticSphereObstacle>(sta_pos_4, 0.4);
    sta_obstacles_.push_back(sta_obs_4);

    Eigen::Vector3d sta_pos_5(4.0, -1.0, 1.0);
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
      sphere_marker.color.a = 1.0;

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
      marker.color.a = 1.0;

      markers.markers.push_back(marker);
    }

    obs_vis_pub_.publish(markers);
    ROS_INFO("Dynamic obstacles visualization published!");
  }

  /**
   * @brief 可视化机器人沿所有路径的运动
   */
  void visualizeRobotMotion() {
    // 检查是否有可用的路径
    if (final_paths_.empty()) {
      return;
    }

    // 计算当前时间（相对于开始时间），与障碍物同步
    double current_time = (ros::Time::now() - start_time_).toSec();

    // 让时间在 0 到 10 秒之间循环，与障碍物同步
    const double CYCLE_TIME = 10.0;
    current_time = fmod(current_time, CYCLE_TIME);

    visualization_msgs::MarkerArray robot_markers;

    // 遍历所有路径，为每条路径创建一个机器人小球
    for (size_t path_idx = 0; path_idx < final_paths_.size(); ++path_idx) {
      const auto& path = final_paths_[path_idx];

      // 检查路径是否有效
      if (path.size() < 2) {
        continue;
      }

      // 计算路径总长度
      double total_length = pathLength(path);
      if (total_length < 1e-6) {
        continue;
      }

      // 计算总的运动时间
      double total_time = total_length / robot_speed_;

      // 如果路径运动时间超过循环周期，则使用循环周期内的时间
      // 如果路径运动时间小于循环周期，则让机器人多次循环该路径
      double effective_time = fmod(current_time, total_time);

      // 计算机器人在路径上的位置
      double traveled_distance = effective_time * robot_speed_;

      // 根据已行驶的距离，在路径上找到对应的位置
      Eigen::Vector3d robot_pos;
      double accumulated_length = 0.0;
      bool found = false;

      for (size_t i = 0; i < path.size() - 1; ++i) {
        double segment_length = (path[i + 1] - path[i]).norm();

        if (accumulated_length + segment_length >= traveled_distance) {
          // 机器人在这一段上
          double ratio = (traveled_distance - accumulated_length) / segment_length;
          robot_pos = path[i] + ratio * (path[i + 1] - path[i]);
          found = true;
          break;
        }

        accumulated_length += segment_length;
      }

      // 如果没找到（理论上不应该发生），使用终点
      if (!found) {
        robot_pos = path.back();
      }

      // 创建机器人的marker（圆球）
      visualization_msgs::Marker robot_marker;
      robot_marker.header.frame_id = "map";
      robot_marker.header.stamp = ros::Time::now();
      robot_marker.ns = "robot";
      robot_marker.id = path_idx;
      robot_marker.type = visualization_msgs::Marker::SPHERE;
      robot_marker.action = visualization_msgs::Marker::ADD;
      robot_marker.lifetime = ros::Duration(0.2);  // 设置lifetime，防止marker消失

      robot_marker.pose.position.x = robot_pos(0);
      robot_marker.pose.position.y = robot_pos(1);
      robot_marker.pose.position.z = robot_pos(2);
      robot_marker.pose.orientation.w = 1.0;

      // 设置球体大小（直径 = 2 * 半径）
      robot_marker.scale.x = 2.0 * robot_radius_;
      robot_marker.scale.y = 2.0 * robot_radius_;
      robot_marker.scale.z = 2.0 * robot_radius_;

      // 为不同路径设置不同的颜色（使用HSV色彩空间生成）
      double hue = (double)path_idx / std::max((size_t)1, final_paths_.size()) * 360.0;
      double s = 1.0, v = 1.0;  // 使用高饱和度和高亮度

      // HSV转RGB
      double c = v * s;
      double x = c * (1 - std::abs(std::fmod(hue / 60.0, 2.0) - 1));
      double m = v - c;
      double r, g, b;

      if (hue < 60) { r = c; g = x; b = 0; }
      else if (hue < 120) { r = x; g = c; b = 0; }
      else if (hue < 180) { r = 0; g = c; b = x; }
      else if (hue < 240) { r = 0; g = x; b = c; }
      else if (hue < 300) { r = x; g = 0; b = c; }
      else { r = c; g = 0; b = x; }

      robot_marker.color.r = r + m;
      robot_marker.color.g = g + m;
      robot_marker.color.b = b + m;
      robot_marker.color.a = 0.9;

      robot_markers.markers.push_back(robot_marker);
    }

    // 发布所有机器人markers
    if (!robot_markers.markers.empty()) {
      // 由于publisher是单个Marker类型，我们需要分别发布每个marker
      for (const auto& marker : robot_markers.markers) {
        robot_vis_pub_.publish(marker);
      }
    }

    // 打印调试信息（每秒打印一次）
    static ros::Time last_print = ros::Time::now();
    if ((ros::Time::now() - last_print).toSec() > 1.0) {
      ROS_INFO("Robot motion: %lu robots, time=%.2f/%.2f",
               final_paths_.size(), current_time, CYCLE_TIME);
      last_print = ros::Time::now();
    }
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

    const int samples = 100;
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

  // 构造 PathAdapter，映射 s ∈ [0,1] → 空间 & 时间
  struct PathAdapter
  {
    std::function<Eigen::Vector3d(double)> posAt;
    std::function<double(double)> timeAt;
  };

  // 将 makePathAdapter 和 checkUTVD 从 lambda 改为类成员函数，避免在类作用域使用 `auto`
  PathAdapter makePathAdapter(const std::array<Eigen::Vector3d, 3> &path, const std::pair<double, double> &corr)
  {
    double t_start = corr.first;
    double t_end = corr.second;
    double T = t_end - t_start;
    PathAdapter P;
    // 捕获 path by value，t_start 和 T by value
    P.posAt = [path](double s) -> Eigen::Vector3d {
      if (s <= 0.5) {
        double u = s / 0.5;
        return (1.0 - u) * path[0] + u * path[1];
      } else {
        double u = (s - 0.5) / 0.5;
        return (1.0 - u) * path[1] + u * path[2];
      }
    };
    P.timeAt = [t_start, T](double s) -> double { return t_start + s * T; };
    return P;
  }

  // UTVD 检查函数（coarse + fine）作为类成员，可直接访问 dyn_obstacles_ 和其它成员
  bool checkUTVD(int Ns, int N_lambda, double alpha, double theta, const PathAdapter &P1, const PathAdapter &P2, double eps_dist = 1e-3)
  {
    for (int i = 0; i < Ns; ++i) {
      double s = double(i) / (Ns - 1);
      double s2 = alpha * s + theta;
      if (s2 < 0.0 - 1e-9 || s2 > 1.0 + 1e-9) return false;

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

        bool coarse_pass = checkUTVD(Ns_coarse, N_lambda_coarse, alpha, theta, P1, P2, eps_dist);
        if (!coarse_pass)
          continue;
        bool fine_pass = checkUTVD(Ns_fine, N_lambda_fine, alpha, theta, P1, P2, eps_dist);

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
    line_step_ = 0.1 * robot_speed_;

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

        // // 计算新加节点的时间安全区间
        // auto safe_intervals = computeSafeIntervals(pt, dyn_obstacles_);
        // safety_data_[guard->id_] = safe_intervals;
      } else if (visib_guards.size() == 2) {
        // **关键部分：考虑动态障碍物**
        auto visib_guard_1 = visib_guards[0];
        auto visib_guard_2 = visib_guards[1];

        // // 计算安全时间区间
        // auto safe_intervals_1 = computeSafeIntervals(visib_guard_1->pos_, dyn_obstacles_);
        // auto safe_intervals_2 = computeSafeIntervals(visib_guard_2->pos_, dyn_obstacles_);
        // auto safe_intervals_pt = computeSafeIntervals(pt, dyn_obstacles_);

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

        // // 计算新加节点的时间安全区间
        // auto safe_intervals = computeSafeIntervals(pt, dyn_obstacles_);
        // safety_data_[connector->id_] = safe_intervals;

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

    // // 可视化边
    // for (const auto& node : graph_) {
    //   for (const auto& neighbor : node->neighbors_) {
    //     // 避免重复绘制边
    //     if (node->id_ < neighbor->id_) {
    //       visualization_msgs::Marker marker;
    //       marker.header.frame_id = "map";
    //       marker.header.stamp = ros::Time::now();
    //       marker.ns = "prm_edges";
    //       marker.id = marker_id++;
    //       marker.type = visualization_msgs::Marker::LINE_STRIP;
    //       marker.action = visualization_msgs::Marker::ADD;

    //       geometry_msgs::Point p1, p2;
    //       p1.x = node->pos_(0);
    //       p1.y = node->pos_(1);
    //       p1.z = node->pos_(2);
    //       p2.x = neighbor->pos_(0);
    //       p2.y = neighbor->pos_(1);
    //       p2.z = neighbor->pos_(2);

    //       marker.points.push_back(p1);
    //       marker.points.push_back(p2);

    //       marker.scale.x = 0.05;
    //       marker.color.r = 1.0;
    //       marker.color.g = 1.0;
    //       marker.color.b = 0.0;
    //       marker.color.a = 0.6;

    //       edge_markers.markers.push_back(marker);
    //     }
    //   }
    // }

    // 可视化搜索到的路径 (filtered_paths_)
    // 可视化最终路径 (final_paths_)
    for (size_t i = 0; i < final_paths_.size(); ++i) {
      const auto& path = final_paths_[i];

      if (path.size() < 2) continue;  // 至少需要2个点才能绘制路径

      visualization_msgs::Marker path_marker;
      path_marker.header.frame_id = "map";
      path_marker.header.stamp = ros::Time::now();
      path_marker.ns = "raw_paths";
      path_marker.id = marker_id++;
      path_marker.type = visualization_msgs::Marker::LINE_STRIP;
      path_marker.action = visualization_msgs::Marker::ADD;

      // 添加路径上的所有点
      for (const auto& point : path) {
        geometry_msgs::Point p;
        p.x = point(0);
        p.y = point(1);
        p.z = point(2);
        path_marker.points.push_back(p);
      }

      // 设置路径线条属性
      path_marker.scale.x = 0.08;  // 线宽，比边稍粗一点

      // 为不同路径设置不同的颜色（使用HSV色彩空间生成）
      double hue = (double)i / std::max((size_t)1, final_paths_.size()) * 360.0;
      double s = 0.8, v = 1.0;

      // HSV转RGB
      double c = v * s;
      double x = c * (1 - std::abs(std::fmod(hue / 60.0, 2.0) - 1));
      double m = v - c;
      double r, g, b;

      if (hue < 60) { r = c; g = x; b = 0; }
      else if (hue < 120) { r = x; g = c; b = 0; }
      else if (hue < 180) { r = 0; g = c; b = x; }
      else if (hue < 240) { r = 0; g = x; b = c; }
      else if (hue < 300) { r = x; g = 0; b = c; }
      else { r = c; g = 0; b = x; }

      path_marker.color.r = r + m;
      path_marker.color.g = g + m;
      path_marker.color.b = b + m;
      path_marker.color.a = 0.8;  // 半透明

      edge_markers.markers.push_back(path_marker);
    }

    node_vis_pub_.publish(node_markers);
    graph_vis_pub_.publish(edge_markers);

    ROS_INFO("Graph visualization published (with %lu filtered paths)!", filtered_paths_.size());
  }

  // search for useful path in the topo graph by DFS
  vector<vector<Eigen::Vector3d>> searchPaths()
  {
    raw_paths_.clear();

    vector<GraphNode::Ptr> visited;
    visited.push_back(graph_.front());

    depthFirstSearch(visited);
    ROS_INFO("raw path num: %d", raw_paths_.size());
    // 检查生成的路径时间可行性
    vector<vector<Eigen::Vector3d>> time_raw_paths;
    for (const auto& path : raw_paths_)
    {
      if (!isPathTimeSafe(path))
      {
        // 如果路径不安全，则丢弃
        continue;
      }
      time_raw_paths.push_back(path);
    }
    raw_paths_ = time_raw_paths;
    ROS_INFO("after time filtered raw path num: %d", raw_paths_.size());

    // sort the path by node number
    int min_node_num = 100000, max_node_num = 1;
    vector<vector<int>> path_list(100);
    for (int i = 0; i < raw_paths_.size(); ++i)
    {
      if (int(raw_paths_[i].size()) > max_node_num)
        max_node_num = raw_paths_[i].size();
      if (int(raw_paths_[i].size()) < min_node_num)
        min_node_num = raw_paths_[i].size();
      path_list[int(raw_paths_[i].size())].push_back(i);
    }

    // select paths with less nodes
    vector<vector<Eigen::Vector3d>> filter_raw_paths;
    for (int i = min_node_num; i <= max_node_num; ++i)
    {
      bool reach_max = false;
      for (int j = 0; j < path_list[i].size(); ++j)
      {
        filter_raw_paths.push_back(raw_paths_[path_list[i][j]]);
        if (filter_raw_paths.size() >= max_raw_path2_)
        {
          reach_max = true;
          break;
        }
      }
      if (reach_max)
        break;
    }
    std::cout << ", raw path num: " << raw_paths_.size() << ", filtered raw path num: " << filter_raw_paths.size();

    raw_paths_ = filter_raw_paths;

    return raw_paths_;
  }

  void depthFirstSearch(vector<GraphNode::Ptr> &vis)
  {
    GraphNode::Ptr cur = vis.back();

    for (int i = 0; i < cur->neighbors_.size(); ++i)
    {
      // check reach goal
      if (cur->neighbors_[i]->id_ == 1)
      {
        // add this path to paths set
        vector<Eigen::Vector3d> path;
        for (int j = 0; j < vis.size(); ++j)
        {
          path.push_back(vis[j]->pos_);
        }
        path.push_back(cur->neighbors_[i]->pos_);

        raw_paths_.push_back(path);
        if (raw_paths_.size() >= max_raw_path_)
          return;

        break;
      }
    }

    for (int i = 0; i < cur->neighbors_.size(); ++i)
    {
      // skip reach goal
      if (cur->neighbors_[i]->id_ == 1)
        continue;

      // skip already visited node
      bool revisit = false;
      for (int j = 0; j < vis.size(); ++j)
      {
        if (cur->neighbors_[i]->id_ == vis[j]->id_)
        {
          revisit = true;
          break;
        }
      }
      if (revisit)
        continue;

      // recursive search
      vis.push_back(cur->neighbors_[i]);
      depthFirstSearch(vis);
      if (raw_paths_.size() >= max_raw_path_)
        return;

      vis.pop_back();
    }
  }

  bool isPathTimeSafe(const vector<Eigen::Vector3d> &path)
  {
    double travel_time = 0.0;
    double eps_dist = 1e-3;
    for (int i = 0; i < path.size() - 2; ++i)
    {
      travel_time += (path[i + 1] - path[i]).norm() / robot_speed_;
      // 检查路径上的节点是否在安全时间区间内
      for (const auto &obs : dyn_obstacles_)
      {
        if (isObstacleCollidingAtTime(obs, path[i + 1], travel_time, eps_dist))
        {
          return false;
        }
      }
    }
    return true;
  }

  vector<vector<Eigen::Vector3d>> pruneEquivalent(vector<vector<Eigen::Vector3d>> &paths)
  {
    vector<vector<Eigen::Vector3d>> pruned_paths;
    if (paths.size() < 1)
      return pruned_paths;

    /* ---------- prune topo equivalent path ---------- */
    // output: pruned_paths
    vector<int> exist_paths_id;
    exist_paths_id.push_back(0);

    for (int i = 1; i < paths.size(); ++i)
    {
      // compare with exsit paths
      bool new_path = true;

      for (int j = 0; j < exist_paths_id.size(); ++j)
      {
        // compare with one path
        bool same_topo = sameTopoPathUTVD(paths[i], paths[exist_paths_id[j]]);

        if (same_topo)
        {
          new_path = false;
          break;
        }
      }

      if (new_path)
      {
        exist_paths_id.push_back(i);
      }
    }

    // save pruned paths
    for (int i = 0; i < exist_paths_id.size(); ++i)
    {
      pruned_paths.push_back(paths[exist_paths_id[i]]);
    }

    std::cout << ", equivalent pruned path num: " << pruned_paths.size() << std::endl;
    filtered_paths_ = pruned_paths;

    return filtered_paths_;
  }

  // 检查两个路径是否在拓扑上等价（UTVD 检查）, 路径初筛选
  bool sameTopoPathUTVD(const vector<Eigen::Vector3d> &path1,
                                     const vector<Eigen::Vector3d> &path2)
  {
    if (path1.size() < 2 || path2.size() < 2)
      return false;

    // ================================
    // 1. 计算路径长度和时间尺度
    // ================================
    double L1 = pathLength(path1);
    double L2 = pathLength(path2);
    double T1 = L1 / robot_speed_;
    double T2 = L2 / robot_speed_;

    double alpha = T1 / T2;
    double theta = 0.0;

    double max_len = std::max(L1, L2);
    int N = std::ceil(max_len / line_step_);
    if (N < 2)
      N = 2;

    // ================================
    // 2. 构造路径采样函数
    // ================================
    auto interpolatePos = [&](const vector<Eigen::Vector3d> &path, double s) -> Eigen::Vector3d
    {
      if (path.size() == 1)
        return path.front();

      double total_len = 0.0;
      vector<double> cum;
      cum.push_back(0.0);
      for (int i = 1; i < path.size(); ++i)
      {
        total_len += (path[i] - path[i - 1]).norm();
        cum.push_back(total_len);
      }

      double target = s * total_len;
      for (int i = 0; i < cum.size() - 1; ++i)
      {
        if (target >= cum[i] && target <= cum[i + 1])
        {
          double t = (target - cum[i]) / (cum[i + 1] - cum[i]);
          // use .eval() to force evaluation to a concrete Eigen::Vector3d
          return ((1 - t) * path[i] + t * path[i + 1]).eval();
        }
      }
      return path.back();
    };

    auto pos1 = [&](double s)
    { return interpolatePos(path1, s); };
    auto pos2 = [&](double s)
    { return interpolatePos(path2, s); };

    auto t1 = [&](double s)
    { return s * T1; };
    auto t2 = [&](double s)
    { return s * T2; };

    // ================================
    // 3. UTVD 检查（两级采样）
    // ================================
    const int Ns_coarse = 12;
    const int N_lambda_coarse = 6;
    const int Ns_fine = 48;
    const int N_lambda_fine = 24;
    const double eps_dist = 1e-3;

    auto checkUTVD = [&](int Ns, int N_lambda)
    {
      for (int i = 0; i < Ns; ++i)
      {
        double s = double(i) / (Ns - 1);
        double s2 = alpha * s + theta;
        if (s2 < 0.0 || s2 > 1.0)
          return false;

        Eigen::Vector3d p1 = pos1(s);
        Eigen::Vector3d p2 = pos2(s2);
        double t_a = t1(s);
        double t_b = t2(s2);

        for (int j = 0; j < N_lambda; ++j)
        {
          double lambda = double(j) / (N_lambda - 1);
          Eigen::Vector3d x = (1 - lambda) * p1 + lambda * p2;
          double tau = (1 - lambda) * t_a + lambda * t_b;

          // 检查所有动态障碍物
          for (const auto &obs : dyn_obstacles_)
          {
            if (isObstacleCollidingAtTime(obs, x, tau, eps_dist))
            {
              return false;
            }
          }
        }
      }
      return true;
    };

    bool coarse_pass = checkUTVD(Ns_coarse, N_lambda_coarse);
    if (!coarse_pass)
      return false;

    bool fine_pass = checkUTVD(Ns_fine, N_lambda_fine);
    return fine_pass;
  }

  vector<vector<Eigen::Vector3d>> selectShortPaths(vector<vector<Eigen::Vector3d>> &paths,
                                                                int step)
  {
    /* ---------- only reserve top short path ---------- */
    vector<vector<Eigen::Vector3d>> short_paths;
    vector<Eigen::Vector3d> short_path;
    double min_len;

    for (int i = 0; i < reserve_num_ && paths.size() > 0; ++i)
    {
      int path_id = shortestPath(paths); // 找到当前最短路径的ID
      if (i == 0)
      {
        short_paths.push_back(paths[path_id]);
        min_len = pathLength(paths[path_id]); // 记录最短路径长度作为基准
        paths.erase(paths.begin() + path_id); // 从候选路径中移除已选路径
      }
      else
      {
        double rat = pathLength(paths[path_id]) / min_len; // 计算与最短路径的长度比
        if (rat < ratio_to_short_)
        { // 如果比例小于阈值，则选择该路径
          short_paths.push_back(paths[path_id]);
          paths.erase(paths.begin() + path_id);
        }
        else
        {
          break; // 如果路径太长，提前结束选择
        }
      }
    }
    std::cout << ", final select path num: " << short_paths.size();

    short_paths = pruneEquivalent(short_paths);

    return short_paths;
  }

  int shortestPath(vector<vector<Eigen::Vector3d>> &paths)
  {
    int short_id = -1;
    double min_len = 100000000;
    for (int i = 0; i < paths.size(); ++i)
    {
      double len = pathLength(paths[i]);
      if (len < min_len)
      {
        short_id = i;
        min_len = len;
      }
    }
    return short_id;
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

    // 搜索路径
    ROS_INFO("\nSearching for paths...");
    raw_paths_ = searchPaths();

    filtered_paths_ = pruneEquivalent(raw_paths_);

    final_paths_ = selectShortPaths(filtered_paths_, 1);

    visualizeGraph();

    ROS_INFO("\n=== Test Complete ===");
    ROS_INFO("Check RViz to see:");
    ROS_INFO("  - Dynamic obstacle trajectories (orange lines)");
    ROS_INFO("  - Dynamic obstacles (red spheres)");
    ROS_INFO("  - Static obstacles (gray spheres)");
    ROS_INFO("  - PRM graph nodes (blue=guards, green=connectors)");
    ROS_INFO("  - PRM graph edges (yellow lines)");
    ROS_INFO("  - Robot sphere (cyan) moving along the first path");
    ROS_INFO("\nThe graph should avoid both static and dynamic obstacles,");
    ROS_INFO("and demonstrate time-homotopic topology classes.");
    ROS_INFO("The robot will follow the first selected path with time synchronized to obstacles.\n");

    // 保持运行以持续发布可视化
    while (ros::ok()) {
      visualizeDynamicObstacles();
      visualizeGraph();
      visualizeRobotMotion();  // 可视化机器人沿路径运动
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
