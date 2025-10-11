/*
 * @Author: Test PRM Graph
 * @Description: Test createGraph function from topo_prm_dyn and visualize in RViz
 */

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <obj_state_msgs/ObjectsStates.h>
#include <tprm/obstacle_impl.h>

#include <iostream>
#include <vector>
#include <list>
#include <memory>
#include <random>
#include <Eigen/Eigen>

/* ---------- GraphNode definition (从topo_prm_dyn.h复制) ---------- */
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

/* ---------- PRM测试类 ---------- */
class PRMTester {
private:
  ros::NodeHandle nh_;
  ros::Publisher graph_vis_pub_;
  ros::Publisher node_vis_pub_;
  ros::Subscriber static_obs_sub_;
  ros::Subscriber dyn_obs_sub_;
  ros::Subscriber goal_sub_;

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

  std::list<GraphNode::Ptr> graph_;
  std::vector<std::shared_ptr<tprm::StaticSphereObstacle>> sta_obstacles_;
  std::vector<std::shared_ptr<tprm::DynamicSphereObstacle>> dyn_obstacles_;
  std::vector<std::shared_ptr<tprm::DynamicSphereObstacle>> effective_dyn_obstacles_;  // 用于createGraph的有效障碍物列表

  // 存储节点的安全时间区间
  std::map<int, std::vector<std::pair<double, double>>> safety_data_;
  std::vector<std::pair<double, double>> safe_edge_windows_1_;
  std::vector<std::pair<double, double>> safe_edge_windows_2_;
  std::vector<std::pair<double, double>> safe_edge_windows_3_;
  std::vector<std::pair<double, double>> safe_edge_windows_4_;

  bool has_goal_;
  bool has_obstacles_;

public:
  PRMTester(ros::NodeHandle& nh) : nh_(nh), has_goal_(false), has_obstacles_(false) {
    // 初始化发布器
    graph_vis_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/prm_graph", 10);
    node_vis_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/prm_nodes", 10);

    // 初始化订阅器
    static_obs_sub_ = nh_.subscribe("/static_obj_states", 10, &PRMTester::staticObsCallback, this);
    dyn_obs_sub_ = nh_.subscribe("/obj_states", 10, &PRMTester::dynObsCallback, this);
    goal_sub_ = nh_.subscribe("/move_base_simple/goal", 1, &PRMTester::goalCallback, this);

    // 初始化随机数生成器
    eng_ = std::default_random_engine(rd_());
    rand_pos_ = std::uniform_real_distribution<double>(-1.0, 1.0);

    // 读取参数
    nh_.param("robot_speed", robot_speed_, 0.5);
    nh_.param("max_sample_time", max_sample_time_, 0.5);
    nh_.param("max_sample_num", max_sample_num_, 1000);
    nh_.param("safety_margin", safety_margin_, 0.2);

    std::vector<double> inflate;
    nh_.param("sample_inflate_x", inflate, std::vector<double>());
    if (inflate.size() >= 3) {
      sample_inflate_ = Eigen::Vector3d(inflate[0], inflate[1], inflate[2]);
    } else {
      sample_inflate_ = Eigen::Vector3d(2.0, 2.0, 1.0);
    }

    line_step_ = 0.5 * robot_speed_;

    // 设置起点
    start_pos_ = Eigen::Vector3d(0.0, 0.0, 1.0);

    ROS_INFO("PRM Tester initialized!");
    ROS_INFO("Waiting for obstacles and goal...");
    ROS_INFO("Please click '2D Nav Goal' in RViz to set the goal position");
  }

  void staticObsCallback(const obj_state_msgs::ObjectsStates::ConstPtr& msg) {
    sta_obstacles_.clear();

    for (const auto& state : msg->states) {
      Eigen::Vector3d position(state.position.x, state.position.y, state.position.z);
      double radius = state.size.x / 2.0;

      auto obstacle = std::make_shared<tprm::StaticSphereObstacle>(position, radius);
      sta_obstacles_.push_back(obstacle);
    }

    has_obstacles_ = true;
    ROS_INFO_THROTTLE(5.0, "Received %lu static obstacles", sta_obstacles_.size());
  }

  void dynObsCallback(const obj_state_msgs::ObjectsStates::ConstPtr& msg) {
    dyn_obstacles_.clear();

    for (const auto& state : msg->states) {
      Eigen::Vector3d position(state.position.x, state.position.y, state.position.z);
      Eigen::Vector3d velocity(state.velocity.x, state.velocity.y, state.velocity.z);
      double radius = state.size.x / 2.0 + safety_margin_;

      auto obstacle = std::make_shared<tprm::DynamicSphereObstacle>(position, velocity, radius);
      dyn_obstacles_.push_back(obstacle);
    }

    ROS_INFO_THROTTLE(2.0, "Received %lu dynamic obstacles with safety margin", dyn_obstacles_.size());
  }

  void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    goal_pos_ = Eigen::Vector3d(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    ROS_INFO("New goal received: [%.2f, %.2f, %.2f]", goal_pos_[0], goal_pos_[1], goal_pos_[2]);
    has_goal_ = true;

    // 如果有障碍物和目标，就创建图
    if (has_obstacles_) {
      createAndVisualizeGraph();
    }
  }

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

    for (int i = 0; i < path.size() - 1; ++i) {
      length += (path[i + 1] - path[i]).norm();
    }
    return length;
  }

  std::vector<Eigen::Vector3d> discretizePath(const std::vector<Eigen::Vector3d>& path, int pt_num) {
    std::vector<double> len_list;
    len_list.push_back(0.0);

    for (int i = 0; i < path.size() - 1; ++i) {
      double inc_l = (path[i + 1] - path[i]).norm();
      len_list.push_back(inc_l + len_list[i]);
    }

    double len_total = len_list.back();
    double dl = len_total / double(pt_num - 1);

    std::vector<Eigen::Vector3d> dis_path;
    for (int i = 0; i < pt_num; ++i) {
      double cur_l = double(i) * dl;

      int idx = -1;
      for (int j = 0; j < len_list.size() - 1; ++j) {
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

  bool needConnection(GraphNode::Ptr g1, GraphNode::Ptr g2, Eigen::Vector3d pt) {
    std::vector<Eigen::Vector3d> path1(3), path2(3);
    GraphNode::Ptr connector;
    path1[0] = g1->pos_;
    path1[1] = pt;
    path1[2] = g2->pos_;

    path2[0] = g1->pos_;
    path2[2] = g2->pos_;

    // 调试：统计共同邻居的数量
    static int need_conn_calls = 0;
    static int found_common_neighbor = 0;
    need_conn_calls++;

    for (int i = 0; i < g1->neighbors_.size(); ++i) {
      for (int j = 0; j < g2->neighbors_.size(); ++j) {
        if (g1->neighbors_[i]->id_ == g2->neighbors_[j]->id_) {
          found_common_neighbor++;

          // 每100次输出一次统计
          if (need_conn_calls % 100 == 0) {
            ROS_INFO("[needConnection] Called %d times, found common neighbor %d times (%.1f%%)",
                     need_conn_calls, found_common_neighbor,
                     100.0 * found_common_neighbor / need_conn_calls);
          }

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

    // ⚠️ 限制预测时间
    const double max_prediction_time = 10.0;

    // 收集所有障碍物的碰撞时间间隔
    for (const auto& obstacle : obstacles) {
      double hit_time_from, hit_time_to;
      if (obstacle->isColliding(position, hit_time_from, hit_time_to)) {
        // 只考虑max_prediction_time内的碰撞
        if (hit_time_from < max_prediction_time) {
          hit_time_to = std::min(hit_time_to, max_prediction_time);
          collision_intervals.emplace_back(hit_time_from, hit_time_to);
        }
      }
    }

    // 合并重叠的碰撞间隔
    collision_intervals = mergeIntervals(collision_intervals);

    // 从碰撞间隔推导安全间隔
    if (collision_intervals.empty()) {
      safe_intervals.emplace_back(0.0, max_prediction_time);
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

      if (collision_intervals.back().second < max_prediction_time) {
        safe_intervals.emplace_back(collision_intervals.back().second, max_prediction_time);
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

    // ⚠️ 关键修复：限制预测时间范围
    // 由于DynamicSphereObstacle假设直线运动，我们只预测未来有限时间
    const double max_prediction_time = 5.0;  // 从10秒减少到5秒

    for (const auto& obstacle : obstacles) {
      auto window = computeObstacleCollisionWindow(from, to, obstacle, robot_speed);
      if (window.first < window.second) {
        // 限制碰撞窗口的结束时间
        if (window.first < max_prediction_time) {
          window.second = std::min(window.second, max_prediction_time);
          collision_windows.push_back(window);
        }
      }
    }

    collision_windows = mergeIntervals(collision_windows);

    if (collision_windows.empty()) {
      // 未来max_prediction_time内都安全
      safe_windows.emplace_back(0.0, max_prediction_time);
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

      // 最后一个碰撞窗口之后到max_prediction_time的时间
      if (collision_windows.back().second < max_prediction_time) {
        safe_windows.emplace_back(collision_windows.back().second, max_prediction_time);
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

  /**
   * @brief 检查动态障碍物在特定时间是否与给定位置碰撞
   * @param obstacle 动态障碍物
   * @param position 空间位置
   * @param time 检查的时间点
   * @param eps_dist 距离容差
   * @return true 如果在该时间点发生碰撞，false 否则
   */
  bool isObstacleCollidingAtTime(
      const std::shared_ptr<tprm::DynamicSphereObstacle>& obstacle,
      const Eigen::Vector3d& position,
      double time,
      double eps_dist = 1e-3) const {
    // 获取障碍物与该位置的碰撞时间区间
    double hit_time_from, hit_time_to;
    if (!obstacle->isColliding(position, hit_time_from, hit_time_to)) {
      // 障碍物永远不会与该位置碰撞
      return false;
    }

    // 检查给定时间是否在碰撞时间区间内（考虑容差）
    return (time >= hit_time_from - eps_dist && time <= hit_time_to + eps_dist);
  }

  bool sameTopoPathUTVD(const GraphNode::Ptr guard1, const GraphNode::Ptr guard2,
                        const GraphNode::Ptr connector, const Eigen::Vector3d pt) {
    // 构造路径
    std::array<Eigen::Vector3d, 3> path_new = {guard1->pos_, pt, guard2->pos_};
    std::array<Eigen::Vector3d, 3> path_exist = {guard1->pos_, connector->pos_, guard2->pos_};

    // 计算边的安全时间窗口
    safe_edge_windows_3_ = computeEdgeSafeWindow(guard1->pos_, connector->pos_, effective_dyn_obstacles_, robot_speed_);
    safe_edge_windows_4_ = computeEdgeSafeWindow(connector->pos_, guard2->pos_, effective_dyn_obstacles_, robot_speed_);

    // 计算两个边的安全时间窗口的交集
    auto corridors1 = intersectIntervals(safe_edge_windows_1_, safe_edge_windows_2_);
    auto corridors2 = intersectIntervals(safe_edge_windows_3_, safe_edge_windows_4_);

    // 调试：记录时间走廊信息（每100次输出一次避免刷屏）
    static int utvd_call_count = 0;
    if (++utvd_call_count % 100 == 0) {
      ROS_INFO("[UTVD Debug #%d] corridors1 size: %lu, corridors2 size: %lu",
               utvd_call_count, corridors1.size(), corridors2.size());
    }

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

              for (const auto &obs : effective_dyn_obstacles_) {
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
          // 调试：记录UTVD成功的情况（前10次）
          static int utvd_success_count = 0;
          if (++utvd_success_count <= 10) {
            ROS_WARN("[UTVD Success #%d] Same topology detected!", utvd_success_count);
            ROS_WARN("  Corridor1: [%.2f, %.2f], T1=%.2f", c1.first, c1.second, T1);
            ROS_WARN("  Corridor2: [%.2f, %.2f], T2=%.2f", c2.first, c2.second, T2);
            ROS_WARN("  Alpha=%.3f, Theta=%.3f", alpha, theta);
          }
          return true;
        }
      }
    }

    return false;
  }

  std::list<GraphNode::Ptr> createGraph() {
    ROS_INFO("\n=== Starting PRM Graph Creation ===");

    // **测试开关：禁用动态障碍物以进行对比**
    bool enable_dynamic_obstacles = true;  // 改为false可以禁用动态障碍物
    nh_.param("enable_dynamic_obstacles", enable_dynamic_obstacles, true);

    // **关键修复：检查动态障碍物数据**
    ROS_INFO("Dynamic obstacles count: %lu", dyn_obstacles_.size());
    if (dyn_obstacles_.empty() || !enable_dynamic_obstacles) {
      if (!enable_dynamic_obstacles) {
        ROS_WARN("⚠️  Dynamic obstacles DISABLED for testing!");
      } else {
        ROS_WARN("⚠️  No dynamic obstacles received! Graph will only avoid static obstacles.");
        ROS_WARN("⚠️  Make sure /obj_states topic is publishing.");
      }
    } else {
      ROS_INFO("✓ Creating graph with %lu dynamic obstacles", dyn_obstacles_.size());
      // 打印前3个动态障碍物的初始位置（时间t=0）
      for (size_t i = 0; i < std::min(size_t(3), dyn_obstacles_.size()); i++) {
        Eigen::Vector3d pos_t0 = dyn_obstacles_[i]->getCOM(0.0);
        Eigen::Vector3d vel = dyn_obstacles_[i]->getVelocity();
        ROS_INFO("  Dyn obs %zu: pos(t=0)=[%.2f, %.2f, %.2f], vel=[%.2f, %.2f, %.2f]",
                 i, pos_t0(0), pos_t0(1), pos_t0(2), vel(0), vel(1), vel(2));
      }
    }

    // 如果禁用动态障碍物，清空列表
    effective_dyn_obstacles_ = enable_dynamic_obstacles ? dyn_obstacles_ :
                               std::vector<std::shared_ptr<tprm::DynamicSphereObstacle>>();

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

    // 统计计数器（用于调试动态障碍物处理）
    int rejected_by_time_window = 0;  // 被时间窗口拒绝的连接数
    int rejected_by_topology = 0;      // 被拓扑检查拒绝的连接数
    int accepted_connections = 0;       // 接受的连接数

    ROS_INFO("Sample region: [%.2f, %.2f, %.2f]", sample_r_(0), sample_r_(1), sample_r_(2));
    ROS_INFO("Translation: [%.2f, %.2f, %.2f]", translation_(0), translation_(1), translation_(2));

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
        // 考虑动态障碍物时,需要检查两个Guard和新采样点形成的两条线段的安全区间是否有重叠
        // 计算2个Guard节点和新采样点的安全时间区间
        auto visib_guard_1 = visib_guards[0];
        auto visib_guard_2 = visib_guards[1];
        auto safe_intervals_1 = computeSafeIntervals(visib_guard_1->pos_, effective_dyn_obstacles_);
        auto safe_intervals_2 = computeSafeIntervals(visib_guard_2->pos_, effective_dyn_obstacles_);
        auto safe_intervals_pt = computeSafeIntervals(pt, effective_dyn_obstacles_);

        // 将安全时间区间绑定guard节点
        safety_data_[visib_guard_1->id_] = safe_intervals_1;
        safety_data_[visib_guard_2->id_] = safe_intervals_2;

        // 检查边的安全时间窗口
        safe_edge_windows_1_ = computeEdgeSafeWindow(
            visib_guard_1->pos_, pt, effective_dyn_obstacles_, robot_speed_);
        safe_edge_windows_2_ = computeEdgeSafeWindow(
            pt, visib_guard_2->pos_, effective_dyn_obstacles_, robot_speed_);

        // 验证调试：输出前5个边的时间窗口详情
        static int edge_check_count = 0;
        if (++edge_check_count <= 5) {
          ROS_INFO("[Edge Verification #%d] Edge1 windows: %lu, Edge2 windows: %lu",
                   edge_check_count, safe_edge_windows_1_.size(), safe_edge_windows_2_.size());
          if (!safe_edge_windows_1_.empty()) {
            auto& w = safe_edge_windows_1_[0];
            ROS_INFO("  Edge1 first window: [%.2f, %.2f], duration=%.2f%s",
                     w.first, w.second, w.second - w.first,
                     std::isinf(w.second) ? " (INFINITE!)" : "");
          }
        }

        // 如果两个边的安全时间窗口没有交集，则继续采样
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

        // 如果两个边的安全时间窗口没有交集，则继续采样
        if (!intervalsOverlap(safe_edge_windows_1_, safe_edge_windows_2_)) {
          rejected_by_time_window++;
          sample_time += (ros::Time::now() - t1).toSec();
          continue;
        }

        // 判断新路径和已有路径是否同拓扑
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

    // 输出动态障碍物处理统计
    ROS_INFO("\n=== Dynamic Obstacle Processing Statistics ===");
    ROS_INFO("  Rejected by time window: %d", rejected_by_time_window);
    ROS_INFO("  Rejected by topology check: %d", rejected_by_topology);
    ROS_INFO("  Accepted connections: %d", accepted_connections);
    int total_2guard_samples = rejected_by_time_window + rejected_by_topology + accepted_connections;
    ROS_INFO("  Total 2-guard samples: %d", total_2guard_samples);
    if (total_2guard_samples > 0) {
      ROS_INFO("  Time window rejection rate: %.1f%%",
               100.0 * rejected_by_time_window / total_2guard_samples);
      ROS_INFO("  Topology rejection rate: %.1f%%",
               100.0 * rejected_by_topology / total_2guard_samples);
      ROS_INFO("  Acceptance rate: %.1f%%",
               100.0 * accepted_connections / total_2guard_samples);
    }

    ROS_INFO("Sample num: %d, Sample time: %.3f s", sample_num, sample_time);

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

  void createAndVisualizeGraph() {
    if (!has_obstacles_ || !has_goal_) {
      ROS_WARN("Missing obstacles or goal, cannot create graph!");
      return;
    }

    // 创建图
    createGraph();

    // 可视化
    visualizeGraph();
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "test_prm_graph");
  ros::NodeHandle nh("~");

  PRMTester tester(nh);

  ROS_INFO("PRM Graph Tester is running...");
  ROS_INFO("1. Make sure static obstacles are published to /static_obj_states");
  ROS_INFO("2. Click '2D Nav Goal' in RViz to set goal and trigger graph creation");

  ros::spin();

  return 0;
}
