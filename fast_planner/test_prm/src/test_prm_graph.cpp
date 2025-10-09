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

  std::list<GraphNode::Ptr> graph_;
  std::vector<std::shared_ptr<tprm::StaticSphereObstacle>> sta_obstacles_;

  bool has_goal_;
  bool has_obstacles_;

public:
  PRMTester(ros::NodeHandle& nh) : nh_(nh), has_goal_(false), has_obstacles_(false) {
    // 初始化发布器
    graph_vis_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/prm_graph", 10);
    node_vis_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/prm_nodes", 10);

    // 初始化订阅器
    static_obs_sub_ = nh_.subscribe("/static_obj_states", 10, &PRMTester::staticObsCallback, this);
    goal_sub_ = nh_.subscribe("/move_base_simple/goal", 1, &PRMTester::goalCallback, this);

    // 初始化随机数生成器
    eng_ = std::default_random_engine(rd_());
    rand_pos_ = std::uniform_real_distribution<double>(-1.0, 1.0);

    // 读取参数
    nh_.param("robot_speed", robot_speed_, 0.5);
    nh_.param("max_sample_time", max_sample_time_, 0.5);
    nh_.param("max_sample_num", max_sample_num_, 1000);

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
    path1[0] = g1->pos_;
    path1[1] = pt;
    path1[2] = g2->pos_;

    path2[0] = g1->pos_;
    path2[2] = g2->pos_;

    for (int i = 0; i < g1->neighbors_.size(); ++i) {
      for (int j = 0; j < g2->neighbors_.size(); ++j) {
        if (g1->neighbors_[i]->id_ == g2->neighbors_[j]->id_) {
          path2[1] = g1->neighbors_[i]->pos_;
          bool same_topo = sameTopoPath(path1, path2);
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

  std::list<GraphNode::Ptr> createGraph() {
    ROS_INFO("\n=== Starting PRM Graph Creation ===");
    graph_.clear();

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

    ROS_INFO("Sample region: [%.2f, %.2f, %.2f]", sample_r_(0), sample_r_(1), sample_r_(2));
    ROS_INFO("Translation: [%.2f, %.2f, %.2f]", translation_(0), translation_(1), translation_(2));

    // 主采样循环
    ros::Time t_start = ros::Time::now();
    while (sample_time < max_sample_time_ && sample_num < max_sample_num_) {
      ros::Time t1 = ros::Time::now();

      Eigen::Vector3d pt = getSample();
      ++sample_num;

      // 检查采样点是否在障碍物内
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
        // 尝试在两个guard之间添加连接
        bool need_connect = needConnection(visib_guards[0], visib_guards[1], pt);
        if (!need_connect) {
          sample_time += (ros::Time::now() - t1).toSec();
          continue;
        }

        // 创建新的connector节点
        GraphNode::Ptr connector = GraphNode::Ptr(new GraphNode(pt, GraphNode::Connector, ++node_id));
        graph_.push_back(connector);

        // 连接guards
        visib_guards[0]->neighbors_.push_back(connector);
        visib_guards[1]->neighbors_.push_back(connector);
        connector->neighbors_.push_back(visib_guards[0]);
        connector->neighbors_.push_back(visib_guards[1]);
      }

      sample_time += (ros::Time::now() - t1).toSec();
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
