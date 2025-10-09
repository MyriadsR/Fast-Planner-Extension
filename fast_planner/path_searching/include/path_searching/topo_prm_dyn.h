/*
 * @Author: xzr && 1841953204@qq.com
 * @Date: 2025-10-09 10:44:02
 * @LastEditors: xzr && 1841953204@qq.com
 * @LastEditTime: 2025-10-09 19:44:17
 * @FilePath: /Fast_Planner_ws/src/Fast-Planner-Extension/fast_planner/path_searching/include/path_searching/topo_prm_dyn.h
 * @Description: 动态拓扑PRM
 * 
 * Copyright (c) 2025 by 1841953204@qq.com, All Rights Reserved. 
 */

#ifndef _TOPO_PRM_DYN_H
#define _TOPO_PRM_DYN_H

#include <ros/ros.h>
#include <tprm/obstacle_impl.h>
#include <tprm/temporal_prm.h>
#include <obj_state_msgs/ObjectsStates.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <std_msgs/ColorRGBA.h>

#include <iostream>
#include <vector>
#include <memory>
#include <algorithm>
#include <cmath>
#include <random>

namespace topo_dyn_planner {

/* ---------- used for iterating all topo combination ---------- */
class TopoIterator {
private:
  /* data */
  vector<int> path_nums_;
  vector<int> cur_index_;
  int combine_num_;
  int cur_num_;

  void increase(int bit_num) {
    cur_index_[bit_num] += 1;
    if (cur_index_[bit_num] >= path_nums_[bit_num]) {
      cur_index_[bit_num] = 0;
      increase(bit_num + 1);
    }
  }

public:
  TopoIterator(vector<int> pn) {
    path_nums_ = pn;
    cur_index_.resize(path_nums_.size());
    fill(cur_index_.begin(), cur_index_.end(), 0);
    cur_num_ = 0;

    combine_num_ = 1;
    for (int i = 0; i < path_nums_.size(); ++i) {
      combine_num_ *= path_nums_[i] > 0 ? path_nums_[i] : 1;
    }
    std::cout << "[Topo]: merged path num: " << combine_num_ << std::endl;
  }
  TopoIterator() {
  }
  ~TopoIterator() {
  }

  bool nextIndex(vector<int>& index) {
    index = cur_index_;
    cur_num_ += 1;

    if (cur_num_ == combine_num_) return false;

    // go to next combination
    increase(0);
    return true;
  }
};

/* ---------- node of topo graph ---------- */
class GraphNode {
private:
  /* data */

public:
  enum NODE_TYPE { Guard = 1, Connector = 2 };

  enum NODE_STATE { NEW = 1, CLOSE = 2, OPEN = 3 };

  GraphNode(/* args */) {
  }
  GraphNode(Eigen::Vector3d pos, NODE_TYPE type, int id) {
    pos_ = pos;
    type_ = type;
    state_ = NEW;
    id_ = id;
  }
  ~GraphNode() {
  }

  vector<shared_ptr<GraphNode>> neighbors_;
  Eigen::Vector3d pos_;
  NODE_TYPE type_;
  NODE_STATE state_;
  int id_;

  typedef shared_ptr<GraphNode> Ptr;
};

class TopologyPRM {
private:
  /* basic data */
  Eigen::Vector3d start_pos_;
  Eigen::Vector3d goal_pos_;
  double robot_speed_;    // 机器人速度
  double line_step_;    // 直线采样步长

  /* obstacle data */
  std::vector<std::shared_ptr<tprm::StaticObstacle>> sta_obstacles_;    // 静态障碍物
  std::vector<std::shared_ptr<tprm::DynamicSphereObstacle>> dyn_obstacles_;    // 动态障碍物
  double safety_margin_;    // 安全裕度

  /* topic */
  ros::Subscriber dyn_obs_sub_;
  ros::Subscriber goal_sub_;

  // sampling generator
  random_device rd_;
  default_random_engine eng_;
  uniform_real_distribution<double> rand_pos_;

  Eigen::Vector3d sample_r_;
  Eigen::Vector3d translation_;
  Eigen::Matrix3d rotation_;

  // roadmap data structure, 0:start, 1:goal, 2-n: others
  list<GraphNode::Ptr> graph_;
  vector<vector<Eigen::Vector3d>> raw_paths_;
  vector<vector<Eigen::Vector3d>> short_paths_;
  vector<vector<Eigen::Vector3d>> final_paths_;
  vector<Eigen::Vector3d> start_pts_, end_pts_;

//   // raycasting
//   vector<RayCaster> casters_;
//   Eigen::Vector3d offset_;

  // parameter
  double max_sample_time_;
  int max_sample_num_;
  int max_raw_path_, max_raw_path2_;
  int short_cut_num_;
  Eigen::Vector3d sample_inflate_;
  double resolution_;

  double ratio_to_short_;
  int reserve_num_;

  bool parallel_shortcut_;

  /* create topological roadmap */
  /* path searching, shortening, pruning and merging */
  list<GraphNode::Ptr> createGraph(Eigen::Vector3d start, Eigen::Vector3d end);
  vector<vector<Eigen::Vector3d>> searchPaths();
  void shortcutPaths();
  vector<vector<Eigen::Vector3d>> pruneEquivalent(vector<vector<Eigen::Vector3d>>& paths);
  vector<vector<Eigen::Vector3d>> selectShortPaths(vector<vector<Eigen::Vector3d>>& paths, int step);

  /* ---------- helper ---------- */
  inline Eigen::Vector3d getSample();
  vector<GraphNode::Ptr> findVisibGuard(Eigen::Vector3d pt);  // find pairs of visibile guard
  bool needConnection(GraphNode::Ptr g1, GraphNode::Ptr g2,
                      Eigen::Vector3d pt);  // test redundancy with existing
                                            // connection between two guard
  bool lineVisib(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2, double thresh,
                 Eigen::Vector3d& pc, int caster_id = 0);
  bool triangleVisib(Eigen::Vector3d pt, Eigen::Vector3d p1, Eigen::Vector3d p2);
  void pruneGraph();

  void depthFirstSearch(vector<GraphNode::Ptr>& vis);

  vector<Eigen::Vector3d> discretizeLine(Eigen::Vector3d p1, Eigen::Vector3d p2);
  vector<vector<Eigen::Vector3d>> discretizePaths(vector<vector<Eigen::Vector3d>>& path);

  vector<Eigen::Vector3d> discretizePath(vector<Eigen::Vector3d> path);
  void shortcutPath(vector<Eigen::Vector3d> path, int path_id, int iter_num = 1);

  vector<Eigen::Vector3d> discretizePath(const vector<Eigen::Vector3d>& path, int pt_num);
  bool sameTopoPath(const vector<Eigen::Vector3d>& path1, const vector<Eigen::Vector3d>& path2,
                    double thresh);
  Eigen::Vector3d getOrthoPoint(const vector<Eigen::Vector3d>& path);

  int shortestPath(vector<vector<Eigen::Vector3d>>& paths);
  
  /* callback function */
  void dynObstaclesCallback(const obj_msgs::ObjStates::ConstPtr& msg);
  void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
  
public:
  double clearance_;

  TopologyPRM(/* args */);
  ~TopologyPRM();

  void init(ros::NodeHandle& nh);

//   void setEnvironment(const EDTEnvironment::Ptr& env);

  void findTopoPaths(Eigen::Vector3d start, Eigen::Vector3d end, vector<Eigen::Vector3d> start_pts,
                     vector<Eigen::Vector3d> end_pts, list<GraphNode::Ptr>& graph,
                     vector<vector<Eigen::Vector3d>>& raw_paths,
                     vector<vector<Eigen::Vector3d>>& filtered_paths,
                     vector<vector<Eigen::Vector3d>>& select_paths);

  double pathLength(const vector<Eigen::Vector3d>& path);
  vector<Eigen::Vector3d> pathToGuidePts(vector<Eigen::Vector3d>& path, int pt_num);

};

}  // namespace topo_dyn_planner

#endif
