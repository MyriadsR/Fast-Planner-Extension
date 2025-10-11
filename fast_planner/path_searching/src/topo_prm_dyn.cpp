/*
 * @Author: xzr && 1841953204@qq.com
 * @Date: 2025-10-08 22:49:40
 * @LastEditors: xzr && 1841953204@qq.com
 * @LastEditTime: 2025-10-11 21:31:05
 * @FilePath: /Fast_Planner_ws/src/Fast-Planner-Extension/fast_planner/path_searching/src/topo_prm_dyn.cpp
 * @Description: 动态拓扑PRM
 * 
 * Copyright (c) 2025 by 1841953204@qq.com, All Rights Reserved. 
 */

#include <path_searching/topo_prm_dyn.h>
#include <thread>

namespace topo_dyn_planner {
TopologyPRM::TopologyPRM(/* args */) {}

TopologyPRM::~TopologyPRM() {}

void TopologyPRM::init(ros::NodeHandle& nh) {
  graph_.clear();
  eng_ = default_random_engine(rd_());
  rand_pos_ = uniform_real_distribution<double>(-1.0, 1.0);

  // init parameter
}

void TopologyPRM::findTopoPaths(
    Eigen::Vector3d start,           // 起点坐标
    Eigen::Vector3d end,             // 终点坐标  
    vector<Eigen::Vector3d> start_pts, // 起点附近采样点
    vector<Eigen::Vector3d> end_pts,   // 终点附近采样点
    list<GraphNode::Ptr>& graph,      // 输出的PRM图结构
    vector<vector<Eigen::Vector3d>>& raw_paths,     // 原始搜索路径
    vector<vector<Eigen::Vector3d>>& filtered_paths, // 过滤后的路径
    vector<vector<Eigen::Vector3d>>& select_paths   // 最终选择的路径
)
{
  ros::Time t1, t2;

  double graph_time, search_time, short_time, prune_time, select_time;
  /* ---------- create the topo graph ---------- */
  t1 = ros::Time::now();

  start_pts_ = start_pts;
  end_pts_ = end_pts_;
  
  // 设置机器人速度
  tprm::HolonomicRobot::movement_speed = robot_speed_;

  graph = createGraph(start, end);

  graph_time = (ros::Time::now() - t1).toSec();

  /* ---------- search paths in the graph ---------- */
  t1 = ros::Time::now();

  raw_paths = searchPaths();

  search_time = (ros::Time::now() - t1).toSec();

  /* ---------- path shortening ---------- */
  // for parallel, save result in short_paths_
  t1 = ros::Time::now();

  shortcutPaths();

  short_time = (ros::Time::now() - t1).toSec();

  /* ---------- prune equivalent paths ---------- */
  t1 = ros::Time::now();

  filtered_paths = pruneEquivalent(short_paths_);

  prune_time = (ros::Time::now() - t1).toSec();
  // cout << "prune: " << (t2 - t1).toSec() << endl;

  /* ---------- select N shortest paths ---------- */
  t1 = ros::Time::now();

  select_paths = selectShortPaths(filtered_paths, 1);

  select_time = (ros::Time::now() - t1).toSec();

  final_paths_ = select_paths;

  double total_time = graph_time + search_time + short_time + prune_time + select_time;

  std::cout << "\n[Topo]: total time: " << total_time << ", graph: " << graph_time
            << ", search: " << search_time << ", short: " << short_time << ", prune: " << prune_time
            << ", select: " << select_time << std::endl;
}

list<GraphNode::Ptr> TopologyPRM::createGraph(Eigen::Vector3d start, Eigen::Vector3d end)
{
  // std::cout << "[Topo]: searching----------------------" << std::endl;

  /* init the start, end and sample region */
  graph_.clear();
  line_step_ = 0.5 * robot_speed_; // 初始步长为机器人速度的一半
  // collis_.clear();

  GraphNode::Ptr start_node = GraphNode::Ptr(new GraphNode(start, GraphNode::Guard, 0));
  GraphNode::Ptr end_node = GraphNode::Ptr(new GraphNode(end, GraphNode::Guard, 1));

  graph_.push_back(start_node);
  graph_.push_back(end_node);

  // sample region
  sample_r_(0) = 0.5 * (end - start).norm() + sample_inflate_(0);
  sample_r_(1) = sample_inflate_(1);
  sample_r_(2) = sample_inflate_(2);

  // transformation
  translation_ = 0.5 * (start + end);

  Eigen::Vector3d xtf, ytf, ztf, downward(0, 0, -1);
  xtf = (end - translation_).normalized();
  ytf = xtf.cross(downward).normalized();
  ztf = xtf.cross(ytf);

  rotation_.col(0) = xtf;
  rotation_.col(1) = ytf;
  rotation_.col(2) = ztf;

  int node_id = 1;

  /* ---------- main loop ---------- */
  int sample_num = 0;
  double sample_time = 0.0;
  Eigen::Vector3d pt;
  ros::Time t1, t2;
  while (sample_time < max_sample_time_ && sample_num < max_sample_num_)
  {
    t1 = ros::Time::now();

    pt = getSample();
    ++sample_num;
    // double dist;
    // // edt_environment_->evaluateEDTWithGrad(pt, -1.0, dist, grad);
    // dist = edt_environment_->evaluateCoarseEDT(pt, -1.0);

    // 订阅障碍物状态
    static_obs_sub_ = nh_.subscribe("/static_obj_states", 10, &TopologyPRM::staticObsCallback, this);
    dyn_obs_sub_ = nh_.subscribe("/obj_states", 10, &TopologyPRM::dynObstaclesCallback, this);
    goal_sub_ = nh_.subscribe("/move_base_simple/goal", 1, &TopologyPRM::goalCallback, this);

    // 检查采样点是否与在静态障碍物中
    bool is_blocked = false;
    for (const auto &obstacle : sta_obstacles_)
    {
      if (obstacle->isColliding(pt))
      {
        is_blocked = true;
        break;
      }
    }

    if (is_blocked)
    {
      sample_time += (ros::Time::now() - t1).toSec();
      continue;
    }

    /* find visible guard */
    // 这里将空间和时间解耦考虑，先只考虑静态障碍物
    vector<GraphNode::Ptr> visib_guards = findVisibGuard(pt);
    if (visib_guards.size() == 0)
    {
      GraphNode::Ptr guard = GraphNode::Ptr(new GraphNode(pt, GraphNode::Guard, ++node_id));
      graph_.push_back(guard);
    }
    else if (visib_guards.size() == 2)
    {
      /* try adding new connection between two guard */
      // vector<pair<GraphNode::Ptr, GraphNode::Ptr>> sort_guards =
      // sortVisibGuard(visib_guards);

      // 考虑动态障碍物时，需要检查两个Guard和新采样点形成的两条线段的安全区间是否有重叠
      // 计算2个Guard节点和新采样点的安全时间区间
      auto visib_guard_1 = visib_guards[0];
      auto visib_guard_2 = visib_guards[1];
      auto safe_intervals_1 = computeSafeIntervals(visib_guard_1->position, dyn_obstacles_);
      auto safe_intervals_2 = computeSafeIntervals(visib_guard_2->position, dyn_obstacles_);
      auto safe_intervals_pt = computeSafeIntervals(pt, dyn_obstacles_);

      // 将安全时间区间绑定guard节点
      safety_data_[visib_guard_1->id_] = safe_intervals_1;
      safety_data_[visib_guard_2->id_] = safe_intervals_2;

      // 检查边的安全时间窗口
      safe_edge_windows_1_ = computeEdgeSafeWindow(
          visib_guard_1->position, pt, dyn_obstacles_, robot_speed_);
      safe_edge_windows_2_ = computeEdgeSafeWindow(
          pt, visib_guard_2->position, dyn_obstacles_, robot_speed_);
      
      // 如果两个边的安全时间窗口没有交集，则继续采样
      auto intervalsOverlap = [](const std::vector<std::pair<double, double>> &a,
                                 const std::vector<std::pair<double, double>> &b,
                                 double eps = 1e-6) -> bool {
        if (a.empty() || b.empty()) return false;
        size_t ia = 0, ib = 0;
        // assume intervals are sorted and non-overlapping within each vector (mergeIntervals used earlier)
        while (ia < a.size() && ib < b.size()) {
          double a_start = a[ia].first, a_end = a[ia].second;
          double b_start = b[ib].first, b_end = b[ib].second;

          // consider overlap when (a_start <= b_end) && (b_start <= a_end)
          if (a_start <= b_end + eps && b_start <= a_end + eps) {
            // also ensure the overlap has positive duration (or within eps)
            double overlap_start = std::max(a_start, b_start);
            double overlap_end = std::min(a_end, b_end);
            if (overlap_end + eps >= overlap_start) return true;
          }

          // advance the interval with the smaller end
          if (a_end < b_end) ++ia; else ++ib;
        }
        return false;
      };

      // 如果两个边的安全时间窗口没有交集，则继续采样
      if (!intervalsOverlap(safe_edge_windows_1_, safe_edge_windows_2_)) {
        sample_time += (ros::Time::now() - t1).toSec();
        continue;
      }
    }

    // 判断新路径和已有路径是否同拓扑
    bool need_connect = needConnection(visib_guards[0], visib_guards[1], pt);
    if (!need_connect)
    {
      sample_time += (ros::Time::now() - t1).toSec();
      continue;
    }
    // new useful connection needed, add new connector
    GraphNode::Ptr connector = GraphNode::Ptr(new GraphNode(pt, GraphNode::Connector, ++node_id));
    graph_.push_back(connector);

    // connect guards
    visib_guards[0]->neighbors_.push_back(connector);
    visib_guards[1]->neighbors_.push_back(connector);

    connector->neighbors_.push_back(visib_guards[0]);
    connector->neighbors_.push_back(visib_guards[1]);
  }

  sample_time += (ros::Time::now() - t1).toSec();
}

/* print record */
std::cout << "[Topo]: sample num: " << sample_num;

pruneGraph();
// std::cout << "[Topo]: node num: " << graph_.size() << std::endl;

return graph_;
// return searchPaths(start_node, end_node);
}

void TopologyPRM::staticObsCallback(const obj_state_msgs::ObjectsStates::ConstPtr &msg)
{
  sta_obstacles_.clear();

  for (const auto &state : msg->states)
  {
    Eigen::Vector3d position(state.position.x, state.position.y, state.position.z);
    double radius = state.size.x / 2.0;

    auto obstacle = std::make_shared<tprm::StaticSphereObstacle>(position, radius);
    sta_obstacles_.push_back(obstacle);
  }

  has_obstacles_ = true;
  ROS_INFO_THROTTLE(5.0, "Received %lu static obstacles", sta_obstacles_.size());
}

void TopologyPRM::dynObstaclesCallback(const obj_state_msgs::ObjectsStates::ConstPtr &msg)
{
    dyn_obstacles_.clear();

    for (const auto &state : msg->states)
    {
        tprm::Vector3d position(state.position.x, state.position.y, state.position.z);
        tprm::Vector3d velocity(state.velocity.x, state.velocity.y, state.velocity.z);
        double radius = state.size.x / 2.0 + safety_margin_; // 添加安全裕度

        auto obstacle = std::make_shared<tprm::DynamicSphereObstacle>(position, velocity, radius);
        dyn_obstacles_.push_back(obstacle);
    }

    ROS_INFO_THROTTLE(2.0, "Received %lu dynamic obstacles with UTVD safety margin", dyn_obstacles_.size());
}

void TopologyPRM::goalCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    goal_pos_ = tprm::Vector3d(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    ROS_INFO("New goal received: [%.2f, %.2f, %.2f]", goal_pos_[0], goal_pos_[1], goal_pos_[2]);
}

vector<GraphNode::Ptr> TopologyPRM::findVisibGuard(Eigen::Vector3d pt) {
  vector<GraphNode::Ptr> visib_guards;
  Eigen::Vector3d pc;

  int visib_num = 0;

  /* find visible GUARD from pt */
  for (list<GraphNode::Ptr>::iterator iter = graph_.begin(); iter != graph_.end(); ++iter) {
    if ((*iter)->type_ == GraphNode::Connector) continue;

    if (lineVisibStatic(pt, (*iter)->pos_, line_step_, pc)) {
      visib_guards.push_back((*iter));
      ++visib_num;
      if (visib_num > 2) break;
    }
  }

  return visib_guards;
}

bool TopologyPRM::needConnection(
    GraphNode::Ptr g1,        // 第一个Guard节点
    GraphNode::Ptr g2,        // 第二个Guard节点  
    Eigen::Vector3d pt        // 新采样点（潜在的Connector节点）
)
{
  vector<Eigen::Vector3d> path1(3), path2(3);
  GraphNode::Ptr connector;
  path1[0] = g1->pos_;
  path1[1] = pt;
  path1[2] = g2->pos_;

  path2[0] = g1->pos_;
  path2[2] = g2->pos_;

  vector<Eigen::Vector3d> connect_pts;
  bool has_connect = false;
  for (int i = 0; i < g1->neighbors_.size(); ++i) {
    for (int j = 0; j < g2->neighbors_.size(); ++j) {
      // 查找g1和g2的共同邻居节点（即现有的Connector节点）
      if (g1->neighbors_[i]->id_ == g2->neighbors_[j]->id_) {
        path2[1] = g1->neighbors_[i]->pos_;
        connector = g1->neighbors_[i];
        bool same_topo = sameTopoPathUTVD(g1, g2, connector, pt);
        if (same_topo) {
          // get shorter connection ?
          if (pathLength(path1) < pathLength(path2)) {
            g1->neighbors_[i]->pos_ = pt;
            // ROS_WARN("shorter!");
          }
          return false;
        }
      }
    }
  }
  return true;
}

Eigen::Vector3d TopologyPRM::getSample() {
  /* sampling */
  Eigen::Vector3d pt;
  pt(0) = rand_pos_(eng_) * sample_r_(0);
  pt(1) = rand_pos_(eng_) * sample_r_(1);
  pt(2) = rand_pos_(eng_) * sample_r_(2);

  pt = rotation_ * pt + translation_;

  return pt;
}

bool TopologyPRM::lineVisibStatic(
    const Eigen::Vector3d& p1,
    const Eigen::Vector3d& p2,
    double base_step,
    Eigen::Vector3d& pc
)
{
    Eigen::Vector3d dir = p2 - p1;
    double total_len = dir.norm();
    if (total_len < 1e-6) return true;
    dir.normalize();

    double s = 0.0;
    Eigen::Vector3d hit_pt;
    bool blocked = false;

    while (s <= total_len)
    {
        Eigen::Vector3d pt = p1 + dir * s;

        // // ========= 动态步长策略 =========
        // // 这里给出一个简易版本：
        // // step = max( base_step * 0.2, min( base_step * 3, 与障碍的最近距离 * 0.5 ) )
        // // 这里我们仅根据 staticObstacles 距离估计
        // double min_dist = std::numeric_limits<double>::infinity();
        // for (const auto &obstacle : staticObstacles)
        // {
        //     double d = obstacle->distanceTo(pt);
        //     if (d < min_dist) min_dist = d;
        // }

        // double step = std::clamp(min_dist * 0.5, base_step * 0.2, base_step * 3.0);
        double step = base_step; // 先使用固定步长，后续可改进

        // ========= 并行检测碰撞 =========
        blocked = false;
        #pragma omp parallel for shared(blocked)
        for (int i = 0; i < sta_obstacles_.size(); ++i)
        {
            if (blocked) continue; // 若已检测到碰撞则跳过
            if (sta_obstacles_[i]->isColliding(pt))
            {
                #pragma omp critical
                {
                    if (!blocked)
                    {
                        blocked = true;
                        hit_pt = pt;
                    }
                }
            }
        }

        if (blocked)
        {
            pc = hit_pt;
            return false;
        }

        s += step;
    }

    // ========= 最后检查终点 =========
    #pragma omp parallel for shared(blocked)
    for (int i = 0; i < sta_obstacles_.size(); ++i)
    {
        if (blocked) continue;
        if (sta_obstacles_[i]->isColliding(p2))
        {
            #pragma omp critical
            {
                if (!blocked)
                {
                    blocked = true;
                    hit_pt = p2;
                }
            }
        }
    }

    if (blocked)
    {
        pc = hit_pt;
        return false;
    }

    return true;
}

// 删除没有邻居的节点或者只有一个邻居的节点
void TopologyPRM::pruneGraph() {
  /* prune useless node */
  if (graph_.size() > 2) {
    for (list<GraphNode::Ptr>::iterator iter1 = graph_.begin();
         iter1 != graph_.end() && graph_.size() > 2; ++iter1) {
      if ((*iter1)->id_ <= 1) continue;

      /* core */
      // std::cout << "id: " << (*iter1)->id_ << std::endl;
      if ((*iter1)->neighbors_.size() <= 1) {
        // delete this node from others' neighbor
        for (list<GraphNode::Ptr>::iterator iter2 = graph_.begin(); iter2 != graph_.end(); ++iter2) {
          for (vector<GraphNode::Ptr>::iterator it_nb = (*iter2)->neighbors_.begin();
               it_nb != (*iter2)->neighbors_.end(); ++it_nb) {
            if ((*it_nb)->id_ == (*iter1)->id_) {
              (*iter2)->neighbors_.erase(it_nb);
              break;
            }
          }
        }

        // delete this node from graph, restart checking
        graph_.erase(iter1);
        iter1 = graph_.begin();
      }
    }
  }
}

/**
 * @brief 计算点位置的安全时间间隔
 * @param position 空间位置
 * @param obstacles 动态障碍物列表
 * @param robot_radius 机器人半径
 * @return 安全时间间隔列表
 */
std::vector<std::pair<double, double>> TopologyPRM::computeSafeIntervals(
    const tprm::Vector3d &position,
    const std::vector<std::shared_ptr<tprm::DynamicSphereObstacle>> &obstacles,
    double robot_radius = 0.3)
{

  std::vector<std::pair<double, double>> safe_intervals;
  std::vector<std::pair<double, double>> collision_intervals;

  // 收集所有障碍物的碰撞时间间隔
  for (const auto &obstacle : obstacles)
  {
    double hit_time_from, hit_time_to;
    if (obstacle->isColliding(position, hit_time_from, hit_time_to))
    {
      collision_intervals.emplace_back(hit_time_from, hit_time_to);
    }
  }

  // 合并重叠的碰撞间隔
  collision_intervals = mergeIntervals(collision_intervals);

  // 从碰撞间隔推导安全间隔
  if (collision_intervals.empty())
  {
    // 如果没有碰撞，整个时间域都是安全的
    safe_intervals.emplace_back(0.0, std::numeric_limits<double>::infinity());
  }
  else
  {
    // 第一个安全间隔：从0到第一个碰撞开始
    if (collision_intervals[0].first > 0)
    {
      safe_intervals.emplace_back(0.0, collision_intervals[0].first);
    }

    // 中间的安全间隔
    for (size_t i = 0; i < collision_intervals.size() - 1; ++i)
    {
      double safe_start = collision_intervals[i].second;
      double safe_end = collision_intervals[i + 1].first;
      if (safe_start < safe_end)
      {
        safe_intervals.emplace_back(safe_start, safe_end);
      }
    }

    // 最后一个安全间隔：从最后一个碰撞结束到无穷大
    if (!collision_intervals.empty() &&
        collision_intervals.back().second < std::numeric_limits<double>::infinity())
    {
      safe_intervals.emplace_back(collision_intervals.back().second,
                                  std::numeric_limits<double>::infinity());
    }
  }

  return safe_intervals;
}

std::vector<std::pair<double, double>> TopologyPRM::mergeIntervals(
    std::vector<std::pair<double, double>> intervals)
{

  if (intervals.empty())
    return intervals;

  std::sort(intervals.begin(), intervals.end());
  std::vector<std::pair<double, double>> merged;
  merged.push_back(intervals[0]);

  for (size_t i = 1; i < intervals.size(); ++i)
  {
    if (intervals[i].first <= merged.back().second)
    {
      merged.back().second = std::max(merged.back().second, intervals[i].second);
    }
    else
    {
      merged.push_back(intervals[i]);
    }
  }

  return merged;
}

std::vector<std::pair<double, double>> TopologyPRM::computeEdgeSafeWindow(
    const tprm::Vector3d &from,
    const tprm::Vector3d &to,
    const std::vector<std::shared_ptr<tprm::DynamicSphereObstacle>> &obstacles,
    double robot_speed)
{

  double distance = (to - from).norm();
  double travel_time = distance / robot_speed;

  std::vector<std::pair<double, double>> safe_windows;
  std::vector<std::pair<double, double>> collision_windows;

  // 对每个障碍物计算碰撞时间窗口
  for (const auto &obstacle : obstacles)
  {
    auto window = computeObstacleCollisionWindow(from, to, obstacle, robot_speed);
    if (window.first < window.second)
    {
      collision_windows.push_back(window);
    }
  }

  // 合并碰撞窗口
  collision_windows = mergeIntervals(collision_windows);

  // 推导安全窗口（与安全间隔计算类似）
  if (collision_windows.empty())
  {
    safe_windows.emplace_back(0.0, std::numeric_limits<double>::infinity());
  }
  else
  {
    if (collision_windows[0].first > 0)
    {
      safe_windows.emplace_back(0.0, collision_windows[0].first);
    }

    for (size_t i = 0; i < collision_windows.size() - 1; ++i)
    {
      double safe_start = collision_windows[i].second;
      double safe_end = collision_windows[i + 1].first;
      if (safe_start < safe_end)
      {
        safe_windows.emplace_back(safe_start, safe_end);
      }
    }

    if (collision_windows.back().second < std::numeric_limits<double>::infinity())
    {
      safe_windows.emplace_back(collision_windows.back().second,
                                std::numeric_limits<double>::infinity());
    }
  }

  return safe_windows;
}

std::pair<double, double> TopologyPRM::computeObstacleCollisionWindow(
    const tprm::Vector3d &from,
    const tprm::Vector3d &to,
    const std::shared_ptr<tprm::DynamicSphereObstacle> &obstacle,
    double robot_speed)
{

  // 简化计算：使用线段与移动球体的碰撞检测
  // 实际实现应根据UTVD理论进行精确计算

  double distance = (to - from).norm();
  double travel_time = distance / robot_speed;

  // 这里使用简化的线性插值碰撞检测
  // 实际UTVD实现应考虑障碍物运动轨迹和机器人运动轨迹的相对关系

  // 记录最早的碰撞开始时间和最晚的碰撞结束时间
  double min_collision_time = std::numeric_limits<double>::infinity();
  double max_collision_time = 0.0;

  // 采样检测点
  const int samples = 10;
  for (int i = 0; i <= samples; ++i)
  {
    double t = static_cast<double>(i) / samples;
    tprm::Vector3d point = from + t * (to - from);
    double segment_time = t * travel_time;

    double hit_from, hit_to;
    if (obstacle->isColliding(point, hit_from, hit_to))
    {
      // 调整碰撞时间考虑机器人的到达时间
      double adjusted_from = std::max(0.0, hit_from - segment_time);
      double adjusted_to = std::max(0.0, hit_to - segment_time);

      // 取所有采样点中最早的碰撞开始时间和最晚的碰撞结束时间
      min_collision_time = std::min(min_collision_time, adjusted_from);
      max_collision_time = std::max(max_collision_time, adjusted_to);
    }
  }

  if (min_collision_time < std::numeric_limits<double>::infinity())
  {
    return {min_collision_time, max_collision_time};
  }

  return {std::numeric_limits<double>::infinity(), 0.0};
}

vector<vector<Eigen::Vector3d>> TopologyPRM::pruneEquivalent(vector<vector<Eigen::Vector3d>>& paths) {
  vector<vector<Eigen::Vector3d>> pruned_paths;
  if (paths.size() < 1) return pruned_paths;

  /* ---------- prune topo equivalent path ---------- */
  // output: pruned_paths
  vector<int> exist_paths_id;
  exist_paths_id.push_back(0);

  for (int i = 1; i < paths.size(); ++i) {
    // compare with exsit paths
    bool new_path = true;

    for (int j = 0; j < exist_paths_id.size(); ++j) {
      // compare with one path
      bool same_topo = sameTopoPath(paths[i], paths[exist_paths_id[j]], 0.0);

      if (same_topo) {
        new_path = false;
        break;
      }
    }

    if (new_path) {
      exist_paths_id.push_back(i);
    }
  }

  // save pruned paths
  for (int i = 0; i < exist_paths_id.size(); ++i) {
    pruned_paths.push_back(paths[exist_paths_id[i]]);
  }

  std::cout << ", pruned path num: " << pruned_paths.size();

  return pruned_paths;
}

vector<vector<Eigen::Vector3d>> TopologyPRM::selectShortPaths(vector<vector<Eigen::Vector3d>>& paths,
                                                              int step) {
  /* ---------- only reserve top short path ---------- */
  vector<vector<Eigen::Vector3d>> short_paths;
  vector<Eigen::Vector3d> short_path;
  double min_len;

  for (int i = 0; i < reserve_num_ && paths.size() > 0; ++i) {
    int path_id = shortestPath(paths);
    if (i == 0) {
      short_paths.push_back(paths[path_id]);
      min_len = pathLength(paths[path_id]);
      paths.erase(paths.begin() + path_id);
    } else {
      double rat = pathLength(paths[path_id]) / min_len;
      if (rat < ratio_to_short_) {
        short_paths.push_back(paths[path_id]);
        paths.erase(paths.begin() + path_id);
      } else {
        break;
      }
    }
  }
  std::cout << ", select path num: " << short_paths.size();

  /* ---------- merge with start and end segment ---------- */
  for (int i = 0; i < short_paths.size(); ++i) {
    short_paths[i].insert(short_paths[i].begin(), start_pts_.begin(), start_pts_.end());
    short_paths[i].insert(short_paths[i].end(), end_pts_.begin(), end_pts_.end());
  }
  for (int i = 0; i < short_paths.size(); ++i) {
    shortcutPath(short_paths[i], i, 5);
    short_paths[i] = short_paths_[i];
  }

  short_paths = pruneEquivalent(short_paths);

  return short_paths;
}

bool TopologyPRM::sameTopoPath(const vector<Eigen::Vector3d>& path1,
                               const vector<Eigen::Vector3d>& path2, double thresh) {
  // calc the length
  double len1 = pathLength(path1);
  double len2 = pathLength(path2);

  double max_len = max(len1, len2);

  int pt_num = ceil(max_len / line_step_);

  // std::cout << "pt num: " << pt_num << std::endl;

  vector<Eigen::Vector3d> pts1 = discretizePath(path1, pt_num);
  vector<Eigen::Vector3d> pts2 = discretizePath(path2, pt_num);

  Eigen::Vector3d pc;
  for (int i = 0; i < pt_num; ++i) {
    // if (!lineVisib(pts1[i], pts2[i], thresh, pc)) {
    //   return false;
    // }
    if (!lineVisibStatic(pts1[i], pts2[i], line_step_, pc)) {
      return false;
    }
  }

  return true;
}

bool TopologyPRM::sameTopoPathUTVD(const GraphNode::Ptr guard1, const GraphNode::Ptr guard2,
                    const GraphNode::Ptr connector, const Eigen::Vector3d pt){
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

  // 可选，检查反向路径

  // 构造 PathAdapter，映射 s ∈ [0,1] → 空间 & 时间
  struct PathAdapter
  {
    std::function<Eigen::Vector3d(double)> posAt;
    std::function<double(double)> timeAt;
  };

  auto makePathAdapter = [&](const std::array<Eigen::Vector3d, 3> &path, const std::pair<double, double> &corr)
  {
    double t_start = corr.first;
    double t_end = corr.second;
    double T = t_end - t_start;
    PathAdapter P;
    P.posAt = [=](double s) -> Eigen::Vector3d
    {
      if (s <= 0.5)
      {
        double u = s / 0.5;
        return (1.0 - u) * path[0] + u * path[1];
      }
      else
      {
        double u = (s - 0.5) / 0.5;
        return (1.0 - u) * path[1] + u * path[2];
      }
    };
    P.timeAt = [=](double s) -> double
    {
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

  for (const auto &c1 : corridors1)
  {
    for (const auto &c2 : corridors2)
    {
      double T1 = c1.second - c1.first;
      double T2 = c2.second - c2.first;
      if (T1 <= eps_time || T2 <= eps_time)
        continue;

      double alpha = T1 / T2;
      double theta = (c1.first - c2.first) / T2;

      if (theta < -eps_time || alpha + theta > 1.0 + eps_time)
        continue; // 不满足时域映射约束

      auto P1 = makePathAdapter(path_new, c1);
      auto P2 = makePathAdapter(path_exist, c2);

      // ======================================================
      // Step 6. UTVD 检查函数（coarse + fine）
      // ======================================================
      auto checkUTVD = [&](int Ns, int N_lambda) -> bool
      {
        for (int i = 0; i < Ns; ++i)
        {
          double s = double(i) / (Ns - 1);
          double s2 = alpha * s + theta;
          if (s2 < 0.0 - 1e-9 || s2 > 1.0 + 1e-9)
            return false;

          Eigen::Vector3d p1 = P1.posAt(s);
          Eigen::Vector3d p2 = P2.posAt(s2);
          double t1 = P1.timeAt(s);
          double t2 = P2.timeAt(s2);

          for (int j = 0; j < N_lambda; ++j)
          {
            double lambda = double(j) / (N_lambda - 1);
            Eigen::Vector3d x = (1.0 - lambda) * p1 + lambda * p2;
            double tau = (1.0 - lambda) * t1 + lambda * t2;

            for (const auto &obs : dyn_obstacles_)
            {
              if (obs->isCollidingAtTime(x, tau, eps_dist))
              {
                return false; // 任一点与障碍物冲突
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

      if (fine_pass)
      {
        return true; // 同拓扑
      }
    }
  }

  return false;
}

// helper: intersect two interval-lists (each sorted, non-overlapping)
std::vector<std::pair<double,double>> TopologyPRM::intersectIntervals(
    const std::vector<std::pair<double,double>>& A,
    const std::vector<std::pair<double,double>>& B)
{
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

int TopologyPRM::shortestPath(vector<vector<Eigen::Vector3d>>& paths) {
  int short_id = -1;
  double min_len = 100000000;
  for (int i = 0; i < paths.size(); ++i) {
    double len = pathLength(paths[i]);
    if (len < min_len) {
      short_id = i;
      min_len = len;
    }
  }
  return short_id;
}
double TopologyPRM::pathLength(const vector<Eigen::Vector3d>& path) {
  double length = 0.0;
  if (path.size() < 2) return length;

  for (int i = 0; i < path.size() - 1; ++i) {
    length += (path[i + 1] - path[i]).norm();
  }
  return length;
}

vector<Eigen::Vector3d> TopologyPRM::discretizePath(const vector<Eigen::Vector3d>& path, int pt_num) {
  vector<double> len_list;
  len_list.push_back(0.0);

  for (int i = 0; i < path.size() - 1; ++i) {
    double inc_l = (path[i + 1] - path[i]).norm();
    len_list.push_back(inc_l + len_list[i]);
  }

  // calc pt_num points along the path
  double len_total = len_list.back();
  double dl = len_total / double(pt_num - 1);
  double cur_l;

  vector<Eigen::Vector3d> dis_path;
  for (int i = 0; i < pt_num; ++i) {
    cur_l = double(i) * dl;

    // find the range cur_l in
    int idx = -1;
    for (int j = 0; j < len_list.size() - 1; ++j) {
      if (cur_l >= len_list[j] - 1e-4 && cur_l <= len_list[j + 1] + 1e-4) {
        idx = j;
        break;
      }
    }

    // find lambda and interpolate
    double lambda = (cur_l - len_list[idx]) / (len_list[idx + 1] - len_list[idx]);
    Eigen::Vector3d inter_pt = (1 - lambda) * path[idx] + lambda * path[idx + 1];
    dis_path.push_back(inter_pt);
  }

  return dis_path;
}

vector<Eigen::Vector3d> TopologyPRM::pathToGuidePts(vector<Eigen::Vector3d>& path, int pt_num) {
  return discretizePath(path, pt_num);
}

void TopologyPRM::shortcutPath(vector<Eigen::Vector3d> path, int path_id, int iter_num) {
  vector<Eigen::Vector3d> short_path = path;
  vector<Eigen::Vector3d> last_path;

  for (int k = 0; k < iter_num; ++k) {
    last_path = short_path;

    vector<Eigen::Vector3d> dis_path = discretizePath(short_path);

    if (dis_path.size() < 2) {
      short_paths_[path_id] = dis_path;
      return;
    }

    /* visibility path shortening */
    Eigen::Vector3d colli_pt, grad, dir, push_dir;
    double dist;
    short_path.clear();
    short_path.push_back(dis_path.front());
    for (int i = 1; i < dis_path.size(); ++i) {
      if (lineVisib(short_path.back(), dis_path[i], resolution_, colli_pt, path_id)) continue;

      edt_environment_->evaluateEDTWithGrad(colli_pt, -1, dist, grad);
      if (grad.norm() > 1e-3) {
        grad.normalize();
        dir = (dis_path[i] - short_path.back()).normalized();
        push_dir = grad - grad.dot(dir) * dir;
        push_dir.normalize();
        colli_pt = colli_pt + resolution_ * push_dir;
      }
      short_path.push_back(colli_pt);
    }
    short_path.push_back(dis_path.back());

    /* break if no shortcut */
    double len1 = pathLength(last_path);
    double len2 = pathLength(short_path);
    if (len2 > len1) {
      // ROS_WARN("pause shortcut, l1: %lf, l2: %lf, iter: %d", len1, len2, k +
      // 1);
      short_path = last_path;
      break;
    }
  }

  short_paths_[path_id] = short_path;
}

void TopologyPRM::shortcutPaths() {
  short_paths_.resize(raw_paths_.size());

  if (parallel_shortcut_) {
    vector<thread> short_threads;
    for (int i = 0; i < raw_paths_.size(); ++i) {
      short_threads.push_back(thread(&TopologyPRM::shortcutPath, this, raw_paths_[i], i, 1));
    }
    for (int i = 0; i < raw_paths_.size(); ++i) {
      short_threads[i].join();
    }
  } else {
    for (int i = 0; i < raw_paths_.size(); ++i) shortcutPath(raw_paths_[i], i);
  }
}

vector<Eigen::Vector3d> TopologyPRM::discretizeLine(Eigen::Vector3d p1, Eigen::Vector3d p2) {
  Eigen::Vector3d dir = p2 - p1;
  double len = dir.norm();
  int seg_num = ceil(len / resolution_);

  vector<Eigen::Vector3d> line_pts;
  if (seg_num <= 0) {
    return line_pts;
  }

  for (int i = 0; i <= seg_num; ++i) line_pts.push_back(p1 + dir * double(i) / double(seg_num));

  return line_pts;
}

vector<Eigen::Vector3d> TopologyPRM::discretizePath(vector<Eigen::Vector3d> path) {
  vector<Eigen::Vector3d> dis_path, segment;

  if (path.size() < 2) {
    ROS_ERROR("what path? ");
    return dis_path;
  }

  for (int i = 0; i < path.size() - 1; ++i) {
    segment = discretizeLine(path[i], path[i + 1]);

    if (segment.size() < 1) continue;

    dis_path.insert(dis_path.end(), segment.begin(), segment.end());
    if (i != path.size() - 2) dis_path.pop_back();
  }
  return dis_path;
}

vector<vector<Eigen::Vector3d>> TopologyPRM::discretizePaths(vector<vector<Eigen::Vector3d>>& path) {
  vector<vector<Eigen::Vector3d>> dis_paths;
  vector<Eigen::Vector3d> dis_path;

  for (int i = 0; i < path.size(); ++i) {
    dis_path = discretizePath(path[i]);

    if (dis_path.size() > 0) dis_paths.push_back(dis_path);
  }

  return dis_paths;
}

Eigen::Vector3d TopologyPRM::getOrthoPoint(const vector<Eigen::Vector3d>& path) {
  Eigen::Vector3d x1 = path.front();
  Eigen::Vector3d x2 = path.back();

  Eigen::Vector3d dir = (x2 - x1).normalized();
  Eigen::Vector3d mid = 0.5 * (x1 + x2);

  double min_cos = 1000.0;
  Eigen::Vector3d pdir;
  Eigen::Vector3d ortho_pt;

  for (int i = 1; i < path.size() - 1; ++i) {
    pdir = (path[i] - mid).normalized();
    double cos = fabs(pdir.dot(dir));

    if (cos < min_cos) {
      min_cos = cos;
      ortho_pt = path[i];
    }
  }

  return ortho_pt;
}

// search for useful path in the topo graph by DFS
vector<vector<Eigen::Vector3d>> TopologyPRM::searchPaths() {
  raw_paths_.clear();

  vector<GraphNode::Ptr> visited;
  visited.push_back(graph_.front());

  depthFirstSearch(visited);

  // sort the path by node number
  int min_node_num = 100000, max_node_num = 1;
  vector<vector<int>> path_list(100);
  for (int i = 0; i < raw_paths_.size(); ++i) {
    if (int(raw_paths_[i].size()) > max_node_num) max_node_num = raw_paths_[i].size();
    if (int(raw_paths_[i].size()) < min_node_num) min_node_num = raw_paths_[i].size();
    path_list[int(raw_paths_[i].size())].push_back(i);
  }

  // select paths with less nodes
  vector<vector<Eigen::Vector3d>> filter_raw_paths;
  for (int i = min_node_num; i <= max_node_num; ++i) {
    bool reach_max = false;
    for (int j = 0; j < path_list[i].size(); ++j) {
      filter_raw_paths.push_back(raw_paths_[path_list[i][j]]);
      if (filter_raw_paths.size() >= max_raw_path2_) {
        reach_max = true;
        break;
      }
    }
    if (reach_max) break;
  }
  std::cout << ", raw path num: " << raw_paths_.size() << ", " << filter_raw_paths.size();

  raw_paths_ = filter_raw_paths;

  return raw_paths_;
}

void TopologyPRM::depthFirstSearch(vector<GraphNode::Ptr>& vis) {
  GraphNode::Ptr cur = vis.back();

  for (int i = 0; i < cur->neighbors_.size(); ++i) {
    // check reach goal
    if (cur->neighbors_[i]->id_ == 1) {
      // add this path to paths set
      vector<Eigen::Vector3d> path;
      for (int j = 0; j < vis.size(); ++j) {
        path.push_back(vis[j]->pos_);
      }
      path.push_back(cur->neighbors_[i]->pos_);

      raw_paths_.push_back(path);
      if (raw_paths_.size() >= max_raw_path_) return;

      break;
    }
  }

  for (int i = 0; i < cur->neighbors_.size(); ++i) {
    // skip reach goal
    if (cur->neighbors_[i]->id_ == 1) continue;

    // skip already visited node
    bool revisit = false;
    for (int j = 0; j < vis.size(); ++j) {
      if (cur->neighbors_[i]->id_ == vis[j]->id_) {
        revisit = true;
        break;
      }
    }
    if (revisit) continue;

    // recursive search
    vis.push_back(cur->neighbors_[i]);
    depthFirstSearch(vis);
    if (raw_paths_.size() >= max_raw_path_) return;

    vis.pop_back();
  }
}

void TopologyPRM::setEnvironment(const EDTEnvironment::Ptr& env) { this->edt_environment_ = env; }

bool TopologyPRM::triangleVisib(Eigen::Vector3d pt, Eigen::Vector3d p1, Eigen::Vector3d p2) {
  // get the traversing points along p1-p2
  vector<Eigen::Vector3d> pts;

  Eigen::Vector3d dir = p2 - p1;
  double length = dir.norm();
  int seg_num = ceil(length / resolution_);

  Eigen::Vector3d pt1;
  for (int i = 1; i < seg_num; ++i) {
    pt1 = p1 + dir * double(i) / double(seg_num);
    pts.push_back(pt1);
  }

  // test visibility
  for (int i = 0; i < pts.size(); ++i) {
    {
      return false;
    }
  }

  return true;
}
// TopologyPRM::
}// namespace topo_dyn_planner