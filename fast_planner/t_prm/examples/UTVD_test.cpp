/*********************************************************************
 * ROS T-PRM Dynamic Test with UTVD Algorithm Integration
 *
 * This node integrates UTVD (Unified Time-Varying Domain) algorithm
 * for safe interval motion planning in dynamic environments.
 *********************************************************************/

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

namespace utvd {

/**
 * @brief UTVD安全间隔计算类
 * 基于SIMP论文中的Unified Time-Varying Domain理论
 */
class UTVDSafetyInterval {
public:
    /**
     * @brief 计算点位置的安全时间间隔
     * @param position 空间位置
     * @param obstacles 动态障碍物列表
     * @param robot_radius 机器人半径
     * @return 安全时间间隔列表
     */
    static std::vector<std::pair<double, double>> computeSafeIntervals(
        const tprm::Vector3d& position,
        const std::vector<std::shared_ptr<tprm::DynamicSphereObstacle>>& obstacles,
        double robot_radius = 0.3) {
        
        std::vector<std::pair<double, double>> safe_intervals;
        std::vector<std::pair<double, double>> collision_intervals;
        
        // 收集所有障碍物的碰撞时间间隔
        for (const auto& obstacle : obstacles) {
            double hit_time_from, hit_time_to;
            if (obstacle->isColliding(position, hit_time_from, hit_time_to)) {
                collision_intervals.emplace_back(hit_time_from, hit_time_to);
            }
        }
        
        // 合并重叠的碰撞间隔
        collision_intervals = mergeIntervals(collision_intervals);
        
        // 从碰撞间隔推导安全间隔
        if (collision_intervals.empty()) {
            // 如果没有碰撞，整个时间域都是安全的
            safe_intervals.emplace_back(0.0, std::numeric_limits<double>::infinity());
        } else {
            // 第一个安全间隔：从0到第一个碰撞开始
            if (collision_intervals[0].first > 0) {
                safe_intervals.emplace_back(0.0, collision_intervals[0].first);
            }
            
            // 中间的安全间隔
            for (size_t i = 0; i < collision_intervals.size() - 1; ++i) {
                double safe_start = collision_intervals[i].second;
                double safe_end = collision_intervals[i + 1].first;
                if (safe_start < safe_end) {
                    safe_intervals.emplace_back(safe_start, safe_end);
                }
            }
            
            // 最后一个安全间隔：从最后一个碰撞结束到无穷大
            if (!collision_intervals.empty() && 
                collision_intervals.back().second < std::numeric_limits<double>::infinity()) {
                safe_intervals.emplace_back(collision_intervals.back().second, 
                                          std::numeric_limits<double>::infinity());
            }
        }
        
        return safe_intervals;
    }
    
    /**
     * @brief 计算边的安全时间窗口（UTVD核心）
     * @param from 起点位置
     * @param to 终点位置  
     * @param obstacles 动态障碍物列表
     * @param robot_speed 机器人速度
     * @return 边的安全时间窗口
     */
    static std::vector<std::pair<double, double>> computeEdgeSafeWindow(
        const tprm::Vector3d& from,
        const tprm::Vector3d& to,
        const std::vector<std::shared_ptr<tprm::DynamicSphereObstacle>>& obstacles,
        double robot_speed) {
        
        double distance = (to - from).norm();
        double travel_time = distance / robot_speed;
        
        std::vector<std::pair<double, double>> safe_windows;
        std::vector<std::pair<double, double>> collision_windows;
        
        // 对每个障碍物计算碰撞时间窗口
        for (const auto& obstacle : obstacles) {
            auto window = computeObstacleCollisionWindow(from, to, obstacle, robot_speed);
            if (window.first < window.second) {
                collision_windows.push_back(window);
            }
        }
        
        // 合并碰撞窗口
        collision_windows = mergeIntervals(collision_windows);
        
        // 推导安全窗口（与安全间隔计算类似）
        if (collision_windows.empty()) {
            safe_windows.emplace_back(0.0, std::numeric_limits<double>::infinity());
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
            
            if (collision_windows.back().second < std::numeric_limits<double>::infinity()) {
                safe_windows.emplace_back(collision_windows.back().second, 
                                         std::numeric_limits<double>::infinity());
            }
        }
        
        return safe_windows;
    }
    
private:
    /**
     * @brief 合并重叠的时间间隔
     */
    static std::vector<std::pair<double, double>> mergeIntervals(
        std::vector<std::pair<double, double>> intervals) {
        
        if (intervals.empty()) return intervals;
        
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
    
    /**
     * @brief 计算单个障碍物对边的碰撞时间窗口
     */
    static std::pair<double, double> computeObstacleCollisionWindow(
        const tprm::Vector3d& from,
        const tprm::Vector3d& to,
        const std::shared_ptr<tprm::DynamicSphereObstacle>& obstacle,
        double robot_speed) {
        
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
        for (int i = 0; i <= samples; ++i) {
            double t = static_cast<double>(i) / samples;
            tprm::Vector3d point = from + t * (to - from);
            double segment_time = t * travel_time;
            
            double hit_from, hit_to;
            if (obstacle->isColliding(point, hit_from, hit_to)) {
                // 调整碰撞时间考虑机器人的到达时间
                double adjusted_from = std::max(0.0, hit_from - segment_time);
                double adjusted_to = std::max(0.0, hit_to - segment_time);
                
                // 取所有采样点中最早的碰撞开始时间和最晚的碰撞结束时间
                min_collision_time = std::min(min_collision_time, adjusted_from);
                max_collision_time = std::max(max_collision_time, adjusted_to);
            }
        }
        
        if (min_collision_time < std::numeric_limits<double>::infinity()) {
            return {min_collision_time, max_collision_time};
        }
        
        return {std::numeric_limits<double>::infinity(), 0.0};
    }
};

} // namespace utvd

class TPRMDynamicTestUTVD {
private:
    ros::NodeHandle nh_;
    ros::Subscriber obs_sub_;
    ros::Subscriber goal_sub_;
    ros::Publisher path_pub_;
    ros::Publisher path_marker_pub_;
    ros::Publisher samples_pub_;
    ros::Publisher start_goal_pub_;
    ros::Publisher safety_intervals_pub_;
    ros::Timer plan_timer_;

    std::shared_ptr<tprm::TemporalPRM> tprm_;
    std::vector<std::shared_ptr<tprm::DynamicSphereObstacle>> obstacles_;

    tprm::Vector3d start_pos_;
    tprm::Vector3d goal_pos_;
    tprm::Vector3d env_min_;
    tprm::Vector3d env_max_;
    bool has_goal_;
    double robot_speed_;
    int num_samples_;
    double connection_radius_;
    double planning_freq_;
    double robot_radius_;

    // UTVD相关参数
    bool use_utvd_safety_;
    bool use_bspline_optimization_;
    double safety_margin_;

    // UTVD可视化参数
    bool enable_safety_interval_viz_;
    int viz_max_nodes_;
    double viz_time_horizon_;

public:
    TPRMDynamicTestUTVD() : nh_("~"), has_goal_(false) {
        // 获取基础参数
        nh_.param("robot_speed", robot_speed_, 1.0);
        nh_.param("num_samples", num_samples_, 200);
        nh_.param("connection_radius", connection_radius_, 2.0);
        nh_.param("planning_freq", planning_freq_, 1.0);
        nh_.param("robot_radius", robot_radius_, 0.3);

        // UTVD特定参数
        nh_.param("use_utvd_safety", use_utvd_safety_, true);
        nh_.param("use_bspline_optimization", use_bspline_optimization_, false);
        nh_.param("safety_margin", safety_margin_, 0.1);

        // UTVD可视化参数
        nh_.param("enable_safety_interval_viz", enable_safety_interval_viz_, true);
        nh_.param("viz_max_nodes", viz_max_nodes_, 50);
        nh_.param("viz_time_horizon", viz_time_horizon_, 10.0);

        std::vector<double> start_vec, goal_vec, env_min_vec, env_max_vec;
        nh_.param("start_position", start_vec, std::vector<double>{0.0, 0.0, 1.0});
        nh_.param("goal_position", goal_vec, std::vector<double>{10.0, 10.0, 1.0});
        nh_.param("environment_min", env_min_vec, std::vector<double>{0.0, 0.0, 0.0});
        nh_.param("environment_max", env_max_vec, std::vector<double>{15.0, 15.0, 3.0});

        start_pos_ = tprm::Vector3d(start_vec[0], start_vec[1], start_vec[2]);
        goal_pos_ = tprm::Vector3d(goal_vec[0], goal_vec[1], goal_vec[2]);
        env_min_ = tprm::Vector3d(env_min_vec[0], env_min_vec[1], env_min_vec[2]);
        env_max_ = tprm::Vector3d(env_max_vec[0], env_max_vec[1], env_max_vec[2]);
        has_goal_ = true;

        // 设置机器人速度
        tprm::HolonomicRobot::movement_speed = robot_speed_;

        // 初始化T-PRM
        tprm_ = std::make_shared<tprm::TemporalPRM>(env_min_, env_max_);

        ROS_INFO("UTVD-TPRM Environment bounds: [%.2f, %.2f, %.2f] to [%.2f, %.2f, %.2f]",
                 env_min_[0], env_min_[1], env_min_[2],
                 env_max_[0], env_max_[1], env_max_[2]);
        ROS_INFO("UTVD Safety: %s, B-spline Optimization: %s", 
                 use_utvd_safety_ ? "Enabled" : "Disabled",
                 use_bspline_optimization_ ? "Enabled" : "Disabled");

        // 订阅障碍物状态
        obs_sub_ = nh_.subscribe("/obj_states", 10, &TPRMDynamicTestUTVD::obstaclesCallback, this);
        goal_sub_ = nh_.subscribe("/move_base_simple/goal", 1, &TPRMDynamicTestUTVD::goalCallback, this);

        // 发布路径和可视化
        path_pub_ = nh_.advertise<nav_msgs::Path>("/tprm_path", 10);
        path_marker_pub_ = nh_.advertise<visualization_msgs::Marker>("/tprm_path_marker", 10);
        samples_pub_ = nh_.advertise<visualization_msgs::Marker>("/tprm_samples", 10);
        start_goal_pub_ = nh_.advertise<visualization_msgs::Marker>("/tprm_start_goal", 10);
        safety_intervals_pub_ = nh_.advertise<visualization_msgs::Marker>("/utvd_safety_intervals", 10);

        // 定时规划
        plan_timer_ = nh_.createTimer(ros::Duration(1.0 / planning_freq_), 
                                     &TPRMDynamicTestUTVD::planningCallback, this);

        ROS_INFO("UTVD-TPRM Dynamic Test Node Initialized");
    }

    void obstaclesCallback(const obj_state_msgs::ObjectsStates::ConstPtr& msg) {
        obstacles_.clear();
        tprm_ = std::make_shared<tprm::TemporalPRM>(env_min_, env_max_);

        for (const auto& state : msg->states) {
            tprm::Vector3d position(state.position.x, state.position.y, state.position.z);
            tprm::Vector3d velocity(state.velocity.x, state.velocity.y, state.velocity.z);
            double radius = state.size.x / 2.0 + safety_margin_;  // 添加安全裕度

            auto obstacle = std::make_shared<tprm::DynamicSphereObstacle>(position, velocity, radius);
            obstacles_.push_back(obstacle);
            tprm_->addDynamicObstacle(obstacle);
        }

        ROS_INFO_THROTTLE(2.0, "Received %lu dynamic obstacles with UTVD safety margin", obstacles_.size());
    }

    void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        goal_pos_ = tprm::Vector3d(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
        has_goal_ = true;
        ROS_INFO("New goal received: [%.2f, %.2f, %.2f]", goal_pos_[0], goal_pos_[1], goal_pos_[2]);
    }

    void planningCallback(const ros::TimerEvent&) {
        if (!has_goal_ || obstacles_.empty()) {
            ROS_WARN_THROTTLE(5.0, "Waiting for goal and obstacles...");
            return;
        }

        ROS_INFO("Planning path with UTVD-TPRM...");

        // 清空之前的图
        tprm_->getTemporalGraph().clear();

        // 构建PRM图
        tprm_->placeSamples(num_samples_);
        
        if (use_utvd_safety_) {
            // 使用UTVD安全间隔构建图连接
            buildPRMWithUTVDSafety();
        } else {
            // 使用原始方法构建图
            tprm_->buildPRM(connection_radius_);
        }

        ROS_INFO("UTVD-PRM Graph - Nodes: %d, Edges: %d",
                 tprm_->getTemporalGraph().getNumNodes(),
                 tprm_->getTemporalGraph().getNumEdges());

        // 可视化
        publishSamples();
        publishStartGoal();
        if (use_utvd_safety_ && enable_safety_interval_viz_) {
            publishSafetyIntervals();
        }

        // 规划路径
        double start_time = ros::Time::now().toSec();
        auto path = tprm_->getShortestPath(start_pos_, goal_pos_, start_time);

        if (path.empty()) {
            ROS_WARN("No path found with UTVD-TPRM!");
        } else {
            ROS_INFO("UTVD-TPRM path found with %lu waypoints", path.size());
            publishPath(path);
        }
    }

private:
    /**
     * @brief 使用UTVD安全间隔构建PRM图连接
     */
    void buildPRMWithUTVDSafety() {
        tprm_->placeSamples(num_samples_);
        
        auto& graph = tprm_->getTemporalGraph();
        int num_nodes = graph.getNumNodes();
        
        ROS_INFO("Building PRM with UTVD safety intervals...");
        
        // 为每个节点计算安全时间间隔
        std::vector<std::vector<std::pair<double, double>>> node_safety_intervals;
        for (int i = 0; i < num_nodes; ++i) {
            auto node = graph.getNode(i);
            auto intervals = utvd::UTVDSafetyInterval::computeSafeIntervals(
                node.position, obstacles_, robot_radius_);
            node_safety_intervals.push_back(intervals);
        }
        
        // 构建边连接（考虑UTVD安全窗口）
        int edges_added = 0;
        for (int i = 0; i < num_nodes; ++i) {
            for (int j = i + 1; j < num_nodes; ++j) {
                auto node_i = graph.getNode(i);
                auto node_j = graph.getNode(j);
                
                double distance = (node_j.position - node_i.position).norm();
                if (distance > connection_radius_) continue;
                
                // 计算边的安全时间窗口
                auto safe_windows = utvd::UTVDSafetyInterval::computeEdgeSafeWindow(
                    node_i.position, node_j.position, obstacles_, robot_speed_);
                
                // 如果存在安全窗口，添加边
                if (!safe_windows.empty() && 
                    safe_windows[0].first < std::numeric_limits<double>::infinity()) {
                    graph.addEdge(tprm::TemporalGraphEdge(i, j));
                    edges_added++;
                }
            }
        }
        
        ROS_INFO("UTVD-PRM: Added %d edges with safety validation", edges_added);
    }

    void publishPath(const std::vector<tprm::PathResultEntry>& path) {
        nav_msgs::Path path_msg;
        path_msg.header.frame_id = "map";
        path_msg.header.stamp = ros::Time::now();

        for (const auto& node : path) {
            geometry_msgs::PoseStamped pose;
            pose.header.frame_id = "map";
            pose.header.stamp = ros::Time(node.time);
            pose.pose.position.x = node.position[0];
            pose.pose.position.y = node.position[1];
            pose.pose.position.z = node.position[2];
            pose.pose.orientation.w = 1.0;
            path_msg.poses.push_back(pose);
        }
        path_pub_.publish(path_msg);

        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();
        marker.ns = "tprm_path";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::LINE_STRIP;
        marker.action = visualization_msgs::Marker::ADD;
        marker.scale.x = 0.1;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 1.0;
        marker.color.a = 1.0;

        for (const auto& node : path) {
            geometry_msgs::Point p;
            p.x = node.position[0];
            p.y = node.position[1];
            p.z = node.position[2];
            marker.points.push_back(p);
        }

        path_marker_pub_.publish(marker);
    }

    void publishSamples() {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();
        marker.ns = "tprm_samples";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::SPHERE_LIST;
        marker.action = visualization_msgs::Marker::ADD;
        marker.scale.x = 0.15;
        marker.scale.y = 0.15;
        marker.scale.z = 0.15;
        marker.color.r = 0.0;
        marker.color.g = 0.5;
        marker.color.b = 1.0;
        marker.color.a = 0.6;
        marker.lifetime = ros::Duration(0);

        int num_nodes = tprm_->getTemporalGraph().getNumNodes();
        for (int i = 0; i < num_nodes; i++) {
            const auto& node = tprm_->getTemporalGraph().getNode(i);
            geometry_msgs::Point p;
            p.x = node.position[0];
            p.y = node.position[1];
            p.z = node.position[2];
            marker.points.push_back(p);
        }

        samples_pub_.publish(marker);
        ROS_INFO_THROTTLE(2.0, "Published %d sample nodes", num_nodes);
    }

    void publishStartGoal() {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();
        marker.ns = "start_goal";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::SPHERE_LIST;
        marker.action = visualization_msgs::Marker::ADD;
        marker.scale.x = 0.3;
        marker.scale.y = 0.3;
        marker.scale.z = 0.3;
        marker.lifetime = ros::Duration(0);

        geometry_msgs::Point start_p, goal_p;
        start_p.x = start_pos_[0]; start_p.y = start_pos_[1]; start_p.z = start_pos_[2];
        goal_p.x = goal_pos_[0]; goal_p.y = goal_pos_[1]; goal_p.z = goal_pos_[2];
        
        marker.points.push_back(start_p);
        marker.points.push_back(goal_p);

        std_msgs::ColorRGBA start_color, goal_color;
        start_color.r = 0.0; start_color.g = 1.0; start_color.b = 0.0; start_color.a = 1.0;
        goal_color.r = 1.0; goal_color.g = 0.0; goal_color.b = 0.0; goal_color.a = 1.0;
        
        marker.colors.push_back(start_color);
        marker.colors.push_back(goal_color);

        start_goal_pub_.publish(marker);
    }

    void publishSafetyIntervals() {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();
        marker.ns = "utvd_safety_intervals";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::LINE_LIST;
        marker.action = visualization_msgs::Marker::ADD;
        marker.scale.x = 0.05;
        marker.color.r = 1.0;
        marker.color.g = 0.5;
        marker.color.b = 0.0;
        marker.color.a = 0.6;

        // 可视化安全间隔（简化版本）
        int num_nodes = tprm_->getTemporalGraph().getNumNodes();
        for (int i = 0; i < std::min(num_nodes, viz_max_nodes_); i++) { // 只显示前viz_max_nodes_个节点
            const auto& node = tprm_->getTemporalGraph().getNode(i);
            auto intervals = utvd::UTVDSafetyInterval::computeSafeIntervals(
                node.position, obstacles_, robot_radius_);
            
            for (const auto& interval : intervals) {
                if (interval.first < viz_time_horizon_) { // 只显示前viz_time_horizon_秒的安全间隔
                    geometry_msgs::Point p1, p2;
                    p1.x = node.position[0]; p1.y = node.position[1]; p1.z = node.position[2];
                    p2 = p1;
                    p2.z += 0.5; // 垂直偏移表示时间
                    
                    marker.points.push_back(p1);
                    marker.points.push_back(p2);
                }
            }
        }

        safety_intervals_pub_.publish(marker);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "utvd_tprm_dynamic_test");
    TPRMDynamicTestUTVD test;
    ros::spin();
    return 0;
}
