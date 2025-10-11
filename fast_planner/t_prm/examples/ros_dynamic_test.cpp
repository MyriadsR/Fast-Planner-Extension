/*********************************************************************
 * ROS T-PRM Dynamic Obstacle Test
 *
 * This node subscribes to dynamic sphere obstacles from fake_obs_sphere
 * and uses T-PRM to plan collision-free paths in real-time.
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

class TPRMDynamicTest {
private:
    ros::NodeHandle nh_;
    ros::Subscriber obs_sub_;
    ros::Subscriber goal_sub_;
    ros::Publisher path_pub_;
    ros::Publisher path_marker_pub_;
    ros::Publisher samples_pub_;
    ros::Publisher start_goal_pub_;
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

public:
    TPRMDynamicTest() : nh_("~"), has_goal_(false) {
        // 获取参数
        nh_.param("robot_speed", robot_speed_, 1.0);
        nh_.param("num_samples", num_samples_, 200);
        nh_.param("connection_radius", connection_radius_, 2.0);
        nh_.param("planning_freq", planning_freq_, 1.0);

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

        // 初始化T-PRM，指定采样空间范围
        tprm_ = std::make_shared<tprm::TemporalPRM>(env_min_, env_max_);

        ROS_INFO("Environment bounds: [%.2f, %.2f, %.2f] to [%.2f, %.2f, %.2f]",
                 env_min_[0], env_min_[1], env_min_[2],
                 env_max_[0], env_max_[1], env_max_[2]);

        // 订阅障碍物状态
        obs_sub_ = nh_.subscribe("/obj_states", 10, &TPRMDynamicTest::obstaclesCallback, this);

        // 订阅目标点 (可选，使用RViz的2D Nav Goal)
        goal_sub_ = nh_.subscribe("/move_base_simple/goal", 1, &TPRMDynamicTest::goalCallback, this);

        // 发布路径
        path_pub_ = nh_.advertise<nav_msgs::Path>("/tprm_path", 10);
        path_marker_pub_ = nh_.advertise<visualization_msgs::Marker>("/tprm_path_marker", 10);
        samples_pub_ = nh_.advertise<visualization_msgs::Marker>("/tprm_samples", 10);
        start_goal_pub_ = nh_.advertise<visualization_msgs::Marker>("/tprm_start_goal", 10);

        // 定时规划
        plan_timer_ = nh_.createTimer(ros::Duration(1.0 / planning_freq_), &TPRMDynamicTest::planningCallback, this);

        ROS_INFO("T-PRM Dynamic Test Node Initialized");
        ROS_INFO("Start: [%.2f, %.2f, %.2f]", start_pos_[0], start_pos_[1], start_pos_[2]);
        ROS_INFO("Goal: [%.2f, %.2f, %.2f]", goal_pos_[0], goal_pos_[1], goal_pos_[2]);
    }

    void obstaclesCallback(const obj_state_msgs::ObjectsStates::ConstPtr& msg) {
        // 清空旧的障碍物
        obstacles_.clear();
        // 重新创建T-PRM，保持环境范围参数
        tprm_ = std::make_shared<tprm::TemporalPRM>(env_min_, env_max_);

        // 添加新的动态障碍物
        for (const auto& state : msg->states) {
            tprm::Vector3d position(state.position.x, state.position.y, state.position.z);
            tprm::Vector3d velocity(state.velocity.x, state.velocity.y, state.velocity.z);
            double radius = state.size.x / 2.0;  // 直径转半径

            auto obstacle = std::make_shared<tprm::DynamicSphereObstacle>(position, velocity, radius);
            obstacles_.push_back(obstacle);
            tprm_->addDynamicObstacle(obstacle);
        }

        ROS_INFO_THROTTLE(2.0, "Received %lu dynamic obstacles", obstacles_.size());
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

        ROS_INFO("Planning path with T-PRM...");

        // 清空之前的图
        tprm_->getTemporalGraph().clear();

        // 重新构建PRM图
        // 计时 placeSamples
        auto t_place_start = std::chrono::steady_clock::now();
        tprm_->placeSamples(num_samples_);
        auto t_place_end = std::chrono::steady_clock::now();
        double place_ms = std::chrono::duration<double, std::milli>(t_place_end - t_place_start).count();
        ROS_INFO("placeSamples took %.3f ms", place_ms);

        // 计时 buildPRM
        auto t_build_start = std::chrono::steady_clock::now();
        tprm_->buildPRM(connection_radius_);
        auto t_build_end = std::chrono::steady_clock::now();
        double build_ms = std::chrono::duration<double, std::milli>(t_build_end - t_build_start).count();
        ROS_INFO("buildPRM took %.3f ms", build_ms);
        ROS_INFO("Total PRM rebuild time: %.3f ms", place_ms + build_ms);

        ROS_INFO("PRM Graph - Nodes: %d, Edges: %d",
                 tprm_->getTemporalGraph().getNumNodes(),
                 tprm_->getTemporalGraph().getNumEdges());

        // 可视化采样节点
        publishSamples();
        publishStartGoal();

        // 规划路径
        double start_time = ros::Time::now().toSec();

        ROS_INFO("Start position: [%.2f, %.2f, %.2f] at time %.2f",
                 start_pos_[0], start_pos_[1], start_pos_[2], start_time);
        ROS_INFO("Goal position: [%.2f, %.2f, %.2f]",
                 goal_pos_[0], goal_pos_[1], goal_pos_[2]);

        auto path = tprm_->getShortestPath(start_pos_, goal_pos_, start_time);

        if (path.empty()) {
            ROS_WARN("No path found! Possible reasons:");
            ROS_WARN("  - Start or goal blocked by obstacles");
            ROS_WARN("  - No graph connectivity between start and goal");
            ROS_WARN("  - Connection radius (%.2f) too small", connection_radius_);
            ROS_WARN("  - Not enough samples (%d)", num_samples_);
        } else {
            ROS_INFO("Path found with %lu waypoints", path.size());
            publishPath(path);
        }
    }

    void publishPath(const std::vector<tprm::PathResultEntry>& path) {
        // 发布nav_msgs::Path
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

        // 发布可视化Marker
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();
        marker.ns = "tprm_path";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::LINE_STRIP;
        marker.action = visualization_msgs::Marker::ADD;
        marker.scale.x = 0.1;  // 线宽
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
        marker.scale.x = 0.15;  // 球体直径
        marker.scale.y = 0.15;
        marker.scale.z = 0.15;
        marker.color.r = 0.0;
        marker.color.g = 0.5;
        marker.color.b = 1.0;
        marker.color.a = 0.6;
        marker.lifetime = ros::Duration(0);  // 永久显示

        // 获取所有采样节点
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
        marker.scale.x = 0.3;  // 更大的球体
        marker.scale.y = 0.3;
        marker.scale.z = 0.3;
        marker.lifetime = ros::Duration(0);  // 永久显示

        // 起点 - 绿色
        geometry_msgs::Point start_p;
        start_p.x = start_pos_[0];
        start_p.y = start_pos_[1];
        start_p.z = start_pos_[2];
        marker.points.push_back(start_p);

        std_msgs::ColorRGBA start_color;
        start_color.r = 0.0;
        start_color.g = 1.0;
        start_color.b = 0.0;
        start_color.a = 1.0;
        marker.colors.push_back(start_color);

        // 终点 - 红色
        geometry_msgs::Point goal_p;
        goal_p.x = goal_pos_[0];
        goal_p.y = goal_pos_[1];
        goal_p.z = goal_pos_[2];
        marker.points.push_back(goal_p);

        std_msgs::ColorRGBA goal_color;
        goal_color.r = 1.0;
        goal_color.g = 0.0;
        goal_color.b = 0.0;
        goal_color.a = 1.0;
        marker.colors.push_back(goal_color);

        start_goal_pub_.publish(marker);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "tprm_dynamic_test");

    TPRMDynamicTest test;

    ros::spin();

    return 0;
}
