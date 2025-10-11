/*
 * @Author: Test PRM Dynamic Obstacles Publisher
 * @Description: Publish dynamic sphere obstacles for PRM testing
 */

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <obj_state_msgs/ObjectsStates.h>
#include <Eigen/Eigen>

#include <vector>
#include <random>
#include <cmath>

using namespace std;
using namespace Eigen;

typedef vector<Eigen::Vector3d> vecVec3;

class DynamicObstaclePublisher {
private:
  ros::NodeHandle nh_;
  ros::Publisher obj_vis_pub_;
  ros::Publisher obj_state_pub_;

  // 参数
  string world_frame_;
  int obs_num_;
  double obs_max_speed_;
  int freq_;
  vector<double> gbbox_o_, gbbox_l_;
  vector<double> size_min_, size_max_;

  // 障碍物状态
  vecVec3 pos_list_, vel_list_, size_list_;
  vecVec3 center_list_;  // 每个障碍物的运动中心点
  vector<double> range_list_;  // 每个障碍物的运动半径
  ros::Time last_t_;

public:
  DynamicObstaclePublisher(ros::NodeHandle& nh) : nh_(nh) {
    // 读取参数
    nh_.param<string>("world_frame", world_frame_, "map");
    nh_.param<int>("obs_num", obs_num_, 5);
    nh_.param<double>("obs_max_speed", obs_max_speed_, 1.0);
    nh_.param<int>("freq", freq_, 30);

    nh_.param("gbbox_min", gbbox_o_, vector<double>({-10.0, -10.0, 0.0}));
    nh_.param("gbbox_size", gbbox_l_, vector<double>({20.0, 20.0, 3.0}));
    nh_.param("size_min", size_min_, vector<double>({0.3, 0.3, 0.3}));
    nh_.param("size_max", size_max_, vector<double>({0.8, 0.8, 0.8}));

    // 初始化发布器
    obj_vis_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/dyn_obstacles_vis", 10);
    obj_state_pub_ = nh_.advertise<obj_state_msgs::ObjectsStates>("/obj_states", 10);

    // 初始化障碍物
    initializeObstacles();

    ROS_INFO("Dynamic Obstacle Publisher initialized!");
    ROS_INFO("  - Number of obstacles: %d", obs_num_);
    ROS_INFO("  - Max speed: %.2f m/s", obs_max_speed_);
    ROS_INFO("  - Bounding box: [%.1f, %.1f, %.1f] to [%.1f, %.1f, %.1f]",
             gbbox_o_[0], gbbox_o_[1], gbbox_o_[2],
             gbbox_o_[0] + gbbox_l_[0], gbbox_o_[1] + gbbox_l_[1], gbbox_o_[2] + gbbox_l_[2]);
  }

  void initializeObstacles() {
    srand((unsigned)time(NULL));

    for (int i = 0; i < obs_num_; i++) {
      Vector3d pos, vel, size, center;

      // 随机生成运动中心位置
      center(0) = gbbox_o_[0] + (rand() % 1000) / 1000.0 * gbbox_l_[0];
      center(1) = gbbox_o_[1] + (rand() % 1000) / 1000.0 * gbbox_l_[1];
      center(2) = gbbox_o_[2] + (rand() % 1000) / 1000.0 * gbbox_l_[2];

      // 随机生成往返运动的半径（1-3米）
      double range = 1.0 + (rand() % 1000) / 1000.0 * 2.0;

      // 初始位置在中心点附近随机偏移
      pos(0) = center(0) + ((rand() % 1000) / 1000.0 * 2.0 - 1.0) * range * 0.5;
      pos(1) = center(1) + ((rand() % 1000) / 1000.0 * 2.0 - 1.0) * range * 0.5;
      pos(2) = center(2) + ((rand() % 1000) / 1000.0 * 2.0 - 1.0) * range * 0.5;

      // 随机生成球体直径
      double diameter = (rand() % 1000) / 1000.0 * (size_max_[0] - size_min_[0]) + size_min_[0];
      size(0) = diameter;
      size(1) = diameter;
      size(2) = diameter;

      // 随机生成初始速度
      vel(0) = ((rand() % 1000) / 1000.0 * 2.0 - 1.0) * obs_max_speed_;
      vel(1) = ((rand() % 1000) / 1000.0 * 2.0 - 1.0) * obs_max_speed_;
      vel(2) = ((rand() % 1000) / 1000.0 * 2.0 - 1.0) * obs_max_speed_;

      pos_list_.push_back(pos);
      vel_list_.push_back(vel);
      size_list_.push_back(size);
      center_list_.push_back(center);
      range_list_.push_back(range);
    }

    last_t_ = ros::Time::now();
    ROS_INFO("✓ Initialized %d obstacles with oscillating motion (range: 1-3m)", obs_num_);
  }

  void updateObstacles() {
    double t_gap = (ros::Time::now() - last_t_).toSec();

    for (int i = 0; i < obs_num_; i++) {
      // 先更新位置
      pos_list_[i] += t_gap * vel_list_[i];

      // 往返运动：检查是否超出中心点±运动半径的范围
      // 对每个轴独立处理
      for (int axis = 0; axis < 3; ++axis) {
        double dist_from_center = pos_list_[i](axis) - center_list_[i](axis);
        double max_range = range_list_[i];

        // 如果超出运动范围，反转该轴的速度
        if (dist_from_center < -max_range) {
          pos_list_[i](axis) = center_list_[i](axis) - max_range;
          vel_list_[i](axis) = fabs(vel_list_[i](axis));  // 正方向
        } else if (dist_from_center > max_range) {
          pos_list_[i](axis) = center_list_[i](axis) + max_range;
          vel_list_[i](axis) = -fabs(vel_list_[i](axis));  // 负方向
        }
      }

      // 额外安全检查：确保不超出全局边界
      double margin = 0.3;
      for (int axis = 0; axis < 3; ++axis) {
        if (pos_list_[i](axis) < gbbox_o_[axis] + margin) {
          pos_list_[i](axis) = gbbox_o_[axis] + margin;
          vel_list_[i](axis) = fabs(vel_list_[i](axis));
        } else if (pos_list_[i](axis) > gbbox_o_[axis] + gbbox_l_[axis] - margin) {
          pos_list_[i](axis) = gbbox_o_[axis] + gbbox_l_[axis] - margin;
          vel_list_[i](axis) = -fabs(vel_list_[i](axis));
        }
      }
    }

    last_t_ = ros::Time::now();
  }

  void publishObjectStates() {
    obj_state_msgs::ObjectsStates states;
    states.header.frame_id = world_frame_;
    states.header.stamp = last_t_;

    for (int i = 0; i < obs_num_; i++) {
      obj_state_msgs::State state;
      state.position.x = pos_list_[i](0);
      state.position.y = pos_list_[i](1);
      state.position.z = pos_list_[i](2);
      state.velocity.x = vel_list_[i](0);
      state.velocity.y = vel_list_[i](1);
      state.velocity.z = vel_list_[i](2);
      state.size.x = size_list_[i](0);
      state.size.y = size_list_[i](0);
      state.size.z = size_list_[i](0);
      state.acceleration.x = 0;
      state.acceleration.y = 0;
      state.acceleration.z = 0;
      states.states.push_back(state);
    }

    obj_state_pub_.publish(states);
  }

  void publishVisualization() {
    visualization_msgs::MarkerArray marker_array;
    int id = 0;

    for (int i = 0; i < obs_num_; i++) {
      // 球体标记
      visualization_msgs::Marker sphere;
      sphere.header.frame_id = world_frame_;
      sphere.header.stamp = ros::Time::now();
      sphere.ns = "dynamic_obstacles";
      sphere.id = id++;
      sphere.type = visualization_msgs::Marker::SPHERE;
      sphere.action = visualization_msgs::Marker::ADD;

      sphere.pose.position.x = pos_list_[i](0);
      sphere.pose.position.y = pos_list_[i](1);
      sphere.pose.position.z = pos_list_[i](2);
      sphere.pose.orientation.w = 1.0;

      sphere.scale.x = size_list_[i](0);
      sphere.scale.y = size_list_[i](0);
      sphere.scale.z = size_list_[i](0);

      sphere.color.r = 1.0;
      sphere.color.g = 0.3;
      sphere.color.b = 0.0;
      sphere.color.a = 0.6;

      sphere.lifetime = ros::Duration(0.2);

      marker_array.markers.push_back(sphere);

      // 速度箭头
      visualization_msgs::Marker arrow;
      arrow.header.frame_id = world_frame_;
      arrow.header.stamp = ros::Time::now();
      arrow.ns = "velocity_arrows";
      arrow.id = id++;
      arrow.type = visualization_msgs::Marker::ARROW;
      arrow.action = visualization_msgs::Marker::ADD;

      geometry_msgs::Point p1, p2;
      p1.x = pos_list_[i](0);
      p1.y = pos_list_[i](1);
      p1.z = pos_list_[i](2);
      p2.x = p1.x + vel_list_[i](0);
      p2.y = p1.y + vel_list_[i](1);
      p2.z = p1.z + vel_list_[i](2);

      arrow.points.push_back(p1);
      arrow.points.push_back(p2);
      arrow.pose.orientation.w = 1.0;

      arrow.scale.x = 0.1;  // 箭杆直径
      arrow.scale.y = 0.2;  // 箭头直径
      arrow.scale.z = 0.0;

      arrow.color.r = 0.0;
      arrow.color.g = 1.0;
      arrow.color.b = 0.0;
      arrow.color.a = 1.0;

      arrow.lifetime = ros::Duration(0.2);

      marker_array.markers.push_back(arrow);
    }

    obj_vis_pub_.publish(marker_array);
  }

  void run() {
    ros::Rate loop_rate(freq_);

    while (ros::ok()) {
      updateObstacles();
      publishObjectStates();
      publishVisualization();

      ros::spinOnce();
      loop_rate.sleep();
    }
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "dynamic_obstacle_publisher");
  ros::NodeHandle nh("~");

  DynamicObstaclePublisher publisher(nh);
  publisher.run();

  return 0;
}
