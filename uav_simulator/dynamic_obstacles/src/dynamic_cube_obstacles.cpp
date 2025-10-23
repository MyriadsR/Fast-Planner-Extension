/*
 * @Author: Dynamic Cube Obstacles Publisher
 * @Description: Publish dynamic cube obstacles as point clouds for ESDF conversion
 * @Date: 2025
 */

#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <Eigen/Eigen>

#include <vector>
#include <random>
#include <cmath>

using namespace std;
using namespace Eigen;

class DynamicCubeObstacles {
private:
  ros::NodeHandle nh_;
  ros::Publisher cloud_pub_;
  ros::Publisher vis_pub_;
  ros::Timer timer_;

  // 参数
  string world_frame_;
  int obs_num_;
  double obs_max_speed_;
  double freq_;
  double resolution_;  // 点云分辨率，用于填充立方体

  // 地图边界（与 topo_replan.launch 一致）
  Vector3d map_min_;   // [-20, -10, 0]
  Vector3d map_max_;   // [20, 10, 5]

  // 立方体尺寸范围
  Vector3d size_min_;
  Vector3d size_max_;

  // 障碍物状态
  struct Obstacle {
    Vector3d position;
    Vector3d velocity;
    Vector3d size;
  };
  vector<Obstacle> obstacles_;

  ros::Time last_update_time_;

public:
  DynamicCubeObstacles(ros::NodeHandle& nh) : nh_(nh) {
    // 读取参数
    nh_.param<string>("world_frame", world_frame_, "world");
    nh_.param<int>("obs_num", obs_num_, 5);
    nh_.param<double>("obs_max_speed", obs_max_speed_, 1.0);
    nh_.param<double>("freq", freq_, 10.0);  // 降低频率减少计算量
    nh_.param<double>("resolution", resolution_, 0.2);  // 点云分辨率

    // 地图边界（基于 topo_replan.launch: 40x20x5）
    double map_size_x = 40.0;
    double map_size_y = 20.0;
    double map_size_z = 5.0;

    map_min_ = Vector3d(-map_size_x/2, -map_size_y/2, 0.0);
    map_max_ = Vector3d(map_size_x/2, map_size_y/2, map_size_z);

    // 立方体尺寸范围
    nh_.param("size_min_x", size_min_(0), 0.5);
    nh_.param("size_min_y", size_min_(1), 0.5);
    nh_.param("size_min_z", size_min_(2), 0.5);

    nh_.param("size_max_x", size_max_(0), 1.5);
    nh_.param("size_max_y", size_max_(1), 1.5);
    nh_.param("size_max_z", size_max_(2), 2.0);

    // 初始化发布器
    cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/dynamic_obstacles/cloud", 10);
    vis_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/dynamic_obstacles/vis", 10);

    // 初始化障碍物
    initializeObstacles();

    // 设置定时器
    timer_ = nh_.createTimer(ros::Duration(1.0 / freq_), &DynamicCubeObstacles::timerCallback, this);

    ROS_INFO("=== Dynamic Cube Obstacles Publisher ===");
    ROS_INFO("  Number of obstacles: %d", obs_num_);
    ROS_INFO("  Max speed: %.2f m/s", obs_max_speed_);
    ROS_INFO("  Update frequency: %.1f Hz", freq_);
    ROS_INFO("  Point cloud resolution: %.2f m", resolution_);
    ROS_INFO("  Map boundary: [%.1f, %.1f, %.1f] to [%.1f, %.1f, %.1f]",
             map_min_(0), map_min_(1), map_min_(2),
             map_max_(0), map_max_(1), map_max_(2));
  }

  void initializeObstacles() {
    srand((unsigned)time(NULL));

    double margin = 2.0;  // 边界留白

    for (int i = 0; i < obs_num_; i++) {
      Obstacle obs;

      // 随机生成初始位置（避免边界）
      obs.position(0) = map_min_(0) + margin +
                        (rand() % 1000) / 1000.0 * (map_max_(0) - map_min_(0) - 2 * margin);
      obs.position(1) = map_min_(1) + margin +
                        (rand() % 1000) / 1000.0 * (map_max_(1) - map_min_(1) - 2 * margin);
      obs.position(2) = map_min_(2) + 0.5 +
                        (rand() % 1000) / 1000.0 * (map_max_(2) - map_min_(2) - 1.0);

      // 随机生成立方体尺寸
      obs.size(0) = size_min_(0) + (rand() % 1000) / 1000.0 * (size_max_(0) - size_min_(0));
      obs.size(1) = size_min_(1) + (rand() % 1000) / 1000.0 * (size_max_(1) - size_min_(1));
      obs.size(2) = size_min_(2) + (rand() % 1000) / 1000.0 * (size_max_(2) - size_min_(2));

      // 随机生成初始速度
      double speed_factor = 0.5 + (rand() % 1000) / 1000.0 * 0.5;
      obs.velocity(0) = ((rand() % 1000) / 1000.0 * 2.0 - 1.0) * obs_max_speed_ * speed_factor;
      obs.velocity(1) = ((rand() % 1000) / 1000.0 * 2.0 - 1.0) * obs_max_speed_ * speed_factor;
      obs.velocity(2) = ((rand() % 1000) / 1000.0 * 2.0 - 1.0) * obs_max_speed_ * speed_factor * 0.3;  // z轴速度较小

      obstacles_.push_back(obs);
    }

    last_update_time_ = ros::Time::now();
    ROS_INFO("✓ Initialized %d cube obstacles", obs_num_);
  }

  void updateObstacles() {
    ros::Time current_time = ros::Time::now();
    double dt = (current_time - last_update_time_).toSec();

    for (auto& obs : obstacles_) {
      // 更新位置
      obs.position += dt * obs.velocity;

      // 边界碰撞检测与反弹
      for (int axis = 0; axis < 3; ++axis) {
        double half_size = obs.size(axis) / 2.0;

        if (obs.position(axis) - half_size < map_min_(axis)) {
          obs.position(axis) = map_min_(axis) + half_size;
          obs.velocity(axis) = -obs.velocity(axis);  // 反弹
        }
        else if (obs.position(axis) + half_size > map_max_(axis)) {
          obs.position(axis) = map_max_(axis) - half_size;
          obs.velocity(axis) = -obs.velocity(axis);  // 反弹
        }
      }
    }

    last_update_time_ = current_time;
  }

  // 为每个立方体生成点云（填充内部）
  void generatePointCloud(pcl::PointCloud<pcl::PointXYZ>& cloud) {
    cloud.clear();

    for (const auto& obs : obstacles_) {
      Vector3d half_size = obs.size / 2.0;

      // 填充立方体内部点云
      for (double x = -half_size(0); x <= half_size(0); x += resolution_) {
        for (double y = -half_size(1); y <= half_size(1); y += resolution_) {
          for (double z = -half_size(2); z <= half_size(2); z += resolution_) {
            pcl::PointXYZ pt;
            pt.x = obs.position(0) + x;
            pt.y = obs.position(1) + y;
            pt.z = obs.position(2) + z;
            cloud.push_back(pt);
          }
        }
      }
    }

    cloud.width = cloud.points.size();
    cloud.height = 1;
    cloud.is_dense = true;
  }

  void publishPointCloud() {
    pcl::PointCloud<pcl::PointXYZ> cloud;
    generatePointCloud(cloud);

    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(cloud, cloud_msg);
    cloud_msg.header.frame_id = world_frame_;
    cloud_msg.header.stamp = ros::Time::now();

    cloud_pub_.publish(cloud_msg);
  }

  void publishVisualization() {
    visualization_msgs::MarkerArray marker_array;

    for (size_t i = 0; i < obstacles_.size(); ++i) {
      const auto& obs = obstacles_[i];

      // 立方体标记
      visualization_msgs::Marker cube;
      cube.header.frame_id = world_frame_;
      cube.header.stamp = ros::Time::now();
      cube.ns = "dynamic_cubes";
      cube.id = i * 2;
      cube.type = visualization_msgs::Marker::CUBE;
      cube.action = visualization_msgs::Marker::ADD;

      cube.pose.position.x = obs.position(0);
      cube.pose.position.y = obs.position(1);
      cube.pose.position.z = obs.position(2);
      cube.pose.orientation.w = 1.0;

      cube.scale.x = obs.size(0);
      cube.scale.y = obs.size(1);
      cube.scale.z = obs.size(2);

      cube.color.r = 1.0;
      cube.color.g = 0.5;
      cube.color.b = 0.0;
      cube.color.a = 0.6;

      cube.lifetime = ros::Duration(0.2);
      marker_array.markers.push_back(cube);

      // 速度箭头
      visualization_msgs::Marker arrow;
      arrow.header.frame_id = world_frame_;
      arrow.header.stamp = ros::Time::now();
      arrow.ns = "velocity_arrows";
      arrow.id = i * 2 + 1;
      arrow.type = visualization_msgs::Marker::ARROW;
      arrow.action = visualization_msgs::Marker::ADD;

      geometry_msgs::Point p1, p2;
      p1.x = obs.position(0);
      p1.y = obs.position(1);
      p1.z = obs.position(2);
      p2.x = p1.x + obs.velocity(0);
      p2.y = p1.y + obs.velocity(1);
      p2.z = p1.z + obs.velocity(2);

      arrow.points.push_back(p1);
      arrow.points.push_back(p2);

      arrow.scale.x = 0.1;
      arrow.scale.y = 0.2;
      arrow.scale.z = 0.0;

      arrow.color.r = 0.0;
      arrow.color.g = 1.0;
      arrow.color.b = 0.0;
      arrow.color.a = 1.0;

      arrow.lifetime = ros::Duration(0.2);
      marker_array.markers.push_back(arrow);
    }

    vis_pub_.publish(marker_array);
  }

  void timerCallback(const ros::TimerEvent&) {
    updateObstacles();
    publishPointCloud();
    publishVisualization();
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "dynamic_cube_obstacles");
  ros::NodeHandle nh("~");

  DynamicCubeObstacles publisher(nh);

  ros::spin();

  return 0;
}
