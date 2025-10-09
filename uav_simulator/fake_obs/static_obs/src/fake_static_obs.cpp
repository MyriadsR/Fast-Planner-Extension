#include <stdio.h>
#include <iostream>
#include <Eigen/Eigen>

#include <ros/ros.h>

#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <obj_state_msgs/ObjectsStates.h>

typedef std::vector<Eigen::Vector3d> vecVec3;
using namespace std;
using namespace Eigen;

ros::Publisher obj_vis_pub, obj_state_pub;

void obj_state_pb(const vecVec3 &ps, const vecVec3 &vs, const vecVec3 &ss, const ros::Time &ts, const string &world_frame_id)
{
    obj_state_msgs::ObjectsStates states;

    states.header.frame_id = world_frame_id;
    states.header.stamp = ts;

    for (uint i = 0; i < ps.size(); i++)
    {
        obj_state_msgs::State state;
        state.position.x = ps[i](0);
        state.position.y = ps[i](1);
        state.position.z = ps[i](2);
        state.velocity.x = vs[i](0);
        state.velocity.y = vs[i](1);
        state.velocity.z = vs[i](2);
        // 对于球体，三个维度都使用相同的直径
        state.size.x = ss[i](0);
        state.size.y = ss[i](0);
        state.size.z = ss[i](0);
        state.acceleration.x = 0;
        state.acceleration.y = 0;
        state.acceleration.z = 0;
        states.states.push_back(state);
    }
    obj_state_pub.publish(states);
}

void static_obs_pb(const vecVec3 &ps, const vecVec3 &ss, const ros::Time &ts, const string &world_frame_id)
{
    visualization_msgs::MarkerArray markers;
    int id = 0;

    for (uint i = 0; i < ps.size(); i++)
    {
        visualization_msgs::Marker obs;
        obs.header.frame_id = world_frame_id;
        obs.header.stamp = ros::Time::now();
        // 球体类型
        obs.type = visualization_msgs::Marker::SPHERE;
        obs.pose.position.x = ps[i](0);
        obs.pose.position.y = ps[i](1);
        obs.pose.position.z = ps[i](2);
        obs.id = id++;
        // 球体的三个scale维度相同
        obs.scale.x = ss[i](0);
        obs.scale.y = ss[i](0);
        obs.scale.z = ss[i](0);

        // 静态障碍物使用红色显示
        obs.color.a = 0.8;
        obs.color.r = 1.0;
        obs.color.g = 0.0;
        obs.color.b = 0.0;
        obs.pose.orientation.x = 0;
        obs.pose.orientation.y = 0;
        obs.pose.orientation.z = 0;
        obs.pose.orientation.w = 1.0;

        // 静态障碍物不需要设置lifetime，让它一直显示
        obs.lifetime = ros::Duration(0);

        // 添加文本标签（可选）
        visualization_msgs::Marker obsid;
        obsid.header.frame_id = world_frame_id;
        obsid.header.stamp = ros::Time::now();
        obsid.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        obsid.pose.position.x = ps[i](0);
        obsid.pose.position.y = ps[i](1);
        obsid.pose.position.z = ps[i](2) + ss[i](0) / 2.0 + 0.3;
        obsid.id = id++;
        obsid.pose.orientation.w = 1;
        obsid.scale.z = 0.5;
        obsid.text = "S" + to_string(i);  // S表示Static
        obsid.color.a = 1;
        obsid.color.r = 1.0;
        obsid.color.g = 1.0;
        obsid.color.b = 1.0;
        obsid.lifetime = ros::Duration(0);

        markers.markers.push_back(obs);
        markers.markers.push_back(obsid);
    }
    obj_vis_pub.publish(markers);
}

int main(int argc, char **argv)
{
    // Initialize ROS
    ros::init(argc, argv, "fake_static_obs_publisher");
    ros::NodeHandle nh("~");

    string world_frame = "map";
    int obs_num = 5;

    ros::Rate loop_rate(1);  // 1 Hz，因为是静态障碍物，不需要高频率更新
    vector<double> gbbox_o, gbbox_l, size_min, size_max;
    nh.getParam("WorldFrameName", world_frame);

    // 障碍物活动区域
    nh.getParam("GlobalBox_min", gbbox_o);
    nh.getParam("GlobalBox_size", gbbox_l);

    // 障碍物尺寸范围
    nh.getParam("StaticObsSize_min", size_min);
    nh.getParam("StaticObsSize_max", size_max);
    nh.getParam("StaticObsNum", obs_num);

    // 发送RViz可视化markers
    obj_vis_pub = nh.advertise<visualization_msgs::MarkerArray>("/static_obs", 10);

    // 发送自定义消息，包含障碍物的位置、速度（为0）、尺寸
    obj_state_pub = nh.advertise<obj_state_msgs::ObjectsStates>("/static_obj_states", 10);

    vecVec3 pos_list, vel_list, size_list;

    // 初始化静态障碍物
    srand((unsigned)time(NULL));
    for (int i = 0; i < obs_num; i++)
    {
        Vector3d pos, vel, size;

        // 在全局范围内随机生成位置
        pos(0) = gbbox_o[0] + (double)(rand() % 1000) / 1000.0 * gbbox_l[0];
        pos(1) = gbbox_o[1] + (double)(rand() % 1000) / 1000.0 * gbbox_l[1];
        pos(2) = gbbox_o[2] + (double)(rand() % 1000) / 1000.0 * gbbox_l[2];

        // 为球体生成随机直径
        double diameter = (double)(rand() % 101) / 101.0 * (size_max[0] - size_min[0]) + size_min[0];
        size(0) = diameter;
        size(1) = diameter;
        size(2) = diameter;

        // 静态障碍物速度为0
        vel << 0.0, 0.0, 0.0;

        pos_list.emplace_back(pos);
        vel_list.emplace_back(vel);
        size_list.emplace_back(size);
    }

    ROS_INFO("Static obstacles initialized: %d obstacles", obs_num);

    // 等待一小段时间，确保订阅者连接
    ros::Duration(0.5).sleep();

    while (nh.ok())
    {
        ros::Time current_time = ros::Time::now();

        // 发布静态障碍物状态
        obj_state_pb(pos_list, vel_list, size_list, current_time, world_frame);
        static_obs_pb(pos_list, size_list, current_time, world_frame);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
