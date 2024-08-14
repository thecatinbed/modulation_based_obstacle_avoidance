#ifndef NEIGHBOR_POSE_SUBSCRIBER_H
#define NEIGHBOR_POSE_SUBSCRIBER_H
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>

class NeighborPoseSubscriber
{
    public:
        nav_msgs::Odometry odom;
        double actual_x, actual_y;
        double desire_x, desire_y;
        NeighborPoseSubscriber() {
            actual_x = 0;
            actual_y = 0;
            desire_x = 0;
            desire_y = 0;
        };
        NeighborPoseSubscriber(ros::NodeHandle& nh, std::string robot_name);
        void odomCallback(const nav_msgs::Odometry::ConstPtr &pmsg);
        geometry_msgs::Pose getPose() const { return pose; }
    private:
        ros::Subscriber odom_sub_;
        geometry_msgs::Pose pose;
};
#endif