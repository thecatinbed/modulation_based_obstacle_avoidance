#include <vel_modulation/NeighborPoseSubscriber.h>

// NeighborPoseSubscriber::NeighborPoseSubscriber(ros::NodeHandle& nh, std::string robot_name){   
//     actual_x = 0;
//     actual_y = 0;
//     desire_x = 0;
//     desire_y = 0; 
//     vel_x = 0;
//     vel_y = 0;
//     std::string topic_name = "/" + robot_name + "/odom";
//     odom_sub_ = nh.subscribe(topic_name, 10, &NeighborPoseSubscriber::odomCallback, this);
// }

void NeighborPoseSubscriber::odomCallback(const nav_msgs::Odometry::ConstPtr &pmsg)
{
    if (!pmsg) {
        ROS_ERROR("Received null pointer!");
        return;
    }      
    tf::Quaternion quaternion(pmsg -> pose.pose.orientation.x, pmsg -> pose.pose.orientation.y, pmsg -> pose.pose.orientation.z, pmsg -> pose.pose.orientation.w); 
    double roll, pitch, yaw;
    // 处理接收到的位姿信息
    pose = pmsg -> pose.pose;
    actual_x = pmsg -> pose.pose.position.x;
    actual_y = pmsg -> pose.pose.position.y;
    double vel = pmsg -> twist.twist.linear.z;
    tf::Matrix3x3 mat(quaternion);
    mat.getRPY(roll, pitch, yaw);
    vel_x = vel * cos(yaw);
    vel_y = vel * sin(yaw);
} 

void NeighborPoseSubscriber::odomCallback(const geometry_msgs::PoseStamped::ConstPtr &pmsg)
{
    if (!pmsg) {
        ROS_ERROR("Received null pointer!");
        return;
    }      
    tf::Quaternion quaternion(pmsg -> pose.orientation.x, pmsg -> pose.orientation.y, pmsg -> pose.orientation.z, pmsg -> pose.orientation.w); 
    double roll, pitch, yaw;
    // 处理接收到的位姿信息
    pose = pmsg -> pose;
    actual_x = pmsg -> pose.position.x;
    actual_y = pmsg -> pose.position.y;
    double vel = 0;
    tf::Matrix3x3 mat(quaternion);
    mat.getRPY(roll, pitch, yaw);
    vel_x = vel * cos(yaw);
    vel_y = vel * sin(yaw);
} 