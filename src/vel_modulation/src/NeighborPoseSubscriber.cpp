#include <vel_modulation/NeighborPoseSubscriber.h>

NeighborPoseSubscriber::NeighborPoseSubscriber(ros::NodeHandle& nh, std::string robot_name){   
    actual_x = 0;
    actual_y = 0;
    desire_x = 0;
    desire_y = 0; 
    std::string topic_name = "/" + robot_name + "/odom";
    odom_sub_ = nh.subscribe(topic_name, 10, &NeighborPoseSubscriber::odomCallback, this);
}

void NeighborPoseSubscriber::odomCallback(const nav_msgs::Odometry::ConstPtr &pmsg)
{
    // if (!pmsg) {
    //     ROS_ERROR("Received null pointer!");
    //     return;
    // }       
    // 处理接收到的位姿信息
    // odom = *pmsg;
    pose = pmsg -> pose.pose;
    actual_x = pmsg -> pose.pose.position.x;
    actual_y = pmsg -> pose.pose.position.y;
    // std::cout<< "x:" << pose.position.x << std::endl;
    // std::cout<< "y:" << pose.position.y << std::endl;
}           