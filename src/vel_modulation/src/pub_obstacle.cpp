#include <ros/ros.h>
#include <geometry_msgs/Point.h>

int main(int argc, char** argv){
    // 初始化控制节点
    ros::init(argc, argv, "pub_obstacle");
    // 定义控制变量
    ros::NodeHandle nh;
    ros::Publisher obstacle_pub = nh.advertise<geometry_msgs::Point>("obstacle_position", 10);
    ros::Publisher obstacle_outer_pub = nh.advertise<geometry_msgs::Point>("obstacle_outer_position", 10);
    geometry_msgs::Point obstacle_position, obstacle_outer_position;
    ros::Rate rate(100);
    ros::Duration(1).sleep();
    ros::Time starting_time = ros::Time::now();
    while (ros::ok())
    {
        ros::Time current_time = ros::Time::now();
        ros::Duration duration = current_time - starting_time;
        double x = 2.1 + 0.20 * cos(20 * duration.toSec());
        double y = - 0.9 + 0.20 * sin(20 * duration.toSec());
        obstacle_position.x = x;
        obstacle_position.y = y;
        x = 2.1 + 0.8 * cos(20 * duration.toSec());
        y = - 0.9 + 0.8 * sin(20 * duration.toSec());
        obstacle_outer_position.x = x;
        obstacle_outer_position.y = y;
        obstacle_pub.publish(obstacle_position);
        obstacle_outer_pub.publish(obstacle_outer_position);
        rate.sleep();
        ros::spinOnce();
    }
}