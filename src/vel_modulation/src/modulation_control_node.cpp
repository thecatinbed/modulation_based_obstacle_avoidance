#include <ros/ros.h>
#include <vel_modulation/TwoWheel_Linear_controller.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/Marker.h>
#include <tf/tf.h>

nav_msgs::Odometry odom;
geometry_msgs::Twist cmd;

struct Trajectory {
    double x;
    double y;
};

Trajectory get_car_trajectory(double t){
    Trajectory traj;
    traj.x = 2 * (0 + cos(0.1 * t - M_PI / 2));
    traj.y = 2 * (1 + sin(0.1 * t - M_PI / 2));
    return traj;
}

void TwoWheel_odom_callback(const nav_msgs::OdometryConstPtr &pmsg){
    odom = *pmsg;
}

int sign(double num){
    if (num > 0)
    {
        return 1;
    }else if(num < 0){
        return -1;
    }else{
        return 0;
    }
    
}

double limit_range(double value, double range){
    if (abs(value) > range)
    {
        return range * sign(value);
    }else{
        return value;
    }
}

int main(int argc, char** argv){
    // 初始化控制节点
    ros::init(argc, argv, "TwoWheelControlNode");
    // 初始化节点句柄
    ros::NodeHandle nh;
    // 初始化订阅者以及发布者
    ros::Subscriber odom_sub = nh.subscribe("odom", 1, TwoWheel_odom_callback);
    ros::Publisher cmd_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    ros::Publisher cmd_marker_pub = nh.advertise<visualization_msgs::Marker>("cmd_marker", 10);
    ros::Publisher modulated_marker_pub = nh.advertise<visualization_msgs::Marker>("modulated_marker", 10);
    ros::Publisher normal_marker_pub = nh.advertise<visualization_msgs::Marker>("normal_marker", 10);

     // 定义参数变量
    double linear_kp, linear_ki, linear_kd;
    double angular_kp, angular_ki, angular_kd;

    // 从参数服务器读取参数
    nh.param("controller/linear_kp", linear_kp, 1.1);
    nh.param("controller/linear_ki", linear_ki, 0.0);
    nh.param("controller/linear_kd", linear_kd, 0.0);
    nh.param("controller/angular_kp", angular_kp, 1.1);
    nh.param("controller/angular_ki", angular_ki, 0.0);
    nh.param("controller/angular_kd", angular_kd, 0.0);
    
    /*  配置各向量可视化  */
     // 设置 Marker
    visualization_msgs::Marker cmd_marker, modulated_marker, normal_marker;
    cmd_marker.header.frame_id = "base_link"; // 参考坐标系
    cmd_marker.header.stamp = ros::Time::now();
    cmd_marker.id = 0;
    cmd_marker.type = visualization_msgs::Marker::ARROW; // 使用箭头表示向量
    cmd_marker.action = visualization_msgs::Marker::ADD;
    
    // 设置位置和方向
    cmd_marker.pose.position.x = 0.0; // 向量起点
    cmd_marker.pose.position.y = 0.0;
    cmd_marker.pose.position.z = 0.0;

    // 设置箭头的方向
    geometry_msgs::Point p1, p2;
    p1.x = 0.0; p1.y = 0.0; p1.z = 0.0; // 起点
    p2.x = 1.0; p2.y = 1.0; p2.z = 0.0; // 终点
    cmd_marker.points.push_back(p1);
    cmd_marker.points.push_back(p2);

    // 设置颜色和大小
    cmd_marker.scale.x = 0.1; // 箭头的粗细
    cmd_marker.scale.y = 0.2; // 箭头的宽度
    cmd_marker.scale.z = 0.0; // 不适用

    cmd_marker.color.r = 1.0; // 红色
    cmd_marker.color.g = 0.0;
    cmd_marker.color.b = 0.0;
    cmd_marker.color.a = 1.0; // 不透明

    modulated_marker = cmd_marker;
    modulated_marker.color.r = 0;
    modulated_marker.color.g = 1;

    normal_marker = cmd_marker;
    normal_marker.color.r = 0;
    normal_marker.color.b = 1;
    normal_marker.points[0].x = 2;
    normal_marker.points[0].y = 2;
    /*  配置完成  */

    // 初始化控制器及其增益
    // 用从参数服务器读取的参数初始化控制器
    TwoWheel_Linear_controller controller(linear_kp, linear_ki, linear_kd,
                                          linear_kp, linear_ki, linear_kd,
                                          angular_kp, angular_ki, angular_kd);
    ros::Rate rate(50);
    rate.sleep();

    ros::Time starting_time = ros::Time::now();
    while (ros::ok())
    {
        ros::Time current_time = ros::Time::now();
        ros::Duration duration = current_time - starting_time;
        Trajectory traj = get_car_trajectory(duration.toSec());
        // 将四元数转换为欧拉角
        // 将 geometry_msgs::Quaternion 转换为 tf::Quaternion
        tf::Quaternion quaternion(odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w);
        tf::Matrix3x3 mat(quaternion);
        double roll, pitch, yaw;
        mat.getRPY(roll, pitch, yaw);
        std::vector<double> control_output = controller.get_vel_and_angularVel((traj.x - odom.pose.pose.position.x), 
                                                                                (traj.y - odom.pose.pose.position.y),
                                                                                yaw,
                                                                                odom.pose.pose.position.x,
                                                                                odom.pose.pose.position.y,
                                                                                cmd_marker, cmd_marker_pub,
                                                                                modulated_marker, modulated_marker_pub);                                       
        cmd.linear.x = limit_range(control_output[0], 0.7);
        // cmd.linear.x = 0;
        cmd.angular.z = limit_range(control_output[1], 0.8);
        cmd_pub.publish(cmd);
        normal_marker.points[1].x = odom.pose.pose.position.x;
        normal_marker.points[1].y = odom.pose.pose.position.y;
        normal_marker_pub.publish(normal_marker);
        rate.sleep();
        ros::spinOnce();
    }
}