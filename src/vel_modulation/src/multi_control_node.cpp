#include <ros/ros.h>
#include <vel_modulation/TwoWheel_Linear_controller.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <tf/tf.h>
#include <vel_modulation/NeighborPoseSubscriber.h>

nav_msgs::Odometry odom;        // gazebo仿真时的机器人位姿
geometry_msgs::PoseStamped pose_; // 动捕环境下的机器人位姿
geometry_msgs::Twist cmd;
geometry_msgs::Point traj_;

struct Trajectory {
    double x;
    double y;
};

Trajectory get_car_trajectory(double t, char robot_num){
    Trajectory traj;
    // traj.x = 1 * (1 + cos(0.2 * t - M_PI));
    // traj.y = 1 * (0 + sin(0.2 * t - M_PI));
    
    traj.x = 2 * (0 + cos(0.1 * t - M_PI / 2));
    traj.y = 2 * (1 + sin(0.1 * t - M_PI / 2));
    // if (t <= 30)
    // {
    //     traj.x = -4;
    //     traj.y = 2.1;
    // }else{
    //     traj.x = 5.0;
    //     traj.y = 2;
    // }
    if (robot_num == '0')
    {
        traj.x += 0.5;
        traj.y += 0.5;
    }else if (robot_num == '1')
    {
        traj.x += 0;
        traj.y += 1.0;
    }else{
        // traj.x += 0;
        // traj.y += 0.8;
    }
    return traj;
}

void TwoWheel_odom_callback(const nav_msgs::Odometry::ConstPtr &pmsg){
    odom = *pmsg;
}

void car0poseCallback(const geometry_msgs::PoseStamped::ConstPtr &posemsg)
{
    pose_ = *posemsg;
}

void Trajectory_cmd_callback(const geometry_msgs::PointConstPtr &pmsg){
    traj_ = *pmsg;
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
    // 定义控制变量
    tf::Quaternion quaternion;
    double actual_x = 0;
    double actual_y = 0;
    double roll, pitch, yaw;
    // 初始化节点句柄
    ros::NodeHandle nh;
    // 定义参数变量
    double linear_kp, linear_ki, linear_kd;
    double angular_kp, angular_ki, angular_kd;
    bool use_simulation;
    // 从参数服务器读取参数
    nh.param("/controller/linear_kp", linear_kp, 1.1);
    nh.param("/controller/linear_ki", linear_ki, 0.0);
    nh.param("/controller/linear_kd", linear_kd, 0.0);
    nh.param("/controller/angular_kp", angular_kp, 8.0);
    nh.param("/controller/angular_ki", angular_ki, 0.0);
    nh.param("/controller/angular_kd", angular_kd, 0.5);
    nh.param("/use_simulation", use_simulation, true);
    // 初始化订阅者以及发布者
    // 获取机器人名称列表，可以从参数服务器获取或直接在代码中定义
    std::vector<std::string> robot_names;
    // 获取自身信息，订阅邻居位姿
    std::map<std::string, NeighborPoseSubscriber> neighbor_pose_subscribers;
    std::string robot_ns = nh.getNamespace();
    char robot_number = robot_ns.back();
    if (!std::isdigit(robot_number))
    {
        robot_number = '0';
    }
    ROS_INFO("The number of this robot is %c", robot_number);
    switch (robot_number)
    {
    case '0':
        robot_names = {"robot_1", "robot_2"};
        break;
    
    case '1':
        robot_names = {"robot_0", "robot_2"};
        break;

    case '2':
        robot_names = {"robot_0", "robot_1"};
        break;

    default:
        break;
    }
    std::vector<ros::Subscriber> neighbor_odom_sub(robot_names.size());
    int index = 0;
    for(auto &robot_name:robot_names)
    {
        // neighbor_pose_subscribers[robot_name] = NeighborPoseSubscriber(nh, robot_name);
        // NeighborPoseSubscriber neighbor_sub = NeighborPoseSubscriber();
        neighbor_pose_subscribers[robot_name] = NeighborPoseSubscriber();
        std::string topic_name = "/" + robot_name + "/odom";
        neighbor_odom_sub[index] = nh.subscribe(topic_name, 10, &NeighborPoseSubscriber::odomCallback, &neighbor_pose_subscribers[robot_name] );
        index ++;
    }
    // 订阅自身位姿和期望轨迹信息
    ros::Subscriber odom_sub = nh.subscribe("odom", 1, TwoWheel_odom_callback);            // 仿真使用
    ros::Subscriber traj_sub = nh.subscribe("trajectory_cmd", 1, Trajectory_cmd_callback); // 暂时无用
    ros::Subscriber car_pose_sub = nh.subscribe("/vrpn_client_node/Tracker2/pose", 1, car0poseCallback); // 实机使用
    // 控制命令发布者
    ros::Publisher cmd_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);

    // 初始化控制器及其增益
    // 用从参数服务器读取的参数初始化控制器
    TwoWheel_Linear_controller controller(linear_kp, linear_ki, linear_kd,
                                          linear_kp, linear_ki, linear_kd,
                                          angular_kp, angular_ki, angular_kd);
    ros::Rate rate(50); // 控制频率
    ros::Duration(1).sleep();   // 延时，等待通信连接完成
    ros::spinOnce();

    ros::Time starting_time = ros::Time::now();
    while (ros::ok())
    {
        ros::Time current_time = ros::Time::now();
        ros::Duration duration = current_time - starting_time;
        Trajectory traj = get_car_trajectory(duration.toSec(), robot_number);
        for (auto &neighbor_pose_sub:neighbor_pose_subscribers)
        {
            Trajectory neighbor_traj = get_car_trajectory(duration.toSec(), neighbor_pose_sub.first.back());
            neighbor_pose_sub.second.desire_x = neighbor_traj.x;
            neighbor_pose_sub.second.desire_y = neighbor_traj.y;
        }
        
        // 将四元数转换为欧拉角
        // 将 geometry_msgs::Quaternion 转换为 tf::Quaternion
        if (use_simulation)
        {
            /* gazebo 仿真 */
            quaternion.setValue(odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w);
            actual_x = odom.pose.pose.position.x;
            actual_y = odom.pose.pose.position.y;

        }
        else
        {
            /* 动捕平台 */
            quaternion.setValue(pose_.pose.orientation.x, pose_.pose.orientation.y, pose_.pose.orientation.z, pose_.pose.orientation.w);
            actual_x = pose_.pose.position.x;
            actual_y = pose_.pose.position.y;
        }
        // ROS_INFO("x:%f, y:%f", actual_x, actual_y);
        tf::Matrix3x3 mat(quaternion);
        mat.getRPY(roll, pitch, yaw);
        std::vector<double> control_output = controller.get_vel_and_angularVel((traj.x - actual_x), 
                                                                               (traj.y - actual_y),
                                                                                yaw,
                                                                                actual_x,
                                                                                actual_y,
                                                                                neighbor_pose_subscribers);
        // for (auto &neighbor_sub:neighbor_pose_subscribers)
        // {
        //     std::cout << neighbor_sub.second.x << neighbor_sub.second.y << std::endl;
        // }
                                               
        cmd.linear.x = limit_range(control_output[0], 0.3);
        // cmd.linear.x = 0.0;
        cmd.angular.z = limit_range(control_output[1], 0.8);
        // cmd.angular.z = 0;
        cmd_pub.publish(cmd);
        rate.sleep();
        ros::spinOnce();
    }
}