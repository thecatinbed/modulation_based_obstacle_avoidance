#include <vel_modulation/TwoWheel_Linear_controller.h>

TwoWheel_Linear_controller::TwoWheel_Linear_controller(double _kpx, double _kix, double _kdx, 
                                                        double _kpy, double _kiy, double _kdy, 
                                                        double _kptheta, double _kitheta, double _kdtheta):
                                                        position_x_controller(_kpx, _kix, _kdx), 
                                                        position_y_controller(_kpy, _kiy, _kdy), 
                                                        theta_controller(_kptheta, _kitheta, _kdtheta)
{
    desire_theta_last = 0.0;
    theta_last = 0.0;
    circle_count = 0;
    desire_circle_count = 0;
}

std::vector<double> TwoWheel_Linear_controller::get_vel_and_angularVel(double x_error, double y_error, double theta, double x, double y, visualization_msgs::Marker cmd_marker, ros::Publisher cmd_marker_pub, visualization_msgs::Marker modulated_marker, ros::Publisher modulated_marker_pub){
    std::vector<double> output;
    // 计算位置控制量
    double desire_velocity_x = position_x_controller.calculate_control_ouput(x_error);
    double desire_velocity_y = position_x_controller.calculate_control_ouput(y_error);
    // desire_velocity_x = 1;
    // desire_velocity_y = -1;
    Eigen::Vector2f agent_position(x, y);
    Eigen::Vector2f cmd_velocity(desire_velocity_x, desire_velocity_y);
    // 修正速度
    Eigen::Vector2f modulated_velocity = avoider.modulate_velocity(agent_position, cmd_velocity);
    // 计算期望姿态
    double desire_theta = atan2(modulated_velocity(1), modulated_velocity(0));
    // double desire_theta = atan2(cmd_velocity(1), cmd_velocity(0));
    // 可视化期望速度方向以及修正后的速度方向
    cmd_marker.points[0].x = agent_position(0);
    cmd_marker.points[0].y = agent_position(1);
    cmd_marker.points[1].x = desire_velocity_x / cmd_velocity.norm() + cmd_marker.points[0].x;
    cmd_marker.points[1].y = desire_velocity_y / cmd_velocity.norm() + cmd_marker.points[0].y;
    cmd_marker_pub.publish(cmd_marker);
    modulated_marker.points[0].x = agent_position(0);
    modulated_marker.points[0].y = agent_position(1);
    modulated_marker.points[1].x = 1 * cos(desire_theta) + modulated_marker.points[0].x;
    modulated_marker.points[1].y = 1 * sin(desire_theta) + modulated_marker.points[0].y;
    modulated_marker_pub.publish(modulated_marker);
    if (desire_theta < 0)
    {
        desire_theta += 2 * M_PI;
    }
    if (theta < 0)
    {
        theta += 2 * M_PI;
    }
    double error_theta;
    if (abs(desire_theta - theta) > M_PI)
    {
        if (desire_theta > theta)
        {
            error_theta  = (desire_theta - theta) - 2 * M_PI;
        }else{
            error_theta = (desire_theta - theta) + 2 * M_PI;
        }
    }else{
        error_theta = desire_theta - theta;
    }
    // 计算期望姿态控制量
    double desire_angular_valocity = theta_controller.calculate_control_ouput(error_theta);
    output.push_back(sqrt(pow(modulated_velocity(0), 2) + pow(modulated_velocity(1), 2)));
    output.push_back(desire_angular_valocity);
    return output;
}

std::vector<double> TwoWheel_Linear_controller::get_vel_and_angularVel(double x_error, double y_error, double theta, double x, double y){
    std::vector<double> output;
    // 计算位置控制量
    double desire_velocity_x = position_x_controller.calculate_control_ouput(x_error);
    double desire_velocity_y = position_x_controller.calculate_control_ouput(y_error);
    // desire_velocity_x = 1;
    // desire_velocity_y = -1;
    Eigen::Vector2f agent_position(x, y);
    Eigen::Vector2f cmd_velocity(desire_velocity_x, desire_velocity_y);
    // 修正速度
    Eigen::Vector2f modulated_velocity = avoider.modulate_velocity(agent_position, cmd_velocity);
    double modulated_velocity_norm = modulated_velocity.norm();
    // 计算期望姿态
    double desire_theta = atan2(modulated_velocity(1), modulated_velocity(0));
    // desire_theta = 1;
    // double desire_theta = atan2(cmd_velocity(1), cmd_velocity(0));
    if (desire_theta < 0){
        desire_theta += 2 * M_PI;
    }
    if (theta < 0){
        theta += 2 * M_PI;
    }
    double error_theta;
    if (abs(desire_theta - theta) > M_PI){
        if (desire_theta > theta){
            error_theta  = (desire_theta - theta) - 2 * M_PI;
        }else{
            error_theta = (desire_theta - theta) + 2 * M_PI;
        }
    }else{
        error_theta = desire_theta - theta;
    }
    if (abs(error_theta) > M_PI / 2){
        modulated_velocity_norm = -modulated_velocity_norm;
        if (error_theta > 0){
            error_theta = M_PI / 2 - error_theta;
        }else{
            error_theta = - error_theta - M_PI / 2;
        }
    }
    
    // 计算期望姿态控制量
    double desire_angular_valocity = theta_controller.calculate_control_ouput(error_theta);
    output.push_back(modulated_velocity_norm);
    output.push_back(desire_angular_valocity);
    return output;
}

std::vector<double>  TwoWheel_Linear_controller::get_vel_and_angularVel(double x_error, double y_error, double theta, double x, double y, std::map<std::string, NeighborPoseSubscriber> neighbor_pose_subscribers){
    std::vector<double> output;
    // 计算位置控制量
    double desire_velocity_x = position_x_controller.calculate_control_ouput(x_error);
    double desire_velocity_y = position_x_controller.calculate_control_ouput(y_error);
    for (auto &neighbor_pose_sub:neighbor_pose_subscribers)
    {
        desire_velocity_x += 0.1 * (x_error - (neighbor_pose_sub.second.desire_x - neighbor_pose_sub.second.actual_x));
        desire_velocity_y += 0.1 * (y_error - (neighbor_pose_sub.second.desire_y - neighbor_pose_sub.second.actual_y));
    }
    
    // desire_velocity_x = 1;
    // desire_velocity_y = -1;
    Eigen::Vector2f agent_position(x, y);
    Eigen::Vector2f cmd_velocity(desire_velocity_x, desire_velocity_y);
    // 修正速度
    // Eigen::Vector2f modulated_velocity = avoider.modulate_velocity(agent_position, cmd_velocity);
    Eigen::Vector2f modulated_velocity = avoider.modulate_velocity(agent_position, cmd_velocity, neighbor_pose_subscribers);
    double modulated_velocity_norm = modulated_velocity.norm();
    // 计算期望姿态
    double desire_theta = atan2(modulated_velocity(1), modulated_velocity(0));
    // desire_theta = 1;
    // double desire_theta = atan2(cmd_velocity(1), cmd_velocity(0));
    if (desire_theta < 0){
        desire_theta += 2 * M_PI;
    }
    if (theta < 0){
        theta += 2 * M_PI;
    }
    double error_theta;
    if (abs(desire_theta - theta) > M_PI){
        if (desire_theta > theta){
            error_theta  = (desire_theta - theta) - 2 * M_PI;
        }else{
            error_theta = (desire_theta - theta) + 2 * M_PI;
        }
    }else{
        error_theta = desire_theta - theta;
    }
    if (abs(error_theta) > M_PI / 2){
        modulated_velocity_norm = -modulated_velocity_norm;
        if (error_theta > 0){
            error_theta = M_PI / 2 - error_theta;
        }else{
            error_theta = - error_theta - M_PI / 2;
        }
    }
    
    // 计算期望姿态控制量，保存并返回结果
    double desire_angular_valocity = theta_controller.calculate_control_ouput(error_theta);
    output.push_back(modulated_velocity_norm);
    output.push_back(desire_angular_valocity);
    return output;
}

std::vector<double>  TwoWheel_Linear_controller::get_vel_and_angularVel_APF(double desire_x, double desire_y, double theta, double actual_x, double actual_y){
    std::vector<double> output;
    const double K_att = 1.5, K_rep = 0.25, R_obs = 0.5;
    // 计算位置控制量
    Eigen::Vector2f force_att, force_rep, force_all;
    force_rep << 0, 0;
    double att_distance = sqrt(pow(desire_x - actual_x, 2) + pow(desire_y - actual_y, 2));
    if (att_distance > 1){
        force_att << K_att * 1 / att_distance * (desire_x - actual_x),
                 K_att * 1 / att_distance * (desire_y - actual_y);
    }else{
        force_att << K_att * (desire_x - actual_x),
                 K_att * (desire_y - actual_y);
    }
    ROS_INFO("attraction is: %f, %f", force_att(0), force_att(1));
    for (auto &obstacle:avoider.obstacle_info){
        double rep_distance = sqrt(pow(actual_x - obstacle(0), 2) + pow(actual_y - obstacle(1), 2));
        Eigen::Vector2f temp_rep_force;
        if (rep_distance <= R_obs){
            temp_rep_force << K_rep * (1 / R_obs - 1 / rep_distance) * (pow(1 / rep_distance, 2)) * (obstacle(0) - actual_x) / rep_distance,
                            K_rep * (1 / R_obs - 1 / rep_distance) * (pow(1 / rep_distance, 2)) * (obstacle(1) - actual_y) / rep_distance;
            force_rep = force_rep + temp_rep_force;
        }
    }
    force_all = force_att + force_rep;
    double velocity_norm = force_all.norm();
    // 计算期望姿态
    double desire_theta = atan2(force_all(1), force_all(0));
    // desire_theta = 1;
    // double desire_theta = atan2(cmd_velocity(1), cmd_velocity(0));
    if (desire_theta < 0){
        desire_theta += 2 * M_PI;
    }
    if (theta < 0){
        theta += 2 * M_PI;
    }
    double error_theta;
    if (abs(desire_theta - theta) > M_PI){
        if (desire_theta > theta){
            error_theta  = (desire_theta - theta) - 2 * M_PI;
        }else{
            error_theta = (desire_theta - theta) + 2 * M_PI;
        }
    }else{
        error_theta = desire_theta - theta;
    }
    if (abs(error_theta) > M_PI / 2){
        velocity_norm = -velocity_norm;
        if (error_theta > 0){
            error_theta = M_PI / 2 - error_theta;
        }else{
            error_theta = - error_theta - M_PI / 2;
        }
    }
    
    // 计算期望姿态控制量
    double desire_angular_valocity = theta_controller.calculate_control_ouput(error_theta);
    output.push_back(velocity_norm);
    output.push_back(desire_angular_valocity);
    return output;
}

std::vector<double>  TwoWheel_Linear_controller::get_vel_and_angularVel_APF(double desire_x, double desire_y, double theta, double actual_x, double actual_y, std::map<std::string, NeighborPoseSubscriber> neighbor_pose_subscribers){
    std::vector<double> output;
    const double K_att = 1.5, K_rep = 0.2, R_obs = 0.5, R_neighbor = 0.4;
    // 计算位置控制量
    Eigen::Vector2f force_att, force_rep, force_all;
    force_rep << 0, 0;
    double att_distance = sqrt(pow(desire_x - actual_x, 2) + pow(desire_y - actual_y, 2));
    if (att_distance > 1){
        force_att << K_att * 1 / att_distance * (desire_x - actual_x),
                 K_att * 1 / att_distance * (desire_y - actual_y);
    }else{
        force_att << K_att * (desire_x - actual_x),
                 K_att * (desire_y - actual_y);
    }
    for (auto &neighbor_pose_sub:neighbor_pose_subscribers)
    {
        force_att(0) += 2.0 * ((desire_x - actual_x) - (neighbor_pose_sub.second.desire_x - neighbor_pose_sub.second.actual_x));
        force_att(1) += 2.0 * ((desire_y - actual_y) - (neighbor_pose_sub.second.desire_y - neighbor_pose_sub.second.actual_y));
        double rep_distance = sqrt(pow(actual_x - neighbor_pose_sub.second.actual_x, 2) + pow(actual_y - neighbor_pose_sub.second.actual_y, 2));
        Eigen::Vector2f temp_rep_force;
        if (rep_distance <= R_neighbor){
            temp_rep_force << K_rep * (1 / R_neighbor - 1 / rep_distance) * (pow(1 / rep_distance, 2)) * (neighbor_pose_sub.second.actual_x - actual_x) / rep_distance,
                            K_rep * (1 / R_neighbor - 1 / rep_distance) * (pow(1 / rep_distance, 2)) * (neighbor_pose_sub.second.actual_y - actual_y) / rep_distance;
            force_rep = force_rep + temp_rep_force;
        }
    }
    // ROS_INFO("attraction is: %f, %f", force_att(0), force_att(1));
    for (auto &obstacle:avoider.obstacle_info){
        double rep_distance = sqrt(pow(actual_x - obstacle(0), 2) + pow(actual_y - obstacle(1), 2));
        Eigen::Vector2f temp_rep_force;
        if (rep_distance <= R_obs){
            temp_rep_force << K_rep * (1 / R_obs - 1 / rep_distance) * (pow(1 / rep_distance, 2)) * (obstacle(0) - actual_x) / rep_distance,
                            K_rep * (1 / R_obs - 1 / rep_distance) * (pow(1 / rep_distance, 2)) * (obstacle(1) - actual_y) / rep_distance;
            force_rep = force_rep + temp_rep_force;
        }
    }
    force_all = force_att + force_rep;
    double velocity_norm = force_all.norm();
    // 计算期望姿态
    double desire_theta = atan2(force_all(1), force_all(0));
    // desire_theta = 1;
    // double desire_theta = atan2(cmd_velocity(1), cmd_velocity(0));
    if (desire_theta < 0){
        desire_theta += 2 * M_PI;
    }
    if (theta < 0){
        theta += 2 * M_PI;
    }
    double error_theta;
    if (abs(desire_theta - theta) > M_PI){
        if (desire_theta > theta){
            error_theta  = (desire_theta - theta) - 2 * M_PI;
        }else{
            error_theta = (desire_theta - theta) + 2 * M_PI;
        }
    }else{
        error_theta = desire_theta - theta;
    }
    if (abs(error_theta) > M_PI / 2){
        velocity_norm = -velocity_norm;
        if (error_theta > 0){
            error_theta = M_PI / 2 - error_theta;
        }else{
            error_theta = - error_theta - M_PI / 2;
        }
    }
    
    // 计算期望姿态控制量
    double desire_angular_valocity = theta_controller.calculate_control_ouput(error_theta);
    output.push_back(velocity_norm);
    output.push_back(desire_angular_valocity);
    return output;
}