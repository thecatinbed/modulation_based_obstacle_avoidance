#ifndef TWOWHEEL_LINEAR_CONTROLLER_H
#define TWOWHEEL_LINEAR_CONTROLLER_H

#include <ros/ros.h>
#include <vel_modulation/PID_controller.h>
#include <vel_modulation/modulation_avoider.h>
#include <visualization_msgs/Marker.h>
#include <vel_modulation/NeighborPoseSubscriber.h>
#include <cmath>

class TwoWheel_Linear_controller{
    public:
        TwoWheel_Linear_controller(double _kpx = 0, double _kix = 0, double _kdx = 0, 
                                    double _kpy = 0, double _kiy = 0, double _kdy = 0, 
                                    double _kptheta = 0, double _kitheta = 0, double _kdtheta = 0);
        std::vector<double>  get_vel_and_angularVel(double, double, double, double, double, visualization_msgs::Marker, ros::Publisher, visualization_msgs::Marker, ros::Publisher);
        std::vector<double>  get_vel_and_angularVel(double, double, double, double, double);
        std::vector<double>  get_vel_and_angularVel(double, double, double, double, double, std::map<std::string, NeighborPoseSubscriber>);
    private:
        PID_controller position_x_controller;
        PID_controller position_y_controller;
        PID_controller theta_controller;
        ModulationAvoider avoider;
        double desire_theta_last;
        double theta_last;
        int desire_circle_count;
        int circle_count;
};

#endif