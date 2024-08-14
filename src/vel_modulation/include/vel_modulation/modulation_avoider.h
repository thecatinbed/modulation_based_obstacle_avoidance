#ifndef MODULATION_AVOIDER_H
#define MODULATION_AVOIDER_H
#include <ros/ros.h>
#include <Eigen/Eigen>
#include <vel_modulation/NeighborPoseSubscriber.h>
class ModulationAvoider{
    public:
        ModulationAvoider();
        Eigen::Vector2f modulate_velocity(Eigen::Vector2f agent_position, Eigen::Vector2f cmd_velocity);
        Eigen::Vector2f modulate_velocity(Eigen::Vector2f agent_position, Eigen::Vector2f cmd_velocity, std::map<std::string, NeighborPoseSubscriber>&);
    private:
        std::vector<Eigen::Vector3f> obstacle_info;
        double get_Gamma(Eigen::Vector2f, Eigen::Vector2f, double);
        Eigen::Vector2f get_reference_vector(Eigen::Vector2f, Eigen::Vector2f);
        Eigen::Vector2f get_normal_vector(Eigen::Vector2f, Eigen::Vector2f);
        Eigen::Vector2f get_tangent_vector(Eigen::Vector2f);
};
#endif