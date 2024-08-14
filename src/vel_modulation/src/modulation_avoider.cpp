#include <vel_modulation/modulation_avoider.h>

ModulationAvoider::ModulationAvoider(){
    // Eigen::Vector3f obs_1(1, -0.95, 0.35);
    Eigen::Vector3f obs_1(1.9, 2, 0.2);
    Eigen::Vector3f obs_2(-2.7, 2, 0.2);
    Eigen::Vector3f obs_3(-1.9, 2, 0.2);
    Eigen::Vector3f obs_4(2.7, 2, 0.2);
    obstacle_info.push_back(obs_1);
    obstacle_info.push_back(obs_2);
    obstacle_info.push_back(obs_3);
    obstacle_info.push_back(obs_4);
}

double ModulationAvoider::get_Gamma(Eigen::Vector2f agent_position, Eigen::Vector2f obstacle_position, double radius){
    double relative_distance = (agent_position - obstacle_position).norm();
    double Gamma = (relative_distance / radius) * (relative_distance / radius);
    return Gamma;
}

Eigen::Vector2f ModulationAvoider::get_reference_vector(Eigen::Vector2f agent_position, Eigen::Vector2f obstacle_position){
    Eigen::Vector2f temp = obstacle_position - agent_position;
    return temp / temp.norm();
}

Eigen::Vector2f ModulationAvoider::get_normal_vector(Eigen::Vector2f agent_position, Eigen::Vector2f obstacle_position){
    Eigen::Vector2f temp = agent_position - obstacle_position;
    return temp / temp.norm();
}

Eigen::Vector2f ModulationAvoider::get_tangent_vector(Eigen::Vector2f normal_vector){
    Eigen::Matrix2f rotation_matrix;
    rotation_matrix << 0, -1,
                        1, 0;
    return rotation_matrix * normal_vector;
}

Eigen::Vector2f ModulationAvoider::modulate_velocity(Eigen::Vector2f agent_position, Eigen::Vector2f cmd_velocity){
    Eigen::MatrixXf modulated_velocity_matrix = Eigen::MatrixXf::Zero(obstacle_info.size(), 2);
    Eigen::VectorXf weights = Eigen::VectorXf::Zero(obstacle_info.size());
    Eigen::Vector2f modulated_velocity = Eigen::Vector2f::Zero();
    for (int i = 0; i < obstacle_info.size(); i++)
    {
        Eigen::Vector3f obstacle = obstacle_info[i];
        double obstacle_radius = obstacle(2);
        Eigen::Vector2f obstacle_position(obstacle(0), obstacle(1));

        if ((agent_position - obstacle_position).norm() > 0.5)
        {
            continue;   // 障碍物在感知范围外
        }

        Eigen::Vector2f normal_vector = get_normal_vector(agent_position, obstacle_position);
        Eigen::Vector2f tangent_vector = get_tangent_vector(normal_vector);
        double Gamma = get_Gamma(agent_position, obstacle_position, obstacle_radius);
        weights(i) = pow((1 / Gamma), 2);
        double rho = 7;
        Eigen::Vector2f eigen_value(1 - pow(1 / Gamma, 1 / rho), 1 + pow( 1 / Gamma, 1 / rho));
        if (eigen_value(0) < 0)
        {
            eigen_value(0) = 0;
        }
        
        Eigen::Matrix2f eigen_value_matrix = eigen_value.asDiagonal();
        Eigen::Matrix2f eigen_vector_matrix;
        eigen_vector_matrix.col(0) = normal_vector;
        eigen_vector_matrix.col(1) = tangent_vector;
        Eigen::Matrix2f eigen_vector_matrix_inv = eigen_vector_matrix.inverse();
        Eigen::Vector2f v1 = eigen_vector_matrix_inv * cmd_velocity;
        if (v1(0) > 0)
        {
            eigen_value_matrix(0, 0) = 1;
        }
        Eigen::Vector2f v2 = eigen_value_matrix * v1;
        
        if (v2.norm() > 0.2)
        {
            v2 = 0.2 * v2 / v2.norm();
        }

        modulated_velocity = eigen_vector_matrix * v2;
        double xi = 1;
        // if(Gamma >= 1){
        //     xi = 1 - 1 / Gamma + 0.1;
        // }else if (Gamma < 0.8)
        // {
        //     xi = 0.1;
        // }
        // if (xi > 1)
        // {
        //     xi = 1;
        // }
        // xi = pow(xi, 0.8);
        
        
        // modulated_velocity =  xi * 0.2 * modulated_velocity / modulated_velocity.norm();
        modulated_velocity_matrix.row(i) = modulated_velocity;
        // ROS_INFO("Gamma:%f, obstacle_radius:%f", Gamma, obstacle_radius);
        // std::cout<< "normal vector:\n" << normal_vector << std::endl;
        // std::cout<< "tangent vector:\n" << tangent_vector << std::endl;
        // std::cout<< "eigen vector matrix:\n" << eigen_vector_matrix << std::endl;
        // std::cout<< "eigen value matrix:\n" << eigen_value_matrix << std::endl;
        // std::cout<< "modulated velocity :\n" << modulated_velocity.norm() << std::endl;
        // if (modulated_velocity.norm() >= 0.4)
        // {
        //     modulated_velocity =  0.4 * modulated_velocity / modulated_velocity.norm();
        // }
    }
    if (weights.isZero())
    {
        return cmd_velocity;   // 没有障碍物处于感知范围内
    }else{
        weights = weights / weights.sum();
        modulated_velocity = modulated_velocity_matrix.transpose() * weights;
    }
     
    return modulated_velocity;
}

Eigen::Vector2f ModulationAvoider::modulate_velocity(Eigen::Vector2f agent_position, Eigen::Vector2f cmd_velocity, std::map<std::string, NeighborPoseSubscriber> &neighbor_pose_subscribers){
    double rho = 20;
    Eigen::MatrixXf modulated_velocity_matrix = Eigen::MatrixXf::Zero(obstacle_info.size() + neighbor_pose_subscribers.size(), 2);
    Eigen::VectorXf weights = Eigen::VectorXf::Zero(obstacle_info.size() + neighbor_pose_subscribers.size());
    Eigen::Vector2f modulated_velocity = Eigen::Vector2f::Zero();
    for (int i = 0; i < obstacle_info.size(); i++)
    {
        Eigen::Vector3f obstacle = obstacle_info[i];
        double obstacle_radius = obstacle(2);
        Eigen::Vector2f obstacle_position(obstacle(0), obstacle(1));

        if ((agent_position - obstacle_position).norm() > 0.5)
        {
            continue;   // 障碍物在感知范围外
        }
        Eigen::Vector2f normal_vector = get_normal_vector(agent_position, obstacle_position);
        Eigen::Vector2f tangent_vector = get_tangent_vector(normal_vector);
        double Gamma = get_Gamma(agent_position, obstacle_position, obstacle_radius);
        if (Gamma <= 1)
        {
            weights(i) = 1e30;
        }else{
            weights(i) = pow((1 / (Gamma - 1)), 1);
        }
        
        Eigen::Vector2f eigen_value(1 - pow(2 / Gamma, 1 / rho), 1 + pow( 1 / Gamma, 1 / rho));
        if (eigen_value(0) < 0)
        {
            eigen_value(0) = 0;
        }
        
        Eigen::Matrix2f eigen_value_matrix = eigen_value.asDiagonal();
        Eigen::Matrix2f eigen_vector_matrix;
        eigen_vector_matrix.col(0) = normal_vector;
        eigen_vector_matrix.col(1) = tangent_vector;
        Eigen::Matrix2f eigen_vector_matrix_inv = eigen_vector_matrix.inverse();
        Eigen::Vector2f v1 = eigen_vector_matrix_inv * cmd_velocity;
        if (v1(0) > 0)
        {
            eigen_value_matrix(0, 0) = 1;
        }
        Eigen::Vector2f v2 = eigen_value_matrix * v1;
        
        if (v2.norm() > 0.2)
        {
            v2 = 0.2 * v2 / v2.norm();
        }

        modulated_velocity = eigen_vector_matrix * v2;
        double xi = 1;
        // if(Gamma >= 1){
        //     xi = 1 - 1 / Gamma + 0.1;
        // }else if (Gamma < 0.8)
        // {
        //     xi = 0.1;
        // }
        // if (xi > 1)
        // {
        //     xi = 1;
        // }
        // xi = pow(xi, 0.8);
        
        
        // modulated_velocity =  xi * 0.2 * modulated_velocity / modulated_velocity.norm();
        modulated_velocity_matrix.row(i) = modulated_velocity;
        // ROS_INFO("Gamma:%f, obstacle_radius:%f", Gamma, obstacle_radius);
        // std::cout<< "normal vector:\n" << normal_vector << std::endl;
        // std::cout<< "tangent vector:\n" << tangent_vector << std::endl;
        // std::cout<< "eigen vector matrix:\n" << eigen_vector_matrix << std::endl;
        // std::cout<< "eigen value matrix:\n" << eigen_value_matrix << std::endl;
        // std::cout<< "modulated velocity :\n" << modulated_velocity.norm() << std::endl;
        // if (modulated_velocity.norm() >= 0.4)
        // {
        //     modulated_velocity =  0.4 * modulated_velocity / modulated_velocity.norm();
        // }
    }
    rho = 20;
    int i = 0;
    for(auto &neighbor_pose_sub:neighbor_pose_subscribers){
        int index = i + obstacle_info.size();
        double obstacle_radius = 0.2;
        // Eigen::Vector2f obstacle_position(neighbor_pose_sub.second.getPose().position.x, neighbor_pose_sub.second.getPose().position.y);
        Eigen::Vector2f obstacle_position(neighbor_pose_sub.second.actual_x, neighbor_pose_sub.second.actual_y);
        // ROS_INFO("get neighbor position:%f, %f", obstacle_position(0), obstacle_position(1));
        if ((agent_position - obstacle_position).norm() > 0.4)
        {
            continue;   // 障碍物在感知范围外
        }
        Eigen::Vector2f normal_vector = get_normal_vector(agent_position, obstacle_position);
        Eigen::Vector2f tangent_vector = get_tangent_vector(normal_vector);
        double Gamma = get_Gamma(agent_position, obstacle_position, obstacle_radius);
        if (Gamma <= 1)
        {
            weights(index) = 1e30;
        }else{
            weights(index) = pow((1 / (Gamma - 1)), 4);
        }
        
        
        Eigen::Vector2f eigen_value(1 - pow(2 / Gamma, 1 / rho), 1 + pow( 1 / Gamma, 1 / rho));
        if (eigen_value(0) < 0)
        {
            eigen_value(0) = 0;
        }
        
        Eigen::Matrix2f eigen_value_matrix = eigen_value.asDiagonal();
        Eigen::Matrix2f eigen_vector_matrix;
        eigen_vector_matrix.col(0) = normal_vector;
        eigen_vector_matrix.col(1) = tangent_vector;
        Eigen::Matrix2f eigen_vector_matrix_inv = eigen_vector_matrix.inverse();
        Eigen::Vector2f v1 = eigen_vector_matrix_inv * cmd_velocity;
        // if (v1(0) > 0)
        // {
        //     eigen_value_matrix(0, 0) = 1;
        // }
        Eigen::Vector2f v2 = eigen_value_matrix * v1;
        
        if (v2.norm() > 0.2)
        {
            v2 = 0.2 * v2 / v2.norm();
        }

        modulated_velocity = eigen_vector_matrix * v2;
        // double xi = 1;
        // if(Gamma >= 1){
        //     xi = 1 - 1 / Gamma + 0.1;
        // }else if (Gamma < 0.8)
        // {
        //     xi = 0.1;
        // }
        // if (xi > 1)
        // {
        //     xi = 1;
        // }
        // xi = pow(xi, 0.8);
        
        
        // modulated_velocity =  xi * 0.2 * modulated_velocity / modulated_velocity.norm();
        modulated_velocity_matrix.row(index) = modulated_velocity;
        // ROS_INFO("Gamma:%f, obstacle_radius:%f", Gamma, obstacle_radius);
        i ++;
    }
    if (weights.isZero())
    {
        return cmd_velocity;   // 没有障碍物处于感知范围内
    }else{
        weights = weights / weights.sum();
        modulated_velocity = modulated_velocity_matrix.transpose() * weights;
    }
     
    return modulated_velocity;
}