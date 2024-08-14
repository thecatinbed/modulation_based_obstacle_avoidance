#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <ros/ros.h>

class PID_controller{
    public:
        PID_controller(double _kp, double _ki=0.0, double _kd=0.0, bool _debug_check = false);
        double calculate_control_ouput(double);
    
    private:
        double error_now;
        double error_last;
        double error_last_last;
        double error_sum;
        double output_last;
        double kp;
        double ki;
        double kd;
        double debug_check;
        
};

#endif