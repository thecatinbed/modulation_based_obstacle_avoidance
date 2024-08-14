#include <vel_modulation/PID_controller.h>

PID_controller::PID_controller(double _kp, double _ki, double _kd, bool _debug_check):kp(_kp), ki(_ki), kd(_kd), debug_check(_debug_check)
{
    error_now = 0.0;
    error_last = 0.0;
    error_last_last = 0.0;
    output_last = 0.0;
    error_sum = 0.0;
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

double PID_controller::calculate_control_ouput(double error){
    error_now = error;
    if (isnan(error))
    {
        ROS_WARN("error is NAN");
    }else{
        if (debug_check)
        {
            ROS_INFO("error sum: %f, error: %f", error_sum, error);
        }
        error_sum += error;
    }
    error_sum = limit_range(error_sum, 5);
    double output = kp * (error_now ) + ki * (error_sum) + kd  * (error_now - error_last);
    error_last = error_now;
    error_last_last = error_last;
    output_last = output;
    return output;
}

