#ifndef PID_CONTROL_H
#define PID_CONTROL_H
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose.h"


class PIDLongitudinalController
{
    private:
        float _k_p;
        float _k_i;
        float _k_d;
        float _error=0.0;
        float _error_integral=0.0;
        float error_derivative=0.0;
    public:

        void set(float k_p,float k_i,float k_d);
        float run_step(float target_vel,float current_vel);

};

class PIDLateralController
{
    private:
        float _k_p;
        float _k_i;
        float _k_d;
        float _error=0.0;
        float _error_integral=0.0;
        float error_derivative=0.0;
    public:
        void set(float k_p,float k_i,float k_d);
    float run_step(const geometry_msgs::Pose& target_pose,const geometry_msgs::Pose& current_pose);


};


class VehiclePIDController
{
    private:
        float _k_p_1=1;
        float _k_i_1=0;
        float _k_d_1=0;
        float _k_p_2=1;
        float _k_i_2=0;
        float _k_d_2=0;
        PIDLongitudinalController pid_speed;
        PIDLateralController pid_sita;
    public:
    void    set(float k_p_1,float k_i_1,float k_d_1,float k_p_2,float k_i_2,float k_d_2);
    geometry_msgs::Twist run_step(geometry_msgs::Pose target_pose,geometry_msgs::Pose current_pose,float target_vel,float current_vel);
};


#endif