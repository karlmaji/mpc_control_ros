#include "pid_control.h"
#include "tf/tf.h"
#include "eigen3/Eigen/Dense"
void VehiclePIDController::set(float k_p_1,float k_i_1,float k_d_1,float k_p_2,float k_i_2,float k_d_2)
{
    this->_k_p_1=k_p_1;
    this->_k_p_2=k_p_2;
    this->_k_i_1=k_i_1;
    this->_k_d_1=k_d_1;
    this->_k_i_2=k_i_2;
    this->_k_d_2=k_d_2;
    this->pid_speed.set(k_p_1,k_i_1,k_d_1);
    this->pid_sita.set(k_p_2,k_i_2,k_d_2);
}
void PIDLongitudinalController::set(float k_p,float k_i,float k_d)
{
    this->_k_p = k_p;
    this->_k_i = k_i;
    this->_k_d = k_d;
}

void PIDLateralController::set(float k_p,float k_i,float k_d)
{
    this->_k_p = k_p;
    this->_k_i = k_i;
    this->_k_d = k_d;
}

float PIDLongitudinalController::run_step(float target_vel,float current_vel)
{
    float previous_error = this->_error;
    this->_error = target_vel - current_vel;
    this->_error_integral = this->_error_integral+this->_error;
    if(this->_error_integral >40) this->_error_integral =40;
    if(this->_error_integral <-40) this->_error_integral =-40;
    
    this->error_derivative = this->_error - previous_error;
    auto output = this->_k_p * this->_error + this->_k_i * this->_error_integral + this->_k_d * this->error_derivative;
    return output;

}

float PIDLateralController::run_step(const geometry_msgs::Pose& target_pose,const geometry_msgs::Pose& current_pose)
{   double yaw;

    yaw =tf::getYaw(current_pose.orientation);
    
    

    geometry_msgs::Point v_begin,v_end;
    v_begin.x = current_pose.position.x ; 
    v_begin.y = current_pose.position.y;
    v_end.x = v_begin.x  + cos(yaw);
    v_end.y = v_begin.y  + sin(yaw);

    Eigen::Vector3d v_vec(v_end.x - v_begin.x,v_end.y - v_begin.y,0);
    Eigen::Vector3d w_vec(target_pose.position.x - v_begin.x,target_pose.position.y - v_begin.y,0);
    auto cos_sita =v_vec.dot(w_vec)/v_vec.norm()/w_vec.norm();
    if(cos_sita>1) cos_sita=1.0;
    if(cos_sita<-1) cos_sita=-1.0;

    

    auto _dot = acos(cos_sita);


    auto _cross = v_vec.cross(w_vec);
    if(_cross[2]<0) _dot*=-1;

    auto previous_error =this->_error;
    this->_error = _dot;

    this->_error_integral = this->_error_integral+this->_error;
    if(this->_error_integral >40) this->_error_integral =400;
    if(this->_error_integral <-40) this->_error_integral =-400;
    
    this->error_derivative = this->_error - previous_error;
    auto output = this->_k_p * this->_error + this->_k_i * this->_error_integral + this->_k_d * this->error_derivative;
    if(output>1) output=1.0;
    if(output<-1)output=-1.0;
    return output;
    
}

geometry_msgs::Twist VehiclePIDController::run_step(geometry_msgs::Pose target_pose,geometry_msgs::Pose current_pose,float target_vel,float current_vel)
{
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = this->pid_speed.run_step(target_vel,current_vel);
    std::cout<< "x: "<<cmd_vel.linear.x <<std::endl;
    cmd_vel.angular.z = this->pid_sita.run_step(target_pose,current_pose);
    std::cout<< "z: "<<cmd_vel.angular.z <<std::endl;
    return cmd_vel;


}