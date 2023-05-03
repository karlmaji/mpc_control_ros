#include <iostream>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <cmath>
#include "OsqpEigen/OsqpEigen.h"
#include <sys/time.h>
#include "ros/ros.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose.h"
#include "nav_msgs/Odometry.h"
#include "tf/tf.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64MultiArray.h"
#include  <vector>
using namespace Eigen;
using namespace std;
//�ú�����������trajectory��s �� x y �Ķ�Ӧ��ϵ�����Կ�����trajectory index2s
double index2s(VectorXd trajectory_x, VectorXd trajectory_y, VectorXd& path_index2s);
float interp1(VectorXd path_index2s, VectorXd trajectory_kappa_init, int path_index2s_end_index, float cur_s);
double interp_heading(VectorXd path_s, VectorXd trajectory_heading, int index, double s);
double interp_time(VectorXd trajectory_time, VectorXd trajectory_x, int index, double time);
void planning_start_condition(float plan_start_vx, float plan_start_vy, float plan_start_ax, float plan_start_ay, float plan_start_heading, float out[2]);
void generate_st_graph(VectorXd& obs_st_s_in_set, VectorXd& obs_st_s_out_set, VectorXd& obs_st_t_in_set, VectorXd& obs_st_t_out_set, VectorXd dynamic_obs_s_set, VectorXd dynamic_obs_l_set, VectorXd dynamic_obs_s_dot_set, VectorXd dynamic_obs_l_dot_set);
void dynamic_planning(VectorXd obs_st_s_in_set, VectorXd obs_st_s_out_set,
	VectorXd obs_st_t_in_set, VectorXd obs_st_t_out_set, float reference_speed_unlimit,
	float w_cost_ref_speed, float w_cost_accel, float w_cost_obs, float plan_start_s_dot,
	VectorXd &dp_speed_s_out, VectorXd &dp_speed_t_out);
void generate_convex_space(VectorXd dp_speed_s, VectorXd dp_speed_t, VectorXd path_index2s, VectorXd obs_st_s_in_set, VectorXd obs_st_s_out_set,
	VectorXd obs_st_t_in_set, VectorXd obs_st_t_out_set, VectorXd trajectory_kappa_init, float max_lateral_accel, VectorXd& s_lb_out, VectorXd& s_ub_out, VectorXd& s_dot_lb_out, VectorXd& s_dot_ub_out);
void quadratic_programming(float plan_start_s_dot, float plan_start_s_dot2, VectorXd dp_speed_s, VectorXd dp_speed_t,
	VectorXd s_lb, VectorXd s_ub, VectorXd s_dot_lb, VectorXd s_dot_ub, float w_cost_s_dot2, float w_cost_v_ref, float w_cost_jerk, float speed_reference,
	VectorXd& qp_s_init_out, VectorXd& qp_s_dot_init_out, VectorXd& qp_s_dot2_init_out, VectorXd& relative_time_init_out);
VectorXd osqp(SparseMatrix<double> hessian, VectorXd gradient, SparseMatrix<double> linearMatrix,
	VectorXd lowerBound, VectorXd upperBound);
void increase_st_point_count(VectorXd s_init, VectorXd s_dot_init, VectorXd s_dot2_init, VectorXd relative_time_init,
	VectorXd& s_out, VectorXd& s_dot_out, VectorXd& s_dot2_out, VectorXd& relative_time_out);
void path_and_speed_merge(VectorXd s, VectorXd s_dot, VectorXd s_dot2, VectorXd relative_time, double current_time, VectorXd path_s,
	VectorXd trajectory_x_init, VectorXd trajectory_y_init, VectorXd trajectory_heading_init, VectorXd trajectory_kappa_init,
	VectorXd& trajectory_x_out, VectorXd& trajectory_y_out, VectorXd& trajectory_heading_out, VectorXd& trajectory_kappa_out,
	VectorXd& trajectory_speed_out, VectorXd& trajectory_accel_out, VectorXd& trajectory_time_out);
void stitch(VectorXd trajectory_x,VectorXd trajectory_y,VectorXd trajectory_heading,VectorXd trajectory_kappa,VectorXd trajectory_speed, 
VectorXd trajectory_accel,VectorXd trajectory_time,VectorXd trajectory_x_pre,VectorXd trajectory_y_pre,VectorXd trajectory_heading_pre,
VectorXd trajectory_kappa_pre,VectorXd trajectory_speed_pre, VectorXd trajectory_accel_pre,VectorXd trajectory_time_pre,double current_time,
VectorXd& trajectory_x_final,VectorXd& trajectory_y_final,VectorXd& trajectory_heading_final,VectorXd& trajectory_kappa_final,
VectorXd& trajectory_speed_final, VectorXd& trajectory_accel_final,VectorXd& trajectory_time_final);
void ST_Planner(VectorXd trajectory_x_pre,VectorXd trajectory_y_pre,VectorXd trajectory_heading_pre,
                VectorXd trajectory_kappa_pre,VectorXd trajectory_speed_pre, VectorXd trajectory_accel_pre,VectorXd trajectory_time_pre,
                VectorXd trajectory_x_init,VectorXd trajectory_y_init,VectorXd trajectory_heading_init,VectorXd  trajectory_kappa_init,
				double current_time,float vx,float vy,float ax,float ay,float heading,
				VectorXd dynamic_obs_s_set,VectorXd dynamic_obs_l_set,VectorXd dynamic_obs_s_dot_set,VectorXd dynamic_obs_l_dot_set,
				VectorXd& trajectory_x_out, VectorXd& trajectory_y_out,VectorXd& trajectory_heading_out,VectorXd& trajectory_kappa_out,
				VectorXd& trajectory_speed_out,VectorXd& trajectory_accel_out,VectorXd& trajectory_time_out,double* timeuse);