#ifndef MPC_CONTROL_NODE_CLASS_H
#define MPC_CONTROL_NODE_CLASS_H

#include "mpc_control_fix.h"
#include "ros/ros.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose.h"
#include "nav_msgs/Odometry.h"
#include "tf/tf.h"

using namespace MPC_CONTROL_FIX;

namespace mpc_control_node_class
{

class mpc_control_node : public MPC_Control
{
    public:
        mpc_control_node(ros::NodeHandle* nodehandle);
        void run_step();
        int control_frequency;


        
    private:
        ros::NodeHandle nh_;
        ros:: Subscriber ref_path_sub;
        ros:: Subscriber pose_sub;
        ros:: Subscriber odom_sub;
        ros::Publisher cmd_vel_pub;
        ros::Publisher path_real_pub;
        ros::Publisher path_follow_pub;
        ros::Publisher path_pred_pub;

        nav_msgs::Path ref_path;
        geometry_msgs::PoseStamped current_pose;
        nav_msgs::Odometry current_odom;
        nav_msgs::Path record_path;



        void ref_path_callbk(const nav_msgs::Path::ConstPtr& pt );
        void current_pose_callbk(const geometry_msgs::PoseStamped::ConstPtr& pt);
        void current_odom_callbk(const nav_msgs::Odometry::ConstPtr& pt);
        void normalize_sita(const double &sita_before,double &sita_after);
        int find_index_for_nowtime(const nav_msgs::Path & follow_path,const int begin_index=0);
        
    










};

}


#endif
