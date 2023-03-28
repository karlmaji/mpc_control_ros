#include <mpc_control/mpc_control.h>
#include "ros/ros.h"
#include "nav_msgs/Path.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose.h"
#include "tf/tf.h"
nav_msgs::Path global_path;
nav_msgs::Odometry current_odom;
nav_msgs::Path path;
using namespace MPC_CONTROL;

void ref_path_callbk(const nav_msgs::Path::ConstPtr& pt )
{
    global_path = *pt;

}
void current_odom_callbk(const nav_msgs::Odometry::ConstPtr& pt)
{
    geometry_msgs::PoseStamped this_pose_stamped;
    current_odom = *pt;
    this_pose_stamped.header = pt->header;
    this_pose_stamped.pose = pt->pose.pose;
    path.poses.push_back(this_pose_stamped);


}
void normalize_sita(const double &sita_before,double &sita_after)
{
    int k = (int)(sita_before / (2.f*M_PI));
    double k_plus = std::abs(sita_after - sita_before + (k + 1)*2.f*M_PI);
    double k_minu = std::abs(sita_after - sita_before + (k - 1)*2.f*M_PI);
    double k_none = std::abs(sita_after - sita_before);

    if(k_plus<k_minu && k_plus<k_none)sita_after +=(k+1)*2.f*M_PI;
    if(k_minu<k_plus && k_minu<k_none)sita_after +=(k-1)*2.f*M_PI;
    
}

int find_index_for_nowtime(const nav_msgs::Path & follow_path,const int begin_index=0)
{
    double t_now = ros::Time::now().toSec();
    for(int i=begin_index;i<follow_path.poses.size();i++)
    {
        double t_point = follow_path.poses[i].header.stamp.toSec();
        ROS_INFO("t_p%f,t_n%f",t_point,t_now);
        if((t_point - t_now)<0) continue;
        else return i;
    }
    return -1;

}

int main(int argc, char **argv)
{
    ros::init(argc,argv,"mpc_control_node");
    ros::NodeHandle n;
    ros::Rate loop_rate(10);
    ros::Rate rate(1);

    MPC_Control mpc;
    mpc.init("mpc_control_node");
    Eigen::Matrix<double,3,1> x0;
    std::vector<Eigen::Matrix<double,3,1>> xref;
    std::vector<Eigen::Matrix<double,2,1>> uref;

    geometry_msgs::Twist cmd_vel;


    ros::Subscriber sub = n.subscribe("path_ref",1000,ref_path_callbk);

    ros::Subscriber odom_sub = n.subscribe("odom",1000,current_odom_callbk);

    ros::Publisher cmd_vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel",1000);

    ros::Publisher path_pub = n.advertise<nav_msgs::Path>("path_real",1000);
    path.header.frame_id ="map";
    
    //-----------------------------------------//
    
    for(int i=0;i<mpc._mpc_window;i++)
    {
        Eigen::Matrix<double,2,1> u_iter;
        u_iter<<0.0,0;
        uref.push_back(u_iter);
    }





    while(ros::ok())
    {
        static int count=0;
        double sita_before;
        double sita;
        
        ros::spinOnce();
    


        if(global_path.poses.size() >0) 
        {
            count = find_index_for_nowtime(global_path);
            ROS_INFO("count:%d",count);

            if(count!=-1 && count < global_path.poses.size()-mpc._mpc_window-1)

            {

            for(int i=0;i<mpc._mpc_window+1;i++)
            {   
                Eigen::Matrix<double,3,1> x_iter;
                double x=global_path.poses.at(count+i).pose.position.x;
                double y=global_path.poses.at(count+i).pose.position.y;
                sita = tf::getYaw(global_path.poses.at(count+i).pose.orientation);

                if(i==0)
                {
                    double sita_now = tf::getYaw(current_odom.pose.pose.orientation);
                    normalize_sita(sita_now,sita);
                    sita_before = sita;

                }
                else
                {
                    ROS_INFO("%f before",sita);
                    normalize_sita(sita_before,sita);
                    ROS_INFO("%f after",sita);
                    sita_before = sita;

                }
                x_iter<<x,y,sita; 
                
                
                xref.push_back(x_iter);
            }
            double x=current_odom.pose.pose.position.x;
            double y=current_odom.pose.pose.position.y;
            sita = tf::getYaw(current_odom.pose.pose.orientation);
            
            
            

            x0 << x,y,sita;
            if(!mpc.step(x0,xref,uref))
            {
                cmd_vel.linear.x = mpc.QPctr(0,0);
                cmd_vel.angular.z = mpc.QPctr(1,0);
                cmd_vel_pub.publish(cmd_vel);

                std::cout<< mpc.QPctr<<std::endl;

            }
            else{
                ROS_INFO("ERROR! SOVER Faild!");
            }


            xref.clear();
            path_pub.publish(path);

            }
            







            //if(count<global_path.poses.size()-1-mpc._mpc_window-1)count++ ;else return 0;
        }

        


        loop_rate.sleep();
    






    }
    


   return 0;


}