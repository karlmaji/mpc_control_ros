#include <mpc_control/mpc_control_fix.h>
#include "ros/ros.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose.h"
#include "nav_msgs/Odometry.h"
#include "tf/tf.h"
nav_msgs::Path global_path;
geometry_msgs::PoseStamped current_pose;
nav_msgs::Odometry current_odom;
nav_msgs::Path path;
using namespace MPC_CONTROL_FIX;

void ref_path_callbk(const nav_msgs::Path::ConstPtr& pt )
{
    global_path = *pt;

}
void current_pose_callbk(const geometry_msgs::PoseStamped::ConstPtr& pt)
{
    geometry_msgs::PoseStamped this_pose_stamped;
    current_pose = *pt;
    this_pose_stamped.header = pt->header;
    this_pose_stamped.pose = pt->pose;
    path.poses.push_back(this_pose_stamped);
}

void current_odom_callbk(const nav_msgs::Odometry::ConstPtr& pt)
{
    
    current_odom = *pt;
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
        if((t_point - t_now)<0) continue;
        else return i;
    }
    return -1;

}


int main(int argc, char **argv)
{
    ros::init(argc,argv,"mpc_control_node_nokov");
    ros::NodeHandle n;
    ros::Rate loop_rate(20);
    MPC_Control mpc;
    mpc.init("mpc_control_node_nokov");


    Eigen::Matrix<double,3,1> x0;
    Eigen::Matrix<double,2,1> u0;
    std::vector<Eigen::Matrix<double,3,1>> xref;
    std::vector<Eigen::Matrix<double,2,1>> uref;

    geometry_msgs::Twist cmd_vel;


    ros::Subscriber sub = n.subscribe("path_ref",1000,ref_path_callbk);

    ros::Subscriber pose_sub = n.subscribe("nokov",1000,current_pose_callbk);
    ros::Subscriber odom_sub = n.subscribe("odom",1000,current_odom_callbk);

    ros::Publisher cmd_vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel",1000);

    ros::Publisher path_pub = n.advertise<nav_msgs::Path>("path_real",1000);
    ros::Publisher path_follow_pub = n.advertise<nav_msgs::Path>("path_to_follow",1000);
    ros::Publisher path_pred_pub = n.advertise<nav_msgs::Path>("path_pred",1000);
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
                double x_now=current_pose.pose.position.x;
                double y_now=current_pose.pose.position.y;
                double sita_now = tf::getYaw(current_pose.pose.orientation);
                double v_now =current_odom.twist.twist.linear.x;
                double sita_a_now = current_odom.twist.twist.angular.z;

                ROS_INFO("%f , %f" , v_now,sita_a_now);
            

                nav_msgs::Path path_to_follow;
                path_to_follow.header.frame_id="map";
                path_to_follow.poses.clear();

                nav_msgs::Path path_pred;
                path_pred.header.frame_id="map";
                path_pred.poses.clear();

            for(int i=0;i<mpc._mpc_window+1;i++)
            {   
                Eigen::Matrix<double,3,1> x_iter;
                double x=global_path.poses.at(count+i).pose.position.x;
                double y=global_path.poses.at(count+i).pose.position.y;
                sita = tf::getYaw(global_path.poses.at(count+i).pose.orientation);
                sita = sita - sita_now;


                geometry_msgs::PoseStamped pose_stamp;
                pose_stamp = global_path.poses.at(count+i);
                
                path_to_follow.poses.push_back(pose_stamp);


                

                

                if(i==0)
                {
                    normalize_sita(0,sita);
                    sita_before = sita;
                }
                else
                {
                    normalize_sita(sita_before,sita);

                    sita_before = sita;

                }
                x_iter<<x - x_now,y - y_now,sita; 
                
                
                xref.push_back(x_iter);
            }

            
            
            

            x0 << x_now,y_now,sita_now;
            u0 << v_now,sita_a_now;
            if(!mpc.step(x0,u0,xref,uref))
            {

                double v_ = mpc.QPctr(0,0);
                double sita_ = mpc.QPctr(1,0);
                if(v_>5 || v_<-5) v_=0;
                if(sita_>5 || sita_ <-5) sita_ = 0;
                cmd_vel.linear.x = v_;
                cmd_vel.angular.z = sita_;
                cmd_vel_pub.publish(cmd_vel);

                std::cout<< mpc.QPctr<<std::endl;

                for(int i=0; i<mpc._mpc_window +1 ;i++)
                {
                    geometry_msgs::PoseStamped this_pose_stamped;
                    this_pose_stamped.pose.position.x = mpc.QPSolution(3*i,0) + x_now;
                    this_pose_stamped.pose.position.y = mpc.QPSolution(3*i+1,0) + y_now;
                    path_pred.poses.push_back(this_pose_stamped);
                
                }
            

            }
            else{
                ROS_INFO("ERROR! SOVER Faild!");
            }



            xref.clear();
            path_pub.publish(path);
            path_follow_pub.publish(path_to_follow);
            path_pred_pub.publish(path_pred);
           

            }
            else
            {
                cmd_vel.linear.x = 0;
                cmd_vel.angular.z = 0;
                cmd_vel_pub.publish(cmd_vel);

            }
            







            //if(count<global_path.poses.size()-1-mpc._mpc_window-1)count++ ;else return 0;
        }

        


        loop_rate.sleep();
    






    }
    


   return 0;


}