#include "mpc_control_increamental.h"
#include "ros/ros.h"
#include "nav_msgs/Path.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose.h"
#include "tf/tf.h"
nav_msgs::Path global_path;
nav_msgs::Odometry current_odom;
nav_msgs::Path path;
double v_ago=0;
double sita_a_ago=0;
using namespace MPC_CONTROL_INCREAMENTAL;

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
    auto time_now = ros::Time::now();
    double t_now = time_now.toSec();
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
    ros::init(argc,argv,"mpc_control_node_increamental");
    ros::NodeHandle n;
    ros::Rate loop_rate(10);
    ros::Rate rate(1);
    MPC_Control mpc;
    mpc.init("mpc_control_node_increamental");
    Eigen::Matrix<double,5,1> x0;
    std::vector<Eigen::Matrix<double,5,1>> xref;

    geometry_msgs::Twist cmd_vel;


    ros::Subscriber sub = n.subscribe("path_ref",10,ref_path_callbk);

    ros::Subscriber odom_sub = n.subscribe("odom",10,current_odom_callbk);

    ros::Publisher cmd_vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel",10);

    ros::Publisher path_pub = n.advertise<nav_msgs::Path>("path_real",10);

    ros::Publisher path_pred_pub = n.advertise<nav_msgs::Path>("path_pred",10);

    ros::Publisher path_follow_pub = n.advertise<nav_msgs::Path>("path_follow",10);
    path.header.frame_id ="map";
    
    //-----------------------------------------//
    double v_now;
    double sita_a_now ;

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
                double x_now = current_odom.pose.pose.position.x;
                double y_now = current_odom.pose.pose.position.y;
                double sita_now = tf::getYaw(current_odom.pose.pose.orientation);
                v_now = current_odom.twist.twist.linear.x ;
                sita_a_now = current_odom.twist.twist.angular.z;
    

                
                ROS_INFO("%f , %f" , v_now,sita_a_now);

                nav_msgs::Path path_pred;
                path_pred.poses.clear();
                path_pred.header.frame_id="map";
                nav_msgs::Path path_follow;
                path_follow.poses.clear();
                path_follow.header.frame_id="map";

            for(int i=0;i<mpc._mpc_window+1;i++)
            {   
                Eigen::Matrix<double,5,1> x_iter;
                double x=global_path.poses.at(count+i).pose.position.x;
                double y=global_path.poses.at(count+i).pose.position.y;
                sita = tf::getYaw(global_path.poses.at(count+i).pose.orientation);
                sita = sita - sita_now;
                geometry_msgs::PoseStamped this_pose_stamped;
                this_pose_stamped.pose.position.x = x;
                this_pose_stamped.pose.position.y = y;
                path_follow.poses.push_back(this_pose_stamped);

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
                x_iter<<x-x_now,y-y_now,sita,0,0; 
                
                
                xref.push_back(x_iter);
            }

            
            
            

            x0 << x_now,y_now,sita_now,v_now,sita_a_now;
            if(!mpc.step(x0,xref))
            {

                cmd_vel.linear.x = mpc.QPSolution(3+5,0);
                cmd_vel.angular.z = mpc.QPSolution(4+5,0);

                cmd_vel_pub.publish(cmd_vel);
                
                
                
                std::cout<< "-------" <<std::endl;
                std::cout<< cmd_vel<<std::endl;

                for(int i =0;i<mpc._mpc_window+1;i++)
                {
                geometry_msgs::PoseStamped this_pose_stamped;
                this_pose_stamped.pose.position.x = mpc.QPSolution(5*i,0) + x_now;
                this_pose_stamped.pose.position.y = mpc.QPSolution(5*i+1,0) + y_now;
                path_pred.poses.push_back(this_pose_stamped);
                path_pred_pub.publish(path_pred);

                }


            }
            else{
                ROS_INFO("ERROR! SOVER Faild!");
            }


            xref.clear();
            path_pub.publish(path);
            path_follow_pub.publish(path_follow);

            }
            







            //if(count<global_path.poses.size()-1-mpc._mpc_window-1)count++ ;else return 0;
        }

        


        loop_rate.sleep();
    






    }
    


   return 0;


}