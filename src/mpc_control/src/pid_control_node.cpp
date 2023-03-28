#include "pid_control/pid_control.h"
#include "ros/ros.h"
#include "nav_msgs/Path.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose.h"

nav_msgs::Path global_path;
nav_msgs::Odometry current_odom;
void ref_path_callbk(const nav_msgs::Path::ConstPtr& pt )
{
    global_path = *pt;

}
void current_odom_callbk(const nav_msgs::Odometry::ConstPtr& pt)
{
    current_odom = *pt;

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
    ros::init(argc,argv,"read_bag");
    ros::NodeHandle n;
    ros::Rate loop_rate(10);

    VehiclePIDController control;
    control.set(1,0,0.1,1,0,0.2);
    geometry_msgs::Pose current_pose,target_pose;
    float target_vel,current_vel;
    geometry_msgs::Twist cmd_vel;


    ros::Subscriber sub = n.subscribe("path_ref",1000,ref_path_callbk);

    ros::Subscriber odom_sub = n.subscribe("odom",1000,current_odom_callbk);

    ros::Publisher cmd_vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel",1000);

    while(ros::ok())
    {
        static int count=0;
        
        ros::spinOnce();
    


        if(global_path.poses.size()!=0) 
        {
            current_pose = current_odom.pose.pose;

            count = find_index_for_nowtime(global_path);
            target_pose = global_path.poses.at(count).pose;

            double distance = pow(target_pose.position.x - current_pose.position.x,2)+ pow(target_pose.position.y - current_pose.position.y,2);

            
            if(distance>0.1){
            target_vel = 1;
            current_vel = current_odom.twist.twist.linear.x;

            cmd_vel = control.run_step(target_pose,current_pose,target_vel,current_vel);
            cmd_vel_pub.publish(cmd_vel);
            }

            //if(count<global_path.poses.size()-1)count++ ;else return 0;
            
        }
        


        loop_rate.sleep();
    






    }


   return 0;


}