
#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseStamped.h"


class create_car_path_class
{
    public:
    ros::NodeHandle n;
    geometry_msgs::PoseStamped pose;
    nav_msgs::Path leader_path;
    void odomCallback(const nav_msgs::Odometry::ConstPtr& od);
    ros::Subscriber path_sub = n.subscribe("odom",1000,&create_car_path_class::odomCallback,this);
    ros::Publisher path_pub = n.advertise<nav_msgs::Path>("leader_path",10);
   
};

void create_car_path_class::odomCallback(const nav_msgs::Odometry::ConstPtr& od){

    geometry_msgs::PoseStamped pose;
    pose.pose = od->pose.pose;
    pose.header = od->header;

    leader_path.poses.push_back(pose);
    leader_path.header.frame_id ="map";
    path_pub.publish(leader_path);
}

int main(int argc, char **argv)
{
    //ros节点初始化 "listener"节点名称，在ROS里同一时间不允许出现两个
    ros::init(argc,argv,"create_car_path");
    //创建节点句柄
    create_car_path_class create_car_path;


    

    

    while (ros::ok())
    {
        ros::spin();
    }
    
    return 0;
} 