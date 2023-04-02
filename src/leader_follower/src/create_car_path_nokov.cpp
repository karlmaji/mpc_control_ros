
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
    geometry_msgs::PoseStamped pose_ago;
    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& od);
    ros::Subscriber path_sub = n.subscribe("nokov",1000,&create_car_path_class::poseCallback,this);
    ros::Publisher path_pub = n.advertise<nav_msgs::Path>("leader_path",10);
   
};

void create_car_path_class::poseCallback(const geometry_msgs::PoseStamped::ConstPtr& od){

    geometry_msgs::PoseStamped pose;
    pose.pose = od->pose;
    pose.header = od->header;
    
    //if(leader_path.poses.size()>1000) leader_path.poses.erase(leader_path.poses.begin());
    if(abs(pose_ago.pose.position.x - pose.pose.position.x)>0.001 || abs(pose_ago.pose.position.y - pose.pose.position.y)>0.001)
    {
    leader_path.poses.push_back(pose);
    leader_path.header.frame_id ="map";
    path_pub.publish(leader_path);
    }
    pose_ago = pose;
}

int main(int argc, char **argv)
{
    //ros节点初始化 "listener"节点名称，在ROS里同一时间不允许出现两个
    ros::init(argc,argv,"create_car_path_nokov");
    //创建节点句柄
    create_car_path_class create_car_path;


    

    

    ros::spin();
    
    return 0;
} 