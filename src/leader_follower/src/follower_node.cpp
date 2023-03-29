#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseStamped.h"
nav_msgs::Path leader_path;
nav_msgs::Path ref_path;

class follower_node
{
    public:
    ros::NodeHandle n;
    ros::Subscriber path_sub = n.subscribe("leader_path",1000,&follower_node::leader_path_callbk,this);
    ros::Publisher path_pub = n.advertise<nav_msgs::Path>("path_ref",10);
    void leader_path_callbk(const nav_msgs::PathConstPtr& pt);
};

void follower_node::leader_path_callbk(const nav_msgs::PathConstPtr& pt)
{
    double s=0;
    int index=-1;
    leader_path = *pt;
    for(int i=leader_path.poses.size()-1;i>0;i--)
    {
        double x= leader_path.poses[i].pose.position.x - leader_path.poses[i-1].pose.position.x;
        double y= leader_path.poses[i].pose.position.y - leader_path.poses[i-1].pose.position.y;
        s += sqrt(pow(x,2) + pow(y,2));
        if(s>1.0) {index = i;break;}

    }
    if(index!=-1){
        ref_path.poses.clear();
        ref_path.header.frame_id = "map";
        auto time_now = ros::Time::now();
        for(int i = 0;i<60;i++)
        {
            geometry_msgs::PoseStamped pose_stamp;
            pose_stamp.header.frame_id = "map";
            pose_stamp.header.stamp = time_now + ros::Duration(0.1 * (i+1));
            pose_stamp.pose = leader_path.poses[index].pose;
            

            ref_path.poses.push_back(pose_stamp);
        }
        path_pub.publish(ref_path);
    }

}

int main(int argc, char **argv)
{
    //ros节点初始化 "listener"节点名称，在ROS里同一时间不允许出现两个
    ros::init(argc,argv,"follower_node");
    //创建节点句柄
    ros::NodeHandle n;
    follower_node follower_node;
    ros::Time::init();
    

    

    while (ros::ok())
    {
        ros::spinOnce();
        /* code */
    }
    
    return 0;
} 