#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
#include "rosbag/bag.h"
#include "rosbag/view.h"

std::vector<nav_msgs::Odometry> ods; 

int main(int argc, char **argv)
{
    //ros节点初始化 "listener"节点名称，在ROS里同一时间不允许出现两个
    ros::init(argc,argv,"odom_to_path");
    //创建节点句柄
    ros::NodeHandle n;
    std::string bag_file;
    ros::NodeHandle n_private("~");

    n_private.param<std::string>("bag_file",bag_file,"/home/karl/ros1/src/mpc_control/src/test.bag");
    rosbag::Bag bag;
    bag.open(bag_file);
    ros::Publisher pub = n.advertise<nav_msgs::Path>("global_path",1000);



    for(rosbag::MessageInstance const m: rosbag::View(bag))
   {
    nav_msgs::Odometry::ConstPtr s = m.instantiate<nav_msgs::Odometry>();
    if(s!=nullptr){
        ROS_INFO("x is : %2f,y is : %2f ",s->pose.pose.position.x,s->pose.pose.position.y);
        ods.push_back(*s);
    }
   }
    //去掉最后一个数据

    bag.close();

    ros::Time current_time;
    //ros::Duration delay_100ms = ros::Duration(100);

    nav_msgs::Path path;
    path.header.stamp = current_time;
    path.header.frame_id = "map";

    geometry_msgs::PoseStamped this_pose_stamped;
    current_time = ros::Time::now();

    for(int i=0 ;i<ods.size() - 5;i++)
    {   
        double x = ods[i].pose.pose.position.x;
        double y = ods[i].pose.pose.position.y;
        if(std::pow(x,2)+std::pow(y,2)>0.1)
        {
        this_pose_stamped.pose = ods[i].pose.pose;
        this_pose_stamped.header = ods[i].header;
        this_pose_stamped.header.stamp = current_time + ros::Duration(0.1 * (i+1));
        path.poses.push_back(this_pose_stamped);
        }

    }
    

    ros::Rate loop_rate(1);


    while(ros::ok())
    {
        pub.publish(path);
        loop_rate.sleep();


    }

    return 0;
}