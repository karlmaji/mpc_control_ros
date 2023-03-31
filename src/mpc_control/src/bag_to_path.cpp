#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Path.h"
#include "rosbag/bag.h"
#include "rosbag/view.h"

std::vector<geometry_msgs::PoseStamped> ods; 

int main(int argc, char **argv)
{
    //ros节点初始化 "listener"节点名称，在ROS里同一时间不允许出现两个
    ros::init(argc,argv,"bag_to_path");
    //创建节点句柄
    ros::NodeHandle n;
    std::string bag_file,path_topic_name;
    rosbag::Bag bag;
    ros::NodeHandle n_private("~");

    n_private.param<std::string>("bag_file",bag_file,"/home/yl-01/mh_code/ros1/src/mpc_control/src/test_nokov.bag");
    n_private.param<std::string>("path_topic_name",path_topic_name,"path_global");
    bag.open(bag_file);
    ros::Publisher pub = n.advertise<nav_msgs::Path>(path_topic_name,10);
    // std::vector<std::string> topics;
    // rosbag::View v(bag,rosbag::TopicQuery(topics));

    for(rosbag::MessageInstance const m: rosbag::View(bag))
   {
    geometry_msgs::PoseStamped::ConstPtr s = m.instantiate<geometry_msgs::PoseStamped>();
    if(s!=nullptr){
        ROS_INFO("x is : %2f,y is : %2f ",s->pose.position.x,s->pose.position.y);
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
        double x = ods[i].pose.position.x;
        double y = ods[i].pose.position.y;
        if(std::pow(x,2)+std::pow(y,2)>0.1)
        {
        this_pose_stamped.pose = ods[i].pose;
        this_pose_stamped.header = ods[i].header;
        this_pose_stamped.header.stamp = current_time + ros::Duration(0.05 * (i+1));
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