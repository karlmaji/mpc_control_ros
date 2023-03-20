
#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "rosbag/bag.h"

nav_msgs::Odometry ods ;
float x=0.;
float y=0.;
void odomCallback(const nav_msgs::Odometry::ConstPtr& od){
    x = od->pose.pose.position.x;
    y = od->pose.pose.position.y;
    ods = *od;
}

int main(int argc, char **argv)
{
    //ros节点初始化 "listener"节点名称，在ROS里同一时间不允许出现两个
    ros::init(argc,argv,"path_publisher");
    //创建节点句柄
    ros::NodeHandle n;
    rosbag::Bag bag;
    ros::Time::init();
    ros::Subscriber path_sub = n.subscribe("odom",1000,odomCallback);
    
    std::string bag_file;
    
    n.param<std::string>("bag_file",bag_file,"/home/karl/ros1/src/mpc_control/src/test1.bag");

    ros::Rate loop_rate(10);

    bag.open(bag_file,rosbag::bagmode::Write);

    while(ros::ok())
    {   
        ROS_INFO("x is : %2f,y is : %2f ",x,y);
        ros::spinOnce();

        bag.write("odom",ros::Time::now(),ods);

        loop_rate.sleep();

        
    }
    bag.close();
    return 0;
} 