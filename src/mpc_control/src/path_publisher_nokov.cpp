
#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"

#include "rosbag/bag.h"

geometry_msgs::PoseStamped ods ;
float x=0.;
float y=0.;
void odomCallback(const geometry_msgs::PoseStamped::ConstPtr& od){
    x = od->pose.position.x;
    y = od->pose.position.y;
    ods = *od;
}

int main(int argc, char **argv)
{
    //ros节点初始化 "listener"节点名称，在ROS里同一时间不允许出现两个
    ros::init(argc,argv,"path_publisher_nokov");
    //创建节点句柄
    ros::NodeHandle n;
    ros::NodeHandle n_private("~");
    std::string bag_file,nokov_topic;
    rosbag::Bag bag;
    ros::Time::init();
    n_private.param<std::string>("bag_file",bag_file,"/home/yl-01/mh_code/ros1/src/mpc_control/src/test_nokov.bag");
    n_private.param<std::string>("nokov_topic",nokov_topic,"nokov");
    ros::Subscriber path_sub = n.subscribe(nokov_topic,100,odomCallback);
    ros::Rate loop_rate(20);

    bag.open(bag_file,rosbag::bagmode::Write);

    while(ros::ok())
    {   
        ROS_INFO("x is : %2f,y is : %2f ",x,y);
        ros::spinOnce();

        bag.write(nokov_topic,ros::Time::now(),ods);

        loop_rate.sleep();

        
    }
    bag.close();
    return 0;
} 