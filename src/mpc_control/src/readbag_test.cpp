#include "ros/ros.h"
#include "rosbag/bag.h"
#include "rosbag/view.h"
#include "nav_msgs/Odometry.h"
int main(int argc, char **argv)
{
    ros::init(argc,argv,"read_bag");
    ros::NodeHandle n;
    rosbag::Bag bag;
    bag.open("/home/karl/ros1/src/mpc_control/src/test.bag");



    for(rosbag::MessageInstance const m : rosbag::View(bag))
   {
    nav_msgs::Odometry::ConstPtr s = m.instantiate<nav_msgs::Odometry>();
    if(s!=nullptr){
        ROS_INFO("x is : %2f,y is : %2f ",s->pose.pose.position.x,s->pose.pose.position.y);
    }

   }
   bag.close();
   return 0;


}