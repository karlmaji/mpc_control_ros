#include "ros/ros.h"
#include "DynamicPlaning.h"
#include "car.h"
#include "transform.h"
#include "Sensor.h"
#include "nav_msgs/Path.h"

nav_msgs::Path ref_path;
void ref_path_ck(nav_msgs::PathConstPtr &pt)
{
    ref_path = *pt;
}



int main(int argc, char **argv)
{
    ros::init(argc,argv,"dynamic_planning_node");
    ros::NodeHandle n;
    std::string ref_path_topic_name;
    n.param<std::string>("/dynamic_planning_node/ref_path_topic_name",
                            ref_path_topic_name,"path_ref");
    
    ros::Subscriber ref_path_sub = n.subscribe(ref_path_topic_name,10,ref_path_ck);
    







   //Reference referenceline[]










}