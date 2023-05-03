#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseStamped.h"
#include "visualization_msgs/Marker.h"
geometry_msgs::PoseStamped pose_car;


void car_odom_callbk(const geometry_msgs::PoseStampedConstPtr &pt)
{
    pose_car = *pt;
}

int main(int argc, char **argv)
{
    //ros节点初始化 "listener"节点名称，在ROS里同一时间不允许出现两个
    ros::init(argc,argv,"show_car_in_nokov");
    ros::NodeHandle n;
    ros::NodeHandle n_private("~");
    float x_scale,y_scale;
    std::string pose_topic,pub_topic_name;
    n_private.param<float>("x_scale",x_scale,0.2);
    n_private.param<float>("y_scale",y_scale,0.1);
    n_private.param<std::string>("pose_topic",pose_topic,"robot_1/nokov");
    n_private.param<std::string>("pub_topic_name",pub_topic_name,"car_marker");
    ros::Time::init();


    ros::Subscriber car_odom_sub = n.subscribe(pose_topic,10,car_odom_callbk);
    ros::Publisher car_marker_pub = n.advertise<visualization_msgs::Marker>(pub_topic_name,10);
    
    boost::shared_ptr<geometry_msgs::PoseStamped const> post_1;
    post_1  = ros::topic::waitForMessage<geometry_msgs::PoseStamped>(pose_topic);

    visualization_msgs::Marker obs_marker;
    obs_marker.header.frame_id = "map";
    obs_marker.header.stamp = ros::Time::now();
    obs_marker.ns = "basic_shapes";
    obs_marker.id = 0;
    obs_marker.type = visualization_msgs::Marker::CUBE;
    obs_marker.action = visualization_msgs::Marker::ADD;
    obs_marker.pose =post_1->pose;
    obs_marker.scale.x = x_scale;
    obs_marker.scale.y = y_scale;
    obs_marker.scale.z = 0.2;
    obs_marker.color.r = 1.0f;
    obs_marker.color.g = 0.0f;
    obs_marker.color.b = 0.0f;
    obs_marker.color.a = 1.0;
    obs_marker.lifetime = ros::Duration();
    car_marker_pub.publish(obs_marker);

    ros::Rate loop_rate(20);
    while (ros::ok())
    {
        ros::spinOnce();
        visualization_msgs::Marker obs_marker;
        obs_marker.header.frame_id = "map";
        obs_marker.header.stamp = ros::Time::now();
        obs_marker.ns = "basic_shapes";
        obs_marker.id = 0;
        obs_marker.type = visualization_msgs::Marker::CUBE;
        obs_marker.action = visualization_msgs::Marker::MODIFY;
        obs_marker.pose = pose_car.pose;
        obs_marker.scale.x = x_scale;
        obs_marker.scale.y = y_scale;
        obs_marker.scale.z = 0.2;
        obs_marker.color.r = 1.0f;
        obs_marker.color.g = 0.0f;
        obs_marker.color.b = 0.0f;
        obs_marker.color.a = 1.0;
        car_marker_pub.publish(obs_marker);
        loop_rate.sleep();
        /* code */
    }
    




}