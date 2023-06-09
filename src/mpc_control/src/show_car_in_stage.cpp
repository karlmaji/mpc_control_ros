#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "visualization_msgs/Marker.h"
nav_msgs::Odometry odom;


void car_odom_callbk(const nav_msgs::OdometryConstPtr &pt)
{
    odom = *pt;
}

int main(int argc, char **argv)
{
    //ros节点初始化 "listener"节点名称，在ROS里同一时间不允许出现两个
    ros::init(argc,argv,"show_car_in_stage");
    ros::NodeHandle n;
    ros::NodeHandle n_private("~");
    float x_scale,y_scale;
    n_private.param<float>("x_scale",x_scale,0.2);
    n_private.param<float>("y_scale",y_scale,0.1);
    ros::Time::init();
    void car_odom_callbk(const nav_msgs::OdometryConstPtr &pt);
    ros::Subscriber car_odom_sub = n.subscribe("odom",10,car_odom_callbk);
    ros::Publisher car_marker_pub = n.advertise<visualization_msgs::Marker>("car_marker",10);

    visualization_msgs::Marker obs_marker;
    obs_marker.header.frame_id = "map";
    obs_marker.header.stamp = ros::Time::now();
    obs_marker.ns = "basic_shapes";
    obs_marker.id = 0;
    obs_marker.type = visualization_msgs::Marker::CUBE;
    obs_marker.action = visualization_msgs::Marker::ADD;
    obs_marker.pose.position.x = 2.00;
    obs_marker.pose.position.y = 2.00;
    obs_marker.scale.x = x_scale;
    obs_marker.scale.y = y_scale;
    obs_marker.scale.z = 0.1;
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
        obs_marker.pose = odom.pose.pose;
        obs_marker.scale.x = x_scale;
        obs_marker.scale.y = y_scale;
        obs_marker.scale.z = 0.1;
        obs_marker.color.r = 1.0f;
        obs_marker.color.g = 0.0f;
        obs_marker.color.b = 0.0f;
        obs_marker.color.a = 1.0;
        car_marker_pub.publish(obs_marker);
        loop_rate.sleep();
        /* code */
    }
    




}