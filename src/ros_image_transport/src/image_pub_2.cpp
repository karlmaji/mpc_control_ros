#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
sensor_msgs::Image img;
void imageCallback(const sensor_msgs::ImageConstPtr msg)
{
    img=*msg;
}

int main(int argc, char **argv)
{
    ros::init(argc,argv,"image_pub_node_2");
    ros::NodeHandle n;
    image_transport::ImageTransport it(n);
    image_transport::Subscriber sub = it.subscribe("image_2",1,imageCallback);
    image_transport::Publisher pub = it.advertise("image_transport_2",1);
    ros::Rate loop(10);
    while(ros::ok())
    {
        ros::spinOnce();
        pub.publish(img);
        loop.sleep();
    }
    
}
