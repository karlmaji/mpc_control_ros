#include<ros/ros.h>
#include<serial/serial.h>
#include<serial_stm32/angles.h>
serial::Serial ser;


void openserial()
{
    try
        {
            ser.setPort("/dev/ttyUSB0");//设备端口号
            ser.setBaudrate(115200);//波特率
            serial::Timeout t = serial::Timeout::simpleTimeout(1000);//这个应该是超时，但是是必须的！！ 
            ser.setTimeout(t); 
            ser.open();//打开串口
        }
    catch (serial::IOException& e) 
            { 
                ROS_ERROR_STREAM("Unable to open port "); 
            } 
    if(ser.isOpen())
        {
            ROS_INFO("OPEN");
        }

}

void angle_callback(const serial_stm32::anglesConstPtr& angles){
    uint8_t data[10];
    data[0] = 0xff;
    data[1] = 0xfe;
    data[2] = angles->horizontal_angle;
    data[3] = angles->vertical_angle;;
    data[4] = 0;
    data[5] = 0;
    data[6] = 0;
    data[7] = 0;
    data[8] = 0;
    data[9] = 0; 
    for(int i=2;i<9;i++)
    {
        data[9]^=data[i];
    }
    ROS_INFO("%d", data[9]);

    ser.write(data,10);
}


int main(int argc,char **argv)
{
    ros::init(argc,argv,"serial_stm32");
    ros::NodeHandle n;
    ros::Subscriber angle_sub = n.subscribe("angles",1,angle_callback);
    //ros::NodeHandle n_private("~");
    openserial();
    ros::Rate loop_rate(1);

    ros::spin();

    return 0;


    




    


}