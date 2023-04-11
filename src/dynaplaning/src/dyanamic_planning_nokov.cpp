#include "ros/ros.h"
#include "DynamicPlaning.h"
#include "car.h"
#include "transform.h"
#include "Sensor.h"
#include "nav_msgs/Path.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PointStamped.h"
#include "tf/tf.h"
#include "visualization_msgs/Marker.h"
nav_msgs::Path global_path;
geometry_msgs::PoseStamped last_time_car_pose;
Car car_p;
std::vector<float> x;
std::vector<float> y;
std::vector<float> sita;


Trajectory traj[300];
std::vector<Reference> global_path_reference;
int GetLocalRef(int &start_index,float car_x,float car_y,float ref_length,std::vector<Reference> &path,Reference* p)
{
  int index=start_index;
  int start,end;
  float distances=0;
  float min_distance=std::sqrt((car_x-path[start_index].x_r)*(car_x-path[start_index].x_r)
                              +(car_y-path[start_index].y_r)*(car_y-path[start_index].y_r));
  int times=0;
  for(int i=start_index+1;i<path.size();i++)
  {
    distances = std::sqrt((car_x-path[i].x_r)*(car_x-path[i].x_r)
                        +(car_y-path[i].y_r)*(car_y-path[i].y_r));
    //std::cout<<"distance:"<<distances<<std::endl;
    if(distances<=min_distance)
    {
        min_distance=distances;
        index=i;
        start_index = index;
    }
    else
    {
        times++;
        if(times>5)
            break;
    }
  }
  start=0;
  end=path.size()-1;
  for(int i=index;i>=0;i--)
  {
    if(path[index].s_r-path[i].s_r<ref_length/6)
    {
        //start=i;
        continue;
    }
    else
    {
       start=i;
       break;
    }
  }

  for(int i=index;i<path.size();i++)
  {
    if(path[i].s_r-path[index].s_r<ref_length*5/6)
    {
        continue;
    }
    else
    {
       end=i;
       break;
    }
  }
 for(int i=0;i<=end-start;i++)
  {
    p[i]=path[i+start];
  }
  return end-start+1;
}

void getref(std::vector<float> &x,std::vector<float> &y,std::vector<float> &sita,std::vector<Reference> &path)
{
   path.resize(x.size());
   float ds;
   path[0].x_r=x[0];
   path[0].y_r=y[0];
   path[0].heading_r=sita[0];
   path[0].s_r=0;
   path[0].k_r=abs((sita[1]-sita[0])/std::sqrt(1e-4+(x[1]-x[0])*(x[1]-x[0])+(y[1]-y[0])*(y[1]-y[0])));
  for(int i=1;i<x.size();i++)
  {
   path[i].x_r=x[i];
   path[i].y_r=y[i];
   path[i].heading_r=sita[i];
   ds=std::sqrt((x[i]-x[i-1])*(x[i]-x[i-1])+(y[i]-y[i-1])*(y[i]-y[i-1])) +1e-4;

   float dsita = sita[i] - sita[i-1];
   if(abs(dsita)> abs(dsita + M_2_PI)) {dsita += M_2_PI;}
   else if(abs(dsita)>abs(dsita- M_2_PI))   {dsita -=M_2_PI;}


   path[i].s_r=path[i-1].s_r+ds;
   path[i].k_r=abs(dsita/ds);
   path[i].k_rd=(path[i].k_r-path[i-1].k_r)/(path[i].s_r-path[i-1].s_r+1e-3);
  }
  path[0].k_rd=(path[1].k_r-path[0].k_r)/(path[1].s_r-path[0].s_r+1e-4);
}

void global_path_ck(const nav_msgs::PathConstPtr &pt)
{
    global_path = *pt;
    std::vector<float> x,y,sita;
    for(int i=0;i<global_path.poses.size();i++)
    {
        x.push_back(global_path.poses[i].pose.position.x);
        y.push_back(global_path.poses[i].pose.position.y);
        sita.push_back(tf::getYaw(global_path.poses[i].pose.orientation));
    }
    getref(x,y,sita,global_path_reference);

}
void car_pose_ck(const geometry_msgs::PoseStamped::ConstPtr& pt)
{
    car_p.car_center_x = pt->pose.position.x;
    car_p.car_center_y = pt->pose.position.y;
    car_p.car_acc = 0;
    car_p.car_hesding = tf::getYaw(pt->pose.orientation);
    car_p.car_length = 0.4;
    car_p.car_width = 0.3;
    car_p.car_velocity = 0;
    float dsita = tf::getYaw(pt->pose.orientation) - tf::getYaw(last_time_car_pose.pose.orientation);
    if(abs(dsita)> abs(dsita + M_2_PI)) {dsita += M_2_PI;}
    else if(abs(dsita)>abs(dsita- M_2_PI))   {dsita -=M_2_PI;}
    //car_p.car_velocity
    car_p.car_k = abs(dsita)/
                  ( std::sqrt( pow((pt->pose.position.x - last_time_car_pose.pose.position.x),2)
                             + pow((pt->pose.position.y - last_time_car_pose.pose.position.y),2)    ) +1e-3  );
    
    last_time_car_pose = *pt;
}



int main(int argc, char **argv)
{
    int start_index=0;
    ros::init(argc,argv,"dynamic_planning_nokov");
    ros::NodeHandle n;
    ros::NodeHandle n_private("~");
    std::string global_path_topic_name,car_pose_topic_name;

    n_private.param<std::string>("global_path_topic_name",
                            global_path_topic_name,"global_path");

    n_private.param<std::string>("car_pose_topic_name",
                            car_pose_topic_name,"nokov");
    

    float w_smooth[3]={100000,5000,2000};
    float q_w_smooth[3]={1000,500,200};

    Obs obs_p[3];

    Reference referenceline[1000];

    boost::shared_ptr<geometry_msgs::PoseStamped const> post_1;
    post_1  = ros::topic::waitForMessage<geometry_msgs::PoseStamped>("robot_2/nokov");
    obs_p[0].obs_acc=0;
    obs_p[0].obs_center_x= post_1->pose.position.x;
    obs_p[0].obs_center_y= post_1->pose.position.y;
    obs_p[0].obs_hesding= tf::getYaw(post_1->pose.orientation);
    obs_p[0].obs_k=0;
    obs_p[0].obs_length=0.4;
    obs_p[0].obs_velocity=0;
    obs_p[0].obs_width=0.3;

    post_1  = ros::topic::waitForMessage<geometry_msgs::PoseStamped>("robot_3/nokov");
    obs_p[1].obs_acc=0;
    obs_p[1].obs_center_x= post_1->pose.position.x;
    obs_p[1].obs_center_y= post_1->pose.position.y;
    obs_p[1].obs_hesding= tf::getYaw(post_1->pose.orientation);
    obs_p[1].obs_k=0;
    obs_p[1].obs_length=0.4;
    obs_p[1].obs_velocity=0;
    obs_p[1].obs_width=0.3;

    post_1  = ros::topic::waitForMessage<geometry_msgs::PoseStamped>("robot_4/nokov");
    obs_p[2].obs_acc=0;
    obs_p[2].obs_center_x= post_1->pose.position.x;
    obs_p[2].obs_center_y= post_1->pose.position.y;
    obs_p[2].obs_hesding= tf::getYaw(post_1->pose.orientation);
    obs_p[2].obs_k=0;
    obs_p[2].obs_length=0.4;
    obs_p[2].obs_velocity=0;
    obs_p[2].obs_width=0.3;


    ros::Subscriber ref_path_sub = n.subscribe(global_path_topic_name,10,global_path_ck);

    ros::Subscriber car_pose_sub = n.subscribe(car_pose_topic_name,10,car_pose_ck);
    
    ros::Publisher ref_path_pub = n.advertise<nav_msgs::Path>("path_ref",10);

    ros::Publisher obs_pub = n.advertise<visualization_msgs::Marker>("obs_show",10);

    ros::Rate rate_loop(2);


    while (ros::ok())
    {
        ros::spinOnce();
        visualization_msgs::Marker obs_marker;

        for(int i=0;i<3;i++)
        {
          obs_marker.header.frame_id = "map";
          obs_marker.header.stamp = ros::Time::now();
          obs_marker.ns = "basic_shapes";
          obs_marker.id = i;
          obs_marker.type = visualization_msgs::Marker::CUBE;
          obs_marker.action = visualization_msgs::Marker::ADD;
          obs_marker.pose.position.x = obs_p[i].obs_center_x;
          obs_marker.pose.position.y = obs_p[i].obs_center_y;
         
          obs_marker.pose.orientation =  tf::createQuaternionMsgFromYaw(obs_p[i].obs_hesding);
          obs_marker.scale.x = 0.3;
          obs_marker.scale.y = 0.2;
          obs_marker.scale.z = 0.2;
          obs_marker.color.r = 0.0f;
          obs_marker.color.g = 1.0f;
          obs_marker.color.b = 0.0f;
          obs_marker.color.a = 1.0;
          obs_marker.lifetime = ros::Duration();
          obs_pub.publish(obs_marker);
        }
        if(global_path_reference.size()>0)
        {
            std::cout << "path size"<<global_path_reference.size()<<std::endl;
            int length_point = GetLocalRef(start_index,car_p.car_center_x,car_p.car_center_y,4,global_path_reference,referenceline); 

            HostCar carp(car_p,referenceline,length_point);
            Obstacle obst(obs_p,3,referenceline,length_point);
            std::cout << "length_point"<<length_point <<std::endl;
            DynaPlaning DP(obst,carp,0,0.1,0.1,100,500,w_smooth,50,500,q_w_smooth,3);
            DP.traj=traj;
            DP.DynamicProgramming();

            std::cout<<"traj_num"<<DP.traj_num<<std::endl;
            nav_msgs::Path path;
            path.header.frame_id = "map";
            auto time_now = ros::Time::now();
            for(int i = 0;i<DP.traj_num;i++)
            {
                geometry_msgs::PoseStamped pose;
                pose.header.frame_id = "map";
                pose.header.stamp = time_now + ros::Duration(0.1 * (i+1));
                pose.pose.position.x = DP.traj[i].x;
                pose.pose.position.y = DP.traj[i].y;
                tf::Quaternion q = tf::createQuaternionFromRPY(0,0,DP.traj[i].heading);
                pose.pose.orientation.x = q.getX();
                pose.pose.orientation.y = q.getY();
                pose.pose.orientation.z = q.getZ();
                pose.pose.orientation.w = q.getW();

                path.poses.push_back(pose);
            }
            ref_path_pub.publish(path);
        }

       




        

        


        rate_loop.sleep();




        // HostCar carp(car_p,referenceline,181);
        // Obstacle obst(obs_p,3,referenceline,181);
        // DynaPlaning DP(obst,carp,0,1,6,100,50,w_smooth,50,10,q_w_smooth,60);
        // DP.traj=traj;
        // DP.DynamicProgramming();







        /* code */
    }
return 0;   
}