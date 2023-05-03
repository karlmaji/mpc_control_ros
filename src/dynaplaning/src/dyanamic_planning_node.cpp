#include "ros/ros.h"
#include "DynamicPlaning.h"
#include "car.h"
#include "transform.h"
#include "Sensor.h"
#include "nav_msgs/Path.h"
#include "nav_msgs/Odometry.h"
#include "tf/tf.h"
#include "visualization_msgs/Marker.h"
#include"std_msgs/Float64MultiArray.h"          
#include"std_msgs/String.h"
#include<vector>
nav_msgs::Path global_path;
nav_msgs::Odometry last_time_car_pose;
Car car_p;
std::vector<float> x;
std::vector<float> y;
std::vector<float> sita;
double obs_life_time1=0;
double obs_life_time2=0;
Trajectory traj[300];
std::vector<Reference> global_path_reference;
int GetLocalRef(int &start_index,float car_x,float car_y,float ref_length,std::vector<Reference> &path,Reference* p)
{
  int index=start_index;
  int start,end;
  float distances=0;
  float min_distance=std::sqrt((car_x-path[start_index].x_r)*(car_x-path[start_index].x_r)
                              +(car_y-path[start_index].y_r)*(car_y-path[start_index].y_r));
  //std::cout<<"x:"<<car_x<<std::endl;
  //std::cout<<"y:"<<car_y<<std::endl;
  //std::cout<<"min_distance:"<<min_distance<<std::endl;
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
  //std::cout<<"index"<<index<<std::endl;
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
  // std::cout<<"start "<<start<<" end "<<end<<std::endl;
  // std::cout<<"start_s "<<path[start].s_r<<" end_s "<<path[end].s_r<<std::endl;
  for(int i=0;i<=end-start;i++)
  {
    p[i]=path[i+start];
  }
//   std::cout<<p[40].x_r<<' '
//            <<p[40].y_r<<' '
//            <<p[40].heading_r<<' '
//            <<p[40].s_r<<' '<<std::endl;
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

void reference_get_s(Reference* path)
{
   float ds;
   path[0].s_r=0;
  for(int i=1;i<60;i++)
  {
   ds=std::sqrt((path[i].x_r-path[i-1].x_r)*(path[i].x_r-path[i-1].x_r)+(path[i].y_r-path[i-1].y_r)*(path[i].y_r-path[i-1].y_r)) +1e-4;
   path[i].s_r=path[i-1].s_r+ds;
  }
}

void global_path_ck(const nav_msgs::PathConstPtr &pt)
{
    global_path = *pt;
    std::vector<float> x,y,sita;
    for(int i=60;i<global_path.poses.size();i++)
    {
        x.push_back(global_path.poses[i].pose.position.x);
        y.push_back(global_path.poses[i].pose.position.y);
        sita.push_back(tf::getYaw(global_path.poses[i].pose.orientation));
    }
    getref(x,y,sita,global_path_reference);

}
void car_pose_ck(const nav_msgs::Odometry::ConstPtr& pt)
{
    car_p.car_center_x = pt->pose.pose.position.x;
    car_p.car_center_y = pt->pose.pose.position.y;
    car_p.car_acc =0;
    car_p.car_hesding = tf::getYaw(pt->pose.pose.orientation);
    car_p.car_length = 0.3;
    car_p.car_width = 0.15;
    car_p.car_velocity = pt->twist.twist.linear.x;
    cout<<"car_v:"<<car_p.car_velocity <<endl;
    float dsita = tf::getYaw(pt->pose.pose.orientation) - tf::getYaw(last_time_car_pose.pose.pose.orientation);
    if(abs(dsita)> abs(dsita + M_2_PI)) {dsita += M_2_PI;}
    else if(abs(dsita)>abs(dsita- M_2_PI))   {dsita -=M_2_PI;}
    //car_p.car_velocity
    car_p.car_k = abs(dsita)/
                  ( std::sqrt( pow((pt->pose.pose.position.x - last_time_car_pose.pose.pose.position.x),2)
                             + pow((pt->pose.pose.position.y - last_time_car_pose.pose.pose.position.y),2)    ) +1e-3  );
    
    last_time_car_pose = *pt;
}


int main(int argc, char **argv)
{
    int start_index=0;
    ros::init(argc,argv,"dynamic_planning_node");
    ros::NodeHandle n;
    std::string global_path_topic_name,car_pose_topic_name;

    n.param<std::string>("/dynamic_planning_node/global_path_topic_name",
                            global_path_topic_name,"global_path");

    n.param<std::string>("/dynamic_planning_node/car_pose_topic_name",
                            car_pose_topic_name,"odom");
    float w_smooth[3]={100000,5000,2000};
    float q_w_smooth[3]={1000,500,200};
    float dc=M_PI/360;
    Obs obs_p[2];
    Obs obs_d[1];
    Reference referenceline[1000];
    Reference dp_referenceline[60];
    for(int i=0;i<1;i++)
    {
        obs_p[i].obs_acc=0;
        obs_p[i].obs_center_x=2.58;
        obs_p[i].obs_center_y=5.17;
        obs_p[i].obs_hesding=0;
        obs_p[i].obs_k=0;
        obs_p[i].obs_length=0.05;
        obs_p[i].obs_velocity=0;
        obs_p[i].obs_width=0.05;
    }
        // obs_p[1].obs_acc=0;
        // obs_p[1].obs_center_x=2.54;
        // obs_p[1].obs_center_y=2.92;
        // obs_p[1].obs_hesding=0;
        // obs_p[1].obs_k=0;
        // obs_p[1].obs_length=0.1;
        // obs_p[1].obs_velocity=0;
        // obs_p[1].obs_width=0.1;



    ros::Subscriber ref_path_sub = n.subscribe(global_path_topic_name,10,global_path_ck);

    ros::Subscriber car_pose_sub = n.subscribe(car_pose_topic_name,10,car_pose_ck);
    
    ros::Publisher ref_path_pub = n.advertise<nav_msgs::Path>("path_dp",10);

    ros::Publisher obs_pub = n.advertise<visualization_msgs::Marker>("obs_show",10);

    ros::Publisher dp_obs_pub = n.advertise<std_msgs::Float64MultiArray>("dp_obs", 10); 

    ros::Publisher path_k = n.advertise<std_msgs::Float64MultiArray>("path_k", 10); 

    ros::Publisher ros_time = n.advertise<std_msgs::Float64MultiArray>("time", 10); 

    ros::Rate rate_loop(5);

vector<double>  pathk(61);
vector<double>  time(1);
int k1=0,k2=0;
bool obs_flag1=false;
bool obs_flag2=false;
auto time_pre=ros::Time::now();
double dt;
    while (ros::ok())
    {
        ros::spinOnce();
        visualization_msgs::Marker obs_marker;
        obs_marker.header.frame_id = "map";
        obs_marker.header.stamp = ros::Time::now();
        obs_marker.ns = "basic_shapes";
        obs_marker.id = 0;
        obs_marker.type = visualization_msgs::Marker::CUBE;
        obs_marker.action = visualization_msgs::Marker::ADD;
        obs_marker.pose.position.x = 2.58;
        obs_marker.pose.position.y = 5.17;
        obs_marker.scale.x = 0.05;
        obs_marker.scale.y = 0.05;
        obs_marker.scale.z = 0.1;
        obs_marker.color.r = 0.0f;
        obs_marker.color.g = 1.0f;
        obs_marker.color.b = 0.0f;
        obs_marker.color.a = 1.0;
        obs_marker.lifetime = ros::Duration();
        obs_pub.publish(obs_marker);

 


        if(global_path_reference.size()>0)
        {
            //std::cout << "path size"<<global_path_reference.size()<<std::endl;
            int length_point = GetLocalRef(start_index,car_p.car_center_x,car_p.car_center_y,4,global_path_reference,referenceline); 

            HostCar carp(car_p,referenceline,length_point);
            Obstacle obst(obs_p,1,referenceline,length_point);
            //std::cout << "length_point"<<length_point <<std::endl;
            DynaPlaning DP(obst,carp,0,0.1,0.1,100,500,w_smooth,50,500,q_w_smooth,2);
            DP.traj=traj;
            DP.DynamicProgramming();

            //std::cout<<"traj_num"<<DP.traj_num<<std::endl;
            nav_msgs::Path path;
            path.header.frame_id = "map";
            auto time_now = ros::Time::now();
            dt=time_now.toSec()-time_pre.toSec();
           // cout<<"dt:"<<dt<<endl;
            time_pre=time_now;
            std_msgs::Float64MultiArray msg_t;
            time[0]=ros::Time::now().toSec();
            msg_t.data = time;
            ros_time.publish(msg_t);


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
                
                dp_referenceline[i].x_r=DP.traj[i].x;
                dp_referenceline[i].y_r=DP.traj[i].y;
                dp_referenceline[i].heading_r=DP.traj[i].heading;
                dp_referenceline[i].k_r=DP.traj[i].k;
                reference_get_s(dp_referenceline);
                pathk[i]=DP.traj[i].k;

                //cout<<"i:"<<i<<' '<<"x:"<<pose.pose.position.x<<' '<<"y"<<pose.pose.position.y<<' '<<"t:"<<pose.header.stamp<<endl;
            }
             if(DP.traj[0].y>2.5)
                obs_flag1=true;
             if(DP.traj[0].x>3)
                obs_flag2=true;
             if(obs_life_time1>8)
                obs_flag1=false;
             if(obs_life_time2>12)
                obs_flag2=false;
            ref_path_pub.publish(path);

            std_msgs::Float64MultiArray msg_k;
            msg_k.data = pathk;
            path_k.publish(msg_k);
           
        }
        //obs1
       if(obs_flag1==true)
        {
        float obs_x= 1,
              obs_y= 4,
              obs_k= 0.07;
        obs_marker.id = 1;
        obs_marker.pose.position.x = obs_x+k1*obs_k;
        obs_marker.pose.position.y = obs_y;
        obs_marker.scale.x = 0.1;
        obs_marker.scale.y = 0.1;
        obs_marker.scale.z = 0.1;
        obs_marker.color.r = 0.0f;
        obs_marker.color.g = 1.0f;
        obs_marker.color.b = 0.0f;
        obs_marker.color.a = 1.0;
        obs_marker.lifetime = ros::Duration(0.5);
        obs_pub.publish(obs_marker);
        
        obs_d[0].obs_acc=0;
        obs_d[0].obs_center_x=obs_x+k1*obs_k;
        obs_d[0].obs_center_y=obs_y;
        obs_d[0].obs_hesding=0;
        obs_d[0].obs_k=0;
        obs_d[0].obs_length=0.05;
        obs_d[0].obs_velocity=0.07/dt;
        obs_d[0].obs_width=0.05;
        obs_life_time1=obs_life_time1+dt;
        Obstacle obsd(obs_d,1,dp_referenceline,60);
        obsd.CarteToFre();
        vector<double>  obs(7);
        obs[0]=obsd.obs_frenet->s;
        obs[1]=obsd.obs_frenet->sd;
        obs[2]=obsd.obs_frenet->sdd;
        obs[3]=obsd.obs_frenet->l;
        obs[4]=obsd.obs_frenet->ld+0.2;
        obs[5]=obsd.obs_frenet->ldd;
        obs[6]=1;
        std_msgs::Float64MultiArray msg;
        msg.data = obs;
        dp_obs_pub.publish(msg);
        k1++;
        }

        //obs2
        if(obs_flag2==true)
        {
        float obs_x= 5,
              obs_y= 4,
              obs_k= 0.02;
        obs_marker.id = 1;
        obs_marker.pose.position.x = obs_x;
        obs_marker.pose.position.y = obs_y-k2*obs_k;
        cout<<obs_marker.pose.position.x <<"  "<<obs_marker.pose.position.y<<endl;
        obs_marker.scale.x = 0.1;
        obs_marker.scale.y = 0.1;
        obs_marker.scale.z = 0.1;
        obs_marker.color.r = 0.0f;
        obs_marker.color.g = 1.0f;
        obs_marker.color.b = 0.0f;
        obs_marker.color.a = 1.0;
        obs_marker.lifetime = ros::Duration(0.5);
        obs_pub.publish(obs_marker);
        
        obs_d[0].obs_acc=0;
        obs_d[0].obs_center_x=obs_x;
        obs_d[0].obs_center_y=obs_y-k2*obs_k;
        obs_d[0].obs_hesding=-M_PI_2l;
        obs_d[0].obs_k=0;
        obs_d[0].obs_length=0.05;
        obs_d[0].obs_velocity=obs_k/dt;
        obs_d[0].obs_width=0.05;
        obs_life_time2=obs_life_time2+dt;
        Obstacle obsd(obs_d,1,dp_referenceline,60);
        obsd.CarteToFre();
        vector<double>  obs(7);
        obs[0]=obsd.obs_frenet->s;
        obs[1]=obsd.obs_frenet->sd;
        obs[2]=obsd.obs_frenet->sdd;
        obs[3]=obsd.obs_frenet->l;
        obs[4]=obsd.obs_frenet->ld;
        obs[5]=obsd.obs_frenet->ldd;
        obs[6]=1;
        std_msgs::Float64MultiArray msg;
        msg.data = obs;
        dp_obs_pub.publish(msg);
        k2++;
        }
        if(obs_flag1==false&&obs_flag2==false)
        {
        vector<double>  obs(7);
        obs[0]=0;
        obs[1]=0;
        obs[2]=0;
        obs[3]=0;
        obs[4]=0;
        obs[5]=0;
        obs[6]=0;
        
        std_msgs::Float64MultiArray msg;
        msg.data = obs;
        dp_obs_pub.publish(msg);


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