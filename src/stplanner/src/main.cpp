#include "main.h"
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/PoseStamped.h>
nav_msgs::Path local_path;
nav_msgs::Odometry last_time_car_pose;
VectorXd trajectory_x_pre(61),trajectory_y_pre(61),trajectory_heading_pre(61),trajectory_kappa_pre(61),trajectory_speed_pre(61),
         trajectory_accel_pre(61),trajectory_time_pre(61);

VectorXd trajectory_x_init(61),trajectory_y_init(61),
		trajectory_heading_init(61), trajectory_kappa_init(61),
		dynamic_obs_s_set(1),dynamic_obs_l_set(1),dynamic_obs_s_dot_set(1),dynamic_obs_l_dot_set(1),
		trajectory_x, trajectory_y, trajectory_heading, trajectory_kappa,trajectory_speed, trajectory_accel, trajectory_time;
float vx,vy,ax,ay,heading;

double current_time,timeuse;
int num=0;
ros::Time t_init[61];



void dp_path_ck(const nav_msgs::PathConstPtr &pt)
{
	local_path = *pt;
	//ST_Planner();
	int n=local_path.poses.size();
	trajectory_x_init.resize(n);
	trajectory_y_init.resize(n);
	trajectory_heading_init.resize(n);
	trajectory_kappa_init.resize(n);
	//cout<<"dp_path"<<n<<endl;
	for(int i=0;i<n;i++)
	{
		trajectory_x_init(i)=local_path.poses[i].pose.position.x;
		trajectory_y_init(i)=local_path.poses[i].pose.position.y;
		trajectory_heading_init(i)=tf::getYaw(local_path.poses[i].pose.orientation); 
		t_init[i]=local_path.poses[i].header.stamp;
		//if(num==0)
		//cout<<"x:"<<trajectory_x_init(i)<<"y:"<<trajectory_y_init(i)<<"t:"<<local_path.poses[i].header.stamp.toSec()<<endl ;
	}//num++;
	//cout<<"sita:"<<endl<<trajectory_heading_init<<endl;

}

void path_k_ck(const std_msgs::Float64MultiArray& k) 
{
	for(int i=0;i<61;i++)
		trajectory_kappa_init(i)=k.data.at(i);

	//cout<<"k"<<trajectory_kappa_init(0)<<endl;
}

void car_pose_ck(const nav_msgs::Odometry::ConstPtr& v_pt,const geometry_msgs::PoseStamped::ConstPtr& pose_pt)
{
    heading = tf::getYaw(pose_pt->pose.orientation);
	vx = v_pt->twist.twist.linear.x * cos(heading);
    vy = v_pt->twist.twist.linear.x * sin(heading);
	    cout<<"car_v:"<<v_pt->twist.twist.linear.x<<endl;
    //cout<<"heading:"<<heading<<endl<<endl;
	//cout<<"V:"<<vx<<" "<<vy<<endl<<endl;
}

void obs_ck(const std_msgs::Float64MultiArray& obs)
{
	if(obs.data.at(6)==0)
	{
	dynamic_obs_s_set(0)=NAN;
	dynamic_obs_s_dot_set(0)=NAN;
	dynamic_obs_l_set(0)=NAN;
	dynamic_obs_l_dot_set(0)=NAN;
	}
	else
	{
	dynamic_obs_s_set(0)=obs.data.at(0);
	dynamic_obs_s_dot_set(0)=obs.data.at(1);
	dynamic_obs_l_set(0)=obs.data.at(3);
	dynamic_obs_l_dot_set(0)=(obs.data.at(4));
	}
	cout<<"obs:"<<endl<<"obs_s: "<<dynamic_obs_s_set<<endl<<"obs_l: "<<dynamic_obs_l_set<<endl<<"obs_s_dot: "<<dynamic_obs_s_dot_set<<endl<<"obs_l_dot: "<<dynamic_obs_l_dot_set<<endl;
}
void time_ck(const std_msgs::Float64MultiArray& time)
{
	//current_time=time.data.at(0);
	
	//cout<<"current_time:"<<current_time<<endl;
}
int main(int argc, char **argv)
{
	
	trajectory_x_init.fill(0);
	trajectory_x_pre.fill(NULL);
	// trajectory_y_init.resize(20);
	// trajectory_heading_init.resize(20);
	// trajectory_kappa_init.resize(20);
	// for (int i = 0; i < 60; i++)
	// {
	// 	trajectory_x_init(i) = 2*(i-1);
	// 	trajectory_y_init(i) = 0;
	// 	trajectory_heading_init(i) = 0;
	// 	trajectory_kappa_init(i) = 0;
	// }
	// dynamic_obs_s_set.resize(1);
	// dynamic_obs_s_set(0) = 8;
	// dynamic_obs_l_set.resize(1);
	// dynamic_obs_l_set(0) = 4;
	// dynamic_obs_s_dot_set.resize(1);
	// dynamic_obs_s_dot_set(0) = -0.2;
	// dynamic_obs_l_dot_set.resize(1);
	// dynamic_obs_l_dot_set(0) = -1;
	// vx=1;vy=0;ax=0;ay=0;heading=0;

  ros::init(argc, argv, "speed_planning_node");
  ros::Time::init();
  ros::NodeHandle n;
  message_filters::Subscriber<nav_msgs::Odometry> v_sub(n, "robot_1/odom", 1);
  message_filters::Subscriber<geometry_msgs::PoseStamped> pose_sub(n, "robot_1/nokov", 1);
  typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, geometry_msgs::PoseStamped> MySyncPolicy;
  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10),v_sub,pose_sub);
  sync.registerCallback(boost::bind(&car_pose_ck,_1,_2));

  
  ros::Subscriber speed_path_sub = n.subscribe("path_dp",10,dp_path_ck);
  ros::Subscriber path_k_sub = n.subscribe("path_k",10,path_k_ck);
  ros::Subscriber obs_sub = n.subscribe("dp_obs", 10,obs_ck); 
  //ros::Subscriber time_sub = n.subscribe("time", 10,time_ck); 
  ros::Publisher speed_path_pub = n.advertise<nav_msgs::Path>("path_ref",10);

  ros::Rate rate_loop(5);
  
 

  ax=0;ay=0;
auto time_1= ros::Time::now();
int k=0;

  while (ros::ok())
    {
     	ros::spinOnce();
	// cout<<"init"<<endl<<"x:"<<trajectory_x_init<<endl<<"y:"<<trajectory_y_init<<endl;
		 current_time = ros::Time::now().toSec();
		if(trajectory_x_init(0)==0&&trajectory_x_init(60)==0)
		{1;}
		else
		{
			//current_time=t_init[0].toSec();
        ST_Planner(trajectory_x_pre,trajectory_y_pre,trajectory_heading_pre,trajectory_kappa_pre, trajectory_speed_pre,
					trajectory_accel_pre,trajectory_time_pre,
					trajectory_x_init,trajectory_y_init,trajectory_heading_init, trajectory_kappa_init,current_time,vx,vy,ax,ay,heading,
					dynamic_obs_s_set,dynamic_obs_l_set,dynamic_obs_s_dot_set,dynamic_obs_l_dot_set,
					trajectory_x, trajectory_y, trajectory_heading, trajectory_kappa,trajectory_speed, trajectory_accel, trajectory_time,&timeuse);


		// cout<<"out"<<endl;
		// for(int i=0;i<trajectory_x.size();i++)	
		// cout<<"x_init:"<<trajectory_x_init(i)<<" "<<"y_init:"<<trajectory_y_init(i)<<" "<<"t_init:"<<t_init[i]<<" "<<
		//       "x_out:"<<trajectory_x(i)<<" "<<"y_out:"<<trajectory_y(i)<<" "<<"t_out:"<<ros::Time().fromSec(trajectory_time(i))<<endl ;
		
		/*
		cout<<"init"<<endl;
		for(int i=0;i<trajectory_x_init.size();i++)	
		cout<<"x_init:"<<trajectory_x_init(i)<<" "<<"y_init:"<<trajectory_y_init(i)<<" "<<"h_init:"<<trajectory_heading_init[i]<<endl;
		cout<<"out"<<endl;
		for(int i=0;i<trajectory_x.size();i++)
		cout<<"x_out:"<<trajectory_x(i)<<" "<<"y_out:"<<trajectory_y(i)<<"h_out:"<<trajectory_heading(i)<<endl ;
		*/
		// for(int i=0;i<trajectory_x.size();i++)
		// cout<<"x_out:"<<trajectory_x(i)<<" "<<"y_out:"<<trajectory_y(i)<<"h_out:"<<trajectory_heading(i)<<endl ;
	
		// cout<<"init"<<endl;
		// cout<<"x:"<<trajectory_x_init<<endl<<"y:"<<trajectory_y_init<<endl;
		// cout<<"out"<<endl;
		// cout<<"x_out:"<<trajectory_x<<endl<<"y_out:"<<trajectory_y<<endl;
// cout<<"h_out:"<<endl ;
// 	for(int i=0;i<trajectory_x.size();i++)
// 		cout<<trajectory_heading(i)<<endl ;



		timeuse /=1000000;//s
		nav_msgs::Path path;
		path.header.frame_id = "map";
		if(trajectory_x(0)>1000)
		{
			cout<<"fail osqp"<<endl;
			//speed_path_pub.publish(local_path);
		}
		else if(trajectory_x_init(0)!=0)
		{
		for(int i = 0;i<trajectory_x.size();i++)
		{
			geometry_msgs::PoseStamped pose;
			pose.header.frame_id = "map";
			pose.header.stamp = ros::Time().fromSec(trajectory_time(i));//ros::Time().fromSec(current_time)+
			pose.pose.position.x = trajectory_x(i);
			pose.pose.position.y = trajectory_y(i);
			//cout<<"i:"<<i<<' '<<"x:"<<trajectory_x(i)<<' '<<"y"<<trajectory_y(i)<<' '<<"t:"<<pose.header.stamp<<endl;
			tf::Quaternion q = tf::createQuaternionFromRPY(0,0,trajectory_heading(i));
			pose.pose.orientation.x = q.getX();
			pose.pose.orientation.y = q.getY();
			pose.pose.orientation.z = q.getZ();
			pose.pose.orientation.w = q.getW();
			path.poses.push_back(pose);
		}
		speed_path_pub.publish(path);
		}
		}

		rate_loop.sleep();
	}




	return 0;





}