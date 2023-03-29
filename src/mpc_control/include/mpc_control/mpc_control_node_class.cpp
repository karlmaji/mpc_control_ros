#include "mpc_control_node_class.h"

using namespace mpc_control_node_class;

mpc_control_node::mpc_control_node(ros::NodeHandle* nodehandle):nh_(*nodehandle)
{
    ROS_INFO("----------mpc control node-----------");

    std::string path_ref_topic,current_pose_topic,current_odom_topic,cmd_vel_topic,
    path_real_topic,path_to_follow_topic,path_pred_topic;

    ros::NodeHandle n_private("~");
    n_private.param<std::string>("path_ref_topic",path_ref_topic,"path_ref");
    n_private.param<std::string>("current_pose_topic",current_pose_topic,"nokov");
    n_private.param<std::string>("current_odom_topic",current_odom_topic,"odom");
    n_private.param<std::string>("cmd_vel_topic",cmd_vel_topic,"cmd_vel");
    n_private.param<std::string>("path_real_topic",path_real_topic,"path_real");
    n_private.param<std::string>("path_to_follow_topic",path_to_follow_topic,"path_to_follow");
    n_private.param<std::string>("path_pred_topic",path_pred_topic,"path_pred");
    

    this->ref_path_sub = nh_.subscribe(path_ref_topic,10,&mpc_control_node::ref_path_callbk,this);
    this->pose_sub = nh_.subscribe(current_pose_topic,10,&mpc_control_node::current_pose_callbk,this);
    this->odom_sub = nh_.subscribe(current_odom_topic,10,&mpc_control_node::current_odom_callbk,this);
    this->cmd_vel_pub = nh_.advertise<geometry_msgs::Twist>(cmd_vel_topic,10);
    this->path_real_pub = nh_.advertise<nav_msgs::Path>(path_real_topic,10);
    this->path_follow_pub = nh_.advertise<nav_msgs::Path>(path_to_follow_topic,10);
    this->path_pred_pub = nh_.advertise<nav_msgs::Path>(path_pred_topic,10);



    std::string node_name = ros::this_node::getName();
    this->init(node_name);


    this->control_frequency = 1/this->_delta_T;
    std::cout<< control_frequency<<std::endl;
}

void mpc_control_node::run_step()
{
    ROS_INFO("-----------------RUN--STEP--------------");
    double sita_before;
    Eigen::Matrix<double,3,1> x0;
    Eigen::Matrix<double,2,1> u0;
    std::vector<Eigen::Matrix<double,3,1>> xref;
    std::vector<Eigen::Matrix<double,2,1>> uref;
    geometry_msgs::Twist cmd_vel;

    ros::spinOnce();
    if(ref_path.poses.size()>0)
    {
        int count = find_index_for_nowtime(ref_path);

        if(count!=-1 && count< ref_path.poses.size()-this->_mpc_window-1)
        {
            double x_now=current_pose.pose.position.x;
            double y_now=current_pose.pose.position.y;
            double sita_now = tf::getYaw(current_pose.pose.orientation);
            double v_now =current_odom.twist.twist.linear.x;
            double sita_a_now = current_odom.twist.twist.angular.z;
            

            nav_msgs::Path path_to_follow;
            path_to_follow.header.frame_id="map";
            path_to_follow.poses.clear();

            nav_msgs::Path path_pred;
            path_pred.header.frame_id="map";
            path_pred.poses.clear();
            
            for(int i=0;i<this->_mpc_window+1;i++)
            {
                Eigen::Matrix<double,3,1> x_iter;
                double x=ref_path.poses.at(count+i).pose.position.x;
                double y=ref_path.poses.at(count+i).pose.position.y;
                double sita = tf::getYaw(ref_path.poses.at(count+i).pose.orientation) - sita_now;

                geometry_msgs::PoseStamped pose_stamp;
                pose_stamp = ref_path.poses.at(count+i);
                path_to_follow.poses.push_back(pose_stamp);

                if(i==0)
                {
                    normalize_sita(0,sita);
                    sita_before = sita;
                }
                else
                {
                    normalize_sita(sita_before,sita);

                    sita_before = sita;

                }
                x_iter<<x - x_now,y - y_now,sita; 
                
                xref.push_back(x_iter);
            }

            for(int i=0;i<this->_mpc_window;i++)
            {
                Eigen::Matrix<double,2,1> u_iter;
                u_iter<<0.0,0;
                uref.push_back(u_iter);
            }

            
            
            

            x0 << x_now,y_now,sita_now;
            u0 << v_now,sita_a_now;
            
            if(!this->step(x0,u0,xref,uref))
            {

                double v_ = this->QPctr(0,0);
                double sita_ = this->QPctr(1,0);

                if(v_>5 || v_<-5) v_=0;
                if(sita_>5 || sita_ <-5) sita_ = 0;

                cmd_vel.linear.x = v_;
                cmd_vel.angular.z = sita_;
                cmd_vel_pub.publish(cmd_vel);

                std::cout<< this->QPctr<<std::endl;

                for(int i=0; i<this->_mpc_window +1 ;i++)
                {
                    geometry_msgs::PoseStamped this_pose_stamped;
                    this_pose_stamped.pose.position.x = this->QPSolution(3*i,0) + x_now;
                    this_pose_stamped.pose.position.y = this->QPSolution(3*i+1,0) + y_now;
                    path_pred.poses.push_back(this_pose_stamped);
                
                }
            

            }
            else{
                ROS_INFO("ERROR! SOVER Faild!");
            }



            xref.clear();
            path_real_pub.publish(record_path);
            path_follow_pub.publish(path_to_follow);
            path_pred_pub.publish(path_pred);


        }

            
        else
        {
            cmd_vel.linear.x = 0;
            cmd_vel.angular.z = 0;
            cmd_vel_pub.publish(cmd_vel);
        }
    
    }



}




void mpc_control_node::ref_path_callbk(const nav_msgs::Path::ConstPtr& pt)
{
    this->ref_path = *pt;
}

void mpc_control_node::current_pose_callbk(const geometry_msgs::PoseStamped::ConstPtr& pt)
{
    geometry_msgs::PoseStamped pose_stamped;
    this->current_pose = *pt;
    pose_stamped.header = pt->header;
    pose_stamped.pose = pt->pose;
    this->record_path.poses.push_back(pose_stamped);
    record_path.header.frame_id = "map";
}


void mpc_control_node::current_odom_callbk(const nav_msgs::Odometry::ConstPtr& pt)
{
    
    this->current_odom = *pt;
}

void mpc_control_node::normalize_sita(const double &sita_before,double &sita_after)
{
    int k = (int)(sita_before / (2.f*M_PI));
    double k_plus = std::abs(sita_after - sita_before + (k + 1)*2.f*M_PI);
    double k_minu = std::abs(sita_after - sita_before + (k - 1)*2.f*M_PI);
    double k_none = std::abs(sita_after - sita_before);

    if(k_plus<k_minu && k_plus<k_none)sita_after +=(k+1)*2.f*M_PI;
    if(k_minu<k_plus && k_minu<k_none)sita_after +=(k-1)*2.f*M_PI;
    
}

int mpc_control_node::find_index_for_nowtime(const nav_msgs::Path & follow_path,const int begin_index)
{
    double t_now = ros::Time::now().toSec();
    for(int i=begin_index;i<follow_path.poses.size();i++)
    {
        double t_point = follow_path.poses[i].header.stamp.toSec();
        if((t_point - t_now)<0) continue;
        else return i;
    }
    return -1;

}


int main(int argc, char **argv)
{
    ros::init(argc,argv,"mpc_control_node_class");
    ros::NodeHandle n;
    mpc_control_node mpc(&n);
    ros::Time::init();
    

    ros::Rate loop_rate(mpc.control_frequency);

    while (ros::ok())
    {
        mpc.run_step();
        loop_rate.sleep();
    }
    return 0;
    
}




