#include"main.h"
void ST_Planner(VectorXd trajectory_x_pre,VectorXd trajectory_y_pre,VectorXd trajectory_heading_pre,
                VectorXd trajectory_kappa_pre,VectorXd trajectory_speed_pre, VectorXd trajectory_accel_pre,VectorXd trajectory_time_pre,
                VectorXd trajectory_x_init,VectorXd trajectory_y_init,VectorXd trajectory_heading_init,VectorXd  trajectory_kappa_init,
				double current_time,float vx,float vy,float ax,float ay,float heading,
				VectorXd dynamic_obs_s_set,VectorXd dynamic_obs_l_set,VectorXd dynamic_obs_s_dot_set,VectorXd dynamic_obs_l_dot_set,
				VectorXd& trajectory_x_out, VectorXd& trajectory_y_out,VectorXd& trajectory_heading_out,VectorXd& trajectory_kappa_out,
				VectorXd& trajectory_speed_out,VectorXd& trajectory_accel_out,VectorXd& trajectory_time_out,double* timeuse)

{
    timeval starttime,endtime;   
		// cout<<"init"<<endl;
		// cout<<"x:"<<endl<<trajectory_x_init<<endl<<"y:"<<endl<<trajectory_y_init<<endl;
	gettimeofday(&starttime,0);
// VectorXd trajectory_x_init(20),trajectory_y_init(20), trajectory_heading_init(20), trajectory_kappa_init(20);
// 	for (int i = 0; i < 20; i++)
// 	{
// 		trajectory_x_init(i) = 2*(i-1);
// 		trajectory_y_init(i) = 0;
// 		trajectory_heading_init(i) = 0;
// 		trajectory_kappa_init(i) = 0;
// 	}
    //计算path_index2s
	VectorXd path_index2s;
	int path_s_end;
	path_s_end=index2s(trajectory_x_init, trajectory_y_init,path_index2s);
	//cout<<"path_index2s"<<path_index2s<<endl<<endl;
	//计算速度规划初始条件
	float out[2];
	planning_start_condition(vx, vy, ax, ay, heading, out);//vx vy ax ay heading
	float plan_start_s_dot = out[0];
	float plan_start_s_dot2 =out[1];
	//cout << out[0] << out[1];
	//生成st图

	// VectorXd dynamic_obs_s_set(1);
	// dynamic_obs_s_set(0) = 8;
	// VectorXd dynamic_obs_l_set(1);
	// dynamic_obs_l_set(0) = 4;
	// VectorXd dynamic_obs_s_dot_set(1);
	// dynamic_obs_s_dot_set(0) = -0.2;
	// VectorXd dynamic_obs_l_dot_set(1);
	// dynamic_obs_l_dot_set(0) = -1;

	//输出初始化
	int n = dynamic_obs_s_set.size();
	VectorXd obs_st_s_in_set(n);
	VectorXd obs_st_s_out_set(n);
	VectorXd obs_st_t_in_set(n);
	VectorXd obs_st_t_out_set(n);

	obs_st_s_in_set.fill(NAN);
	obs_st_s_out_set.fill(NAN);
	obs_st_t_in_set.fill(NAN);
	obs_st_t_out_set.fill(NAN);

	
	generate_st_graph(obs_st_s_in_set, obs_st_s_out_set, obs_st_t_in_set, obs_st_t_out_set,
		dynamic_obs_s_set, dynamic_obs_l_set, dynamic_obs_s_dot_set, dynamic_obs_l_dot_set);

	//和path_s一样放大7倍
	int obs_k=7;
	obs_st_s_in_set(0)=obs_st_s_in_set(0)*obs_k;
	obs_st_s_out_set(0)=obs_st_s_out_set(0)*obs_k;
	//障碍物输出
	// VectorXd obs_s(4);
	// obs_s << obs_st_s_in_set, obs_st_s_out_set, obs_st_t_in_set, obs_st_t_out_set;
	
	//1   1.8   0   2

	//  if(isnan(obs_st_s_in_set(0))==false)
	//  cout <<"obs"<<endl<<"obs_s_in:"<<obs_st_s_in_set << endl <<"obs_s_out:"<< obs_st_s_out_set << endl <<"obs_t_in:"<< obs_st_t_in_set << endl << "obs_t_out:"<<obs_st_t_out_set << endl<<endl;
	// // //速度动态规划
	float reference_speed_unlimit =0.4*7,
		w_cost_ref_speed = 4000,
		w_cost_accel =0.01,
		w_cost_obs = 10000000;
	VectorXd dp_speed_s, dp_speed_t;
	
	dynamic_planning(obs_st_s_in_set,obs_st_s_out_set,
		obs_st_t_in_set, obs_st_t_out_set,reference_speed_unlimit,
		w_cost_ref_speed,w_cost_accel,  w_cost_obs, plan_start_s_dot,
		dp_speed_s, dp_speed_t);


	// if(isnan(obs_st_s_in_set(0))==false)
	//cout <<"dp_s:"<<endl<< dp_speed_s << endl;//"dp_speed_t:"<<endl<<dp_speed_t<<endl 
	//cout<<"start:"<<endl<<plan_start_s_dot<<" "<<plan_start_s_dot2<<endl<<endl;


	//生成凸空间
	VectorXd s_lb, s_ub, s_dot_lb, s_dot_ub;
	float max_lateral_accel = 0.2*9.8;
	generate_convex_space(dp_speed_s,dp_speed_t,path_index2s,  obs_st_s_in_set, obs_st_s_out_set,
		obs_st_t_in_set, obs_st_t_out_set,trajectory_kappa_init, max_lateral_accel, s_lb,
		 s_ub,  s_dot_lb, s_dot_ub);
	//cout <<"constrait"<<endl<< s_lb << endl<<endl << s_ub <<endl<<endl << s_dot_lb<<endl <<endl<< s_dot_ub<<endl<<endl;
	//for (int i = 0; i < 16; i++)
	//{
	//	s_dot_ub(i) = 4;


	//}

//二次规划
	float w_cost_s_dot2 = 0.01,
		w_cost_v_ref = 50000,
		w_cost_jerk = 0.01,
		speed_reference = dp_speed_s(0)*2;
		plan_start_s_dot=speed_reference;

	VectorXd  qp_s_init, qp_s_dot_init, qp_s_dot2_init, relative_time_init;

	quadratic_programming(plan_start_s_dot, plan_start_s_dot2, dp_speed_s, dp_speed_t,
		s_lb, s_ub, s_dot_lb, s_dot_ub, w_cost_s_dot2, w_cost_v_ref, w_cost_jerk, speed_reference,
		qp_s_init, qp_s_dot_init, qp_s_dot2_init, relative_time_init);
	//cout <<"qp"<<endl<<"s:" <<qp_s_init << endl << endl;//<<"t:"<<relative_time_init <<endl;
	//cout << qp_s_dot_init << endl << endl;

	int k=path_index2s.size()-1;
	for(int i=0;i<qp_s_init.size();i++)
	{
		if(isnan(qp_s_init(qp_s_init.size()-1))==false)
		{
			if(path_index2s(k)!=0)
			1;
			else
			{
			for(;k>0;k--)
			if(path_index2s(k)==0)
				break;
			}
			for(int j=0;j<qp_s_init.size();j++)
				if(qp_s_init(j)>path_index2s(k))
				{
					qp_s_init(j)=NAN;
					qp_s_dot_init(j)=NAN;
					qp_s_dot2_init(j)=NAN;
					relative_time_init(j)=NAN;
				}
		break;
		}
		if(isnan(qp_s_init(i))==1)
		{
			if(path_index2s(k)!=0)
			1;
			else
			{
			for(;k>0;k--)
			if(path_index2s(k)==0)
				break;
			}
			for(int j=0;j<i;j++)
				if(qp_s_init(j)>path_index2s(k))
				{
					qp_s_init(j)=NAN;
					qp_s_dot_init(j)=NAN;
					qp_s_dot2_init(j)=NAN;
					relative_time_init(j)=NAN;
				}
		break;
		}
	}
	//cout <<"qp"<<endl<< qp_s_init << endl <<endl;
	// cout << qp_s_dot_init << endl << endl;
	// cout << qp_s_dot2_init << endl << endl;
	//cout << relative_time_init << endl << endl;
	

	//轨迹增密
	VectorXd  s, s_dot,s_dot2, relative_time;
	increase_st_point_count(qp_s_init, qp_s_dot_init, qp_s_dot2_init, relative_time_init,
		s, s_dot, s_dot2, relative_time);
	
	//cout << "s:"<<endl<<s << endl << endl;
	//cout << s_dot << endl << endl;
	//cout << s_dot2 << endl << endl;
	//cout << "time: "<<relative_time << endl << endl;
	//合并速度和轨迹
	VectorXd trajectory_x, trajectory_y, trajectory_heading, trajectory_kappa,trajectory_speed, trajectory_accel, trajectory_time;

	path_and_speed_merge(s, s_dot, s_dot2, relative_time, current_time, path_index2s,
		trajectory_x_init, trajectory_y_init, trajectory_heading_init, trajectory_kappa_init,
		trajectory_x, trajectory_y, trajectory_heading, trajectory_kappa,
		trajectory_speed, trajectory_accel, trajectory_time);
	//cout<<"x:"<<endl<<trajectory_x<<endl;
	//cout<<"time:"<<endl<<trajectory_time<<endl;
	//cout<<"time_now:"<<current_time<<endl;
    //轨迹拼接
//     VectorXd trajectory_x_final, trajectory_y_final, trajectory_heading_final, trajectory_kappa_final,trajectory_speed_final, trajectory_accel_final, trajectory_time_final;
//     stitch(trajectory_x,trajectory_y,trajectory_heading,trajectory_kappa,trajectory_speed, 
// trajectory_accel,trajectory_time,trajectory_x_pre,trajectory_y_pre,trajectory_heading_pre,
// trajectory_kappa_pre,trajectory_speed_pre, trajectory_accel_pre,trajectory_time_pre,current_time,
// trajectory_x_final,trajectory_y_final,trajectory_heading_final,trajectory_kappa_final,
// trajectory_speed_final, trajectory_accel_final,trajectory_time_final);

	//时间插值
VectorXd trajectory_x_final,trajectory_y_final,trajectory_heading_final,trajectory_kappa_final,trajectory_speed_final,trajectory_accel_final,trajectory_time_final;


double time_start=trajectory_time(0);
double dt=0.05;
double time_len=1;
while(time_start+time_len*dt<=trajectory_time(trajectory_time.size()-1))
	time_len++;
trajectory_time_final.resize(time_len);
trajectory_x_final.resize(time_len);
trajectory_y_final.resize(time_len);
trajectory_heading_final.resize(time_len);
trajectory_kappa_final.resize(time_len);
trajectory_speed_final.resize(time_len);
trajectory_accel_final.resize(time_len);
for(int i=0;i<time_len;i++)
	{
		trajectory_time_final(i)=time_start+i*dt;
		trajectory_x_final(i) = interp_time(trajectory_time, trajectory_x, trajectory_x.size()-1, trajectory_time_final(i));
		trajectory_y_final(i) = interp_time(trajectory_time, trajectory_y, trajectory_y.size()-1, trajectory_time_final(i));
		trajectory_heading_final(i) = interp_heading(trajectory_time, trajectory_heading, trajectory_heading.size()-1, trajectory_time_final(i));
	}

//cout<<"time: "<<endl<<trajectory_time<<endl<<endl<<trajectory_time_final<<endl;
//cout<<"y: "<<endl<<trajectory_x_final<<endl;





// VectorXd trajectory_x_final(30),trajectory_y_final(30),trajectory_heading_final(30),trajectory_kappa_final(30),trajectory_speed_final(30),
//          trajectory_accel_final(30),trajectory_time_final(30);
// for(int i=0;i<60;i++)
// {
// if(i%2==0)
// {
// trajectory_x_final(i/2)=trajectory_x(i);
// trajectory_y_final(i/2)=trajectory_y(i);
// trajectory_heading_final(i/2)=trajectory_heading(i);
// trajectory_kappa_final(i/2)=trajectory_kappa(i);
// trajectory_speed_final(i/2)=trajectory_speed(i);
// trajectory_accel_final(i/2)=trajectory_accel(i);
// trajectory_time_final(i/2)=trajectory_time(i);
// }
// }






// trajectory_x_out=trajectory_x;
// trajectory_y_out=trajectory_y;
// trajectory_heading_out=trajectory_heading;
// trajectory_kappa_out=trajectory_kappa;
// trajectory_speed_out=trajectory_speed;
// trajectory_accel_out=trajectory_accel;
// trajectory_time_out=trajectory_time;

trajectory_x_out=trajectory_x_final;
trajectory_y_out=trajectory_y_final;
trajectory_heading_out=trajectory_heading_final;
trajectory_kappa_out=trajectory_kappa_final;
trajectory_speed_out=trajectory_speed_final;
trajectory_accel_out=trajectory_accel_final;
trajectory_time_out=trajectory_time_final;
//if(trajectory_x(60)<-1000)
//cout<<"qp:"<<endl<<qp_s_dot_init<<endl<<"path_s"<<endl<<path_index2s<<endl<<"x:"<<trajectory_x;
gettimeofday(&endtime,0);
*timeuse  = 1000000*(endtime.tv_sec - starttime.tv_sec) + endtime.tv_usec - starttime.tv_usec;
//cout<<"Linux DBSCAN time: "<<timeuse<<" ms"<<endl; 
}