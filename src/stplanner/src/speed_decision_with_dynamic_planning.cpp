#include "main.h"
float CalcCollisionCost(float w_cost_obs, float min_dis)
{
	//锟斤拷锟斤拷锟较帮拷锟斤拷锟斤拷锟斤拷锟斤拷
	float collision_cost;
	if (abs(min_dis) < 0.5)
		collision_cost = w_cost_obs;
	else if (abs(min_dis) > 0.5 && abs(min_dis) < 1.5)
		// min_dis = 0.5    collision_cost = w_cost_obs ^ 1;
		// min_dis = 1.5    collision_cost = w_cost_obs ^ 0 = 1
		collision_cost = pow(w_cost_obs, ((0.5 - min_dis) + 1));
	else
		collision_cost = 0;
	return collision_cost;

}
float CalcObsCost(float s_start, float t_start, float s_end, float t_end, VectorXd obs_st_s_in_set, VectorXd obs_st_s_out_set,
	VectorXd obs_st_t_in_set, VectorXd obs_st_t_out_set, float w_cost_obs)
{
	//锟矫猴拷锟斤拷锟斤拷锟斤拷锟斤拷叩锟斤拷习锟斤拷锟斤拷锟斤拷
	//锟斤拷锟诫：锟竭碉拷锟斤拷锟斤拷盏锟絪_start, t_start, s_end, t_end
	//锟较帮拷锟斤拷锟斤拷息 obs_st_s_in_set, obs_st_s_out_set, obs_st_t_in_set, obs_st_t_out_set
	//锟较帮拷锟斤拷锟斤拷锟饺拷锟絯_cost_obs
	//锟斤拷锟斤拷锟斤拷叩锟斤拷习锟斤拷锟斤拷锟斤拷obs_cost

//锟斤拷锟斤拷锟绞硷拷锟�
	float obs_cost = 0;
	//锟斤拷锟斤拷锟斤拷母锟斤拷锟�
	int n = 5;
	//锟斤拷锟斤拷时锟斤拷锟斤拷
	float dt = (t_end - t_start) / (n - 1);
	//锟竭碉拷斜锟斤拷
	float k = (s_end - s_start) / (t_end - t_start);
	//锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷

	for (int i = 1; i <= n; i++)
	{
		//锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷
		float t = t_start + (i - 1) * dt;
		float s = s_start + k * (i - 1) * dt;
		//锟斤拷锟斤拷锟斤拷锟斤拷锟较帮拷锟斤拷
		if(obs_st_s_in_set(0)!=NULL)
		{
		for (int j = 0; j < obs_st_s_in_set.size(); j++)
		{
			if (isnan(obs_st_s_in_set(j)))
				continue;
			//锟斤拷锟斤拷锟较帮拷锟斤拷牡愕絪t锟竭段碉拷锟斤拷叹锟斤拷锟�
			Vector2d obs_in(obs_st_s_in_set(j), obs_st_t_in_set(j));
			Vector2d obs_out(obs_st_s_out_set(j), obs_st_t_out_set(j));
			Vector2d st(s, t);
			Vector2d vector1 = obs_in - st;
			Vector2d vector2 = obs_out - st;
			Vector2d vector3 = vector2 - vector1;
			float min_dis = 0;
			float dis1 = sqrt(vector1.transpose()*vector1);
			float dis2 = sqrt(vector2.transpose()*vector2);
			float dis3 = abs(vector1(0)*vector3(1) - vector1(1)*vector3(0)) / sqrt(vector3.transpose()*vector3);
			if ((vector1.transpose()*vector3 > 0 && vector2.transpose()*vector3 > 0) || (vector1.transpose()*vector3 < 0 && vector2.transpose()*vector3 < 0))
				min_dis = min(dis1, dis2);
			else
				min_dis = dis3;
			//加入避障硬约束
			if (t< obs_st_t_out_set(j) && s>(((obs_st_s_out_set(j)-obs_st_s_in_set(j))*(t- obs_st_t_out_set(j))/(obs_st_t_out_set(j)- obs_st_t_in_set(j)))+ obs_st_s_out_set(j)))
				min_dis = 0;
			//
			obs_cost = obs_cost + CalcCollisionCost(w_cost_obs, min_dis);
		}
		}
	}
	return obs_cost;
}




void CalcSTCoordinate(int row, int col, VectorXd s_list, VectorXd t_list, float out[2])
{
	//  锟矫猴拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟节碉拷锟絪 t 锟斤拷锟斤拷
	//	锟斤拷锟斤拷 row col 锟节碉拷锟节撅拷锟斤拷锟斤拷泻藕锟斤拷泻锟�
	//	s_list t_list 锟斤拷锟斤拷锟斤拷锟斤拷锟�
	//	锟斤拷锟斤拷锟絪_value t_value 锟节碉拷锟� s t锟斤拷锟斤拷
	//	锟斤拷锟斤拷锟�(1, 1) 锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟较角碉拷元锟斤拷 s 锟斤拷锟� t 锟斤拷小
	//  锟斤拷锟斤拷取锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟�
	int m = s_list.size();
	//  1 1 锟斤拷应锟斤拷锟斤拷 s 锟斤拷锟斤拷锟街� t 锟斤拷锟斤拷小值
	float s_value = s_list(m - row);
	float t_value = t_list(col - 1);
	out[0] = s_value; out[1] = t_value;
}
float CalcDpCost(int row_start, int col_start, int row_end, int col_end, VectorXd obs_st_s_in_set, VectorXd obs_st_s_out_set,
	VectorXd obs_st_t_in_set, VectorXd obs_st_t_out_set, float w_cost_ref_speed, float reference_speed, float w_cost_accel,
	float w_cost_obs, float plan_start_s_dot, VectorXd s_list, VectorXd t_list, MatrixXf dp_st_s_dot)
{

	//锟矫猴拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟节碉拷之锟斤拷叩拇锟斤拷锟�
	//	锟斤拷锟诫：锟竭碉拷锟斤拷锟斤拷锟斤拷锟叫猴拷row_start, col_start 锟竭碉拷锟秸碉拷锟斤拷锟叫猴拷row_end, col_end
	//	锟较帮拷锟斤拷st锟斤拷息obs_st_s_in_set, obs_st_s_out_set, obs_st_t_in_set, obs_st_t_out_set
	//	锟狡硷拷锟劫度达拷锟斤拷权锟斤拷w_cost_ref_speed, 锟斤拷锟劫度达拷锟斤拷权锟斤拷w_cost_accel, 锟较帮拷锟斤拷锟斤拷锟饺拷锟絯_cost_obs
	//	锟狡硷拷锟劫讹拷reference_speed
	//	拼锟斤拷锟斤拷锟斤拷锟劫讹拷plan_start_s_dot
	//	s_list, t_list 锟斤拷锟斤拷锟斤拷锟斤拷
	//	dp_st_s_dot 锟斤拷锟节硷拷锟斤拷锟斤拷俣锟�
	//	锟斤拷锟饺硷拷锟斤拷锟秸碉拷锟絪t锟斤拷锟斤拷
	float out[2];
	CalcSTCoordinate(row_end, col_end, s_list, t_list, out);
	float s_end = out[0]; float t_end = out[1];

	float s_start, t_start, s_dot_start;
	//锟芥定锟斤拷锟斤拷锟斤拷锟叫猴拷为0 
	if (row_start == 0)
		//锟竭碉拷锟斤拷锟轿猟p锟斤拷锟斤拷锟�
	{
		s_start = 0;
		t_start = 0;
		s_dot_start = plan_start_s_dot;
	}
	//锟竭碉拷锟斤拷悴伙拷锟絛p锟斤拷锟斤拷锟�
	else
	{
		CalcSTCoordinate(row_start, col_start, s_list, t_list, out);
		s_start = out[0]; t_start = out[1];
		s_dot_start = dp_st_s_dot(row_start - 1, col_start - 1);
	}

	float cur_s_dot = (s_end - s_start) / (t_end - t_start);
	float cur_s_dot2 = (cur_s_dot - s_dot_start) / (t_end - t_start);
	//锟斤拷锟斤拷锟狡硷拷锟劫度达拷锟斤拷
	float cost_ref_speed = w_cost_ref_speed * pow((cur_s_dot - reference_speed), 2);
	//锟斤拷锟斤拷锟斤拷俣却锟斤拷郏锟斤拷锟斤拷锟阶拷猓拷锟斤拷俣炔锟斤拷艹锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟窖э拷锟斤拷锟斤拷锟�
	float cost_accel;
	if (cur_s_dot2 < 4 && cur_s_dot2 > -6)
		cost_accel = w_cost_accel * pow(cur_s_dot2, 2);
	else
		//锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷学锟斤拷锟狡ｏ拷锟斤拷锟桔伙拷锟斤拷锟斤拷芏啾�
		cost_accel = 100000 * w_cost_accel * pow(cur_s_dot2, 2);
	//锟斤拷锟斤拷锟较帮拷锟斤拷锟斤拷锟�
	float cost_obs = CalcObsCost(s_start, t_start, s_end, t_end, obs_st_s_in_set, obs_st_s_out_set, obs_st_t_in_set, obs_st_t_out_set, w_cost_obs);
	//锟斤拷锟斤拷锟斤拷酆锟�
	float cost = cost_obs + cost_accel + cost_ref_speed;
	return cost;


}
void dynamic_planning(VectorXd obs_st_s_in_set, VectorXd obs_st_s_out_set,
	VectorXd obs_st_t_in_set, VectorXd obs_st_t_out_set, float reference_speed_unlimit,
	float w_cost_ref_speed, float w_cost_accel, float w_cost_obs, float plan_start_s_dot,
	VectorXd &dp_speed_s_out, VectorXd &dp_speed_t_out)
{
	int s_size = 40;
	//锟劫度讹拷态锟芥划
	//时锟斤拷锟�0锟斤拷8锟斤拷始锟芥划锟斤拷锟斤拷锟�8锟斤拷
	//s锟侥凤拷围锟斤拷0锟斤拷始锟斤拷路锟斤拷锟芥划锟斤拷path锟斤拷锟杰筹拷锟斤拷为止
	int pre_row, pre_col;
	float out[2];
	VectorXd s_list(s_size);
	s_list(0) = 0;
	float ds=1.5;
	if (s_size == 40)
		for (int i = 1; i < s_size; i++)
		{
			//改距离
			if (i < 10)
				s_list(i) = s_list(i - 1) + ds;
			else if (i >= 10 && i < 20)
				s_list(i) = s_list(i - 1) + ds;
			else if (i >= 20 && i < 30)
				s_list(i) = s_list(i - 1) + ds;
			else if (i >= 30 && i < 40)
				s_list(i) = s_list(i - 1) + ds;
		}
	else
		for (int i = 1; i < s_size; i++)
		{
			if (i < 10)
				s_list(i) = s_list(i - 1) + 0.5;
			else if (i >= 10 && i < 20)
				s_list(i) = s_list(i - 1) + 1;

			// if (i < 5)
			// 	s_list(i) = s_list(i - 1) + 0.1;
			// else if (i >= 5 && i < 10)
			// 	s_list(i) = s_list(i - 1) + 0.5;
			// else if (i >= 10 && i < 20)
			// 	s_list(i) = s_list(i - 1) + 1;
		}
	//t 锟斤拷锟矫撅拷锟饺诧拷锟斤拷每 0.5s锟斤拷锟斤拷一锟斤拷锟斤拷
	VectorXd t_list(16);
	t_list(0) = 0.5;
	for (int i = 1; i < 16; i++)
		t_list(i) = t_list(i - 1) + 0.5;

	//锟斤拷锟斤拷st锟斤拷锟桔撅拷锟襟，憋拷示锟斤拷锟斤拷憧硷拷锟�(i, j)锟斤拷锟斤拷锟叫★拷锟斤拷锟轿猟p_st_cost(i-1, j-1)
	//s 锟斤拷锟斤拷40锟斤拷 t 锟斤拷锟斤拷 16锟斤拷 一锟斤拷锟斤拷 40 * 16
	MatrixXf dp_st_cost(s_size, 16);
	dp_st_cost.fill(INFINITY);
	//dp_st_s_dot(i-1,j-1)锟斤拷锟斤拷示锟斤拷锟斤拷憧硷拷锟�(i,j)锟斤拷锟斤拷锟斤拷锟铰凤拷锟斤拷锟侥╋拷俣锟�,(i,j)为锟叫号猴拷锟叫猴拷
	MatrixXf dp_st_s_dot(s_size, 16);
	dp_st_s_dot.fill(0);
	//锟斤拷要一锟斤拷锟斤拷锟襟保筹拷锟斤拷锟斤拷路锟斤拷锟斤拷前一锟斤拷锟节点方锟斤拷锟斤拷锟�
	//dp_st_node(i-1, j-1) 锟斤拷示位锟斤拷为(i, j)锟侥节碉拷锟叫ｏ拷锟斤拷锟脚碉拷锟斤拷一锟斤拷诘锟斤拷锟叫猴拷为dp_st_node(i-1, j-1)
	MatrixXf dp_st_node(s_size, 16);
	//锟斤拷锟斤拷锟絛p锟斤拷愕斤拷锟揭伙拷械锟絚ost
	for (int i = 1; i <= s_list.size(); i++)
	{

		dp_st_cost(i - 1, 0) = CalcDpCost(0, 0, i, 1, obs_st_s_in_set, obs_st_s_out_set, obs_st_t_in_set, obs_st_t_out_set,
			w_cost_ref_speed, reference_speed_unlimit, w_cost_accel, w_cost_obs, plan_start_s_dot, s_list, t_list, dp_st_s_dot);
		//锟斤拷锟斤拷锟揭伙拷锟斤拷锟斤拷薪诘锟侥碉拷s_dot锟斤拷锟斤拷锟芥储锟斤拷dp_st_s_dot锟斤拷
		//锟斤拷一锟叫碉拷前一锟斤拷锟节碉拷只锟斤拷锟斤拷悖拷锟斤拷锟絪 t 锟斤拷锟斤拷0
		CalcSTCoordinate(i, 1, s_list, t_list, out);
		float s_end = out[0]; float t_end = out[1];
		dp_st_s_dot(i - 1, 0) = s_end / t_end;
	}
	//锟斤拷态锟芥划锟斤拷锟斤拷锟斤拷
	for (int i = 1; i < t_list.size(); i++)
		for (int j = 0; j < s_list.size(); j++)
		{
			//锟斤拷前锟斤拷为 j+1 锟斤拷为 i+1
			int cur_row = j + 1;
			int cur_col = i + 1;
			//锟斤拷锟斤拷前一锟斤拷
			for (int k = 0; k < s_list.size(); k++)
			{
				pre_row = k + 1;
				pre_col = i;
				//锟斤拷锟斤拷叩拇锟斤拷锟� 锟斤拷锟斤拷锟斤拷锟轿猵re_row,pre_col 锟秸碉拷为cur_row cur_col
				float cost_temp = CalcDpCost(pre_row, pre_col, cur_row, cur_col, obs_st_s_in_set, obs_st_s_out_set,
					obs_st_t_in_set, obs_st_t_out_set, w_cost_ref_speed, reference_speed_unlimit, w_cost_accel, w_cost_obs,
					plan_start_s_dot, s_list, t_list, dp_st_s_dot);
				if (cost_temp + dp_st_cost(pre_row - 1, pre_col - 1) < dp_st_cost(cur_row - 1, cur_col - 1))
				{
					dp_st_cost(cur_row - 1, cur_col - 1) = cost_temp + dp_st_cost(pre_row - 1, pre_col - 1);
					//锟斤拷锟斤拷锟斤拷锟脚碉拷s_dot
					CalcSTCoordinate(pre_row, pre_col, s_list, t_list, out);
					float s_start = out[0]; float t_start = out[1];
					CalcSTCoordinate(cur_row, cur_col, s_list, t_list, out);
					float s_end = out[0]; float t_end = out[1];
					dp_st_s_dot(cur_row - 1, cur_col - 1) = (s_end - s_start) / (t_end - t_start);
					//锟斤拷锟斤拷锟铰凤拷锟斤拷锟角耙伙拷锟斤拷诘锟斤拷锟叫号硷拷录锟斤拷锟斤拷
					dp_st_node(cur_row - 1, cur_col - 1) = pre_row;
				}
			}
		}

	//锟斤拷锟斤拷锟绞硷拷锟�
	VectorXd dp_speed_s(t_list.size());
	dp_speed_s.fill(NAN);
	VectorXd dp_speed_t = dp_speed_s;
	//锟揭碉拷dp_node_cost 锟较边斤拷锟斤拷冶呓锟斤拷锟斤拷锟斤拷小锟侥节碉拷
	float min_cost = INFINITY;
	int min_row = 999999;
	int min_col = 999999;

	for (int i = 0; i < s_list.size(); i++)
		//锟斤拷锟斤拷锟揭边斤拷
		if (dp_st_cost(i, t_list.size() - 1) <= min_cost)
		{
			min_cost = dp_st_cost(i, t_list.size() - 1);
			min_row = i + 1;
			min_col = t_list.size();
		}
	for (int j = 0; j < t_list.size(); j++)
		//锟斤拷锟斤拷锟较边斤拷
		if (dp_st_cost(0, j) <= min_cost)
		{
			min_cost = dp_st_cost(0, j);
			min_row = 1;
			min_col = j + 1;
		}
	//锟斤拷锟斤拷要注锟解，锟斤拷然锟斤拷锟斤拷锟斤拷t锟斤拷每0.5s锟斤拷锟斤拷一锟斤拷时锟戒，锟斤拷锟斤拷16锟斤拷锟姐，锟斤拷锟斤拷min_col 未锟截碉拷锟斤拷16
	//	也锟斤拷锟斤拷说锟斤拷态锟芥划锟斤拷锟斤拷锟脚斤拷锟绞憋拷锟轿达拷锟斤拷锟�8锟斤拷
	//	锟斤拷锟斤拷锟斤拷锟角讹拷态锟芥划锟斤拷锟斤拷锟斤拷锟絪 t 锟斤拷一锟斤拷锟斤拷写锟斤拷
	//	锟饺帮拷锟秸碉拷锟絊T锟斤拷锟斤拷锟斤拷锟�
	CalcSTCoordinate(min_row, min_col, s_list, t_list, out);
	dp_speed_s(min_col - 1) = out[0]; dp_speed_t(min_col - 1) = out[1];
	//锟斤拷锟斤拷锟斤拷锟�
	while (min_col != 1)
	{
		pre_row = dp_st_node(min_row - 1, min_col - 1);
		pre_col = min_col - 1;
		CalcSTCoordinate(pre_row, pre_col, s_list, t_list, out);
		dp_speed_s(pre_col - 1) = out[0]; dp_speed_t(pre_col - 1) = out[1];
		min_row = pre_row;
		min_col = pre_col;


	}
	dp_speed_s_out = dp_speed_s;
	dp_speed_t_out = dp_speed_t;
}




