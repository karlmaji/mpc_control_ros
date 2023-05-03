#include "main.h"

void generate_convex_space(VectorXd dp_speed_s, VectorXd dp_speed_t, VectorXd path_index2s, VectorXd obs_st_s_in_set, VectorXd obs_st_s_out_set,
	VectorXd obs_st_t_in_set, VectorXd obs_st_t_out_set, VectorXd trajectory_kappa_init,float max_lateral_accel, VectorXd& s_lb_out, VectorXd& s_ub_out, VectorXd& s_dot_lb_out, VectorXd& s_dot_ub_out)
{
	//�ú����������s, s_dot�����½磬����͹�ռ乩���ι滮ʹ��
	//dp_speed_s t �����16���������ʼ��n = 16
	int n = 16;
	VectorXd s_lb(n);
	s_lb.fill(-OsqpEigen::INFTY);
	VectorXd s_ub(n);
	s_ub.fill(OsqpEigen::INFTY);
	VectorXd s_dot_lb(n);
	s_dot_lb.fill(-OsqpEigen::INFTY);
	VectorXd s_dot_ub(n);
	s_dot_ub.fill(OsqpEigen::INFTY);
	int path_index2s_end_index = path_index2s.size()-1;
	int dp_speed_end_index = dp_speed_s.size()-1;
	//path_index2sҲ���л���ģ��ҵ���Ч��path_index2s��ĩβ��λ�ã���ֵ��path_index2s_end_index
	for (int k = 1; k < path_index2s.size(); k++)
	{
		if (path_index2s(k) == 0 && path_index2s(k - 1) != 0)
		{
			path_index2s_end_index = k - 1;
			break;
		}
			path_index2s_end_index = k;
	}
	//�ҵ�dp_speed_s����Ч��λ�ã���ֵ��dp_speed_end_index
	for (int k = 0;k< dp_speed_s.size();k++)
		if (isnan(dp_speed_s(k)))
		{
			dp_speed_end_index = k - 1;
			break;
		}
	for (int i = 0; i < n; i++)
		//��ѭ����ʩ�ӳ�������ѧԼ��
	{
		if (isnan(dp_speed_s(i)))
			break;
		float cur_s = dp_speed_s(i);
		//ͨ����ֵ�ķ�ʽ�ҵ�cur_s ����Ӧ������
		float cur_kappa = interp1(path_index2s, trajectory_kappa_init, path_index2s_end_index, cur_s);
		//a = v ^ 2 * k = > v ^ 2 = a / k ע���0����
		float max_speed = sqrt(max_lateral_accel / (abs(cur_kappa) + 1e-10));
	    float min_speed = 0;
		s_dot_lb(i) = min_speed;
		s_dot_ub(i) = max_speed;
		//if (isnan(max_speed)||max_speed>2)
			s_dot_ub(i) = 4;
		
	}
	for (int i = 0; i < obs_st_s_in_set.size(); i++)
	{
		if (isnan(obs_st_s_in_set(i)))
			continue;
		//ȡs t ֱ�ߵ��е㣬��Ϊobs_s obs_t ������ 
		float obs_t = (obs_st_t_in_set(i) + obs_st_t_out_set(i)) / 2;
		float obs_s = (obs_st_s_in_set(i) + obs_st_s_out_set(i)) / 2;
		//�����ϰ���������ٶ�
		float obs_speed = (obs_st_s_out_set(i) - obs_st_s_in_set(i)) / (obs_st_t_out_set(i) - obs_st_t_in_set(i));
		//��ֵ�ҵ���t = obs_tʱ��dp_speed_s ��sֵ
		float dp_s = interp1(dp_speed_t,dp_speed_s,dp_speed_end_index ,obs_t);
		//�ҵ�dp_speed_t�� ��obs_st_t_in_set(i)�����ʱ�䣬������ʱ��ı�Ÿ�ֵ��t_lb_index
		int t_lb_index = 0, t_ub_index = 0;
		for (int j = 0; j < dp_speed_t.size() - 1; j++)
		{
			//����ϰ�������ʱ���0.5�뻹Ҫ�̣���ôt_lb_index = 0
			if (dp_speed_t(0) > obs_st_t_in_set(i))
			{		
				t_lb_index = 0;
				break;
			}
			else if (dp_speed_t(j) <= obs_st_t_in_set(i) && dp_speed_t(j + 1) > obs_st_t_in_set(i))
			{//�������dp_speed_t �ҵ���obs_st_t_in_set(i)����ĵ�ı��
				t_lb_index = j;
				break;
			}
		}
		//�ҵ�dp_speed_t�� ��obs_st_t_out_set(i)�����ʱ�䣬������ʱ��ı�Ÿ�ֵ��t_ub_index
		for (int j = 0; j < dp_speed_t.size() - 1; j++)
		{
			t_ub_index = j;
			if (dp_speed_t(0) > obs_st_t_out_set(i))
			{
				
				break;
			}
			else if (dp_speed_t(j) <= obs_st_t_out_set(i) && dp_speed_t(j + 1) > obs_st_t_out_set(i))
			{
				break;
			}
		}

		//������΢�������壬�� t_lb_index ��΢��СһЩ��t_ub_index��΢�Ŵ�һЩ
		t_lb_index = max(t_lb_index - 2, 2); //���Ϊ3 ��Ϊ����û������
		t_ub_index = min(t_ub_index + 2, dp_speed_end_index);
		//���߲���
		float dp_t = 0;

		//重新写超车避让逻辑
		// if (obs_s > dp_s)
		// {
		// 	cout<<"avoid"<<endl;
		// 	//����Ϊ���ٱ���
		// 	for (int m = t_lb_index; m <= t_ub_index; m++)
		// 	{
		// 		//��t_lb_index:t_ub_index�������� s���Ͻ粻���Գ����ϰ���stбֱ��
		// 		dp_t = dp_speed_t(m);
		// 		s_ub(m) = min(s_ub(m), obs_st_s_in_set(i) + obs_speed * (dp_t - obs_st_t_in_set(i)));
		// 	}

		// }
		// else
		// {
		// 	//����Ϊ���ٳ���
		// 	cout<<"exceed"<<endl;
		// 	for (int m = t_lb_index; m <= t_ub_index; m++)
		// 	{// ��t_lb_index : t_ub_index�������� s���½粻��С���ϰ���stбֱ��
		// 		dp_t = dp_speed_t(m);
		// 		s_lb(m) = max(s_lb(m), obs_st_s_in_set(i) + obs_speed * (dp_t - obs_st_t_in_set(i)));

		// 	}


		// }


		//避让
			cout<<"avoid"<<endl;
			for (int m = t_lb_index; m <= t_ub_index; m++)
			{
				dp_t = dp_speed_t(m);
				s_ub(m) = min(s_ub(m), obs_st_s_in_set(i) + obs_speed * (dp_t - obs_st_t_in_set(i)));

			}


		
		// for(int t=0;t<=dp_speed_end_index;t++)
		// {
		// if(dp_speed_t(t)>=obs_st_t_in_set(0))
		// {
		// if(dp_speed_s(t)>=obs_st_s_in_set(0))
		// {//超车
		// cout<<"exceed"<<endl;
		// // for (int m = t_lb_index; m <= t_ub_index; m++)
		// // {
		// // dp_t = dp_speed_t(m);
		// // s_lb(m) = max(s_lb(m), obs_st_s_in_set(i) + obs_speed * (dp_t - obs_st_t_in_set(i)));
		// // }
		// }
		// else
		// {
		// 	//避让
		// 	cout<<"avoid"<<endl;
		// 	for (int m = t_lb_index; m <= t_ub_index; m++)
		// 	{
		// 		dp_t = dp_speed_t(m);
		// 		s_ub(m) = min(s_ub(m), obs_st_s_in_set(i) + obs_speed * (dp_t - obs_st_t_in_set(i)));

		// 	}
		// }
		// break;
		// }
		
		// }
	}
	
	s_lb_out = s_lb;
	s_ub_out = s_ub;
	s_dot_lb_out = s_dot_lb;
	s_dot_ub_out = s_dot_ub;

}

