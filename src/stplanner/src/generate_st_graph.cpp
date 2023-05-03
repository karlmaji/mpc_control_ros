#include <iostream>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <cmath>
using namespace std;
using namespace Eigen;
void generate_st_graph(VectorXd& obs_st_s_in_set, VectorXd& obs_st_s_out_set, VectorXd& obs_st_t_in_set, VectorXd& obs_st_t_out_set,
	VectorXd dynamic_obs_s_set, VectorXd dynamic_obs_l_set, VectorXd dynamic_obs_s_dot_set, VectorXd dynamic_obs_l_dot_set)
{
	//�ú���������stͼ��ʹ��бֱ��ģ��
	int n = dynamic_obs_s_set.size();
	float obs_sensor=1.5;
	for (int i = 0; i < n; i++)
	{
		if (isnan(dynamic_obs_s_set(i)))
			break;
		if (abs(dynamic_obs_l_set(i)) > obs_sensor)
				{continue;cout<<dynamic_obs_l_set(i)<<endl;}
		// if( abs(dynamic_obs_l_dot_set(i)) < 0.3)
		// {
		// 	//�������ƶ����ϰ���
		// 		if (abs(dynamic_obs_l_set(i)) > 2)
		// 			// �������̫Զ���ٶȹ滮ֱ�Ӻ���
		// 			continue;
		// 		// else
		// 		// 	continue;
		// }
		
		//t_zero Ϊ��̬�ϰ����l��0������Ҫ��ʱ��
		//ʱ�����·�̳����ٶ�
		float t_zero = -dynamic_obs_l_set(i) / dynamic_obs_l_dot_set(i); 
		//���� + -2����ʱ��
		float t_boundary1 = 0.5 / dynamic_obs_l_dot_set(i) + t_zero;
		float t_boundary2 = -0.5 / dynamic_obs_l_dot_set(i) + t_zero;
		float t_max, t_min;
		if (t_boundary1 > t_boundary2)
		{
			t_max = t_boundary1;
			t_min = t_boundary2;
		}
		else
		{
			t_max = t_boundary2;
			t_min = t_boundary1;
		}
		if (t_max < 1 || t_min > 8)
			//���������г�̫Զ�ģ��������ɵģ�����
			//�����˶���Ҫ�ܵ���������ѧ��Լ�ģ���������ɵģ���ʹ�滮���˺ܴ�ļ��ٶȣ�����Ҳִ�в���
			//�����ϰ���Ҳ��Ҫ�������ϰ����·���滮һ����
			continue;
		else if (t_min < 0 && t_max > 0)
			//�ڸ�֪������ʱ���ϰ����Ѿ��� + -2���ڲ���
		{
			obs_st_s_in_set(i) = dynamic_obs_s_set(i);
			//�����˶�
			obs_st_s_out_set(i) = dynamic_obs_s_set(i) + dynamic_obs_s_dot_set(i) * t_max;
			obs_st_t_in_set(i) = 0;
			obs_st_t_out_set(i) = t_max;
		}
		else
			//�����ϰ���
		{
			obs_st_s_in_set(i) = dynamic_obs_s_set(i) + dynamic_obs_s_dot_set(i) * t_min;
			obs_st_s_out_set(i) = dynamic_obs_s_set(i) + dynamic_obs_s_dot_set(i) * t_max;
			obs_st_t_in_set(i) = t_min;
			obs_st_t_out_set(i) = t_max;
		}
		//cout << obs_st_s_in_set(i);
	}


}

