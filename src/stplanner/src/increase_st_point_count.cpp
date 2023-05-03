#include "main.h"
void increase_st_point_count(VectorXd s_init, VectorXd s_dot_init, VectorXd s_dot2_init, VectorXd relative_time_init,
	VectorXd& s_out, VectorXd& s_dot_out, VectorXd& s_dot2_out, VectorXd& relative_time_out)
{
	/*�ú���������s s_dot s_dot2
		Ϊʲô��Ҫ���ܣ���Ϊ���Ƶ�ִ��Ƶ���ǹ滮��10�����켣����������ܣ���Ȼ�ᵼ�¹滮Ч������
		���������ٶȶ��ι滮�е�ȡ��̫�࣬�ᵼ�¶��ι滮�ľ����ģ̫�����̫��
		���Է������ڶ��ι滮��ѡȡ�����ĵ��Ż���Ϻ����ô˺�������*/
	int t_end = relative_time_init.size()-1;
	for (int i = 0; i < relative_time_init.size(); i++)
	{
		if (isnan(relative_time_init(i)))
		{
			t_end = i - 1;
			break;
		}
	}
	float T = relative_time_init(t_end);
	int n = 61;
	float dt = T / (n - 1);
	VectorXd s(n);
	s.fill(0);
	VectorXd s_dot(n);
	s_dot.fill(0);
	VectorXd s_dot2(n);
	s_dot2.fill(0);
	VectorXd relative_time(n);
	relative_time.fill(0);
	for (int i = 0; i < n; i++)
	{
		int k = 0;
		float current_t = i * dt;
		for (int j = 0; j <= t_end - 1; j++)
		{
			if (relative_time_init(j) <= current_t && relative_time_init(j + 1) >= current_t)
			{
				k = j;
				break;
			}
			if (abs(relative_time_init(j + 1)-current_t)<1e-4&&j==t_end-1)
			{
				k = j;
				break;
			}
				
		}
		float x = current_t - relative_time_init(k);
		//s(i) = s_init(k) + s_dot_init(k)*x + (1.0 / 3)*s_dot2_init(k)*pow(x, 2) + (1.0 / 6)*s_dot2_init(k + 1)*pow(x, 2);
		s_dot(i) = s_dot_init(k) + 0.5*s_dot2_init(k)*x + 0.5*s_dot2_init(k + 1)*x;
		s_dot2(i) = s_dot2_init(k) + (s_dot2_init(k + 1) - s_dot2_init(k))*x / (relative_time_init(k + 1) - relative_time_init(k));
		relative_time(i) = current_t;
	}
	//重新增密
	int s_index;
	for(int i=s_init.size()-1;i>0;i--)
	{
		if(isnan(s_init(s_init.size()-1))==false)
		{s_index=s_init.size()-1;break;}
		else if(isnan(s_init(i)))
		s_index=i;
	}
	for(int i=0;i<s.size();i++)
	s(i) = interp_time(relative_time_init, s_init, s_index, relative_time(i));


	s_out = s;
	s_dot_out = s_dot;
	s_dot2_out = s_dot2;
	relative_time_out = relative_time;

}