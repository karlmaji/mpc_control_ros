#include"main.h"
float interp1(VectorXd path_index2s, VectorXd trajectory_kappa_init, int path_index2s_end_index, float cur_s)
{
	//��ֵ��������path_index2s�ķ�Χ��᷵��nan
	float cur_kappa = -10086;
	for (int i = 0; i <= path_index2s_end_index; i++)
	{
		if (cur_s > path_index2s(i))
		{
			if (i == path_index2s_end_index)
				break;
			else if (cur_s < path_index2s(i + 1))
			{
				cur_kappa = (trajectory_kappa_init(i + 1) - trajectory_kappa_init(i)) / (path_index2s(i + 1) - path_index2s(i))*(cur_s - path_index2s(i)) + trajectory_kappa_init(i);
				break;
			}
		}
		else if (cur_s == path_index2s(i)&&i!= path_index2s_end_index)
			cur_kappa = trajectory_kappa_init(i);
	}
	return cur_kappa;
}

double interp_time(VectorXd trajectory_time, VectorXd trajectory_x, int index, double time)
{
	//��ֵ��������path_index2s�ķ�Χ��᷵��nan
	double cur_x = -10086;
	// if(time==trajectory_time(0))
	// 	cur_x=trajectory_x(0);
	// else if(time==trajectory_time(index))
	// 	cur_x=trajectory_x(index);
	// else
	// {
	for (int i = 0; i <= index; i++)
	{
		if (time > trajectory_time(i))
		{
			if (i == index)
				break;
			else if (time < trajectory_time(i + 1))
			{
				cur_x = (trajectory_x(i + 1) - trajectory_x(i)) / (trajectory_time(i + 1) - trajectory_time(i))*(time - trajectory_time(i)) + trajectory_x(i);
				break;
			}
		}
		else if(time == trajectory_time(i))
			cur_x=trajectory_x(i);
	}
	// }

	return cur_x;
}
double interp_heading(VectorXd path_s, VectorXd trajectory_heading, int index, double s)
{
	//��ֵ��������path_index2s�ķ�Χ��᷵��nan
	double cur_heading= -10086;
	// if(time==trajectory_time(0))
	// 	cur_x=trajectory_x(0);
	// else if(time==trajectory_time(index))
	// 	cur_x=trajectory_x(index);
	// else
	// {
	cout<<"heading:"<<endl;
	for (int i = 0; i <= index; i++)
	{
		if (s > path_s(i))
		{
			if (i == index)
				break;
			else if (s < path_s(i + 1))
			{
				if(abs(trajectory_heading(i + 1))> M_PI_2l && abs(trajectory_heading(i))>M_PI_2l &&(trajectory_heading(i + 1)*trajectory_heading(i)<0))
				{
					if(trajectory_heading(i)<0)
					cur_heading = (trajectory_heading(i + 1) - (2*M_PIl+trajectory_heading(i))) / (path_s(i + 1) - path_s(i))*(s - path_s(i)) + (2*M_PIl+trajectory_heading(i));
					else if(trajectory_heading(i+1)<0)
					cur_heading = ((2*M_PIl+trajectory_heading(i + 1)) - trajectory_heading(i)) / (path_s(i + 1) - path_s(i))*(s - path_s(i)) + trajectory_heading(i);
					
					if(cur_heading>M_PIl)
					cur_heading=cur_heading-2*M_PIl;
					else if(cur_heading<-M_PIl)
					cur_heading=cur_heading+2*M_PIl;
					//cout<<trajectory_heading(i)<<"  "<<trajectory_heading(i + 1)<<"  ";
					// cout<<"heading1: "<<cur_heading<<" ";
					// if(cur_heading>0)
					// cur_heading=cur_heading+M_PIl;
					// else
					// cur_heading=cur_heading-M_PIl;

					//cout<<"heading2: "<<cur_heading<<endl;
				}
				else
				cur_heading = (trajectory_heading(i + 1) - trajectory_heading(i)) / (path_s(i + 1) - path_s(i))*(s - path_s(i)) + trajectory_heading(i);
				cout<<cur_heading<<"  "<<trajectory_heading(i + 1)<<"  "<<trajectory_heading(i)<<"   "<<endl;
				break;
			}
			
		}
		else if(s == path_s(i))
			cur_heading=trajectory_heading(i);

	}
	

	return cur_heading;
}