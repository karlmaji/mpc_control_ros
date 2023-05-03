#include <iostream>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <cmath>
#include "math.h"
using namespace std;
using namespace Eigen;
//�ú�����������trajectory��s �� x y �Ķ�Ӧ��ϵ�����Կ�����trajectory index2s
double index2s(VectorXd trajectory_x, VectorXd trajectory_y, VectorXd& path_index2s_out)//��&��֤������������
{

	float k=7;
	double path_s_end;
	int n = trajectory_x.size();
	VectorXd path_index2s(trajectory_x.size());
	path_index2s.fill(0);

	double sum = 0;
	int j;
	for (int i = 1; i < n ; i++)
	{

		if (isnan(trajectory_x(i)))
			break;
		else
		{
			sum = sum + k*sqrt(pow((trajectory_x(i) - trajectory_x(i-1)), 2) + pow((trajectory_y(i) - trajectory_y(i-1)), 2));
			path_index2s(i) = sum;
		}
		j = i;
	}
	if (j == n - 1)
		path_s_end = path_index2s(n - 1);
	else
		//��Ϊѭ�����˳�����Ϊisnan(trajectory_x(i)) ���� i ����Ӧ����Ϊ nan
		path_s_end = path_index2s(j - 1);

	path_index2s_out = path_index2s;
	return path_s_end;
}
