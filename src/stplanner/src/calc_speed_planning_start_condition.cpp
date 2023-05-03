#include <iostream>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <cmath>
using namespace std;
using namespace Eigen;
void planning_start_condition(float plan_start_vx, float plan_start_vy, float plan_start_ax, float plan_start_ay, float plan_start_heading ,float out[2])
{
	//�ú��������ٶȹ滮�ĳ�ʼ����
	VectorXd tor(2);
	// tor(0)= cos(M_PI / 180 * plan_start_heading); 
	// tor(1)= sin(M_PI / 180 * plan_start_heading);
	tor(0)= cos(plan_start_heading); 
	tor(1)= sin(plan_start_heading);
	//cout << "tor:"<<tor(0) << endl<<endl;
	//��������v�������ͶӰ
	VectorXd v(2);
	v(0) = plan_start_vx;
	v(1) = plan_start_vy;
	VectorXd a(2);
	a(0) = plan_start_ax;
	a(1) = plan_start_ay;
	float plan_start_s_dot = tor.transpose() *v;
	float plan_start_s_dot2 = tor.transpose() *a;
	out[0] = plan_start_s_dot;
	out[1] = plan_start_s_dot2;
}
