
#include"main.h"




void path_and_speed_merge(VectorXd s, VectorXd s_dot, VectorXd s_dot2, VectorXd relative_time,double time_now, VectorXd path_s,
	VectorXd trajectory_x_init, VectorXd trajectory_y_init, VectorXd trajectory_heading_init, VectorXd trajectory_kappa_init,
	VectorXd& trajectory_x_out, VectorXd& trajectory_y_out, VectorXd& trajectory_heading_out, VectorXd& trajectory_kappa_out,
	VectorXd& trajectory_speed_out,VectorXd& trajectory_accel_out, VectorXd& trajectory_time_out)
{
	/*�ú������ϲ�path��speed
		����path �� 60���㣬speed �� 401���㣬�ϲ���path��speed��401���㣬�����Ҫ����ֵ*/
	int n = 61;
	int k=7;
	VectorXd trajectory_x(n);
	trajectory_x.fill(0);
	VectorXd trajectory_y(n);
	trajectory_y.fill(0);
	VectorXd trajectory_heading(n);
	trajectory_heading.fill(0);
	VectorXd trajectory_kappa(n);
	trajectory_kappa.fill(0);
	VectorXd trajectory_speed(n);
	trajectory_speed.fill(0);
	VectorXd trajectory_accel(n);
	trajectory_accel.fill(0);
	VectorXd trajectory_time(n);
	trajectory_time.fill(0);
	int index = 0;
	//����trajectory_x_init�в������е�ֵ����Ч��(��nan) ����Ҫ�ҵ���Чֵ�ķ�Χ
	while (!isnan(trajectory_x_init(index)))
	{
		index = index + 1;
		if (index == trajectory_x_init.size())
			break;
	}
	index = index - 1;
	for (int i = 0; i < n ; i++)
	{
		//cout<<path_s;
		/*interp1 ���Բ�ֵ
			�� x = [1, 2] y = [3, 4]
			interp1(x, y, 1.5) ��õ�3.5
			interp1 ��ȱ���� �ڶ˵�򳬹��˵�ģ����Ϊnan
			interp1(x, y, 2) �������4 ���ǻ����nan
			interp1(x, y, 6) Ҳ�����nan�������ڶ˵㴦Ҫ��������*/
		trajectory_x(i) = interp1(path_s, k*trajectory_x_init, index, s(i))/k;
		trajectory_y(i) = interp1(path_s, k*trajectory_y_init, index, s(i))/k;
		if(trajectory_x(i)<-1000&&i>5)
		{
			// trajectory_x(i)=trajectory_x(i-1)+trajectory_x(i-1)-trajectory_x(i-2);
			// trajectory_y(i)=trajectory_y(i-1)+trajectory_y(i-1)-trajectory_y(i-2);
	    cout<<"NAN:"<<s(i)<<endl;
		cout<<"path_s"<<path_s<<endl<<endl;
		}
		trajectory_heading(i) = interp_heading(path_s, trajectory_heading_init, index, s(i));
		trajectory_kappa(i) = interp_time(path_s, trajectory_kappa_init, index, s(i));
		trajectory_time(i) = relative_time(i) + time_now; //�ٶȹ滮��õ����ʱ�䣬ת���ɾ���ʱ��
		trajectory_speed(i) = s_dot(i);
		trajectory_accel(i) = s_dot2(i);

	}

	//�˵㵥������
	//trajectory_x(n-1) = trajectory_x_init(trajectory_x_init.size()-1);
	//trajectory_y(n-1) = trajectory_y_init(trajectory_y_init.size()-1);
	//trajectory_heading(n-1) = trajectory_heading_init(trajectory_heading_init.size()-1);
	//trajectory_kappa(n-1) = trajectory_kappa_init(trajectory_kappa_init.size()-1);
	//trajectory_time(n-1) = relative_time(relative_time.size()-1) + current_time;
	//trajectory_speed(n-1) = s_dot(s_dot.size()-1);
	//trajectory_accel(n-1) = s_dot2(s_dot2.size()-1);

	//���
	trajectory_x_out = trajectory_x;
	trajectory_y_out = trajectory_y;
	trajectory_heading_out = trajectory_heading;
	trajectory_kappa_out = trajectory_kappa;
	trajectory_speed_out = trajectory_speed;
	trajectory_accel_out = trajectory_accel;
	trajectory_time_out = trajectory_time;

}