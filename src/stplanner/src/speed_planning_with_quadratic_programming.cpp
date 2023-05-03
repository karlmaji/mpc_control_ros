#include "main.h"
void quadratic_programming(float plan_start_s_dot,float plan_start_s_dot2,VectorXd dp_speed_s,VectorXd dp_speed_t,
	VectorXd s_lb, VectorXd s_ub, VectorXd s_dot_lb, VectorXd s_dot_ub,float w_cost_s_dot2,float w_cost_v_ref,float w_cost_jerk,float speed_reference,
	VectorXd& qp_s_init_out, VectorXd& qp_s_dot_init_out, VectorXd& qp_s_dot2_init_out,VectorXd& relative_time_init_out)
{
	//�ٶȶ��ι滮
	//	���룺�滮���plan_start_s_dot, plan_start_s_dot2
	//	��̬�滮���dp_speed_s, dp_speed_t
	//	͹�ռ�Լ�� s_lb, s_ub, s_dot_lb, s_dot_ub
	//	���ٶȴ���Ȩ�أ��Ƽ��ٶȴ���Ȩ�أ�jerk����Ȩ��w_cost_s_dot2, w_cost_v_ref, w_cost_jerk
	//	�ο��ٶ�speed_reference
	//	������ٶ�����qp_s_init, qp_s_dot_init, qp_s_dot2_init, relative_time_init
	//	coder.extrinsic("quadprog")
	//	����dp�Ľ��δ����16�����㷨������dp_speed_end�����Ƕ���
	int dp_speed_end = 15;
	for (int i = 0; i < dp_speed_s.size(); i++)
	{
		if (isnan(dp_speed_s(i)))
		{
			dp_speed_end = i - 1;
			break;
		}
	}
	/*����dp_speed_endʵ�����ǲ�ȷ���ģ��������Ҫ�������ȷ�����ȵ�ֵ�����
		�����ʼ��ѡ��dp_speed_end�����ֵ + �滮�����Ϊ�����ʼ���Ĺ�ģ*/
	int n = 17;
	VectorXd qp_s_init(n);
	qp_s_init.fill(NAN);
	VectorXd qp_s_dot_init(n);
	qp_s_dot_init.fill(NAN);
	VectorXd qp_s_dot2_init(n);
	qp_s_dot2_init.fill(NAN);
	VectorXd relative_time_init(n);
	relative_time_init.fill(NAN);
	float s_end = dp_speed_s(dp_speed_end);
	//��ʱdp_speed_end��ʾ������Ч��dp_speed_t��Ԫ�ظ�����ȡ��dp_speed_t��Ч�����һ��Ԫ����Ϊ�滮��ʱ���յ�
	//	��Ϊrecommend_T
	float recommend_T = dp_speed_t(dp_speed_end);
	//qp�Ĺ�ģӦ����dp����ЧԪ�صĸ��� + �滮���
	int qp_size = (dp_speed_end +1)+ 1;
	//������Լ�������ʼ��������ĵ�ʽԼ��Ӧ���� Aeq'*X == beq ��ת��
	MatrixXd Aeq(3 * qp_size, 2 * qp_size - 2);
	Aeq.fill(0);
	VectorXd beq(2 * qp_size - 2);
	beq.fill(0);
	//����ʽԼ����ʼ��
	VectorXd lb(3 * qp_size);
	lb.fill(1);
	VectorXd ub(3 * qp_size);
	ub.fill(1);
	//����������ʱ��dt
	float dt = recommend_T / (dp_speed_end+1);
	MatrixXd A_sub(6,2);
	A_sub << 1, 0,
			dt, 1,
			(1.0 / 3)*pow(dt, 2), (1 * dt / 2),
			-1, 0,
			0, -1,
			(1.0 / 6)*pow(dt, 2), dt / 2;
	for (int i = 0; i < qp_size - 1; i++)
		Aeq.block<6, 2>(3 *(i+1)-3, 2 *(i+1)-2) = A_sub;
	//�����ʼ������������Լ����Ҳ���� s(i) - s(i+1) <= 0
	MatrixXd A(qp_size - 1, 3 * qp_size);
	A.fill(0);
	VectorXd b(qp_size - 1);
	b.fill(0);

	for (int i = 0; i < qp_size - 1; i++)
	{
		A(i, 3 * (i + 1) - 3) = 1;
		A(i, 3 * (i + 1)) = -1;
	}
	/*�������ɵ�͹�ռ�Լ��s_lb s_ub������㣬����lb(i) = s_lb(i - 1) �Դ�����
		������С���ٶ�Ϊ - 6 �����ٶ�Ϊ4(���ڳ�������ѧ)*/
	for (int i = 1; i < qp_size; i++)
	{
		lb(3 * (i+1) - 3) = s_lb(i-1);
		lb(3 * (i+1) - 2) = s_dot_lb(i-1);
		lb(3 * (i+1)-1) = -6;
		ub(3 * (i+1) - 3) = s_ub(i-1);
		ub(3 * (i+1) - 2) = s_dot_ub(i-1);
		ub(3 * (i+1)-1) = 4;
	}
	//���Լ��
	lb(0) = 0;
	lb(1) = plan_start_s_dot;
	lb(2) = plan_start_s_dot2;
	ub(0) = lb(0);
	ub(1) = lb(1);
	ub(2) = lb(2);
	// ���ٶȴ��� jerk���� �Լ��Ƽ��ٶȴ���
	MatrixXd A_s_dot2(3 * qp_size, 3 * qp_size);
	A_s_dot2.fill(0);
	MatrixXd A_jerk(3 * qp_size, qp_size - 1);
	A_jerk.fill(0);
	MatrixXd A_ref(3 * qp_size, 3 * qp_size);
	A_ref.fill(0);
	VectorXd A4_sub(6);
	A4_sub << 0, 0, 1, 0, 0, -1;
	for (int i = 0; i < qp_size; i++)
	{
		A_s_dot2(3 * (i+1)-1, 3 * (i+1)-1) = 1;
		A_ref(3 * (i+1) - 2, 3 *(i+1) - 2) = 1;
	}
	for (int i = 0; i < qp_size - 1; i++)
		A_jerk.block<6,1>(3*(i+1)-3,i)=A4_sub;
	//����H f
	MatrixXd H = w_cost_s_dot2 * (A_s_dot2 * A_s_dot2.transpose()) + w_cost_jerk * (A_jerk * A_jerk.transpose()) + w_cost_v_ref * (A_ref * A_ref.transpose());
	H = 2 * H;
	VectorXd f(3 * qp_size);
	f.fill(0);
	for(int i = 0;i<qp_size;i++)
		f(3 * (i+1) - 2) = -2 * w_cost_v_ref * speed_reference;
	//cout << f;
	//Լ��ת��
	MatrixXd linearMatrix(11*qp_size-5,3*qp_size);
	MatrixXd I = MatrixXd::Identity(3 * qp_size, 3 * qp_size);
	linearMatrix.block(0,0,qp_size - 1, 3 * qp_size) = A;   //λ�ã���С
	linearMatrix.block(qp_size - 1,   0, 3 * qp_size, 3 * qp_size)= I;
	linearMatrix.block(4*qp_size - 1, 0, 3 * qp_size, 3 * qp_size) = -I;
	linearMatrix.block(7 * qp_size - 1, 0, 2 * qp_size-2, 3 * qp_size) =Aeq.transpose();
	linearMatrix.block(9 * qp_size - 3, 0, 2 * qp_size - 2, 3 * qp_size) = -Aeq.transpose();
	//cout << linearMatrix.row(120);
	//SparseMatrix<double>linear=linearMatrix.sparseView();
	//SparseMatrix<double>hessian = H.sparseView();
	SparseMatrix<double>linear(linearMatrix.rows(), linearMatrix.cols());
	//linear.resize(linearMatrix.rows(), linearMatrix.cols());
	for (int i = 0; i < linearMatrix.rows(); i++)
		for (int j = 0; j < linearMatrix.cols(); j++)
			linear.insert(i, j) = linearMatrix(i, j);
	SparseMatrix<double>hessian(H.rows(), H.cols());
	//hessian.resize(H.rows(),H.cols());
	for (int i = 0; i < H.rows(); i++)
		for (int j = 0; j < H.cols(); j++)
			hessian.insert(i, j) = H(i, j);
	//�Ͻ�
	VectorXd upperBound(11*qp_size-5);
	upperBound.block(0, 0, qp_size - 1, 1) = b;
	upperBound.block(qp_size - 1, 0, 3*qp_size , 1) = ub;
	upperBound.block(4*qp_size - 1, 0, 3 * qp_size, 1) = -lb;
	upperBound.block(7*qp_size - 1, 0, 2* qp_size-2, 1) = beq;
	upperBound.block(9 * qp_size - 3, 0, 2 * qp_size - 2, 1) = -beq;

	//�½�Լ��
	VectorXd lowerBound(11 * qp_size - 5);
	lowerBound.fill(-OsqpEigen::INFTY);
	//������ι滮
	//cout << "upper" << upperBound << endl << endl;
	VectorXd X(3 * qp_size); X= osqp(hessian, f, linear, lowerBound, upperBound);
	//��������
	//cout << "x:"<<X<<endl<<endl;
	for (int i = 0; i < qp_size ; i++)
	{
		qp_s_init(i) = X(3 * (i+1) - 3);
		qp_s_dot_init(i) = X(3 *(i+1) - 2);
		qp_s_dot2_init(i) = X(3 * (i+1)-1);
		relative_time_init(i) = i*dt;
	}
	//��qp_s_init(0)���ô���0
	if (qp_s_init(0) < 0)
		qp_s_init(0) = 0;
	if (qp_s_dot_init(0) < 0)
		qp_s_dot_init(0) = 0;
	
	//改负数

	for(int i=0;i<qp_s_init.size();i++)
	{
		if(qp_s_init(i)<0)
		qp_s_init(i) = 0;
		if(qp_s_dot_init(i)<0)
		qp_s_dot_init(i) = 0;
	}
	//修倒排

	for(int i=qp_s_init.size()-2;i>0;i--)
	{
		if(qp_s_init(i)>qp_s_init(i+1)&&qp_s_init(i)!=0)
			if(qp_s_init(i-1)>qp_s_init(i))//倒排3个
				{
					double k;
					k=qp_s_init(i+1);
					qp_s_init(i+1)=qp_s_init(i-1);
					qp_s_init(i-1)=k;
				}
			else
				qp_s_init(i)=(qp_s_init(i-1)+qp_s_init(i+1))/2;

	}
	
	//改0
	for(int i=qp_s_init.size()-2;i>0;i--)
	{
		if(qp_s_init(i)==0)
			qp_s_init(i)=(qp_s_init(i-1)+qp_s_init(i+1))/2;
	}



	// for(int i=1;i<qp_s_dot_init.size();i++)
	// {
	// 	if(qp_s_dot_init(i)<qp_s_dot_init(i-1))
	// 		qp_s_dot_init(i)=(qp_s_dot_init(i-1)+qp_s_dot_init(i+1))/2;

	// }










	qp_s_init_out = qp_s_init;
	qp_s_dot_init_out = qp_s_dot_init;
	qp_s_dot2_init_out = qp_s_dot2_init;
	relative_time_init_out = relative_time_init;
}

