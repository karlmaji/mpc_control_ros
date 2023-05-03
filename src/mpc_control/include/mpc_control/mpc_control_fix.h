#ifndef MPC_CONTROL_FIX_H
#define MPC_CONTROL_FIX_H
// osqp-eigen
#include "OsqpEigen/OsqpEigen.h"
#include "ros/ros.h"
// eigen
#include <Eigen/Dense>

namespace MPC_CONTROL_FIX
{
class MPC_Control
{
    protected:
        //-----------通过yaml传入的参数-------------//
        Eigen::Matrix<double,3,1> _xMax;
        Eigen::Matrix<double,3,1> _xMin;
        Eigen::Matrix<double,2,1> _uMax;
        Eigen::Matrix<double,2,1> _uMin;


        Eigen::DiagonalMatrix<double, 3> _Q;
        Eigen::DiagonalMatrix<double, 2> _R;
        Eigen::DiagonalMatrix<double, 2> _T;

        //Qn =Q * weight_decay^n
        double _Q_weight_decay=1;
        
         
        
        /*
        //----------------------------------------//
        //动态转移矩阵A
        Eigen::Matrix<double, 3, 3> _a;
        //动态转移矩阵B
        Eigen::Matrix<double, 3, 2> _b;
        */


        //------------需要输入的动态赋值的参数-------------//
        /*
        Eigen::Matrix<double,3,1> _x0;

        std::vector<Eigen::Matrix<double,3,1>> _xref;

        std::vector<Eigen::Matrix<double,2,1>> _uref;
        */

        //------------运算中的动态参数------------//
        Eigen::SparseMatrix<double> _hessian;

        Eigen::VectorXd _gradient;

        Eigen::SparseMatrix<double> _linearMatrix;

        Eigen::VectorXd _lowerBound;
        Eigen::VectorXd _upperBound;

        OsqpEigen::Solver _solver;
    public:
                


        //离散时间deltaT
        double _delta_T=0.1;
        //MPC预测点数
        int _mpc_window = 4;
        int _control_window =4;
        //动态转移矩阵A
        Eigen::Matrix<double, 3, 3> _a;
        //动态转移矩阵B
        Eigen::Matrix<double, 3, 2> _b;
        //所有状态量及控制量的求解结果
        Eigen::VectorXd QPSolution;
        //求解的控制量结果u0
        Eigen::Vector2d QPctr;

        
        //bool getParamYamlFromRos(ros::NodeHandle &n);
        bool getParamYamlFromRos(const std::string &name_node);
        //设定 动态转移矩阵A B
        void setDynamicsMatrices(Eigen::Matrix<double, 3, 3> &a, Eigen::Matrix<double, 3, 2> &b,const Eigen::Matrix<double,3,1> &x0,const Eigen::Matrix<double,2,1>&u0);
        //设定P Hession
        void castMPCToQPHessian(const Eigen::DiagonalMatrix<double, 3> &Q, const Eigen::DiagonalMatrix<double, 2> &R,const Eigen::DiagonalMatrix<double, 2> &T ,int mpcWindow,
                        Eigen::SparseMatrix<double> &hessianMatrix,double (*weight_func)(double,int)=[](double weight_decay,int n){return std::pow(weight_decay,n);});
        //设定 q gradient vector
        void castMPCToQPGradient(const Eigen::DiagonalMatrix<double, 3> &Q, const Eigen::DiagonalMatrix<double, 2> &T,const std::vector<Eigen::Matrix<double, 3, 1>> &xRef, const std::vector<Eigen::Matrix<double,2,1>> &uRef,int mpcWindow,
                         Eigen::VectorXd &gradient,double (*weight_func)(double,int)=[](double weight_decay,int n){return std::pow(weight_decay,n);});
        //设定Ac  The linear constraint matrix
        void castMPCToQPConstraintMatrix(const Eigen::Matrix<double, 3, 3> &dynamicMatrix, const Eigen::Matrix<double, 3, 2> &controlMatrix,
                                 int mpcWindow, int controlWindow,Eigen::SparseMatrix<double> &constraintMatrix);
        //设定upper and lower bound
        void castMPCToQPConstraintVectors(const Eigen::Matrix<double, 3, 1> &xMax, const Eigen::Matrix<double, 3, 1> &xMin,
                                   const Eigen::Matrix<double, 2, 1> &uMax, const Eigen::Matrix<double, 2, 1> &uMin,
                                   const Eigen::Matrix<double, 3, 1> &x0,
                                   int mpcWindow, int controlWindow,Eigen::VectorXd &lowerBound, Eigen::VectorXd &upperBound);
        /*初始化*/
        bool init(const std::string &name_node);
        //求解
        bool step(const Eigen::Matrix<double,3,1>&x0,const Eigen::Matrix<double,2,1>&u0,const std::vector<Eigen::Matrix<double,3,1>> &xref,const std::vector<Eigen::Matrix<double,2,1>> &uref);




};
}

#endif