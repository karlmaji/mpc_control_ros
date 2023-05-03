#include "mpc_control_fix.h"

using namespace MPC_CONTROL_FIX;
/*



*/
bool MPC_Control::getParamYamlFromRos(const std::string &name_node)
{

    float map_x_max,map_x_min,map_y_max,map_y_min,sita_max,sita_min;
    float v_max,v_min,sita_a_max,sita_a_min;
    float Q_x,Q_y,Q_sita,R_v,R_sita_a,T_v,T_sita_a;
    float Q_weight_decay;
    int mpc_window,control_window;
    double delta_T;
    ros::NodeHandle n(name_node);
    n.param<float>("map_x_max",map_x_max,10);
    n.param<float>("map_x_min",map_x_min,0);
    n.param<float>("map_y_max",map_y_max,10);
    n.param<float>("map_y_min",map_y_min,0);
    n.param<float>("sita_max",sita_max,OsqpEigen::INFTY);
    n.param<float>("sita_min",sita_min,-OsqpEigen::INFTY);
    n.param<float>("v_max",v_max,1);
    n.param<float>("v_min",v_min,-1);
    n.param<float>("sita_a_max",sita_a_max,M_PI/2);
    n.param<float>("sita_a_min",sita_a_min,-M_PI/2);
    n.param<float>("Q_x",Q_x,10);
    n.param<float>("Q_y",Q_y,10);
    n.param<float>("Q_sita",Q_sita,1);
    n.param<float>("R_v",R_v,1);
    n.param<float>("R_sita_a",R_sita_a,0.1);
    n.param<float>("T_v",T_v,0);
    n.param<float>("T_sita_a",T_sita_a,0);
    n.param<float>("Q_weight_decay",Q_weight_decay,1);
    n.param<int>("mpc_window",mpc_window,10);
    n.param<int>("control_window",control_window,10);
    n.param<double>("delta_T",delta_T,0.1);

    // if(n.hasParam("map_x_max")) n.getParam("map_x_max",map_x_max); else map_x_max =10;
    // if(n.hasParam("map_x_min")) n.getParam("map_x_min",map_x_min); else map_x_min =0;
    // if(n.hasParam("map_y_max")) n.getParam("map_y_max",map_y_max); else map_y_max =10;
    // if(n.hasParam("map_y_min")) n.getParam("map_y_min",map_y_min); else map_y_min =0;
    // if(n.hasParam("sita_max")) n.getParam("sita_max",sita_max); else sita_max =OsqpEigen::INFTY;
    // if(n.hasParam("sita_min")) n.getParam("sita_min",sita_min); else sita_min =-OsqpEigen::INFTY;
    // if(n.hasParam("v_max")) n.getParam("v_max",v_max); else v_max =1;
    // if(n.hasParam("v_min")) n.getParam("v_min",v_min); else v_min =-1;
    // if(n.hasParam("sita_a_max")) n.getParam("sita_a_max",sita_a_max); else sita_a_max = M_PI/2;
    // if(n.hasParam("sita_a_min")) n.getParam("sita_a_min",sita_a_min); else sita_a_min =-M_PI/2;
    // if(n.hasParam("Q_x")) n.getParam("Q_x",Q_x); else Q_x =10;
    // if(n.hasParam("Q_y")) n.getParam("Q_y",Q_y); else Q_y =10;
    // if(n.hasParam("Q_sita")) n.getParam("Q_sita",Q_sita); else Q_sita =1;
    // if(n.hasParam("R_v")) n.getParam("R_v",R_v); else R_v =1;
    // if(n.hasParam("R_sita_a")) n.getParam("R_sita_a",R_sita_a); else R_sita_a =0.1;
    // if(n.hasParam("T_v")) n.getParam("T_v",T_v); else T_v =0;
    // if(n.hasParam("T_sita_a")) n.getParam("T_sita_a",T_sita_a); else T_sita_a = 0;
    // if(n.hasParam("Q_weight_decay")) n.getParam("Q_weight_decay",Q_weight_decay); else Q_weight_decay =1;
    // if(n.hasParam("mpc_window")) n.getParam("mpc_window",mpc_window); else mpc_window = 10;
    // if(n.hasParam("delta_T")) n.getParam("delta_T",delta_T); else delta_T = 0.1;

    this->_xMax<<map_x_max,map_y_max,sita_max;
    this->_xMin<<map_x_min,map_y_min,sita_min;
    this->_uMax<<v_max,sita_a_max;
    this->_uMin<<v_min,sita_a_min;
  
    this->_Q.diagonal()<<Q_x,Q_y,Q_sita; 
    this->_R.diagonal()<<R_v,R_sita_a;
    this->_T.diagonal()<<T_v,T_sita_a;
    this->_Q_weight_decay = Q_weight_decay;

    this->_mpc_window = mpc_window;
    this->_control_window = control_window;
    this->_delta_T = delta_T;

    

    return 0;
}

void MPC_Control::setDynamicsMatrices(Eigen::Matrix<double, 3, 3> &a, Eigen::Matrix<double, 3, 2> &b,const Eigen::Matrix<double,3,1> &x0,const Eigen::Matrix<double,2,1>&u0)
{
    a<< 1.,0.,-sin(x0(2,0))*u0(0,0)*this->_delta_T,
        0.,1.,cos(x0(2,0))*u0(0,0)*this->_delta_T,
        0.,0.,1.;
    
    b<<cos(x0(2,0)) * this->_delta_T  , 0.,
       sin(x0(2,0)) * this->_delta_T  , 0.,
       0.                                  ,this->_delta_T;

}

void MPC_Control::castMPCToQPHessian(const Eigen::DiagonalMatrix<double, 3> &Q, const Eigen::DiagonalMatrix<double, 2> &R,const Eigen::DiagonalMatrix<double, 2> &T, int mpcWindow,
                        Eigen::SparseMatrix<double> &hessianMatrix,double (*weight_func)(double,int))
{
    hessianMatrix.resize(3*(mpcWindow+1) + 2 * mpcWindow, 3*(mpcWindow+1) + 2 * mpcWindow);
    //populate hessian matrix
    for(int i = 0; i<3*(mpcWindow+1) + 2 * mpcWindow; i++){
        if(i < 3*(mpcWindow+1)){
            int posQ=i%3;
            int posQ_id=i/3;
            double weight = (*weight_func)(this->_Q_weight_decay,posQ_id);
            float value = weight * Q.diagonal()[posQ];
            if(value != 0)
                hessianMatrix.insert(i,i) = value;
        }
        else{
            int posR=(i-3*(mpcWindow+1))%2;
            float value = R.diagonal()[posR] + T.diagonal()[posR];
            if(value != 0)
                hessianMatrix.insert(i,i) = value;
        }
    }
    
}

void MPC_Control::castMPCToQPGradient(const Eigen::DiagonalMatrix<double, 3> &Q, const Eigen::DiagonalMatrix<double, 2> &T,const std::vector<Eigen::Matrix<double, 3, 1>> &xRef, 
                        const std::vector<Eigen::Matrix<double,2,1>> &uRef,int mpcWindow,
                        Eigen::VectorXd &gradient,double (*weight_func)(double,int))
{   
    std::vector<Eigen::Matrix<double,3,1>> Qx_ref;
    for(int i=0;i<mpcWindow+1;i++)
    {
        Qx_ref.push_back(Q * (-xRef.at(i)) * (*weight_func)(this->_Q_weight_decay,i));
    }
    std::vector<Eigen::Matrix<double,2,1>> Tu_ref;
    for(int i=0;i<mpcWindow;i++)
    {
        Tu_ref.push_back(T * (-uRef.at(i)));
    }

    gradient = Eigen::VectorXd::Zero(3*(mpcWindow+1)+2*(mpcWindow),1);
    for(int i = 0;i<3*(mpcWindow+1)+2*(mpcWindow);i++)
    {
        if(i<3*(mpcWindow+1))
        {
            int posVq = i / 3;
            int posq = i % 3;

            float value = Qx_ref.at(posVq)(posq,0);
            gradient(i,0) = value;
        }
        else
        {
            int posVq = (i-3*(mpcWindow+1)) / 2;
            int posq = (i-3*(mpcWindow+1)) % 2;
            float value = Tu_ref.at(posVq)(posq,0);
            gradient(i,0) = value;
        }    
    }
}



void MPC_Control::castMPCToQPConstraintMatrix(const Eigen::Matrix<double, 3, 3> &dynamicMatrix, const Eigen::Matrix<double, 3, 2> &controlMatrix,
                                 int mpcWindow, int controlWindow , Eigen::SparseMatrix<double> &constraintMatrix)
{
    constraintMatrix.resize(3*(mpcWindow+1)  + 3*(mpcWindow+1) + 2 * mpcWindow, 3*(mpcWindow+1) + 2 * mpcWindow);

    // populate linear constraint matrix
    for(int i = 0; i<3*(mpcWindow+1); i++){
        constraintMatrix.insert(i,i) = -1;
    }

    for(int i = 0; i < mpcWindow; i++)
        for(int j = 0; j<3; j++)
            for(int k = 0; k<3; k++){
                float value = dynamicMatrix(j,k);
                if(value != 0){
                    constraintMatrix.insert(3 * (i+1) + j, 3 * i + k) = value;
                }
            }

    for(int i = 0; i < mpcWindow; i++)
        for(int j = 0; j < 3; j++)
            for(int k = 0; k < 2; k++){
                float value = controlMatrix(j,k);
                if(value != 0){
                    constraintMatrix.insert(3*(i+1)+j, 2*i+k+3*(mpcWindow + 1)) = value;
                }
            }

    for(int i = 0; i<3*(mpcWindow+1) + 2*mpcWindow; i++){
        constraintMatrix.insert(i+(mpcWindow+1)*3,i) = 1;
    }

    for(int i=0;i<2*(mpcWindow-controlWindow);i++)
    {
        constraintMatrix.insert(i+(mpcWindow+1)*3+(mpcWindow+1)*3 + 2*(controlWindow) ,i+3*(mpcWindow+1)+2*(controlWindow-1))=-1;

    }





}

void MPC_Control::castMPCToQPConstraintVectors(const Eigen::Matrix<double, 3, 1> &xMax, const Eigen::Matrix<double, 3, 1> &xMin,
                                   const Eigen::Matrix<double, 2, 1> &uMax, const Eigen::Matrix<double, 2, 1> &uMin,
                                   const Eigen::Matrix<double, 3, 1> &x0,
                                   int mpcWindow, int controlWindow,Eigen::VectorXd &lowerBound, Eigen::VectorXd &upperBound)
{
    // evaluate the lower and the upper inequality vectors
    Eigen::VectorXd lowerInequality = Eigen::MatrixXd::Zero(3*(mpcWindow+1) +  2 * mpcWindow, 1);
    Eigen::VectorXd upperInequality = Eigen::MatrixXd::Zero(3*(mpcWindow+1) +  2 * mpcWindow, 1);
    
    Eigen::Matrix<double, 3, 1> x_min = xMin -x0;
    Eigen::Matrix<double, 3, 1> x_max = xMax -x0;


    for(int i=0; i<mpcWindow+1; i++){
        lowerInequality.block(3*i,0,3,1) = x_min;
        upperInequality.block(3*i,0,3,1) = x_max;
    }
    for(int i=0; i<mpcWindow; i++){
        lowerInequality.block(2 * i + 3 * (mpcWindow + 1), 0, 2, 1) = uMin;
        upperInequality.block(2 * i + 3 * (mpcWindow + 1), 0, 2, 1) = uMax;
    }
    // for(int i=0;i<2*(mpcWindow-controlWindow);i++)
    // {
    //     lowerInequality(i + 2*controlWindow+3*(mpcWindow+1)) = 0;
    // }

    // evaluate the lower and the upper equality vectors
    Eigen::VectorXd lowerEquality = Eigen::MatrixXd::Zero(3*(mpcWindow+1),1 );
    Eigen::VectorXd upperEquality;
    lowerEquality.block(0,0,3,1) = x0 - x0;  // x0 - x0 
    upperEquality = lowerEquality;
    

    // merge inequality and equality vectors
    lowerBound = Eigen::MatrixXd::Zero(2*3*(mpcWindow+1) +  2*mpcWindow,1 );
    lowerBound << lowerEquality,
        lowerInequality;

    upperBound = Eigen::MatrixXd::Zero(2*3*(mpcWindow+1) +  2*mpcWindow,1 );
    upperBound << upperEquality,
        upperInequality;
}


bool MPC_Control::init(const std::string &name_node)
{
    this->getParamYamlFromRos(name_node);
    
    this->_solver.settings()->setWarmStart(true);
    this->_solver.data()->setNumberOfVariables(3 * (this->_mpc_window + 1) + 2 * this->_mpc_window);
    this->_solver.data()->setNumberOfConstraints(2 * 3 * (this->_mpc_window + 1) + 2 * this->_mpc_window);
    
    
    this->castMPCToQPHessian(this->_Q,this->_R,this->_T,this->_mpc_window,this->_hessian);
    

    return 0;






}




bool MPC_Control::step(const Eigen::Matrix<double,3,1>&x0,const Eigen::Matrix<double,2,1>&u0,const std::vector<Eigen::Matrix<double,3,1>> &xref,
                         const std::vector<Eigen::Matrix<double,2,1>> &uref)
{   
    this->setDynamicsMatrices(this->_a,this->_b,x0,u0);
    this->castMPCToQPConstraintMatrix(this->_a,this->_b,this->_mpc_window,this->_control_window,this->_linearMatrix);
    this->castMPCToQPConstraintVectors(this->_xMax,this->_xMin,this->_uMax,this->_uMin,
                                       x0,
                                       this->_mpc_window,this->_control_window,this->_lowerBound,this->_upperBound);
    
    this->castMPCToQPGradient(this->_Q,this->_T,xref,uref,this->_mpc_window,this->_gradient);

    if(this->_solver.data()->isSet())
    {
        if(!this->_solver.updateBounds(this->_lowerBound,this->_upperBound)) return 1;
        if(!this->_solver.updateGradient(this->_gradient)) return 1;
        if(!this->_solver.updateHessianMatrix(this->_hessian)) return 1;
        if(!this->_solver.updateLinearConstraintsMatrix(this->_linearMatrix))return 1;


    }
    else
    {
    if(!this->_solver.data()->setHessianMatrix(this->_hessian)) return 1;
    if(!this->_solver.data()->setGradient(this->_gradient)) return 1;
    if(!this->_solver.data()->setLinearConstraintsMatrix(this->_linearMatrix)) return 1;
    if(!this->_solver.data()->setLowerBound(this->_lowerBound)) return 1;
    if(!this->_solver.data()->setUpperBound(this->_upperBound)) return 1;
    if(!this->_solver.initSolver()) return 1;
    }

    



    // solve the QP problem
    if(this->_solver.solveProblem() != OsqpEigen::ErrorExitFlag::NoError) return 1;

    // get the controller input
    this->QPSolution = this->_solver.getSolution();
    this->QPctr = this->QPSolution.block(3 * (this->_mpc_window+1),0,2,1);
    
    return 0;
}


