#include <mpc_control/mpc_control.h>
using namespace MPC_CONTROL;
int main(int argc, char **argv)
{
    MPC_Control mpc;
    ros::init(argc,argv,"mpc_test");
    ros::NodeHandle n;
    // mpc._mpc_window=4;
    // Eigen::DiagonalMatrix<double, 3> Q;
    // Eigen::Matrix<double, 3, 1> x_ref;
    // Eigen::Matrix<double,2,1> u_ref;

    // Eigen::DiagonalMatrix<double, 2> R;
    // std::vector<Eigen::Matrix<double, 3, 1>> xRef;
    // std::vector<Eigen::Matrix<double,2,1>> uRef;
    // int mpcWindow=2;
    // Eigen::VectorXd gradient;

    // Q.diagonal() << 1,2,3;
    // R.diagonal() << 1,2;
    // x_ref << 1,1,1;
    // u_ref << 1,1;
    // for(int i=0;i<mpcWindow+1;i++)
    // {xRef.push_back(x_ref);}
    // for(int i=0;i<mpcWindow;i++)
    // {uRef.push_back(u_ref);}
    

    // mpc.castMPCToQPGradient(Q,R,xRef,uRef,mpcWindow,gradient);

    // std::cout << gradient<<std::endl;
    // std::cout << "-----------"<<std::endl;


    // Eigen::SparseMatrix<double> linearMatrix;
    // Eigen::Matrix<double, 3, 3> a;
    // Eigen::Matrix<double, 3, 2> b;

    // a<< 1.,0.,0.,
    //     0.,1.,0.,
    //     0.,0.,1.;
    // b<< 1.,0.,
    //     1.,0.,
    //     0.,1.;

    // mpc.castMPCToQPConstraintMatrix(a,b,mpcWindow,linearMatrix);

    // std::cout << linearMatrix<<std::endl;
    // std::cout << "-----------"<<std::endl;



    // Eigen::Matrix<double, 3, 1> xMax;
    // Eigen::Matrix<double, 3, 1> xMin;
    // Eigen::Matrix<double, 2, 1> uMax;
    // Eigen::Matrix<double, 2, 1> uMin;
    // Eigen::Matrix<double, 3, 1> x0;
    // Eigen::VectorXd lowerBound;
    // Eigen::VectorXd upperBound;
    // xMax<<10,10,M_PI;
    // xMin<<0,0,-M_PI;
    // uMax<<2,1;
    // uMin<<-2,-1;
    // x0<<0,0,0;
    // mpc.castMPCToQPConstraintVectors(xMax,xMin,uMax,uMin,x0,mpcWindow,lowerBound,upperBound);

    // std::cout << lowerBound <<std::endl;
    // std::cout << upperBound <<std::endl;
    // std::cout << "-----------"<<std::endl;



    mpc._delta_T =0.1 ;
    mpc._mpc_window = 4;
    mpc.init("mpc_test");
    Eigen::Matrix<double,3,1> x0_;
    std::vector<Eigen::Matrix<double,3,1>> xref;
    std::vector<Eigen::Matrix<double,2,1>> uref;
    Eigen::Vector2d ctr;
    x0_<< 0,0, 0;
    
        // number of iteration steps
    int numberOfSteps = 10;
    for(int i=0;i<mpc._mpc_window+1;i++)
    {   
        Eigen::Matrix<double,3,1> x_iter;
        x_iter<<4*sin(0.01*i),4*(1-cos(0.01*i)),0.01*i; 
        xref.push_back(x_iter);
    }

    for(int i=0;i<mpc._mpc_window;i++)
    {
        Eigen::Matrix<double,2,1> u_iter;
        u_iter<<0.4,0;
        uref.push_back(u_iter);
    }


    for (int k = 0; k < numberOfSteps; k++)
    {
        Eigen::Matrix<double,3,1> x_iter;
        x_iter<<4*sin(0.01*(5+k)),4*(1-cos(0.01*(5+k))),0.01*(5+k); 

        xref.push_back(x_iter);

        

        if(!mpc.step(x0_,xref,uref))
        {
        ctr = mpc.QPSolution.block(3 * (mpc._mpc_window+1),0,2,1);
        //std::cout << x_iter <<std::endl;
        
        std::cout << ctr <<std::endl;
        x0_ = mpc._a * x0_ + mpc._b * ctr;
        std::cout << x0_ <<std::endl;
        }
        






        std::vector<Eigen::Matrix<double,3,1>>::iterator vit=xref.begin();
        xref.erase(vit);





        
    }








    return 0;


}
/*
4.51177e-08
4.51206e-08
   0.785398

    1.41436
    1.41436
     1.2705

    2.82867
    2.82867
     1.4553

    3.93981
    3.93981
     1.5246

    4.44907
    4.44907
     1.5477

    2.00021
   0.485099

    2.00013
     0.1848

    1.57139
  0.0692998

    0.72021
  0.0230999
  
  */

