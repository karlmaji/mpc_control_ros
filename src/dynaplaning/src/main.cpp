#include <iostream>
#include "OsqpEigen/OsqpEigen.h"
#include <Eigen/Dense>
#include "DynamicPlaning.h"
#include "car.h"
#include "transform.h"
#include "Sensor.h"
#include<fstream>  
#include <time.h>
using namespace std;
using namespace Eigen;

Trajectory traj[200];
int main()
{

    clock_t start, finish;
    double totaltime;
    start = clock(); 
    
    Reference referenceline[181];
    float w_smooth[3]={100000,5000,2000};
    float q_w_smooth[3]={1000,500,200};
    float dc=M_PI/360;
    for(int i=0;i<181;i++)
    {
      referenceline[i].heading_r=i*dc;
      referenceline[i].k_r=300;
      referenceline[i].k_rd=0;
      referenceline[i].s_r=i*300*dc;
      referenceline[i].x_r=300*sin(i*dc);
      referenceline[i].y_r=-300*cos(i*dc);  
    }
    Obs obs_p[6]; 
    Car car_p;
    car_p.car_acc=0;
    car_p.car_center_x=296*sin(178*dc);
    car_p.car_center_y=-296*cos(178*dc);
    car_p.car_hesding=178*dc;
    car_p.car_k=0;
    car_p.car_length=5;
    car_p.car_velocity=0;
    car_p.car_width=2.1;
    for(int i=0;i<6;i++)
    {
        obs_p[i].obs_acc=0;
        obs_p[i].obs_center_x=(295+2*i)*sin((32+4*i)*dc);
        obs_p[i].obs_center_y=-(295+2*i)*cos((32+4*i)*dc);
        obs_p[i].obs_hesding=0;
        obs_p[i].obs_k=0;
        obs_p[i].obs_length=2+0.2*i;
        obs_p[i].obs_velocity=0;
        obs_p[i].obs_width=2+0.2*i;
    }
    HostCar carp(car_p,referenceline,181);
    Obstacle obst(obs_p,3,referenceline,181);
    DynaPlaning DP(obst,carp,0,1,6,100,50,w_smooth,50,10,q_w_smooth,60);
    DP.traj=traj;
    DP.DynamicProgramming();
    
    finish = clock();
    totaltime = (double)(finish - start) / CLOCKS_PER_SEC;
    cout << "clock " << totaltime << "s" << endl;

    ofstream ofs; //打开一个写的文件流

	ofs.open("result.txt", ios::out);  //把这个流和文件关联起来

    for(int i=0;i<DP.plan_s+1;i++)
    {
	ofs <<DP.dynamic_point[i].l <<",";
    }
    ofs <<endl;
    for(int i=0;i<obst.obs_num;i++)
    {
      ofs <<(DP.dp_obs.obs_frenet+i)->s-78.5<<","
          <<(DP.dp_obs.obs_frenet+i)->l <<","
          <<(DP.dp_obs.obs+i)->obs_length <<","
          <<(DP.dp_obs.obs+i)->obs_width <<endl;  
    }
	for(int i=0;i<DP.plan_s+1;i++)
    {
	ofs <<DP.dynamic_point[i].sd <<",";
    }
    ofs <<endl;	
    for(int i=0;i<DP.plan_s+1;i++)
    {
	ofs <<DP.dynamic_point[i].sdd <<",";
    }
    ofs <<endl;
    for(int i=0;i<DP.plan_s+1;i++)
    {
	ofs <<traj[i].x <<",";
    }
    ofs <<endl;
    for(int i=0;i<DP.plan_s+1;i++)
    {
	ofs <<traj[i].y <<",";
    }
    ofs <<endl;
	ofs.close();   //操作完成记得close()关闭


    
    return 0;
}
