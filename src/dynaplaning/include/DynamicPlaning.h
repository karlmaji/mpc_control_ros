#pragma once
#include <iostream>
#include <Eigen/Dense>
#include "OsqpEigen/OsqpEigen.h"
using namespace std;
using namespace Eigen;
#include"Sensor.h"
#include"car.h"

struct Trajectory
{
    float x;
    float y;
    float heading;
    float vx;
    float vy;
    float ax;
    float ay;
    float k;
    float time;
};
struct Sample
{
  float s;
  float l;
  int last_index;
  float min_cost;
};

class DynaPlaning
{
  public:
     DynaPlaning(Obstacle obst,HostCar carp,int num,float sp_l,float sp_s,float w_coll,float w_ref,float w_smooth[3],float q_w_mid,float q_w_ref,float q_w_smooth[3],float pl_s);
     Trajectory GetStartPlaningPoint();
     Frenet CarToFre(Trajectory trj);
     void DynamicProgramming();
     float gettime(){return 0;}
     void CalculateQuinticPolynomial(double* para,float start_s,float start_l,float start_dl,float start_ddl,float end_s,float end_l);
     float CalculateCost(double* para,float start_s,Frenet* valid_obs,int valid_num);
     float CalculateCollisionCost(float s,float l,Frenet* valid_obs,int valid_num);
     void CalculateMinCost(Sample* dp_point,Frenet* valid_obs,int valid_num,int index_s,int index_l,double* para);
     void AddPoint(int* best_index,double* para,Sample* dp_point,Frenet origin);

     int SecondaryPlanning(Frenet* valid_obs,Obs* valid_obsf,int valid_num);
     void GetConvexSpace(Frenet* valid_obs,Obs* valid_obsf,int valid_num);
     void GetValue(float* ub,float* lb);
     void FreToCarte();
     Reference GetFTCproj(float s);
     void gethessian(int* r,int* c,double* val);
     ~DynaPlaning();

    Trajectory* GetTraj(){return traj;}
     
     Obstacle dp_obs;
     HostCar dp_car;
     Trajectory* traj;
     Frenet dynamic_point[61];
     Frenet dynamic_point_f[241];
     float para_f[10][6];
     int traj_num;
     int montage_index;
     float sample_l;
     float sample_s;
     float road_lu;
     float road_ld;
     float plan_s;
     float plan_s_local;
     float dp_cost_collision;
     float dp_cost_ref;
     float dp_cost_smooth[3];
     float qp_cost_ref;
     float qp_cost_mid;
     float qp_cost_smooth[3];
};