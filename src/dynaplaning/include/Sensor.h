#pragma once
#include <iostream>
using namespace std;
#include"transform.h"

struct Obs
{
    float obs_center_x; 
    float obs_center_y; 
    float obs_velocity;
    float obs_hesding;
    float obs_acc;
    float obs_k;
    float obs_length;
    float obs_width;
};

class Obstacle:public Coordinate
{
 public:
    Obstacle(Obs* obs_p,int num,Reference* p,int len);
    Obstacle(const Obstacle &obst);
    void CarteToFre();
    ~Obstacle();
    Obs * obs;
    int obs_num;
    Frenet* obs_frenet;
};