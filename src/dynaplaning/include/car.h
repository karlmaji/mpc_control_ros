#pragma once
#include <iostream>
using namespace std;
#include"transform.h"

struct Car
{
    float car_center_x; 
    float car_center_y; 
    float car_velocity;
    float car_hesding;
    float car_acc;
    float car_k;
    float car_length;
    float car_width;
};

class HostCar:public Coordinate
{
 public:
    HostCar(Car car_p,Reference* p,int len);
    void CarteToFre();
    ~HostCar(){};
    Car car;
    Frenet car_frenet;
};