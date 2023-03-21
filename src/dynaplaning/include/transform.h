#pragma once
#include <iostream>
using namespace std;
#include"math.h"
//参考线数据格式
struct Reference
{
 float x_r;
 float y_r;
 float heading_r;
 float s_r;
 float k_r;
 float k_rd;
};
//笛卡尔坐标系数据格式
struct Cartesian
{
 float x_x;
 float y_x;
 float heading_x;
 float v_x;
 float a_x;
 float k_x;
};
//自然坐标系数据格式
struct Frenet
{
 float s;
 float sd;
 float sdd;
 float l;
 float ld;
 float ldd;
};
//坐标转换类——抽象类
class Coordinate
{
    public:
      Coordinate(Reference* p,int len);
      //virtual void CarteToFre()=0;
      Reference getproj(float x,float y);
      int sign(float num);
      float getdistance(float x,float y,int index);
      ~Coordinate();
      Reference * referenceline; 
      int Point_num;
};