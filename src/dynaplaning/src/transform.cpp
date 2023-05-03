#include "transform.h"
using namespace std;
Coordinate::Coordinate(Reference* p,int len)
{
  referenceline=p;
  Point_num=len;
}
Reference Coordinate::getproj(float x,float y)//计算投影点
{
  Reference proj,match_point,match_pnext;
  int index=0;
  int start_index,end_index;
  int step=2;
  int num=this->Point_num/step;
  float min_distance=this->getdistance(x,y,index);//以第一个点距离最短开始
  float distance;
  int i;
  float ds;
  for(i=1;i<num+1;i++) //每step个点计算一次，找出最小的一个点——粗解
  {
    distance=this->getdistance(x,y,i*step);
    if(distance<min_distance)
    {
      index=i*step;
      if(index>=this->Point_num)
      {
         index=this->Point_num-1;
      }
      min_distance=distance;
    }
  }
  if(index==0)  //计算精确解范围
  {
    start_index=0;
    end_index=step;
  }
  else if(index==this->Point_num-1)
  {
    start_index=index-step;
    end_index=index;
  }
  else if(this->getdistance(x,y,index-1)<min_distance&&this->getdistance(x,y,index+1)>min_distance)
  {
    start_index=index-step;
    end_index=index;
  }
  else if(this->getdistance(x,y,index-1)>min_distance&&this->getdistance(x,y,index+1)<min_distance)
  {
    start_index=index;
    end_index=index+step;
    if(end_index>(this->Point_num-1))
       end_index=this->Point_num-1;
  }
  else
  {
    start_index=index-step;
    end_index=index+step;
    if(end_index>(this->Point_num-1))
       end_index=this->Point_num-1;
  }
  for(i=start_index;i<end_index;i++)//找出精确解
  {
    distance=this->getdistance(x,y,i);
    if(distance<min_distance)
    {
      index=i;
      min_distance=distance;
    }
  }
  

  match_point=*((this->referenceline)+index);//取出匹配点
  //cout<<"index: "<<index<<endl;

  ds=(x-match_point.x_r)*cos(match_point.heading_r)+(y-match_point.y_r)*sin(match_point.heading_r);//计算ds
  if(ds>0)//分情况使用公式计算投影点
  {
    if(index<this->Point_num-1)
    {
        match_pnext=*((this->referenceline)+index+1);
        proj.s_r=match_point.s_r+ds;
        proj.heading_r=match_point.heading_r;
        proj.k_r=(match_point.k_r+match_pnext.k_r)/2;
        proj.x_r=match_point.x_r+ds*cos(match_point.heading_r);
        proj.y_r=match_point.y_r+ds*sin(match_point.heading_r);
        proj.k_rd=(match_pnext.k_r-match_point.k_r)/(match_pnext.s_r-match_point.s_r);
    }
    else
    {
        match_pnext=*((this->referenceline)+index-1);
        proj=match_point;
        proj.k_rd=(match_point.k_r-match_pnext.k_r)/(match_point.s_r-match_pnext.s_r);
    }
  }
  else if(ds<0)
  { if(index>0)
    {
        match_pnext=*((this->referenceline)+index-1);
        proj.s_r=match_point.s_r+ds;
        proj.heading_r=match_point.heading_r;
        proj.k_r=(match_point.k_r+match_pnext.k_r)/2;
        proj.x_r=match_point.x_r+ds*cos(match_point.heading_r);
        proj.y_r=match_point.y_r+ds*sin(match_point.heading_r);
        proj.k_rd=(match_point.k_r-match_pnext.k_r)/(match_point.s_r-match_pnext.s_r);
    }
    else
    {
        match_pnext=*((this->referenceline)+index+1);
        proj=match_point;
        proj.k_rd=(match_pnext.k_r-match_point.k_r)/(match_pnext.s_r-match_point.s_r);
    }
  }
  else
  {
    if(index==0)
    {
      match_pnext=*((this->referenceline)+index+1);
      proj=match_point;
      proj.k_rd=(match_pnext.k_r-match_point.k_r)/(match_pnext.s_r-match_point.s_r);
    }
    else
    {
      match_pnext=*((this->referenceline)+index-1);
      proj=match_point;
      proj.k_rd=(match_point.k_r-match_pnext.k_r)/(match_point.s_r-match_pnext.s_r);
    }
  }

  return proj;
}

float Coordinate::getdistance(float x,float y,int index)
{
  float distance=(x-((this->referenceline)+index)->x_r)*(x-((this->referenceline)+index)->x_r)
                +(y-((this->referenceline)+index)->y_r)*(y-((this->referenceline)+index)->y_r);
  return distance;
}

int Coordinate::sign(float num)
{
  int flag;
  if(num>0)
      flag=1;
  if(num==0)
      flag=0;
  if(num<0)
      flag=-1;
  return flag;
}

Coordinate::~Coordinate()
{
  referenceline=nullptr;
  Point_num=0;  
}