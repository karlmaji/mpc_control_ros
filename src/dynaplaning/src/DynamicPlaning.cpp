#include "DynamicPlaning.h"
using namespace std;

DynaPlaning::DynaPlaning(Obstacle obst,HostCar carp,int num,float sp_l,float sp_s,float w_coll,float w_ref,
                  float w_smooth[3],float q_w_mid,float q_w_ref,float q_w_smooth[3],float pl_s):dp_car(carp),dp_obs(obst)
{
    this->traj=nullptr;
    this->traj_num=num;
    this->montage_index=0;
    this->sample_l=sp_l;
    this->sample_s=sp_s;
    this->plan_s = pl_s;
    this->dp_cost_collision=w_coll;
    this->dp_cost_ref=w_ref;
    this->road_ld=-0.8;
    this->road_lu=0.8;
    this->scale = 100;
    for(int i=0;i<3;i++)
    {
        (this->dp_cost_smooth)[i]=w_smooth[i];
    }
    this->qp_cost_mid=q_w_mid;
    this->qp_cost_ref=q_w_ref;
    for(int i=0;i<3;i++)
    {
        (this->qp_cost_smooth)[i]=q_w_smooth[i];
    }
}

Trajectory DynaPlaning::GetStartPlaningPoint()
{
    int i,index,start_index;
    Trajectory start_point;
    float cur_time=this->gettime();
    float t_error,n_error,dx,dy,dt;
    Car car=this->dp_car.car;
    if(this->traj_num==0)
    {
         Reference proj=this->dp_car.getproj(car.car_center_x,car.car_center_y);
         start_point.x= car.car_center_x;
         start_point.y=car.car_center_y;
         start_point.vx=0;
         start_point.vy=0;
         start_point.ax=0;
         start_point.ay=0;
         start_point.k=proj.k_r;
         start_point.heading=car.car_hesding;
         start_point.time=cur_time+0.1;  
         this->montage_index=0;
    }
    else
    {
            for(i=0;i<this->traj_num;i++)//可优化
            {
                if(cur_time>((this->traj)+i)->time&&cur_time<((this->traj)+i+1)->time)
                {
                                index=i;
                                break;
                }
            }
            Trajectory target_point=*((this->traj)+index);
            dx=target_point.x-dp_car.car.car_center_x;
            dy=target_point.y-dp_car.car.car_center_y;
            t_error=dx*cos(target_point.heading)+dy*sin(target_point.heading);
            n_error=-dx*sin(target_point.heading)+dy*cos(target_point.heading);
            for(i=index;i<this->traj_num;i++)//可优化
            {
                    if(cur_time>(((this->traj)+i)->time+0.1)&&cur_time<(((this->traj)+i+1)->time+0.1))
                    {
                        start_index=i;
                        break;
                    }
            }
            if(t_error<2.5&&n_error<0.5)
            {
                start_point=*((this->traj)+start_index);
                if(start_index>=20)
                {
                   for(i=0;i<20;i++)
                   {
                     *((this->traj)+i)=*((this->traj)+start_index-20+i);
                   }
                   this->traj_num=20;
                   this->montage_index=20;
                }
                else
                {
                   this->traj_num=start_index;
                   this->montage_index=start_index;
                }
            }
            else
            {
            dt=0.1;
            start_point.x=car.car_center_x+car.car_velocity*cos(car.car_hesding)*dt+0.5*car.car_acc*cos(car.car_hesding)*dt*dt;
            start_point.y=car.car_center_y+car.car_velocity*sin(car.car_hesding)*dt+0.5*car.car_acc*sin(car.car_hesding)*dt*dt;
            start_point.vx=car.car_velocity*cos(car.car_hesding)+car.car_acc*cos(car.car_hesding)*dt;
            start_point.vy=car.car_velocity*sin(car.car_hesding)+car.car_acc*sin(car.car_hesding)*dt;
            start_point.ax=car.car_acc*cos(car.car_hesding);
            start_point.ay=car.car_acc*sin(car.car_hesding);
            start_point.k=car.car_k;
            start_point.heading=atan2(start_point.vy,start_point.vx);
            start_point.time=cur_time+0.1;
            this->montage_index=0;
            this->traj_num=0;
            }
    }
    return start_point;
}

Frenet DynaPlaning::CarToFre(Trajectory trj)
{
    Reference proj;
    Frenet start_f;
    float dx,dy,dh,s,sd,sdd,l,ld,ldd;
    sd=0;
    sdd=0;
    ld=0;
    ldd=0;
    proj=this->dp_car.getproj(trj.x,trj.y);
    dx=trj.x-proj.x_r;
    dy=trj.y-proj.y_r;
    
    dh=trj.heading-proj.heading_r;
    s=proj.s_r;
    l=(this->dp_car.sign(dy*cos(proj.heading_r)-dx*sin(proj.heading_r)))*sqrt(dx*dx+dy*dy);
    if(trj.vx!=0||trj.vy!=0||trj.ax!=0||trj.ay!=0)
    {
      ld=(1-proj.k_r*l)*tan(dh);
      ldd=-(proj.k_rd*l+proj.k_r*ld)*tan(dh)+(1-proj.k_r*l)*(((1-proj.k_r*l)*(trj.k)/cos(dh))-proj.k_r)/(cos(dh)*cos(dh));
      sd=((sqrt((trj.vx*trj.vx)+trj.vy*trj.vy))*cos(dh))/(1-proj.k_r*l);
      sdd=((sqrt((trj.ax*trj.ax)+trj.ay*trj.ay))*cos(dh)-sd*sd*(ld*(((1-proj.k_r*l)*(trj.k)/cos(dh))-proj.k_r)-(proj.k_rd*l+proj.k_r*ld)))/(1-proj.k_r*l);
    }
    start_f.s=s;
    start_f.l=l;
    start_f.ld=ld;
    start_f.ldd=ldd;
    start_f.sd=sd;
    start_f.sdd=sdd;
     
    return start_f;
}

void DynaPlaning::DynamicProgramming()
{
    this->dp_obs.CarteToFre();//障碍物坐标转换
    Trajectory start_point=this->GetStartPlaningPoint();//取得规划起点
    Frenet origin=this->CarToFre(start_point);  //规划起点坐标转换
    // std::cout<<origin.s<<' '<<origin.l<<' '<<origin.ld<<' '<<origin.ldd<<std::endl;
    // std::cout<<(this->dp_car.referenceline+this->dp_car.Point_num-1)->s_r<<std::endl;
    plan_s_local=(this->dp_car.referenceline+this->dp_car.Point_num-1)->s_r-origin.s;
    if(plan_s_local<sample_s)
    {
     for(int i=0;i<61;i++)
     {
      *(this->traj+i)=start_point;
     }
     this->traj_num=61;
    }
    else
    {
      if(plan_s_local>this->plan_s)
      {
        plan_s_local=this->plan_s;
      }
      else
      {
        plan_s_local=plan_s_local;
      }
      Frenet*  valid_obs=new Frenet [this->dp_obs.obs_num];//有效障碍物存储
      int valid_num=0;  //用于记录有效障碍物个数
      int best_index[int(plan_s_local/sample_s)];//
      int ret;
      double para[6];//
      int s_num=int(plan_s_local/sample_s);
      int l_num=int((road_lu-road_ld)/sample_l-1);
      Sample dp_point[s_num][l_num];//散点数组
      Frenet* all_obs=this->dp_obs.obs_frenet;//
      Obs* valid_obsf=new Obs [this->dp_obs.obs_num];//有效障碍物完整信息存储
      for(int i=0;i<this->dp_obs.obs_num;i++)//取出有效障碍物
      {
        if((all_obs+i)->l<(road_lu*1.5)&&(all_obs+i)->l>(road_ld*1.5)&&((all_obs+i)->s-origin.s)<plan_s_local)
        {
          if(((all_obs+i)->s-origin.s)>0&&(all_obs+i)->ld==0&&(all_obs+i)->sd==0)
          {
            *(valid_obs+valid_num)=*(all_obs+i);
            *(valid_obsf+valid_num)=*(dp_obs.obs+i);
            valid_num++;
            //cout<<valid_num<<endl;
          }
          //cout<<valid_num<<endl;
        }
      }
      if(valid_num==0)
      {
        dynamic_point[0]=origin;
        for(int i=0;i<60*plan_s_local/plan_s+1;i++) 
        {
          dynamic_point[i].s=origin.s+i*plan_s/60;
          dynamic_point[i].sd=road_lu;
          dynamic_point[i].sdd=road_ld;
        }
        ret=this->SecondaryPlanning(valid_obs,valid_obsf,valid_num);//二次规划
        if(plan_s_local<plan_s)
        {
          for(int i=60*plan_s_local/plan_s+1;i<61;i++)
          {
            dynamic_point[i].l=dynamic_point[int(60*plan_s_local/plan_s)].l;
            dynamic_point[i].ld=dynamic_point[int(60*plan_s_local/plan_s)].ld;
            dynamic_point[i].ldd=dynamic_point[int(60*plan_s_local/plan_s)].ldd;
            dynamic_point[i].s=dynamic_point[int(60*plan_s_local/plan_s)].s;
            dynamic_point[i].sd=dynamic_point[int(60*plan_s_local/plan_s)].sd;
            dynamic_point[i].sdd=dynamic_point[int(60*plan_s_local/plan_s)].sdd;
          }
          // std::cout<<int(60*plan_s_local/plan_s)<<std::endl;
          // std::cout<<plan_s_local<<std::endl;
        }
          // for(int i=0;i<61;i++)
          // {std::cout << "  "<<dynamic_point[i].s;
          // }
          // std::cout<< std::endl;
          if(!ret)
          {
            this->FreToCarte();
          } 
      }
      else
      {
          for(int j=0;j<s_num;j++)//初始化散点
          {
              for(int k=0;k<l_num;k++)
              {
                  dp_point[j][k].s=origin.s+sample_s*(j+1);
                  dp_point[j][k].l=road_lu-sample_l*(k+1);
                  dp_point[j][k].min_cost=0;
                  dp_point[j][k].last_index=0;
              }
          }
          for(int i=0;i<s_num;i++)//计算cost
          {
              
              for(int j=0;j<l_num;j++)
              {
                  if(i==0)
                  {
                    this->CalculateQuinticPolynomial(para,0,origin.l,origin.ld,origin.ldd,dp_point[i][j].s-origin.s,dp_point[i][j].l);//计算五次多项值系数,刷新para
                    dp_point[i][j].min_cost=this->CalculateCost(para,origin.s,valid_obs,valid_num);//计算起点到第一列当前点的cost
                  }
                  else
                  {
                    this->CalculateMinCost(&dp_point[0][0],valid_obs,valid_num,i,j,para);//计算到该点最小的cost，更新该点cost以及上一点的列索引
                  }
              }
          }
          best_index[s_num-1]=0;//初始化终点编号
          for(int i=0;i<l_num;i++)//找出最好的规划终点
          { 
            if(dp_point[s_num-1][i].min_cost<dp_point[s_num-1][best_index[s_num-1]].min_cost)
                    best_index[s_num-1]=i;
          }
          for(int i=0;i<s_num-1;i++)//算出最优轨迹各点编号
          {
              best_index[s_num-2-i]=dp_point[s_num-1-i][best_index[s_num-1-i]].last_index;

          }
          
          this->AddPoint(best_index,para,&dp_point[0][0],origin);//轨迹增密

          ret=this->SecondaryPlanning(valid_obs,valid_obsf,valid_num);//二次规划
          

          if(plan_s_local<plan_s)
          {
          for(int i=60*plan_s_local/plan_s+1;i<61;i++)
          {
            dynamic_point[i].l=dynamic_point[int(60*plan_s_local/plan_s)].l;
            dynamic_point[i].ld=dynamic_point[int(60*plan_s_local/plan_s)].ld;
            dynamic_point[i].ldd=dynamic_point[int(60*plan_s_local/plan_s)].ldd;
            dynamic_point[i].s=dynamic_point[int(60*plan_s_local/plan_s)].s;
            dynamic_point[i].sd=dynamic_point[int(60*plan_s_local/plan_s)].sd;
            dynamic_point[i].sdd=dynamic_point[int(60*plan_s_local/plan_s)].sdd;
          }
          // std::cout<<int(60*plan_s_local/plan_s)<<std::endl;
          // std::cout<<plan_s_local<<std::endl;
          }        
          // for(int i=0;i<61;i++)
          // {std::cout << "  "<<dynamic_point[i].l;
          // }
          // std::cout<< std::endl;

          if(!ret)
          {
            this->FreToCarte();
          } 
      }
      delete[] valid_obs;
      delete[] valid_obsf;
    }
}

void DynaPlaning::CalculateQuinticPolynomial(double* para,float start_s,float start_l,float start_dl,float start_ddl,float end_s,float end_l)
{
    scale = int(1/end_s);
    
    end_s = scale * end_s;
    float s=start_s;
    float s2=start_s*start_s;
    float s3=s2*start_s;
    float s4=s3*start_s;
    float s5=s4*start_s;
    float es=end_s;
    float es2=es*end_s;
    float es3=es2*end_s;
    float es4=es3*end_s;
    float es5=es4*end_s;
    Eigen::MatrixXd A(6,6);
    Eigen::VectorXd B(6);
    Eigen::VectorXd C(6);
    end_l = end_l * scale;
    start_l = start_l *scale;
    start_ddl = start_ddl /scale;
    B<<start_l,start_dl,start_ddl,end_l,0,0;
    A<<1    ,s    ,s2    ,s3    ,s4    ,s5    ,
       0    ,1    ,2*s   ,3*s2  ,4*s3  ,5*s4  ,
       0    ,0    ,2     ,6*s   ,12*s2 ,20*s3 ,
       1    ,es   ,es2   ,es3   ,es4   ,es5   ,
       0    ,1    ,2*es  ,3*es2 ,4*es3 ,5*es4 ,
       0    ,0    ,2     ,6*es  ,12*es2,20*es3;
    C=A.inverse()*B;
    for(int i=0;i<6;i++)
    {
        para[i]=C(i);
    }
}


float DynaPlaning::CalculateCost(double* para,float start_s,Frenet* valid_obs,int valid_num)
{
    Frenet samp_point[10];
    float s;
    
    float collision_cost=0;
    float ref_cost=0;
    float smooth_cost=0;
    float cost=0;
    for(int i=0;i<10;i++)//采样10个点用于计算cost
    {
        s=i*sample_s/10;
        samp_point[i].s=s;
        s = s * scale;
        samp_point[i].l=(para[0]+para[1]*s+para[2]*s*s+para[3]*s*s*s+para[4]*s*s*s*s+para[5]*s*s*s*s*s) / scale;
        samp_point[i].ld=(para[1]+2*para[2]*s+3*para[3]*s*s+4*para[4]*s*s*s+5*para[5]*s*s*s*s);
        samp_point[i].ldd=(2*para[2]+2*3*para[3]*s+3*4*para[4]*s*s+4*5*para[5]*s*s*s) * scale;
        samp_point[i].sd=(6*para[3]+24*para[4]*s+60*para[5]*s*s)*scale * scale;;//此处使用sd存储dddl
    }
    for(int i=0;i<10;i++)
    {
        ref_cost+=this->dp_cost_ref*samp_point[i].l*samp_point[i].l;
        smooth_cost+=this->dp_cost_smooth[0]*samp_point[i].ld*samp_point[i].ld
                   +this->dp_cost_smooth[1]*samp_point[i].ldd*samp_point[i].ldd
                   +this->dp_cost_smooth[2]*samp_point[i].sd*samp_point[i].sd;
        collision_cost+=this->CalculateCollisionCost(samp_point[i].s+start_s,samp_point[i].l,valid_obs,valid_num);
    }
    cost=ref_cost+smooth_cost+collision_cost;
    return cost;
}

float DynaPlaning::CalculateCollisionCost(float s,float l,Frenet* valid_obs,int valid_num)
{
    float distance,ds,dl;
    float cost=0;
    for(int i=0;i<valid_num;i++)
    {
        ds=(valid_obs+i)->s-s;
        dl=(valid_obs+i)->l-l;
        distance=ds*ds+dl*dl;
        if(distance<pow(dp_car.car.car_width*3,2) &&distance>pow(dp_car.car.car_width*1.5,2))
              cost+=this->dp_cost_collision*100/(distance*scale*scale);
        else if(distance< pow(dp_car.car.car_width*1.5,2))
              cost+=this->dp_cost_collision*100;
        
        //cost = 0 ;
    }
    return cost;
}

void DynaPlaning::CalculateMinCost(Sample* dp_point,Frenet* valid_obs,int valid_num,int index_s,int index_l,double* para)
{
   float min_cost;
   int s_num=int(plan_s_local/sample_s);
   int l_num=int((road_lu-road_ld)/sample_l-1);
   for(int i=0;i<l_num;i++) 
   {
      this->CalculateQuinticPolynomial(para,0,(dp_point+(index_s-1)*l_num+i)->l,0,0,sample_s,(dp_point+index_s*l_num+index_l)->l);
      min_cost=this->CalculateCost(para,(dp_point+(index_s-1)*l_num+i)->s,valid_obs,valid_num)
              +(dp_point+(index_s-1)*l_num+i)->min_cost;
      if(i==0)
      {
        (dp_point+index_s*l_num+index_l)->min_cost=min_cost;
        (dp_point+index_s*l_num+index_l)->last_index=i;
      }
      else
      {
        if((dp_point+index_s*l_num+index_l)->min_cost>min_cost)
        {
           (dp_point+index_s*l_num+index_l)->min_cost=min_cost;
           (dp_point+index_s*l_num+index_l)->last_index=i;
        }
      }
   }
}

void DynaPlaning::AddPoint(int* best_index,double* para,Sample* dp_point,Frenet origin)
{
   float s;
   int s_num=int(plan_s_local/sample_s);
   int l_num=int((road_lu-road_ld)/sample_l-1);

   this->dynamic_point[0]=origin;
   for(int i=0;i<61;i++)
   {
     this->dynamic_point[i].sd=road_lu;//先用于存储边界lmax，初始化为6
     this->dynamic_point[i].sdd=road_ld;//先用于存储边界lmin，初始化为-6
   }
   for(int i=0;i<s_num;i++)
   { 
        if(i==0)
        {
          this->CalculateQuinticPolynomial(para,0,origin.l,origin.ld,origin.ldd,sample_s,(dp_point+i*l_num+best_index[i])->l);
        }
        else
        {
          this->CalculateQuinticPolynomial(para,0,(dp_point+(i-1)*l_num+best_index[i-1])->l,0,0,sample_s,(dp_point+i*l_num+best_index[i])->l);
        }
        for(int j=1;j<sample_s/(plan_s/60)+1;j++)
        {
            s=j*plan_s/60;
            this->dynamic_point[i*int(sample_s/(plan_s/60))+j].s=origin.s+i*sample_s+s;
            s = s * scale;                                                      
            this->dynamic_point[i*int(sample_s/(plan_s/60))+j].l=(para[0]+para[1]*s+para[2]*s*s+para[3]*s*s*s+para[4]*s*s*s*s+para[5]*s*s*s*s*s)/scale;;
            this->dynamic_point[i*int(sample_s/(plan_s/60))+j].ld=para[1]+2*para[2]*s+3*para[3]*s*s+4*para[4]*s*s*s+5*para[5]*s*s*s*s;
            this->dynamic_point[i*int(sample_s/(plan_s/60))+j].ldd=(2*para[2]+2*3*para[3]*s+3*4*para[4]*s*s+4*5*para[5]*s*s*s)  * scale;
        }
    }
}

int DynaPlaning::SecondaryPlanning(Frenet* valid_obs,Obs* valid_obsf,int valid_num)
{
   int n=60*plan_s_local/plan_s+1;
   float lb[4*n];
   float ub[4*n];
   float end_l_desire=0;
   float end_dl_desire=0;
   float end_ddl_desire=0;
   double ds=plan_s/60;
   if(valid_num!=0)
     this->GetConvexSpace(valid_obs,valid_obsf,valid_num);
   this->GetValue(ub,lb);
   Eigen::SparseMatrix<double> hessian(3*n, 3*n);      //P: n*n正定矩阵,必须为稀疏矩阵SparseMatrix
   Eigen::VectorXd gradient(3*n);                    //Q: n*1向量
   Eigen::SparseMatrix<double> linearMatrix(9*n-2,3*n); //A: m*n矩阵,必须为稀疏矩阵SparseMatrix
   Eigen::VectorXd lowerBound(9*n-2);                  //L: m*1下限向量
   Eigen::VectorXd upperBound(9*n-2);                  //U: m*1上限向量


   //P矩阵构造
   std::vector<Triplet<double>> triplets;
   int r[5*n-2];
   int c[5*n-2];
   double val[5*n-2];
   this->gethessian(r,c,val);
   for(int i=0;i<5*n-2;i++)
       triplets.emplace_back(r[i],c[i],val[i]);
   hessian.setFromTriplets(triplets.begin(),triplets.end());


   //构造Q矩阵
   for(int i=0;i<n;i++)
   {
      if(i<n-1)
      {
       gradient(3*i)=-this->qp_cost_mid*(dynamic_point[i].sd+dynamic_point[i].sdd);
       gradient(3*i+1)=0;
       gradient(3*i+2)=0;
      }
      else
      {
       gradient(3*i)=-2*(this->qp_cost_ref*end_l_desire);
       gradient(3*i+1)=-2*(this->qp_cost_smooth[0]*end_dl_desire);
       gradient(3*i+2)=-2*(this->qp_cost_smooth[1]*end_ddl_desire);
      }
   }
   
   //构造A矩阵
   for(int i=0;i<n;i++)
   {
     if(i<n-1)
     {
        linearMatrix.insert(2*i, 3*i) = 1.0;
        linearMatrix.insert(2*i, 3*i+1) = ds;
        linearMatrix.insert(2*i, 3*i+2) = 1.0/3.0*ds*ds;
        linearMatrix.insert(2*i, 3*i+3) = -1.0;
        linearMatrix.insert(2*i, 3*i+5) = 1.0/6.0*ds*ds*ds;
        linearMatrix.insert(2*i+1, 3*i+1) = 1.0;
        linearMatrix.insert(2*i+1, 3*i+2) = 0.5*ds;
        linearMatrix.insert(2*i+1, 3*i+4) = -1.0;
        linearMatrix.insert(2*i+1, 3*i+5) = 0.5*ds;
     }

        linearMatrix.insert(2*n-2+4*i, 3*i) = 1.0;
        linearMatrix.insert(2*n-2+4*i+1, 3*i) = 1.0;
        linearMatrix.insert(2*n-2+4*i+2, 3*i) = 1.0;
        linearMatrix.insert(2*n-2+4*i+3, 3*i) = 1.0;
        linearMatrix.insert(2*n-2+4*i, 3*i+1) = 0.5*(this->dp_car.car.car_length);
        linearMatrix.insert(2*n-2+4*i+1, 3*i+1) = 0.5*(this->dp_car.car.car_length);
        linearMatrix.insert(2*n-2+4*i+2, 3*i+1) = -0.5*(this->dp_car.car.car_length);
        linearMatrix.insert(2*n-2+4*i+3, 3*i+1) = -0.5*(this->dp_car.car.car_length);

        linearMatrix.insert(6*n-2+3*i, 3*i) = 1.0;
        linearMatrix.insert(6*n-2+3*i+1, 3*i+1) = 1.0;
        linearMatrix.insert(6*n-2+3*i+2, 3*i+2) = 1.0;
   }
   //构造L，U
   for(int i=0;i<2*n-2;i++)
   {
    lowerBound(i)=0;
    upperBound(i)=0;
   }
   for(int i=0;i<4*n;i++)
   {
    lowerBound(i+2*n-2)=lb[i];
    upperBound(i+2*n-2)=ub[i];
   }
   for(int i=0;i<3*n;i++)
   {
    if(i<3)
    {
      lowerBound(0+6*n-2)=dynamic_point[0].l-0.05;
      upperBound(0+6*n-2)=dynamic_point[0].l+0.05;
      lowerBound(1+6*n-2)=dynamic_point[0].ld;
      upperBound(1+6*n-2)=dynamic_point[0].ld;
      lowerBound(2+6*n-2)=dynamic_point[0].ldd;
      upperBound(2+6*n-2)=dynamic_point[0].ldd;
    }
    else
    {
      lowerBound(i+6*n-2)=-OsqpEigen::INFTY;//相当于无限制，在前面已进行限制
      upperBound(i+6*n-2)=OsqpEigen::INFTY;//相当于无限制，在前面已进行限制
    }
   }
  
   // instantiate the solver
    OsqpEigen::Solver solver;

    // settings
    solver.settings()->setVerbosity(false);
    solver.settings()->setWarmStart(true);
    // set the initial data of the QP solver
    solver.data()->setNumberOfVariables(3*n);   //变量数n
    solver.data()->setNumberOfConstraints(9*n-2); //约束数m
    if (!solver.data()->setHessianMatrix(hessian)) // hessian必须为sparse，否则类型不匹配
        return 1;
    if (!solver.data()->setGradient(gradient))
        return 1;
    if (!solver.data()->setLinearConstraintsMatrix(linearMatrix))
        return 1;
    if (!solver.data()->setLowerBound(lowerBound))
        return 1;
    if (!solver.data()->setUpperBound(upperBound))
        return 1;

    // instantiate the solver
    if (!solver.initSolver())
        return 1;

    Eigen::VectorXd QPSolution;

    // solve the QP problem
    if(solver.solveProblem() != OsqpEigen::ErrorExitFlag::NoError) 
        return 1;

    QPSolution = solver.getSolution();
    //std::cout<<QPSolution<<std::endl;
    if(QPSolution(3)>1000)
    {
      std::cout<<"fail OSQP"<<std::endl;
      for(int i=0;i<n;i++)
      { 
        dynamic_point[i].s=dynamic_point[0].s;
        dynamic_point[i].l=dynamic_point[0].l;
        dynamic_point[i].ld=dynamic_point[0].ld;
        dynamic_point[i].ldd=dynamic_point[0].ldd;
      }
    }
    else
    {
      for(int i=0;i<n;i++)
      {
        dynamic_point[i].l=QPSolution(3*i);
        dynamic_point[i].ld=QPSolution(3*i+1);
        dynamic_point[i].ldd=QPSolution(3*i+2);
      }
    }
    
    return 0;
}


void DynaPlaning::GetConvexSpace(Frenet* valid_obs,Obs* valid_obsf,int valid_num)
{

   int start_index,end_index,center_index;
   for(int i=0;i<valid_num;i++)
   {
      start_index=floor(60/plan_s*((valid_obs+i)->s-0.5*(valid_obsf+i)->obs_length-dynamic_point[0].s));
      end_index=ceil(60/plan_s*((valid_obs+i)->s+0.5*(valid_obsf+i)->obs_length-dynamic_point[0].s));
      center_index=floor(60/plan_s*((valid_obs+i)->s-dynamic_point[0].s));
      if(start_index<0)
          start_index=0;
      if(end_index>60*plan_s_local/plan_s)
           end_index=60*plan_s_local/plan_s+1;;
      if(dynamic_point[center_index].l<(valid_obs+i)->l)//向右绕
      {
        for(int j=start_index;j<end_index+1;j++)
        {
          if(dynamic_point[j].sd>((valid_obs+i)->l-0.5*(valid_obsf+i)->obs_width))
              dynamic_point[j].sd=(valid_obs+i)->l-0.5*(valid_obsf+i)->obs_width;
        }
      }
      else//向左绕
      {
        for(int j=start_index;j<end_index+1;j++)
        {
           if(dynamic_point[j].sdd<((valid_obs+i)->l+0.5*(valid_obsf+i)->obs_width))
                  dynamic_point[j].sdd=(valid_obs+i)->l+0.5*(valid_obsf+i)->obs_width;
        }
      }
   }
}

void DynaPlaning::GetValue(float* ub,float* lb)
{
  int n=60*plan_s_local/plan_s+1;
  int head_index,tail_index;
  for(int i=0;i<n;i++)
  {
    head_index=ceil(60/plan_s*(dynamic_point[i].s+0.5*dp_car.car.car_length-dynamic_point[0].s));
    if(head_index>n-1)
    {
        head_index=n-1;
    }
    tail_index=ceil(60/plan_s*(dynamic_point[i].s-0.5*dp_car.car.car_length-dynamic_point[0].s));
    if(tail_index<0)
    {
        tail_index=0;
    }
    *(ub+4*i)=dynamic_point[head_index].sd-0.5*dp_car.car.car_width;
    *(lb+4*i)=dynamic_point[head_index].sdd-0.5*dp_car.car.car_width;
    *(ub+4*i+1)=dynamic_point[head_index].sd+0.5*dp_car.car.car_width;
    *(lb+4*i+1)=dynamic_point[head_index].sdd+0.5*dp_car.car.car_width;
    *(ub+4*i+2)=dynamic_point[tail_index].sd-0.5*dp_car.car.car_width;
    *(lb+4*i+2)=dynamic_point[tail_index].sdd-0.5*dp_car.car.car_width;
    *(ub+4*i+3)=dynamic_point[tail_index].sd+0.5*dp_car.car.car_width;
    *(lb+4*i+3)=dynamic_point[tail_index].sdd+0.5*dp_car.car.car_width;
  }
}

void DynaPlaning::FreToCarte()
{
    this->montage_index=0;
    Reference proj;
    float xr,yr,l,cr,ld,kr,ldd,kdr,dc;
    for(int i=0;i<60*plan_s/plan_s+1;i++)
    {
      proj=this->GetFTCproj(dynamic_point[i].s);
      xr=proj.x_r;
      yr=proj.y_r;
      cr=proj.heading_r;
      kr=proj.k_r;
      kdr=proj.k_rd;
      l=dynamic_point[i].l;
      ld=dynamic_point[i].ld;
      ldd=dynamic_point[i].ldd;
      (this->traj+this->traj_num)->x=xr-l*sin(cr);
      (this->traj+this->traj_num)->y=yr+l*cos(cr);

      float dheading = atan2(ld,(1-kr*l))+cr;
      

      if(abs(dheading-cr)>abs(dheading-cr + M_2_PI)) dheading+=M_2_PI;
      else if(abs(dheading-cr)>abs(dheading-cr - M_2_PI)) dheading-=M_2_PI;
      (this->traj+this->traj_num)->heading=dheading;

      //(this->traj+this->traj_num)->heading=atan2(ld,(1-kr*l))+cr;
      
      if((this->traj+this->traj_num)->heading>M_PI)
      {
        (this->traj+this->traj_num)->heading-=2*M_PI;
        //std::cout<< "-M" <<std::endl;
      }
      if((this->traj+this->traj_num)->heading<-M_PI)
      {
        (this->traj+this->traj_num)->heading+=2*M_PI;
        //std::cout<< "+M" <<std::endl;
      }

      //std::cout << (this->traj+this->traj_num)->heading <<std::endl;
      dc=(this->traj+this->traj_num)->heading-cr;
      (this->traj+this->traj_num)->k=((ldd+(kdr*l+kr*ld)*tan(dc))*cos(dc)*cos(dc)/(1-kr*l)+kr)*(cos(dc)/(1-kr*l));
      this->traj_num+=1;
    }
}

Reference DynaPlaning::GetFTCproj(float s)
{
    int index=this->montage_index;//从上一次投影的序号开始往后搜索
    float s_cur,s_next,ds;
    Reference proj;
    for(int i=this->montage_index;i<this->dp_car.Point_num;i++)
    {
       s_cur=(this->dp_car.referenceline+i)->s_r;
       s_next=(this->dp_car.referenceline+i+1)->s_r;
       if(s>=s_cur&&s<s_next)
       {
         this->montage_index=i;
         ds=s-s_cur;
         proj.heading_r=(this->dp_car.referenceline+i)->heading_r;
         proj.x_r=(this->dp_car.referenceline+i)->x_r+ds*cos(proj.heading_r);
         proj.y_r=(this->dp_car.referenceline+i)->y_r+ds*sin(proj.heading_r);
         proj.s_r=s;
         proj.k_r=0.5*((this->dp_car.referenceline+i)->k_r+(this->dp_car.referenceline+i+1)->k_r);
         proj.k_rd=((this->dp_car.referenceline+i+1)->k_r-(this->dp_car.referenceline+i)->k_r)
                  /((this->dp_car.referenceline+i+1)->s_r-(this->dp_car.referenceline+i)->s_r);
       }
    }
    return proj;
}

void DynaPlaning::gethessian(int* r,int* c,double* val)
{
  int n=60*plan_s_local/plan_s+1;
   r[0]=0;
   r[1]=1;
   r[2]=2;
   r[3]=5;
   r[5*n-6]=3*n-3;
   r[5*n-5]=3*n-2;
   r[5*n-4]=3*n-4;
   r[5*n-3]=3*n-1;
   for(int i=0;i<n-2;i++)
   {
       r[4+i*5]=3*(i+1);
       r[4+i*5+1]=3*(i+1)+1;
       r[4+i*5+2]=3*(i+1)-1;
       r[4+i*5+3]=3*(i+1)+2;
       r[4+i*5+4]=3*(i+1)+5;
   }
   c[0]=0;
   c[1]=1;
   c[2]=2;
   c[3]=2;
   c[5*n-6]=3*n-3;
   c[5*n-5]=3*n-2;
   c[5*n-4]=3*n-1;
   c[5*n-3]=3*n-1;
   for(int i=0;i<n-2;i++)
   {
    
       c[4+i*5]=3*(i+1);
       c[4+i*5+1]=3*(i+1)+1;
       c[4+i*5+2]=3*(i+1)+2;
       c[4+i*5+3]=3*(i+1)+2;
       c[4+i*5+4]=3*(i+1)+2;
    
   }
   val[0]=2*(this->qp_cost_ref+this->qp_cost_mid);
   val[1]=2*(this->qp_cost_smooth[0]);
   val[2]=2*(this->qp_cost_smooth[1]+this->qp_cost_smooth[2]);
   val[3]=-2*(this->qp_cost_smooth[2]);
   val[5*n-6]=2*(this->qp_cost_ref+this->qp_cost_mid);
   val[5*n-5]=2*(this->qp_cost_smooth[0]);
   val[5*n-4]=-2*(this->qp_cost_smooth[2]);
   val[5*n-3]=2*(this->qp_cost_smooth[2]+this->qp_cost_smooth[1]);
   for(int i=0;i<n-2;i++)
   {
     
       val[4+i*5]=2*(this->qp_cost_ref+this->qp_cost_mid);
       val[4+i*5+1]=2*(this->qp_cost_smooth[0]);
       val[4+i*5+2]=-2*(this->qp_cost_smooth[2]);
       val[4+i*5+3]=2*(this->qp_cost_smooth[1]+2*(this->qp_cost_smooth[2]));
       val[4+i*5+4]=-2*(this->qp_cost_smooth[2]);
     
   }
}

DynaPlaning::~DynaPlaning()
{
    this->traj_num=0;
    this->montage_index=0;
    // if(this->traj!=nullptr)
    //    delete[] this->traj;
}
