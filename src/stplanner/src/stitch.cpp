#include "main.h"
void stitch(VectorXd trajectory_x,VectorXd trajectory_y,VectorXd trajectory_heading,VectorXd trajectory_kappa,VectorXd trajectory_speed, 
VectorXd trajectory_accel,VectorXd trajectory_time,VectorXd trajectory_x_pre,VectorXd trajectory_y_pre,VectorXd trajectory_heading_pre,
VectorXd trajectory_kappa_pre,VectorXd trajectory_speed_pre, VectorXd trajectory_accel_pre,VectorXd trajectory_time_pre,double current_time,
VectorXd& trajectory_x_final,VectorXd& trajectory_y_final,VectorXd& trajectory_heading_final,VectorXd& trajectory_kappa_final,
VectorXd& trajectory_speed_final, VectorXd& trajectory_accel_final,VectorXd& trajectory_time_final)
{
if(trajectory_x_pre(0)==NULL)//首次运行
{
trajectory_x_final=trajectory_x;
trajectory_y_final=trajectory_y;
trajectory_heading_final=trajectory_heading;
trajectory_kappa_final=trajectory_kappa;
trajectory_speed_final=trajectory_speed;
trajectory_accel_final=trajectory_accel;
trajectory_time_final=trajectory_time;
}
else
{
for(int i=0;i<trajectory_time_pre.size()-1;i++)
{
    if(current_time>=trajectory_time_pre(i)&&current_time<trajectory_time_pre(i+1))
        {
            if(i>=9)//满足十个点
            {   
                trajectory_x_final.resize(10+trajectory_x.size());
                trajectory_y_final.resize(10+trajectory_x.size());
                trajectory_heading_final.resize(10+trajectory_x.size());
                trajectory_kappa_final.resize(10+trajectory_x.size());
                trajectory_speed_final.resize(10+trajectory_x.size());
                trajectory_accel_final.resize(10+trajectory_x.size());
                trajectory_time_final.resize(10+trajectory_x.size());
                for(int j=0;j<10+trajectory_x.size();j++)
                {
                    if(j<10)
                    {
                        trajectory_x_final(j)=trajectory_x_pre(j);
                        trajectory_y_final(j)=trajectory_y_pre(j);
                        trajectory_heading_final(j)=trajectory_heading_pre(j);
                        trajectory_kappa_final(j)=trajectory_kappa_pre(j);
                        trajectory_speed_final(j)=trajectory_speed_pre(j);
                        trajectory_accel_final(j)=trajectory_accel_pre(j);
                        trajectory_time_final(j)=trajectory_time_pre(j);
                    }
                    else
                    {
                        trajectory_x_final(j)=trajectory_x(j-10);
                        trajectory_y_final(j)=trajectory_y(j-10);
                        trajectory_heading_final(j)=trajectory_heading(j-10);
                        trajectory_kappa_final(j)=trajectory_kappa(j-10);
                        trajectory_speed_final(j)=trajectory_speed(j-10);
                        trajectory_accel_final(j)=trajectory_accel(j-10);
                        trajectory_time_final(j)=trajectory_time(j-10);
                    }
                }
            }

            else//不满足十个点
            {
                trajectory_x_final.resize(i+trajectory_x.size());
                trajectory_y_final.resize(i+trajectory_x.size());
                trajectory_heading_final.resize(i+trajectory_x.size());
                trajectory_kappa_final.resize(i+trajectory_x.size());
                trajectory_speed_final.resize(i+trajectory_x.size());
                trajectory_accel_final.resize(i+trajectory_x.size());
                trajectory_time_final.resize(i+trajectory_x.size());
                for(int j=0;j<i+trajectory_x.size();j++)
                {
                    if(j<10)
                    {
                        trajectory_x_final(j)=trajectory_x_pre(j);
                        trajectory_y_final(j)=trajectory_y_pre(j);
                        trajectory_heading_final(j)=trajectory_heading_pre(j);
                        trajectory_kappa_final(j)=trajectory_kappa_pre(j);
                        trajectory_speed_final(j)=trajectory_speed_pre(j);
                        trajectory_accel_final(j)=trajectory_accel_pre(j);
                        trajectory_time_final(j)=trajectory_time_pre(j);
                    }
                    else
                    {
                        trajectory_x_final(j)=trajectory_x(j-i);
                        trajectory_y_final(j)=trajectory_y(j-i);
                        trajectory_heading_final(j)=trajectory_heading(j-i);
                        trajectory_kappa_final(j)=trajectory_kappa(j-i);
                        trajectory_speed_final(j)=trajectory_speed(j-i);
                        trajectory_accel_final(j)=trajectory_accel(j-i);
                        trajectory_time_final(j)=trajectory_time(j-i);
                    }
                }



            }



        }



}





}
}