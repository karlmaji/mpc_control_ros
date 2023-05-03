#include "Sensor.h"
using namespace std;

Obstacle::Obstacle(Obs* obs_p,int num,Reference* p,int len):Coordinate(p,len)
{
 this->obs=obs_p;
 this->obs_num=num;
 this->obs_frenet=new Frenet [num];
}

Obstacle::Obstacle(const Obstacle &obst):Coordinate(obst.referenceline,obst.Point_num)
{
 this->obs=obst.obs;
 this->obs_num=obst.obs_num;
 this->obs_frenet=new Frenet [obst.obs_num];
 int i;
 for(i=0;i<this->obs_num;i++)
 {
    *((this->obs_frenet)+i)=*(obst.obs_frenet+i);
 }
}

void Obstacle::CarteToFre()
{
    Reference proj;
    float dx,dy,dh,s,sd,sdd,l,ld,ldd,l_dot;
    ld=0;
    sd=0;
    ldd=0;
    sdd=0;
    l_dot=0;
    for(int i=0;i<this->obs_num;i++)
    {
        proj=this->getproj(((this->obs)+i)->obs_center_x,((this->obs)+i)->obs_center_y);
        dx=((this->obs)+i)->obs_center_x-proj.x_r;
        dy=((this->obs)+i)->obs_center_y-proj.y_r;
        dh=((this->obs)+i)->obs_hesding-proj.heading_r;
        float tandh;
        if(dh<-M_PI_2l)
            tandh=tan(-M_PI-dh);//dh=dh+M_PI_2l;
        else if(dh>M_PI_2l)
            tandh=tan(M_PI-dh);
        else 
            tandh=tan(dh);
        s=proj.s_r;
        l=this->sign(dy*cos(proj.heading_r)-dx*sin(proj.heading_r))*sqrt(dx*dx+dy*dy);
        if(((this->obs)+i)->obs_velocity!=0)
        {
            ld=((this->obs)+i)->obs_velocity*(1-proj.k_r*l)*tandh;//ld=(1-proj.k_r*l)*tan(dh);
            ldd=-(proj.k_rd*l+proj.k_r*ld)*tandh+(1-proj.k_r*l)*(((1-proj.k_r*l)*(((this->obs)+i)->obs_k)/cos(dh))-proj.k_r)/(cos(dh)*cos(dh));//ldd=-(proj.k_rd*l+proj.k_r*ld)*tan(dh)+(1-proj.k_r*l)*(((1-proj.k_r*l)*(((this->obs)+i)->obs_k)/cos(dh))-proj.k_r)/(cos(dh)*cos(dh));
            sd=((this->obs)+i)->obs_velocity*cos(dh)/(1-proj.k_r*l);
            sdd=((((this->obs)+i)->obs_acc)*cos(dh)-sd*sd*(ld*(((1-proj.k_r*l)*(((this->obs)+i)->obs_k)/cos(dh))-proj.k_r)-(proj.k_rd*l+proj.k_r*ld)))/(1-proj.k_r*l);
            l_dot=((this->obs)+i)->obs_velocity*sin(dh)/(1-proj.k_r*l);
        }
        if(ld!=0&&s>1)//"tandh:"<<tandh<<endl<<
        cout<<endl<<"l_dot:"<<l_dot<<endl<<"sd:"<<sd<<endl<<"s:"<<s<<endl<<"proj.k_r:"<<proj.k_r<<endl<<"dh:"<<dh<<endl<<"l:"<<l<<endl<<"proj.heading_r:"<<proj.heading_r<<endl<<"obs_hesding:"<<((this->obs)+i)->obs_hesding<<endl;
       //cout<<"l:"<<l<<endl<<"s:"<<s<<endl;
       // if(ld==-1.75077)
        //cout<<"ld=-1.75077"<<endl<<"proj.k_r:"<<proj.k_r<<endl<<"dh:"<<dh<<endl;
        //if(ld==8.87957)
        //cout<<"ld=8.87957"<<endl<<"proj.k_r:"<<proj.k_r<<endl<<"dh:"<<dh<<endl;
        ((this->obs_frenet)+i)->s=s;
        ((this->obs_frenet)+i)->l=l;
        ((this->obs_frenet)+i)->ld=l_dot;
        ((this->obs_frenet)+i)->ldd=ldd;
        ((this->obs_frenet)+i)->sd=sd;
        ((this->obs_frenet)+i)->sdd=sdd;
    }
    /*
    for(int i=0;i<this->obs_num;i++)
    {
        cout<<(obs_frenet+i)->s-78.5<<"  "<<(obs_frenet+i)->l<<endl;
    }
    */
    
}
Obstacle::~Obstacle()
{
 this->obs=nullptr;
 this->obs_num=0;
 delete[] this->obs_frenet;
 this->obs_frenet=nullptr;
}