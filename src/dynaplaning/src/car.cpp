#include "car.h"
using namespace std;

HostCar::HostCar(Car car_p,Reference* p,int len):Coordinate(p,len)
{
 this->car=car_p;
}

void HostCar::CarteToFre()
{
    Reference proj;
    float dx,dy,dh,s,sd,sdd,l,ld,ldd;
        proj=this->getproj(car.car_center_x,car.car_center_y);
        dx=car.car_center_x-proj.x_r;
        dy=car.car_center_y-proj.y_r;
        dh=car.car_hesding-proj.heading_r;
        s=proj.s_r;
        l=this->sign(dy*cos(proj.heading_r)-dx*sin(proj.heading_r))*sqrt(dx*dx+dy*dy);
    
        ld=(1-proj.k_r*l)*tan(dh);
        ldd=-(proj.k_rd*l+proj.k_r*ld)*tan(dh)+(1-proj.k_r*l)*(((1-proj.k_r*l)*(car.car_k)/cos(dh))-proj.k_r)/(cos(dh)*cos(dh));
        sd=((car.car_velocity)*cos(dh))/(1-proj.k_r*l);
        sdd=((car.car_acc)*cos(dh)-sd*sd*(ld*(((1-proj.k_r*l)*(car.car_k)/cos(dh))-proj.k_r)-(proj.k_rd*l+proj.k_r*ld)))/(1-proj.k_r*l);
        car_frenet.s=s;
        car_frenet.l=l;
        car_frenet.ld=ld;
        car_frenet.ldd=ldd;
        car_frenet.sd=sd;
        car_frenet.sdd=sdd;
}