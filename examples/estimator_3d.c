/* 六状态三维位置—速度 Kalman 教学实现；每轴共享模型但独立保留2x2协方差。 */
#include <math.h>
#include <stdio.h>
typedef struct{double p,v,P00,P01,P11;} AxisKF;
static void update(AxisKF*k,double z,double dt,double q,double r){
 k->p+=dt*k->v;double a=k->P00+2*dt*k->P01+dt*dt*k->P11+q*dt*dt*dt/3;
 double b=k->P01+dt*k->P11+q*dt*dt/2,c=k->P11+q*dt,S=a+r,K0=a/S,K1=b/S,e=z-k->p;
 k->p+=K0*e;k->v+=K1*e;k->P00=(1-K0)*a;k->P01=(1-K0)*b;k->P11=c-K1*b;
}
int main(void){AxisKF k[3]={{0,0,100,0,25},{0,0,100,0,25},{0,0,100,0,25}};double dt=.05,q=2,r=9;
 for(int n=0;n<200;n++){double t=n*dt,z[3]={20*t+sin(t),-4*t+.5*sin(2*t),1000+2*t+.3*cos(t)};for(int i=0;i<3;i++)update(&k[i],z[i],dt,q,r);}
 printf("r_hat=[%.3f %.3f %.3f] m, v_hat=[%.3f %.3f %.3f] m/s, tr(P_pos)=%.3f\n",k[0].p,k[1].p,k[2].p,k[0].v,k[1].v,k[2].v,k[0].P00+k[1].P00+k[2].P00);return 0;}
