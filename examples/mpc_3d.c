/* 三维双积分器投影梯度 MPC：控制序列逐时刻按向量模投影。 */
#include <math.h>
#include <stdio.h>
#define N 20
typedef struct{double x,y,z;} V3;
static V3 add(V3 a,V3 b){return(V3){a.x+b.x,a.y+b.y,a.z+b.z};}
static V3 mul(V3 a,double k){return(V3){a.x*k,a.y*k,a.z*k};}
static double norm(V3 a){return hypot(hypot(a.x,a.y),a.z);}
static V3 project(V3 a,double m){double n=norm(a);return n>m?mul(a,m/n):a;}
int main(void){const double dt=.1,amax=3,q=2,r=.08;V3 p={-2,1,.7},v={0,0,0},u[N]={0};
 for(int step=0;step<100;step++){for(int it=0;it<60;it++){V3 pp[N+1],vv[N+1];pp[0]=p;vv[0]=v;for(int j=0;j<N;j++){pp[j+1]=add(pp[j],add(mul(vv[j],dt),mul(u[j],.5*dt*dt)));vv[j+1]=add(vv[j],mul(u[j],dt));}V3 lp=mul(pp[N],8*q),lv=mul(vv[N],2);for(int j=N-1;j>=0;j--){V3 g=add(mul(u[j],2*r),add(mul(lp,.5*dt*dt),mul(lv,dt)));u[j]=project(add(u[j],mul(g,-.02)),amax);lv=add(lv,mul(lp,dt));lp=add(lp,mul(pp[j],2*q));}}
  V3 a=u[0];for(int j=0;j<N-1;j++)u[j]=u[j+1];u[N-1]=(V3){0};p=add(p,add(mul(v,dt),mul(a,.5*dt*dt)));v=add(v,mul(a,dt));}
 printf("terminal p=[%.4f %.4f %.4f], v=[%.4f %.4f %.4f], |u0|=%.4f\n",p.x,p.y,p.z,v.x,v.y,v.z,norm(u[0]));return 0;}
