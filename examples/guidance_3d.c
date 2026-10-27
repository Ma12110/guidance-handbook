/* cc -std=c99 -Wall -Wextra -Wpedantic examples/guidance_3d.c -lm -o guidance_3d */
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
typedef struct { double x,y,z; } Vec3;
typedef struct { Vec3 r,vr,a_target,terminal_dir; double tgo; } Input;
typedef struct { Vec3 pn,apn,zem,iac,tzc; bool valid; } Output;
static Vec3 add(Vec3 a,Vec3 b){return(Vec3){a.x+b.x,a.y+b.y,a.z+b.z};}
static Vec3 sub(Vec3 a,Vec3 b){return(Vec3){a.x-b.x,a.y-b.y,a.z-b.z};}
static Vec3 mul(Vec3 a,double k){return(Vec3){k*a.x,k*a.y,k*a.z};}
static double dot(Vec3 a,Vec3 b){return a.x*b.x+a.y*b.y+a.z*b.z;}
static Vec3 cross(Vec3 a,Vec3 b){return(Vec3){a.y*b.z-a.z*b.y,a.z*b.x-a.x*b.z,a.x*b.y-a.y*b.x};}
static double norm(Vec3 a){return hypot(hypot(a.x,a.y),a.z);}
static Vec3 unit(Vec3 a){double n=norm(a);return n>1e-9?mul(a,1/n):(Vec3){0,0,0};}
static Vec3 normal(Vec3 a,Vec3 axis){Vec3 e=unit(axis);return sub(a,mul(e,dot(a,e)));}
static Vec3 limit(Vec3 a,double m){double n=norm(a);return n>m?mul(a,m/n):a;}
static Output guidance_3d(Input in,double N,double Ka,double amax){
 Output o={0}; double R=norm(in.r),T=in.tgo;
 if(R<1e-6||T<.1||amax<=0)return o;
 Vec3 los=mul(in.r,1/R); double Vc=-dot(in.r,in.vr)/R;
 if(Vc<=0)return o;
 Vec3 omega=mul(cross(in.r,in.vr),1/(R*R));
 o.pn=limit(mul(cross(omega,los),N*Vc),amax);
 o.apn=limit(add(o.pn,mul(normal(in.a_target,los),Ka)),amax);
 Vec3 predicted=add(add(in.r,mul(in.vr,T)),mul(in.a_target,.5*T*T));
 o.zem=limit(mul(normal(predicted,los),-3/(T*T)),amax);
 Vec3 ep=normal(in.r,in.terminal_dir),ev=normal(in.vr,in.terminal_dir);
 o.iac=limit(add(mul(ep,-6/(T*T)),mul(ev,-4/T)),amax);
 o.tzc=limit(add(mul(ep,-12/(T*T)),mul(ev,-6/T)),amax);
 o.valid=true; return o;
}
int main(void){Input in={{6000,1200,200},{-400,15,-8},{0,3,2},{-1,0,-.1},15};
 Output o=guidance_3d(in,3,.5,30*9.80665);if(!o.valid)return 1;
 printf("PN=[%.3f %.3f %.3f], ZEM=[%.3f %.3f %.3f] m/s^2\n",o.pn.x,o.pn.y,o.pn.z,o.zem.x,o.zem.y,o.zem.z);return 0;}
