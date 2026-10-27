/* 三维中制导—交班—末制导闭环教学参考
 * cc -std=c99 -O2 -Wall -Wextra -Wpedantic examples/full_guidance_3d.c -lm -o full_guidance_3d
 */
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#define PI 3.14159265358979323846
typedef struct{double x,y,z;} V3;
typedef struct{V3 r,v,a;} Body;
typedef enum{MIDCOURSE,HANDOFF,TERMINAL} Mode;
typedef enum{PN,APN,ZEM,IAC,TZC} Law;
typedef struct{Mode mode;unsigned on,off;double start;} Handoff;
typedef struct{double range,off_angle,age,nis,cov,vc,required,available;bool track;} Conditions;
static V3 add(V3 a,V3 b){return(V3){a.x+b.x,a.y+b.y,a.z+b.z};}
static V3 sub(V3 a,V3 b){return(V3){a.x-b.x,a.y-b.y,a.z-b.z};}
static V3 mul(V3 a,double k){return(V3){a.x*k,a.y*k,a.z*k};}
static double dot(V3 a,V3 b){return a.x*b.x+a.y*b.y+a.z*b.z;}
static V3 cross(V3 a,V3 b){return(V3){a.y*b.z-a.z*b.y,a.z*b.x-a.x*b.z,a.x*b.y-a.y*b.x};}
static double norm(V3 a){return hypot(hypot(a.x,a.y),a.z);}
static V3 unit(V3 a){double n=norm(a);return n>1e-9?mul(a,1/n):(V3){0,0,0};}
static V3 normal(V3 a,V3 axis){V3 e=unit(axis);return sub(a,mul(e,dot(a,e)));}
static V3 limit(V3 a,double m){double n=norm(a);return n>m?mul(a,m/n):a;}
static double clip(double x,double a,double b){return x<a?a:(x>b?b:x);}
static V3 target_accel(double t,V3 v){double vh=hypot(v.x,v.y),q=0;if(vh<1)return(V3){0,0,0};if(t>7&&t<10)q=8;if(t>13&&t<16)q=-6;return(V3){-q*v.y/vh,q*v.x/vh,2*sin(.5*t)};}
static V3 midcourse(V3 r,V3 vr,double amax){double T=clip(norm(r)/fmax(-dot(unit(r),vr),10),1,20);return limit(mul(add(r,mul(vr,T)),-2/(T*T)),amax);}
static V3 terminal(Law law,V3 r,V3 vr,V3 at,V3 vf,double T,double amax){
 double R=norm(r),vc=-dot(r,vr)/R;V3 los=unit(r),omega=mul(cross(r,vr),1/(R*R));
 V3 pn=mul(cross(omega,los),3*vc),pred=add(add(r,mul(vr,T)),mul(at,.5*T*T));
 V3 ep=normal(r,vf),ev=normal(vr,vf),a=pn;
 if(law==APN)a=add(pn,mul(normal(at,los),.5));
 if(law==ZEM)a=mul(normal(pred,los),-3/(T*T));
 if(law==IAC)a=add(mul(ep,-6/(T*T)),mul(ev,-4/T));
 if(law==TZC)a=add(mul(ep,-12/(T*T)),mul(ev,-6/T));
 return limit(a,amax);
}
static Conditions conditions(V3 r,V3 vr,V3 seeker_axis,double age,double nis,double cov,double req,double avail,bool track){
 Conditions c={0};c.range=norm(r);c.off_angle=acos(clip(dot(unit(seeker_axis),unit(r)),-1,1));c.age=age;c.nis=nis;c.cov=cov;c.vc=-dot(unit(r),vr);c.required=req;c.available=avail;c.track=track;return c;
}
static void update_handoff(Handoff*h,Conditions c,double t){
 bool enter=c.range<3000&&c.off_angle<18*PI/180&&c.age<.060&&c.nis<9&&c.cov<400&&c.vc>30&&c.required<.8*c.available&&c.track;
 bool exit=c.off_angle>22*PI/180||c.age>.100||c.nis>16||c.required>c.available||!c.track;
 if(h->mode==MIDCOURSE){h->on=enter?h->on+1:0;if(h->on>=5){h->mode=HANDOFF;h->start=t;}}
 else if(h->mode==HANDOFF){if(exit){h->mode=MIDCOURSE;h->on=0;}else if(t-h->start>=.2)h->mode=TERMINAL;}
 else{h->off=exit?h->off+1:0;if(h->off>=3){h->mode=MIDCOURSE;h->off=0;}}
}
static void step(Body*b,V3 a,double dt){b->v=add(b->v,mul(a,dt));b->r=add(b->r,mul(b->v,dt));}
int main(int argc,char**argv){
 Law law=PN;if(argc>1){int q=atoi(argv[1]);if(q>=0&&q<=4)law=(Law)q;}
 const double dt=.01,amax=30*9.80665,tau=.05;Body m={{0,0,1000},{300,0,0},{0,0,0}},tg={{6000,600,1100},{-100,0,0},{0,0,0}};
 Handoff h={MIDCOURSE,0,0,0};double best=1e99,best_t=0,handoff_t=-1,effort=0,peak=0;unsigned sat=0,steps=0;V3 best_r={0},best_vm={0},vf={-1,0,-.08};
 for(int k=0;k<3000;k++){double t=k*dt;tg.a=target_accel(t,tg.v);V3 r=sub(tg.r,m.r),vr=sub(tg.v,m.v);double R=norm(r),vc=-dot(unit(r),vr);if(vc<=0&&t>1)break;double T=clip(R/fmax(vc,20),.25,20);
  V3 amid=midcourse(r,vr,amax),aterm=terminal(law,r,vr,tg.a,vf,T,amax);double req=norm(aterm),age=.035+.015*(.5+.5*sin(.9*t)),nis=4+2*(.5+.5*sin(.4*t)),cov=180+60*(.5+.5*cos(.3*t));
  Conditions c=conditions(r,vr,r,age,nis,cov,req,amax,true);Mode before=h.mode;update_handoff(&h,c,t);if(before==MIDCOURSE&&h.mode==HANDOFF)handoff_t=t;
  double w=h.mode==MIDCOURSE?0:(h.mode==TERMINAL?1:clip((t-h.start)/.2,0,1));w=w*w*(3-2*w);V3 cmd=limit(add(mul(amid,1-w),mul(aterm,w)),amax);
  if(norm(cmd)>.999*amax){sat++;}steps++;m.a=add(m.a,mul(sub(cmd,m.a),dt/tau));m.a=limit(m.a,amax);step(&m,m.a,dt);step(&tg,tg.a,dt);effort+=dot(m.a,m.a)*dt;peak=fmax(peak,norm(m.a));V3 rel=sub(tg.r,m.r);double d=norm(rel);if(d<best){best=d;best_t=t+dt;best_r=rel;best_vm=m.v;}
 }
 double angle=acos(clip(dot(unit(best_vm),unit(vf)),-1,1))*180/PI;
 printf("law=%d handoff=%.3f s miss=%.3f m closest_t=%.3f s peak=%.3f m/s2 effort=%.3f\n",law,handoff_t,best,best_t,peak,effort);
 printf("miss_vector=[%.3f %.3f %.3f] m terminal_direction_error=%.3f deg saturation=%.4f\n",best_r.x,best_r.y,best_r.z,angle,steps?(double)sat/steps:0);return 0;
}
