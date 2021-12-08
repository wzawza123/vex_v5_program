/*
 * @Description: 
 * @Date: 2021-10-12 16:45:16
 * @LastEditTime: 2021-11-20 22:48:20
 */
#include <cmath>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "v5.h"
#include "v5_vcs.h"
vex::brain Brain;
vex::controller Controller = vex::controller();
#define DIS(a) a.distance(vex::distanceUnits::mm)
#define MOTOR(a,b,c) vex::motor a =vex::motor(vex::PORT##b,c);

auto  L1= vex::PORT1;
auto  L2= vex::PORT2;
auto  L3= vex::PORT3;
auto  L4= vex::PORT4;
auto  FRONT_PAW= vex::PORT5;
auto  R1= vex::PORT6;
auto  R2= vex::PORT8;
auto  R3= vex::PORT9;
auto  R4= vex::PORT7;
auto  UP1= vex::PORT12;
auto  UP2= vex::PORT13;
auto  SIDE_ARM= vex::PORT14;
auto  FRONT_ARM1= vex::PORT11;
auto  FRONT_ARM2= vex::PORT18;
auto  TAKEIN1= vex::PORT19;
auto  TAKEIN2= vex::PORT20;

vex::motor rf=vex::motor(R1,false);
vex::motor rm1=vex::motor(R2,true);
vex::motor rm2=vex::motor(R3,true);
vex::motor rb=vex::motor(R4,false);
vex::motor lf=vex::motor(L1,true);
vex::motor lm1=vex::motor(L2,false);
vex::motor lm2=vex::motor(L3,false);
vex::motor lb=vex::motor(L4,true);
vex::motor lup=vex::motor(UP1,false);
vex::motor rup=vex::motor(UP2,true);
vex::motor inhale=vex::motor(TAKEIN1,true);
vex::motor inhale2=vex::motor(TAKEIN2,false);
vex::motor frontarm1=vex::motor(FRONT_ARM1,false);
vex::motor frontarm2=vex::motor(FRONT_ARM2,true);
vex::motor frontpaw=vex::motor(FRONT_PAW,false);


//MOTOR(dup,1,true)MOTOR(uup,3,false)
#define VSEC vex::timeUnits::sec
#define VDEG vex::rotationUnits::deg
#define VPCT vex::velocityUnits::pct
#define VRPM vex::velocityUnits::rpm
#define VFWD vex::directionType::fwd
#define VST(a,b); vexMotorVoltageSet(a, b);//a is port b is
#define ROTS(a,b,c); a.startRotateTo(b,VDEG,c,VPCT);//a is motor ,b is deg,c is pct
#define ROT(a,b,c); a.rotateTo(b,VDEG,c,VPCT);//a is motor ,b is deg,c is pct
#define ROFS(a,b,c); a.startRotateFor(b,VDEG,c,VPCT);//a is motor ,b is deg,c is pct
#define ROF(a,b,c); a.rotateFor(b,VDEG,c,VPCT);//
#define RST(a); a.resetRotation();//a is motor
#define DEG(a) a.rotation(VDEG)//a is motor
#define SP(a,b); {a.spin(VFWD,b,VPCT);}//a is motor
#define AV(a) Controller.Axis##a.value()//a is controller Axis
#define BP(a) Controller.Button##a.pressing()//a is controller Button
#define SLEEP(a); vex::task::sleep(a);//a is time
#define LIM(a) (a>100?100:(a<-100?-100:a))//no use
#define CPC (M_PI*10.16)//M_PI is 3.14....
#define AD(cm) (360*cm/CPC)//turn degree into cm
#define VVST(a,b,c){VST(a,(c)*120)VST(b,(c)*120)}
#define VSTT(a,c){VST(a,c)}
#define FAV(a)(abs(AV(a))<15?0:(100.0*AV(a)/127))
