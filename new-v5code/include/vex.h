#ifndef VEX_H
#define VEX_H
#include <cmath>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "v5.h"
#include "v5_vcs.h"
#include "Autonomous.h"
#include "basic_movement.h"
extern vex::brain Brain;
extern vex::controller Controller;
extern vex::motor rf;
extern vex::motor rm1;
extern vex::motor rm2;
extern vex::motor rb;
extern vex::motor lf;
extern vex::motor lm1;
extern vex::motor lm2;
extern vex::motor lb;
extern vex::motor lup;
extern vex::motor rup;
extern vex::motor inhale;
extern vex::motor frontarm1;
extern vex::motor frontarm2;
extern vex::motor frontpaw;


#define DIS(a) a.distance(vex::distanceUnits::mm)
#define MOTOR(a,b,c) vex::motor a =vex::motor(vex::PORT##b,c);

//MOTOR(dup,1,true)MOTOR(uup,3,false)
#define VSEC vex::timeUnits::sec
#define VDEG vex::rotationUnits::deg
#define VPCT vex::velocityUnits::pct
#define VRPM vex::velocityUnits::rpm
#define VFWD vex::directionType::fwd
#define VST(a,b); vexMotorVoltageSet(vex::PORT##a, b);//a is port b is
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
#define GTD(a) ((double)(1811 - a) / 8.16833)
#define GDEG GTD(gyr.value(vex::analogUnits::range12bit))
#define LO(a) a.largestObject
#define ERR(a, b, c) ((a > b + c) || (a < b - c))
#endif