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
MOTOR(rf,6,false)MOTOR(rm1,8,true)MOTOR(rm2,9,true)MOTOR(rb,10,false)
MOTOR(lf,1,true)MOTOR(lm1,2,false)MOTOR(lm2,3,false)MOTOR(lb,4,true)
MOTOR(lup,11,true)MOTOR(rup,20,false)
MOTOR(inhale,12,true)
MOTOR(frontarm1,17,false)
MOTOR(frontarm2,15,true)
MOTOR(frontpaw,19,false)
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
