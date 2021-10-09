#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "v5.h"
#include "v5_vcs.h"

vex::brain Brain;
vex::controller Controller = vex::controller();
vex::motor l1 = vex::motor(vex::PORT10,true);
vex::motor l2 = vex::motor(vex::PORT9,true);
vex::motor l3 = vex::motor(vex::PORT8,true);
vex::motor l4 = vex::motor(vex::PORT7,true);
vex::motor r1 = vex::motor(vex::PORT1,false);
vex::motor r2 = vex::motor(vex::PORT2,false);
vex::motor r3 = vex::motor(vex::PORT3,false);
vex::motor r4 = vex::motor(vex::PORT4,false);
vex::motor bodl1 = vex::motor(vex::PORT20, true);//big
vex::motor bodr1 = vex::motor(vex::PORT11, false);
vex::motor bodl2 = vex::motor(vex::PORT6, false);
vex::motor bodr2 = vex::motor(vex::PORT5, true);//small
vex::motor eatl1 = vex::motor(vex::PORT17, false);
vex::motor eatr1 = vex::motor(vex::PORT13, true);
vex::motor eatl2 = vex::motor(vex::PORT18, false);
vex::motor eatr2 = vex::motor(vex::PORT14, true);
vex::motor arml = vex::motor(vex::PORT19,false);
vex::motor armr = vex::motor(vex::PORT12,true);
#define CPC (M_PI*10.16*0.714286*1.14)
#define cwheel CPC
#define AD(cm) (360*cm/CPC*0.6)
#define VSEC vex::timeUnits::sec
#define VDEG vex::rotationUnits::deg
#define VPCT vex::velocityUnits::pct
#define VRPM vex::velocityUnits::rpm
#define VFWD vex::directionType::fwd
#define RTT(a,b,c) a.rotateFor(b,VSEC,c,VPCT);
#define ROTS(a,b,c) a.startRotateTo(b,VDEG,c,VPCT);
#define ROT(a,b,c) a.rotateTo(b,VDEG,c,VPCT);
#define ROFS(a,b,c) a.startRotateFor(b,VDEG,c,VPCT);
#define ROF(a,b,c) a.rotateFor(b,VDEG,c,VPCT);
#define SP(a,b) a.spin(VFWD,b,VPCT);
#define AV(a) Controller.Axis##a.value()
#define FAV(a) (abs(AV(a))<15?0:(100.0*AV(a)/127)) 
#define VST(a,b) vexMotorVoltageSet(vex::PORT##a, b);
#define BP(a) Controller.Button##a.pressing()
#define SLEEP(a) vex::task::sleep(a);
#define LIM(a) (a>100?100:(a<-100?-100:a))
#define K 1    //red
#define AF(l,v) {ROFS(l1,AD(l),v)ROFS(l2,AD(l),v)ROFS(l3,AD(l),v)ROFS(l4,AD(l),v)ROFS(r1,AD(l),v)ROFS(r2,AD(l),v)ROFS(r3,AD(l),v)ROF(r4,AD(l),v)}
//AF(CM,V)
#define AFS(l,v) {ROFS(l1,AD(l),v)ROFS(l2,AD(l),v)ROFS(l3,AD(l),v)ROFS(l4,AD(l),v)ROFS(r1,AD(l),v)ROFS(r2,AD(l),v)ROFS(r3,AD(l),v)ROFS(r4,AD(l),v)}
#define AFT(l,v) {ROTS(l1,AD(l),v)ROTS(l2,AD(l),v)ROTS(l3,AD(l),v)ROTS(l4,AD(l),v)ROTS(r1,AD(l),v)ROTS(r2,AD(l),v)ROTS(r3,AD(l),v)ROT(r4,AD(l),v)}
//AFT(CM,V) start rotate to ---cm
#define AT(ll,v) {double l=0.25*ll;ROFS(l1,K*AD(l),v)ROFS(l2,K*AD(l),v)ROFS(l3,K*AD(l),v)ROFS(l4,K*AD(l),v)ROFS(r1,-K*AD(l),v)ROFS(r2,-K*AD(l),v)ROFS(r3,-K*AD(l),v)ROF(r4,-K*AD(l),v)}
//AT(DEG,V)//turn right is + USING ENCODER
#define DEG(a) a.rotation(vex::rotationUnits::deg)
#define SMT(a,b) a.setMaxTorque(b,vex::percentUnits::pct);
#define ASMT(b) {SMT(l1,b)SMT(l2,b)SMT(l4,b)SMT(l3,b)SMT(r4,b)SMT(r1,b)SMT(r2,b)SMT(r3,b)}
#define ASMT_eat(b){SMT(eatl1,b)SMT(eatl2,b)SMT(eatr1,b)SMT(eatr2,b)}
#define ASP(v) {SP(l1,v)SP(l2,v)SP(l3,v)SP(l4,v)SP(r4,v)SP(r1,v)SP(r2,v)SP(r3,v)}
#define ASP_eat(v) {SP(eatl1,v)SP(eatl2,v)SP(eatr1,v)SP(eatr2,v)}
#define DAW(v,t) {ASMT(15)ASP(v)SLEEP(t)ASP(0)ASMT(100)}
//SPIN(V,T)
#define DAF(v,t) {ASMT_eat(10)ASP_eat(v)SLEEP(t)ASP_eat(0)ASMT_eat(50)}//ASMT depends on num of cubes eaten

/***************************************
the line below is the definition of gyro and functions
***************************************/

vex::analog_in gyro_in = vex::analog_in(Brain.ThreeWirePort.A);
vex::digital_out gyro_out = vex::digital_out(Brain.ThreeWirePort.B);

void initRobot();
void vclear();
void clearfly();
void vstop();
int lim(int a,int t);
double getEncoderValue();
void put(double l,double v);
void Sput(double l,double v);
void veat(double v);
void vrun(double lv,double rv);
void vbod(double v);
void vfly(double v);
//额当时没有在这里函数声明，习惯上把主函数写在最后（和竞赛模板结构有关）
//懒得粘了你们看看形式就可以了


void initGyro();

void underSpinSpeed(int leftSpeed, int rightSpeed, vex::velocityUnits units=vex::velocityUnits::pct);

void setUnderVoltage(int leftVoltage, int rightVoltage);

int getGyroValue();

int getRelativeAngle(int val, int refer);

void turnAngle(int angleVal);

void turnAngle_simple_accurate(int angleVal);

void turnAngle_simple(int angleVal);

void turnToTargetAccurate(int targetAngle);

void turnToTarget(int targetAngle);//-1800-+1800

