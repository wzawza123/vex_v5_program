/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       vex.h                                                     */
/*    Author:       Vex Robotics                                              */
/*    Created:      1 Feb 2019                                                */
/*    Description:  Default header for V5 projects                            */
/*                                                                            */
/*----------------------------------------------------------------------------*/
//
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "v5.h"
#include "v5_vcs.h"

//硬件定义
vex::brain Brain;
vex::controller Controller = vex::controller();

//正反转需要调整一下
vex::motor l1 = vex::motor(vex::PORT4,true);
vex::motor l2 = vex::motor(vex::PORT3,true);
vex::motor l3 = vex::motor(vex::PORT11,false);

vex::motor r1 = vex::motor(vex::PORT9,false);
vex::motor r2 = vex::motor(vex::PORT10,false);
vex::motor r3 = vex::motor(vex::PORT13,true);

vex::motor u1 = vex::motor(vex::PORT17,false);
vex::motor u2 = vex::motor(vex::PORT18,false);
vex::motor u3 = vex::motor(vex::PORT19 ,false);

vex::motor eat1 = vex::motor(vex::PORT1,true);
vex::motor eat2 = vex::motor(vex::PORT16,false);

//通用头文件，无需修改
#define CPC (M_PI*10.16*0.714286*1.14)//需要调试
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
#define K 1 
//通用头文件，以下均需要根据底盘电机修改
#define AF(l,v) {ROFS(l1,AD(l),v)ROFS(l2,AD(l),v)ROFS(l3,AD(l),v)ROFS(r1,AD(l),v)ROFS(r2,AD(l),v)ROF(r3,AD(l),v)}
//AF(CM,V)
#define AFS(l,v) {ROFS(l1,AD(l),v)ROFS(l2,AD(l),v)ROFS(l3,AD(l),v)ROFS(r1,AD(l),v)ROFS(r2,AD(l),v)ROFS(r3,AD(l),v)}//startRotateFor
//






/////////////////////留意
#define AT(ll,v) {double l=0.27*ll;ROFS(l1,K*AD(l),v)ROFS(l2,K*AD(l),v)ROFS(l3,K*AD(l),v)ROFS(r1,-K*AD(l),v)ROFS(r2,-K*AD(l),v)ROF(r3,-K*AD(l),v)}
//AT(DEG,V)//turn right is + USING ENCODER









#define DEG(a) a.rotation(vex::rotationUnits::deg)
#define SMT(a,b) a.setMaxTorque(b,vex::percentUnits::pct);
#define ASMT(b) {SMT(l1,b)SMT(l2,b)SMT(l4,b)SMT(l3,b)SMT(r4,b)SMT(r1,b)SMT(r2,b)SMT(r3,b)}
#define ASMT_eat(b){SMT(eatl1,b)SMT(eatl2,b)SMT(eatr1,b)SMT(eatr2,b)}

#define DAW(v,t) {ASMT(15)ASP(v)SLEEP(t)ASP(0)ASMT(100)}
//SPIN(V,T)


//陀螺仪定义
vex::analog_in gyro_in = vex::analog_in(Brain.ThreeWirePort.A);
vex::digital_out gyro_out = vex::digital_out(Brain.ThreeWirePort.B);

//--------------------------函数声明-------------------------
void vrun(double lv,double rv);

#define GTD(a) ((double)(1811-a)/8.16833)
#define GDEG GTD(gyro_in.value(vex::analogUnits::range12bit))

double ii=0;