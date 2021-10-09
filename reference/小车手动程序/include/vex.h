/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       vex.h                                                     */
/*    Author:       Vex Robotics                                              */
/*    Created:      1 Feb 2019                                                */
/*    Description:  Default header for V5 projects                            */
/*                                                                            */
/*----------------------------------------------------------------------------*/
//
#include <cmath>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "v5.h"
#include "v5_vcs.h"

using namespace vex;
#define waitUntil(condition)                                                   \
  do {                                                                         \
    wait(5, msec);                                                             \
  } while (!(condition))

#define repeat(iterations)                                                     \
  for (int iterator = 0; iterator < iterations; iterator++)

vex::brain Brain;
vex::controller Controller = vex::controller();
vex::motor lf1 = vex::motor(vex::PORT4);
vex::motor lf0 = vex::motor(vex::PORT9,true);

vex::motor lb1 = vex::motor(vex::PORT3);
vex::motor lb0 = vex::motor(vex::PORT1,true);

vex::motor rf1 = vex::motor(vex::PORT19);
vex::motor rf0 = vex::motor(vex::PORT17,true);

vex::motor rb1 = vex::motor(vex::PORT18);
vex::motor rb0 = vex::motor(vex::PORT20,true);

vex::motor h1 = vex::motor(vex::PORT5,true);
vex::motor hr = vex::motor(vex::PORT15);

vex::motor bu = vex::motor(vex::PORT6);
vex::motor bd = vex::motor(vex::PORT16,true);

vex::motor fdl = vex::motor(vex::PORT7,true);
vex::motor ful = vex::motor(vex::PORT8,true);

vex::motor fdr = vex::motor(vex::PORT14);
vex::motor fur = vex::motor(vex::PORT2);


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


void initGyro();

void underSpinSpeed(int leftSpeed, int rightSpeed, vex::velocityUnits units=vex::velocityUnits::pct);

void setUnderVoltage(int leftVoltage, int rightVoltage);

int getGyroValue();

int getRelativeAngle(int val, int refer);

void turnAngle(int angleVal);

void turnAngle_simple(int angleVal);

void turnToTarget(int targetAngle);

vex::analog_in gyro_in = vex::analog_in(Brain.ThreeWirePort.A);
vex::digital_out gyro_out = vex::digital_out(Brain.ThreeWirePort.B);
