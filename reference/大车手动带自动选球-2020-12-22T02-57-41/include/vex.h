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
vex::brain Brain;
//摄像头定义
vex::vision::signature BLUE (1, -4063, -2689, -3376, 7939, 11865, 9902, 3, 0);
vex::vision::signature RED (2, 6869, 10447, 8658, -443, 589, 73, 3, 0);
vex::vision::signature SIG_3 (3, 0, 0, 0, 0, 0, 0, 2.4000000953674316, 0);
vex::vision::signature SIG_4 (4, 0, 0, 0, 0, 0, 0, 2.5, 0);
vex::vision::signature SIG_5 (5, 0, 0, 0, 0, 0, 0, 2.700000047683716, 0);
vex::vision::signature SIG_6 (6, 0, 0, 0, 0, 0, 0, 2.0999999046325684, 0);
vex::vision::signature SIG_7 (7, 0, 0, 0, 0, 0, 0, 1.7999999523162842, 0);
vex::vision Vision (vex::PORT18, 50, BLUE, RED, SIG_3, SIG_4, SIG_5, SIG_6, SIG_7);//记得改端口

vex::controller Controller = vex::controller();
vex::sonar fsn =vex::sonar(Brain.ThreeWirePort.C);
vex::vision::signature GREEN (1, -5899, -4443, -5171, -6605, -5173, -5889, 3, 0);

/*vex::vision::signature BLUE (1, -3553, -2097, -2825, 4211, 6871, 5541, 1.8, 0);
vex::vision::signature RED (2, 6343, 9859, 8101, -1899, -651, -1275, 0.8, 0);
vex::vision::signature VOID (4, 0, 0, 0, 0, 0, 0, 3, 0);
vex::vision vin (vex::PORT7, 50, BLUE,RED, VOID, VOID,VOID,VOID,VOID);
vex::vision vout(vex::PORT15, 50, GREEN,VOID,VOID,VOID,VOID,VOID,VOID);
*/
#define DIS(a) a.distance(vex::distanceUnits::mm)
#define MOTOR(a,b,c) vex::motor a =vex::motor(vex::PORT##b,c);
MOTOR(lf1,12,true)MOTOR(lm0,13,false)MOTOR(lb1,15,true)MOTOR(lup,16,true)
MOTOR(rf0,2,false)MOTOR(rm1,3,true)MOTOR(rb0,4,false)MOTOR(rup,5,false)
MOTOR(fpawl,11,true)MOTOR(fpawr,1,false)MOTOR(bpaw,10,true)
MOTOR(fd,9,false)MOTOR(fm,6,false)MOTOR(fu,8,false)
MOTOR(sl,20,false)


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
#define CCV chooseColor.value(vex::analogUnits::range12bit)

vex::analog_in gyro_in = vex::analog_in(Brain.ThreeWirePort.A);
vex::digital_out gyro_out = vex::digital_out(Brain.ThreeWirePort.B);
vex::analog_in chooseColor = vex::analog_in(Brain.ThreeWirePort.E);

