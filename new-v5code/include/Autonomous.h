#ifndef AUTONOMOUS_H
#define AUTONOMOUS_H
/*vex-vision-config:begin*/
#include "vex.h"

//reset chassis encoder
void resetChassisEncoder();
//brake the chassis
void stopChassis();
//get encoder value of oneside (default:left)
float getSideChassisEncoder(bool isLeft=true);
//get chassis encoder
float getChassisEncoder();
//to limit a in [-t,t]
int lim(int a,int t);
//run cm distance with velocity of v, and is forward by k
void vdis(double cm,double v,int k);
void auto_runDistance_smoothly(float dist_cm);
void auto_runDistance(float dist_cm);
//rotate distance clockwise
void auto_rotate_chassis(double distance);
////////////////////////////////////////////////////////////////
//autonomous process
//start ring installation
void auto_ringsStart();
//hold the move goal
void auto_drop_paw();
//lift the paw
void auto_lift_paw();
//auto hold the moving goal
void auto_drop_and_hold_side();
#endif