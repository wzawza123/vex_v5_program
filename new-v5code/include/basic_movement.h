/*vex-vision-config:begin*/
#ifndef BASIC_MOVEMENT_H
#define BASIC_MOVEMENT_H
#include "vex.h"
extern int VIN_SPEED;
extern int VSIDE_ARM_SPEED;
extern int VFRONT_ARM_SPEED;
extern int VFRONT_PAW_SPEED;
//control the chassis movement
void VRUN(double l, double r);
//control the paw
void Vpaw(double v, int temp);
//control the vin motor
void Vin(double v, int temp);
//大车侧向手臂控制，v：速度，temp：0/1 控制方向
void VSideArm(double v, int temp);
//大车前方平行四边形上臂驱动
void VFrontArm(double v,int temp);
//打车前方平行四边形末端爪子驱动
void VFrontPaw(double v,int temp);
//底盘运动
void vrun(double lv,double rv);
#endif