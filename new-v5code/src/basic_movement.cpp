#include "basic_movement.h"
int VIN_SPEED = 65;  //吸环电机速度
int VSIDE_ARM_SPEED = 100; //侧向手臂电机速度
int VFRONT_ARM_SPEED=100; //前向手臂电机速度
int VFRONT_PAW_SPEED=60; //前向手臂末端爪子电机速度

void VRUN(double l, double r) {
  VVST(1, 2, l)
  VVST(3, 4, l)
  VVST(6,8, r)
  VVST(9,10, r)
}

void Vpaw(double v, int temp) // temp is 1 or -1
{
  int V = v * temp;
  if (temp) {
    // if(DEG(lup)<=135)
    VVST(11, 20, V)
  }
  if (!temp) {
    // if(DEG(lup)<=135)
    VVST(11, 20, V)
  }
}

void Vin(double v, int temp) // thw大车组吸球测试
{

  int V = v * temp;
  if (temp) {
    // if(DEG(lup)<=135)
    VST(12, V * 120)
  }
  if (!temp) {
    // if(DEG(lup)<=135)
    VST(12, V * 120)
  }
}

//大车侧向手臂控制，v：速度，temp：0/1 控制方向
void VSideArm(double v, int temp) {
  int V = v * temp;
  VST(16, V * 120);
}
//大车前方平行四边形上臂驱动
void VFrontArm(double v,int temp){
  int V=v*temp;
  VST(17, V * 120);
  VST(18, V * 120);
}
//打车前方平行四边形末端爪子驱动
void VFrontPaw(double v,int temp){
  int V=v*temp;
  VST(19,V*120);
}
//底盘运动
void vrun(double lv,double rv)
//底盘电机加电压
//Vrun
{
    lv*=120,rv*=120;
    VST(1,lv)VST(2,lv)VST(3,lv)VST(4,lv)
    VST(10,rv)VST(9,rv)VST(8,rv)VST(7,rv)
}