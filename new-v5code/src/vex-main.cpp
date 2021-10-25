#include "vex.h"
vex::brain Brain;
vex::controller Controller = vex::controller();
MOTOR(rf,6,false)
MOTOR(rm1,8,true)
MOTOR(rm2,9,true)
MOTOR(rb,10,false)
MOTOR(lf,1,true)
MOTOR(lm1,2,false)
MOTOR(lm2,3,false)
MOTOR(lb,4,true)
MOTOR(lup,11,true)
MOTOR(rup,20,false)
MOTOR(inhale,12,true)
MOTOR(frontarm1,17,false)
MOTOR(frontarm2,18,true)
MOTOR(frontpaw,19,false)
void autonomous(){
  float firstDistance=60;
  float secondDistance=30;
  float thirdDistance=20;
  SLEEP(3000);
  resetChassisEncoder();
  auto_ringsStart();
  auto_runDistance(firstDistance);
  auto_drop_paw();
  SLEEP(500);
  auto_runDistance(-secondDistance);
  auto_rotate_chassis(-800);
  auto_lift_paw();
  auto_runDistance(-thirdDistance);
  SLEEP(500);
}
void manual(){
  // 工作主循环
  while (true) {
    //控制底盘的移动
    if (std::abs(FAV(3)) != 0 || std::abs(FAV(1)) != 0)
      VRUN(0.8 * FAV(3) + 0.8 * FAV(1), 0.8 * FAV(3) - 0.8 * FAV(1));
    else
      VRUN(0, 0);
    //前方的爪子的控制
    if (BP(L1))  //检测按键L1是否按下
      Vpaw(50, 1);
    if (BP(L2))
      Vpaw(70, -1);
    if (!BP(L1) && !BP(L2))
      Vpaw(0, 1);
    if (BP(L1) && BP(L2))
      Vpaw(0, 1);
    //吸环电击的控制
    if (BP(R1))
      Vin(VIN_SPEED, 1);
    if (BP(R2))
      Vin(VIN_SPEED, -1);
    if (!BP(R1) && !BP(R2))
      Vin(0, 1);
    if (BP(R1) && BP(R2))
      Vin(0, 1);
    //侧向套环手臂控制
    if(BP(X)){
      VSideArm(VSIDE_ARM_SPEED,1);
    }else
    if(BP(A)){
      //放下来
      VSideArm(VSIDE_ARM_SPEED,-1);
    }else{
      VSideArm(0,1);
    }
    //前项手臂控制
    if(BP(Up)){
      VFrontArm(VFRONT_ARM_SPEED,1);
    }else
    if(BP(Down)){
      VFrontArm(VFRONT_ARM_SPEED,-1);
    }
    else{
      VFrontArm(0,0);
    }
    if(BP(Left)){
      VFrontPaw(VFRONT_PAW_SPEED,1);
    }else
    if(BP(Right)){
      VFrontPaw(VFRONT_PAW_SPEED,-1);
    }else{
      VFrontPaw(0,0);
    }
    //循环间延时，防止程序卡死
    SLEEP(8);
  }
}
int main() {
  //autonomous();
  //SLEEP(1000);
  manual();
  return 0;
}