/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       C:\Users\HosnLS                                           */
/*    Created:      Tue Oct 27 2020                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// ---- END VEXCODE CONFIGURED DEVICES ----

//#define PORT3 PORT14

#include "vex.h"

using namespace vex;

int main() {
  int global_counter = 0;   //全局计数器
  // Initializing Robot Configuration. DO NOT REMOVE!
  initRobot();
  while(true){

    //chassis auto adjust and chassis move
#if EnableAutoChassisAdjust == 1
    if (Controller.ButtonUp.pressing()){
      setChassisVolt(0.4,0,0);
      autoChassisAdjust('f');
    }
    else if (Controller.ButtonDown.pressing()){
      setChassisVolt(-0.4,0,0);
      autoChassisAdjust('b');
    }
    else if (Controller.ButtonLeft.pressing()){
      setChassisVolt(0,-0.4,0);
      autoChassisAdjust('l');
    }
    else if (Controller.ButtonRight.pressing()){
      setChassisVolt(0,0.4,0);
      autoChassisAdjust('r');
    }
    else{
      getLAxis();
      setChassisVolt();
    }
#elif EnableAutoChassisAdjust == 2
    if (Controller.ButtonUp.pressing()){
        double a2 = Controller.Axis2.value();
        double a1 = Controller.Axis1.value();
        double sqr = sqrt(a2 * a2 + a1 * a1);
        if(sqr > 60){
          MasterAxisAngel = asin(a1 / sqr) * 57.2957795130823;
          MasterAxisAngel = (a2 < 0 && MasterAxisAngel >= 0) ? 180 - MasterAxisAngel : MasterAxisAngel;
          MasterAxisAngel = (a2 < 0 && MasterAxisAngel < 0) ? -MasterAxisAngel - 180 : MasterAxisAngel;
          MasterAxisAngel += MasterAxisAngel < 0 ? 360 : 0;
        }
    }
    else{
      getGyro();
      MasterAxisAngel = MasterAxisAngel + GyroDif;
      MasterGetLAxis();
      setChassisVolt();
    }
#else 
    getLAxis();
    setChassisVolt();
#endif

    //upper structure operation
    if(Controller.ButtonL2.pressing()){     //LT 爪正 中正 上反 --吸球
      if(Controller.ButtonR2.pressing())setThrowVolt('f');
      else setThrowVolt('b');
      setTranspoterVolt('f');
      setClawVolt('f');
    }
    else if(Controller.ButtonL1.pressing() ||Controller.ButtonR1.pressing() ||Controller.ButtonR2.pressing()){//LB RT RB
      //claw
      if(Controller.ButtonL1.pressing())setClawVolt('b');     //LB 爪反 --下放球
      else setClawVolt('s');
      //transport and throw
      if(Controller.ButtonR2.pressing()){         //RT 中正 上正 --吐球
        setTranspoterVolt('f');
        setThrowVolt('f');
      }
      else if(Controller.ButtonR1.pressing()){    //RB 中反 上反 --球吐过了吸回来
        setTranspoterVolt('b');
        setThrowVolt('b');
      }
      else{
        setTranspoterVolt('s');
        setThrowVolt('s');
      }
    }
    else{
      setClawVolt('s');
      setTranspoterVolt('s');
      setThrowVolt('s');
    }
    
    if(global_counter)getGyro();
    //getballColor();
    vex::this_thread::sleep_for(5);
    global_counter++;
    if(global_counter%20==0)debug_display();
  }
}

void debug_display(){
    /*Brain.Screen.clearScreen();
    Vision.takeSnapshot(1, 1);
    Brain.Screen.printAt(10, 100, "Largest : %d %d %d", Vision.objects[0].id, Vision.objects[0].width, Vision.objects[0].height);
    Vision.takeSnapshot(2, 1);
    Brain.Screen.printAt(10, 120, "Largest : %d %d %d", Vision.objects[0].id, Vision.objects[0].width, Vision.objects[0].height);
    Brain.Screen.printAt(10, 140, "Second  : %d %d %d", Vision.objects[1].id, Vision.objects[1].width, Vision.objects[1].height);*/
    Brain.Screen.printAt(10, 20, "LF Offset : %7f", ChassisLFOffset);
    Brain.Screen.printAt(10, 50, "LB Offset : %7f", ChassisLBOffset);
    Brain.Screen.printAt(10, 80, "RF Offset : %7f", ChassisRFOffset);
    Brain.Screen.printAt(10, 110, "RB Offset : %7f", ChassisRBOffset);
    //Brain.Screen.printAt(10, 110, "now f : %7f", ChassisForwardSpeed);

    Brain.Screen.printAt(10, 140, "GyroLast : %7f", GyroLast);
    Brain.Screen.printAt(10, 170, "GyroDif : %7f", GyroDif);
    Brain.Screen.printAt(10, 200, "GyroDifSum : %7f", GyroDifSum);

#if EnableAutoChassisAdjust == 2
    Brain.Screen.printAt(10, 230, "MasterAxisAngel : %7f", MasterAxisAngel);
#endif
    //Brain.Screen.printAt(10, 50, "LeftOffset : %.3f", LeftOffset);
    //Brain.Screen.printAt(10, 80, "LeftOffset : %.3f", RightOffset);
    //Brain.Screen.printAt(10, 110, "ballColor : %.3f", ballColor);
}

void initRobot(){
  //reset rotation
  lf1.resetRotation();
  lf2.resetRotation();
  lb1.resetRotation();
  lb2.resetRotation();
  rf1.resetRotation();
  rf2.resetRotation();
  rb1.resetRotation();
  rb1.resetRotation();

  lz.resetRotation();
  rz.resetRotation();
  ful.resetRotation();
  fur.resetRotation();
  fm.resetRotation();
  fd.resetRotation();
  bu.resetRotation();
  bd.resetRotation();
  initGyro();
}
void initGyro(){  //initiate Gyro
  gyro_out.set(false);
  vex::this_thread::sleep_for(200);
  gyro_out.set(true);
  vex::this_thread::sleep_for(100);
  GyroLast = (gyro_in.value(analogUnits::range12bit) - 335) / 8.183844;
}

void autoChassisAdjust(char mode){
      if (GyroDifSum > 0.1 || GyroDifSum < -0.1){  //update left and right offset to make it run directly
        if (mode == 'f'){
          ChassisLFOffset += GyroDifSum < 0 ? 0.01 : -0.01;
          ChassisLBOffset += GyroDifSum < 0 ? 0.01 : -0.01;
          ChassisRFOffset += GyroDifSum < 0 ? -0.01 : 0.01;
          ChassisRBOffset += GyroDifSum < 0 ? -0.01 : 0.01;
        }
        else if(mode == 'b'){
          ChassisLFOffset += GyroDifSum < 0 ? -0.01 : 0.01;
          ChassisLBOffset += GyroDifSum < 0 ? -0.01 : 0.01;
          ChassisRFOffset += GyroDifSum < 0 ? 0.01 : -0.01;
          ChassisRBOffset += GyroDifSum < 0 ? 0.01 : -0.01;
        }
        else if(mode == 'l'){
          ChassisLFOffset += GyroDifSum < 0 ? -0.01 : 0.01;
          ChassisLBOffset += GyroDifSum < 0 ? 0.01 : -0.01;
          ChassisRFOffset += GyroDifSum < 0 ? -0.01 : 0.01;
          ChassisRBOffset += GyroDifSum < 0 ? 0.01 : -0.01;
        }
        else{
          ChassisLFOffset += GyroDifSum < 0 ? 0.01 : -0.01;
          ChassisLBOffset += GyroDifSum < 0 ? -0.01 : 0.01;
          ChassisRFOffset += GyroDifSum < 0 ? 0.01 : -0.01;
          ChassisRBOffset += GyroDifSum < 0 ? -0.01 : 0.01;
        }
        ChassisLFOffset = ChassisLFOffset > 1.3 ? 1.3 : ChassisLFOffset < 0.7 ? 0.7 : ChassisLFOffset;
        ChassisLBOffset = ChassisLBOffset > 1.3 ? 1.3 : ChassisLBOffset < 0.7 ? 0.7 : ChassisLBOffset;
        ChassisRFOffset = ChassisRFOffset > 1.3 ? 1.3 : ChassisRFOffset < 0.7 ? 0.7 : ChassisRFOffset;
        ChassisRBOffset = ChassisRBOffset > 1.3 ? 1.2 : ChassisRBOffset < 0.7 ? 0.7 : ChassisRBOffset;
        GyroDifSum = 0;
      }
}

void getLAxis(){  //get left L axis value, store in LeftSpeed and RightSpeed
  double axisf = 0;
  double axisr = 0;
  double axisrt = 0;

#if ChassisControllerMode == 'l'
  axisf = Controller.Axis3.value() > AxisThreshold || Controller.Axis3.value() < -AxisThreshold? Controller.Axis3.value() : 0;
  axisr = Controller.Axis4.value() > AxisThreshold || Controller.Axis4.value() < -AxisThreshold? Controller.Axis4.value() : 0;
  axisrt = Controller.Axis1.value() > AxisThreshold || Controller.Axis1.value() < -AxisThreshold? Controller.Axis1.value() : 0;
#else
  axisf = Controller.Axis3.value() > AxisThreshold || Controller.Axis3.value() < -AxisThreshold? Controller.Axis3.value() : 0;
  axisr = Controller.Axis4.value() > AxisThreshold || Controller.Axis4.value() < -AxisThreshold? Controller.Axis4.value() : 0;
  axisrt = Controller.Axis1.value() > AxisThreshold || Controller.Axis1.value() < -AxisThreshold? Controller.Axis1.value() : 0;
#endif
  ChassisForwardSpeed = axisf / 127;    //0-1
  ChassisRightSpeed = axisr / 127;
  ChassisRotateSpeed = axisrt / 127;
}
void MasterGetLAxis(){  //王者荣耀模式的遥感检测
  double axisf = 0;
  double axisr = 0;
  double axisrt = 0;

  axisf = Controller.Axis3.value() > AxisThreshold || Controller.Axis3.value() < -AxisThreshold? Controller.Axis3.value() : 0;
  axisr = Controller.Axis4.value() > AxisThreshold || Controller.Axis4.value() < -AxisThreshold? Controller.Axis4.value() : 0;
  axisrt = Controller.Axis1.value() > AxisThreshold || Controller.Axis1.value() < -AxisThreshold? Controller.Axis1.value() : 0;
  
  ChassisForwardSpeed = (axisf * cos(MasterAxisAngel / 57.2957795130823) + axisr * sin(MasterAxisAngel / 57.2957795130823)) / 127;    //0-1
  ChassisRightSpeed = (-axisf * sin(MasterAxisAngel / 57.2957795130823) + axisr * cos(MasterAxisAngel / 57.2957795130823)) / 127;
  ChassisRotateSpeed = axisrt / 127;
}

void getGyro(){
  double GyroNow = (gyro_in.value(analogUnits::range12bit) - 335) / 8.183844;  //0 - 359
  GyroDif = -(GyroNow - GyroLast);                 //update GyroDif
  GyroDif = (GyroDif > 200) ? (GyroDif - 360) : (GyroDif < -200 ? GyroDif + 360 : GyroDif);
  GyroLast = GyroNow;                           //update GryoNow

  GyroDifSum = GyroDifSum * GyroDifSumEWM + (1 - GyroDifSumEWM) * GyroDif;                  //update GyroDifSum
}

void getballColor(){//摄像机检测球的颜色的函数，使用了移动平均来减慢检测变化
  int sig1_w, sig1_h;
  Vision.takeSnapshot(1, 1);
  sig1_w = Vision.objects[0].width;
  sig1_h = Vision.objects[0].height;
  int sig2_w, sig2_h;
  Vision.takeSnapshot(2, 1);
  sig2_w = Vision.objects[0].width;
  sig2_h = Vision.objects[0].height;
  if(sig1_w > sig2_w && sig1_h > sig2_h){
    ballColor = ballColor * BallColorDetectionEWM + 1 - BallColorDetectionEWM;
  }
  else if(sig1_w < sig2_w && sig1_h < sig2_h){
    ballColor = ballColor * BallColorDetectionEWM - 1 + BallColorDetectionEWM;
  }
  else{
    ballColor = ballColor * BallColorDetectionEWM;
  }
}

void setChassisVolt(){   //设置底盘电压
  double lfVolt = ChassisForwardSpeed + ChassisRightSpeed + ChassisRotateSpeed;
  double lbVolt = ChassisForwardSpeed - ChassisRightSpeed + ChassisRotateSpeed;
  double rfVolt = ChassisForwardSpeed - ChassisRightSpeed - ChassisRotateSpeed;
  double rbVolt = ChassisForwardSpeed + ChassisRightSpeed - ChassisRotateSpeed;
    
  lfVolt *= 12000 * ChassisLFOffset;// * ChassisLFOffset;
  lbVolt *= 12000 * ChassisLBOffset;// * ChassisLBOffset;
  rfVolt *= 12000 * ChassisRFOffset;// * ChassisRFOffset;
  rbVolt *= 12000 * ChassisRBOffset;// * ChassisRBOffset;

  double limVolt = max(max(abs(lfVolt),abs(lbVolt)), max(abs(rfVolt),abs(rbVolt)));     //get maximun speed
  limVolt /=12000;

  if(limVolt > 1){                                                //if volt touch the limit, reduce it with scale in order to keep dicrection
    lfVolt = lfVolt / limVolt;
    lbVolt = lbVolt / limVolt;
    rfVolt = rfVolt / limVolt;
    rbVolt = rbVolt / limVolt;
  }

  vexMotorVoltageSet(vex::PORT7,  lfVolt);
  vexMotorVoltageSet(vex::PORT8, lfVolt);
  vexMotorVoltageSet(vex::PORT9,  lbVolt);
  vexMotorVoltageSet(vex::PORT10, lbVolt);
  vexMotorVoltageSet(vex::PORT17,  rfVolt);
  vexMotorVoltageSet(vex::PORT18, rfVolt);
  vexMotorVoltageSet(vex::PORT19,  rbVolt);
  vexMotorVoltageSet(vex::PORT20, rbVolt);
}

void setChassisVolt(double f, double r, double rt){   // set the volt of Chassis
  double lfVolt = f + r + rt;
  double lbVolt = f - r + rt;
  double rfVolt = f - r - rt;
  double rbVolt = f + r - rt;
  
  lfVolt *= 12000 * ChassisLFOffset;// * ChassisLFOffset;
  lbVolt *= 12000 * ChassisLBOffset;// * ChassisLBOffset;
  rfVolt *= 12000 * ChassisRFOffset;// * ChassisRFOffset;
  rbVolt *= 12000 * ChassisRBOffset;// * ChassisRBOffset;

  double limVolt = max(max(abs(lfVolt),abs(lbVolt)), max(abs(rfVolt),abs(rbVolt)));     //get maximun speed
  limVolt /=12000;

  if(limVolt > 1){                                                //if volt touch the limit, reduce it with scale in order to keep dicrection
    lfVolt = lfVolt / limVolt;
    lbVolt = lbVolt / limVolt;
    rfVolt = rfVolt / limVolt;
    rbVolt = rbVolt / limVolt;
  }

  vexMotorVoltageSet(vex::PORT7,  lfVolt);
  vexMotorVoltageSet(vex::PORT8, lfVolt);
  vexMotorVoltageSet(vex::PORT9,  lbVolt);
  vexMotorVoltageSet(vex::PORT10, lbVolt);
  vexMotorVoltageSet(vex::PORT17,  rfVolt);
  vexMotorVoltageSet(vex::PORT18, rfVolt);
  vexMotorVoltageSet(vex::PORT19,  rbVolt);
  vexMotorVoltageSet(vex::PORT20, rbVolt);
}

void setClawVolt(char c){   //设置爪子转向
  if(c == 'f'){
    vexMotorVoltageSet(vex::PORT1,  12000);
    vexMotorVoltageSet(vex::PORT2,  12000);
  }
  else if(c == 'b'){
    vexMotorVoltageSet(vex::PORT1,  -12000);
    vexMotorVoltageSet(vex::PORT2,  -12000);
  }
  else if(c == 's'){
    vexMotorVoltageSet(vex::PORT1,  0);
    vexMotorVoltageSet(vex::PORT2,  0);
  }
}

void setTranspoterVolt(char c){   //设置传送带转向
  if(c == 'f'){   //向上送球
    vexMotorVoltageSet(vex::PORT5,  12000);
    vexMotorVoltageSet(vex::PORT6,  12000);
    vexMotorVoltageSet(vex::PORT11,  12000);
    vexMotorVoltageSet(vex::PORT12,  12000);
  }
  else if(c == 'b'){//向下送球
    vexMotorVoltageSet(vex::PORT5,  -12000);
    vexMotorVoltageSet(vex::PORT6,  -12000);
    vexMotorVoltageSet(vex::PORT11,  -12000);
    vexMotorVoltageSet(vex::PORT12,  -12000);
  }
  else if (c == 't'){//向后吐球
    vexMotorVoltageSet(vex::PORT5,  12000);
    vexMotorVoltageSet(vex::PORT6,  12000);
    vexMotorVoltageSet(vex::PORT11,  -12000);
    vexMotorVoltageSet(vex::PORT12,  12000);
  }
  else if(c == 's'){
    vexMotorVoltageSet(vex::PORT5,  0);
    vexMotorVoltageSet(vex::PORT6,  0);
    vexMotorVoltageSet(vex::PORT11,  0);
    vexMotorVoltageSet(vex::PORT12,  0);
  }
}

void setThrowVolt(char c){        //设置最上方滚筒转向
  if(c == 'f'){
    vexMotorVoltageSet(vex::PORT3,  12000);
    vexMotorVoltageSet(vex::PORT4,  12000);
  }
  else if(c == 'b'){
    vexMotorVoltageSet(vex::PORT3,  -12000);
    vexMotorVoltageSet(vex::PORT4,  -12000);
  }
  else if(c == 's'){
    vexMotorVoltageSet(vex::PORT3,  0);
    vexMotorVoltageSet(vex::PORT4,  0);
  }
}

/*
void setTranspoterVolt(char c){
  if(c == 'f'){
    vexMotorVoltageSet(vex::PORT1,  12000);
    vexMotorVoltageSet(vex::PORT2,  12000);
    vexMotorVoltageSet(vex::PORT3,  12000);
    vexMotorVoltageSet(vex::PORT4,  12000);
    vexMotorVoltageSet(vex::PORT5,  12000);
    vexMotorVoltageSet(vex::PORT6,  12000);
  }
  else if(c == 'b'){
    vexMotorVoltageSet(vex::PORT1,  12000);
    vexMotorVoltageSet(vex::PORT2,  12000);
    vexMotorVoltageSet(vex::PORT3,  -12000);
    vexMotorVoltageSet(vex::PORT4,  12000);
    vexMotorVoltageSet(vex::PORT5,  12000);
    vexMotorVoltageSet(vex::PORT6,  12000);
  }
  else if(c == 's'){
    vexMotorVoltageSet(vex::PORT1,  0);
    vexMotorVoltageSet(vex::PORT2,  0);
    vexMotorVoltageSet(vex::PORT3,  0);
    vexMotorVoltageSet(vex::PORT4,  0);
    vexMotorVoltageSet(vex::PORT5,  0);
    vexMotorVoltageSet(vex::PORT6,  0);
  }
}
*/