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

#define SROF(a,b) a.startRotateFor(b, rotationUnits::deg);


using namespace vex;
int global_counter = 0;   //全局计数器

int main() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  initRobot();
  task Task_Sensor = task(getSensor);
  task Task_Operation = task(operationMain);
  while(true){
    global_counter++;
    this_thread::sleep_for(10);
    if(global_counter%20==0)debug_display();
  }
}

int operationMain(){/*
  lf1.startRotateFor(1000, rotationUnits::deg);
  lf2.startRotateFor(1000, rotationUnits::deg);
  lb1.startRotateFor(1000, rotationUnits::deg);
  lb2.startRotateFor(1000, rotationUnits::deg);
  rf1.startRotateFor(1000, rotationUnits::deg);
  rf2.startRotateFor(1000, rotationUnits::deg);
  rb1.startRotateFor(1000, rotationUnits::deg);
  rb2.rotateFor(1000, rotationUnits::deg);*/

  //ChassisPIDTurn(-180);
  //ChassisPIDMove('f', 1000);
  //ChassisPIDMove('r', 1000);
  //ChassisPIDMove('l', 1000);
  ChassisPIDMove('f', 1000);
  //ChassisPIDTurn(-180);

  return 0;
}
void initRobot(){
  //reset rotation
  Vision.setLedBrightness(100);
  Vision.setLedColor(255, 255, 255);
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
  //gyro_out.set(false);
  //vex::this_thread::sleep_for(2000);
  gyro_out.set(true);
  vex::this_thread::sleep_for(1500);
  GyroLast = (gyro_in.value(analogUnits::range12bit) - 335) / 8.183844;
  GyroCon = 0;
}

int getSensor(){
  while(true){
    //gyro
    double GyroNow = (gyro_in.value(analogUnits::range12bit) - 335) / 8.183844;  //0 - 359
    GyroDif = -(GyroNow - GyroLast);                 //update GyroDif
    GyroDif = (GyroDif > 200) ? (GyroDif - 360) : (GyroDif < -200 ? GyroDif + 360 : GyroDif);
    GyroCon += GyroDif;
    GyroLast = GyroNow;                           //update GryoNow
    GyroDifSum = GyroDifSum * GyroDifSumEWM + (1 - GyroDifSumEWM) * GyroDif;                  //update GyroDifSum

    //camera
    /*
    int sig1_w, sig1_h;
    Vision.takeSnapshot(1, 1);
    sig1_w = Vision.objects[0].width;
    sig1_h = Vision.objects[0].height;
    int sig2_w, sig2_h;
    Vision.takeSnapshot(2, 1);
    sig2_w = Vision.objects[0].width;
    sig2_h = Vision.objects[0].height;
    if(sig1_w > sig2_w && sig1_h > sig2_h) ballColor = ballColor * BallColorDetectionEWM + 1 - BallColorDetectionEWM;
    else if(sig1_w < sig2_w && sig1_h < sig2_h) ballColor = ballColor * BallColorDetectionEWM - 1 + BallColorDetectionEWM;
    else ballColor = ballColor * BallColorDetectionEWM;
    this_thread::sleep_for(10);
    */
    this_thread::sleep_for(2);
  }
  return 0;
}

void debug_display(){
    /*Brain.Screen.clearScreen();
    Vision.takeSnapshot(1, 1);
    Brain.Screen.printAt(10, 100, "Largest : %d %d %d", Vision.objects[0].id, Vision.objects[0].width, Vision.objects[0].height);
    Vision.takeSnapshot(2, 1);
    Brain.Screen.printAt(10, 120, "Largest : %d %d %d", Vision.objects[0].id, Vision.objects[0].width, Vision.objects[0].height);
    Brain.Screen.printAt(10, 140, "Second  : %d %d %d", Vision.objects[1].id, Vision.objects[1].width, Vision.objects[1].height);*/
    //Brain.Screen.printAt(10, 20, "LF Offset : %7f", ChassisLFOffset);
    //Brain.Screen.printAt(10, 20, "i : %7f", ChassisLFOffset);
    Brain.Screen.printAt(10, 80, "RF Offset : %7f", ChassisRFOffset);
    Brain.Screen.printAt(10, 110, "RB Offset : %7f", ChassisRBOffset);

    Brain.Screen.printAt(10, 140, "GyroLast : %7f", GyroCon);
    Brain.Screen.printAt(10, 170, "GyroDif : %7f", GyroDif);
    Brain.Screen.printAt(10, 200, "GyroDifSum : %7f", GyroDifSum);
}

void ChassisMove(char mode, double dist){

}
void ChassisTurn(double angel){

}

void ChassisPIDMove(char mode, double dist){
  if(mode == 'f'){
    Brain.Screen.printAt(10, 20, "i : %5f", lf1.rotation(rotationUnits::deg));
    double target_v = (lf1.rotation(rotationUnits::deg) + lf2.rotation(rotationUnits::deg)) / 2 + dist;
    double target_a = GyroCon;
    PID_move MOVE(target_v);
    PID_turn_none ANGEL(GyroCon);

    for(int i = 0; i < PID_TimeBlocks;){
      Brain.Screen.printAt(10, 20, "i : %5d", i);
      double now_v = (lf1.rotation(rotationUnits::deg) + lf2.rotation(rotationUnits::deg)) / 2;
      if(abs(target_v - now_v) < PID_VSameThreshold)i++;
      else i = 0;
      double mov = MOVE.tune(now_v);
      double ang = ANGEL.tune(GyroCon);
      setChassisVolt(mov, 0, ang);
      this_thread::sleep_for(10);
    }
    ChassisPIDTurn(target_a - GyroCon, 100);
    setChassisVolt(0, 0, 0);
  }
  else if(mode == 'b'){
    Brain.Screen.printAt(10, 20, "i : %5f", lf1.rotation(rotationUnits::deg));
    double target_v = (lf1.rotation(rotationUnits::deg) + lf2.rotation(rotationUnits::deg)) / 2  - dist;
    double target_a = GyroCon;
    PID_move MOVE(target_v);
    PID_turn_none ANGEL(GyroCon);

    for(int i = 0; i < PID_TimeBlocks;){
      Brain.Screen.printAt(10, 20, "i : %5d", i);
      double now_v = (lf1.rotation(rotationUnits::deg) + lf2.rotation(rotationUnits::deg)) / 2;
      if(abs(target_v - now_v) < PID_VSameThreshold)i++;
      else i = 0;
      double mov = MOVE.tune(now_v);
      double ang = ANGEL.tune(GyroCon);
      setChassisVolt(mov, 0, ang);
      this_thread::sleep_for(10);
    }
    ChassisPIDTurn(target_a - GyroCon, 100);
    setChassisVolt(0, 0, 0);
  }
  else if(mode == 'l'){
    Brain.Screen.printAt(10, 20, "i : %5f", lf1.rotation(rotationUnits::deg));
    double target_v = (lf1.rotation(rotationUnits::deg) + lf2.rotation(rotationUnits::deg)) / 2 - dist;
    double target_a = GyroCon;
    PID_move MOVE(target_v);
    PID_turn_none ANGEL(GyroCon);

    for(int i = 0; i < PID_TimeBlocks;){
      Brain.Screen.printAt(10, 20, "i : %5d", i);
      double now_v = (lf1.rotation(rotationUnits::deg) + lf2.rotation(rotationUnits::deg)) / 2;
      if(abs(target_v - now_v) < PID_VSameThreshold)i++;
      else i = 0;
      double mov = MOVE.tune(now_v);
      double ang = ANGEL.tune(GyroCon);
      setChassisVolt(0, mov, ang);
      this_thread::sleep_for(10);
    }
    ChassisPIDTurn(target_a - GyroCon, 100);
    setChassisVolt(0, 0, 0);
  }
  else{
    Brain.Screen.printAt(10, 20, "i : %5f", lf1.rotation(rotationUnits::deg));
    double target_v = (lf1.rotation(rotationUnits::deg) + lf2.rotation(rotationUnits::deg)) / 2 + dist;
    double target_a = GyroCon;
    PID_move MOVE(target_v);
    PID_turn_none ANGEL(GyroCon);

    for(int i = 0; i < PID_TimeBlocks;){
      Brain.Screen.printAt(10, 20, "i : %5d", i);
      double now_v = (lf1.rotation(rotationUnits::deg) + lf2.rotation(rotationUnits::deg)) / 2;
      if(abs(target_v - now_v) < PID_VSameThreshold)i++;
      else i = 0;
      double mov = MOVE.tune(now_v);
      double ang = ANGEL.tune(GyroCon);
      setChassisVolt(0, mov, ang);
      this_thread::sleep_for(10);
    }
    ChassisPIDTurn(target_a - GyroCon, 100);
    setChassisVolt(0, 0, 0);
  }
}

void ChassisPIDTurn(double angel, int maxTime){
  double targetAngle = GyroCon + angel;
  double last_counter = global_counter;
  PID_turn ANGLE(targetAngle);
  for(int i = 0; i < PID_TimeBlocks;){
    if(abs(GyroCon - targetAngle) < PID_ASameThreshold)i++;
    else i = 0;
    //Brain.Screen.printAt(10, 20, "i : %5d", i);
    setChassisVolt(0, 0, ANGLE.tune(GyroCon));
    task::sleep(10);
    if(global_counter - last_counter > maxTime)break;
  }
  setChassisVolt(0, 0, 0);
}

void autoChassisAdjust(char mode){
      if (GyroDifSum > 1 || GyroDifSum < -1){  //update left and right offset to make it run directly
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
        ChassisLFOffset = ChassisLFOffset > 1.2 ? 1.2 : ChassisLFOffset < 0.8 ? 0.8 : ChassisLFOffset;
        ChassisLBOffset = ChassisLBOffset > 1.2 ? 1.2 : ChassisLBOffset < 0.8 ? 0.8 : ChassisLBOffset;
        ChassisRFOffset = ChassisRFOffset > 1.2 ? 1.2 : ChassisRFOffset < 0.8 ? 0.8 : ChassisRFOffset;
        ChassisRBOffset = ChassisRBOffset > 1.2 ? 1.2 : ChassisRBOffset < 0.8 ? 0.8 : ChassisRBOffset;
        GyroDifSum = 0;
        
      }
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