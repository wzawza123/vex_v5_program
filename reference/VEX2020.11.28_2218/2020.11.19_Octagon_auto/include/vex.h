/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       vex.h                                                     */
/*    Author:       Vex Robotics                                              */
/*    Created:      1 Feb 2019                                                */
/*    Description:  Default header for V5 projects                            */
/*                                                                            */
/*----------------------------------------------------------------------------*/
//
//#define PORT3 PORT14
//#define PORT11 PORT16

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "robot-config.h"
#include "v5.h"
#include "v5_vcs.h"

#define party 'r'

//chassis motor variables
double ChassisLFOffset = ChassisLFOffsetInitial;
double ChassisLBOffset = ChassisLBOffsetInitial;
double ChassisRFOffset = ChassisRFOffsetInitial;
double ChassisRBOffset = ChassisRBOffsetInitial;

double MasterAxisAngel = 0;
//gyro statues
double GyroLast = 0;
double GyroCon = 0;
double GyroDif = 0;
double GyroDifSum = 0;

//vision variables
double ballColor = 0.0;

// A global instance of vex::brain used for printing to the V5 brain screen
vex::brain Brain = vex::brain();

// define your global instances of motors and other devices here
vex::controller Controller = vex::controller();

vex::motor lf1 = vex::motor(vex::PORT7, true);   //左前轮1
vex::motor lf2 = vex::motor(vex::PORT8, true);   //左前轮2
vex::motor lb1 = vex::motor(vex::PORT9, true);   //左后轮1
vex::motor lb2 = vex::motor(vex::PORT10, true);  //左后轮2

vex::motor rf1 = vex::motor(vex::PORT17, false);   //右前轮1
vex::motor rf2 = vex::motor(vex::PORT18, false);   //右前轮1
vex::motor rb1 = vex::motor(vex::PORT19, false);   //右后轮1
vex::motor rb2 = vex::motor(vex::PORT20, false);   //右后轮1

vex::motor lz = vex::motor(vex::PORT1, false);     //左爪
vex::motor rz = vex::motor(vex::PORT2, true);    //右爪
//zffzzf
vex::motor ful = vex::motor(vex::PORT3, true);   //前上方左电机
vex::motor fur = vex::motor(vex::PORT4, false);  //前上方右电机
vex::motor fm = vex::motor(vex::PORT5, false);   //前中方电机
vex::motor fd = vex::motor(vex::PORT6, true);    //前下方电机
vex::motor bu = vex::motor(vex::PORT11, false);   //后上方电机
vex::motor bd = vex::motor(vex::PORT12, true);  //后下方电机


vex::digital_out gyro_out = vex::digital_out(Brain.ThreeWirePort.A);    //陀螺仪供电口
vex::analog_in gyro_in = vex::analog_in(Brain.ThreeWirePort.B);     //336 - 3272 = 2936     /360 = 8.1556

vex::timer T = vex::timer();

#if party == 'r'
vex::vision::signature SIG_1 (1, 8669, 10751, 9710, -3199, -2239, -2718, 3, 0);
vex::vision::signature SIG_2 (2, -2579, -1215, -1896, 1935, 4551, 3242, 3, 0);
//vex::vision::signature SIG_1 (1, 3327, 7135, 5231, -841, 1, -420, 2, 0);
//vex::vision::signature SIG_2 (2, -2979, -1909, -2444, 4781, 10003, 7392, 1.5, 0);
#else
vex::vision::signature SIG_1 (2, -2979, -1909, -2444, 4781, 10003, 7392, 1.5, 0);
vex::vision::signature SIG_2 (1, 3327, 7135, 5231, -841, 1, -420, 2, 0);
#endif
vex::vision::signature SIG_3 (3, 0, 0, 0, 0, 0, 0, 3, 0);
vex::vision::signature SIG_4 (4, 0, 0, 0, 0, 0, 0, 3, 0);
vex::vision::signature SIG_5 (5, 0, 0, 0, 0, 0, 0, 3, 0);
vex::vision::signature SIG_6 (6, 0, 0, 0, 0, 0, 0, 3, 0);
vex::vision::signature SIG_7 (7, 0, 0, 0, 0, 0, 0, 3, 0);
vex::vision Vision (vex::PORT15, 50, SIG_1, SIG_2, SIG_3, SIG_4, SIG_5, SIG_6, SIG_7);    //摄像头

//declear functions
void initRobot();

void initGyro();
int getSensor();
int operationMain();


void debug_display();

void ChassisMove(char mode, double dist);
void ChassisTurn(double angel);
void ChassisPIDMove(char mode, double dist);
void ChassisPIDTurn(double angel, int maxTime = PID_TimeMax);

void autoChassisAdjust(char mode);

void setChassisVolt(double f, double r, double rt);
void setClawVolt(char c);
void setTranspoterVolt(char c);
void setThrowVolt(char c);

#define max(a,b) (a>b?a:b)
#define abs(a) (a>0?a:-a)

class PID_turn{
  double target;
  double err_last;           //定义上一个偏差值
  double integral;           //定义积分值
  public:
  PID_turn(double set_target){
    target = set_target;
    err_last=0.0;
    integral=0.0;
  }
  double tune(double set_now){
    double index;
    double err = target - set_now;
    if(abs(err)>30) index = 0.0;
    else if(abs(err)<10) index = 1.0;
    else index=(30-abs(err))/20;
    integral += err * index;
    if(integral * err < 0) integral = 0;
    double action = PID_turn_Kp * err + PID_turn_Ki * integral + PID_turn_Kd * (err-err_last);
    err_last=err;

    if(err > 0) action += PID_turn_r_thres;
    else action -= PID_turn_r_thres;

    action = action > 0.7 ? 0.7 : action < -0.7 ? -0.7 : action;
    return action;
  }
};

class PID_move{
  double target;
  double err_last;           //定义上一个偏差值
  double integral;           //定义积分值
  public:
  PID_move(double set_target){
    target = set_target;
    err_last=0.0;
    integral=0.0;
  }
  double tune(double set_now){
    double index;
    double err = target - set_now;
    if(abs(err) > 300) index = 0.0;
    else if(abs(err) < 100) index = 1.0;
    else index = (300 - abs(err)) / 200;
    integral += err * index;
    if(integral * err < 0) integral = 0;
    double action = PID_move_Kp * err + PID_move_Ki * integral + PID_move_Kd * (err-err_last);
    err_last=err;

    if(err > 0) action += PID_move_f_thres;
    else action -= PID_move_f_thres;
    
    action = action > 0.6 ? 0.6 : action < -0.6 ? -0.6  : action;
    return action;
  }
};

class PID_turn_none{
  double target;
  double err_last;           //定义上一个偏差值
  double integral;           //定义积分值
  public:
  PID_turn_none(double set_target){
    target = set_target;
    err_last=0.0;
    integral=0.0;
  }
  double tune(double set_now){
    double index;
    double err = target - set_now;
    if(abs(err) > 200) index = 0.0;
    else if(abs(err) < 100) index = 1.0;
    else index = (200 - abs(err)) / 100;
    integral += err * index;
    if(integral * err < 0) integral = 0;
    double action = PID_move_Kp * err + PID_move_Ki * integral + PID_move_Kd * (err-err_last);
    err_last=err;
    
    action = action > 1 ? 1 : action < -1 ? -1 : action;
    return action;
  }
};
/*if(mode == 't'){
      if(action > 0) action = PID_turn_r_thres + action * (1 - PID_turn_r_thres);
      else action = - PID_turn_r_thres + action * (1 - PID_turn_r_thres);
    }
    else if(mode == 'f'){
      if(action > 0) action = PID_move_f_thres + action * (1 - PID_move_f_thres);
      else action = - PID_move_b_thres + action * (1 - PID_move_b_thres);
    }
    else if(mode == 'l'){
      if(action > 0) action = PID_move_r_thres + action * (1 - PID_move_r_thres);
      else action = - PID_move_l_thres + action * (1 - PID_move_l_thres);
    }
    else if(mode == 'n'){
    }*/