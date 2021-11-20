/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       C:\Users\hp                                               */
/*    Created:      Tue Dec 08 2020                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"
using namespace vex;

void initRobot()
//初始化
{
  l1.resetRotation();
  l2.resetRotation();
  l3.resetRotation();

  r1.resetRotation();
  r2.resetRotation();
  r3.resetRotation();

  u1.resetRotation();
  u2.resetRotation();
  u3.resetRotation();

  eat1.resetRotation();
  eat2.resetRotation();

  SLEEP(10)
}

void initGyro()
{
  gyro_out.set(false);
  //SLEEP(1000)
  gyro_out.set(true);
  vex::this_thread::sleep_for(3000);
}

void vbreak()
{
  l1.stop(vex::brakeType::brake);
  l2.stop(vex::brakeType::brake);
  l3.stop(vex::brakeType::brake);
  
  r1.stop(vex::brakeType::brake);
  r2.stop(vex::brakeType::brake);
  r3.stop(vex::brakeType::brake);
  
}

void vstop(){
    l1.spin(vex::directionType::fwd,0,vex::velocityUnits::rpm);
    l2.spin(vex::directionType::fwd,0,vex::velocityUnits::rpm);
    l3.spin(vex::directionType::fwd,0,vex::velocityUnits::rpm);
    r1.spin(vex::directionType::fwd,0,vex::velocityUnits::rpm);
    r2.spin(vex::directionType::fwd,0,vex::velocityUnits::rpm);
    r3.spin(vex::directionType::fwd,0,vex::velocityUnits::rpm);
    SLEEP(10)
}


///宋
double lim(double a,double t) 
//min{ABS(A),ABS(T)}
{
	a=a>t?t:(a<-t?-t:a);
	return a;
	
}
/////


void vrun(double lv,double rv)
//需要更改端口，这里只是一个示意
//lv、rv均为-100~+100之间
{
  //////宋
  lim(lv,100);
  lim(rv,100);
  ///////
  lv*=120,rv*=120;//最大电压为12000
  VST(4,lv)VST(3,lv)VST(11,lv)
  VST(9,rv)VST(10,rv)VST(13,rv)
}

void vup(double v)
//抬升
{
  v*=120;
  VST(17,v)VST(18,v)VST(19,v)                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         

}
void veat(double v)
//吸球
{
  v*=120;
  VST(1,v)VST(16,v)VST(17,v)

}



/////////////////////////////////////
//==================================
//==========转弯部分===============
//==================================
////////////////////////////////////



double degAdj(double deg){
  if(deg<-180) deg+=360;
  if(deg>180) deg-=360;

  return deg; 

}



void gturn(double aim,int stopTime)
//stopTime是秒
{
  double f=18,kp=0.23,ki=0.000000000000000001,kd=1;
  //double f=18,kp=0.23,ki=0.000000000000000001,kd=1;
  //double f=18,kp=0.23,ki=0.00000000000001,kd=6;//
  double p=GDEG-aim,pp,i=0,v;//初始时i没有被定义
  p=degAdj(p);
  int dir=p<0?1:-1;
  int cnt=0;
  int timeCount=0;
  while(cnt<40)
  {
    if(5*timeCount>1000*stopTime)break;
    if(-dir*p<1) cnt++;
    else cnt=0;
    pp=p;
    //p=degAdj(p);
    //Brain.Screen.printAt(1,40,"deg to go:%f",p);
    Brain.Screen.printAt(1,80,"-dir*p:%f",-dir*p);
    Brain.Screen.printAt(1,40,"time:%d",timeCount);
    p=GDEG-aim;
    p=degAdj(p);
    //if(p*dir>0) i=0;//过零就把i清零//初始时没有被注释
    dir=p<0?1:-1;
    i+=p;
    v=-(kp*p+ki*i+kd*(p-pp))+f*dir;
    vrun(v,-v);
    SLEEP(5)
    timeCount++;
  }

  vstop();
}





/*
double getGyroValue()
//0-360
{
  double val = ((gyro_in.value(vex::analogUnits::range12bit) * 5.0 / 4096.0) - 0.4) * 1000;//0-3600
  //return val;
  if(val < 0) 
  {
    return -1;
  }
  else if(val < 3600) 
  {
    return val/10;
  }
  else if(val == 3600)
  {
    return 0;
  }
	else
  {
    return -1;
  }
}

double getRelativeAngle(int val, int refer)
{
	val -= refer;

	while(val >   180) val -= 360;
	while(val <= -180) val += 360;

	return val;
}


void turnAngle1(double angleVal)
//
{
  //angleVal CCW -179~180//lim of turnAngle

  const double EPS = 1;
  
  const double Kp =  0.05;//
  const double Ki = 0*0.00005;     //0.00005
  const double Kd = 0*220;
  const double f = 21;//新电池满电摩擦力20-21之间

  double bgnVal = 0;//deg at the beginning
  double endVal = 0;//deg at terminal
  double curVal = 0;//current angle left//to control spd and EPS
 
  double cc=0,i=0;
  int dir=curVal<0?1:-1;

  double v = 0.0;//spd
  bgnVal = getGyroValue();
  endVal = bgnVal + angleVal;

	while(endVal >= 360) endVal -= 360;
	while(endVal <     0) endVal += 360;

  curVal = getRelativeAngle(getGyroValue(), endVal);//current angle left//to control spd and EPS
  Brain.Screen.printAt(1,40,"deg to go:%f",cc);
  
  while(-dir*curVal>EPS)
  {
    cc=curVal;
    Brain.Screen.printAt(1,40,"deg to go:%f",cc);
    curVal=getRelativeAngle(getGyroValue(), endVal);
    if(curVal*dir>0) i=0;
    dir=curVal<0?1:-1;
    i+=curVal;
    v=-(Kp*curVal+Ki*i+Kd*(curVal-cc))+f*dir;
    vrun(v,-v);
 
    SLEEP(5);//5
  }

  vrun(0, 0);//加电压为0，停止//不清楚惯性影响有多大，是否需要vbreak()?
}



void turnToTarget(int targetAngle)
{
  //1800 ~ 0 ~ -1799 , CCW

  int curAngle = getGyroValue() - 1800;

  int relativeAngle = getRelativeAngle(targetAngle, curAngle);

  turnAngle1(relativeAngle);
}

*/
int main() {
  initGyro();
  while(1)
  { 
    if(BP(Left))
    {
      gturn(90,6);
    }
    if(BP(Right))
    {
      gturn(-90,6);
 
    }
    if(BP(Up))
    {
      gturn(0,6);
    }
    if(BP(Down))
    {
      gturn(180,6);
 
    }
    if(BP(A))
    {
      gturn(45,6);

    }
    if(BP(B))
    {
      gturn(-45,6);

    }
    
  SLEEP(10);
  }

    
    
}
