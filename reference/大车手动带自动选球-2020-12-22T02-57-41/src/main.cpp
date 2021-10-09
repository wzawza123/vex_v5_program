/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       C:\Users\DELL                                             */
/*    Created:      Fri Nov 06 2020                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"


int getGyroValue()
//获取陀螺仪角度值
{
  
	return ((gyro_in.value(vex::analogUnits::range12bit) * 5.0 / 4096.0) - 0.4) * 1000;
}


void SETZERO()
{
    VVST(11,1,0)
    VVST(10,9,0)
    VST(8,0)
    VST(6,0)
}



void VRUN(double l,double r)
{
    VVST(12,13,l)
    VVST(15,16,l)
    VVST(2,3,r)
    VVST(4,5,r)
}

void Get_All(double v)
{
  VST(11,v*120);     
  VST(1,v*120);     
  VVST(10,9,v)
  VVST(6,8,v)

}


void GetBallFront(double v)
{
    double V= v*120;
    VST(1,V);
    VST(11,V);
    VST(9,V);     
}
void GetBallBehind(double v)
{
  VVST(9,10,v)
}
void GetBall(double v)
{
  VST(6,120*v);     
  VST(8,0);     
  VVST(1,11,v)
  VVST(9,10,v)
}
void OutBall_paw(double v)
{
  VST(6,0);     
  VST(8,0);     
  VVST(9,10,-v)
  VVST(11,1,-v)
}
// MOTOR(lf1,2,true)MOTOR(lm0,3,false)MOTOR(lb1,4,true)MOTOR(lup,17,true)
// MOTOR(rf0,7,false)MOTOR(rm1,8,true)MOTOR(rb0,9,false)MOTOR(rup,19,false)
// MOTOR(fpawl,1,true)MOTOR(fpawr,6,false)MOTOR(bpaw,10,true)
// MOTOR(fd,16,false)MOTOR(fm,15,false)MOTOR(fu,14,false)
// MOTOR(sl,18,false)

void OutBall_F(int v)
{
    double V =v*120;
    VST(1,0);
    VST(11,0);
    VST(9,V);  
    VST(8,V);
    VST(6,V);     
    VST(10,V);
}
void Select(double deg)
{
    double x,lastx,startx,stage;
    stage=0;
    x=sl.rotation(vex::rotationUnits::deg);
    startx=x;
    lastx = x+deg;
    ROTS(sl,deg,80)
    while(fabs(lastx-x)>5)
    {
      x=sl.rotation(vex::rotationUnits::deg);
      SLEEP(300)
    }
    VST(18,0)
}

int selectColor()
{
  Vision.takeSnapshot(1,80);
  if(Vision.largestObject.height>80&&Vision.largestObject.width>80){
    return(1);
  }
  Vision.takeSnapshot(2,80);
  if(Vision.largestObject.height>80&&Vision.largestObject.width>80){
    return(2);
  }
  else{
    return(0);
  }
}

int main() 
{
    while(true)
    {
        Brain.Screen.printAt(10,140,"%d",CCV);
        if(std::abs(FAV(3))!=0 || std::abs(FAV(1))!=0)
            VRUN(FAV(3)+0.8*FAV(1),FAV(3)-0.8*FAV(1));
        else
            VRUN(0,0);            
        if(BP(R2)&&!BP(L2))
        {           
            GetBall(100);
        }
        else if(!BP(R2)&&BP(L2))
        {
            OutBall_F(100);        
        }
        else if(BP(R2)&&BP(L2))
        {
            Get_All(100);
        }
        else if(BP(R1))
        {
            OutBall_paw(100);
        }
        else
        {
          SETZERO();            
        }

        if(selectColor()==2)
        {
          ROTS(sl,58,100) 
          SLEEP(500);
          Brain.Screen.printAt(10,140,"%d",selectColor());
        }
        else
        {
          ROTS(sl,0,80) 
          Brain.Screen.printAt(10,140,"%d",selectColor());
        }

        SLEEP(7);


    }
    return 0;
}

    
    
