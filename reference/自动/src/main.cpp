//----------------------------------------------------------------------------
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

#define MAXV 12000;
int vfir=100;
void vclear()

{
    l1.resetRotation();
    l2.resetRotation();
    l3.resetRotation();
    l4.resetRotation();
    
    r1.resetRotation();
    r2.resetRotation();
    r3.resetRotation();
    r4.resetRotation();

    SLEEP(200)
 
}
void initGyro()
//陀螺仪初始化，需要断电2秒再上电才能是陀螺仪回复初始状态
{
  gyro_out.set(false);
  SLEEP(2000)
  gyro_out.set(true);
  SLEEP(200)
}
int getGyroValue()
//获取陀螺仪角度值
{
  
	return ((gyro_in.value(vex::analogUnits::range12bit) * 5.0 / 4096.0) - 0.4) * 1000;
}

int getRelativeAngle(int val, int refer)
//val为当前角度值，refer为需要转到的角度
//该函数主要计算出机器人需要往顺时针/逆时针转多少角度
{
  
	val -= refer;

	while(val >  1800) val -= 3600;
	while(val < -1800) val += 3600;

	return val;
}

double lim(double a,double t) 
//min{ABS(A),ABS(T)}
{
	a=a>t?t:(a<-t?-t:a);
	return a;
	
}
void vrun(double lv,double rv)
//Vrun
{
    lv*=120,rv*=120;
    VST(11,lv)VST(12,lv)VST(13,lv)VST(14,lv)
    VST(16,rv)VST(18,rv)VST(19,rv)VST(20,rv)
}
void vbreak()
{
  l1.stop(vex::brakeType::brake);
  l2.stop(vex::brakeType::brake);
  l3.stop(vex::brakeType::brake);
  l4.stop(vex::brakeType::brake);
  r1.stop(vex::brakeType::brake);
  r2.stop(vex::brakeType::brake);
  r3.stop(vex::brakeType::brake);
  r4.stop(vex::brakeType::brake);
}

double encoderValue(int ch)
//get encoder value
//-1 is left 
//1 is right
//0 or othoer is both
{
	double x;
	if (ch==-1)
	{
		x=(l1.rotation(vex::rotationUnits::deg)+l2.rotation(vex::rotationUnits::deg)+l3.rotation(vex::rotationUnits::deg)+l4.rotation(vex::rotationUnits::deg))/4;
	}
	else if (ch==1)
	{
		x=(r1.rotation(vex::rotationUnits::deg)+r2.rotation(vex::rotationUnits::deg)+r3.rotation(vex::rotationUnits::deg)+r4.rotation(vex::rotationUnits::deg))/4;
	}
	else 
	    x=(l1.rotation(vex::rotationUnits::deg)+l2.rotation(vex::rotationUnits::deg)+l3.rotation(vex::rotationUnits::deg)+l4.rotation(vex::rotationUnits::deg)+r1.rotation(vex::rotationUnits::deg)+r2.rotation(vex::rotationUnits::deg)+r3.rotation(vex::rotationUnits::deg)+r4.rotation(vex::rotationUnits::deg))/8;
	return x;
}
double calSpd(double delta)
//计算底盘左右两侧电机编码器数值只差，即算出走的距离只差
//如果差太多就给另一侧补速度，以纠正偏离
//适用于非外力作用下小范围纠偏
//原理简单，非常好用！之前拆的差不多的破车都能走完整个场地误差只有3cm左右，角度偏差极小
//cal spd that need to be added
//k need to adjust
{
	delta=fabs(delta);
  double v;
	double k;
  double cm=delta/AD(1);//将编码器数值换算成直观的厘米数
  
  if(delta<AD(1)) 
  k=0.35;///参数要自己调的
  else if(delta>=AD(1)) 
  k=0.35+0.25*cm;///同上一行

	v=k*AD(delta);
	return v;
}

void gturn(double aim)
{
  double f=12,ki=0.001,kp=0.325,kd=220;
  double p=GDEG-aim,pp,i,v;
  int dir=p<0?1:-1;
  while(-dir*p>0.5)
  {
    pp=p;
    p=GDEG-aim;
    if(p*dir>0) i=0;
    dir=p<0?1:-1;
    i+=p;
    v=-(kp*p+ki*i+kd*(p-pp))+f*dir;
    vrun(v,-v);
    SLEEP(5)
  }
}
void vturn(double deg,double v,int k)//k等于1右转
{ 
  deg=2.15*deg;
  vclear();
  double x=encoderValue(1);
  while(true)
  {
    x=encoderValue(1);
    if(fabs(x)<0.3*deg)
    vrun(0.7*k*v,0.7*(-k)*v);
    else if(fabs(x)<0.85*deg)
    vrun(k*v*0.9,(-k)*v*0.9);
    else if(fabs(x)<deg)
      vrun(k*0.6*v,0.6*(-k)*v);
    else
            vbreak();//换了个停止方式，也挺好用
        if(fabs(x)>deg)
        {
            vbreak();
            break;
        }
      
  }
}
       
void vdis(double cm,double v,int k)
{
    vclear();
    double x=encoderValue(0),y=k*v,le,re,l=0,r=0,delta;
    double cv;
    double m=0.8;//m为以最大速度行进的比例，30%~m的比例为最大速度行进，也就是形参v速度前进
    if(cm>=25) m=0.8;//距离长就大一点
    else if(cm<25) m=0.60;//短就小点
    while(1)
    {
    	  le=fabs(encoderValue(-1));
    	  re=fabs(encoderValue(1));
    	  delta=le-re;
    	  if(delta>0)
    	  {
    		    l=0;
    		    r=k*calSpd(delta);//这里的k是前进（+1）后退（-1）
		    }
		    else if(delta<0)
    	  {
    		    r=0;
    		    l=k*calSpd(delta);
		    }
		    else 
		    {
			      l=0;r=0;
		    }
      ////////////////////////////
        x=encoderValue(0);
        if(fabs(x)<0.3*AD(cm))//前30%缓慢加速
        {
          if(cm>=25)
            cv=k*19+0.15*x;//if v=60 then spdup=25cm
          else if(cm<25)
            cv=k*21+0.42*x;//if v=60 then spdup=8.6cm
          cv=lim(cv,v);
          vrun(cv+l,cv+r);
        }
        else if(fabs(x)<m*AD(cm))
        {
            vrun(y+l,y+r);
        }
        else if(fabs(x)<AD(cm))
        {
            cv=fabs(AD(cm)-fabs(x))*y/((1-m)*fabs(AD(cm)));//cv=fabs(AD(cm)-fabs(x))*y/fabs(AD(cm));//快速减小电压来减速
            cv=lim(cv,v);
            if (fabs(cv)<=17)//parametre may need changing
            {
                if (cv<0) cv=-17;//需要测出机器人能够前进的最小电压，然后比这个稍微大一点，以确保能够前进
                else if(cv>0) cv=17;
                else {vbreak();break;}//函数写得不完善，但至今没出BUG
            }
            vrun(cv+l,cv+r);
        }
        else
            vbreak();//换了个停止方式，也挺好用
        if(fabs(x)>=AD(cm))
        {
            vbreak();
            break;
        }
      
    }
}
void veat(double v)//吸取球
{
     v*=120;
     VST(1,v);
     VST(10,v);
     VST(4,v);
     VST(9,v);
}
// MOTOR(lpaw,1,true)MOTOR(frol,4,true)MOTOR(lfly,5,true)
// MOTOR(rfly,6,false)MOTOR(brol,9,true)MOTOR(rpaw,10,false)
double val_v = 50;
void vfire(double v)
{
  VVST(5,6,v);
  VVST(4,9,v);
}

void cto1_1()//投球
{
  vturn(105,50,1); 
  vdis(45,50,1);
  vfire(vfir);
  SLEEP(500);
  vfire(0);
  vdis(45,50,-1);
  vturn(130,50,1);   
}

void cto1_2()//清空桩的操作
{
  vturn(90,50,-1); 
  vdis(65,50,1);
  vturn(93,50,-1); 
  vdis(25,50,1); 
  vrun(70,70);
  SLEEP(100);
  vrun(0,0);
  veat(100);//吸取球
  SLEEP(3000);
  veat(0);
  vdis(31,val_v,-1);
  SLEEP(500);
  vturn(90,50,1); 
  vdis(65,50,-1);
  vturn(90,50,1);   
}

void cto1_3()//清空桩并投一个球
{
  vturn(90,50,-1); 
  vdis(65,50,1);
  vturn(93,50,-1); 
  vdis(25,50,1); 
  vrun(70,70);
  SLEEP(100);
  vrun(0,0);
  veat(100);//吸取球
  SLEEP(3000);
  veat(0);
  vdis(10,val_v,-1);
  vfire(vfir);
  SLEEP(1000);
  vfire(0);
  vdis(21,val_v,-1);
  SLEEP(500);
  vturn(90,50,1); 
  vdis(65,50,-1);
  vturn(90,50,1);   
}


void cto2_1()//完成吸下面的球然后吐一个球进去的操作
{
  vturn(122,val_v,1); 
  veat(60);
  vdis(40,val_v,1);  
  SLEEP(1000);//看看情况
  veat(0);
  vdis(10,val_v,-1);

  vfire(vfir);
  SLEEP(2000);
  vfire(0);

  vdis(30,val_v,-1);
  SLEEP(300);
  vturn(124,val_v,-1); 
}

void cto2_2()//完成吸下面的球然后清空桩的操作
{
  vturn(122,val_v,1); 
  veat(60);
  vdis(48,val_v,1);  
  SLEEP(1000);//看看情况 
  veat(0);
  vdis(3,val_v,1);

  veat(100);
  SLEEP(2000);
  veat(0);

  vdis(51,val_v,-1);
  SLEEP(300);
  vturn(124,val_v,-1); 
}

void cto2_3()
{
  vturn(128,val_v,1); //吸地上的球
  veat(60);
  vdis(40,val_v,1);  
  SLEEP(1000);//看看情况
  veat(0);

  // vdis(10,val_v,-1);
  vfire(100);//吐一个球
  SLEEP(500);//时间有要求
  vfire(0);


  // vdis(10,val_v,1);
  veat(100);//吸两个球
  vrun(50,50);
  SLEEP(300);
  vrun(0,0);  
  SLEEP(500);//时间有要求
  vrun(50,50);
  SLEEP(1500);//时间有要求
  vrun(0,0);  

  vdis(55,val_v,-1);//回到原位
  veat(0);
  SLEEP(300);
  // vturn(127,val_v,-1); 
}

void cto3_1()
{
  
}

void first_1(int deg)
{
  vdis(47,50,deg*1);
}
int main () 
{
    gyron.set(true);
//*******这里是第一步操作*******//
    first_1(1);
//***********end*************//
    SLEEP(300);
    cto2_3();
    SLEEP(300);
    cto1_1();//投球
    return 0;
}
   
