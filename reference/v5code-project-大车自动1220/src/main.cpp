#include "vex.h"

#define MAXV 12000;
int vfir=100;//不用它
// MOTOR(lf1,2,true)MOTOR(lm0,3,false)MOTOR(lb1,4,true)MOTOR(lup,11,true)
// MOTOR(rf0,7,false)MOTOR(rm1,8,true)MOTOR(rb0,9,false)MOTOR(rup,12,false)
// MOTOR(fpawl,1,true)MOTOR(fpawr,6,false)MOTOR(bpaw,10,true)
// MOTOR(fd,16,false)MOTOR(fm,15,false)MOTOR(fu,14,false)
// MOTOR(sl,18,false)
void vclear()
{
    lf1.resetRotation();
    lm0.resetRotation();
    lb1.resetRotation();
    lup.resetRotation();
    
    rf0.resetRotation();
    rm1.resetRotation();
    rb0.resetRotation();
    rup.resetRotation();

    SLEEP(200)
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
    VST(2,lv)VST(3,lv)VST(4,lv)VST(11,lv)
    VST(7,rv)VST(8,rv)VST(9,rv)VST(12,rv)
}
void vbreak()
{
  lf1.stop(vex::brakeType::brake);
  lm0.stop(vex::brakeType::brake);
  lb1.stop(vex::brakeType::brake);
  lup.stop(vex::brakeType::brake);
  rf0.stop(vex::brakeType::brake);
  rm1.stop(vex::brakeType::brake);
  rb0.stop(vex::brakeType::brake);
  rup.stop(vex::brakeType::brake);
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
		x=(lf1.rotation(vex::rotationUnits::deg)+lm0.rotation(vex::rotationUnits::deg)+lb1.rotation(vex::rotationUnits::deg)+lup.rotation(vex::rotationUnits::deg))/4;
	}
	else if (ch==1)
	{
		x=(rf0.rotation(vex::rotationUnits::deg)+rm1.rotation(vex::rotationUnits::deg)+rb0.rotation(vex::rotationUnits::deg)+rup.rotation(vex::rotationUnits::deg))/4;
	}
	else 
	    x=(lf1.rotation(vex::rotationUnits::deg)+lm0.rotation(vex::rotationUnits::deg)+lb1.rotation(vex::rotationUnits::deg)+lup.rotation(vex::rotationUnits::deg)+rf0.rotation(vex::rotationUnits::deg)+rm1.rotation(vex::rotationUnits::deg)+rb0.rotation(vex::rotationUnits::deg)+rup.rotation(vex::rotationUnits::deg))/8;
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
  double f=15.9,ki=0.000000011,kp=0.65,kd=250;
  double p=GDEG-aim,pp,i,v;
  int dir=p<0?1:-1;
  int count1 = 0;
  while(count1<50)
  {
    if(-dir*p<0.5)
    {
      count1++;
    }
    else
      count1 = 0;
    pp=p;
    Brain.Screen.printAt(10,140,"deg : %5f",GDEG);
    p=GDEG-aim;
    if(p<-180)
      p+=360;
    if(p>180)
      p-=360;
    if(p*dir>0) i=0;
    dir=p<0?1:-1;
    if(fabs(p)<=30)
      i+=p;    
    v=-(kp*p+ki*i+kd*(p-pp)) + f*dir;
    vrun(v,-v);
    SLEEP(3)
  }
  vrun(0,0);
}
void vturn(double deg,double v,int k)//k等于1右转
{ 
  deg=1.98*deg;
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
     VST(6,v);
     VST(10,v);
     VST(16,v);
}
void vfire(double v)
{
  VVST(16,15,v)
  VVST(14,10,v)
}
// MOTOR(lf1,2,true)MOTOR(lm0,3,false)MOTOR(lb1,4,true)MOTOR(lup,11,true)
// MOTOR(rf0,7,false)MOTOR(rm1,8,true)MOTOR(rb0,9,false)MOTOR(rup,12,false)
// MOTOR(fpawl,1,true)MOTOR(fpawr,6,false)MOTOR(bpaw,10,true)
// MOTOR(fd,16,false)MOTOR(fm,15,false)MOTOR(fu,14,false)
// MOTOR(sl,18,false)

void first_getb()
{
  veat(80);
  vdis(52,40,1);
  SLEEP(200);//吸球   
  veat(0); 
}


int main () 
{
    //gyron.set(false);
    gyron.set(true);
    SLEEP(500)
    //
    first_getb();
    SLEEP(100)
    vdis(22,40,-1);//后退
    SLEEP(200)
    gturn(-135);
    //
    veat(60);
    vdis(35,40,1);
    SLEEP(100)
    vrun(30,30);
    SLEEP(500)
    veat(0);
    vrun(0,0);
    vfire(100);
    SLEEP(500);
    vfire(0);
    //
    vdis(28,40,-1);
    gturn(135);
    vdis(28,40,1);
    vrun(30,30);
    veat(60);
    SLEEP(500)
    veat(0);
    vrun(0,0);
    vfire(100);
    SLEEP(500);
    vfire(0);
    //
    vdis(20,40,-1);
    vturn(45,40,-1);
    vdis(15,40,10);
    // vdis(20,40,1);
    // SLEEP(1000)
    // gturn(90);
    // SLEEP(1000)
    // gturn(0);
    // vdis(20,40,-1);
    // vrun(50,-50);
    return 0;
}
   
