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

  SLEEP(100)
}

void initGyro()
{
  gyro_out.set(false);
  gyro_out.set(true);
  vex::this_thread::sleep_for(1000);
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
    SLEEP(50)
}

void vrun(double lv,double rv)
//需要更改端口，这里只是一个示意
//lv、rv均为-100~+100之间
{
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

void veatStop(){
  VST(1,0)VST(16,0)
}

double lim(double a,double t) 
{
	a=a>t?t:(a<-t?-t:a);
	return a;
	
}

double min(double a,double t)//t>0
{
    if(fabs(a)<fabs(t))
    {
      if (a>0) a=t;
      else if (a<0) a=-t;
      else a=0;
    }
    return a;

}

void vclear()
{
    l1.resetRotation();
    l2.resetRotation();
    l3.resetRotation();
    
    
    r1.resetRotation();
    r2.resetRotation();
    r3.resetRotation();


    SLEEP(200)
}
/////////////////////////////////////
//==================================
//==========走直线部分===============
//==================================
////////////////////////////////////
//工具函数
double encoderValue(int ch)
//get encoder value
//-1 is left 
//1 is right
//0 or othoer is both
{
	double x;
	if (ch==-1)
	{
		x=(l1.rotation(vex::rotationUnits::deg)+l2.rotation(vex::rotationUnits::deg)+l3.rotation(vex::rotationUnits::deg))/4;
	}
	else if (ch==1)
	{
		x=(r1.rotation(vex::rotationUnits::deg)+r2.rotation(vex::rotationUnits::deg)+r3.rotation(vex::rotationUnits::deg))/4;
	}
	else 
	    x=(l1.rotation(vex::rotationUnits::deg)+l2.rotation(vex::rotationUnits::deg)+l3.rotation(vex::rotationUnits::deg)+r1.rotation(vex::rotationUnits::deg)+r2.rotation(vex::rotationUnits::deg)+r3.rotation(vex::rotationUnits::deg))/8;
	return x;
}

//包括：calSpd(计算编码器数值差) vids （走长直线，加速-匀速-减速）speed(加速)、speedconstant（匀速）speed cut(减速) 
/*
费解的部分：

*/
double calSpd(double delta,double v_)
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
  if(v_<50)
  {
  if(delta<AD(1)) 
  k=0.7;///参数要自己调的
  else if(delta>=AD(1)) 
  k=0.7+0.5*cm;///同上一行
  }
  else
  {
    if(delta<AD(1)) 
  k=1.50;///参数要自己调的
  else if(delta>=AD(1)) 
  k=1.50+3.85*cm;///同上一行
  }
	v=k*AD(delta);
	return v;
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
    		    r=k*calSpd(delta,v);//这里的k是前进（+1）后退（-1）
		    }
		    else if(delta<0)
    	  {
    		    r=0;
    		    l=k*calSpd(delta,v);
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
            cv=k*19+0.45*x;//if v=60 then spdup=25cm
          else if(cm<25)
            cv=k*21+0.62*x;//if v=60 then spdup=8.6cm
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
            if (fabs(cv)<=18)//parametre may need changing
            {
                if (cv<0) cv=-18;//需要测出机器人能够前进的最小电压，然后比这个稍微大一点，以确保能够前进
                else if(cv>0) cv=18;
                else {vstop();break;}//函数写得不完善，但至今没出BUG
            }
            vrun(cv+l,cv+r);
        }
        else
            vstop();//换了个停止方式，也挺好用
        if(fabs(x)>=AD(cm))
        {
            vstop();
            break;
        }     
    }
}
//====================加速========================
void speed(double cm,double v,int k)
{
vclear();
    double x=encoderValue(0),y=k*v,le,re,l=0,r=0,delta;
    double cv;
     double m=0.8;//m为以最大速度行进的比例，30%~m的比例为最大速度行进，也就是形参v速度前进
       while(1)
    {
      //=====================================================================直线修正
    	  le=fabs(encoderValue(-1));
    	  re=fabs(encoderValue(1));
    	  delta=le-re;
    	  if(delta>0)
    	  {
    		    l=0;
    		    r=k*calSpd(delta,v);//这里的k是前进（+1）后退（-1）
		    }
		    else if(delta<0)
    	  {
    		    r=0;
    		    l=k*calSpd(delta,v);
		    }
		    else 
		    {
			      l=0;r=0;
		    }
      //======================================================================直线修正
        x=encoderValue(0);
       //加速        
          if(cm>=25)
            cv=k*19+0.15*x;//if v=60 then spdup=25cm
          else if(cm<25)
            cv=k*21+0.42*x;//if v=60 then spdup=8.6cm
          cv=lim(cv,v);
          vrun(cv+l,cv+r);        
    
     if(fabs(x)>=AD(cm))
        {
            vstop();
            break;
        }  
    }   
}
//====================匀速========================
void speedconstant(double cm,double v,int k)
{
vclear();
    double x=encoderValue(0),y=k*v,le,re,l=0,r=0,delta;
       while(1)
    {
      //=====================================================================直线修正
    	  le=fabs(encoderValue(-1));
    	  re=fabs(encoderValue(1));
    	  delta=le-re;
    	  if(delta>0)
    	  {
    		    l=0;
    		    r=k*calSpd(delta,v);//这里的k是前进（+1）后退（-1）
		    }
		    else if(delta<0)
    	  {
    		    r=0;
    		    l=k*calSpd(delta,v);
		    }
		    else 
		    {
			      l=0;r=0;
		    }
      //======================================================================直线修正
        x=encoderValue(0);
       //匀速
        vrun(y+l,y+r);
        if(fabs(x)>=AD(cm))
        {
            vstop();
            break;
        }     
    }        
}
//====================减速========================
void speedcut(double cm,double v,int k)
{
vclear();
    double x=encoderValue(0),y=k*v,le,re,l=0,r=0,delta;
    double cv;
     double m=0.8;//m为以最大速度行进的比例，30%~m的比例为最大速度行进，也就是形参v速度前进
       while(1)
    {
      //=====================================================================直线修正
    	  le=fabs(encoderValue(-1));
    	  re=fabs(encoderValue(1));
    	  delta=le-re;
    	  if(delta>0)
    	  {
    		    l=0;
    		    r=k*calSpd(delta,v);//这里的k是前进（+1）后退（-1）
		    }
		    else if(delta<0)
    	  {
    		    r=0;
    		    l=k*calSpd(delta,v);
		    }
		    else 
		    {
			      l=0;r=0;
		    }
      //======================================================================直线修正
        x=encoderValue(0);
       //减速
         cv=fabs(AD(cm)-fabs(x))*y/((1-m)*fabs(AD(cm)));//cv=fabs(AD(cm)-fabs(x))*y/fabs(AD(cm));//快速减小电压来减速
            cv=lim(cv,v);
            if (fabs(cv)<=18)//parametre may need changing
            {
                if (cv<0) cv=-18;//需要测出机器人能够前进的最小电压，然后比这个稍微大一点，以确保能够前进
                else if(cv>0) cv=18;
                else {vstop();break;}//函数写得不完善，但至今没出BUG
            }
            vrun(cv+l,cv+r);
            if(fabs(x)>=AD(cm))
        {
            vstop();
            break;
        }     
    }
}



/////////////////////////////////////
//==================================
//==========走直线结束===============
//==================================
////////////////////////////////////


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
    Brain.Screen.printAt(1,40,"deg to go:%f",p);
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













/////////////////////////////////////
//==================================
//==========转弯部分结束===============
//==================================
////////////////////////////////////

/////////////////////////////////////
//==================================
//==========对框部分===============
//==================================
////////////////////////////////////
void goToGreen(){
  //不同的场地必须重新调整摄像头参数！！！

  double dist,cnt,angl,aedg,cedg,cang,acos,ljust;
  //double AEDG=3800,CANG=0.25*3.1416/180,bedg=15,alpha=1.22,Kjus=-200;//原始参数
  double AEDG=4235,CANG=-5.33*0.001,bedg=8,alpha=1.22,Kjus=180;//原来是120！！
  int timecnt=0;
  int exitcnt=0;

  //AEDG是一个a边的比例换算参数，CANG=为弧度制比例参数；bedg可能表示摄像头到同高水平中心的距离（B边）；cos内均为弧度制参数
  //alpha为调整系数，角度制，用于计算∠C后补足误差
  //Kjus代表底盘转速的系数，越大转速越大；在∠a≈90°时近似认为cosa=a
  /*
  vout.takeSnapshot(GREEN,5);
  aedg=AEDG/LO(vout).height;//表示摄像头与篮筐的距离
  cang=CANG*(125-LO(vout).centerX)+alpha;//cang为角度制∠c
  cedg=sqrt(aedg*aedg+bedg*bedg-2*cos(cang)*aedg*bedg);//c边，表示车的同高水平中心到篮筐距离
  acos=(bedg*bedg+cedg*cedg-aedg*aedg)/2/bedg/cedg;//cosA，近似表示∠A
  ljust=Kjus*acos;
  if(!ERR(ljust,0,22)) ljust=0;
  Brain.Screen.printAt(1,40,"aedg:%f",aedg);
  Brain.Screen.printAt(1,60,"cang:%f",cang);
  Brain.Screen.printAt(1,80,"cedg:%f",cedg);
  Brain.Screen.printAt(1,100,"acos:%f",acos);
  Brain.Screen.printAt(1,120,"ljust:%f",ljust);
  */
  
    for(int i=0;i<350;i++)
  {
      dist=angl=cnt=0;
      for(int i=0;i<100;i++)
      {
        vout.takeSnapshot(GREEN,5);
        if(ERR(LO(vout).height,60,50)||ERR(LO(vout).width,60,50)) continue;//距离太远或太近不计算
        aedg=AEDG/LO(vout).height;//表示摄像头与篮筐的距离
        cang=CANG*(125-LO(vout).centerX)+alpha;//cang为角度制∠c
        cedg=sqrt(aedg*aedg+bedg*bedg-2*cos(cang)*aedg*bedg);//c边，表示车的同高水平中心到篮筐距离
        acos=(bedg*bedg+cedg*cedg-aedg*aedg)/2/bedg/cedg;//cosA，近似表示∠A
        dist+=cedg;//C边长度的和
        angl+=acos;//∠A角度的和
        cnt++;
      }
      if(cnt>30)
      {
        angl/=cnt;//A平均角度
        dist/=cnt;//C边平均长度
        ljust=Kjus*angl;//调角度对准篮筐
        vrun(ljust,-ljust);//转弯
        if(!ERR(ljust,0,11)){exitcnt++;}//如果底盘速度小于20则退出循环//曾经为5！！
        else exitcnt=0;//达到要求20次才推出程序
        if(exitcnt>20) {vrun(0,0); break;}
      }
      else 
      {
        ljust=0;   //ljust重置   
      }
    SLEEP(10)//
  }
  
}


/////////////////////////////////////
//==================================
//==========对框部分结束===============
//==================================
////////////////////////////////////

/////////////////////////////////////
//==================================
//==============其他===============
//==================================
////////////////////////////////////
void vopen()
//展开
{
  vup(80);
  SLEEP(100)
  vup(-50);
  SLEEP(100);
  vup(0);
}



/////////////////////////////////////
//==================================
//=============其他结束===============
//==================================
////////////////////////////////////




//先加速后匀速
void sp_spconstant(double cm,double v,int k)
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
    		    r=k*calSpd(delta,v);//这里的k是前进（+1）后退（-1）
		    }
		    else if(delta<0)
    	  {
    		    r=0;
    		    l=k*calSpd(delta,v);
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
            cv=k*19+0.45*x;//if v=60 then spdup=25cm
          else if(cm<25)
            cv=k*21+0.62*x;//if v=60 then spdup=8.6cm
          cv=lim(cv,v);
          vrun(cv+l,cv+r);
        }
        else if(fabs(x)<AD(cm))
        {
            vrun(y+l,y+r);
        }
        if(fabs(x)>=AD(cm))
        {
            vstop();
            break;
        }     
    }
}

void vup2(int v){
  v*=120;
  VST(18,v)VST(19,v) 
}
/*
//


*/
void zhang(){
  initRobot();

vopen();
SLEEP(100)
//R2

veat(120);
vdis(50,50,1);
//61


//投球，桩1
AT(-93,25)

goToGreen();//桩1自瞄
SLEEP(10)
veat(0);
vdis(35,55,1);
DAW(61,780)
//vr(15,15);
vup(100);
SLEEP(300)
vup(0);


//取R3
vdis(27,60,-1);
initGyro();
AT(150,25);
//AT(156,25)
veat(120);
vdis(71,30,1);

//桩2
veat(0);
AT(-91,30)
SLEEP(100)
goToGreen();//桩2自瞄
vup2(-60);//防止球太靠上
SLEEP(250)
vup(0);
vdis(11,50,1);
//=========================
//==============goToGreen();//桩2自瞄
DAW(40,600)
vup(100);
SLEEP(300)
vup(0);

//取R4
vdis(19,70,-1);


AT(208,25);
veat(120);
//goToGreen();
vdis(46,40,1);


//////////////////////////////
AT(25,30);
vdis(10,40,1);
DAW(60,450)
AT(-30,30);
vup(100);
SLEEP(300)
vup(0);

vdis(14,40,-1);

////////////角桩//////////////////
//取R5
//vdis(35,40,-1);
AT(-115,25)
//goToGreen();//桩3自瞄
veat(120);
vdis(65,30,1);
SLEEP(200)
goToGreen();//桩3自瞄
vdis(65,30,1);
goToGreen();//桩3自瞄
DAW(40,750)
goToGreen();//桩3自瞄
vup(100);//吐球

SLEEP(400)
vup(0);


}

void song(){
  initRobot();
    initGyro();

    //////////开    ************     始///////////////////////
     goToGreen();          //对框
    speedconstant(9,60,-1);   //后退
     goToGreen();          //对框
    SLEEP(200);
    AT(85,30);                  //顺时针90
    SLEEP(20); 

    veat(100);                    
    speedconstant(90,30,1);   //前进吸球
    AT(-45,35);                 //逆时针45
    goToGreen();          //对框
    
    speedconstant(15,60,1);   //前进
    veat(0);
     goToGreen();          //对框
  
    DAW(50,500)           //前进
    SLEEP(20);
    vup(100);                //吐球
    SLEEP(300);
    vup(0);
    speed(27,60,-1);           //后退
    SLEEP(20);
    AT(147,35);     //顺时针转135

    SLEEP(20);
    veat(100);      //吸球
    vdis(82,35,1);             //走直线
    SLEEP(20);
   
                    
      //vup(80);

    //turnAngle_simple(-900);    //逆时针转90
    
    AT(-90,35);           //逆时针转90
    SLEEP(20);
   veat(0);
    goToGreen();          //对框
  
 
    speedconstant(15,60,1); //前进
    SLEEP(20);
     goToGreen();          //对框
  
    DAW(40,500)

     SLEEP(20);
     vup(100);                  //吐球
     SLEEP(300);
      vup(0);
    speed(15,60,-1);           //后退
    SLEEP(20);
    AT(190,40);      //转180
    SLEEP(20);
   
    /////R8
     veat(80);                   //吸球
    speedconstant(45,60,1);     //前进
    
    

    AT(10,35); 
    DAW(100,1000);          //捅lan'qiu
    
    
    speedconstant(15,40,-1);            //后退*1
    SLEEP(500);
    veat(0);
    speedconstant(13,40,1);     //前进
    DAW(80,500)           //前进
    AT(-20,40);
    vup(100);                    //吐球
    SLEEP(1000);
    vup(0);


    speedconstant(10,60,-1);            //后退*1
    SLEEP(500);
    AT(10,35);
    SLEEP(20);
    speedconstant(10,40,1);             //前进*2 
    DAW(20,500)
    
    speedconstant(10,60,-1);            //后退*2
    SLEEP(500);
    speedconstant(10,40,1);             //前进*3 
    DAW(60,500)
    speedconstant(20,40,-1);            //后退*3
    SLEEP(500);
    
   
    
    SLEEP(20);
    SLEEP(20);
    
    /////////////////////////////////////////*
    /*
    speedconstant(40,60,-1);    //后退
    SLEEP(20);
    AT(-90,40);      //逆时针转90
    SLEEP(20);


    vdis(70,70,1);         //前进
    SLEEP(20);
//////////////////////////////////////////////////////////
    ////R9
    //turnAngle_simple(450);   //顺时针转45
    AT(45,40);
    SLEEP(20);
    speedconstant(10,50,1); //前进
    SLEEP(20);
    veat(80);                //吸球
    SLEEP(20);
    speedconstant(10,50,-1);//后退
SLEEP(20);
    //turnAngle_simple(-900);   //逆时针转90
    AT(-90,40);
    SLEEP(20);
    goToGreen();
    SLEEP(20);
    speedconstant(40,60,1);//前进
    DAW(50,500)
    SLEEP(20);
    
    vup(80);           //吐球
    SLEEP(20);
    vdis(70,60,-1);//后退
    SLEEP(20);
    //turnAngle_simple(1350);
    AT(135,60);
    SLEEP(20);
    vdis(120,70,1);  //前进
    SLEEP(20);
    //turnAngle_simple(-900); //逆时针转90
    AT(-90,60);
    SLEEP(20);
    goToGreen();
    SLEEP(20);
    speedconstant(30,60,1); //前进
    SLEEP(20);
    vup(80);         //吐球
   SLEEP(20);
    /////R10
    vdis(30,60,1);         //后退
SLEEP(20);
    //turnAngle_simple(1800);   //转180
    AT(180,60);
    SLEEP(20);
    speed(60,60,1);   //前进
SLEEP(20);
    veat(80);   //吸球
    SLEEP(20);
    AT(10,60);
    SLEEP(20);
    speedconstant(70,60,1);     //前进
    SLEEP(20);
    speed(20,60,-1);            //后退*1
    SLEEP(20);
    speed(20,60,1);             //前进*2 
    SLEEP(20);
    speed(20,60,-1);            //后退*2
    SLEEP(20);
    speed(20,60,1);             //前进*3 
    SLEEP(20);
    speed(20,60,-1);            //后退*3
    SLEEP(20);
    goToGreen();
    SLEEP(20);
    speedconstant(20,60,1);     //前进
    SLEEP(20);
    vup(80);                    //吐球
    SLEEP(20);*/
}






int main() {
  //initRobot();
  /*initGyro();
  while(1){
  
  if(BP(A)) AT(-90,26);
  if(BP(B)) gturn(45,6);
  }*/
  zhang();
  song();
}
