/**********************************************************************
这是一个从老版本移植进来的程序，如果使用此文件的程序结构，可以实现直接粘贴移植
每个人的程序风格不同，这里加了注释以便于大家理解
程序中陀螺仪转弯的程序是由另一位学长写的
大家着重看一下底盘前进、转弯的函数，其他不知道是啥的机器人上层结构不懂的可以不看
去年10月南昌赛后曾迭代过一次前进函数，效果还不错但没再打过比赛了，转弯函数我这里没有最新迭代版本
前进、转弯的函数虽然打过比赛但不一定就是很好的，有些方法值得借鉴，但也需要改进
***********************************************************************/

/********************
这个自动程序去年南昌亚锦赛排第二，还算不错
********************/


#include "vex.h"

using namespace vex;




double left_speed  = 0;
double right_speed = 0;
double eat_speed = 0;  


void initRobot()
//机器人所有电机编码器清零，陀螺仪初始化

{
    l1.resetRotation();
    l2.resetRotation();
    l3.resetRotation();
    l4.resetRotation();
    
    r1.resetRotation();
    r2.resetRotation();
    r3.resetRotation();
    r4.resetRotation();
    
    bodl1.resetRotation();
    bodr1.resetRotation();
    bodl2.resetRotation();
    bodr2.resetRotation();
    
    eatl1.resetRotation();
    eatr1.resetRotation();

    eatl2.resetRotation();
    eatr2.resetRotation();

    arml.resetRotation();
    armr.resetRotation();
    
    initGyro();
 
}


void vclear()
//机器人底盘电机编码器清零
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

void clearfly()
//机器人翅膀结构电机清零
//关于什么是“翅膀”可以不用管，只需知道是一个结构就行了，下均同
{
    arml.resetRotation();
    armr.resetRotation();
    SLEEP(100);
}

void vstop()
//底盘停止
//此处使用0速度spin，以提供一个强力让其停止，抵消惯性影响
{
    l1.spin(vex::directionType::fwd,0,vex::velocityUnits::rpm);
    l2.spin(vex::directionType::fwd,0,vex::velocityUnits::rpm);
    l3.spin(vex::directionType::fwd,0,vex::velocityUnits::rpm);
    l4.spin(vex::directionType::fwd,0,vex::velocityUnits::rpm);
    r1.spin(vex::directionType::fwd,0,vex::velocityUnits::rpm);
    r2.spin(vex::directionType::fwd,0,vex::velocityUnits::rpm);
    r3.spin(vex::directionType::fwd,0,vex::velocityUnits::rpm);
    r4.spin(vex::directionType::fwd,0,vex::velocityUnits::rpm);
    /*l1.stop();
    l2.stop();
    l3.stop();
    l4.stop();
    
    r1.stop();
    r2.stop();
    r3.stop();
    r4.stop();*/
    SLEEP(100)
}


int lim(int a,int t) 
{
	a=a>t?t:(a<-t?-t:a);
	return a;
	
}

double getEncoderValue()
//basic function to get the BOTTOM motor's rotation
//左前右前左后右后取平均
{
	float x=(l1.rotation(vex::rotationUnits::deg)+r1.rotation(vex::rotationUnits::deg)+r4.rotation(vex::rotationUnits::deg)+l4.rotation(vex::rotationUnits::deg))/4;
	return x;
}


void put(double l,double v)
//basic function to make body up
//使抬升结构抬升至一定角度，完成后才跳出函数
{
    ROTS(bodl1,l,v)
    ROTS(bodr1,l,v)
    ROTS(bodl2,l*7,v*7)
    ROT(bodr2,l*7,v*7)
    SLEEP(200)
}


void Sput(double l,double v)
//basic function to make body up
//开始使抬升结构抬升至一定角度，无需完成即可执行后续代码
{
    ROTS(bodl1,l,v)
    ROTS(bodr1,l,v)
    ROTS(bodl2,l*7,v*7)
    ROTS(bodr2,l*7,v*7)
    //SLEEP(200)
}


void veat(double v)
//吸“方块”结构电机加电压
//eat
{
    v*=120;
    VST(17,v)VST(18,v)VST(13,v)VST(14,v)
}


void vrun(double lv,double rv)
//底盘电机加电压
//Vrun
{
    lv*=120,rv*=120;
    VST(1,lv)VST(2,lv)VST(3,lv)VST(4,lv)
    VST(10,rv)VST(9,rv)VST(8,rv)VST(7,rv)
}


void vbod(double v)
//抬升结构加电压
//basic function to make body up
{
    v*=120;
    VST(20,v)VST(11,v)VST(6,v)VST(5,v)
}

void vfly(double v)
//翅膀加电压
{
    v*=120;
    VST(19,v)VST(12,v)
}


/******************************************************************
the code below are the functions of the gyro
以下为使用陀螺仪转弯的函数
这部分不是我写的，底层函数可能有重复
*******************************************************************/

void initGyro()
//陀螺仪初始化，需要断电3秒再上电才能是陀螺仪回复初始状态
{
  gyro_out.set(false);
  vex::this_thread::sleep_for(3000);//也可以用SLEEP(3000)
  gyro_out.set(true);
  vex::this_thread::sleep_for(200);
}


void setUnderVoltage(int leftVoltage, int rightVoltage)
//有一个底盘加电压的函数，和上面一模一样只不过单位不同
{
  //-12000~12000 mV
  //left
  vexMotorVoltageSet(vex::PORT10, leftVoltage);
  vexMotorVoltageSet(vex::PORT9 , leftVoltage);
  vexMotorVoltageSet(vex::PORT8, leftVoltage);
  vexMotorVoltageSet(vex::PORT7, leftVoltage);

  //right
  vexMotorVoltageSet(vex::PORT1, rightVoltage);
  vexMotorVoltageSet(vex::PORT2,  rightVoltage);
  vexMotorVoltageSet(vex::PORT3, rightVoltage);
  vexMotorVoltageSet(vex::PORT4, rightVoltage);
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


void turnAngle_simple_accurate(int angleVal)
{
  //先声明以下函数1年前写的当时没注释，现在注释有错误请指出谢谢
  //务必写程序一定要写注释！！！
  //angleVal CCW（逆时针） -1800~1800
  //////LOW/HIGH/BREAK NEED TO CHENGE
  /////NUM OF FUNCTION PARAMETER MAY CHANGING
  //这个函数精度比较高，只有达到精度要求才能跳出循环，但没用PID
  //后面一些注释中的数值是调参遗迹
  const int EPS = 10;//10->15//误差
  const int LOW_VOLTAGE = 3400;//2800->3000//最低电压，能让机器人转动的最低电压
  const int HIGH_VOLTAGE = 5800; //5500->5800//高电压，用于快速转动
  const int BRAKE_VOLTAGE = -1500;//刹车电压，用于克服惯性刹车

  int bgnVal = 0;
  int endVal = 0;
  int curVal = 0;

  int Voltage = 0;

  bgnVal = getGyroValue();//初始角度获取陀螺仪角度
  endVal = bgnVal + angleVal;//结束角度为当前角度+相对角度

//下面2行将当前角度控制在0-3600之间
  while(endVal >  3600) endVal -= 3600;
  while(endVal < 0) endVal += 3600;

//计算当前相差角度值
  curVal = getRelativeAngle(getGyroValue(), endVal);
  while(abs(curVal) > 500)//如果需要转的角度很大，大于50度，则用高电压转
  {
    Voltage = HIGH_VOLTAGE * (curVal > 0 ? 1 : -1);
    setUnderVoltage(Voltage, -Voltage);

    vex::this_thread::sleep_for(2);
    curVal = getRelativeAngle(getGyroValue(), endVal);
  }

//快速转动完毕，这里克服一下惯性把速度降下来
  Voltage = BRAKE_VOLTAGE * (Voltage > 0 ? 1 : -1);
  setUnderVoltage(Voltage, -Voltage);
  vex::this_thread::sleep_for(500);

//再算一下角度
  curVal = getRelativeAngle(getGyroValue(), endVal);
  while(abs(curVal) > EPS)//以小速度转剩余角度，EPS为误差
  {
    Voltage = LOW_VOLTAGE * (curVal > 0 ? 1 : -1);
    setUnderVoltage(Voltage, -Voltage);

    vex::this_thread::sleep_for(2);
    curVal = getRelativeAngle(getGyroValue(), endVal);
  }
//刹车
  Voltage = BRAKE_VOLTAGE * (Voltage > 0 ? 1 : -1);
  setUnderVoltage(Voltage, -Voltage);
  vex::this_thread::sleep_for(275);
//停止，不过不建议这样停止，建议使用vstop() 
  Voltage = 0;
  setUnderVoltage(Voltage, Voltage);
  vex::this_thread::sleep_for(200);
}


void turnAngle_simple(int angleVal)
//类似上述函数，不再给详细注释，下均同
//angleVal CCW -1800~1800
  //////LOW/HIGH/BREAK NEED TO CHENGE
  /////NUM OF FUNCTION PARAMETER MAY CHANGING
{
  //angleVal CCW -1800~1800
  const int EPS = 20;//15->20
  const int LOW_VOLTAGE = 5300;//2800//3200//5000
  const int HIGH_VOLTAGE = 5800;//5500
  const int BRAKE_VOLTAGE = -1300;//-2000
    
  int bgnVal = 0;
  int endVal = 0;
  int curVal = 0;

  int Voltage = 0;
    
  int k=0;

  bgnVal = getGyroValue();
  endVal = bgnVal + angleVal;

  while(endVal >  3600) endVal -= 3600;
  while(endVal < 0) endVal += 3600;

  curVal = getRelativeAngle(getGyroValue(), endVal);
  while(abs(curVal) > 500)//
  {
    Voltage = HIGH_VOLTAGE * (curVal > 0 ? 1 : -1);
    setUnderVoltage(Voltage, -Voltage);

    SLEEP(2);
    curVal = getRelativeAngle(getGyroValue(), endVal);
  }

  Voltage = BRAKE_VOLTAGE * (Voltage > 0 ? 1 : -1);
  setUnderVoltage(Voltage, -Voltage);
  SLEEP(150);//500

  
  curVal = getRelativeAngle(getGyroValue(), endVal);
  while(abs(curVal) > EPS)
  {
    Voltage = LOW_VOLTAGE * (curVal > 0 ? 1 : -1);
    setUnderVoltage(Voltage, -Voltage);

    SLEEP(2);
    curVal = getRelativeAngle(getGyroValue(), endVal);
    //加入k来控制循环次数，防止死循环，但精度降低
      k++;
      if(k>80)////////////////////40->80   40is too low
      break;
  }

  Voltage = BRAKE_VOLTAGE * (Voltage > 0 ? 1 : -1);
  setUnderVoltage(Voltage, -Voltage);
  SLEEP(100);//150
  
  Voltage = 0;
  setUnderVoltage(Voltage, Voltage);
  SLEEP(100);
}

void turnAngle_fast(int angleVal)
//名字啥的都不重要了说是fast实际上就是改了一下LOW_VOLTAGE、HIGH_VOLTAGE什么的实际上也没改多少
//主要是当初需要不同的电压配型所以写了好多差不多的函数
//angleVal CCW -1800~1800
  //////LOW/HIGH/BREAK NEED TO CHENGE
  /////NUM OF FUNCTION PARAMETER MAY CHANGING
{
  //angleVal CCW -1800~1800
  const int EPS = 30;//15->20
  const int LOW_VOLTAGE = 5300;//2800//3200//5000
  const int HIGH_VOLTAGE = 5800;//5500
  const int BRAKE_VOLTAGE = -1300;//1500
    
  int bgnVal = 0;
  int endVal = 0;
  int curVal = 0;

  int Voltage = 0;
    
  int k=0;

  bgnVal = getGyroValue();
  endVal = bgnVal + angleVal;

  while(endVal >  3600) endVal -= 3600;
  while(endVal < 0) endVal += 3600;

  curVal = getRelativeAngle(getGyroValue(), endVal);
  while(abs(curVal) > 500)
  {
    Voltage = HIGH_VOLTAGE * (curVal > 0 ? 1 : -1);
    setUnderVoltage(Voltage, -Voltage);

    SLEEP(2);
    curVal = getRelativeAngle(getGyroValue(), endVal);
  }

  Voltage = BRAKE_VOLTAGE * (Voltage > 0 ? 1 : -1);
  setUnderVoltage(Voltage, -Voltage);
  SLEEP(150);

  
  curVal = getRelativeAngle(getGyroValue(), endVal);
  while(abs(curVal) > EPS)
  {
    Voltage = LOW_VOLTAGE * (curVal > 0 ? 1 : -1);
    setUnderVoltage(Voltage, -Voltage);

    SLEEP(2);
    curVal = getRelativeAngle(getGyroValue(), endVal);
      k++;
      if(k>80)////////////////////40->80   40is too low
      break;
  }

  Voltage = BRAKE_VOLTAGE * (Voltage > 0 ? 1 : -1);
  setUnderVoltage(Voltage, -Voltage);
  SLEEP(100);//275
  
  Voltage = 0;
  setUnderVoltage(Voltage, Voltage);
  SLEEP(100);
}



void turnToTarget(int targetAngle)
//把上面的函数包装了以下，转换成了绝对坐标系值（车头为0，逆时针为0~+1800，顺时针为-~-1800）下均同
{
  //1800 ~ 0 ~ -1800 , CCW

  int curAngle = getGyroValue() - 1800;

  int relativeAngle = getRelativeAngle(targetAngle, curAngle);

  turnAngle_simple(relativeAngle);
}

void turnToTargetAccurate(int targetAngle)
{
  //1800 ~ 0 ~ -1800 , CCW

  int curAngle = getGyroValue() - 1800;

  int relativeAngle = getRelativeAngle(targetAngle, curAngle);

  turnAngle_simple_accurate(relativeAngle);
}

/**********************************************************************
END gyro function
看代码写注释真累一定要在写代码的同时就写注释啊
************************************************************************/

/************************
code below is functions to make car go straight


************************/
void vturnto_simple(int deg);

void keepStraight(double gyrodeg,double exa)
//使用陀螺仪纠偏，主要用于走直线
//使用陀螺仪检测当前角度，当走直线偏离角度太大就停下来摆正
//虽然有用但很慢，新版本中舍弃了，不过竞赛中为技能赛精度提升做了不少贡献
{ 
    int x=getGyroValue()-1800;
    if(x>=(gyrodeg+exa))  
    {
        turnAngle_fast(gyrodeg-x);
    }
    else if(x<=(gyrodeg-exa))
    {
        turnAngle_fast(gyrodeg-x);
    }
}


void vturnDEG(double degree)
//编码器转弯，转变了一下方向，逆时针为正
//NEED TO BE TESTED
{
	AT(-degree,30);
    
}

void vturnDEG(double degree,double v)
//NEED TO BE TESTED
{
	AT(-degree,v);
    
}

//////////////////////////////////////
//下面是最顶层的函数包装
//思想是前95%的角度用编码器，后5%的角度用陀螺仪转弯
//加速转弯，并用很快速度拨开挡在路线上的方块，进一步加快转弯速度
//代码简单自己看
/////////////////////////////////////
void vturnto_simple(int deg)
//turnToTarget_simple(quick
{
	//1800 ~ 0 ~ -1800 , CCW

    int curAngle = getGyroValue() - 1800;
    int relativeAngle = getRelativeAngle(deg, curAngle);
	vturnDEG(0.095*relativeAngle,40);//0.1
    SLEEP(100)
	curAngle = getGyroValue() - 1800;
    relativeAngle = getRelativeAngle(deg, curAngle);
	turnAngle_simple(relativeAngle);
	
}


void vturnto(int deg)
//turnToTarget(quick
{
	//1800 ~ 0 ~ -1800 , CCW

    int curAngle = getGyroValue() - 1800;
    int relativeAngle = getRelativeAngle(deg, curAngle);
	vturnDEG(0.095*relativeAngle);
    SLEEP(100)
	curAngle = getGyroValue() - 1800;
    relativeAngle = getRelativeAngle(deg, curAngle);
	turnAngle(relativeAngle);
	
}

//////////////////////////////////
//下面的代码是走直线的函数
//这一堆代码已经被迭代过了所以就不写注释了，在新文件里面写注释
///////////////////////////////////
void vSpeedStill(double cm,double spd,double deg,double exa)
{
    vclear();
    double x=getEncoderValue();
    if (cm<0) spd=-spd;
	while(fabs(x)<fabs(AD(cm)))
	{
	keepStraight(deg,exa);
	x=getEncoderValue();
	vrun(spd,spd);	
	if(fabs(x)>=fabs(AD(cm))) 
    {break;}
    }
    
}


void vdis(double cm,double v,double deg,double exa,int k)
    /*
    function to make car go straight,keep straight for a distance
    slowly speed up and speed down
    */
{
    vclear();
    double x=getEncoderValue(),y=k*v;
    double cv;
    while(1)
    {
        x=getEncoderValue();
        if(fabs(x)<0.2*AD(cm))
        {
            cv=k*25+0.2*x;
            cv=lim(cv,v);
            vrun(cv,cv);
        }
        else if(fabs(x)<0.6*AD(cm))
        {
            vrun(y,y);
        }
        else if(fabs(x)<AD(cm))
        {
            cv=fabs(AD(cm)-fabs(x))*y/fabs(AD(cm));
            cv=lim(cv,v);
            if (fabs(cv)<=25)
            {
                if (cv<0) cv=-25;
                else if(cv>0) cv=25;
                else {vstop();break;}
            }
            vrun(cv,cv);
        }
        else
            vstop();
        if(fabs(x)>=AD(cm))
        {
            vstop();
            break;
        }
        keepStraight(deg,exa);
    }
}

void vdis(double cm,double v,int k)
    /*
    function to make car go straight,keep straight for a distance
    slowly speed up and speed down
    */
{
    vclear();
    double x=getEncoderValue(),y=k*v;
    double cv;
    while(1)
    {
        x=getEncoderValue();
        if(fabs(x)<0.2*AD(cm))
        {
            cv=k*25+0.2*x;
            cv=lim(cv,v);
            vrun(cv,cv);
        }
        else if(fabs(x)<0.6*AD(cm))
        {
            vrun(y,y);
        }
        else if(fabs(x)<AD(cm))
        {
            cv=fabs(AD(cm)-fabs(x))*y/fabs(AD(cm));
            cv=lim(cv,v);
            if (fabs(cv)<=25)
            {
                if (cv<0) cv=-25;
                else if(cv>0) cv=25;
                else {vstop();break;}
            }
            vrun(cv,cv);
        }
        else
            vstop();
        if(fabs(x)>=AD(cm))
        {
            vstop();
            break;
        }
    }
}

void vdisFast(double cm,double v,double deg,double exa,int k)
    /*
    function to make car go straight,keep straight for a distance
    slowly speed up and speed down
    */
{
    vclear();
    double x=getEncoderValue(),y=k*v;
    double cv;
    while(1)
    {
        x=getEncoderValue();
        if(fabs(x)<0.3*AD(cm))
        {
            cv=k*25+0.2*x;
            cv=lim(cv,v);
            vrun(cv,cv);
        }
        else if(fabs(x)<(AD(cm)))
        {
            vrun(y,y);
        }
                /*
        else if(fabs(x)<AD(cm))
        {
            cv=fabs(AD(cm)-fabs(x))*y/fabs(AD(cm));
            cv=lim(cv,v);
            if (fabs(cv)<=35)
            {
                if (cv<0) cv=-35;
                else if(cv>0) cv=35;
                else {vstop();break;}
            }
            vrun(cv,cv);
        }*/
        else
            vstop();
        if(fabs(x)>=AD(cm))
        {
            vstop();
            break;
        }
        keepStraight(deg,exa);
    }
}

/**********************
END vdis
**********************/

void daw(int v,int t,int k) 
//撞墙兼陀螺仪初始化，减少陀螺仪累计误差
    //k==1 initGyro
{
    ASMT(15);
    ASP(v);
    SLEEP(t);
    ASP(0);
    if (k==1)
        initGyro();
    ASMT(100);
}

/*****************
THE function below is to make the arm move while going forward
下面的是让翅膀结构动的程序，别看了今年赛季绝对没有
说实在这个设计也很诡异没啥用
*****************/


void arm(int ldeg,int rdeg)    
{
    SMT(arml,7)SMT(armr,7)
    ROTS(arml,ldeg,40)//
    ROTS(armr,rdeg,40)//
    SLEEP(50);
}

void set_arm_hold()//set arm motor stopping hold
{
 arml.setStopping(vex::brakeType::hold);
 armr.setStopping(vex::brakeType::hold);
}
int aimarm;
int aimalm;
int ARM()
{
    SMT(arml,7)SMT(armr,7)
    while(true)
    {
        ROTS(arml,-aimalm,60)
        ROTS(armr,-aimarm,60)
        SLEEP(50)
    }
    return 0;
}

void arm()
{
    ROFS(arml,10,90)
    ROFS(armr,10,90)
}


void arm_test()
    
{clearfly();
 initGyro();
 //initRobot();
 arm(170,170);
 AF(50,30)
 arm(197,197);
 for(int i=1;i<=2;i++) AF(-20,5*i)
 veat(-60);
 SLEEP(1000);
 arm(0,0);
 veat(-60);
 AF(100,30)    
}

/***************************
下面是抬升结构的函数，不重要但可以了解一下，只当看看除底盘外其他结构怎么控制
比较简单不做注释了
***************************/



void bodyDown();

void bodyUp()
{
    
    int m=fabs(DEG(bodl1));
    veat(50);
    SLEEP(400);//900//700
    veat(-30);//23
    SLEEP(1365);
    veat(15);//20
    while(1)
    {
        m=fabs(DEG(bodl1));
        vbod(-100);
        if(m>=30)
        {
            break;
        }
    }
    veat(0);
    put(-95,4);
    veat(-5);
    put(-110,1.6);//added//1.6
    AFS(6,4.5);//6,5
    put(-136,1.4);//132.1,1.6
    veat(0);
    //AFS(5,5);
    SLEEP(2000);
    put(0,4.28);
    veat(-100);
    AF(-29,100);
    veat(0);
    
}

void bodyDown()
{
    put(0,14);
}


int main() {
//自动程序主函数
//可以看看语句怎么写，场地路线啥的不用管
  vex::task TA(ARM);//强烈不建议用多线程！！！！！！！！！！！！！！
  //initRobot();   
    set_arm_hold();
    //clearfly();
    //initRobot();//clear
    //arm(180,180);
    aimarm=aimalm=160;
    veat(80);
    initRobot();
    //SLEEP(3000);//eat pre-load tube
    //put(-30,5);
    SLEEP(50);
    //Sput(0,10);//undo finished
    //veat(60);
    vdis(46,60,0,50,1);
    //arm(207,207);
    aimarm=aimalm=207;
    SLEEP(500);
    veat(0);/////////////////////////////////////////////
    //vdis(52,20,0,50,-1);//full down 4 tubes
    /*AF(-30,25);
    AF(-20,40);*/
    AF(-50,30);
    veat(80);
    //arm(0,0);
    aimarm=aimalm=10;
    AF(98,30);//eat 4 tubes//25//35
    SLEEP(100);
    //vturnto_simple(900);
    turnToTarget(900);
   
    
    vdis(6.5,60,900,50,1);
    SLEEP(50);
    //arm(207,207);
    aimarm=aimalm=207;
    SLEEP(700);
    veat(0);//40
    vdis(60,40,-1);//pull down 3tubes//
    aimarm=aimalm=10;
    veat(100);
    vdis(95,30,900,60,1);
    SLEEP(300);
    vdisFast(107,100,900,35,-1);
    //aimalm=0;//////////
    daw(-30,1000,1);//against the wall and initGyro
    
    
    veat(-40);
    SLEEP(700);
    veat(70);
    vdis(26,60,0,50,1);//33
    //SLEEP(200);
    turnToTarget(900);//915
    //SLEEP(200)
    //aimarm=aimalm=0;/////////////
    //veat(80);
    vdis(76.4,80,900,50,1);//72
    //DAW(20,500)
    turnToTarget(1450);//turn to point area
    vdis(14.5,35,1);
    DAW(35,700);//20,1000
    bodyUp();
    
    aimarm=aimalm=-1;
    turnToTarget(800);
    vdisFast(67,100,800,1000,-1);
    turnToTarget(880);
    vdisFast(135,100,880,1000,-1);
    AF(60,100);
  
}