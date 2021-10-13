#include "vex.h"
#define GTD(a) ((double)(1811 - a) / 8.16833)
#define GDEG GTD(gyr.value(vex::analogUnits::range12bit))
#define LO(a) a.largestObject
#define ERR(a, b, c) ((a > b + c) || (a < b - c))

const int VIN_SPEED = 40;  //吸环电机速度
const int VSIDE_ARM_SPEED = 70; //侧向手臂电机速度
const int VFRONT_ARM_SPEED=100; //前向手臂电机速度
const int VFRONT_PAW_SPEED=60; //前向手臂末端爪子电机速度

void VRUN(double l, double r) {
  VVST(1, 2, l)
  VVST(3, 4, l)
  VVST(6,8, r)
  VVST(9,10, r)
}

void Vpaw(double v, int temp) // temp is 1 or -1
{
  int V = v * temp;
  if (temp) {
    // if(DEG(lup)<=135)
    VVST(11, 20, V)
  }
  if (!temp) {
    // if(DEG(lup)<=135)
    VVST(11, 20, V)
  }
}

void Vin(double v, int temp) // thw大车组吸球测试
{

  int V = v * temp;
  if (temp) {
    // if(DEG(lup)<=135)
    VST(12, V * 120)
  }
  if (!temp) {
    // if(DEG(lup)<=135)
    VST(12, V * 120)
  }
}

//大车侧向手臂控制，v：速度，temp：0/1 控制方向
void VSideArm(double v, int temp) {
  int V = v * temp;
  VST(16, V * 120);
}
//大车前方平行四边形上臂驱动
void VFrontArm(double v,int temp){
  int V=v*temp;
  VST(17, V * 120);
  VST(18, V * -120);
}
//打车前方平行四边形末端爪子驱动
void VFrontPaw(double v,int temp){
  int V=v*temp;
  VST(19,V*120);
}
/*void Select()
{
    double x,lastx,startx,stage;
    stage=0;
    x=sl.rotation(vex::rotationUnits::deg);
    startx=lastx=x;
    ROTS(sl,120,80)
    while((x-lastx)>5)
    {
    x=sl.rotation(vex::rotationUnits::deg);
    lastx=x;
    SLEEP(300)
    }
    VST(18,0)
}
*/
//底盘运动
void vrun(double lv,double rv)
//底盘电机加电压
//Vrun
{
    lv*=120,rv*=120;
    VST(1,lv)VST(2,lv)VST(3,lv)VST(4,lv)
    VST(10,rv)VST(9,rv)VST(8,rv)VST(7,rv)
}
//重置底盘电机的编码器
void resetChassisEncoder(){
  rf.resetRotation();
  rm1.resetRotation();
  rm2.resetRotation();
  rb.resetRotation();
  lf.resetRotation();
  lm1.resetRotation();
  lm2.resetRotation();
  lb.resetRotation();
}
//底盘刹车
void stopChassis(){
  rf.spin(vex::directionType::fwd,0,vex::velocityUnits::rpm);
  rm1.spin(vex::directionType::fwd,0,vex::velocityUnits::rpm);
  rm2.spin(vex::directionType::fwd,0,vex::velocityUnits::rpm);
  rb.spin(vex::directionType::fwd,0,vex::velocityUnits::rpm);
  lf.spin(vex::directionType::fwd,0,vex::velocityUnits::rpm);
  lm1.spin(vex::directionType::fwd,0,vex::velocityUnits::rpm);
  lm2.spin(vex::directionType::fwd,0,vex::velocityUnits::rpm);
  lb.spin(vex::directionType::fwd,0,vex::velocityUnits::rpm);
  SLEEP(200); //等待刹车完全停止
}
float getChassisEncoder(){
  float rightAns=0.0,leftAns=0.0;
  rightAns=(rf.rotation(vex::rotationUnits::deg)+rm1.rotation(vex::rotationUnits::deg)+rm2.rotation(vex::rotationUnits::deg)+rb.rotation(vex::rotationUnits::deg))/4;
  leftAns=(lf.rotation(vex::rotationUnits::deg)+lm1.rotation(vex::rotationUnits::deg)+lm2.rotation(vex::rotationUnits::deg)+lb.rotation(vex::rotationUnits::deg))/4;
  return (rightAns+leftAns)/2;
}
int lim(int a,int t) 
{
	a=a>t?t:(a<-t?-t:a);
	return a;
	
}
void vdis(double cm,double v,int k)
    /*
    function to make car go straight,keep straight for a distance
    slowly speed up and speed down
    */
{
    resetChassisEncoder();
    double x=getChassisEncoder(),y=k*v;
    double cv;
    while(1)
    {
        x=getChassisEncoder();
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
                else {stopChassis();break;}
            }
            vrun(cv,cv);
        }
        else
            stopChassis();
        if(fabs(x)>=AD(cm))
        {
            stopChassis();
            break;
        }
    }
}
void autonomous(){
  resetChassisEncoder();
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
  manual();
  return 0;
}
