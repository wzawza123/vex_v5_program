#include "vex.h"
#define GTD(a) ((double)(1811 - a) / 8.16833)
#define GDEG GTD(gyr.value(vex::analogUnits::range12bit))
#define LO(a) a.largestObject
#define ERR(a, b, c) ((a > b + c) || (a < b - c))

const int VIN_SPEED = 100;  //吸环电机速度
const int VSIDE_ARM_SPEED = 100; //侧向手臂电机速度
const int VFRONT_ARM_SPEED=100; //前向手臂电机速度
const int VFRONT_PAW_SPEED=60; //前向手臂末端爪子电机速度

// vex::pwm_out PWM1=vex::pwm_out(Brain.ThreeWirePort.A);
// vex::pwm_out PWM2=vex::pwm_out(Brain.ThreeWirePort.B);

void VRUN(double l, double r) {
  VVST(L1, L2, l)
  VVST(L3, L4, l)
  VVST(R1,R2, r)
  VVST(R3,R4, r)
}

void Vpaw(double v, int temp) // temp is 1 or -1
{
  int V = v * temp;
  if (temp) {
    // if(DEG(lup)<=135)
    VVST(UP1,UP2, V)
  }
  if (!temp) {
    // if(DEG(lup)<=135)
    VVST(UP1,UP2, V)
  }
}

void Vin(double v, int temp) // thw大车组吸球测试
{

  int V = v * temp;
  if (temp) {
    // if(DEG(lup)<=135)
    VST(TAKEIN1, V * 120)
    VST(TAKEIN2,V*120)
  }
  if (!temp) {
    // if(DEG(lup)<=135)
    VST(TAKEIN1, V * 120)
    VST(TAKEIN2,V*120)
  }
}

//大车侧向手臂控制，v：速度，temp：0/1 控制方向
void VSideArm(double v, int temp) {
  int V = v * temp;
  VST(SIDE_ARM, V * 120);
}
//大车前方平行四边形上臂驱动
void VFrontArm(double v,int temp){
  int V=v*temp;
  VST(FRONT_ARM1, V * 120)0;
  VST(FRONT_ARM2, V * 120);
}
//打车前方平行四边形末端爪子驱动
void VFrontPaw(double v,int temp){
  int V=v*temp;
  VST(FRONT_PAW,V*120);
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
    VST(15,0)
}
*/
//底盘运动
void vrun(double lv,double rv)
//底盘电机加电压
//Vrun
{
    lv*=120,rv*=120;
    VST(L1,lv)VST(L2,lv)VST(L3,lv)VST(L4,lv)
    VST(R2,rv)VST(R2,rv)VST(R3,rv)VST(R4,rv)
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
float getSideChassisEncoder(bool isLeft=true){
  float rightAns=0.0,leftAns=0.0;
  rightAns=(rf.rotation(vex::rotationUnits::deg)+rm1.rotation(vex::rotationUnits::deg)+rm2.rotation(vex::rotationUnits::deg)+rb.rotation(vex::rotationUnits::deg))/4;
  leftAns=(lf.rotation(vex::rotationUnits::deg)+lm1.rotation(vex::rotationUnits::deg)+lm2.rotation(vex::rotationUnits::deg)+lb.rotation(vex::rotationUnits::deg))/4;
  if(isLeft){
    return leftAns;
  }else{
    return rightAns;
  }
}
void auto_ringsStart(){
  VSideArm(100,-1);
  SLEEP(150);
  VSideArm(-100,1);
  SLEEP(100);
  VSideArm(0,0);
}
void auto_reset_ringsStart(){
  VSideArm(100,-1);
  SLEEP(1000);
  VSideArm(100,1);
}
void auto_runDistance_smoothly(float dist_cm){
   float currentRot;
  float targetRot=AD(dist_cm);
  float speedup_stage_ratio=0.2;
  double velocity=80;
  float v_ratio=1;
  resetChassisEncoder();
  if(dist_cm>0){
    v_ratio=1;
    for(currentRot=getChassisEncoder();(currentRot)<(targetRot);currentRot=getChassisEncoder()){
      if(currentRot>=targetRot*0.2){
        v_ratio=1.0;
      }else{
        v_ratio=currentRot/(targetRot*speedup_stage_ratio);
      }

      vrun(velocity*v_ratio,velocity*v_ratio);
    }
  }else{
    v_ratio=-1;
    for(currentRot=getChassisEncoder();(currentRot)>(targetRot);currentRot=getChassisEncoder()){
      if(currentRot<=targetRot*0.2){
        v_ratio=-1.0;
      }else{
        v_ratio=currentRot/(targetRot*speedup_stage_ratio);
      }

      vrun(velocity*v_ratio,velocity*v_ratio);
    }
  }
  stopChassis();
  SLEEP(200);
}
void auto_runDistance(float dist_cm,int stop_ms=200,float v_ratio=1){
  float currentRot;
  float targetRot=AD(dist_cm);
  // float speedup_stage_ratio=0.2;
  double velocity=80;
  // float v_ratio=1;
  resetChassisEncoder();
  if(dist_cm>0){
    v_ratio=v_ratio;
    for(currentRot=getChassisEncoder();(currentRot)<(targetRot);currentRot=getChassisEncoder()){


      vrun(velocity*v_ratio,velocity*v_ratio);
    }
  }else{
    v_ratio=-v_ratio;
    for(currentRot=getChassisEncoder();(currentRot)>(targetRot);currentRot=getChassisEncoder()){

      vrun(velocity*v_ratio,velocity*v_ratio);
    }
  }
  stopChassis();
  SLEEP(stop_ms);
}
void auto_drop_front_paw(){
  VFrontPaw(VFRONT_PAW_SPEED,1)
  SLEEP(250)
  //VST(19,0)
}
void auto_drop_and_hold_side(){
  VSideArm(-100,1);
}
void auto_lift_front_paw(){
  VFrontPaw(VFRONT_PAW_SPEED,-1);
  SLEEP(600)
  // VFrontPaw(VFRONT_PAW_SPEED,0);
}
void auto_rotate_chassis(double distance){
  resetChassisEncoder();
  float currentRot=0;
  float targetRot=distance;
  float velocity=60;
  if(distance>0){
    for(;currentRot<targetRot;currentRot=getSideChassisEncoder()){
      vrun(velocity,-velocity);
    }
  }else{
    for(;currentRot>targetRot;currentRot=getSideChassisEncoder()){
      vrun(-velocity,velocity);
    }
  }
  stopChassis();
  SLEEP(200);
}
void auto_rotate_chassis_only_left_move(double distance){
  resetChassisEncoder();
  float currentRot=0;
  float targetRot=distance;
  float velocity=60;
  if(distance>0){
    for(;currentRot<targetRot;currentRot=getSideChassisEncoder()){
      vrun(velocity,-velocity/3);
    }
  }else{
    for(;currentRot>targetRot;currentRot=getSideChassisEncoder()){
      vrun(-velocity,velocity/3);
    }
  }
  stopChassis();
  SLEEP(200);
}
void auto_rotate_chassis_only_right_move(double distance){
  resetChassisEncoder();
  float currentRot=0;
  float targetRot=distance;
  float velocity=60;
  if(distance>0){
    for(;currentRot<targetRot;currentRot=getSideChassisEncoder()){
      vrun(velocity/3,-velocity);
    }
  }else{
    for(;currentRot>targetRot;currentRot=getSideChassisEncoder()){
      vrun(-velocity/3,velocity);
    }
  }
  stopChassis();
  SLEEP(200);
}
void auto_runDistance_and_take_rings(double dist_cm){
  float currentRot;
  float targetRot=AD(dist_cm);
  float speedup_stage_ratio=0.2;
  double velocity=80;
  float v_ratio=1;
  resetChassisEncoder();
  if(dist_cm>0){
    v_ratio=1;
    VFrontArm(VFRONT_ARM_SPEED,1);
    SLEEP(300);
    VFrontArm(VFRONT_ARM_SPEED,0);
    for(currentRot=getChassisEncoder();(currentRot)<(targetRot);currentRot=getChassisEncoder()){
      vrun(100*v_ratio,velocity*v_ratio);
      Vin(VIN_SPEED, 1);
    }
  }else{
    v_ratio=-1;
    for(currentRot=getChassisEncoder();(currentRot)>(targetRot);currentRot=getChassisEncoder()){
      vrun(velocity*v_ratio,velocity*v_ratio);
      Vin(-VIN_SPEED, 1);
    }
  }
  Vin(0,1);
  stopChassis();
  //SLEEP(200);
}
bool getGoal(){
  const long lim=5;
  return lineB.reflectivity()>lim;
}
void auto_runDistance_and_rotate(float dist_cm,float ratio,int stop_ms=200){
  float currentRot;
  float targetRot=AD(dist_cm);
  // float speedup_stage_ratio=0.2;
  double velocity=80;
  float v_ratio=1;
  resetChassisEncoder();
  if(dist_cm>0){
    v_ratio=1;
    for(currentRot=getChassisEncoder();(currentRot)<(targetRot);currentRot=getChassisEncoder()){


      vrun(velocity*v_ratio*ratio,velocity*v_ratio);
    }
  }else{
    v_ratio=-1;
    for(currentRot=getChassisEncoder();(currentRot)>(targetRot);currentRot=getChassisEncoder()){

      vrun(velocity*v_ratio,velocity*v_ratio*ratio);
    }
  }
  stopChassis();
  SLEEP(stop_ms);
}
void auto_drop_paw(){
  Vpaw(80,-1);
  SLEEP(2000);
}
void auto_lift_paw(){
  Vpaw(70,1);
  SLEEP(2000);
  Vpaw(0,0);
}
void autonomous(){
  float firstDistance=65;
  float backwardDistance=120; //向后多一些撞墙以矫正车位
  float getoutDistance=20;
  float getoutDistance2=35;
  float getGoalDistance=40;
  float removeDistance=40;
  float guideDistance=30;
  float getoutGetDistance=35;
  float getHomeDistance=30;
  //SLEEP(3000);
  resetChassisEncoder();
  //侧边手臂上环
  // auto_ringsStart();
  
  //Step1:get out and take the goal and get back
  
  VSideArm(-80,1);
  auto_runDistance(firstDistance,100);
  VSideArm(0,0);
  auto_drop_front_paw();
  printf("%ld\n",lineB.reflectivity());
  // SLEEP(500);
  // auto_runDistance(-secondDistance);
  auto_runDistance(-backwardDistance,1000,0.7);

  //Step2:rotate and get back to take the goal away
  
  // auto_runDistance(getoutDistance);
  auto_runDistance_and_rotate(getoutDistance2,0.2);
  // auto_rotate_chassis(-200);
  if(getGoal()){
    auto_rotate_chassis_only_right_move(-50);
    auto_lift_front_paw(); 
  }
  auto_drop_paw();
  auto_runDistance(-getGoalDistance,1000,0.7);
  auto_lift_paw();
  Vin(VIN_SPEED,1);
  VFrontArm(VFRONT_ARM_SPEED,1);
  // Vpaw(70,-1);
  SLEEP(400);
  auto_rotate_chassis(300);
  SLEEP(1000);
  Vin(VIN_SPEED,1);
  SLEEP(500);
  Vin(VIN_SPEED,-1);
  SLEEP(200);
  Vin(VIN_SPEED,1);
  auto_rotate_chassis(-100);
  auto_runDistance(20,200,0.2);
  auto_rotate_chassis(40);
  auto_runDistance(getoutGetDistance,200,0.2);
  // auto_runDistance(-removeDistance);
  auto_runDistance(-getHomeDistance);
  auto_drop_paw();
  Vin(VIN_SPEED,0);
  SLEEP(100000);
}
void manual(){
  frontpaw.resetRotation();
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
    if (BP(X))
      Vin(VIN_SPEED, 1);
    if (BP(B))
      Vin(VIN_SPEED, -1);
    if (!BP(X) && !BP(B))
      Vin(0, 1);
    if (BP(R1) && BP(R2))
      Vin(0, 1);
    //侧向套环手臂控制
    if(BP(Up)){
      VSideArm(VSIDE_ARM_SPEED,1);
    }else
    if(BP(Down)){
      //放下来
      VSideArm(VSIDE_ARM_SPEED,-1);
    }else{
      VSideArm(0,1);
    }
    //前项手臂控制
    if(BP(R1)){
      VFrontArm(VFRONT_ARM_SPEED,1);
    }else
    if(BP(R2)){
      VFrontArm(VFRONT_ARM_SPEED/2,-1);
    }
    else{
      VFrontArm(0,0);
    }
    if(BP(Y)){
      VFrontPaw(VFRONT_PAW_SPEED,1);
    }else
    if(BP(A)){
      VFrontPaw(VFRONT_PAW_SPEED,-1);
    }else{
      VFrontPaw(0,0);
    }
    printf("%lf",frontpaw.rotation(vex::rotationUnits::deg));
    //循环间延时，防止程序卡死
    SLEEP(8);
  }
}
// void getout(int k){
//   if(k==1){
//     PWM1.state(87,vex::percentUnits::pct);
//     PWM2.state(87,vex::percentUnits::pct);
//   }else
//   if(k==0){
//     PWM1.state(0,vex::percentUnits::pct);
//     PWM2.state(0,vex::percentUnits::pct);
//   }
// }
// void pneumaticsTest(){
//   while(true){
//     if(BP(A)){
//       getout(1);
//     }else
//     if(BP(B)){
//       getout(0);
//     }
//     SLEEP(8);
//   }
// }
void basicSetting(){
  frontarm1.setStopping(vex::brakeType::hold);
  frontarm2.setStopping(vex::brakeType::hold);
}
void pidRuntest(){
  const int distance=80;
  double targetRot=AD(distance);
  
}
void test_bumper(){
  while(true){
    printf("%ld\n",BumperA.pressing());
    printf("%ld\n",BumperA.value());
    SLEEP(10);
  }
}
void test_line_tracker(){
  while(true){
    SLEEP(100);
    printf("%ld\n",lineB.reflectivity());
  }
}
int main() {
  basicSetting();
  autonomous();
  //SLEEP(1000);
  // test_bumper();
  // test_line_tracker();
  // manual();
  //pneumaticsTest();
  //pidRuntest();
  return 0;
}