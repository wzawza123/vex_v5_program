#include "Autonomous.h"
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
float getSideChassisEncoder(bool isLeft){
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
  VSideArm(70,-1);
  SLEEP(1000);
  VSideArm(70,1);
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
void auto_runDistance(float dist_cm){
  float currentRot;
  float targetRot=AD(dist_cm);
  //float speedup_stage_ratio=0.2;
  double velocity=80;
  float v_ratio=1;
  resetChassisEncoder();
  if(dist_cm>0){
    v_ratio=1;
    for(currentRot=getChassisEncoder();(currentRot)<(targetRot);currentRot=getChassisEncoder()){
      /*if(currentRot>=targetRot*0.2){
        v_ratio=1.0;
      }else{
        v_ratio=currentRot/(targetRot*speedup_stage_ratio);
      }*/

      vrun(velocity*v_ratio,velocity*v_ratio);
    }
  }else{
    v_ratio=-1;
    for(currentRot=getChassisEncoder();(currentRot)>(targetRot);currentRot=getChassisEncoder()){
      /*if(currentRot>=targetRot*0.2){
        v_ratio=1.0;
      }else{
        v_ratio=currentRot/(targetRot*speedup_stage_ratio);
      }*/

      vrun(velocity*v_ratio,velocity*v_ratio);
    }
  }
  stopChassis();
  SLEEP(200);
}
void auto_drop_paw(){
  VFrontPaw(VFRONT_PAW_SPEED,1)
  SLEEP(1000)
  //VST(19,0)
}
void auto_drop_and_hold_side(){
  VSideArm(-100,1);
}
void auto_lift_paw(){
  VFrontPaw(VFRONT_PAW_SPEED,-1);
  SLEEP(200)
  VFrontPaw(VFRONT_PAW_SPEED,0);
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