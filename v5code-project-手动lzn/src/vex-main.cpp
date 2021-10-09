#include "vex.h"
#define GTD(a) ((double)(1811 - a) / 8.16833)
#define GDEG GTD(gyr.value(vex::analogUnits::range12bit))
#define LO(a) a.largestObject
#define ERR(a, b, c) ((a > b + c) || (a < b - c))

const int VIN_SPEED = 80;  //吸环电机速度
const int VARM_SPEED = 65; //侧向手臂电机速度

void VRUN(double l, double r) {
  VVST(1, 2, l)
  VVST(3, 4, l)
  VVST(8, 10, r)
  VVST(6, 9, r)
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
void Varm(double v, int temp) {
  int V = v * temp;
  VST(16, V * VARM_SPEED);
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
int main() {
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
      Varm(VARM_SPEED,1);
    }else
    if(BP(A)){
      Varm(VARM_SPEED,-1);
    }else{
      Varm(0,1);
    }
    //循环间延时，防止程序卡死
    SLEEP(8);
  }
  return 0;
}
