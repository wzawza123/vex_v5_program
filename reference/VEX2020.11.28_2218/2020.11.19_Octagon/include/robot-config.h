#define ChassisLFOffsetInitial 1.00
#define ChassisLBOffsetInitial 1.00
#define ChassisRFOffsetInitial 1.00
#define ChassisRBOffsetInitial 1.00

#define EnableAutoChassisAdjust 1
#define BallColorDetectionEWM 0.8
#define GyroDifSumEWM 0.95
#define ChassisControllerMode 'l'
#define AxisThreshold 0

//前四项为底盘校准值

//是否启用自动校准  0不使用     1使用（必须使用陀螺仪）    2王者荣耀模式（必须使用陀螺仪）
//球颜色检测的平滑移动gamma值
//陀螺仪差分平均的平滑移动gamma值
//底盘控制模式，目前无用
//操控转轴角度阈值，超过这个值的拖动遥感才会被检测