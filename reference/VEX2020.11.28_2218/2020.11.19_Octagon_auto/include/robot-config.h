#define ChassisLFOffsetInitial 1.0
#define ChassisLBOffsetInitial 1.0
#define ChassisRFOffsetInitial 1.0
#define ChassisRBOffsetInitial 1.0

#define EnableAutoChassisAdjust 1
#define BallColorDetectionEWM 0.8
#define GyroDifSumEWM 0.9

#define AxisThreshold 30

#define PID_turn_Kp 0.006
#define PID_turn_Ki 0.00008
#define PID_turn_Kd 0.08
#define PID_ASameThreshold 2
#define PID_move_Kp 0.01
#define PID_move_Ki 0
#define PID_move_Kd 0.0
#define PID_VSameThreshold 1

#define PID_TimeBlocks 50
#define PID_TimeMax 400
#define PID_turn_r_thres 0.065
#define PID_move_f_thres 0.08
#define PID_move_r_thres 0.08
//前四项为底盘校准值

//是否启用自动校准  0不使用     1使用（必须使用陀螺仪）    2王者荣耀模式（必须使用陀螺仪）
//球颜色检测的平滑移动gamma值
//陀螺仪差分平均的平滑移动gamma值

//操控转轴角度阈值，超过这个值的拖动遥感才会被检测