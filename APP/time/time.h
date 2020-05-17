#ifndef _time_H
#define _time_H

#include "system.h"
#include "CAN_Receive.h"
#include "pid.h"


//底盘3508最大can发送电流值
#define MAX_MOTOR_CAN_CURRENT 16000.0f

//m3508转化成底盘速度(m/s)的比例，做两个宏 是因为可能换电机需要更换比例
//#define M3508_MOTOR_RPM_TO_VECTOR 0.000415809748903494517209f
//#define CHASSIS_MOTOR_RPM_TO_VECTOR_SEN M3508_MOTOR_RPM_TO_VECTOR

//底盘任务控制频率，尚未使用这个宏
//#define CHASSIS_CONTROL_FREQUENCE 500.0f
//底盘电机最大速度
//#define MAX_WHEEL_SPEED 4.0f
//底盘运动过程最大前进速度
//#define NORMAL_MAX_CHASSIS_SPEED_X 3.0f

//底盘电机速度环PID
#define M3505_MOTOR1_SPEED_PID_KP 2.05f
#define M3505_MOTOR1_SPEED_PID_KI 0.03f
#define M3505_MOTOR1_SPEED_PID_KD 0.08f

#define M3505_MOTOR2_SPEED_PID_KP 2.05f
#define M3505_MOTOR2_SPEED_PID_KI 0.03f
#define M3505_MOTOR2_SPEED_PID_KD 0.30f

#define M3505_MOTOR_SPEED_PID_MAX_OUT MAX_MOTOR_CAN_CURRENT
#define M3505_MOTOR_SPEED_PID_MAX_IOUT 2000.0f

#define speed_A -0.117
#define speed_B 30


typedef struct
{
  const motor_measure_t *chassis_motor_measure;
  fp32 accel;
  fp32 speed;
  fp32 speed_set;
  int16_t give_current;
} Chassis_Motor_t;		//motor_measure_t 定义在"CAN_Receive.h"

typedef struct
{
  Chassis_Motor_t motor_chassis[2];          //底盘电机数据
  PidTypeDef motor_speed_pid[2];             //底盘电机速度pid
} chassis_move_t;

/*增量式PID算法，接口参数结构类型*/
typedef struct 
{
 /*PID算法接口变量，用于给用户获取或修改PID算法的特性*/
 float kp;     //比例系数
 float ki;     //积分系数
 float kd;     //微分系数
 
 float errNow; //当前的误差
 float dCtrOut;//控制增量输出
 float  ctrOut;//控制输出
 
 /*PID算法内部变量，其值不能修改*/
 float errOld1;
 float errOld2;
 
}PID_IncrementType;


//底盘运动数据
extern chassis_move_t chassis_move;
extern int32_t wheel_speed[2];
//底盘初始化，主要是pid初始化
extern void chassis_init(chassis_move_t *chassis_move_init);

void TIM2_Init(u16 per,u16 psc);
void TIM3_Init(u16 per,u16 psc);
void TIM4_Init(u16 per,u16 psc);
#endif


