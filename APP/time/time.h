#ifndef _time_H
#define _time_H

#include "system.h"
#include "CAN_Receive.h"
#include "pid.h"


//����3508���can���͵���ֵ
#define MAX_MOTOR_CAN_CURRENT 16000.0f

//m3508ת���ɵ����ٶ�(m/s)�ı������������� ����Ϊ���ܻ������Ҫ��������
//#define M3508_MOTOR_RPM_TO_VECTOR 0.000415809748903494517209f
//#define CHASSIS_MOTOR_RPM_TO_VECTOR_SEN M3508_MOTOR_RPM_TO_VECTOR

//�����������Ƶ�ʣ���δʹ�������
//#define CHASSIS_CONTROL_FREQUENCE 500.0f
//���̵������ٶ�
//#define MAX_WHEEL_SPEED 4.0f
//�����˶��������ǰ���ٶ�
//#define NORMAL_MAX_CHASSIS_SPEED_X 3.0f

//���̵���ٶȻ�PID
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
} Chassis_Motor_t;		//motor_measure_t ������"CAN_Receive.h"

typedef struct
{
  Chassis_Motor_t motor_chassis[2];          //���̵������
  PidTypeDef motor_speed_pid[2];             //���̵���ٶ�pid
} chassis_move_t;

/*����ʽPID�㷨���ӿڲ����ṹ����*/
typedef struct 
{
 /*PID�㷨�ӿڱ��������ڸ��û���ȡ���޸�PID�㷨������*/
 float kp;     //����ϵ��
 float ki;     //����ϵ��
 float kd;     //΢��ϵ��
 
 float errNow; //��ǰ�����
 float dCtrOut;//�����������
 float  ctrOut;//�������
 
 /*PID�㷨�ڲ���������ֵ�����޸�*/
 float errOld1;
 float errOld2;
 
}PID_IncrementType;


//�����˶�����
extern chassis_move_t chassis_move;
extern int32_t wheel_speed[2];
//���̳�ʼ������Ҫ��pid��ʼ��
extern void chassis_init(chassis_move_t *chassis_move_init);

void TIM2_Init(u16 per,u16 psc);
void TIM3_Init(u16 per,u16 psc);
void TIM4_Init(u16 per,u16 psc);
#endif


