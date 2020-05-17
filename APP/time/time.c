
#include "stm32f10x.h"
#include "time.h"
#include "can.h"
#include "led.h"
#include "CAN_Receive.h"
#include "math.h"
//#include "oled.h"
#include "stdlib.h"
//#include "tftlcd.h"
#include "usart.h"
#include "ps2.h"

u8 flag[2]={0,0};
s32 spdTag, spdNow, control;//����һ��Ŀ���ٶȣ������ٶȣ�������
fp32 spd=0.0f;
int32_t speed1,speed2;
int32_t wheel_speed[2] = {0, 0};
//int32_t wheel_spd_set[2]={0,0};

//�����˶�����
 chassis_move_t chassis_move;

PID_IncrementType PID_Control;//����PID�㷨�Ľṹ��
//PID�������㺯��
void PID_Move_Clear(chassis_move_t *chassis_move_clear);
//���̳�ʼ������Ҫ��pid��ʼ�� 
void chassis_init(chassis_move_t *chassis_move_init);
//�������ݸ���
void chassis_feedback_update(chassis_move_t *chassis_move_update);
//����PID�����Լ��˶��ֽ�
void chassis_control_loop(chassis_move_t *chassis_move_control_loop);
//2�ջ�
void Motor_error_equalize(chassis_move_t *motor_error_e);
void PID_IncrementMode(PID_IncrementType* PID);




void TIM2_Init(u16 per,u16 psc)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);//ʹ��TIM2ʱ��
	
	TIM_TimeBaseInitStructure.TIM_Period=per;   //�Զ�װ��ֵ
	TIM_TimeBaseInitStructure.TIM_Prescaler=psc; //��Ƶϵ��
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1;
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; //�������ϼ���ģʽ
	TIM_TimeBaseInit(TIM2,&TIM_TimeBaseInitStructure);
	
	TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE); //������ʱ���ж�
	TIM_ClearITPendingBit(TIM2,TIM_IT_Update);
	
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;//��ʱ���ж�ͨ��
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3;//��ռ���ȼ�
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;		//�����ȼ�
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	
	
}

void TIM3_Init(u16 per,u16 psc)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);//ʹ��TIM3ʱ��
	
	TIM_TimeBaseInitStructure.TIM_Period=per;   //�Զ�װ��ֵ
	TIM_TimeBaseInitStructure.TIM_Prescaler=psc; //��Ƶϵ��
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1;
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; //�������ϼ���ģʽ
	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseInitStructure);
	
	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE); //������ʱ���ж�
	TIM_ClearITPendingBit(TIM3,TIM_IT_Update);
	
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;//��ʱ���ж�ͨ��
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3;//��ռ���ȼ�
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;		//�����ȼ�
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	
	
}

void TIM4_Init(u16 per,u16 psc)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);//ʹ��TIM4ʱ��
	
	TIM_TimeBaseInitStructure.TIM_Period=per;   //�Զ�װ��ֵ
	TIM_TimeBaseInitStructure.TIM_Prescaler=psc; //��Ƶϵ��
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1;
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; //�������ϼ���ģʽ
	TIM_TimeBaseInit(TIM4,&TIM_TimeBaseInitStructure);
	
	TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE); //������ʱ���ж�
	TIM_ClearITPendingBit(TIM4,TIM_IT_Update);
	
	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;//��ʱ���ж�ͨ��
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1;//��ռ���ȼ�
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;		//�����ȼ�
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	
	
	TIM_Cmd(TIM4,ENABLE); //ʹ�ܶ�ʱ��	
}

//��ת��ʱ�ж�
//����
void TIM2_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM2,TIM_IT_Update))
	{
		if(abs(chassis_move.motor_chassis[0].speed)>abs(chassis_move.motor_chassis[0].speed_set*0.6)||
			abs(chassis_move.motor_chassis[0].speed)<abs(chassis_move.motor_chassis[0].speed_set*1.4))
			flag[0]=0;
		else 
			flag[0]=1;
	}
	TIM_ClearITPendingBit(TIM2,TIM_IT_Update);
}
//����
void TIM3_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM3,TIM_IT_Update))
	{
		led3=!led3;
		if(abs(chassis_move.motor_chassis[1].speed)>abs(chassis_move.motor_chassis[1].speed_set*0.6)||
			abs(chassis_move.motor_chassis[1].speed)<abs(chassis_move.motor_chassis[1].speed_set*1.4))
			flag[1]=0;
		else 
			flag[1]=1;
	}
	TIM_ClearITPendingBit(TIM3,TIM_IT_Update);
}


void TIM4_IRQHandler(void)
{
	u8 key=0;
	s16 speed; 
	s16 swerve; 
	if(TIM_GetITStatus(TIM4,TIM_IT_Update))
	{
		led2=!led2;
		key=PS2_DataKey();
		if(key!=0&&PS2_RedLight()==0)                   //�а�������
    {
			if(key == 11)
			{
//				printf(" %5d %5d %5d %5d\r\n",PS2_AnologData(PSS_LX),PS2_AnologData(PSS_LY),
//		                              PS2_AnologData(PSS_RX),PS2_AnologData(PSS_RY) );
//				printf(" %5d %5d\r\n",PS2_AnologData(PSS_LY),PS2_AnologData(PSS_RX));
				speed = PS2_AnologData(PSS_LY)-127;
				swerve = (PS2_AnologData(PSS_RX)-128)*25*((float)abs(speed)/128); //	speedȡ����ֵ��	�������㣬�õ�ת������
				spd =(speed_A*((PS2_AnologData(PSS_LY)-127)*(PS2_AnologData(PSS_LY)-127)))+speed_B*(PS2_AnologData(PSS_LY)-127);	   
				speed=-((int)spd);//����ǰ����  ��������
//				delay_ms(10);
//				printf(" %5d \r\n",speed);
//				printf(" %5d \r\n",swerve);
				if(speed==0&&swerve==0)
				{
					wheel_speed[0]=0;
					wheel_speed[1]=0;
					PID_Move_Clear(&chassis_move);
				}
				else
				{
					if(speed > 0) //��ǰ
					{
						if(swerve < 0)//��ת��
						{
	//						speed1 = speed + swerve;
	//						speed2 = speed;		
							wheel_speed[0]=(speed + swerve);
							wheel_speed[1]=(~speed+1);
	//						printf(" %5d %5d\r\n",wheel_speed[0],wheel_speed[1]);
						}
						else          //��ת��
						{
	//						speed1 = speed; 
	//						speed2 = speed - swerve;		
							wheel_speed[0]=(speed);
							wheel_speed[1]=(~(speed - swerve)+1);
	//						printf(" %5d %5d\r\n",wheel_speed[0],wheel_speed[1]);
						}
					}
					else if(speed < 0)//���
					{
						if(swerve < 0)//��ת��
						{
	//						speed1 = speed - swerve;
	//						speed2 = speed;	
							wheel_speed[0]=(speed - swerve);
							wheel_speed[1]=(~speed+1);
						}
						
						else//��ת��
						{
	//						speed1 = speed; 
	//						speed2 = speed + swerve;
							wheel_speed[0]=(speed);
							wheel_speed[1]=(~(speed + swerve)+1);
						}
					}
				}
			}
			else 
			{
//				delay_ms(10);
				wheel_speed[0]=0;
				wheel_speed[1]=0;
				PID_Move_Clear(&chassis_move);
//				printf("  \r\n   %d  is  light  \r\n",Data[1]);//ID
//				printf("  \r\n   %d  is  pressed  \r\n",key);
//				printf("  \r\n   %d  is  pressed  \r\n",MASK[key]);
				if(key == 12)
				{
					PS2_Vibration(0x00,0xFF);  //�����𶯺��������ʱ  delay_ms(1000);
					delay_ms(500);
				}
				else
				 PS2_Vibration(0x00,0x00); 
    	}
		}
		else
		{
			wheel_speed[0]=0;
			wheel_speed[1]=0;
			PID_Move_Clear(&chassis_move);
		}
//		printf(" %5d %5d %5d %5d\r\n",PS2_AnologData(PSS_LX),PS2_AnologData(PSS_LY),
//		                              PS2_AnologData(PSS_RX),PS2_AnologData(PSS_RY) );
		//�������ݸ���
		chassis_feedback_update(&chassis_move);
		Motor_error_equalize(&chassis_move);
		delay_ms(20);		//��ʱ�ȴ���Ϊ�˱����ֱ���������
		//���̿���PID����
		chassis_control_loop(&chassis_move);
		
		CAN_CMD_CHASSIS(chassis_move.motor_chassis[0].give_current, chassis_move.motor_chassis[1].give_current);
	}
	TIM_ClearITPendingBit(TIM4,TIM_IT_Update);
}

//PID������ʼ��
void PID_Move_Clear(chassis_move_t *chassis_move_clear)
{
	u8 i=0;
	for(i=0;i<2;i++)
		PID_clear(&chassis_move_clear->motor_speed_pid[i]);
}

void chassis_init(chassis_move_t *chassis_move_init)
{
	uint8_t i;
	//�����ٶȻ�pidֵ
	const static fp32 motor1_speed_pid[3] = {M3505_MOTOR1_SPEED_PID_KP, M3505_MOTOR1_SPEED_PID_KI, M3505_MOTOR1_SPEED_PID_KD};
	const static fp32 motor2_speed_pid[3] = {M3505_MOTOR2_SPEED_PID_KP, M3505_MOTOR2_SPEED_PID_KI, M3505_MOTOR2_SPEED_PID_KD};
	if (chassis_move_init == NULL)
	{
			return;
	}
	//��ʼ��PID 
	for (i = 0; i < 2; i++)
	{
		chassis_move_init->motor_chassis[i].chassis_motor_measure = get_Chassis_Motor_Measure_Point(i);		
		PID_clear(&chassis_move_init->motor_speed_pid[i]);
	}
	PID_Init(&chassis_move_init->motor_speed_pid[0], PID_DELTA, motor1_speed_pid, M3505_MOTOR_SPEED_PID_MAX_OUT, M3505_MOTOR_SPEED_PID_MAX_IOUT);
	PID_Init(&chassis_move_init->motor_speed_pid[1], PID_DELTA, motor2_speed_pid, M3505_MOTOR_SPEED_PID_MAX_OUT, M3505_MOTOR_SPEED_PID_MAX_IOUT);
	//����һ������
	chassis_feedback_update(chassis_move_init);
}
void chassis_feedback_update(chassis_move_t *chassis_move_update)
{
	uint8_t i = 0;
	if (chassis_move_update == NULL)
	{
		return;
	}
	for (i = 0; i < 2; i++)
	{
			//���µ���ٶȣ����ٶ����ٶȵ�PID΢��
		chassis_move_update->motor_chassis[i].speed = chassis_move_update->motor_chassis[i].chassis_motor_measure->speed_rpm;
		chassis_move_update->motor_chassis[i].accel = chassis_move_update->motor_speed_pid[i].Dbuf[0];
//		printf(" %f\r\n",chassis_move_update->motor_chassis[i].accel);
	}
}

void chassis_control_loop(chassis_move_t *chassis_move_control_loop)
{
	uint8_t i = 0;
/*************************************Ŀ���ٶ�******************************************/
//	 wheel_speed[0]=0;
//	 wheel_speed[1]=0;
//	printf(" %5d %5d\r\n",wheel_speed[0],wheel_speed[1]);
/**************************************************************************************/		
	//�������ӿ�������ٶȣ�������������ٶ�
	for (i = 0; i < 2; i++)
	{
		chassis_move_control_loop->motor_chassis[i].speed_set = wheel_speed[i];
//		temp = fabs(chassis_move_control_loop->motor_chassis[i].speed_set);
//		if (max_vector < temp)
//		{
//			max_vector = temp;
//		}
//	}
//	if (max_vector > MAX_WHEEL_SPEED)
//	{
//		vector_rate = MAX_WHEEL_SPEED / max_vector;
//		for (i = 0; i < 2; i++)
//		{
//			chassis_move_control_loop->motor_chassis[i].speed_set *= vector_rate;
//		}
	}
	/*****************��ת��ʱ��״̬�ж�********************/
		if(abs(chassis_move_control_loop->motor_chassis[0].speed)>abs(chassis_move_control_loop->motor_chassis[0].speed_set)*0.6||
			abs(chassis_move_control_loop->motor_chassis[0].speed)<abs(chassis_move_control_loop->motor_chassis[0].speed_set)*1.4)
		{
			TIM_Cmd(TIM2,DISABLE); //�رն�ʱ��2	
			flag[0]=0;
		}
		else
			TIM_Cmd(TIM2,ENABLE); //ʹ�ܶ�ʱ��2
		if(abs(chassis_move_control_loop->motor_chassis[1].speed)>abs(chassis_move_control_loop->motor_chassis[1].speed_set)*0.6||
			abs(chassis_move_control_loop->motor_chassis[1].speed)<abs(chassis_move_control_loop->motor_chassis[1].speed_set)*1.4)
		{
			TIM_Cmd(TIM3,DISABLE); //�رն�ʱ��3	
			flag[1]=0;
		}
		else
			TIM_Cmd(TIM3,ENABLE); //ʹ�ܶ�ʱ��3
	
	//����pid
	for (i = 0; i < 2; i++)
	{
		PID_Calc(&chassis_move_control_loop->motor_speed_pid[i], chassis_move_control_loop->motor_chassis[i].speed, chassis_move_control_loop->motor_chassis[i].speed_set,flag[i]);
	}
	if(wheel_speed[0]==wheel_speed[1]&&((wheel_speed[0]!=0)&&(wheel_speed[1]!=0)))
	{
		Motor_error_equalize(chassis_move_control_loop);
		if(abs(chassis_move_control_loop->motor_chassis[0].speed)>abs(chassis_move_control_loop->motor_chassis[1].speed))
		{
			chassis_move_control_loop->motor_speed_pid[1].out=chassis_move_control_loop->motor_speed_pid[1].out-control;
		}
		else if(abs(chassis_move_control_loop->motor_chassis[0].speed)<abs(chassis_move_control_loop->motor_chassis[1].speed))
		{
			chassis_move_control_loop->motor_speed_pid[0].out=chassis_move_control_loop->motor_speed_pid[0].out-control;
		}
	}
	//��ֵ����ֵ
	for (i = 0; i < 2; i++)
	{
		chassis_move_control_loop->motor_chassis[i].give_current = (int16_t)(chassis_move_control_loop->motor_speed_pid[i].out);
	}
}



void Motor_error_equalize(chassis_move_t *motor_error_e)
{
//	float dErrP, dErrI, dErrD;
//	s16 error=0;
//	u8 i=0;
//	if(wheel_speed[0]!=wheel_speed[1])
//	{
//		for(i=0;i<2;i++)
//		{
//			wheel_spd_set[i]=wheel_speed[i];
//		}
//	}
//	else if((wheel_speed[0]!=0)&&(wheel_speed[1]!=0))
//	{
//		error=motor_error_e->motor_chassis[0].speed+motor_error_e->motor_chassis[1].speed;
//		if(error>0)
//		{
//			wheel_spd_set[0]=wheel_speed[0];
//			wheel_spd_set[1]=wheel_speed[1]-error;
//		}
//		else if(error<0)
//		{
//			wheel_spd_set[0]=wheel_speed[0]-error;
//			wheel_spd_set[1]=wheel_speed[1];
//		}
//	}
//	else 
//	{
//		for(i=0;i<2;i++)
//		{
//			wheel_spd_set[i]=wheel_speed[i];
//		}
//	}
	if(abs(motor_error_e->motor_chassis[0].speed)>abs(motor_error_e->motor_chassis[1].speed))
	{
		spdTag=motor_error_e->motor_chassis[0].speed;
		spdNow=motor_error_e->motor_chassis[1].speed;
	}
	else if(abs(motor_error_e->motor_chassis[0].speed)<abs(motor_error_e->motor_chassis[1].speed))
	{
		spdTag=motor_error_e->motor_chassis[1].speed;
		spdNow=motor_error_e->motor_chassis[0].speed;
	}
	 PID_Control.errNow = spdTag + spdNow; //���㲢д���ٶ����
		
	 PID_Control.kp      = 2.5;             //д�����ϵ��Ϊ15
	 PID_Control.ki      = 0.2;              //д�����ϵ��Ϊ5
	 PID_Control.kd      = 1.5;              //д��΢��ϵ��Ϊ5

	 PID_IncrementMode(&PID_Control);       //ִ�о���ʽPID�㷨
//	 if(PID->kp < 0)    PID->kp = -PID->kp;
//	 if(PID->ki < 0)	PID->ki = -PID->ki;
//	 if(PID->kd < 0)    PID->kd = -PID->kd;

//	 dErrP = PID->errNow - PID->errOld1;

//	 dErrI = PID->errNow;

//	 dErrD = PID->errNow - 2 * PID->errOld1 + PID->errOld2;

//	 PID->errOld2 = PID->errOld1; //�������΢��
//	 PID->errOld1 = PID->errNow;  //һ�����΢��

//	 /*����ʽPID����*/
//	 PID->dCtrOut = PID->kp * dErrP + PID->ki * dErrI + PID->kd * dErrD;
//	 
//	 if(PID->kp == 0 && PID->ki == 0 && PID->kd == 0)   PID->ctrOut = 0;

//	 else PID->ctrOut += PID->dCtrOut;

	 control = PID_Control.ctrOut;         //��ȡ����ֵ

}

void PID_IncrementMode(PID_IncrementType* PID)
{
 float dErrP, dErrI, dErrD;
 
 if(PID->kp < 0)    PID->kp = -PID->kp;
 if(PID->ki < 0)	PID->ki = -PID->ki;
 if(PID->kd < 0)    PID->kd = -PID->kd;

 dErrP = PID->errNow - PID->errOld1;

 dErrI = PID->errNow;

 dErrD = PID->errNow - 2 * PID->errOld1 + PID->errOld2;

 PID->errOld2 = PID->errOld1; //�������΢��
 PID->errOld1 = PID->errNow;  //һ�����΢��

 /*����ʽPID����*/
 PID->dCtrOut = PID->kp * dErrP + PID->ki * dErrI + PID->kd * dErrD;
 
 if(PID->kp == 0 && PID->ki == 0 && PID->kd == 0)   PID->ctrOut = 0;

 else PID->ctrOut += PID->dCtrOut;
}

