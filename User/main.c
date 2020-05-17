/**********************************************************
*Project Name:chassis motor control(Demon)
*Task:				M3508 control
*							PID	calculate
*							PS2 control
*Author:			Huang.F.Y.
*Update:			PS2 debug	
*Date:				2019.8.15
************************************************************
***************************�ܽ�ͼ***************************
											STM32F103C8T6
PA9	->USART1 RX					PC16->led3(time3)
PA10->USART1 TX					PB12->PS2 DI(DAT)
PA11->CAN1 RX						PB13->PS2 DO(CMD)
PA12->CAN1 TX						PB14->PS2 CS
PC14->led1(main)				PB15->PS2 CLK
PC15->led2(time4)
************************************************************/

#include "SysTick.h"
#include "system.h"
#include "led.h"
#include "pid.h"
#include "can.h"
#include "usart.h"
#include "time.h"
#include "oled.h"
#include "main.h"
#include "stdlib.h"
#include "ps2.h"


void All_Init(void);

int main()
{
	u8 i=0;  
	All_Init();
	
	while(1)
	{
		i++;
		if(i%20==0)
		{
			led1=!led1;
		}
		delay_ms(10);
	}
}


void All_Init()
{
	SysTick_Init(72);
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_3);  //�ж����ȼ����� ��3��
//	OLED_Init();			//��ʼ��OLED 
	LED_Init();
	PS2_Init();					//PS2�ܽų�ʼ��
	PS2_SetInit();		 //PS2���ó�ʼ��,���á����̵�ģʽ������ѡ���Ƿ�����޸�
	                   //������ģʽ
	CAN1_Mode_Init(CAN_SJW_1tq,CAN_BS2_8tq,CAN_BS1_9tq,2,CAN_Mode_Normal);	//1Mbps������
	USART1_Init(9600);		//����1��ʼ��
//	TFTLCD_Init();			//LCD��ʼ��
//	FRONT_COLOR=BLACK;	//��LCD��ʾ��������ɫ��ʼ��
	chassis_init(&chassis_move);//���̳�ʼ��
	delay_ms(50);
	TIM2_Init(3000,36000-1);//1.5�붨ʱ
	TIM3_Init(3000,36000-1);//1.5�붨ʱ
	TIM4_Init(2000,72-1);  //��ʱ4ms
	delay_ms(20);		//�ȴ���ʼ��
}
