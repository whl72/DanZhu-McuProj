#include "timer.h"
#include "led.h"
#include "esp8266.h"
#include "iocontrol.h"
  	 

volatile unsigned short sys_SysTickFlags = 0;
extern scoket_power_status_t scoket_power_status;

//ͨ�ö�ʱ��3�жϳ�ʼ��
//����ʱ��ѡ��ΪAPB1��2������APB1Ϊ36M
//arr���Զ���װֵ��
//psc��ʱ��Ԥ��Ƶ��
//����ʹ�õ��Ƕ�ʱ��3!
void TIM3_Int_Init(u16 arr,u16 psc)
{
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); //ʱ��ʹ��
	
	//��ʱ��TIM3��ʼ��
	TIM_TimeBaseStructure.TIM_Period = arr; //��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ	
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //����ʱ�ӷָ�:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM���ϼ���ģʽ
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure); //����ָ���Ĳ�����ʼ��TIMx��ʱ�������λ
 
	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE ); //ʹ��ָ����TIM3�ж�,��������ж�

	//�ж����ȼ�NVIC����
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;  //TIM3�ж�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  //��ռ���ȼ�0��
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;  //�����ȼ�3��
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQͨ����ʹ��
	NVIC_Init(&NVIC_InitStructure);  //��ʼ��NVIC�Ĵ���


	TIM_Cmd(TIM3, ENABLE);  //ʹ��TIMx					 
}


//��ʱ��3�жϷ������
void TIM3_IRQHandler(void)   //TIM3�ж�
{	
	static unsigned int SysTick_cnt = 0;
	
	SysTick_cnt++;
	if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)  //���TIM3�����жϷ������
	{
		TIM_ClearITPendingBit(TIM3, TIM_IT_Update  );  //���TIMx�����жϱ�־ 
		
		
		sys_SysTickFlags |= SysTickFlag_1ms;

	//+++++++ every 3 ms ++++++++++++++++++++++++++++++++++++++
		if((SysTick_cnt % 3) == 0)
		{
			sys_SysTickFlags |= SysTickFlag_3ms;
		}
	//+++++++ every 5 ms ++++++++++++++++++++++++++++++++++++++
		if((SysTick_cnt % 5) == 0)
		{
			sys_SysTickFlags |= SysTickFlag_5ms;
		}
	//+++++++ every 10 ms +++++++++++++++++++++++++++++++++++++
		if((SysTick_cnt % 10) == 0)
		{
			sys_SysTickFlags |= SysTickFlag_10ms;
		}
	//+++++++ every 25 ms +++++++++++++++++++++++++++++++++++++
		if((SysTick_cnt % 25) == 0)
		{
			sys_SysTickFlags |= SysTickFlag_25ms;
		}

		//+++++++ every 25 ms +++++++++++++++++++++++++++++++++++++
		if((SysTick_cnt % 50) == 0)
		{
			sys_SysTickFlags |= SysTickFlag_50ms;
		}

	//+++++++ every 100 ms ++++++++++++++++++++++++++++++++++++
		if((SysTick_cnt % 100) == 0)
		{
			sys_SysTickFlags |= SysTickFlag_100ms;
		}

		if((SysTick_cnt % 333) == 0)
		{
			sys_SysTickFlags |= SysTickFlag_333ms;
		}

		if((SysTick_cnt % 500) == 0)
		{
			sys_SysTickFlags |= SysTickFlag_500ms;
		}

		if((SysTick_cnt % 1000) == 0)
		{
			scoket_power_status.ts_val++;
			sys_SysTickFlags |= SysTickFlag_1000ms;
		}
		
		if((SysTick_cnt % 3000) == 0)
		{
			sys_SysTickFlags |= SysTickFlag_3000ms;
		}	
		
		if((SysTick_cnt % 5500) == 0)
		{
			sys_SysTickFlags |= SysTickFlag_5500ms;
		}
		
		if((SysTick_cnt % 60000) == 0)
		{
			sys_SysTickFlags |= SysTickFlag_1min; 
			SysTick_cnt = 0;
		}
		
//		if(timer_counter >1000)
//		{
//			timer_counter = 0;
//			LED1=!LED1; //������ ռ��8266��rst����
//		}
	}
}












