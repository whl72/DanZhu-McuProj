#include "led.h"
#include "delay.h"
#include "key.h"
#include "sys.h"
#include "usart.h"	
#include "esp8266.h"
#include "RN8209.h"
#include "timer.h"
#include "string.h"
#include "iocontrol.h"

//#define uint8_t 	unsigned char
//#define uint16_t	unsigned int
//#define uint32_t	unsigned long

//uart bound
#define UART1_BOUND		115200
#define UART2_BOUND		115200

//RN8209 spi hardware interface
#define	RN_SPI_MISO_PIN		GPIO_Pin_15
#define RN_SPI_MOSI_PIN		GPIO_Pin_13
#define RN_SPI_CLK_PIN		GPIO_Pin_14
#define RN_SPI_CS_PIN			GPIO_Pin_12
#define RN_SPI_PORT				GPIOB

//id hardware interface
#define ID_QH_PIN		GPIO_Pin_7
#define ID_CLK_PIN	GPIO_Pin_8
#define ID_SHLD_PIN	GPIO_Pin_9
#define ID_PORT			GPIOB

#define SET_ID_SHLD_HIGH()		GPIO_SetBits(ID_PORT, ID_SHLD_PIN)
#define SET_ID_SHLD_LOW()		GPIO_ResetBits(ID_PORT, ID_SHLD_PIN)
#define SET_ID_CLK_HIGH()		GPIO_SetBits(ID_PORT, ID_CLK_PIN)
#define SET_ID_CLK_LOW()		GPIO_ResetBits(ID_PORT, ID_CLK_PIN)
#define GET_ID_QH_DATA()		GPIO_ReadInputDataBit(ID_PORT, ID_QH_PIN)

//jumper hardware interface
#define JUMPER_PIN		GPIO_Pin_15

//iic hardware interface
#define NFC_IIC_SCL		GPIO_Pin_10
#define NFC_IIC_SDA		GPIO_Pin_11

scoket_hw_status_t scoket_hw_status;
scoket_power_status_t scoket_power_status;
socket_reboot_t  socket_reboot;

uint16_t Device_ID;

extern unsigned int      voltage;
extern unsigned int      Freq;
extern unsigned int      current;
extern unsigned int      power;

extern volatile unsigned short sys_SysTickFlags;
extern uint8_t wifi_receive_flag;

extern uint8_t wifi_receive_buff[WIFI_RECEIVE_BUFF_LENGTH];

void HW_GPIO_Init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOC, ENABLE);	 //enable port clock GPIOB GPIOA

	GPIO_InitStructure.GPIO_Pin = RN_SPI_MISO_PIN|RN_SPI_MOSI_PIN|RN_SPI_CLK_PIN|RN_SPI_CS_PIN|RELAY_CONTROL_PIN;		 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = BUZZER_CONTROL_PIN;	    		 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	
	GPIO_Init(BUZZZER_PORT, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = ESP_RST_PIN;	    		 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = ID_QH_PIN;			
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; 	
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = JUMPER_PIN;			
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; 	
	GPIO_Init(GPIOA, &GPIO_InitStructure);	  				 
}

void UART1_Init(uint32_t bound)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1|RCC_APB2Periph_GPIOA, ENABLE);	//使能USART1，GPIOA时钟
  
	//USART1_TX   GPIOA.9
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; //PA.9
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//复用推挽输出
  GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIOA.9
   
  //USART1_RX	  GPIOA.10
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;//PA10
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//浮空输入
  GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIOA.10  

  //Usart1 NVIC 配置
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0 ;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;		//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器
  
	USART_InitStructure.USART_BaudRate = bound;//串口波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式

  USART_Init(USART1, &USART_InitStructure); //初始化串口1
  USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//开启串口接受中断
  USART_Cmd(USART1, ENABLE);                    //使能串口1 
}

void UART2_Init(u32 bound)
{
  //GPIO端口设置
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	 
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);	//使能USART2时钟
  
	//USART1_TX   GPIOA.9
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2; //PA.9
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//复用推挽输出
  GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIOA.9
   
  //USART1_RX	  GPIOA.10初始化
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;//PA10
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//浮空输入
  GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIOA.10  

  //Usart1 NVIC 配置
  NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3 ;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;		//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器
  
   //USART 初始化设置

	USART_InitStructure.USART_BaudRate = bound;//串口波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式

  USART_Init(USART2, &USART_InitStructure); //初始化串口2
  USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);//开启串口接受中断
  USART_Cmd(USART2, ENABLE);                    //使能串口2

}

void IIC_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(	RCC_APB2Periph_GPIOB, ENABLE );	//使能GPIOB时钟
	   
	GPIO_InitStructure.GPIO_Pin = NFC_IIC_SCL|NFC_IIC_SDA;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP ;   //推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_SetBits(GPIOB,NFC_IIC_SCL|NFC_IIC_SDA); 	//PB6,PB7 输出高
}

uint8_t Get_Jumper_Status(void)
{
	return GPIO_ReadInputDataBit(GPIOA,JUMPER_PIN);
}

uint16_t Get_ID(void)
{
	unsigned int val = 0;
	unsigned int count = 16;        
	unsigned int i, tmp;

	SET_ID_SHLD_HIGH();
	delay_ms(2);
	SET_ID_SHLD_LOW();
	delay_ms(2);
	SET_ID_SHLD_HIGH();
	delay_ms(2);
	SET_ID_CLK_HIGH();
	delay_ms(2);

	if (GET_ID_QH_DATA())
		val += 1;

	for (i = 0; i < count - 1; i++) 
	{
		SET_ID_CLK_LOW();
		delay_ms(2);
		SET_ID_CLK_HIGH();
		delay_ms(2);
		val <<= 1;
		if (GET_ID_QH_DATA())
				val += 1;
	}

	SET_ID_CLK_HIGH();
	SET_ID_SHLD_HIGH();

	tmp = ((val & 0xff00) >> 8 | (val & 0xff00));

	return tmp;
}

int main(void)
{	    
	SystemInit();
	delay_init();	    	 
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//设置中断优先级分组为组2：2位抢占优先级，2位响应优先级
	HW_GPIO_Init();
	UART1_Init(UART1_BOUND);
#ifdef DEBUG_ON
	UART2_Init(UART2_BOUND);	 	
#endif
	IIC_Init();
	//TIM3_Int_Init(4999,7199);//10Khz的计数频率，计数到5000为500ms
	TIM3_Int_Init(9,7199);//10Khz的计数频率，计数到5000为500ms
	
	initWifi();
	
	InitMeasure();
	scoket_power_status.ts_val = (uint32_t)1525607842;
	
//	while(1)
//	{
//		Device_ID = Get_ID();
//		if(Device_ID != 0) break;
//	}
#ifdef DEBUG_ON
	printf("Get ID success !! Device ID = :%d\r\n",Device_ID);
#endif
	if(Get_Jumper_Status())
		scoket_hw_status.sw_stat = 1;
	else
		scoket_hw_status.sw_stat = 0;
	setRelay(scoket_hw_status.sw_stat);
#ifdef DEBUG_ON
	printf("switch status = :%d\r\n",Get_Jumper_Status());
#endif
	
	while(1)
	{
		if((sys_SysTickFlags & SysTickFlag_1ms) != 0)
		{
			if(wifi_receive_flag)
			{
				handleWifiReciveData(wifi_receive_buff);
				memset(wifi_receive_buff, 0, sizeof(wifi_receive_buff));
				wifi_receive_flag = 0;
			}
			handle_1ms_iocontrol();
			sys_SysTickFlags &= ~SysTickFlag_1ms;
		}
		if((sys_SysTickFlags & SysTickFlag_5ms) != 0)
		{
			sendToWifi();
			sys_SysTickFlags &= ~SysTickFlag_5ms;
		}
		if((sys_SysTickFlags & SysTickFlag_10ms) != 0)
		{
			exeIOEvent();
			sys_SysTickFlags &= ~SysTickFlag_10ms;
		}
		if((sys_SysTickFlags & SysTickFlag_1000ms) != 0)
		{
//			//测试打印
			LED1=!LED1; //测试用 占了8266的rst引脚
//			get_Measur_Data(scoket_power_status);
//			sendDeviceStatus(scoket_power_status);
//			sendDeviceID(0xf003);
//			setEspTTOn();
//			setEspTTOff();
			
			send_1s_power_status();
			
			sys_SysTickFlags &= ~SysTickFlag_1000ms;
		}
		
		
//		delay_ms(1000);
//#ifdef DEBUG_ON
//		Cal_Measur_Rms(0);		             //频率
//		Cal_Measur_Rms(1);			           //电压
//		Cal_Measur_Rms(2);			           //电流
//		Cal_Measur_Rms(3);			           //功率
//		printf("voltage = :%d\r\n",voltage);
//		printf("Freq = :%d\r\n",Freq);
//		printf("current = :%d\r\n",current);
//		printf("power = :%d\r\n\r\n",power);
//#endif
	}
}

