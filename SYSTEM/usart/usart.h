#ifndef __USART_H
#define __USART_H
#include "stdio.h"	
#include "sys.h" 

#define USART_REC_LEN  			200  	//定义最大接收字节数 200
#define EN_USART1_RX 			1		//使能（1）/禁止（0）串口1接收

#define WIFI_RECEIVE_BUFF_LENGTH				256
#define WIFI_STATUS_LENGTH							8
#define WIFI_TT_REV_FARME_LENGTH				8

#define WIFI_ACK_TIMEOUT			  			10000
#define WIFI_ACK_TIMELIMIT_SHORT			2000	
#define WIFI_ACK_TIMELIMIT_LONG				5000	
#define WIFI_UART_REV_TIMEOUT   			50
	  	
extern u8  USART_RX_BUF[USART_REC_LEN]; //接收缓冲,最大USART_REC_LEN个字节.末字节为换行符 
extern u16 USART_RX_STA;         		//接收状态标记	
//如果想串口中断接收，请不要注释以下宏定义
void UART2_Init(u32 bound);
#endif


