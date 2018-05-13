#ifndef __USART_H
#define __USART_H
#include "stdio.h"	
#include "sys.h" 

#define USART_REC_LEN  			200  	//�����������ֽ��� 200
#define EN_USART1_RX 			1		//ʹ�ܣ�1��/��ֹ��0������1����

#define WIFI_RECEIVE_BUFF_LENGTH				256
#define WIFI_STATUS_LENGTH							8
#define WIFI_TT_REV_FARME_LENGTH				8

#define WIFI_ACK_TIMEOUT			  			10000
#define WIFI_ACK_TIMELIMIT_SHORT			2000	
#define WIFI_ACK_TIMELIMIT_LONG				5000	
#define WIFI_UART_REV_TIMEOUT   			50
	  	
extern u8  USART_RX_BUF[USART_REC_LEN]; //���ջ���,���USART_REC_LEN���ֽ�.ĩ�ֽ�Ϊ���з� 
extern u16 USART_RX_STA;         		//����״̬���	
//����봮���жϽ��գ��벻Ҫע�����º궨��
void UART2_Init(u32 bound);
#endif


