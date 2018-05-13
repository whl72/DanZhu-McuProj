#include "sys.h"
#include "usart.h"	  
#include "string.h"
////////////////////////////////////////////////////////////////////////////////// 	 
//如果使用ucos,则包括下面的头文件即可.
#if SYSTEM_SUPPORT_OS
#include "includes.h"					//ucos 使用	  
#endif

uint8_t wifi_receive_flag;

uint16_t wifi_ack_timeout_cnt;
uint16_t wifi_ack_time_cnt;
uint16_t wifi_uart_rev_timout_cnt;

uint8_t rec_cnt = 0;

uint8_t wifi_receive_buff[WIFI_RECEIVE_BUFF_LENGTH];
uint8_t wifi_rev_tt_data[WIFI_RECEIVE_BUFF_LENGTH];
//////////////////////////////////////////////////////////////////
//加入以下代码,支持printf函数,而不需要选择use MicroLIB	  
#if 1
#pragma import(__use_no_semihosting)             
//标准库需要的支持函数                 
struct __FILE 
{ 
	int handle; 

}; 

FILE __stdout;       
//定义_sys_exit()以避免使用半主机模式    
_sys_exit(int x) 
{ 
	x = x; 
} 
//重定义fputc函数 
int fputc(int ch, FILE *f)
{      
	while((USART2->SR&0X40)==0);//循环发送,直到发送完毕   
    USART2->DR = (u8) ch;      
	return ch;
}
#endif 

/*使用microLib的方法*/
 /* 
int fputc(int ch, FILE *f)
{
	USART_SendData(USART1, (uint8_t) ch);

	while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET) {}	
   
    return ch;
}
int GetKey (void)  { 

    while (!(USART1->SR & USART_FLAG_RXNE));

    return ((int)(USART1->DR & 0x1FF));
}
*/
 
#if EN_USART1_RX   //如果使能了接收
//串口1中断服务程序
//注意,读取USARTx->SR能避免莫名其妙的错误   	
u8 USART_RX_BUF[USART_REC_LEN];     //接收缓冲,最大USART_REC_LEN个字节.
//接收状态
//bit15，	接收完成标志
//bit14，	接收到0x0d
//bit13~0，	接收到的有效字节数目
u16 USART_RX_STA=0;       //接收状态标记	  

void USART1_IRQHandler(void)                	//串口1中断服务程序
{		 
	uint16_t temp;
//	static uint8_t rec_cnt = 0;
	
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
	{
		USART_ClearITPendingBit(USART1,USART_IT_RXNE);
		
		temp = USART_ReceiveData(USART1);		
//		//接收超时清空缓存数据,如果超过接收缓存，为收到异常数据，清空
//		if((wifi_uart_rev_timout_cnt == 0) ||(rec_cnt >= WIFI_RECEIVE_BUFF_LENGTH))
//		{
//			rec_cnt = 0;
//			memset(wifi_receive_buff, 0, sizeof(wifi_receive_buff));
//		}
		//接收超时清空缓存数据
		if(wifi_uart_rev_timout_cnt == 0)
		{
			rec_cnt = 0;
			memset(wifi_receive_buff, 0, sizeof(wifi_receive_buff));
		}
		wifi_receive_buff[rec_cnt] = (char)temp;		
		rec_cnt++;
		wifi_uart_rev_timout_cnt = WIFI_UART_REV_TIMEOUT;
		//如果超过接收缓存，为收到异常数据，清空
		if(rec_cnt >= WIFI_RECEIVE_BUFF_LENGTH)
		{
			rec_cnt = 0;
			memset(wifi_receive_buff, 0, sizeof(wifi_receive_buff));
		}
		
		if((wifi_receive_buff[rec_cnt-1] == '\n')&&(wifi_receive_buff[rec_cnt-2] == '\r'))
		{
			rec_cnt = 0;
			wifi_receive_flag = 1;
		}

//		//模块下发状态
//		if((wifi_receive_buff[0] == 'S')&&(wifi_receive_buff[1] == 'T') &&
//			 (wifi_receive_buff[2] == 'A')&&(wifi_receive_buff[3] == 'T'))
//		{
//			if(rec_cnt == WIFI_STATUS_LENGTH)
//			{
//				//memcpy(&wifi_rev_tt_data, wifi_receive_buff, WIFI_TT_REV_FARME_LENGTH);
//				memset(&wifi_rev_tt_data, 0, sizeof(wifi_rev_tt_data));
//				memcpy(&wifi_rev_tt_data, wifi_receive_buff, strlen((const char*)wifi_receive_buff));
//				wifi_receive_flag = 1;
//				rec_cnt = 0;
//			}
//			return;
//		}
//		if(rec_cnt == WIFI_TT_REV_FARME_LENGTH)
//		{ 
//			memcpy(&wifi_rev_tt_data, wifi_receive_buff, WIFI_TT_REV_FARME_LENGTH);
//			wifi_receive_flag = 1;
//			rec_cnt = 0;
//		}
//		else if((wifi_receive_buff[rec_cnt-1] == '\n')&&(wifi_receive_buff[rec_cnt-2] == '\r') &&
//			 (wifi_receive_buff[rec_cnt-3] == 'K')&&(wifi_receive_buff[rec_cnt-4] == 'O'))
//		{
//			memcpy(&wifi_rev_tt_data, "OK\r\n", 4);
//			wifi_receive_flag = 1;
//			rec_cnt = 0;
//		}
//		switch(wifi_com_mode_flag)
//		{
//			case idle_mode:
//				break;
//			case tt_data_mode:
//				if(rec_cnt == WIFI_TT_REV_FARME_LENGTH)
//				{ 
//					memcpy(&wifi_rev_tt_data, wifi_receive_buff, WIFI_TT_REV_FARME_LENGTH);
//					wifi_receive_flag = 1;
//					rec_cnt = 0;
//				}
//				else if((wifi_receive_buff[rec_cnt-1] == '\n')&&(wifi_receive_buff[rec_cnt-2] == '\r') &&
//					 (wifi_receive_buff[rec_cnt-3] == 'K')&&(wifi_receive_buff[rec_cnt-4] == 'O'))
//				{
//					memcpy(&wifi_rev_tt_data, "OK\r\n", 4);
//					wifi_receive_flag = 1;
//					rec_cnt = 0;
//				}
//				break;
//			case update_fw_mode:
//				if(rec_cnt == WIFI_UPDATE_REV_FRAME_LENGTH)
//				{
//					memcpy(&wifi_updata_data, wifi_receive_buff, WIFI_UPDATE_REV_FRAME_LENGTH);
//					//memset(wifi_receive_buff, 0, sizeof(wifi_receive_buff));
//					wifi_receive_flag = 1;
//					rec_cnt = 0;
//				}
//				else if((wifi_receive_buff[rec_cnt-1] == '\n')&&(wifi_receive_buff[rec_cnt-2] == '\r') &&
//					 (wifi_receive_buff[rec_cnt-3] == 'K')&&(wifi_receive_buff[rec_cnt-4] == 'O'))
//				{
//					memcpy(&wifi_rev_tt_data, "OK\r\n", 4);
//					wifi_receive_flag = 1;
//					rec_cnt = 0;
//				}
//				else if(strstr((const char*)wifi_receive_buff, "ERROR") != NULL)
//				{
//					//这里需要重启
////					memcpy(&wifi_rev_tt_data, "ERROR\r\n", 4);
////					wifi_receive_flag = 1;
////					rec_cnt = 0;
//				}
//				break;
//			default:
//				rec_cnt = 0;
//				break;
//		}
	}
} 
#endif	

void USART2_IRQHandler(void)
{
	u8 Res;
#if SYSTEM_SUPPORT_OS 		//如果SYSTEM_SUPPORT_OS为真，则需要支持OS.
	OSIntEnter();    
#endif
	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)  //接收中断(接收到的数据必须是0x0d 0x0a结尾)
	{
		Res =USART_ReceiveData(USART2);	//读取接收到的数据
		
		if((USART_RX_STA&0x8000)==0)//接收未完成
		{
			if(USART_RX_STA&0x4000)//接收到了0x0d
			{
				if(Res!=0x0a)USART_RX_STA=0;//接收错误,重新开始
				else USART_RX_STA|=0x8000;	//接收完成了 
			}
			else //还没收到0X0D
			{	
				if(Res==0x0d) USART_RX_STA|=0x4000;
				else
				{
					USART_RX_BUF[USART_RX_STA&0X3FFF]=Res ;
					USART_RX_STA++;
					if(USART_RX_STA>(USART_REC_LEN-1))USART_RX_STA=0;//接收数据错误,重新开始接收	  
				}		 
			}
		}   		 
	} 
#if SYSTEM_SUPPORT_OS 	//如果SYSTEM_SUPPORT_OS为真，则需要支持OS.
	OSIntExit();  											 
#endif
}
