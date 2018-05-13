#include "sys.h"
#include "usart.h"	  
#include "string.h"
////////////////////////////////////////////////////////////////////////////////// 	 
//���ʹ��ucos,����������ͷ�ļ�����.
#if SYSTEM_SUPPORT_OS
#include "includes.h"					//ucos ʹ��	  
#endif

uint8_t wifi_receive_flag;

uint16_t wifi_ack_timeout_cnt;
uint16_t wifi_ack_time_cnt;
uint16_t wifi_uart_rev_timout_cnt;

uint8_t rec_cnt = 0;

uint8_t wifi_receive_buff[WIFI_RECEIVE_BUFF_LENGTH];
uint8_t wifi_rev_tt_data[WIFI_RECEIVE_BUFF_LENGTH];
//////////////////////////////////////////////////////////////////
//�������´���,֧��printf����,������Ҫѡ��use MicroLIB	  
#if 1
#pragma import(__use_no_semihosting)             
//��׼����Ҫ��֧�ֺ���                 
struct __FILE 
{ 
	int handle; 

}; 

FILE __stdout;       
//����_sys_exit()�Ա���ʹ�ð�����ģʽ    
_sys_exit(int x) 
{ 
	x = x; 
} 
//�ض���fputc���� 
int fputc(int ch, FILE *f)
{      
	while((USART2->SR&0X40)==0);//ѭ������,ֱ���������   
    USART2->DR = (u8) ch;      
	return ch;
}
#endif 

/*ʹ��microLib�ķ���*/
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
 
#if EN_USART1_RX   //���ʹ���˽���
//����1�жϷ������
//ע��,��ȡUSARTx->SR�ܱ���Ī������Ĵ���   	
u8 USART_RX_BUF[USART_REC_LEN];     //���ջ���,���USART_REC_LEN���ֽ�.
//����״̬
//bit15��	������ɱ�־
//bit14��	���յ�0x0d
//bit13~0��	���յ�����Ч�ֽ���Ŀ
u16 USART_RX_STA=0;       //����״̬���	  

void USART1_IRQHandler(void)                	//����1�жϷ������
{		 
	uint16_t temp;
//	static uint8_t rec_cnt = 0;
	
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
	{
		USART_ClearITPendingBit(USART1,USART_IT_RXNE);
		
		temp = USART_ReceiveData(USART1);		
//		//���ճ�ʱ��ջ�������,����������ջ��棬Ϊ�յ��쳣���ݣ����
//		if((wifi_uart_rev_timout_cnt == 0) ||(rec_cnt >= WIFI_RECEIVE_BUFF_LENGTH))
//		{
//			rec_cnt = 0;
//			memset(wifi_receive_buff, 0, sizeof(wifi_receive_buff));
//		}
		//���ճ�ʱ��ջ�������
		if(wifi_uart_rev_timout_cnt == 0)
		{
			rec_cnt = 0;
			memset(wifi_receive_buff, 0, sizeof(wifi_receive_buff));
		}
		wifi_receive_buff[rec_cnt] = (char)temp;		
		rec_cnt++;
		wifi_uart_rev_timout_cnt = WIFI_UART_REV_TIMEOUT;
		//����������ջ��棬Ϊ�յ��쳣���ݣ����
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

//		//ģ���·�״̬
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
//					//������Ҫ����
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
#if SYSTEM_SUPPORT_OS 		//���SYSTEM_SUPPORT_OSΪ�棬����Ҫ֧��OS.
	OSIntEnter();    
#endif
	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)  //�����ж�(���յ������ݱ�����0x0d 0x0a��β)
	{
		Res =USART_ReceiveData(USART2);	//��ȡ���յ�������
		
		if((USART_RX_STA&0x8000)==0)//����δ���
		{
			if(USART_RX_STA&0x4000)//���յ���0x0d
			{
				if(Res!=0x0a)USART_RX_STA=0;//���մ���,���¿�ʼ
				else USART_RX_STA|=0x8000;	//��������� 
			}
			else //��û�յ�0X0D
			{	
				if(Res==0x0d) USART_RX_STA|=0x4000;
				else
				{
					USART_RX_BUF[USART_RX_STA&0X3FFF]=Res ;
					USART_RX_STA++;
					if(USART_RX_STA>(USART_REC_LEN-1))USART_RX_STA=0;//�������ݴ���,���¿�ʼ����	  
				}		 
			}
		}   		 
	} 
#if SYSTEM_SUPPORT_OS 	//���SYSTEM_SUPPORT_OSΪ�棬����Ҫ֧��OS.
	OSIntExit();  											 
#endif
}
