
#include "esp8266.h"
#include "string.h"
#include "sys.h"
#include "stdlib.h"
#include "stdio.h"
#include "delay.h"
#include "RN8209.h"
#include "usart.h"
#include "iocontrol.h"


//WIFI_com_mode_t wifi_com_mode_flag;
WIFI_status_t wifi_work_status;
uint8_t wifi_com_status_flag;
WIFI_com_status_t wifi_send_status;

char wifi_send_buff[256];
uint8_t nfc_num[5] = {0};

extern scoket_hw_status_t scoket_hw_status;
extern scoket_power_status_t scoket_power_status;
extern socket_reboot_t  socket_reboot;
extern uint8_t wifi_receive_flag;
extern io_control_flag_t io_control_flag;

extern uint8_t wifi_receive_buff[WIFI_RECEIVE_BUFF_LENGTH];

void initWifi(void)
{
	ESP_RST_LOW();
	delay_ms(1000);
	ESP_RST_HIGH();
	
	//test here
	wifi_work_status = wifi_stat_tt;
}

void sendDeviceStatus(scoket_power_status_t power_status)
{
//	char buff[256] = {0};
	char temp1[10];
	char temp2[10];
	char temp3[10];
	char temp4[20];
	
	sprintf(temp1, "%.2f", power_status.current_val);
	sprintf(temp2, "%.2f", power_status.voltate_val);
	sprintf(temp3, "%.2f", power_status.watt_val);
	sprintf(temp4, "%d", power_status.ts_val);
	
	memset(wifi_send_buff, 0 , sizeof(wifi_send_buff));
	sprintf(wifi_send_buff, "{\"Name\":\"ReportRealtimeLoad\",\"Current\":\"%s\",\"Voltage\":\"%s\",\"Watt\":\"%s\",\"Timestamp\":\"%s\"}",
	        temp1, temp2, temp3, temp4);
	
#ifdef DEBUG_ON
	printf("1 min update status :%s\r\n",wifi_send_buff);
#endif
}

void sendDeviceID(uint16_t id_number)
{
//	char buff[32] = {0};
	char temp1[10] = {0};
	uint8_t num1,num2;
	
	num1 = (id_number >>8) & 0xff;
	num2 = id_number & 0xff;
	
	sprintf(temp1, "%03d%03d", num1, num2);
	memset(wifi_send_buff, 0 , sizeof(wifi_send_buff));
	sprintf(wifi_send_buff, "AT+SETID=%s", temp1);
#ifdef DEBUG_ON
	printf("%s\r\n",wifi_send_buff);
#endif
}

void setEspTTOn(void)
{
//	char buff[32] = {0};
	
	memset(wifi_send_buff, 0 , sizeof(wifi_send_buff));
	sprintf(wifi_send_buff, "AT+TRANSON");
#ifdef DEBUG_ON
	printf("%s\r\n",wifi_send_buff);
#endif
}

void setEspTTOff(void)
{
//	char buff[32] = {0};
	
	memset(wifi_send_buff, 0 , sizeof(wifi_send_buff));
	sprintf(wifi_send_buff, "AT+TRANSOFF");
#ifdef DEBUG_ON
	printf("%s\r\n",wifi_send_buff);
#endif
}

void sendNFCNum(uint8_t *nfc_num)
{
	memset(wifi_send_buff, 0 , sizeof(wifi_send_buff));
	sprintf(wifi_send_buff, "{\"Name\":\"ReportNfcCardData\",\"Data\":\"%s\"\"}", nfc_num);
#ifdef DEBUG_ON
	printf("%s\r\n",wifi_send_buff);
#endif
}

void setWifiWorkStatus(WIFI_status_t status)
{
	wifi_work_status = status;
}

void setWifiComStatus(WIFI_com_status_t status)
{
	wifi_send_status = status;
}

void clearWifiComStatus(void)
{
	wifi_send_status = idle_status;
}

void setWifiComFlag(void)
{
	wifi_com_status_flag = 1;
}

void clearWifiComFlag(void)
{
	wifi_com_status_flag = 0;
}

void processTTData(uint8_t *pbuff)
{
	char *ptemp;
	
	ptemp = strstr((const char*)pbuff, "SetDeviceStatus");
	//received sever set socket status commond
	if(ptemp != 0)
	{
		ptemp = strstr((const char*)pbuff, "Switch");
		ptemp += strlen("Switch") + 3; //Switch":1
		scoket_hw_status.sw_stat = atoi(ptemp);
		ptemp = strstr((const char*)pbuff, "SwithcDelay");
		ptemp += strlen("SwithcDelay") + 3; //SwithcDelay":100
		scoket_hw_status.sw_delay = atoi(ptemp);
		ptemp = strstr((const char*)pbuff, "LightCount");
		ptemp += strlen("LightCount") + 3; //LightCount":5
		scoket_hw_status.led_cnt = atoi(ptemp);
		ptemp = strstr((const char*)pbuff, "LightOnTime");
		ptemp += strlen("LightOnTime") + 3; //LightOnTime":5
		scoket_hw_status.led_on_time = atoi(ptemp);
		ptemp = strstr((const char*)pbuff, "LightOffTime");
		ptemp += strlen("LightOffTime") + 3; //LightOnTime":5
		scoket_hw_status.led_off_time = atoi(ptemp);
		ptemp = strstr((const char*)pbuff, "LightColor");
		ptemp += strlen("LightColor") + 3; //LightOnTime":5
		scoket_hw_status.led_color = atoi(ptemp);
		ptemp = strstr((const char*)pbuff, "BuzzerCount");
		ptemp += strlen("BuzzerCount") + 3; //LightOnTime":5
		scoket_hw_status.buz_cnt = atoi(ptemp);
		ptemp = strstr((const char*)pbuff, "BuzzerOnTime");
		ptemp += strlen("BuzzerOnTime") + 3; //LightOnTime":5
		scoket_hw_status.buz_on_time = atoi(ptemp);
		ptemp = strstr((const char*)pbuff, "BuzzerOffTime");
		ptemp += strlen("BuzzerOffTime") + 3; //LightOnTime":5
		scoket_hw_status.buz_off_time = atoi(ptemp);
		ptemp = strstr((const char*)pbuff, "BuzzerVolume");
		ptemp += strlen("BuzzerVolume") + 3; //LightOnTime":5
		scoket_hw_status.buz_vol = atoi(ptemp);
		
		if(scoket_hw_status.sw_stat)
			setIOControlFlag(io_control_flag.relay_flag);
		if(scoket_hw_status.led_cnt)
			setIOControlFlag(io_control_flag.led_flag);
		if(scoket_hw_status.buz_cnt)
		{
			setIOControlFlag(io_control_flag.buzzer_flag);
			io_control_flag.buzzer_flag = 1;
		}
	}
	ptemp = strstr((const char*)pbuff, "RebootDevice");
	//received sever set socket reset commond
	if(ptemp != 0)
	{
		ptemp = strstr((const char*)pbuff, "Mode");
		ptemp += strlen("Mode") + 3; //LightOnTime":5
		socket_reboot.reboot_mode = atoi(ptemp);
		ptemp = strstr((const char*)pbuff, "DelayDelay");
		ptemp += strlen("DelayDelay") + 3; //LightOnTime":5
		socket_reboot.reboot_delay = atoi(ptemp);
	}
	ptemp = strstr((const char*)pbuff, "GetRealtimeLoad");
	//received sever get socket power status commond
	if(ptemp != 0)
	{
		send_1s_power_status();
	}
}

void handleWifiReciveData(uint8_t *pbuff)
{
	uint8_t temp_data;
	
	//wifi status information
	if(strstr((char *)pbuff, (const char *)"STATUS"))
	{
		temp_data = pbuff[7] - 0x30;//ÌáÈ¡×´Ì¬
		if((temp_data > 0)&&(temp_data < 10))
		{
			setWifiWorkStatus(temp_data);
		}
		//memset(&wifi_rev_tt_data, 0, sizeof(wifi_rev_tt_data));
	}
	//send success ack. means you can send data now .
	if(strstr((char *)pbuff, (const char *)"OK"))
	{
		clearWifiComFlag();
	}
	
	switch(wifi_work_status)
	{
		case wifi_stat_sta:
			break;
		case wifi_stat_ap:
			break;
		case wifi_stat_idle:
			//setEspTTOn();
			setWifiComStatus(send_tt_on_status);
			break;
		case wifi_stat_tt:
//			if(wifi_work_status != wifi_stat_tt)
//				setWifiComStatus(send_tt_1s_power_status);
			processTTData(wifi_receive_buff);
//			memset(wifi_receive_buff, 0, sizeof(wifi_receive_buff));
			break;
		case wifi_stat_updata:
			break;
		case wifi_stat_recon:
			break;
		default:
			break;
	}
}

void senNByteUartData(char *buff)
{
	uint16_t i;
	
	for(i = 0; *(buff+i)!=0; i++)
	{
			USART_SendData(USART1, *(buff+i)); 
			while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
	}	
}

void sendToWifi(void)
{
	if(wifi_com_status_flag == 0)
	{
		if(wifi_send_status != idle_status)
		{
			switch(wifi_send_status)
			{
//				case idle_status:
//					break;
				case send_tt_on_status:
					setEspTTOn();
					break;
				case send_tt_off_status:
					setEspTTOff();
				case send_tt_1s_power_status:
					sendDeviceStatus(scoket_power_status);
					break;
				case send_tt_nfc_card_status:
					sendNFCNum(nfc_num);
					break;
				default:
					break;
			}
			senNByteUartData(wifi_send_buff);
			
			setWifiComFlag();
			clearWifiComStatus();
		}
	}
}

void send_1s_power_status(void)
{
	get_Measur_Data(scoket_power_status);
	if(wifi_work_status == wifi_stat_tt)
		setWifiComStatus(send_tt_1s_power_status);
}


