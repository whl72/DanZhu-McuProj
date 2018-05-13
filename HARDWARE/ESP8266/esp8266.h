
#ifndef _ESP8266_H
#define _ESP8266_H

#include "sys.h"

#define ESP_RST_PIN		GPIO_Pin_13
#define ESP_RST_PORT	GPIOC
#define ESP_RST_HIGH()		GPIO_SetBits(ESP_RST_PORT, ESP_RST_PIN)
#define ESP_RST_LOW()			GPIO_ResetBits(ESP_RST_PORT, ESP_RST_PIN)

typedef struct _scoket_hw_status
{
	uint8_t sw_stat;
	uint16_t sw_delay;
	uint8_t led_cnt;
	uint16_t led_on_time;
	uint16_t led_off_time;
	uint8_t led_color;
	uint16_t buz_cnt;
	uint16_t buz_on_time;
	uint16_t buz_off_time;
	uint16_t buz_vol;
}scoket_hw_status_t;

typedef struct _scoket_power_status
{
	float current_val;
	float voltate_val;
	float watt_val;
	uint32_t ts_val;
}scoket_power_status_t;

typedef enum
{
	restart = 0,
	reset = 1
}reboot_mode_t;

typedef struct _socket_reboot_status
{
	reboot_mode_t reboot_mode;
	uint16_t reboot_delay;
}socket_reboot_t;

typedef enum
{
	idle_mode = 0,
	at_cmd_mode = 1,
	tt_data_mode = 2,
	update_fw_mode = 3
}WIFI_com_mode_t;

typedef enum
{
	wifi_stat_sta = 1,
	wifi_stat_ap = 2,
	wifi_stat_idle = 4,
	wifi_stat_tt = 5,
	wifi_stat_wait = 7,
	wifi_stat_updata = 8,
	wifi_stat_recon = 9
}WIFI_status_t;

typedef enum
{
	idle_status = 0,
	send_id_status = 1,
	send_tt_on_status = 2,
	send_tt_off_status = 3,
	send_tt_1s_power_status = 4,
	send_tt_nfc_card_status = 5
}WIFI_com_status_t;

//typedef struct
//{
//	uint8_t idle_status;
//	uint8_t send_tt_on_status;
//	uint8_t send_tt_off_status;
//	uint8_t send_tt_data_status;
//}WIFI_com_status_t;


void sendDeviceStatus(scoket_power_status_t power_status);
void sendDeviceID(uint16_t id_number);
void setEspTTOn(void);
void setEspTTOff(void);
void initWifi(void);
void setWifiComStatus(WIFI_com_status_t status);
void handleWifiReciveData(uint8_t *pbuff);
void sendToWifi(void);
void send_1s_power_status(void);


#endif
