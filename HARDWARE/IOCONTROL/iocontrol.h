
#ifndef _IOCONTROL_H
#define _IOCONTROL_H

#include "stdint.h"
#include "stm32f10x.h"

//relay hardware interface
#define RELAY_CONTROL_PIN		GPIO_Pin_3
#define RELAY_PORT					GPIOB

//buzzer hardware interface
#define BUZZER_CONTROL_PIN		GPIO_Pin_1
#define BUZZZER_PORT					GPIOB

#define SET_BUZZER_ON()				GPIO_SetBits(BUZZZER_PORT, BUZZER_CONTROL_PIN)
#define SET_BUZZER_OFF()			GPIO_ResetBits(BUZZZER_PORT, BUZZER_CONTROL_PIN)
#define SET_RELAY_ON()				GPIO_SetBits(RELAY_PORT, RELAY_CONTROL_PIN)
#define SET_RELAT_OFF()				GPIO_ResetBits(RELAY_PORT, RELAY_CONTROL_PIN)

typedef struct
{
	uint8_t relay_flag;
	uint8_t led_flag;
	uint8_t buzzer_flag;
}io_control_flag_t;



void setIOControlFlag(uint8_t io_cont_flag);
void handle_1ms_iocontrol(void);
void exeIOEvent(void);
void setRelay(uint8_t stat);

#endif