/*
	* designed by bill
	*	2018.5.11
	*	this file is io controling includ key led switch
*/

#include "iocontrol.h"
#include "esp8266.h"

io_control_flag_t io_control_flag;
scoket_hw_status_t io_control_value;

uint8_t buzzer_run_status;
uint8_t buzzer_1time_finish_flag;
uint8_t relay_1time_finish_flag;

extern scoket_hw_status_t scoket_hw_status;

void setIOControlFlag(uint8_t io_cont_flag)
{
	io_cont_flag = 1;
}

void clearIOControlFlag(uint8_t io_cont_flag)
{
	io_cont_flag = 0;
}

void setBuzzerOn(void)
{
	SET_BUZZER_ON();
	buzzer_run_status = 1;
}

void setBuzzerOff(void)
{
	SET_BUZZER_OFF();
	buzzer_run_status = 0;
}

void setBuzzerValue(void)
{
	io_control_value.buz_on_time = scoket_hw_status.buz_on_time;
	io_control_value.buz_off_time = scoket_hw_status.buz_off_time;
}

uint8_t runBuzzer(void)
{
	uint8_t _ststs = 0;
	static uint8_t step = 0;
	
	if(step == 0)
	{
		setBuzzerOn();
		setBuzzerValue();
		buzzer_1time_finish_flag = 0;
		step++;
	}
	
	if(buzzer_1time_finish_flag)
	{
		step = 0;
		_ststs = 1;
	}
	
	return _ststs;
}

uint8_t runRelay(void)
{
	io_control_value.sw_delay = scoket_hw_status.sw_delay;
}

void setRelay(uint8_t stat)
{
	if(stat)
		SET_RELAY_ON();
	else
		SET_RELAT_OFF();
}

void exeIOEvent(void)
{
	if(io_control_flag.buzzer_flag)
	{
		if(scoket_hw_status.buz_cnt)
		{
			if(runBuzzer())
			{
				scoket_hw_status.buz_cnt--;
				setBuzzerValue();
			}
		}
		else
		{
			buzzer_1time_finish_flag = 0;
			io_control_flag.buzzer_flag = 0;
			clearIOControlFlag(io_control_flag.buzzer_flag);
		}
	}
	if(io_control_flag.relay_flag)
	{
		if(relay_1time_finish_flag)
		{
			if(scoket_hw_status.sw_stat == 1)
				setRelay(1);
			else if(scoket_hw_status.sw_stat == 2)
				setRelay(0);
			relay_1time_finish_flag = 0;
			clearIOControlFlag(io_control_flag.relay_flag);
		}
	}
	if(io_control_flag.led_flag)
	{
		
	}
}

void handle_1ms_iocontrol(void)
{
	//buzzer process
	if(io_control_flag.buzzer_flag)
	{
		if(buzzer_run_status == 1)
		{
			if(io_control_value.buz_on_time)
				io_control_value.buz_on_time--;
			else
			{
				setBuzzerOff();
				buzzer_run_status = 0;
			}		
		}
		else
		{
			if(io_control_value.buz_off_time)
				io_control_value.buz_off_time--;
			else
				buzzer_1time_finish_flag = 1;
		}
	}
	if(io_control_flag.relay_flag)
	{
		if(io_control_value.sw_delay)
			io_control_value.sw_delay--;
		else
			relay_1time_finish_flag = 1;
	}
}