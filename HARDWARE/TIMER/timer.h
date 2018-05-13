#ifndef __TIMER_H
#define __TIMER_H
#include "sys.h"

#define SysTickFlag_1ms			0x00000001
#define SysTickFlag_3ms			0x00000002
#define SysTickFlag_5ms			0x00000004
#define SysTickFlag_10ms		0x00000008
#define SysTickFlag_25ms		0x00000010
#define SysTickFlag_50ms		0x00000020
#define SysTickFlag_100ms		0x00000040
#define SysTickFlag_333ms		0x00000080
#define SysTickFlag_500ms		0x00000100
#define SysTickFlag_1000ms	0x00000200
#define SysTickFlag_3000ms	0x00000400
#define SysTickFlag_1min		0x00000800
#define SysTickFlag_5500ms	0x00001000
#define MillisecondsIT 		1000


void TIM3_Int_Init(u16 arr,u16 psc);
 
#endif
