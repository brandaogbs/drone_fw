#ifndef __HCSR04__H
#define __HCSR04__H

#include "stm32f4xx_hal.h"


typedef union 
{
	uint8_t		HCSR04Stream[4];
	uint32_t	HCSR04data;	
}HCSR04_PackType;

#endif  //__INIT__H



