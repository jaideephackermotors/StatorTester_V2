#ifndef GLOBALSW_H
#define GLOBALSW_H
#include <stdint.h>
#include <math.h>
#include <string.h>
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"


typedef enum
{

	FALSE=0,
	TRUE
}bool;

typedef enum
{

	EXTRACT=0,
	INSERT
}rotor;

typedef enum
{

	RELEASE=0,
	BLOCK
}secure;

typedef enum
{
	PWMOFF=3
}pwmstate;

typedef enum
{

	OFF=0,
	ON
}state;

typedef enum
{

	RAC=0,
	RAB,
	RBC
}resistance;

//typedef bitset<8> BYTE;

#define __IO volatile


#define GetTime()	systime //HAL_GetTick()  //time in ms

#define FWVersion		0x0114

#define 	AIRCR_VECTKEY_MASK   ((uint32_t)0x05FA0000)

#endif
