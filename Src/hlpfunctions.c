#include "hlpfunctions.h"
#include <stdint.h>
#include <math.h>
#include <string.h>
#include <malloc.h>
#include "stm32f4xx_hal.h"
#include "globalsw.h"
#include <stdlib.h>
#include <str2dbl.h>


uint16_t crc16(uint16_t crc, unsigned char data)
{
   uint16_t ret_val;
     data ^= (unsigned char)(crc) & (unsigned char)(0xFF);
     data ^= data << 4;
     ret_val = ((((uint16_t)data << 8) | ((crc & 0xFF00) >> 8))
               ^ (unsigned char)(data >> 4)
               ^ ((uint16_t)data << 3));
     return ret_val;
}

uint16_t crc16arr(uint8_t *arr, uint8_t size)
{
	//uint8_t vel = sizeof(arr);
	uint16_t vysl = 0;
	uint8_t h=0;
	for(h=0; h<size; h++)
		{
		vysl = crc16(vysl,*(arr+h));
		}
return vysl;
}