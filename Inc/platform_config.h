// platform_config.h
#ifndef __PLATFORM_CONFIG_H
#define __PLATFORM_CONFIG_H

#include "stm32f4xx_hal.h"
#include "cmsis_os.h"

// Shared HAL handles (extern only — no definitions here)
extern ADC_HandleTypeDef  hadc1;
extern DMA_HandleTypeDef  hdma_adc1;
extern I2C_HandleTypeDef  hi2c1;
extern TIM_HandleTypeDef  htim1;
extern TIM_HandleTypeDef  htim2;
extern TIM_HandleTypeDef  htim7;
extern TIM_HandleTypeDef  htim8;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart4;
extern TIM_HandleTypeDef htim6;


// Functions
void HWInit(void);
void SystemClock_Config(void);

#endif
