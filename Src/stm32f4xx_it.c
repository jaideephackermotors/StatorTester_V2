/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "stm32f4xx.h"
#include "stm32f4xx_it.h"
#include "cmsis_os.h"
#include "platform_config.h"


/* USER CODE BEGIN 0 */
#include "communication.h"
#include "globalsw.h"

/* pro UART4 - GOM komunikace */
uint32_t dtr=0;
UART_HandleTypeDef *huart;
uint32_t errorflags = 0x00U;
char GOM_rxbuffer[GOM_RXBUFFER_SIZE];
bool gom_usartfinal=FALSE;
uint8_t gom_buffIndex=0;

/* pro JETIBOX */
#define D_DelkaSWUART		13

extern bool SendingJBData;
bool ButtonsRec=FALSE;
extern uint16_t screen[];
uint8_t JetiBoxData = 0;
uint8_t guc_CountSWUART = 0;
uint8_t ButtonsBit = 0;
bool ButtonsReady = FALSE;
uint8_t ButtonsHlp = 0;
extern uint8_t Buttons;

//extern __IO uint16_t hlpCounterRxHrana;
extern __IO uint16_t hlpCounterRxByte;

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_adc1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim7;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart2;

extern TIM_HandleTypeDef htim6;

/******************************************************************************/
/*            Cortex-M4 Processor Interruption and Exception Handlers         */ 
/******************************************************************************/

/**
* @brief This function handles Non maskable interrupt.
*/
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
* @brief This function handles Hard fault interrupt.
*/
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
  /* USER CODE BEGIN HardFault_IRQn 1 */

  /* USER CODE END HardFault_IRQn 1 */
}

/**
* @brief This function handles Memory management fault.
*/
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
  /* USER CODE BEGIN MemoryManagement_IRQn 1 */

  /* USER CODE END MemoryManagement_IRQn 1 */
}

/**
* @brief This function handles Pre-fetch fault, memory access fault.
*/
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
  /* USER CODE BEGIN BusFault_IRQn 1 */

  /* USER CODE END BusFault_IRQn 1 */
}

/**
* @brief This function handles Undefined instruction or illegal state.
*/
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
  /* USER CODE BEGIN UsageFault_IRQn 1 */

  /* USER CODE END UsageFault_IRQn 1 */
}

/**
* @brief This function handles Debug monitor.
*/
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
* @brief This function handles System tick timer.
*/
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  osSystickHandler();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
* @brief This function handles EXTI line 1 interrupt.
*/
void EXTI1_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI1_IRQn 0 */

  /* USER CODE END EXTI1_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_1);
  /* USER CODE BEGIN EXTI1_IRQn 1 */
  //HAL_NVIC_DisableIRQ(EXTI1_IRQn);

  if(HAL_GPIO_ReadPin(Jetibox_GPIO_Port,Jetibox_Pin)==GPIO_PIN_RESET)
  {
  JBpin_for_Rx();//prenastaveni pinu pro prijem tlacitek jetiboxu
  __HAL_TIM_SET_COUNTER(&htim7 ,(uint16_t)4638);//nastaveni polovicniho aktualniho casu timeru 103us na 51,5us
  //GOMReleIn2(ON);
  //TIM_ResetCounter(&htim7);
 // HAL_TIM_Base_Init(&htim7);
  HAL_TIM_IRQHandler(&htim7);//vynulovani pripadnych flagu timeru
  ButtonsRec = TRUE;//povoleni zaznamu dat tlacitek jetiboxu
  //hlpCounterRxHrana++;
  }
  else{
	  //HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
  }
  /* USER CODE END EXTI1_IRQn 1 */
}

/**
* @brief This function handles TIM2 global interrupt.
*/
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */

  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */

  /* USER CODE END TIM2_IRQn 1 */
}

/**
* @brief This function handles USART2 global interrupt.
*/
void USART2_IRQHandler(void)
{
  /* USER CODE BEGIN USART2_IRQn 0 */

  /* USER CODE END USART2_IRQn 0 */
  HAL_UART_IRQHandler(&huart2);
  /* USER CODE BEGIN USART2_IRQn 1 */

  /* USER CODE END USART2_IRQn 1 */
}

/**
* @brief This function handles UART4 global interrupt.
*/
void UART4_IRQHandler(void)
{
  /* USER CODE BEGIN UART4_IRQn 0 */

	  huart=&huart4;
	 /* If no error occurs */
	 errorflags = (UART4->SR & (uint32_t)(USART_SR_PE | USART_SR_FE | USART_SR_ORE | USART_SR_NE));
	 if(errorflags == RESET)
	 {
	   /* UART in mode Receiver -------------------------------------------------*/
	   if(((UART4->SR & USART_SR_RXNE) != RESET) && ((UART4->CR1 & USART_CR1_RXNEIE) != RESET))
	   {
	     //cti data
		   dtr =((uint8_t)UART4->DR);
		   //if(dtr=="/n"){}
		   if(dtr == 0x0A){gom_usartfinal=TRUE;}//kontroluju zda preisel ukoncovaci znak zpravy
		   //else{gom_buffIndex++;}
		   //else{GOM_rxbuffer[gom_buffIndex++]=(uint8_t)dtr;}
		   GOM_rxbuffer[gom_buffIndex++]=(uint8_t)dtr;

	   }
	 }
  /* USER CODE END UART4_IRQn 0 */
  HAL_UART_IRQHandler(&huart4);
  /* USER CODE BEGIN UART4_IRQn 1 */

  /* USER CODE END UART4_IRQn 1 */
}

/**
* @brief This function handles TIM6 global interrupt and DAC1, DAC2 underrun error interrupts.
*/
void TIM6_DAC_IRQHandler(void)
{
  /* USER CODE BEGIN TIM6_DAC_IRQn 0 */

  /* USER CODE END TIM6_DAC_IRQn 0 */
  HAL_TIM_IRQHandler(&htim6);
  /* USER CODE BEGIN TIM6_DAC_IRQn 1 */

  /* USER CODE END TIM6_DAC_IRQn 1 */
}

/**
* @brief This function handles TIM7 global interrupt.
*/
void TIM7_IRQHandler(void)
{
  /* USER CODE BEGIN TIM7_IRQn 0 */
//timer 103us

  /* USER CODE END TIM7_IRQn 0 */
  HAL_TIM_IRQHandler(&htim7);
  /* USER CODE BEGIN TIM7_IRQn 1 */

  if(SendingJBData){
	if (JetiBoxData > JB_PACKETS)
		{
		  JetiBoxData = 0;
		  SendingJBData = FALSE;
		  //povoleni preruseni na sestupnou hranu - cekani na tlacitka
		  JBpin_for_RxIt();
		  ButtonsRec = FALSE;
		  ButtonsBit = 0;
		  ButtonsHlp = 0;
		  Buttons = 0;
		 }
	else
		{
		if((screen[JetiBoxData]>>guc_CountSWUART)&1){JB_Output(ON);}else{JB_Output(OFF);}
		//po poslani 12bitu zameni bajt v poli
		if (guc_CountSWUART == 0){guc_CountSWUART = D_DelkaSWUART;JetiBoxData++;}
		else{guc_CountSWUART--;}
		}
  }

  /*
	if (SendingJBData)
	   {
	      //posilani jednotlivych bitu na port
		if((screen[JetiBoxData]>>guc_CountSWUART)&1){JB_Output(ON);}else{JB_Output(OFF);}

	      if (guc_CountSWUART == 0)
	      {
	         if (JetiBoxData >= JB_PACKETS)
	         {
	        	SendingJBData = FALSE;
	            JetiBoxData = 0;
	            guc_CountSWUART = D_DelkaSWUART;

	         }

	         else
	         {
	            JetiBoxData++;
	            guc_CountSWUART = D_DelkaSWUART;
	         }
	      }
	      else{guc_CountSWUART--;}
	   }
	*/
  else{
    if (ButtonsRec)
    {
  	  //GOMReleIn3(ON);
  	  GPIO_PinState PinVal = HAL_GPIO_ReadPin(Jetibox_GPIO_Port,Jetibox_Pin);
  	  //if((GPIOB->IDR & 0x02)!=0){Buttons++;}
  	  //if(PinVal == GPIO_PIN_SET){Buttons++;};
  	  //hlpCounterRxByte++;
  	  ButtonsBit++;
       if (ButtonsBit < 6){}
       else
       {
          //cteni az toho 4 (5)bitu tlacitek
          if (ButtonsBit < 14)
          {
             ButtonsHlp <<=1;
             ButtonsHlp |= PinVal;//nacteni bitu
          }
          else
          {
             //hotove nacteni 4bitu, 8-eho nuloveho bitu, Parity a dvou Stop bitu = 8bit
             ButtonsRec = FALSE;
             ButtonsBit = 0;
             Buttons = ButtonsHlp;
             ButtonsReady = TRUE;
          }
       }
    }
  }
  /* USER CODE END TIM7_IRQn 1 */
}

/**
* @brief This function handles DMA2 stream0 global interrupt.
*/
void DMA2_Stream0_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream0_IRQn 0 */

  /* USER CODE END DMA2_Stream0_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_adc1);
  /* USER CODE BEGIN DMA2_Stream0_IRQn 1 */

  /* USER CODE END DMA2_Stream0_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
