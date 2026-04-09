/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2019 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"

/* USER CODE BEGIN Includes */
#include "platform_config.h"
#include "communication.h"
#include "globalsw.h"
#include <stdlib.h>
#include <str2dbl.h>
#include "hlpfunctions.h"

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart2;

osThreadId defaultTaskHandle;
osThreadId StateMachineHandle;
osThreadId FastLoopHandle;
osThreadId JetiboxHandle;
osThreadId USBconnectionHandle;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

uint64_t systime=0;
extern uint32_t dtr;
//extern char GOM_rxbuffer[];
uint8_t success=0;
extern uint8_t gom_buffIndex;
extern bool GOM_OverRange;
extern int32_t GOM_val;
//extern __IO gom GOM_RangeSet;
//extern float GOM_val_hlp;
extern char GOM_out[];
extern uint32_t GOM_uOhm;
static bool GOM_FlagAB=FALSE;
static bool GOM_FlagAC=FALSE;
static bool GOM_FlagBC=FALSE;
__IO uint32_t GOM_uOhm_ABC[]={0,0,0};

double pokus=0;
char *ptr;
uint32_t g_ADC1Buffer[ADC1Inputs];
uint16_t VCRAFT_ADCU;
uint16_t VCRAFT_ADCI;
uint16_t SERVO_ADCU;
uint16_t SERVO_ADCI;

uint8_t StateTester=MAIN_WAITING;
uint8_t CalErr = MAIN_NONE;
__IO int32_t CalErrValue = 0;
__IO uint8_t CalStep = 0;
__IO int32_t CallValue = 0;
bool USBcomReady = FALSE;//flag na start komunikace na UART2 - USB s horni deskou

__IO uint16_t hlpCounterTx = 0;
__IO uint16_t hlpCounterRxHrana = 0;
__IO uint16_t hlpCounterRxByte = 0;
//float hlprelval=0;
__IO uint32_t hlpdisp=0;
uint8_t Buttons = 0;

uint16_t volt=0;

// variables for RPM frequency averaging
__IO uint32_t PeriodBuffer[FREQ_AVG_SAMPLES];
__IO uint8_t PeriodIndex = 0;
__IO uint32_t PeriodSum = 0;

typedef struct tester {
	__IO uint32_t Current_A;
	__IO uint32_t Voltage;
}TESTER;

//JETIBOX
bool SendingJBData = 0;
__IO uint16_t screen[35];
uint8_t uc_i=0;
char screenhlp[33];
char JBscreen[33];
extern bool ButtonsReady;
//extern uint8_t ButtonsHlp;
uint8_t AllBtn=0;
uint8_t menuCode=JB_MainMenu;
__IO uint64_t MenuTime=0;
__IO uint8_t lcdrunning=1;

//rpm
__IO uint32_t ICPval = 0;
__IO uint32_t RPMval = 0;
__IO uint32_t ICPcnt = 0;
__IO uint32_t ArrICP[155];
bool TIM2Over=TRUE;
uint8_t stati=0;
bool Direction = FALSE;

//I2C
bool SetVoltcraft=FALSE;

//Err
bool ErrEnd=FALSE;
uint32_t timeSave=0;


struct tester MotorSource={0,0};
struct tester ServoSource={0,0};
extern struct flagy Flag;
extern struct tset Tset;
extern struct commands Command;
__IO uint32_t MotorKV=0;
__IO uint32_t MotorIo=0;
__IO uint32_t MotorRpm=0;
__IO uint32_t MotorUo=0;
__IO uint32_t MotorImax=0;
bool MotorDirection=FALSE;
bool MotorOvercurrent=FALSE;
uint8_t OI_counter=0;

resistance Res;

extern uint8_t USB_Rxbuffer[];


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_UART4_Init(void);
static void MX_TIM8_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM7_Init(void);
static void MX_TIM2_Init(void);
static void MX_I2C1_Init(void);
void StartDefaultTask(void const * argument);
void StartStateMachineTask(void const * argument);
void StartFastLoop(void const * argument);
void StartJetibox(void const * argument);
void StartUSBconnection(void const * argument);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                                

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void HAL_UART_MspInit(UART_HandleTypeDef* huart);
void Set_Source_Voltage(uint32_t val);
void Set_Source_Current(uint32_t val);
void Set_GOMConnectAllPhaseOff(void);
void Set_GOMConnectAB(void);
void Set_GOMConnectAC(void);
void Set_GOMConnectBC(void);
//void Set_GOMZkratIN4IN2(void);
void clr_LCD_1R(void);
void clr_LCD_2R(void);
void set_LCD_data(char* screenh);
void JBpin_for_Tx(void);
void JBpin_for_RxIt(void);
void JBpin_for_Rx(void);
uint8_t checkBtn(uint8_t butts);
double str2dbl(char *s);
void TIM_ResetCounter(TIM_HandleTypeDef *htim);
void SendJBDisplay(void);
bool Btn(BTN button);
void Insertion(void);
void Extraction(void);
void ServoOff(void);
void ConnectVoltcraft(state val, uint32_t voltage, uint32_t current);
void USBcom_FStopTest(void);
void USBcom_FCalibration(void);

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* AdcHandle)
{
	if(AdcHandle->Instance == ADC1)
	{
	  static float avgVal1=0;
	  static float avgVal2=0;
	  static float avgVal3=0;
	  static float avgVal4=0;
	  static int32_t samplesCnt=0;

	    avgVal1+=g_ADC1Buffer[0];
	    avgVal2+=g_ADC1Buffer[1];
	    avgVal3+=g_ADC1Buffer[2];
	    avgVal4+=g_ADC1Buffer[3];
	    samplesCnt++;
	    if(samplesCnt>=D_DivNo)
	    {
	    	VCRAFT_ADCU = (uint16_t)(avgVal1/D_DivNo);
	    	VCRAFT_ADCI = (uint16_t)(avgVal2/D_DivNo);
	    	SERVO_ADCU = (uint16_t)(avgVal3/D_DivNo);
	    	SERVO_ADCI = (uint16_t)(avgVal4/D_DivNo);
			/*if(VCRAFT_ADCU>0 || VCRAFT_ADCI>0){
			HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
			}
*/
			samplesCnt=0;
			avgVal1=0;
			avgVal2=0;
			avgVal3=0;
			avgVal4=0;
	    }
	}
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin==GPIO_PIN_1){

	}

}

/*void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if (huart == &huart2) {
		USBDataResult();
	}
}*/


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if (huart == &huart2) {
		USBDataResult();
	}
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim){
	if(htim->Instance == TIM2)
		{
		__HAL_TIM_SetCounter(htim , 0);
		hlpCounterRxHrana++;
		TIM2Over=FALSE;
		}
}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  //SystemClock_Config();
  //HWInit();
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_UART4_Init();
  MX_TIM8_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_TIM7_Init();
  MX_TIM2_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)g_ADC1Buffer, ADC1Inputs);
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);//PWM pro regulator
  //HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  //HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  //HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1|TIM_CHANNEL_2|TIM_CHANNEL_3);
  HAL_TIM_Base_Start_IT(&htim7);
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_4);

  //USBTransmit();

  //HAL_ADC_Start_IT(&hadc1);
  //float atof();
  //xTaskCreate(State,(const char* const)"state",configMINIMAL_STACK_SIZE,0,2,0);
  //Task pointer, Task name, Stack depth, Parameter to pass, Task priority, Pass handle to create task

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 256);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of StateMachine */
  osThreadDef(StateMachine, StartStateMachineTask, osPriorityNormal, 0, 256);
  StateMachineHandle = osThreadCreate(osThread(StateMachine), NULL);

  /* definition and creation of FastLoop */
  osThreadDef(FastLoop, StartFastLoop, osPriorityAboveNormal, 0, 256);
  FastLoopHandle = osThreadCreate(osThread(FastLoop), NULL);

  /* definition and creation of Jetibox */
  osThreadDef(Jetibox, StartJetibox, osPriorityBelowNormal, 0, 256);
  JetiboxHandle = osThreadCreate(osThread(Jetibox), NULL);

  /* definition and creation of USBconnection */
  osThreadDef(USBconnection, StartUSBconnection, osPriorityNormal, 0, 256);
  USBconnectionHandle = osThreadCreate(osThread(USBconnection), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
 

  /* Start scheduler */
  osKernelStart();
  
  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */


  while (1)
  {

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Activate the Over-Drive mode 
    */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);
}

/* ADC1 init function */
static void MX_ADC1_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 4;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_13;
  sConfig.Rank = 4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM1 init function */
static void MX_TIM1_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = TIM1_PRESCALER;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = TIM1_PERIOD;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim1);

}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_IC_InitTypeDef sConfigIC;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 15;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM7 init function */
static void MX_TIM7_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;

  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 0;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = JBPeriod;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM8 init function */
static void MX_TIM8_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;

  htim8.Instance = TIM8;
  htim8.Init.Prescaler = TIM8_PRESCALER;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = TIM8_PERIOD;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim8);

}

/* UART4 init function */
static void MX_UART4_Init(void)
{

  huart4.Instance = UART4;
  huart4.Init.BaudRate = 9600;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_8;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, VOLTCRAFT_OutputOnOff_Pin|Rele1_R_Pin|Rele2_R_Pin|Rele2_S_Pin 
                          |ReglDischarge_Pin|ReglCharge_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|Rele4_S_Pin|VCC_ServoOnOff_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, Rele3_S_Pin|Rele4_R_Pin|Rele3_R_Pin|Rele1_S_Pin 
                          |ConnectVoltcraftToReglOnOff_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : VOLTCRAFT_OutputOnOff_Pin Rele1_R_Pin Rele2_R_Pin Rele2_S_Pin 
                           ReglDischarge_Pin ReglCharge_Pin */
  GPIO_InitStruct.Pin = VOLTCRAFT_OutputOnOff_Pin|Rele1_R_Pin|Rele2_R_Pin|Rele2_S_Pin 
                          |ReglDischarge_Pin|ReglCharge_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin Rele4_S_Pin VCC_ServoOnOff_Pin */
  GPIO_InitStruct.Pin = LD2_Pin|Rele4_S_Pin|VCC_ServoOnOff_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : Jetibox_Pin */
  GPIO_InitStruct.Pin = Jetibox_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Jetibox_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Rele3_S_Pin Rele4_R_Pin Rele3_R_Pin Rele1_S_Pin 
                           ConnectVoltcraftToReglOnOff_Pin */
  GPIO_InitStruct.Pin = Rele3_S_Pin|Rele4_R_Pin|Rele3_R_Pin|Rele1_S_Pin 
                          |ConnectVoltcraftToReglOnOff_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : MotorDirection_Pin */
  GPIO_InitStruct.Pin = MotorDirection_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MotorDirection_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);



}

/* USER CODE BEGIN 4 */
void JBpin_for_Tx(void){

	  HAL_NVIC_DisableIRQ(EXTI1_IRQn);

	  GPIO_InitTypeDef GPIO_InitStruct;

	  GPIO_InitStruct.Pin = Jetibox_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	  HAL_GPIO_Init(Jetibox_GPIO_Port, &GPIO_InitStruct);


}

void JBpin_for_RxIt(void){

	  GPIO_InitTypeDef GPIO_InitStruct;

	  GPIO_InitStruct.Pin = Jetibox_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	  GPIO_InitStruct.Pull = GPIO_PULLUP;
	  HAL_GPIO_Init(Jetibox_GPIO_Port, &GPIO_InitStruct);

	 // HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_1);
	  /* EXTI interrupt init*/
	  HAL_NVIC_SetPriority(EXTI1_IRQn, 5, 0);
	  HAL_NVIC_EnableIRQ(EXTI1_IRQn);
}

void JBpin_for_Rx(void){

	  HAL_NVIC_DisableIRQ(EXTI1_IRQn);

	  GPIO_InitTypeDef GPIO_InitStruct;

	  GPIO_InitStruct.Pin = Jetibox_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	  GPIO_InitStruct.Pull = GPIO_PULLUP;
	  HAL_GPIO_Init(Jetibox_GPIO_Port, &GPIO_InitStruct);

}

void Set_Source_Voltage(uint32_t val)
{
	if(val<34){val=0;}
	else{val=(133*val-4512)/100;}
	val=val*TIM8_PERIOD/1000;
	TIM8->CCR1 = (uint16_t)val;
}

void Set_Source_Current(uint32_t val)
{
	val=val*TIM8_PERIOD/1000;
	TIM8->CCR2 = (uint16_t)val;
}

void Set_RegulatorPWM(uint32_t val)
{
	if(val>MAXIMUM_REGULATOR_PWM){val=MAXIMUM_REGULATOR_PWM;}
	TIM1->CCR1 = (uint16_t)val;
}

/* Update Voltcraft supply voltage/current via I2C only.
 * Safe to call while motor is running — no relay or pre-charge toggling.
 * voltage and current are in the same units as Flag.Voltage (0.1V / 0.1A),
 * multiplied by 10 internally to match the supply register format (10mV / 10mA). */
void UpdateVoltcraftVoltage(uint32_t voltage, uint32_t current)
{
	uint8_t mArray[2];
	voltage *= 10;
	current *= 10;
	mArray[0]=(uint8_t)voltage;
	mArray[1]=(uint8_t)(voltage>>8);
	HAL_I2C_Mem_Write(&hi2c1,(uint16_t)I2C_ADDRESS, 0x70, I2C_MEMADD_SIZE_8BIT, (uint8_t *)mArray, 2, 100);
	mArray[0]=(uint8_t)current;
	mArray[1]=(uint8_t)(current>>8);
	HAL_I2C_Mem_Write(&hi2c1,(uint16_t)I2C_ADDRESS, 0x72, I2C_MEMADD_SIZE_8BIT, (uint8_t *)mArray, 2, 100);
}


void Set_GOMConnectAllPhaseOff(void)
{
	GOM_FlagAB=FALSE;
	GOM_FlagBC=FALSE;
	GOM_FlagAC=FALSE;
	HAL_GPIO_WritePin(Rele1_R_GPIO_Port, Rele1_R_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(Rele2_R_GPIO_Port, Rele2_R_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(Rele3_R_GPIO_Port, Rele3_R_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(Rele4_R_GPIO_Port, Rele4_R_Pin, GPIO_PIN_SET);
	osDelay(RelayTime);
	HAL_GPIO_WritePin(Rele1_R_GPIO_Port, Rele1_R_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(Rele2_R_GPIO_Port, Rele2_R_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(Rele3_R_GPIO_Port, Rele3_R_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(Rele4_R_GPIO_Port, Rele4_R_Pin, GPIO_PIN_RESET);
	osDelay(RelayTime);
}

void Set_GOMConnectAB(void)
{
	Res=RAB;
	if(!GOM_FlagAB){
	Set_GOMConnectAllPhaseOff();
	HAL_GPIO_WritePin(Rele2_S_GPIO_Port, Rele2_S_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(Rele3_S_GPIO_Port, Rele3_S_Pin, GPIO_PIN_SET);
	osDelay(RelayTime);
	HAL_GPIO_WritePin(Rele2_S_GPIO_Port, Rele2_S_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(Rele3_S_GPIO_Port, Rele3_S_Pin, GPIO_PIN_RESET);
	osDelay(RelayTime);
	GOM_FlagAB=TRUE;
	}
}

void Set_GOMConnectAC(void)
{
	Res=RAC;
	if(!GOM_FlagAC){
	Set_GOMConnectAllPhaseOff();
	HAL_GPIO_WritePin(Rele2_S_GPIO_Port, Rele2_S_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(Rele4_S_GPIO_Port, Rele4_S_Pin, GPIO_PIN_SET);
	osDelay(RelayTime);
	HAL_GPIO_WritePin(Rele2_S_GPIO_Port, Rele2_S_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(Rele4_S_GPIO_Port, Rele4_S_Pin, GPIO_PIN_RESET);
	osDelay(RelayTime);
	GOM_FlagAC=TRUE;
	}
}

void Set_GOMConnectBC(void)
{
	Res=RBC;
	if(!GOM_FlagBC){
	Set_GOMConnectAllPhaseOff();
	HAL_GPIO_WritePin(Rele1_S_GPIO_Port, Rele1_S_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(Rele4_S_GPIO_Port, Rele4_S_Pin, GPIO_PIN_SET);
	osDelay(RelayTime);
	HAL_GPIO_WritePin(Rele1_S_GPIO_Port, Rele1_S_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(Rele4_S_GPIO_Port, Rele4_S_Pin, GPIO_PIN_RESET);
	osDelay(RelayTime);
	GOM_FlagBC=TRUE;
	}
}

void clr_LCD_2R(void)
{
  memset(JBscreen+16,' ',16);
}

void clr_LCD_1R(void)
{
  memset(JBscreen,' ',16);
}

void set_LCD_data(char* screenh)
{
uint8_t Screenj = 0;
uint8_t JBtemp0 = 0;
uint8_t JBtemp1 = 0;
uint8_t JBtemp2 = 0;
uint8_t JBtemp3 = 0;
uint8_t JBParitaTx = 0;

screen[Screenj++] = 0xfe7;
	for (JBtemp3 = 0; JBtemp3 < 32; JBtemp3++)
	{
		JBParitaTx = 0;
		JBtemp2 = 0;
		for (uc_i = 0; uc_i <= 7; uc_i++)
		{
			JBtemp0 = (screenh[JBtemp3]>>uc_i)&1;
			JBParitaTx += JBtemp0;
			JBtemp1 = 7-uc_i;
			JBtemp2 |= JBtemp0<<JBtemp1;
		}
		screen[(uint8_t)(JBtemp3+Screenj)] = ((uint16_t)JBtemp2<<4);
		JBParitaTx &= 1;
		if (JBParitaTx)
		{//suda parita
			screen[(uint8_t)(JBtemp3+Screenj)] |=0x0f;
		}
		else
		{//licha parita
			screen[(uint8_t)(JBtemp3+Screenj)] |=0x0b;
		}
		screen[(uint8_t)(JBtemp3+Screenj)]<<=1;
		screen[(uint8_t)(JBtemp3+Screenj)]|=1;
	}
 screen[(uint8_t)(Screenj+32)] = 0x1fef;
}

uint8_t checkBtn(uint8_t butts)
{
/*~+:Kontrola prijmu stavu tlaciteka priprava dat k odeslani do VF modulu*/
uint8_t JBParitaRx = 0;
uint8_t uc_i;
bool JBButtOK;
static uint8_t oldval=0xF7;
static uint8_t valOKcounter=0;
static uint8_t ButtOKCounter=0;
static uint8_t valout=0xF7;
//nacteni poctu "1" do JBParitaRx
for (uc_i = 4; uc_i < 8; uc_i++)
	{JBParitaRx += (butts>>uc_i)&1;}
	JBParitaRx &= 1;
if (JBParitaRx)
	{
   /*~+:-> licha parita, porovnani s prijatou paritou*/
   if (((butts>>2)&1)==0){JBButtOK = TRUE;}
   else{JBButtOK = FALSE;}
	}
else
	{
   /*~+:-> suda parita, porovnani s prijatou paritou*/
   if (((butts>>2)&1)){JBButtOK = TRUE;}
   else{JBButtOK = FALSE;}
	}
	//Overeni dvou koncovych stop bitu v komunikaci
if (JBButtOK && ((butts&3) == 3))
	{
   if (ButtOKCounter >= 6)
   	   {

	   if(butts==oldval)
	   {
		if(valOKcounter>1)//filtr
		{
			valout=oldval;
		}
		else{valOKcounter++;}
	   }
	   else{oldval=butts;valOKcounter=0;}

	   return (~valout>>4)&0xF;
   	   }
   else{ButtOKCounter++;}
	}
else
	{
   if (ButtOKCounter > 0){ButtOKCounter--;}
   else{ButtOKCounter = 0;}
	}
return (~valout>>4)&0xF;
}

void TIM_ResetCounter(TIM_HandleTypeDef *htim)
{
  /* Check the parameters */
  assert_param(IS_TIM_ALL_PERIPH(htim));

  /* Reset the Counter Register value */
  htim->Instance->CNT = 0;
}

void SendJBDisplay(void)
{
	memcpy(screenhlp, JBscreen,32);
	set_LCD_data(screenhlp);
	JBpin_for_Tx();
	ButtonsReady = FALSE;
	SendingJBData=TRUE;
}

bool Btn(BTN button)
{
	static bool pushstate=FALSE;

	if(AllBtn == button){if(pushstate==FALSE){pushstate=TRUE;return TRUE;}}
	else{pushstate=FALSE;}
	return FALSE;
}

void Set_RotorMovement(rotor val)
{
	if(val==INSERT)
	{
		TIM1->CCR2 = (uint16_t)2000;
		HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);

		//osDelay(1000);
		//HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
	}
	else if(val==EXTRACT)
	{
		TIM1->CCR2 = (uint16_t)1000;
		HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
		//osDelay(1000);
		//HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
	}
	else{HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);}
}

void Set_StatorSecure(secure val)
{
	if(val==RELEASE)
	{
		TIM1->CCR3 = (uint16_t)2000;
		HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
		//osDelay(1000);
		//HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
	}
	else if(val==BLOCK)
	{
		TIM1->CCR3 = (uint16_t)1000;
		HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
		//osDelay(1000);
		//HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
	}
	else{HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);}
}

void Insertion(void)
{
	return; //added this line to completely bypass extract and insert features
	SourceServo(ON);
	osDelay(300);
	CallValue=ServoSource.Voltage;
	if(CallValue>80 || CallValue<70) {CalErr=MAIN_ERR_SERVO_VCC;CalErrValue=CallValue;StateTester=MAIN_ERR;POWEROFF;SET_ERREND;return;}
	Set_StatorSecure(BLOCK);
	osDelay(1000);
	Set_RotorMovement(INSERT);
	osDelay(4000);
}

void Extraction(void)
{
	return; //added this line to completely bypass extract and insert features
	//SourceServo(OFF);
	//osDelay(100);
	SourceServo(ON);
	osDelay(300);
	Set_StatorSecure(BLOCK);
	CallValue=ServoSource.Voltage;
	if(CallValue>80 || CallValue<70) {CalErr=MAIN_ERR_SERVO_VCC;CalErrValue=CallValue;StateTester=MAIN_ERR;POWEROFF;SET_ERREND;return;}
	osDelay(1000);
	Set_RotorMovement(EXTRACT);
	osDelay(3000);
	Set_StatorSecure(RELEASE);
	osDelay(1000);
	Set_RotorMovement(PWMOFF);
	Set_StatorSecure(PWMOFF);
	osDelay(100);
	SourceServo(OFF);
	osDelay(200);
}

void ServoOff(void)
{
	Set_RotorMovement(PWMOFF);
	Set_StatorSecure(PWMOFF);
	osDelay(100);
	SourceServo(OFF);
	osDelay(200);
}

void RegulatorSourceOff()
{
	uint8_t mArray[]={0,0};
	//nastaveni napeti
	mArray[0]=0;
	mArray[1]=0;
	HAL_I2C_Mem_Write(&hi2c1,(uint16_t)I2C_ADDRESS, 0x70, I2C_MEMADD_SIZE_8BIT, (uint8_t *)mArray, 2, 100);

	//nastaveni proudu
	mArray[0]=0;
	mArray[1]=0;
	HAL_I2C_Mem_Write(&hi2c1,(uint16_t)I2C_ADDRESS, 0x72, I2C_MEMADD_SIZE_8BIT, (uint8_t *)mArray, 2, 100);

	//vypnuti zdroje
	mArray[0]=0x84;
	HAL_I2C_Mem_Write(&hi2c1,(uint16_t)I2C_ADDRESS, 0x7c, I2C_MEMADD_SIZE_8BIT, (uint8_t *)mArray, 1, 100);
}

void ConnectVoltcraft(state val, uint32_t voltage, uint32_t current)
{
	SetVoltcraft=TRUE;
		uint8_t mArray[]={0,0};
		uint64_t deltime = 0;
		uint8_t xptry = 0;
		uint8_t mfault = 0;
		voltage *= 10;
		current *= 10;
		osDelay(20);//delay pro ujisteni ze v rychle smycce nedojde ke komunikaci s XP zdrojem zaroven

		if(val==ON){
			Set_GOMConnectAllPhaseOff();

			//nastaveni napeti
			mArray[0]=(uint8_t)voltage;
			mArray[1]=(uint8_t)(voltage>>8);
			HAL_I2C_Mem_Write(&hi2c1,(uint16_t)I2C_ADDRESS, 0x70, I2C_MEMADD_SIZE_8BIT, (uint8_t *)mArray, 2, 100);

			//nastaveni proudu
			mArray[0]=(uint8_t)current;
			mArray[1]=(uint8_t)(current>>8);
			HAL_I2C_Mem_Write(&hi2c1,(uint16_t)I2C_ADDRESS, 0x72, I2C_MEMADD_SIZE_8BIT, (uint8_t *)mArray, 2, 100);


			mfault=MAIN_ERR_XPCOMMAND;
			while(mfault!=0 && xptry<100)
			{

			mfault=0;
			//zapnuti zdroje a potvrzeni nastavenych hodnot
			mArray[0]=0x85;
			HAL_I2C_Mem_Write(&hi2c1,(uint16_t)I2C_ADDRESS, 0x7c, I2C_MEMADD_SIZE_8BIT, (uint8_t *)mArray, 1, 100);

			deltime=GetTime();
			//cekani na nastaveni hodnoty zdroje
			while((mArray[0]==0x85) && (GetTime()-deltime<20)){
			HAL_I2C_Mem_Read(&hi2c1,(uint16_t)I2C_ADDRESS, 0x7c, I2C_MEMADD_SIZE_8BIT, (uint8_t *)mArray, 1, 100);
				}

			//kontrola command error
			if((mArray[0]&0x08)==0x08){mfault=
					MAIN_ERR_XPCOMMAND;}
			osDelay(20);
			mArray[0]=0;
			//kontrola zapnuti zdroje
			HAL_I2C_Mem_Read(&hi2c1,(uint16_t)I2C_ADDRESS, 0x6F, I2C_MEMADD_SIZE_8BIT, (uint8_t *)mArray, 1, 100);
			if((mArray[0]&0x10)==0){
				mfault=MAIN_ERR_XPPOWERON;}
			xptry++;
			}

			if(mfault!=0){
				CallValue=mArray[0];
				if(mfault==MAIN_ERR_XPCOMMAND){CalErr=MAIN_ERR_XPCOMMAND;}
				else{CalErr=MAIN_ERR_XPPOWERON;}
				CalErrValue=CallValue;StateTester=MAIN_ERR;SetVoltcraft=FALSE;RegulatorSourceOff();POWEROFFCUT;SET_ERREND;return;
				}

			osDelay(100);
			ReglCharge(ON);
			osDelay(1000);
			ConnectVoltcraftToReglOnOff(ON);
			osDelay(10);
			ReglCharge(OFF);
			}
		else{
			Set_RegulatorPWM(0);
			osDelay(1000);
			ConnectVoltcraftToReglOnOff(OFF);
			osDelay(10);

			RegulatorSourceOff();

			mfault=MAIN_ERR_XPCOMMAND;
			while(mfault!=0 && xptry<100)
			{
			mfault=0;

			deltime=GetTime();
			//cekani na nastaveni hodnoty zdroje
			while((mArray[0]==0x84) && (GetTime()-deltime<20)){
			HAL_I2C_Mem_Read(&hi2c1,(uint16_t)I2C_ADDRESS, 0x7c, I2C_MEMADD_SIZE_8BIT, (uint8_t *)mArray, 1, 100);
			}

			//kontrola command error
			if((mArray[0]&0x08)==0x08){
				mfault=MAIN_ERR_XPCOMMAND;}
			osDelay(20);
			mArray[0]=0x10;
			//kontrola vypnuti zdroje
			HAL_I2C_Mem_Read(&hi2c1,(uint16_t)I2C_ADDRESS, 0x6F, I2C_MEMADD_SIZE_8BIT, (uint8_t *)mArray, 1, 100);
			if((mArray[0]&0x10)==0x10){
				mfault=MAIN_ERR_XPPOWEROFF;}

			xptry++;
			}

			if(mfault!=0){
				CallValue=mArray[0];
				if(mfault==MAIN_ERR_XPCOMMAND){CalErr=MAIN_ERR_XPCOMMAND;}
				else{CalErr=MAIN_ERR_XPPOWEROFF;}
				CalErrValue=CallValue;StateTester=MAIN_ERR;SetVoltcraft=FALSE;RegulatorSourceOff();POWEROFFCUT;SET_ERREND;return;
				}

			ReglDischarge(ON);
			osDelay(2000);
			ReglDischarge(OFF);
			}
		SetVoltcraft=FALSE;
}

//void USBcom_FStopTest(void){CalErr = MAIN_STOP;menuCode=JB_Stop;StateTester=MAIN_STOP_STATE;}


/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{

  /* USER CODE BEGIN 5 */

  /* Infinite loop */
  for(;;)
  {
	  HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
    osDelay(100);
  }
  /* USER CODE END 5 */ 
}

/* USER CODE BEGIN Header_StartStateMachineTask */
/**
* @brief Function implementing the StateMachine thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartStateMachineTask */
void StartStateMachineTask(void const * argument)
{
  /* USER CODE BEGIN StartStateMachineTask */
  /* Infinite loop */

	POWEROFF;
	StateTester=MAIN_WAITING;
	osDelay(100);

  for(;;)
  {

	  switch (StateTester) {

	  		case MAIN_WAITING:CalStep=StateTester;
	  			USBcomReady = TRUE;
	  			break;

	  			//zjisteni jestli je zdroj skutecne vypnuty
	  		case MAIN_ZERO:CalStep=StateTester;
	  			//Insertion();
	  			//StateTester = MAIN_EXTRACT;
	  			RESET_ERREND;

  				CalErr = MAIN_NONE;
  				OI_counter=0;
  				MotorOvercurrent=FALSE;
  				MotorImax = 0;
  				CalErrValue = 0;
  				CallValue=MotorSource.Voltage;
	  			if(CallValue>10) {CalErr=MAIN_ERR_VOLTCRAFT_VOUT;CalErrValue=CallValue;StateTester=MAIN_ERR;POWEROFF;SET_ERREND;break;}
	  			CallValue=MotorSource.Current_A;
	  			if(CallValue>10) {CalErr=MAIN_ERR_VOLTCRAFT_IOUT;CalErrValue=CallValue;StateTester=MAIN_ERR;POWEROFF;SET_ERREND;break;}
	  			if(StateTester!=MAIN_ERR)
	  				{
	  				if(Flag.Insertion){StateTester = MAIN_INSERTION;}
	  				else{StateTester = MAIN_RESISTANCE;}
	  				}

	  			break;

			case MAIN_EXTRACT:
				osDelay(2000);
				Extraction();
				StateTester = MAIN_DATAOK;
				break;

			case MAIN_INSERTION:CalStep=StateTester;
				Insertion();
				if(StateTester!=MAIN_ERR){StateTester = MAIN_RESISTANCE;}
				break;

			case MAIN_RESISTANCE:CalStep=StateTester;
				if(Flag.Resistance){StateTester = MAIN_GOM_SET;}
				else{StateTester = MAIN_START_REGULATOR;}
				break;

			case MAIN_GOM_SET:CalStep=StateTester;
	  			if(Flag.ChangeGOMSetting){StateTester = MAIN_GOM_RESET;}
	  			else{StateTester = MAIN_MEASURE_R_AC;}
				break;


	  		case MAIN_GOM_RESET:CalStep=StateTester;
	  			TRANSMIT_GOM(GOM_RESET);
	  			TRANSMIT_GOM(GOM_FUNCTION_OHM);
	  			TRANSMIT_GOM(GOM_RANGE_AUTO_OFF);
	  			TRANSMIT_GOM(Flag.Range);
	  			TRANSMIT_GOM(GOM_AVERAGE_ON);
	  			TRANSMIT_GOM(GOM_AVERAGE_DATA_10);
	  			osDelay(3000);//musi byt cekacka po kazde zmene range, at zachyti korektni hodnoty
	  			Flag.ChangeGOMSetting=FALSE;
	  			StateTester = MAIN_MEASURE_R_AC;
	  			break;

			case MAIN_MEASURE_R_AC:CalStep=StateTester;
				ConnectVoltcraft(OFF,0,0);
				if(Flag.ResistanceAC)
				{
					Set_GOMConnectAC();
					osDelay(Tset.TimeResMeasure*1000);
					TRANSMIT_GOM(GOM_READ);
					if(GOM_OverRange==TRUE){CalErr=MAIN_ERR_OVERRANGE;CalErrValue=0xffff;StateTester=MAIN_ERR;BREAKTEST;SET_ERREND;break;}
					if(GOM_uOhm<MINUOHM){CalErr=MAIN_ERR_SHORTPHASEAC;CalErrValue=GOM_uOhm;StateTester=MAIN_ERR;BREAKTEST;SET_ERREND;break;}
					GOM_uOhm_ABC[RAC] = GOM_uOhm;
				}
				if(StateTester!=MAIN_ERR){
				StateTester = MAIN_MEASURE_R_AB;
				}
				break;

			case MAIN_MEASURE_R_AB:CalStep=StateTester;
				if(Flag.ResistanceAB)
				{
					Set_GOMConnectAB();
					osDelay(Tset.TimeResMeasure*1000);
					TRANSMIT_GOM(GOM_READ);
					if(GOM_OverRange==TRUE){CalErr=MAIN_ERR_OVERRANGE;CalErrValue=0xffff;StateTester=MAIN_ERR;BREAKTEST;SET_ERREND;break;}
					if(GOM_uOhm<MINUOHM){CalErr=MAIN_ERR_SHORTPHASEAB;CalErrValue=GOM_uOhm;StateTester=MAIN_ERR;BREAKTEST;SET_ERREND;break;}
					GOM_uOhm_ABC[RAB] = GOM_uOhm;
				}
				StateTester = MAIN_MEASURE_R_BC;
				break;

			case MAIN_MEASURE_R_BC:CalStep=StateTester;
				if(Flag.ResistanceBC)
				{
					Set_GOMConnectBC();
					osDelay(Tset.TimeResMeasure*1000);
					TRANSMIT_GOM(GOM_READ);
					if(GOM_OverRange==TRUE){CalErr=MAIN_ERR_OVERRANGE;CalErrValue=0xffff;StateTester=MAIN_ERR;BREAKTEST;SET_ERREND;break;}
					if(GOM_uOhm<MINUOHM){CalErr=MAIN_ERR_SHORTPHASEBC;CalErrValue=GOM_uOhm;StateTester=MAIN_ERR;BREAKTEST;SET_ERREND;break;}
					GOM_uOhm_ABC[RBC] = GOM_uOhm;
				}
				Set_GOMConnectAllPhaseOff();
				osDelay(500);
				StateTester = MAIN_START_REGULATOR;
				break;

			case MAIN_START_REGULATOR:CalStep=StateTester;
				if(Flag.SpinMotor)
				{
					if(Flag.GearVoltage > 0)
					{
						/* --- GEAR WARMUP PHASE ---
						 * Power on at run-in voltage, spin motor continuously.
						 * No measurement is taken here — grease/gear activates. */
						ConnectVoltcraft(ON,Flag.GearVoltage,Flag.Current);
						Set_RegulatorPWM(1000);
						osDelay(1000);
						Set_RegulatorPWM(Flag.Impuls);
						osDelay((uint32_t)Tset.TimeGearRun*1000);

						/* --- VOLTAGE SWITCH (motor keeps spinning) ---
						 * Only update I2C registers — relay and pre-charge are
						 * already active, do NOT call ConnectVoltcraft again. */
						UpdateVoltcraftVoltage(Flag.Voltage, Flag.Current);
						osDelay(500); /* allow supply output to settle */
					}
					else
					{
						/* Standard non-geared motor startup */
						ConnectVoltcraft(ON,Flag.Voltage,Flag.Current);
						Set_RegulatorPWM(1000);
						osDelay(1000);
						Set_RegulatorPWM(Flag.Impuls);
					}

					/* --- MEASUREMENT PHASE --- */
					osDelay((uint32_t)Tset.TimeMotorRun*1000);
					CallValue = RPMval;
					MotorUo = MotorSource.Voltage;
					MotorKV = (CallValue*100/MotorUo);
					MotorIo = MotorSource.Current_A;
					MotorRpm = CallValue;
					MotorDirection = Direction;
					Set_RegulatorPWM(1000);
					osDelay(2000);
					ConnectVoltcraft(OFF,0,1);
				}
				if(Flag.Insertion){Extraction();}
				POWEROFF;
				if(StateTester!=MAIN_ERR){
				StateTester = MAIN_DATAOK;}
				break;

			case MAIN_DATAOK:CalStep=StateTester;
				CalErr = MAIN_CALOK;
				break;

			case MAIN_CALIB:CalStep=StateTester;
				ConnectVoltcraft(OFF,0,1);
				TRANSMIT_GOM(GOM_RESET);
				TRANSMIT_GOM(GOM_FUNCTION_OHM);
				TRANSMIT_GOM(GOM_RANGE_AUTO_ON);
				TRANSMIT_GOM(GOM_AVERAGE_ON);
				TRANSMIT_GOM(GOM_AVERAGE_DATA_10);
				Set_GOMConnectAC();
				osDelay(RSTABLE_TIME_AUTO);
				TRANSMIT_GOM(GOM_READ);
				if(GOM_OverRange==TRUE){CalErr=MAIN_ERR_OVERRANGE;CalErrValue=0xffff;StateTester=MAIN_ERR;BREAKTEST;SET_ERREND;break;}
				Set_GOMConnectBC();
				osDelay(RSTABLE_TIME_AUTO);
				TRANSMIT_GOM(GOM_READ);
				if(GOM_OverRange==TRUE){CalErr=MAIN_ERR_OVERRANGE;CalErrValue=0xffff;StateTester=MAIN_ERR;BREAKTEST;SET_ERREND;break;}
				Set_GOMConnectAB();
				osDelay(RSTABLE_TIME_AUTO);
				TRANSMIT_GOM(GOM_READ);
				if(GOM_OverRange==TRUE){CalErr=MAIN_ERR_OVERRANGE;CalErrValue=0xffff;StateTester=MAIN_ERR;BREAKTEST;SET_ERREND;break;}
				Set_GOMConnectAllPhaseOff();
				POWEROFF;
				if(StateTester!=MAIN_ERR){
				StateTester = MAIN_DATAOK;}
				break;

			case MAIN_INDEPENDENT_INSERTION:CalStep=StateTester;
				Insertion();
				ServoOff();
				if(StateTester!=MAIN_ERR){StateTester = MAIN_DATAOK;}
				break;

			case MAIN_INDEPENDENT_EXTRACTION:CalStep=StateTester;
				Extraction();
				if(StateTester!=MAIN_ERR){StateTester = MAIN_DATAOK;}
				break;

			default:
				break;
			}
		osDelay(100);
  }
  /* USER CODE END StartStateMachineTask */
}

/* USER CODE BEGIN Header_StartFastLoop */
/**
* @brief Function implementing the FastLoop thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartFastLoop */
void StartFastLoop(void const * argument)
{
  /* USER CODE BEGIN StartFastLoop */
  /* Infinite loop */
  uint8_t LCDscreen=1;
  uint16_t timerOK=0;
  //uint16_t decval=0;

  osDelay(50);

  for(;;)
  {
	  	uint16_t VCRAFT_ADCU_local = VCRAFT_ADCU;
	  	uint16_t VCRAFT_ADCI_local = VCRAFT_ADCI;
	  	//uint16_t SERVO_ADCU_local = SERVO_ADCU;
	  	//uint16_t SERVO_ADCI_local = SERVO_ADCI;
	  	uint8_t qArray[]={0,0};


	    //vypocet napeti zdroje
	  	MotorSource.Voltage = ((uint32_t)VCRAFT_ADCU_local * 1326) / 10000;

		//vypocet proudu zdroje
	  	//if((uint32_t)VCRAFT_ADCI_local*71<202847){MotorSource.Current_A=0;} //202847 is the new offset for new MUI. ALWAYS check offsets for new MUI when replacing
	  	//else{MotorSource.Current_A = ((uint32_t)VCRAFT_ADCI_local*71-202847)/1000;}//xx.xA , max je 24,832A

	  	//if((uint32_t)VCRAFT_ADCI_local*405 < 1155465){MotorSource.Current_A=0;}
	  	//else{MotorSource.Current_A = ((uint32_t)VCRAFT_ADCI_local*405-1155465)/1000;}//works good

		// ACS722LLCTR-20AU-T: 0-20A, 132 mV/A, offset = VCC*0.1 = 330mV
       if((uint32_t)VCRAFT_ADCI_local * 64 < 26190){MotorSource.Current_A = 0;}
         else{MotorSource.Current_A = ((uint32_t)VCRAFT_ADCI_local * 64 - 26190) / 1000;}


	  	if(MotorImax<MotorSource.Current_A){MotorImax=MotorSource.Current_A;}

	  	//vypocet napeti na servech
	  	//ServoSource.Voltage = ((uint32_t)SERVO_ADCU_local*1092)/10000;//xx.xV ma byt 75, tedy 7.5V

	  	//vypocet proudu odebiraneho servy
	  	//if((uint32_t)SERVO_ADCI_local*679<386172){ServoSource.Current_A=0;}
	  	//else{ServoSource.Current_A = ((uint32_t)SERVO_ADCI_local*679-386172)/10000;}//xx.xA , max zdroje je 13.5A

        //	if(ServoSource.Voltage>80 || ServoSource.Voltage<70) {SourceServo(OFF);CalErr=MAIN_ERR_SERVO_VCC;CalErrValue=CallValue;StateTester=MAIN_ERR;}
	  	//if(ServoSource.Current_A>80) {
	  		//SourceServo(OFF);CalErr=MAIN_ERR_SERVO_OVERCURRENT;CalErrValue=ServoSource.Current_A;StateTester=MAIN_ERR;SET_ERREND;
	  	//}

	  	if(SetVoltcraft==FALSE){
	  		HAL_I2C_Mem_Read(&hi2c1,(uint16_t)I2C_ADDRESS, 0x6C, I2C_MEMADD_SIZE_8BIT, (uint8_t *)qArray, 1, 100);
	  		if(qArray[0]!=0){

	  			CalErr=MAIN_ERR_XPFAIL;CalErrValue=qArray[0];StateTester=MAIN_ERR;POWEROFF;SET_ERREND;

	  		}
	  	}




	  	//otacky
	  	ICPcnt = __HAL_TIM_GetCounter(&htim2);
	  	if((ICPcnt/90000000)>1){//nuluje otacky pokud je interval casu delsi nez x sekund, pri 42 magnetech je to cca 14ot/min
	  		TIM2Over=TRUE;
	  	}
	  	if(TIM2Over){ICPval=0;RPMval=0;}//rpm will be zero if any signal is comming

	  	else{
	  	    ICPval = __HAL_TIM_GetCompare(&htim2, TIM_CHANNEL_4);
	  	    if(ICPval > 0) {
	  	    	// Remove oldest sample from sum
	  	    	        PeriodSum -= PeriodBuffer[PeriodIndex];
	  	    	        // Add new sample
	  	    	        PeriodBuffer[PeriodIndex] = ICPval;
	  	    	        PeriodSum += ICPval;
	  	    	        // Move to next position
	  	    	        PeriodIndex = (PeriodIndex + 1) % FREQ_AVG_SAMPLES;

	  	    	        // Calculate averaged frequency
	  	    	        uint32_t avgPeriod = PeriodSum / FREQ_AVG_SAMPLES;
	  	    	        RPMval = 90000000 / avgPeriod;  // Averaged frequency
	  	    	    } else {
	  	    	        RPMval = 0;
	  	    	    }
	  	}


	  	//else{
	  		//ICPval = __HAL_TIM_GetCompare(&htim2, TIM_CHANNEL_4);
	  		//RPMval = (uint32_t)((uint64_t)60*90000000/((uint64_t)ICPval*(Tset.Magnets/2)));//RPMval = 60*f/(ICPval*(count of magnets/2)), count of magnets = S or N, S+N=couples
	  	//}
	  	Direction = HAL_GPIO_ReadPin(MotorDirection_GPIO_Port, MotorDirection_Pin);

		//kontrola nactenych tlacitek JB
		if(ButtonsReady){ButtonsReady = FALSE;}//vyrazeni pouzivani tlacitek jetiboxu

	  	clr_LCD_1R();
	  	clr_LCD_2R();
		//nastaveni co se bude zobrazovat na JB
		 switch(menuCode)
		  {

		    case JB_MainMenu:
		    {
		    	if(Btn(BTNRIGHT)){CalErr=MAIN_NONE;StateTester=MAIN_ZERO;lcdrunning=1;menuCode=JB_Running;}
		    	if(Btn(BTNUP)){StateTester=MAIN_CALIB_REQUEST;menuCode=JB_Calib_Request;}
		    	if(Command.Launch){CalErr=MAIN_NONE;StateTester=MAIN_ZERO;lcdrunning=1;menuCode=JB_Running;}
		    	if(Command.LaunchCalibration){CalErr=MAIN_NONE;StateTester=MAIN_CALIB;menuCode=JB_Calib;}
		    	if(Command.Insertion){CalErr=MAIN_NONE;StateTester=MAIN_INDEPENDENT_INSERTION;menuCode=JB_Insertion;}
		    	if(Command.Extraction){CalErr=MAIN_NONE;StateTester=MAIN_INDEPENDENT_EXTRACTION;menuCode=JB_Extraction;}

		    	sprintf(JBscreen,"Stator %5lu",(uint32_t)TIM1->CCR2);
		    	sprintf(JBscreen+16,"%6lu,%7lu",RPMval,ICPcnt);
			  	JBscreen[31] = 126;/*sipka vpravo*/
		      break;
		    }

		    case JB_Running:
   		    {
    		   
	             sprintf(JBscreen,"running/step: %2u",CalStep);
	             sprintf(JBscreen+16,"%5ld",CallValue);
		    	 JBscreen[21]=' ';
		    		         	switch (lcdrunning) {
		    		         		case 1:
		    		         		{
		    		         			MenuTime=GetTime();
		    		         			lcdrunning++;
		    		         		}
		    		     			case 2:
		    		     			{
		    		     				if (GetTime()-MenuTime>RSPEED) {MenuTime=GetTime();lcdrunning++;}
		    		     				break;
		    		     			}
		    		     			case 3:
		    		     			{
		    		     				sprintf(JBscreen+22,"*      ");JBscreen[29]=' ';
		    		     				//osDelay(1000);
		    		     				if (GetTime()-MenuTime>RSPEED) {MenuTime=GetTime();lcdrunning++;}
		    		     				break;
		    		     			}
		    		     			case 4:
		    		     			{
		    		     				sprintf(JBscreen+22,"**     ");JBscreen[29]=' ';
		    		     				if (GetTime()-MenuTime>RSPEED) {MenuTime=GetTime();lcdrunning++;}
		    		     				break;
		    		     			}
		    		     			case 5:
		    		     			{
		    		     				sprintf(JBscreen+22,"***    ");JBscreen[29]=' ';
		    		     				if (GetTime()-MenuTime>RSPEED) {MenuTime=GetTime();lcdrunning++;}
		    		     				break;
		    		     			}
		    		     			case 6:
		    		     			{
		    		     				sprintf(JBscreen+22,"****   ");JBscreen[29]=' ';
		    		     				if (GetTime()-MenuTime>RSPEED) {MenuTime=GetTime();lcdrunning++;}
		    		     				break;
		    		     			}
		    		     			case 7:
		    		     			{
		    		     				sprintf(JBscreen+22,"*****  ");JBscreen[29]=' ';
		    		     				if (GetTime()-MenuTime>RSPEED) {MenuTime=GetTime();lcdrunning++;}
		    		     				break;
		    		     			}
		    		     			case 8:
		    		     			{
		    		     				sprintf(JBscreen+22,"****** ");JBscreen[29]=' ';
		    		     				if (GetTime()-MenuTime>RSPEED) {MenuTime=GetTime();lcdrunning++;}
		    		     				break;
		    		     			}
		    		     			case 9:
		    		     			{
		    		     				sprintf(JBscreen+22,"*******");JBscreen[29]=' ';
		    		     				if (GetTime()-MenuTime>RSPEED) {MenuTime=GetTime();lcdrunning=2;}
		    		     				break;
		    		     			}
		    		     			default:
		    		     			{
		    		     				break;
		    		     			}
		    		     		}
		    		         	sprintf(JBscreen+24,",%5u",hlpCounterRxHrana);

		    		         	if(CalErr!=MAIN_NONE){menuCode=JB_Error;Command.Launch=FALSE;}
		    		         	if(CalErr==MAIN_CALOK){menuCode=JB_TestOK;Command.Launch=FALSE;}
		    		      break;
		    		    }

		    case JB_TestOK:
		    {
		    	if(Btn(BTNRIGHT)){CalErr=MAIN_NONE;StateTester=MAIN_ZERO;lcdrunning=1;menuCode=JB_Running;}
		    	if(Command.Launch){CalErr=MAIN_NONE;StateTester=MAIN_ZERO;lcdrunning=1;menuCode=JB_Running;}
		    	if(Command.LaunchCalibration){CalErr=MAIN_NONE;StateTester=MAIN_CALIB;menuCode=JB_Calib;}
		    	if(Command.Insertion){CalErr=MAIN_NONE;StateTester=MAIN_INDEPENDENT_INSERTION;menuCode=JB_Insertion;}
		    	if(Command.Extraction){CalErr=MAIN_NONE;StateTester=MAIN_INDEPENDENT_EXTRACTION;menuCode=JB_Extraction;}
		    	sprintf(JBscreen,"Stator measure. ");
		    	if(timerOK>=3000){timerOK=0;LCDscreen++;}
		    	else{timerOK+=10;}
	         	switch (LCDscreen) {
	         		case 1:
	         		{
	    		    	sprintf(JBscreen+8,"%6lu",GOM_uOhm_ABC[RBC]);
	    		    	sprintf(JBscreen+16,"%6lu",GOM_uOhm_ABC[RAC]);
	    		    	sprintf(JBscreen+24,"%6lu",GOM_uOhm_ABC[RAB]);
	    		    	break;
	         		}
	     			case 2:
	     			{
	    		    	sprintf(JBscreen+8,"%6lu",MotorKV);
	    		    	sprintf(JBscreen+16,"%6lu",MotorRpm);
	    		    	sprintf(JBscreen+24,"%6lu",MotorIo);
	     				break;
	     			}
	     			case 3:
	     			{
	     				break;
	     			}
	     			case 4:
	     			{
	     				break;
	     			}
	     			case 5:
	     			{
	     				break;
	     			}
	     			case 6:
	     			{
	     				break;
	     			}

	     			default:
	     			{
	     				break;
	     			}
	     		}
		    	if(LCDscreen>2){LCDscreen=1;}

		    	JBscreen[31] = 126;/*sipka vpravo*/
		      break;
		    }

		    case JB_Calib_Request:
		    {
		    	if(Btn(BTNRIGHT)){StateTester=MAIN_CALIB;menuCode=JB_Calib;}
		    	if(Btn(BTNLEFT)){CalErr=MAIN_NONE;StateTester=MAIN_WAITING;menuCode=JB_MainMenu;}
			  	sprintf(JBscreen,"Calibration     ");
			  	sprintf(JBscreen+16,"ShortAllPhases!");
			  	JBscreen[31] = 126;/*sipka vpravo*/
		      break;
		    }

		    case JB_Calib:
		    {
			  	sprintf(JBscreen,"Calibration     ");
			  	sprintf(JBscreen+16,"wait!");
			  	if(CalErr!=MAIN_NONE){menuCode=JB_Error;Command.LaunchCalibration=FALSE;}
	         	if(CalErr==MAIN_CALOK){menuCode=JB_TestOK;Command.LaunchCalibration=FALSE;}
		      break;
		    }

		    case JB_Insertion:
		    {
			  	sprintf(JBscreen,"Insertion       ");
			  	sprintf(JBscreen+16,"wait!");
			  	if(CalErr!=MAIN_NONE){menuCode=JB_Error;Command.Insertion=FALSE;}
	         	if(CalErr==MAIN_CALOK){menuCode=JB_TestOK;Command.Insertion=FALSE;}
		      break;
		    }

		    case JB_Extraction:
		    {
			  	sprintf(JBscreen,"Extraction      ");
			  	sprintf(JBscreen+16,"wait!");
			  	if(CalErr!=MAIN_NONE){menuCode=JB_Error;Command.Extraction=FALSE;}
	         	if(CalErr==MAIN_CALOK){menuCode=JB_TestOK;Command.Extraction=FALSE;}
		      break;
		    }

		    case JB_Error:
		    {
		    	if(ErrEnd){RESET_ERREND;menuCode=JB_Error_End;}
		    	sprintf(JBscreen,"Error process   ");
		    	sprintf(JBscreen+16,"wait!");
		    	sprintf(JBscreen+24,"ERR:%2u",CalErr);
		      break;
		    }


		    case JB_Error_End:
		    {
		    	CalStep=MAIN_ERR;
		    	if(Btn(BTNRIGHT)){CalStep=0;CalErr=MAIN_NONE;StateTester=MAIN_ZERO;lcdrunning=1;menuCode=JB_Running;}
		    	if(Command.Launch){CalStep=0;CalErr=MAIN_NONE;StateTester=MAIN_ZERO;lcdrunning=1;menuCode=JB_Running;}
		    	if(Command.LaunchCalibration){CalStep=0;CalErr=MAIN_NONE;StateTester=MAIN_CALIB;menuCode=JB_Calib;}
		    	if(Command.Insertion){CalStep=0;CalErr=MAIN_NONE;StateTester=MAIN_INDEPENDENT_INSERTION;menuCode=JB_Insertion;}
		    	if(Command.Extraction){CalStep=0;CalErr=MAIN_NONE;StateTester=MAIN_INDEPENDENT_EXTRACTION;menuCode=JB_Extraction;}
		    	if(Btn(BTNUP)){CalStep=0;CalErr=MAIN_NONE;StateTester=MAIN_CALIB_REQUEST;menuCode=JB_Calib_Request;}
		    	if(CalErr>9)
		    	{
		    	sprintf(JBscreen,"ERROR : -%2u-",CalErr);
		    	JBscreen[12]=' ';
		    	}
		    	else
		    	{sprintf(JBscreen,"ERROR : -%1u-",CalErr);
		    	JBscreen[11]=' ';
		    	}
		    	sprintf(JBscreen+16,"val=%5ld Start",CalErrValue);
		    	JBscreen[31] = 126;/*sipka vpravo*/
		      break;
		    }

		    default:
		    {
		      menuCode = JB_Running;
		      break;
		    }
		  }

		//vypocet provadim co 10ms
		osDelay(10);
  }
  /* USER CODE END StartFastLoop */
}

/* USER CODE BEGIN Header_StartJetibox */
/**
* @brief Function implementing the Jetibox thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartJetibox */
void StartJetibox(void const * argument)
{
  /* USER CODE BEGIN StartJetibox */
  /* Infinite loop */
	clr_LCD_1R();
	clr_LCD_2R();
	JBpin_for_Tx();
  for(;;)
  {
	SendJBDisplay();
    osDelay(70);
  }
  /* USER CODE END StartJetibox */
}

/* USER CODE BEGIN Header_StartUSBconnection */
/**
* @brief Function implementing the USBconnection thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartUSBconnection */
void StartUSBconnection(void const * argument)
{
  /* USER CODE BEGIN StartUSBconnection */
  /* Infinite loop */
  for(;;)
  {
	  if(USBcomReady){
	USBTransfer();
	  }
	//HAL_UART_Receive_IT(&huart2,GOM_hlpbuffer,GOM_RXBUFFER_IT_SIZE);
    osDelay(5);
  }
  /* USER CODE END StartUSBconnection */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
  if (htim->Instance == TIM6) {
	  systime++;
  }

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
