/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H__
#define __MAIN_H__

/* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/
#define JBPeriod 9277

#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define ADC_U_Serva_Pin GPIO_PIN_0
#define ADC_U_Serva_GPIO_Port GPIOC
#define VOLTCRAFT_OutputOnOff_Pin GPIO_PIN_1
#define VOLTCRAFT_OutputOnOff_GPIO_Port GPIOC
#define ADC_I_Serva_Pin GPIO_PIN_3
#define ADC_I_Serva_GPIO_Port GPIOC
#define GOM_UART4_TX_Pin GPIO_PIN_0
#define GOM_UART4_TX_GPIO_Port GPIOA
#define GOM_UART4_RX_Pin GPIO_PIN_1
#define GOM_UART4_RX_GPIO_Port GPIOA
#define Pi_USART2_TX_Pin GPIO_PIN_2
#define Pi_USART2_TX_GPIO_Port GPIOA
#define Pi_USART2_RX_Pin GPIO_PIN_3
#define Pi_USART2_RX_GPIO_Port GPIOA
#define ADC_U_Regulator_Pin GPIO_PIN_4
#define ADC_U_Regulator_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define ADC_I_Regulator_Pin GPIO_PIN_6
#define ADC_I_Regulator_GPIO_Port GPIOA
#define Rele4_S_Pin GPIO_PIN_7
#define Rele4_S_GPIO_Port GPIOA
#define Rele1_R_Pin GPIO_PIN_5
#define Rele1_R_GPIO_Port GPIOC
#define Jetibox_Pin GPIO_PIN_1
#define Jetibox_GPIO_Port GPIOB
#define Jetibox_EXTI_IRQn EXTI1_IRQn
#define ICP_RPM_Pin GPIO_PIN_2
#define ICP_RPM_GPIO_Port GPIOB
#define Rele3_S_Pin GPIO_PIN_10
#define Rele3_S_GPIO_Port GPIOB
#define Rele4_R_Pin GPIO_PIN_12
#define Rele4_R_GPIO_Port GPIOB
#define MotorDirection_Pin GPIO_PIN_13
#define MotorDirection_GPIO_Port GPIOB
#define Rele3_R_Pin GPIO_PIN_15
#define Rele3_R_GPIO_Port GPIOB
#define PWM_Voltage_Pin GPIO_PIN_6
#define PWM_Voltage_GPIO_Port GPIOC
#define PWM_Current_Pin GPIO_PIN_7
#define PWM_Current_GPIO_Port GPIOC
#define Rele2_R_Pin GPIO_PIN_8
#define Rele2_R_GPIO_Port GPIOC
#define Rele2_S_Pin GPIO_PIN_9
#define Rele2_S_GPIO_Port GPIOC
#define PWM_Regulator_Pin GPIO_PIN_8
#define PWM_Regulator_GPIO_Port GPIOA
#define PWM_OpenCloseRotor_Pin GPIO_PIN_9
#define PWM_OpenCloseRotor_GPIO_Port GPIOA
#define PWM_StatorSecure_Pin GPIO_PIN_10
#define PWM_StatorSecure_GPIO_Port GPIOA
#define VCC_ServoOnOff_Pin GPIO_PIN_12
#define VCC_ServoOnOff_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define ReglDischarge_Pin GPIO_PIN_10
#define ReglDischarge_GPIO_Port GPIOC
#define ReglCharge_Pin GPIO_PIN_11
#define ReglCharge_GPIO_Port GPIOC
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define Rele1_S_Pin GPIO_PIN_6
#define Rele1_S_GPIO_Port GPIOB
#define ConnectVoltcraftToReglOnOff_Pin GPIO_PIN_7
#define ConnectVoltcraftToReglOnOff_GPIO_Port GPIOB
#define POWER_SCL_Pin GPIO_PIN_8
#define POWER_SCL_GPIO_Port GPIOB
#define POWER_SDA_Pin GPIO_PIN_9
#define POWER_SDA_GPIO_Port GPIOB
#define FREQ_AVG_SAMPLES 40  // Number of cycles to average the frequency



/* USER CODE END Private defines */

/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the 
  *        HAL drivers code
  */
/* #define USE_FULL_ASSERT    1U */

/* USER CODE BEGIN Private defines */

#define ADC1Inputs				4//count of used channels of ADC1

#define D_DivNo					1024//pocet vzorku ze ktereho se pocita prumer ADC, jedno mereni cca 0.4us
#define HCLKAPB2				((uint32_t)180000000)//systemova frekvence pro Timer8

//PWM pro zdroj Voltcraft
#define TIM8_PRESCALER			0 //presclaer navysim jen pokud mi vychazi perioda vyssi nez 65535 (16bit)
#define TIM8FREQUENCY			( HCLKAPB2 / (TIM8_PRESCALER + 1))
#define PWMTIM8FREQUENCY		24000//pozadovana frekvence PWM v HZ
#define TIM8_PERIOD				((TIM8FREQUENCY / PWMTIM8FREQUENCY) - 1)

//PWM pro serva
#define PWMTIM1FREQUENCY		50//pozadovana frekvence PWM v HZ, 100Hz = 10ms
#define PWMTIM1RESOLUTION		20000//rozliseni periody
#define COUNTERTIM1FREQUENCY	(uint32_t)(PWMTIM1FREQUENCY*PWMTIM1RESOLUTION)
#define TIM1_PRESCALER			(uint32_t)((HCLKAPB2/COUNTERTIM1FREQUENCY) - 1)
#define TIM1_PERIOD				(uint32_t)((COUNTERTIM1FREQUENCY / PWMTIM1FREQUENCY) - 1)

#define MAXIMUM_SOURCE_VOLTAGE	600//nastavitelne maximum napeti, 600=60V
#define MAXIMUM_SOURCE_CURRENT	100//nastavitelne maximum napeti, 100=10A
#define MAXIMUM_REGULATOR_PWM	2000//maximalni delka impulsu

#define SourceVoltcraft(able)					(((able) == (ON))? (HAL_GPIO_WritePin(VOLTCRAFT_OutputOnOff_GPIO_Port, VOLTCRAFT_OutputOnOff_Pin, GPIO_PIN_SET)) : (HAL_GPIO_WritePin(VOLTCRAFT_OutputOnOff_GPIO_Port, VOLTCRAFT_OutputOnOff_Pin, GPIO_PIN_RESET)))
#define ReglCharge(able)						(((able) == (ON))? (HAL_GPIO_WritePin(ReglCharge_GPIO_Port, ReglCharge_Pin, GPIO_PIN_SET)) : (HAL_GPIO_WritePin(ReglCharge_GPIO_Port, ReglCharge_Pin, GPIO_PIN_RESET)))
#define ReglDischarge(able)						(((able) == (ON))? (HAL_GPIO_WritePin(ReglDischarge_GPIO_Port, ReglDischarge_Pin, GPIO_PIN_SET)) : (HAL_GPIO_WritePin(ReglDischarge_GPIO_Port, ReglDischarge_Pin, GPIO_PIN_RESET)))
#define ConnectVoltcraftToReglOnOff(able)		(((able) == (ON))? (HAL_GPIO_WritePin(ConnectVoltcraftToReglOnOff_GPIO_Port, ConnectVoltcraftToReglOnOff_Pin, GPIO_PIN_SET)) : (HAL_GPIO_WritePin(ConnectVoltcraftToReglOnOff_GPIO_Port, ConnectVoltcraftToReglOnOff_Pin, GPIO_PIN_RESET)))
#define SourceServo(able)						(((able) == (ON))? (HAL_GPIO_WritePin(VCC_ServoOnOff_GPIO_Port, VCC_ServoOnOff_Pin, GPIO_PIN_SET)) : (HAL_GPIO_WritePin(VCC_ServoOnOff_GPIO_Port, VCC_ServoOnOff_Pin, GPIO_PIN_RESET)))

//RELE
#define RelayTime			200//ms, it has to be much more than 100ms!!
#define RSTABLE_TIME		3000//stabilizacni cas po pripojeni relatek pred ctenim hodnoty odporu vinuti
#define RSTABLE_TIME_AUTO	6000//stabilizacni cas po pripojeni relatek pred ctenim hodnoty odporu vinuti pri Autorange

#define MINUOHM				1000//minimalni mereny odpor vinuti, pod ktery se povazuje zkrat vinuti, v uOhm

//current musi byt nastaven do 1, jinak preblikava ze je ve zkratu
#define POWEROFF		ConnectVoltcraft(OFF,0,0);\
						Set_GOMConnectAllPhaseOff();\
						ServoOff();

#define BREAKTEST		ConnectVoltcraft(OFF,0,0);\
						Set_GOMConnectAllPhaseOff();\
						Extraction();\
						ServoOff();

#define HARDBREAKTEST	ConnectVoltcraft(OFF,0,0);\
						Set_GOMConnectAllPhaseOff();\
						ServoOff();

#define POWEROFFCUT		Set_GOMConnectAllPhaseOff();\
						ServoOff();


#define JB_PACKETS	33
#define JB_Output(able)		(((able) == (ON))? (HAL_GPIO_WritePin(Jetibox_GPIO_Port, Jetibox_Pin, GPIO_PIN_SET)) : (HAL_GPIO_WritePin(Jetibox_GPIO_Port, Jetibox_Pin, GPIO_PIN_RESET)))

#define TRANSMIT_GOM(able)	  			if(Transmit(able)){}else{CalErr=able;StateTester=MAIN_ERR;BREAKTEST;SET_ERREND;break;}


#define SET_ERREND			ErrEnd=TRUE;
#define RESET_ERREND		ErrEnd=FALSE;

//JB menu
#define	RSPEED				 100//v ms

//MAIN states
typedef enum
{
BTNRIGHT=0b1000,
BTNLEFT=0b0001,
BTNUP=0b0100,
BTNDOWN=0b0010
}BTN;

//MAIN states
typedef enum
{

MAIN_WAITING=0,
MAIN_ZERO,

MAIN_INSERTION,
MAIN_RESISTANCE,
MAIN_GOM_SET,
MAIN_GOM_RESET,
MAIN_MEASURE_R_AC,
MAIN_MEASURE_R_AB,
MAIN_MEASURE_R_BC,
MAIN_EXTRACT,
MAIN_START_REGULATOR,
MAXCALSTEP,

MAIN_CALIB_REQUEST,
MAIN_CALIB,

MAIN_INDEPENDENT_INSERTION,
MAIN_INDEPENDENT_EXTRACTION,

//MAIN_STOP_STATE,
MAIN_DATAOK=150,
MAIN_ERR=200,
}StateCal;

//MAIN Errors, errors lower than 40 are equal gom position in COMMUNICATION_H
typedef enum
{
MAIN_NONE=0,
//MAIN_STOP,
MAIN_ERR_OVERRANGE=50,
MAIN_ERR_VOLTCRAFT_VOUT,
MAIN_ERR_VOLTCRAFT_IOUT,
MAIN_ERR_SERVO_VCC,
MAIN_ERR_SERVO_OVERCURRENT,
MAIN_ERR_MOTOR_OVERCURRENT,
MAIN_ERR_SHORTPHASEAC,
MAIN_ERR_SHORTPHASEBC,
MAIN_ERR_SHORTPHASEAB,
MAIN_ERR_XPCOMMAND,
MAIN_ERR_XPPOWERON,
MAIN_ERR_XPPOWEROFF,
MAIN_ERR_XPFAIL,


MAIN_PUSHBUTTON1,
MAIN_PUSHBUTTON2,
MAIN_PUSHBUTTON3,
MAIN_ERR_TIMEOUT_SEPARATOR,
MAIN_ERR_TIMEOUT1,
MAIN_CALOK=150,
}CalibrationError;

typedef enum
{
  JB_MainMenu=0,
  JB_Running,

  JB_Calib_Request,
  JB_Calib,

  JB_Insertion,
  JB_Extraction,

  //Aktualni hodnoty
  JB_Error,
  JB_Error_End,
  JB_TestOK,
  JB_Stop,
  JB_Command1,
  JB_Command2,
  JB_Command3,
  //Pocet polozek menu
  JB_MENU_SIZE
}JetiboxMenu;


/* USER CODE END Private defines */

#ifdef __cplusplus
 extern "C" {
#endif
void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)
#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
