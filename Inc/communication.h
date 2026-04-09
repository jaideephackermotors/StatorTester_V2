#ifndef COMMUNICATION_H
#define COMMUNICATION_H
#include <stdint.h>
#include <math.h>
#include <string.h>
#include "globalsw.h"
//#include "stringcpp.h"









typedef enum
{

	GOM_IDENTIFICATION=1,
	GOM_READ,
	GOM_RANGE_AUTO_ON,
	GOM_RANGE_AUTO_OFF,
	GOM_RANGE_AUTO_CHECK,
	GOM_RANGE_SET_CHECK,
	GOM_RANGE_SET_5mOHM,
	GOM_RANGE_SET_50mOHM,
	GOM_RANGE_SET_500mOHM,
	GOM_RANGE_SET_5OHM,
	GOM_RANGE_SET_50OHM,
	GOM_RANGE_SET_500OHM,
	GOM_RESET,
	GOM_FUNCTION_OHM,
	GOM_FUNCTION_CHECK,
	GOM_AVERAGE_ON,
	GOM_AVERAGE_CHECK,
	GOM_AVERAGE_DATA_10,
	GOM_AVERAGE_DATA_CHECK,
	GOM_QUESTIONABLE_STATUS_ENABLE_OVERLOAD,
	GOM_QUESTIONABLE_STATUS_ENABLE_CHECK,
	GOM_QUESTIONABLE_STATUS_EVENT,
	GOM_EVENT_ERRORS_ENABLE,
	GOM_EVENT_ERRORS_ENABLE_CHECK,
	GOM_EVENT_ERRORS,
	GOM_EVENT_ERRORS_CLEAR,
	GOM_COMMANDS_SIZE


}gom;

typedef enum
{

	XP_GET_TEMPERATURE=0,
	XP_COMMANDS_SIZE
}xp;

typedef enum
{
	uOhm=0,
	mOhm,
	Ohm
}unit;

typedef enum
{
	USBcom_LaunchTest=0x4d53,
	USBcom_StopTest,
	USBcom_config_UI,
	USBcom_config_Impulse,
	USBcom_GetStatus,
	USBcom_GetResult,
	USBcom_GetVersion,
	USBcom_Insertion,
	USBcom_Extraction,
	USBcom_GetCalibData,
	USBcom_SetCalibDataRAC,
	USBcom_SetCalibDataRBC,
	USBcom_SetCalibDataRAB,
	USBcom_Launch_Calibration,
	USBcom_ConfigTester,
	USBcom_ConfigTesterTimers,
	USBcom_Default,

}USBcom;

typedef enum
{
	USBcomAnswer_LaunchTest=0x2A53,
	USBcomAnswer_StopTest,
	USBcomAnswer_config_UI,
	USBcomAnswer_config_Impulse,
	USBcomAnswer_GetStatus,
	USBcomAnswer_GetResult,
	USBcomAnswer_GetVersion,
	USBcomAnswer_Insertion,
	USBcomAnswer_Extraction,
	USBcomAnswer_GetCalibData,
	USBcomAnswer_SetCalibDataRAC,
	USBcomAnswer_SetCalibDataRBC,
	USBcomAnswer_SetCalibDataRAB,
	USBcomAnswer_Launch_Calibration,
	USBcomAnswer_ConfigTester,
	USBcomAnswer_ConfigTesterTimers,
}USBcomAnswer;

typedef struct flagy {
	bool Insertion;
	bool Resistance;
	bool ChangeGOMSetting;
	bool ResistanceAC;
	bool ResistanceBC;
	bool ResistanceAB;
	//bool Star;
	bool SpinMotor;
	gom Range;
	uint16_t Voltage;
	uint16_t Current;
	uint16_t Impuls;
}FLAGY;

typedef struct commands {
	bool Launch;
	bool LaunchCalibration;
	bool Insertion;
	bool Extraction;
}COMMANDS;


typedef struct tset {
	uint8_t Magnets;
	uint16_t TimeMotorRun;
	uint16_t TimeResMeasure;
}TSET;

//GOM_RXBUFFER_SIZE musi byt mensi nez GOM_RXBUFFER_IT_SIZE
#define GOM_RXBUFFER_SIZE		50//velikost bufferu pro prijem, musi byt vetsi nez max predpokladana delka
#define I2C_ADDRESS				(0x50<<1)//1010 000


bool Transmit(gom tosend);
void USBTransfer(void);
void USBDataResult(void);

#endif