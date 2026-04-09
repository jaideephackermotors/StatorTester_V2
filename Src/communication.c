#include "communication.h"
#include <stdint.h>
#include <math.h>
#include <string.h>
#include <malloc.h>
#include "stm32f4xx_hal.h"
#include "globalsw.h"
#include <stdlib.h>
#include <str2dbl.h>
#include "hlpfunctions.h"
#include "platform_config.h"

#define GOM_TIMEOUT				100//tiemout pro prijem 1 zpravy v ms
#define GOM_TRY_MAX				10//kolikrat zkusi poslat a prijmout data nez zahlasi error
#define GOM_RXBUFFER_IT_SIZE	60//velikost dat ktera jsou pro prichod akceptovana

#define TRANSMIT(com)	sizemsg=0;\
						s=concat(GOM_Command[com],LF,&sizemsg);\
						HAL_UART_Transmit(&huart4, (uint8_t *)s, sizemsg, 0xffff);\
						free(s);// deallocate the string

#define USB_RXBUFFER_MAX_SIZE	20//max velikost bufferu pro prijem dat
#define USB_RXBUFFER_DATA_SIZE	8//velikost dat ktera chci prijmout
#define USB_TXBUFFER_MAX_SIZE	100//max velikost bufferu pro odchozi data

struct flagy Flag={1,1,1,1,1,1,1,GOM_RANGE_SET_50mOHM,150,230,2000};
struct tset Tset={2,4,3};
struct commands Command={0,0};

char LF[]="\n";
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart2;
extern bool gom_usartfinal;
uint8_t GOM_hlpbuffer[GOM_RXBUFFER_IT_SIZE];
extern char GOM_rxbuffer[];
char GOM_out[GOM_RXBUFFER_SIZE];
extern uint8_t gom_buffIndex;
uint16_t status=0;
bool GOM_OverRange = FALSE;
int32_t GOM_val=0;
float GOM_rel_val=0;
uint32_t GOM_uOhm=0;
gom GOM_RangeSet=GOM_RANGE_SET_5mOHM;
extern __IO uint32_t MotorKV;
extern __IO uint32_t MotorIo;
extern __IO uint32_t MotorRpm;
extern __IO uint32_t MotorUo;
extern __IO uint32_t MotorImax;
extern bool MotorDirection;
extern bool MotorOvercurrent;
extern __IO uint32_t GOM_uOhm_ABC[];

extern uint64_t systime;

extern resistance Res;
extern uint8_t StateTester;
extern uint8_t CalErr;
extern __IO int32_t CalErrValue;
extern __IO uint8_t CalStep;
float CalibZero[]={0,0,0};
uint8_t *CalibRAC = (uint8_t *)&CalibZero[0];

extern void USBcom_FStopTest();
extern void Insertion();
extern void Extraction();

__IO uint16_t countedCRC = 0;

uint8_t USB_Rxbuffer[USB_RXBUFFER_MAX_SIZE];
uint8_t USB_Txbuffer[USB_TXBUFFER_MAX_SIZE];

uint8_t USB_GetVersion(uint8_t * outputBuffer);
uint8_t USB_SendConfirmation(uint8_t * outputBuffer, uint16_t request, bool error);
uint8_t USB_GetCalibData(uint8_t * outputBuffer);
uint8_t USB_GetStatus(uint8_t * outputBuffer);
uint8_t USB_GetResult(uint8_t * outputBuffer);

static const char* const GOM_Command[GOM_COMMANDS_SIZE]=
//prikazy mOhm metru GOM
{ "none",
  "*idn?",
  "read?",
  "SENSE:AUTO ON",
  "SENSE:AUTO OFF",
  "SENSE:AUTO?",
  "SENSE:RANGE?",
  //"SENSE:RANGE 0.005",
  "SENSE:RANGE 5E-3",
  //"SENSE:RANGE 0.05",
  "SENSE:RANGE 5E-2",
  "SENSE:RANGE 5E-1",
  "SENSE:RANGE 5E+0",
  "SENSE:RANGE 5E+1",
  "SENSE:RANGE 5E+2",
  "*RST",
  "SENSE:FUNCTION OHM",
  "SENSE:FUNCTION?",
  "SYSTEM:AVERAGE:STATE ON",
  "SYSTEM:AVERAGE:STATE?",
  "SYSTEM:AVERAGE:DATA 10",
  "SYSTEM:AVERAGE:DATA?",
  "STATus:QUEStionable:ENABle 512",
  "STATus:QUEStionable:ENABle?",
  "STATus:QUEStionable:EVENt?",
  "*ESE 60",
  "*ESE?",
  "*ESR?",
  "*CLS",
};


char* concat(const char *s1, const char *s2, uint8_t *csize)
{//spoji dohromady dva textove retezce a ulozi jejich delku bez nuly do ukazatele csize
    const size_t len1 = strlen(s1);
    const size_t len2 = strlen(s2);
    *csize = (len1 + len2);
    char *result = malloc(*csize + 1); // +1 for the null-terminator
    memcpy(result, s1, len1);
    memcpy(result + len1, s2, len2 + 1); // +1 to copy the null-terminator
    return result;
}

bool Transmit(gom tosend)
{
	char* s;
	uint8_t sizemsg=0;
	uint8_t try_receive=0;
	gom_usartfinal=FALSE;
	bool result=TRUE;
	bool error_event=TRUE;


	while(error_event==TRUE && try_receive<GOM_TRY_MAX)
	{
	error_event=FALSE;
	//poslu adekvatni data - prikaz - do GOM
	switch (tosend)
		{
		case GOM_IDENTIFICATION:
			TRANSMIT(GOM_IDENTIFICATION);
		break;

		case GOM_READ:
			TRANSMIT(GOM_READ);
		break;

		case GOM_RANGE_AUTO_ON:
			TRANSMIT(GOM_RANGE_AUTO_ON);
			TRANSMIT(GOM_RANGE_AUTO_CHECK);
		break;

		case GOM_RANGE_AUTO_OFF:
			TRANSMIT(GOM_RANGE_AUTO_OFF);
			TRANSMIT(GOM_RANGE_AUTO_CHECK);
		break;

		case GOM_RANGE_SET_5mOHM:
			TRANSMIT(GOM_RANGE_SET_5mOHM);
			TRANSMIT(GOM_RANGE_SET_CHECK);
		break;

		case GOM_RANGE_SET_50mOHM:
			TRANSMIT(GOM_RANGE_SET_50mOHM);
			TRANSMIT(GOM_RANGE_SET_CHECK);
		break;

		case GOM_RANGE_SET_500mOHM:
			TRANSMIT(GOM_RANGE_SET_500mOHM);
			TRANSMIT(GOM_RANGE_SET_CHECK);
		break;

		case GOM_RANGE_SET_5OHM:
			TRANSMIT(GOM_RANGE_SET_5OHM);
			TRANSMIT(GOM_RANGE_SET_CHECK);
		break;

		case GOM_RANGE_SET_50OHM:
			TRANSMIT(GOM_RANGE_SET_50OHM);
			TRANSMIT(GOM_RANGE_SET_CHECK);
		break;

		case GOM_RANGE_SET_500OHM:
			TRANSMIT(GOM_RANGE_SET_500OHM);
			TRANSMIT(GOM_RANGE_SET_CHECK);
		break;

		case GOM_RESET:
			TRANSMIT(GOM_RESET);
			osDelay(500); //added delay for better reliability
			gom_usartfinal=TRUE;//=bez cekani prijmu data
		break;

		case GOM_FUNCTION_OHM:
			TRANSMIT(GOM_FUNCTION_OHM);
			TRANSMIT(GOM_FUNCTION_CHECK);
		break;

		case GOM_AVERAGE_ON:
			TRANSMIT(GOM_AVERAGE_ON);
			TRANSMIT(GOM_AVERAGE_CHECK);
		break;

		case GOM_AVERAGE_DATA_10:
			TRANSMIT(GOM_AVERAGE_DATA_10);
			TRANSMIT(GOM_AVERAGE_DATA_CHECK);
		break;

		case GOM_QUESTIONABLE_STATUS_ENABLE_OVERLOAD:
			TRANSMIT(GOM_QUESTIONABLE_STATUS_ENABLE_OVERLOAD);
			TRANSMIT(GOM_QUESTIONABLE_STATUS_ENABLE_CHECK);
		break;

		case GOM_QUESTIONABLE_STATUS_EVENT:
			TRANSMIT(GOM_QUESTIONABLE_STATUS_EVENT);
		break;

		case GOM_EVENT_ERRORS_ENABLE:
			TRANSMIT(GOM_EVENT_ERRORS_ENABLE);
			TRANSMIT(GOM_EVENT_ERRORS_ENABLE_CHECK);
		break;

		case GOM_EVENT_ERRORS:
			TRANSMIT(GOM_EVENT_ERRORS);
		break;

		default:
		break;
		}

	  uint64_t RxTime = GetTime();
	  memset(GOM_rxbuffer,0,GOM_RXBUFFER_SIZE);//vynulovani bufferu pro prijem dat z GOM
	  gom_buffIndex=0;//vynuluju ukazatel posledniho prichoziho znaku do bufferu

	  //testuju jestli prisel ukoncovaci znak LF = 0x0A nebo jestli vyprsel casovy limit prichodu zpravy
	  while(gom_usartfinal==FALSE && (GetTime()-RxTime<GOM_TIMEOUT))
	  {
	  HAL_UART_Receive_IT(&huart4,GOM_hlpbuffer,GOM_RXBUFFER_IT_SIZE);
	  }
	  HAL_UART_AbortReceive_IT(&huart4);

	  if(gom_usartfinal==FALSE){error_event=TRUE;}//doslo k chybe, cyklus se bude opakovat
	  gom_usartfinal = FALSE;

	  //otestuju jestli pri komunikaci doslo na strane GOM k chybe
	  memcpy(GOM_out, GOM_rxbuffer, GOM_RXBUFFER_SIZE);
	  memset(GOM_rxbuffer,0,GOM_RXBUFFER_SIZE);
	  gom_buffIndex=0;//vynuluju ukazatel posledniho prichoziho znaku do bufferu
	  TRANSMIT(GOM_EVENT_ERRORS);//vycteni eventu chyb
	  //testuju jestli prisel ukoncovaci znak LF = 0x0A nebo jestli vyprsel casovy limit prichodu zpravy
	  while(gom_usartfinal==FALSE && (GetTime()-RxTime<GOM_TIMEOUT))
	  {
	  HAL_UART_Receive_IT(&huart4,GOM_hlpbuffer,GOM_RXBUFFER_IT_SIZE);
	  }
	  HAL_UART_AbortReceive_IT(&huart4);

	  if(gom_usartfinal==FALSE){error_event=TRUE;}//doslo k chybe, cyklus se bude opakovat
	  gom_usartfinal = FALSE;

	  status=((uint16_t)atoi(GOM_rxbuffer))&0x20;
	  if(status){
		  TRANSMIT(GOM_EVENT_ERRORS_CLEAR);error_event=TRUE;//doslo k chybe, cyklus se bude opakovat
	  }


	  try_receive++;//pokud neprisla spravna data nebo vyprsel datovy limit, opakuju vyzvu do GOM_TRY_MAX
	}


	if(try_receive==GOM_TRY_MAX){return FALSE;}//neprisla adekvatni zprava, vracim false

	switch (tosend)
		{

		case GOM_IDENTIFICATION:
			//zpracovani dat?
		break;

		//vrati FALSE pokud poslana hodnota a prijata na dotaz nejsou shodne, FALSE vyvola Error v hlavnim programu
		case GOM_FUNCTION_OHM:
			if(strcmp(GOM_out,"OHM\n") != 0){result=FALSE;}
		break;

		case GOM_RANGE_AUTO_ON:
			if(strcmp(GOM_out,"1\n") != 0){result=FALSE;}
		break;

		case GOM_RANGE_AUTO_OFF:
			if(strcmp(GOM_out,"0\n") != 0){result=FALSE;}
		break;

		case GOM_RANGE_SET_5mOHM:
			GOM_RangeSet=tosend;
			if(strcmp(GOM_out,"5.0000E-3\n") != 0){result=FALSE;}
		break;

		case GOM_RANGE_SET_50mOHM:
			GOM_RangeSet=tosend;
			if(strcmp(GOM_out,"5.0000E-2\n") != 0){result=FALSE;}
		break;

		case GOM_RANGE_SET_500mOHM:
			GOM_RangeSet=tosend;
			if(strcmp(GOM_out,"5.0000E-1\n") != 0){result=FALSE;}
		break;

		case GOM_RANGE_SET_5OHM:
			GOM_RangeSet=tosend;
			if(strcmp(GOM_out,"5.0000E+0\n") != 0){result=FALSE;}
		break;

		case GOM_RANGE_SET_50OHM:
			GOM_RangeSet=tosend;
			if(strcmp(GOM_out,"5.0000E+1\n") != 0){result=FALSE;}
		break;

		case GOM_RANGE_SET_500OHM:
			GOM_RangeSet=tosend;
			if(strcmp(GOM_out,"5.0000E+2\n") != 0){result=FALSE;}
		break;

		case GOM_AVERAGE_ON:
			if(strcmp(GOM_out,"1\n") != 0){result=FALSE;}
		break;

		case GOM_AVERAGE_DATA_10:
			if(strcmp(GOM_out,"010\n") != 0){result=FALSE;}
		break;

		case GOM_QUESTIONABLE_STATUS_ENABLE_OVERLOAD:
			if(strcmp(GOM_out,"00512\n") != 0){result=FALSE;}
		break;

		case GOM_EVENT_ERRORS_ENABLE:
			if(strcmp(GOM_out,"060\n") != 0){result=FALSE;}
		break;

		case GOM_QUESTIONABLE_STATUS_EVENT:
			status=(uint16_t)atoi(GOM_out)&512;
			if(status){GOM_OverRange=TRUE;}else{GOM_OverRange=FALSE;}
		break;

		case GOM_READ:

			GOM_rel_val = atof(GOM_out);
			if(GOM_rel_val>500){GOM_OverRange=TRUE;}else{GOM_OverRange=FALSE;}

			if(StateTester==MAIN_CALIB && GOM_OverRange==FALSE){
				switch (Res)
						{
							case RAC:CalibZero[RAC]=GOM_rel_val;break;
							case RBC:CalibZero[RBC]=GOM_rel_val;break;
							case RAB:CalibZero[RAB]=GOM_rel_val;break;
							default:
							break;
						}
			}

			switch (Res)
			{	//tady odectu od hodnoty kalibracni offset
				case RAC:GOM_rel_val=GOM_rel_val-CalibZero[RAC];break;
				case RBC:GOM_rel_val=GOM_rel_val-CalibZero[RBC];break;
				case RAB:GOM_rel_val=GOM_rel_val-CalibZero[RAB];break;
				default:
				break;
			}
			GOM_uOhm=0;
			if(GOM_rel_val>0 && GOM_OverRange==FALSE)
			{
				switch (GOM_RangeSet)
						{
						//prevedu namerenou hodnotu na uOhmy
							case GOM_RANGE_SET_5mOHM:GOM_uOhm=(uint32_t)((double)GOM_rel_val*10000);break;
							case GOM_RANGE_SET_50mOHM:GOM_uOhm=(uint32_t)((double)GOM_rel_val*1000000);break;
							case GOM_RANGE_SET_500mOHM:GOM_uOhm=(uint32_t)((double)GOM_rel_val*1000000);break;
							case GOM_RANGE_SET_5OHM:GOM_uOhm=(uint32_t)(GOM_rel_val*1000000);break;
							case GOM_RANGE_SET_50OHM:GOM_uOhm=(uint32_t)(GOM_rel_val*10000000);break;
							case GOM_RANGE_SET_500OHM:GOM_uOhm=(uint32_t)(GOM_rel_val*100000000);break;
							default:
							break;
						}
				//if(GOM_uOhm>400000000){GOM_OverRange=TRUE;}else{GOM_OverRange=FALSE;}
			}
		break;

		default:
		break;
		}


return result;//zprava prisla v poradku
}

void USBTransfer(void){
	HAL_UART_Receive_IT(&huart2,USB_Rxbuffer,USB_RXBUFFER_DATA_SIZE);
return;
}


void USBDataResult(void){

		uint8_t size=0;
		uint16_t out=0;
		uint16_t data[2];
		bool WError=FALSE;

		uint16_t receiveCRC = USB_Rxbuffer[6];//LSB first, MSB second
		receiveCRC|=USB_Rxbuffer[7]<<8;
		countedCRC = crc16arr(USB_Rxbuffer,6);
		if(receiveCRC==countedCRC)//if CRC is correct
		{
			uint16_t request = USB_Rxbuffer[0]<<8;//LSB first, MSB second
			request|=USB_Rxbuffer[1];

			data[out]=USB_Rxbuffer[2];
			data[out++]|=USB_Rxbuffer[3]<<8;
			data[out]=USB_Rxbuffer[4];
			data[out++]|=USB_Rxbuffer[5]<<8;

			switch (request)
				{
				case USBcom_LaunchTest://prikaz
					size = USB_SendConfirmation(USB_Txbuffer,USBcomAnswer_LaunchTest,WError);
					Command.Launch=TRUE;
					break;

				case USBcom_StopTest://prikaz
					size = USB_SendConfirmation(USB_Txbuffer,USBcomAnswer_StopTest,WError);
					//hned poslu odpoved
					HAL_UART_Transmit(&huart2, USB_Txbuffer, size, 100);
		    		//generace sw resetu
					HAL_NVIC_SystemReset();
					break;

				case USBcom_config_UI:
					if(data[0]<=600){Flag.Voltage=data[0];}else{WError=TRUE;}
					if(data[1]<=240){Flag.Current=data[1];}else{WError=TRUE;}
					size = USB_SendConfirmation(USB_Txbuffer,USBcomAnswer_config_UI,WError);
					break;

				case USBcom_config_Impulse:
					if(data[0]>=1000 && data[0]<=2000){Flag.Impuls=data[0];}else{WError=TRUE;}
					Flag.ResistanceAC = (USB_Rxbuffer[4]>>0)&1;
					Flag.ResistanceBC = (USB_Rxbuffer[4]>>1)&1;
					Flag.ResistanceAB = (USB_Rxbuffer[4]>>2)&1;
					Flag.Insertion = (USB_Rxbuffer[4]>>3)&1;
					Flag.SpinMotor = (USB_Rxbuffer[4]>>4)&1;
					if((USB_Rxbuffer[5]>=GOM_RANGE_SET_5mOHM) && (USB_Rxbuffer[5]<=GOM_RANGE_SET_500OHM)){Flag.Range = USB_Rxbuffer[5];}else{WError=TRUE;}
					if(Flag.ResistanceAB || Flag.ResistanceBC || Flag.ResistanceAC){Flag.Resistance=TRUE;Flag.ChangeGOMSetting = TRUE;}else{Flag.Resistance=FALSE;}
					size = USB_SendConfirmation(USB_Txbuffer,USBcomAnswer_config_Impulse,WError);
					break;

				case USBcom_GetStatus:
					size = USB_GetStatus(USB_Txbuffer);
					break;

				case USBcom_GetResult:
					size = USB_GetResult(USB_Txbuffer);
					break;

				case USBcom_GetVersion:
					size = USB_GetVersion(USB_Txbuffer);
					break;

				case USBcom_GetCalibData:
					size = USB_GetCalibData(USB_Txbuffer);
					break;

				case USBcom_SetCalibDataRAC:
					memcpy(&CalibZero[RAC], &USB_Rxbuffer[2], sizeof(float));
					size = USB_SendConfirmation(USB_Txbuffer,USBcomAnswer_SetCalibDataRAC,WError);
					break;

				case USBcom_SetCalibDataRBC:
					memcpy(&CalibZero[RBC], &USB_Rxbuffer[2], sizeof(float));
					size = USB_SendConfirmation(USB_Txbuffer,USBcomAnswer_SetCalibDataRBC,WError);
					break;

				case USBcom_SetCalibDataRAB:
					memcpy(&CalibZero[RAB], &USB_Rxbuffer[2], sizeof(float));
					size = USB_SendConfirmation(USB_Txbuffer,USBcomAnswer_SetCalibDataRAB,WError);
					break;

				case USBcom_Insertion:
					size = USB_SendConfirmation(USB_Txbuffer,USBcomAnswer_Insertion,WError);
					//HAL_UART_Transmit(&huart2, USB_Txbuffer, size, 100);
					Command.Insertion=TRUE;
					break;

				case USBcom_Extraction:
					size = USB_SendConfirmation(USB_Txbuffer,USBcomAnswer_Extraction,WError);
					//HAL_UART_Transmit(&huart2, USB_Txbuffer, size, 100);
					Command.Extraction=TRUE;
					break;

				case USBcom_Launch_Calibration://prikaz
					size = USB_SendConfirmation(USB_Txbuffer,USBcomAnswer_Launch_Calibration,WError);
					Command.LaunchCalibration=TRUE;
					break;

				case USBcom_ConfigTester:
					size = USB_SendConfirmation(USB_Txbuffer,USBcomAnswer_ConfigTester,WError);
					Tset.Magnets = USB_Rxbuffer[2];
					break;

				case USBcom_ConfigTesterTimers:
					size = USB_SendConfirmation(USB_Txbuffer,USBcomAnswer_ConfigTesterTimers,WError);
					Tset.TimeMotorRun = data[0];
					Tset.TimeResMeasure = data[1];
					break;

				default:
					request=USBcom_Default;
					break;
				}
			if(/*request==USBcom_Insertion || request==USBcom_Extraction || */request==USBcom_Default){}
			else{HAL_UART_Transmit(&huart2, USB_Txbuffer, size, 100);}
		}
}

//*****************************************************************************************
//******************************* USB serial communication ********************************
//*****************************************************************************************

uint8_t USB_GetVersion(uint8_t * outputBuffer) {
	uint8_t addr=0;
	outputBuffer[addr++] = (uint8_t)(USBcomAnswer_GetVersion>>8); //adresa LSB, adresa 1024 pro defaultni nastaveni
	outputBuffer[addr++] = (uint8_t)USBcomAnswer_GetVersion; //adresa MSB
	outputBuffer[addr++] = 7; //delka ukladanych dat
	outputBuffer[addr++] = (uint8_t)FWVersion; //data
	outputBuffer[addr++] = (uint8_t)(FWVersion>>8); //data

	countedCRC = crc16arr(outputBuffer,addr);
	outputBuffer[addr++] = (uint8_t)(countedCRC & 0xFF);
	outputBuffer[addr++] = (uint8_t)((countedCRC >> 8) & 0xFF);
	return addr;	//vraci delku cele zpravy
}

uint8_t USB_SendConfirmation(uint8_t * outputBuffer, uint16_t request, bool error) {
	uint8_t addr=0;
	outputBuffer[addr++] = (uint8_t)(request>>8); //adresa LSB, adresa 1024 pro defaultni nastaveni
	outputBuffer[addr++] = (uint8_t)request; //adresa MSB
	outputBuffer[addr++] = 6; //delka ukladanych dat
	if(error){
	outputBuffer[addr++] = 1;}
	else{
	outputBuffer[addr++] = 0;
	}
	countedCRC = crc16arr(outputBuffer,addr);
	outputBuffer[addr++] = (uint8_t)(countedCRC & 0xFF);
	outputBuffer[addr++] = (uint8_t)((countedCRC >> 8) & 0xFF);
	return addr;	//vraci delku cele zpravy
}

uint8_t USB_GetCalibData(uint8_t * outputBuffer) {
	uint8_t addr=0;
	outputBuffer[addr++] = (uint8_t)(USBcomAnswer_GetCalibData>>8); //adresa LSB, adresa 1024 pro defaultni nastaveni
	outputBuffer[addr++] = (uint8_t)USBcomAnswer_GetCalibData; //adresa MSB
	outputBuffer[addr++] = 17; //delka ukladanych dat

	outputBuffer[addr++] = ((uint8_t *)&CalibZero[RAC])[0];
	outputBuffer[addr++] = ((uint8_t *)&CalibZero[RAC])[1];
	outputBuffer[addr++] = ((uint8_t *)&CalibZero[RAC])[2];
	outputBuffer[addr++] = ((uint8_t *)&CalibZero[RAC])[3];

	outputBuffer[addr++] = ((uint8_t *)&CalibZero[RBC])[0];
	outputBuffer[addr++] = ((uint8_t *)&CalibZero[RBC])[1];
	outputBuffer[addr++] = ((uint8_t *)&CalibZero[RBC])[2];
	outputBuffer[addr++] = ((uint8_t *)&CalibZero[RBC])[3];

	outputBuffer[addr++] = ((uint8_t *)&CalibZero[RAB])[0];
	outputBuffer[addr++] = ((uint8_t *)&CalibZero[RAB])[1];
	outputBuffer[addr++] = ((uint8_t *)&CalibZero[RAB])[2];
	outputBuffer[addr++] = ((uint8_t *)&CalibZero[RAB])[3];

	countedCRC = crc16arr(outputBuffer,addr);
	outputBuffer[addr++] = (uint8_t)(countedCRC & 0xFF);
	outputBuffer[addr++] = (uint8_t)((countedCRC >> 8) & 0xFF);
	return addr;	//vraci delku cele zpravy
}

uint8_t USB_GetStatus(uint8_t * outputBuffer) {
	uint8_t addr=0;
	outputBuffer[addr++] = (uint8_t)(USBcomAnswer_GetStatus>>8); //adresa LSB, adresa 1024 pro defaultni nastaveni
	outputBuffer[addr++] = (uint8_t)USBcomAnswer_GetStatus; //adresa MSB
	outputBuffer[addr++] = 12; //delka ukladanych dat
	outputBuffer[addr++] = MAXCALSTEP; //data
	outputBuffer[addr++] = CalStep; //data
	if(MotorOvercurrent){
		//outputBuffer[addr++] = MAIN_ERR_MOTOR_OVERCURRENT;//vyrazeni chyby overcurrent
		outputBuffer[addr++] = CalErr;
	}
	else{outputBuffer[addr++] = CalErr;}
	outputBuffer[addr++] = (uint8_t)CalErrValue;
	outputBuffer[addr++] = (uint8_t)(CalErrValue>>8); //data
	outputBuffer[addr++] = (uint8_t)(CalErrValue>>16); //data
	outputBuffer[addr++] = (uint8_t)(CalErrValue>>24); //data


	countedCRC = crc16arr(outputBuffer,addr);
	outputBuffer[addr++] = (uint8_t)(countedCRC & 0xFF);
	outputBuffer[addr++] = (uint8_t)((countedCRC >> 8) & 0xFF);
	return addr;	//vraci delku cele zpravy
}


uint8_t USB_GetResult(uint8_t * outputBuffer) {
	uint8_t addr=0;
	outputBuffer[addr++] = (uint8_t)(USBcomAnswer_GetResult>>8); //adresa LSB, adresa 1024 pro defaultni nastaveni
	outputBuffer[addr++] = (uint8_t)USBcomAnswer_GetResult; //adresa MSB
	outputBuffer[addr++] = 30; //delka ukladanych dat
	outputBuffer[addr++] = (uint8_t)MotorUo; //data
	outputBuffer[addr++] = (uint8_t)(MotorUo>>8); //data
	outputBuffer[addr++] = (uint8_t)MotorIo; //data
	outputBuffer[addr++] = (uint8_t)(MotorIo>>8); //data
	outputBuffer[addr++] = (uint8_t)MotorRpm; //data
	outputBuffer[addr++] = (uint8_t)(MotorRpm>>8); //data
	outputBuffer[addr++] = (uint8_t)(MotorRpm>>16); //data
	outputBuffer[addr++] = (uint8_t)(MotorRpm>>24); //data
	outputBuffer[addr++] = (uint8_t)MotorDirection; //data
	outputBuffer[addr++] = (uint8_t)GOM_uOhm_ABC[RAC]; //data
	outputBuffer[addr++] = (uint8_t)(GOM_uOhm_ABC[RAC]>>8); //data
	outputBuffer[addr++] = (uint8_t)(GOM_uOhm_ABC[RAC]>>16); //data
	outputBuffer[addr++] = (uint8_t)(GOM_uOhm_ABC[RAC]>>24); //data
	outputBuffer[addr++] = (uint8_t)GOM_uOhm_ABC[RBC]; //data
	outputBuffer[addr++] = (uint8_t)(GOM_uOhm_ABC[RBC]>>8); //data
	outputBuffer[addr++] = (uint8_t)(GOM_uOhm_ABC[RBC]>>16); //data
	outputBuffer[addr++] = (uint8_t)(GOM_uOhm_ABC[RBC]>>24); //data
	outputBuffer[addr++] = (uint8_t)GOM_uOhm_ABC[RAB]; //data
	outputBuffer[addr++] = (uint8_t)(GOM_uOhm_ABC[RAB]>>8); //data
	outputBuffer[addr++] = (uint8_t)(GOM_uOhm_ABC[RAB]>>16); //data
	outputBuffer[addr++] = (uint8_t)(GOM_uOhm_ABC[RAB]>>24); //data
	outputBuffer[addr++] = (uint8_t)MotorKV; //data
	outputBuffer[addr++] = (uint8_t)(MotorKV>>8); //data
	outputBuffer[addr++] = (uint8_t)MotorImax; //data
	outputBuffer[addr++] = (uint8_t)(MotorImax>>8); //data

	countedCRC = crc16arr(outputBuffer,addr);
	outputBuffer[addr++] = (uint8_t)(countedCRC & 0xFF);
	outputBuffer[addr++] = (uint8_t)((countedCRC >> 8) & 0xFF);
	return addr;	//vraci delku cele zpravy
}


