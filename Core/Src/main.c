/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "string.h"
#include "math.h"
#include "fastmath.h"
#include "MB_veribl_my.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
//********************************* МО�? ПЕРЕМЕННЫЕ *********************************
////////......................... для измерений .........................////////
volatile uint16_t adc_A[200]={0,};              //переменная для хранения данных с АЦП
volatile uint16_t adc_B[200]={0,};              //переменная для хранения данных с АЦП
volatile uint16_t adc_C[200]={0,};              //переменная для хранения данных с АЦП
uint32_t izm2period=147;						//кол-во измерений за период 50Гц
uint32_t dacOporn=90;							//опорное напряжение*2 (в у.е.) для ТТ

////////.................... МОДбус ...MB_veribl_my.h......////////

////////... МОДбус для китайского Ваттметра...MB_veribl_my.h...////////

//-------------------------- тестовая хрень --------------------------
uint8_t TESTmassiv[50]={0,};                    //вывод отладочной инфы в УАРТ
uint16_t flagTest1=0;
uint8_t flagTest2=0;
//-------------------------- прочее --------------------------
union {
  float    vtosh; //VTOSH    vtosh0.vtosh
  struct {
    uint8_t   byte1,
              byte2,
              byte3,
              byte4;
  };
} vtosh0;

union {
  float    zadan_v; //ZADAN_V    zadan_v0.zadan_v
  struct {
    uint8_t   byte1,
              byte2,
              byte3,
              byte4;
  };
} zadan_v0;

union {
  float    tekysheeU; //ZADAN_V    tekysheeNapr.tekysheeU
  struct {
    uint8_t   byte1,
              byte2,
              byte3,
              byte4;
  };
} tekysheeNapr;

union {
  float    min_v; //MIN_V    volt0.min_v
  struct {
    uint8_t   byte1,
              byte2,
              byte3,
              byte4;
  };
} volt0;

union {
  float    max_v; //MAX_V    volt1.max_v
  struct {
    uint8_t   byte1,
              byte2,
              byte3,
              byte4;
  };
} volt1;

uint16_t timUzapis=0;
uint16_t provSvyazCount=0,
		 provSvyazCount2=0;
float	zadanMin=10.0,
		provSvyaz=0.0,
		zadan_vZash=0.0,
		zadan_v001p=0.0,
		zadan_v001m=0.0,
		zadan_v005p=0.0,
		zadan_v005m=0.0;
_Bool errGPM=0;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

unsigned int Crc16Table[256] = {
/*
  Name  : CRC-16
  Poly  : 0x8005    x^16 + x^15 + x^2 + 1
  Init  : 0xFFFF
  Revert: true
  XorOut: 0x0000
  Check : 0x4B37 ("123456789")
  MaxLen: 4095 байт (32767 бит) - обнаружение одинарных, двойных, тройных и всех нечетных ошибок
*/
    0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241,
    0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440,
    0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40,
    0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841,
    0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40,
    0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41,
    0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641,
    0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040,
    0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240,
    0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441,
    0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41,
    0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840,
    0x2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41,
    0xEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1, 0xEC81, 0x2C40,
    0xE401, 0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640,
    0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041,
    0xA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240,
    0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441,
    0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41,
    0xAA01, 0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840,
    0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41,
    0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40,
    0xB401, 0x74C0, 0x7580, 0xB541, 0x7700, 0xB7C1, 0xB681, 0x7640,
    0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0, 0x7080, 0xB041,
    0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241,
    0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440,
    0x9C01, 0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40,
    0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841,
    0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40,
    0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41,
    0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641,
    0x8201, 0x42C0, 0x4380, 0x8341, 0x4100, 0x81C1, 0x8081, 0x4040};

unsigned int Crc16(unsigned char *pcBlock, unsigned int len){
    unsigned int crc = 0xFFFF;
    while (len--) crc = (crc >> 8) ^ Crc16Table[(crc & 0xFF) ^ *pcBlock++];
    return crc;
    }

void IDmyMB (void){
	mb_data[MY_ID]=0;

	mb_data[MY_ID]=(~( 0xff00 |
//									7							6						5							4
						((GPIOB->IDR&(1<<8))>>1)|((GPIOB->IDR&(1<<9))>>3)|((GPIOC->IDR&(1<<13))>>8)|((GPIOC->IDR&(1<<0))<<4)|
//									3							2						1							0
						((GPIOC->IDR&(1<<1))<<2)|((GPIOC->IDR&(1<<2))<<0)|((GPIOC->IDR&(1<<3))>>2)|((GPIOA->IDR&(1<<0))<<0) ) );
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//................. Таймерs
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){

	if(htim->Instance==TIM1){		// каждые 0,075мс
		MB_counT35++;  //до ~23 (1,75)
		MB3_counT35++;  //до ~23 (1,75)
		MB3_count_Tx++; //счётчик периода запроса
		provSvyazCount2++;
	} //end каждые 0,075мс

	if(htim->Instance==TIM3){		// каждые 10 мс
		timUzapis++;					// время измерения напряжения
		//HAL_GPIO_TogglePin(GPIOB, OutRezerv1_Pin);	// мигание кнопок М и Б
		//if (flagTest1++>=2000) {flagTest1=0;}		//времянка
		//////////////////  СиГНАЛиЗАТОР  /////////////////////////////////////
		#define SIGNAL1 14
		#define SIGNAL2 100
		if (signal_0) {			//сигнализировать "Отмена" или "Отключить"
			if      ( (out_Sign_count1>0) && (out_Sign_count1<=SIGNAL1) ) {GPIOB->ODR|=(1<<4);}		// л1
			else if ( (out_Sign_count1>SIGNAL1) && (out_Sign_count1<=SIGNAL2) ) {GPIOB->ODR&=~(1<<4);}	// л0
			else if ( (out_Sign_count1>SIGNAL2) && (out_Sign_count1<=(SIGNAL1+SIGNAL2)) ) {GPIOB->ODR|=(1<<4);}	// л1
			else if (out_Sign_count1>(SIGNAL1+SIGNAL2)) {GPIOB->ODR&=~(1<<4); signal_0=0;}				// л0
			out_Sign_count1++;
		}
		//else {out_Sign_count1=0;}
		else if (signal_1) {			//сигнализировать "Принять" или "Включить"
			if      ( (out_Sign_count1>0) && (out_Sign_count1<=SIGNAL1) ) {GPIOB->ODR|=(1<<4);}		// л1
			else if ( (out_Sign_count1>SIGNAL1) && (out_Sign_count1<=SIGNAL2) ) {GPIOB->ODR&=~(1<<4);}	// л0
			else if ( (out_Sign_count1>SIGNAL2) && (out_Sign_count1<=(2*SIGNAL2)) ) {GPIOB->ODR|=(1<<4);}	// л1
			else if (out_Sign_count1>(2*SIGNAL2)) {GPIOB->ODR&=~(1<<4); signal_1=0;}				// л0
			out_Sign_count1++;
		}
		else {out_Sign_count1=0;}

		//////////////////  ФиЛЬТР  длительность ~5 мкс /////////////////////////////////////
		//GPIOB->ODR |=(1<<5);
		if ( (!(GPIOA->IDR & (1<<15)))^(mb_data[IN_IST] & (1<<IN_AVTO2)) )	//Режим Авто-2
			{In_Auto1_count++;}
		else {In_Auto1_count=0;}
		if ( (!(GPIOA->IDR & (1<<8)))^(mb_data[IN_IST] & (1<<IN_AVTO1)) )	//Режим Авто-1
			{In_Auto2_count++;}
		else {In_Auto2_count=0;}
		if ( (!(GPIOC->IDR & (1<<9)))^(mb_data[IN_IST] & (1<<IN_AUTO_NOM)) )
			{In_Rezerv_count++;}
		else {In_Rezerv_count=0;}
		if ( (GPIOC->IDR & (1<<8)) ^ (mb_data[IN_IST] & (1<<IN_ATR_MIN)) )	//Конечник Мин
			{In_ATRmin_count++;}
		else {In_ATRmin_count=0;}
		if ( (GPIOC->IDR & (1<<7)) ^ (mb_data[IN_IST] & (1<<IN_ATR_MAX)) )	//Конечник Макс
			{In_ATRmax_count++;}
		else {In_ATRmax_count=0;}
		if ( (!(GPIOC->IDR & (1<<6)))^(mb_data[IN_IST] & (1<<IN_KN_MEN)) )	//Кн Меньше
			{In_KnMen_count++;}
		else {In_KnMen_count=0;}
		if ( (!(GPIOB->IDR & (1<<15)))^(mb_data[IN_IST] & (1<<IN_KN_BOL)) )	//Кн Больше
			{In_KnBol_count++;}
		else {In_KnBol_count=0;}

		////////////////// Прошедшие ФиЛЬТР /////////////////////////////////////
		if (In_Auto1_count>=In_count){   In_Auto1_count=0;
			if (GPIOA->IDR & (1<<15))	{mb_data[IN_IST] &=~ (1<<IN_AVTO2);}
			else						{mb_data[IN_IST] |= (1<<IN_AVTO2);}
		}
		if (In_Auto2_count>=In_count){   In_Auto2_count=0;
			if (GPIOA->IDR & (1<<8))	{mb_data[IN_IST] &=~ (1<<IN_AVTO1);}
			else						{mb_data[IN_IST] |= (1<<IN_AVTO1);}
		}
		if (In_Rezerv_count>=In_count){   In_Rezerv_count=0;
			if (GPIOC->IDR & (1<<9))	{mb_data[IN_IST] &=~ (1<<IN_AUTO_NOM);}
			else						{mb_data[IN_IST] |= (1<<IN_AUTO_NOM);}
		}
		if (In_ATRmin_count>=In_count){   In_ATRmin_count=0;
			if (GPIOC->IDR & (1<<8))	{mb_data[IN_IST] |= (1<<IN_ATR_MIN);}
			else						{mb_data[IN_IST] &=~ (1<<IN_ATR_MIN);}
		}
		if (In_ATRmax_count>=In_count){   In_ATRmax_count=0;
			if (GPIOC->IDR & (1<<7))	{mb_data[IN_IST] |= (1<<IN_ATR_MAX);}
			else						{mb_data[IN_IST] &=~ (1<<IN_ATR_MAX);}
		}
		if (In_KnMen_count>=In_count){   In_KnMen_count=0;
			if (GPIOC->IDR & (1<<6))	{mb_data[IN_IST] &=~ (1<<IN_KN_MEN);}
			else						{mb_data[IN_IST] |= (1<<IN_KN_MEN);}
		}
		if (In_KnBol_count>=In_count){   In_KnBol_count=0;
			if (GPIOB->IDR & (1<<15))	{mb_data[IN_IST] &=~ (1<<IN_KN_BOL);}
			else						{mb_data[IN_IST] |= (1<<IN_KN_BOL);}
		}

		//////////////////////////////////// СПЕЦ-НАЖАТиЯ /////////////////////////////////////
		if ( (mb_data[IN_IST] & (1<<IN_KN_MEN)) && (mb_data[IN_IST] & (1<<IN_KN_BOL)) )	// М+Б
			{In_KnMpB_count++;}
		else {In_KnMpB_count=0;}
		if (In_KnMpB_count>=In_count2) {mb_data[IN_IST] |= (1<<IN_KN_MPB);
			In_KnMpB_count=0;
			signal_1=1;
		}

		if ( (mb_data[IN_IST] & (1<<IN_KN_MEN)) && !(mb_data[IN_IST] & (1<<IN_KN_BOL)) ) {In_KnMMMen_count++;}	// М (долгое)
		else {In_KnMMMen_count=0;}
		if (In_KnMMMen_count>=In_count2) {mb_data[IN_IST] |= (1<<IN_KN_MMMEN);In_KnMMMen_count=0;}

		if ( !(mb_data[IN_IST] & (1<<IN_KN_MEN)) && (mb_data[IN_IST] & (1<<IN_KN_BOL)) ) {In_KnBBBol_count++;}	// Б (долгое)
		else {In_KnBBBol_count=0;}
		if (In_KnBBBol_count>=In_count2) {mb_data[IN_IST] |= (1<<IN_KN_BBBOL);In_KnBBBol_count=0;}

		//GPIOB->ODR &=~ (1<<5); GPIOB->ODR |= (1<<5);
	}//end каждые 10 мс
}

//................. Таймер на ШиМ
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim){
	if(htim->Instance==TIM2){
		if (mb_data[OUT_IST]&(1<<OUT_ED_NAPR))	{mb_data[ATR_POL_OTN]++; atr_pol_abs++;}
		else 									{mb_data[ATR_POL_OTN]--; atr_pol_abs--;}
		mb_data[ATR_POL_ABS] = atr_pol_abs;
		//if (mb_data[IN_IST]&(1<<IN_ATR_MIN))		{mb_data[ATR_POL_ABS]=0;}
		//Delitel_count++;
		//HAL_GPIO_TogglePin(GPIOB, PinLEDdebug_Pin);
	}
}

/*
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc1){
	//HAL_GPIO_TogglePin(GPIOB, PinLEDdebug_Pin);
	if(hadc1->Instance == ADC1) {;
	 }
}
*/

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
	if(huart==&huart1){;}
	if(huart==&huart3){;}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if(huart==&huart1){
		HAL_UART_Receive_IT(&huart1, (uint8_t*)MB_preBUFF, rxPac); //запуск следующего приёма
		MB_BUFF[MB_buffCount++]=MB_preBUFF[0];   //сохраняем полученый байт в буфер
		MB_counT35=0;  //до ~23..24 (1,75)
		MB_MyPac_OK=0;
	}
	if(huart==&huart3){
		//HAL_UART_Receive_IT(&huart3, (uint8_t*)MB3_preBUFF, 25); //запуск следующего приёма
		//MB3_BUFF[MB3_buffCount++]=MB3_preBUFF[0];   //сохраняем полученый байт в буфер
		//mb_data[21]=( (MB3_preBUFF[3]<<8)|(MB3_preBUFF[4]) );
		//MB3_counT35=0;  //до ~23..24 (1,75)
		MB3_MyPac_OK=0;
	}
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  MX_USART3_UART_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
//***************************************************** Мой СЕТУП! СТАРТ *******
  //HAL_OPAMP_SelfCalibrate(&hopamp1); HAL_Delay(30);
  //HAL_OPAMP_SelfCalibrate(&hopamp2); HAL_Delay(30);
  //HAL_OPAMP_SelfCalibrate(&hopamp3); HAL_Delay(30);
  //HAL_OPAMP_SelfCalibrate(&hopamp4); HAL_Delay(30);
  //HAL_ADCEx_Calibration_Start(&hadc1,ADC_SINGLE_ENDED); HAL_Delay(30);
  //HAL_ADCEx_Calibration_Start(&hadc2,ADC_SINGLE_ENDED); HAL_Delay(30);
  //HAL_ADCEx_Calibration_Start(&hadc3,ADC_SINGLE_ENDED); HAL_Delay(30);
//............ Запуск ЦАП (на схеме стоит делитель 1/2 !)
  //HAL_DAC_Start(&hdac1, DAC_CHANNEL_1); HAL_Delay(100);
  //HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, dacOporn); HAL_Delay(100);

  //HAL_OPAMP_Start(&hopamp1);
  //HAL_OPAMP_Start(&hopamp2);
  //HAL_OPAMP_Start(&hopamp3);
  //HAL_OPAMP_Start(&hopamp4); HAL_Delay(100);
//............ Запуск преобразования с передачей по DMA
  //HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&adc_A, izm2period);
  //HAL_ADC_Start_DMA(&hadc2, (uint32_t*)&adc_B, izm2period);
  //HAL_ADC_Start_DMA(&hadc3, (uint32_t*)&adc_C, izm2period);

  //............ Запуск таймера
  HAL_TIM_PWM_Start_IT(&htim2, TIM_CHANNEL_2);
  HAL_TIM_Base_Start_IT(&htim1);
  HAL_TIM_Base_Start_IT(&htim3);
  //HAL_TIM_Base_Start_IT(&htim2);

  //HAL_TIM_OC_Start_IT(&htim17, TIM_CHANNEL_1);
  //HAL_TIM_Base_Start_IT(&htim17);

//***************************************************** Мой СЕТУП! СТОП *******
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
/////////////////////// Запускаем ожидание приёма по УАРТ
  HAL_UART_Receive_IT(&huart1, (uint8_t*)MB_preBUFF, rxPac);
  HAL_UART_Receive_IT(&huart3, (uint8_t*)MB3_preBUFF, rx3Pac);

//............ Уставки по умолчанию ...MB_veribl_my.h...
  mb_data[0]=0x11;  //времяночка
  mb_data[21]=21; mb_data[22]=22; mb_data[23]=23;
  rxPac=1;
  mb_data[55]=719;
  count_stepHome=0;
  mb_data[ATR_FSTP_HD]=8150; //шагов от 0 до 450 В
HAL_Delay(1000);
IDmyMB ();  //считывание ID для MB со свичей
/*************************************************************************/
/***************************** П О Г Н А Л и *****************************/
/*************************************************************************/
  while (1){


//********************** Применение ПАКЕТА **********************//
	//if (mb_data[20]&(1<<0)) {HAL_GPIO_WritePin(GPIOB, PinLEDdebug_Pin, GPIO_PIN_SET);}
	//                   else {HAL_GPIO_WritePin(GPIOB, PinLEDdebug_Pin, GPIO_PIN_RESET);}
	//htim2.Instance->PSC = mb_data[55]; //
	//if (flagTest1+169<=2000)
		//htim2.Instance->ARR = (169+flagTest1);
		//htim2.Instance->ARR

//********************** Мигание Б и М **********************//
	//if ( (Delitel_count>=10)&(GPIOB->ODR&(1<<3)) )      {GPIOB->ODR|=(1<<4);}
	//else if( (Delitel_count>=10)&!(GPIOB->ODR&(1<<3)) ) {Delitel_count=0; GPIOB->ODR&=~(1<<4);}


//********************** АВТО-1 и АВТО-2 **********************//
//*************************************************************//
uint16_t viderjka1=300;
	  // Авто-1. Работа АТР по уставкам от ПО с отслеживанием просадки U
	if ( (mb_data[IN_IST]&(1<<IN_AVTO1)) && !(mb_data[IN_IST]&(1<<IN_AVTO2)) ){
		//GPIOB->ODR|=(1<<5);
		//********************** Функции долгих нажатий (долгое М или Б) **********************//
		if (mb_data[IN_IST] & (1<<IN_KN_MMMEN)) {mb_data[IN_IST]&=~(1<<IN_KN_MMMEN);}	//не назначено
		if (mb_data[IN_IST] & (1<<IN_KN_BBBOL)) {mb_data[IN_IST]&=~(1<<IN_KN_BBBOL);}	//не назначено
		//********************** Нахождение МиН и МАКС (долгое М+Б) **********************//
		if ((mb_data[IN_IST]&(1<<IN_AUTO_HOM)) || (mb_data[IN_IST]&(1<<IN_KN_MPB))) {
			if (count_stepHome==0) {
				if (!(mb_data[IN_IST]&(1<<IN_ATR_MIN)))	{mb_data[IN_IST]|=(1<<IN_AUTO_MEN);
														 mb_data[IN_IST]&=~(1<<IN_AUTO_BOL);}
				else									{mb_data[IN_IST]&=~(1<<IN_AUTO_MEN);
														 mb_data[IN_IST]&=~(1<<IN_AUTO_BOL);
														 mb_data[ATR_POL_OTN]=0; timUzapis=0;
														 count_stepHome=1;} //Положение относительно (0В)
			}
			if ( (count_stepHome==1)&&(timUzapis<=viderjka1) ) { //сохранение Umin viderjka1-3секунды для устаканивания значниея U
				mb_data[ATR_MIN_V_H]=mb_data[GPM_V_H]; mb_data[ATR_MIN_V_L]=mb_data[GPM_V_L];}
			if ( (count_stepHome==1)&&(timUzapis>viderjka1) ) {
				count_stepHome=2; timUzapis=0;}
			if (count_stepHome==2) {
				if (!(mb_data[IN_IST]&(1<<IN_ATR_MAX)))	{mb_data[IN_IST]&=~(1<<IN_AUTO_MEN);
														 mb_data[IN_IST]|=(1<<IN_AUTO_BOL);}
				else 									{mb_data[IN_IST]&=~(1<<IN_AUTO_MEN);
														 mb_data[IN_IST]&=~(1<<IN_AUTO_BOL);
														 mb_data[ATR_FSTP]=mb_data[ATR_POL_OTN]; //Полное кол-во шагов (при 450В)
														 timUzapis=0; count_stepHome=3;
				}
			}
			if ( (count_stepHome==3)&&(timUzapis<=viderjka1) ) { //сохранение Umin
				mb_data[ATR_MAX_V_H]=mb_data[GPM_V_H]; mb_data[ATR_MAX_V_L]=mb_data[GPM_V_L];}
			if ( (count_stepHome==3)&&(timUzapis>viderjka1) ) {
				count_stepHome=4; timUzapis=0;}
			if (count_stepHome==4) {mb_data[IN_IST]&=~(1<<IN_AUTO_HOM); mb_data[IN_IST]&=~(1<<IN_KN_MPB);}
		}//end Нахождение МиН и МАКС

		//********************** ДВиЖЕНиЯ по уставкам ШАГАМ **********************//
		/*if ( (mb_data[ATR_FSTP]) && (mb_data[ATR_ZADAN_SH]) &&						//если определены МиН/МАКС и есть уставка
				((mb_data[IN_IST]&(1<<IN_AUTO_HOM))||(mb_data[OUT_IST]&(1<<OUT_SLEJEN))) )	//если ищем Дом или Следим за U
		{
			if (mb_data[ATR_ZADAN_SH] > (mb_data[ATR_POL_OTN])){		//если задание БОЛЬШЕ текущго состояния
				mb_data[IN_IST] |= (1<<IN_AUTO_BOL);
				mb_data[IN_IST] &=~ (1<<IN_AUTO_MEN);
			}
			else if (mb_data[ATR_ZADAN_SH] < (mb_data[ATR_POL_OTN])) {	//если задание МЕНЬШЕ текущго состояния
				mb_data[IN_IST] &=~ (1<<IN_AUTO_BOL);
				mb_data[IN_IST] |= (1<<IN_AUTO_MEN);
			}
			else {
				mb_data[IN_IST] &=~ (1<<IN_AUTO_BOL);					//иначе ОСТАНОВ
				mb_data[IN_IST] &=~ (1<<IN_AUTO_MEN);
			}
		}//end ДВиЖЕНиЯ по уставкам
		*/
		//********************** ДВиЖЕНиЯ по уставкам НАПРЯЖЕНиЮ  **********************//
		//if ( tekysheeNapr.tekysheeU && zadan_v0.zadan_v &&						//если определены МиН/МАКС и есть уставка
		if ( !errGPM &&  //(mb_data[IN_IST]&(1<<IN_AUTO_NOM))						//если ищем Дом
				( (mb_data[OUT_IST]&(1<<OUT_SLEJEN)) || (mb_data[KOM_IST]&(1<<KOM_SLEJEN)) ) && //или Следим за U
				( (tekysheeNapr.tekysheeU > zadanMin) && (zadan_v0.zadan_v > zadanMin) ) )	// и если Уставка не ниже минимальной!
		{
			if (tekysheeNapr.tekysheeU < zadan_v005m) {		//если текущее U МЕНЬШЕ заданного
				mb_data[IN_IST] |= (1<<IN_AUTO_BOL);			//zadan_v001p
				mb_data[IN_IST] &=~ (1<<IN_AUTO_MEN);
				htim2.Instance->ARR = 169;//настройки скорости
			}
			else if ( (tekysheeNapr.tekysheeU >= zadan_v005m) &&
					  (tekysheeNapr.tekysheeU <= zadan_v001m) ) {		//если текущее U МЕНЬШЕ заданного
				mb_data[IN_IST] |= (1<<IN_AUTO_BOL);			//zadan_v001p
				mb_data[IN_IST] &=~ (1<<IN_AUTO_MEN);
				htim2.Instance->ARR = 1699;//настройки скорости
			}
			else if (tekysheeNapr.tekysheeU > zadan_v005p) {	//если текущее U больше заданного
				mb_data[IN_IST] &=~ (1<<IN_AUTO_BOL);
				mb_data[IN_IST] |= (1<<IN_AUTO_MEN);
				htim2.Instance->ARR = 169;//настройки скорости
			}
			else if ( (tekysheeNapr.tekysheeU <= zadan_v005p) &&
					  (tekysheeNapr.tekysheeU >= zadan_v001p) ) {	//если текущее U больше заданного
				mb_data[IN_IST] &=~ (1<<IN_AUTO_BOL);
				mb_data[IN_IST] |= (1<<IN_AUTO_MEN);
				htim2.Instance->ARR = 1699;//настройки скорости
			}
			else {
				mb_data[IN_IST] &=~ (1<<IN_AUTO_BOL);					//иначе ОСТАНОВ
				mb_data[IN_IST] &=~ (1<<IN_AUTO_MEN);
			}
		}//end ДВиЖЕНиЯ по уставкам НАПРЯЖЕНиЮ
		else {
			mb_data[IN_IST] &=~ (1<<IN_AUTO_BOL);					//иначе ОСТАНОВ
			mb_data[IN_IST] &=~ (1<<IN_AUTO_MEN);
		}


		//********************** само ДВиЖЕНиЕ **********************//
		//движение против частовой в МиН
		if ( !(mb_data[IN_IST]&(1<<IN_ATR_MIN)) &&						//если АТР не на МиН
				//((mb_data[ATR_FSTP]==0) ||										//едем если не знаем общее количество шагов или
				//((mb_data[ATR_FSTP])&&(mb_data[ATR_POL_OTN]>0))) &&				//если знаем, но не доехали до 0
				( ((mb_data[IN_IST]&(1<<IN_STAT_START))&&!(mb_data[IN_IST]&(1<<IN_STAT_NAPR))) ||//едем по командам или
				(mb_data[IN_IST]&(1<<IN_AUTO_MEN)) ) )											//едем по Авто Меньше
			{
			mb_data[OUT_IST]&=~(1<<OUT_ED_NAPR);												//против Часовой
			mb_data[OUT_IST]&=~(1<<OUT_ED_ENAB);												//держим ЭД
			bitEdemNet=1;																		//старт ШиМ
			}
		//движение по частовой в МАКС
		else if ( !(mb_data[IN_IST]&(1<<IN_ATR_MAX)) &&							//если АТР не на МАКС
				//((mb_data[ATR_FSTP]==0) || 										//едем если ещё не сохранили МАКС или
				//(mb_data[ATR_POL_OTN]<=mb_data[ATR_FSTP]) ) &&					//не доехали до него
				( ((mb_data[IN_IST]&(1<<IN_STAT_START))&&(mb_data[IN_IST]&(1<<IN_STAT_NAPR))) ||//едем по командам или
				(mb_data[IN_IST]&(1<<IN_AUTO_BOL)) ) )											//едем по по Авто Больше
			{
			mb_data[OUT_IST]|=(1<<OUT_ED_NAPR);													//по Часовой
			mb_data[OUT_IST]&=~(1<<OUT_ED_ENAB);												//держим ЭД
			bitEdemNet=1;																		//старт Ш�?М
			}
		else {bitEdemNet=0;}
	}//end Авто-1

	// Авто-2. Работа от кнопок с отслеживанием просадки U
	// долгое М+Б=Х ; долгое М=Х ; долгое Б=Х
	// можно сохранять текущую уставку через 5 секунд, после уставки кнопками
	else if ( !(mb_data[IN_IST]&(1<<IN_AVTO1)) && (mb_data[IN_IST]&(1<<IN_AVTO2)) ){
		//********************** Функции долгих нажатий (долгое М или Б) **********************//
		if (mb_data[IN_IST] & (1<<IN_KN_MMMEN)) {mb_data[IN_IST]&=~(1<<IN_KN_MMMEN);}	//не назначено
		if (mb_data[IN_IST] & (1<<IN_KN_BBBOL)) {mb_data[IN_IST]&=~(1<<IN_KN_BBBOL);}	//не назначено
		mb_data[IN_IST]&=~(1<<IN_KN_MPB);

		//********************** движения ЭД с кнопок **********************//
		//движение против частовой в МиН
		if ( !(mb_data[IN_IST]&(1<<IN_ATR_MIN))&&								//если АТР не на МиН
				((mb_data[ATR_FSTP]==0) ||										//едем если не знаем общее количество шагов или
				((mb_data[ATR_FSTP])&&(mb_data[ATR_POL_OTN]>0))) &&				//    если знаем, но не доехали до 0
				(mb_data[IN_IST]&(1<<IN_KN_MEN)) )								//едем по Кн М
			{
			mb_data[OUT_IST]&=~(1<<OUT_ED_NAPR);								//против Часовой
			mb_data[OUT_IST]&=~(1<<OUT_ED_ENAB);								//держим ЭД
			bitEdemNet=1;														//старт ШиМ
			}
		//движение по частовой в МАКС
		else if ( !(mb_data[IN_IST]&(1<<IN_ATR_MAX)) &&							//если АТР не на МАКС
				((mb_data[ATR_FSTP]==0) ||										//едем если ещё не сохранили МАКС или
				(mb_data[ATR_POL_OTN]<=mb_data[ATR_FSTP]) ) &&					//     не доехали до него
				(mb_data[IN_IST]&(1<<IN_KN_BOL)) )								//едем по Кн Б
			{
			mb_data[OUT_IST]|=(1<<OUT_ED_NAPR);									//по Часовой
			mb_data[OUT_IST]&=~(1<<OUT_ED_ENAB);								//держим ЭД
			bitEdemNet=1;														//старт ШиМ
			}
		else {bitEdemNet=0;}
	} // end Авто-2

	// Ручное управление. Отпускание ЭД! и вращение АТР за ручку
	// долгое М+Б=>0< ; долгое М=>220В< ; долгое Б=ВКЛ/ВЫКЛ слежения за напряжением
	else if ( !(mb_data[IN_IST]&(1<<IN_AVTO1)) && !(mb_data[IN_IST]&(1<<IN_AVTO2)) ){
		bitEdemNet=0;
		mb_data[OUT_IST] |= (1<<OUT_ED_ENAB);
		mb_data[IN_IST]&=~(1<<IN_AUTO_MEN);
		mb_data[IN_IST]&=~(1<<IN_AUTO_BOL);
		//********************** Функции долгих нажатий (долгое М или Б) **********************//
		if (mb_data[IN_IST]&(1<<IN_KN_MMMEN)) {		//Руками установили на >220В< и расчитали все другие переменные
			//mb_data[ATR_FSTP]=mb_data[ATR_FSTP_HD]; // так НЕНАДО!!!!!
			//mb_data[ATR_POL_OTN]=(22*mb_data[ATR_FSTP])/45;
			//atr_ZADAN_V=220.0;
			//mb_data[ATR_ZADAN_SH]=mb_data[ATR_POL_OTN];
			mb_data[IN_IST]&=~(1<<IN_KN_MMMEN);
			signal_1=1;
		}
		if (mb_data[IN_IST]&(1<<IN_KN_BBBOL)) {		//ВКЛ/ОТКЛ слежения за U и отработка
			mb_data[OUT_IST]^=(1<<OUT_SLEJEN);
			if (mb_data[OUT_IST]&(1<<OUT_SLEJEN)) {signal_1=1;} else {signal_0=1;}
			mb_data[IN_IST]&=~(1<<IN_KN_BBBOL);
		}
		if (mb_data[IN_IST]&(1<<IN_KN_MPB)) {	//М+Б в ручном режиме = обнуление уставок и М�?Н МАКС
			mb_data[ATR_FSTP]=0;				//МАКС=0
			mb_data[ATR_POL_OTN]=0;				//относительное положе=0
			mb_data[ATR_ZADAN_SH]=0;			//уставка по шагам=0
			count_stepHome=0;					//сброс функции "Домой"
			mb_data[IN_IST]&=~(1<<IN_KN_MPB);	//сброс М+Б
		}
	}
	// Аварийная комбинация (залип переключатель) НО! это комбинация может возникнуть при быстром переключении!
	else {GPIOC->ODR |= (1<<12); bitEdemNet=0;} //отпустить ЭД, отключить Ш�?М.


//обнуление текущей позиции по IN_ATR_MIN и запись максимума по IN_ATR_MAX
		//if (mb_data[IN_IST]&(1<<IN_ATR_MIN)) {mb_data[ATR_POL_OTN]=0;}
		//if (mb_data[IN_IST]&(1<<IN_ATR_MAX)) {mb_data[ATR_FSTP]=mb_data[ATR_POL_OTN];}

////////////////////////////////////////////////////////////////////
//********************** Применение выходов **********************//
////////////////////////////////////////////////////////////////////
	if (mb_data[OUT_IST]&(1<<OUT_ED_NAPR))	{GPIOD->ODR |= (1<<2);}		// ЭД по частовой
	else									{GPIOD->ODR &=~ (1<<2);}	// ЭД против частовой
	if (mb_data[OUT_IST]&(1<<OUT_ED_ENAB))	{GPIOC->ODR |= (1<<12);}	// ЭД отпущен
	else									{GPIOC->ODR &=~ (1<<12);}	// ЭД держим
	if (bitEdemNet) {HAL_TIM_PWM_Start_IT(&htim2, TIM_CHANNEL_2);}		// ЭД едет
	else {HAL_TIM_PWM_Stop_IT(&htim2, TIM_CHANNEL_2);}					// ЭД стоит

//********************** Проверка связи **********************//
	//if (provSvyazCount>=5){
	//	if (tekysheeNapr.tekysheeU==provSvyaz) {errGPM=1;}//provSvyazCount
	//	else {errGPM=0;}
	//	provSvyaz = tekysheeNapr.tekysheeU;
	//	provSvyazCount=0;
	//errGPM=0;
	//}


	//********************** >Т35 **********************//
	if ( !MB_MyPac_OK && (MB_counT35>=MB_dT35) ) {
		// СЮДА ПРОПИСАЛЬ ЗАЩИТУ ОТ ОБРЫВА СВЯЗИ !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
		MB_counT35=0;
		MB_buffCount=0;
		if (provSvyazCount2>=1000) {errGPM=1;}
		//else {errGPM=0;}
		MB_BUFF[0]=0xff; MB_BUFF[1]=0xff;   //это стираем ID и COM навсякий, но это ОПАСНО!
	}

//********************** (Tx) ОТВЕЧАЕМ КОГДА ПАКЕТ МОЙ и Т35 **********************//
	if ( MB_MyPac_OK && (MB_counT35>=MB_dT35) ) {
		MB_counT35=0;
		MB_buffCount=0;
		MB_MyPac_OK=0;
		HAL_UART_Transmit_IT(&huart1,(uint8_t*)MB_BUFF, txPac);
		MB_BUFF[0]=0xff;
	}

//************** Подготовка данных пакета отправки китайскому ваттемтру **********//
	MB3_BUFF[0]=248; //id китайского ваттметра
	MB3_BUFF[1]=4;   //0x04 код функции чтения регистров
	MB3_BUFF[2]=0; MB3_BUFF[3]=0; //адрес 1 регистра
	MB3_BUFF[4]=0; MB3_BUFF[5]=10; //количество регистров
	MB3_BUFF[7]=0x64; MB3_BUFF[6]=0x64; //crc-16
//********************* Опрашиваем китайский ваттметр ********************//
	if (MB3_count_Tx>=MB3_Tim_Tx){
		MB3_count_Tx=0;
		HAL_UART_Receive_IT(&huart3, (uint8_t*)MB3_preBUFF, rx3Pac); //запуск следующего приёма
		HAL_UART_Transmit_IT(&huart3,(uint8_t*)MB3_BUFF, tx3Pac);
	}
//********************* Парсим что пришло от кит.ват. *********************//
	if ( (!MB3_MyPac_OK)&&(!Crc16(MB3_preBUFF, 25)) ){
		MB3_Prot_adr1reg = KITW_YSTW;	//адресс первого регистра с которого начнём запись в stm
		MB3_Prot_kolreg = (MB3_BUFF[4]<<8)|MB3_BUFF[5];	//кол регистров
		MB3_Prot_kolbyt = MB3_preBUFF[2];				//кол байт далее
		while(MB3_Prot_kolreg)   //сохраняем того что пришло в регистры stm
			{
			mb_data[--MB3_Prot_kolreg + MB3_Prot_adr1reg] = (MB3_preBUFF[2 + MB3_Prot_kolbyt--]) |
				(MB3_preBUFF[2 + MB3_Prot_kolbyt--]<<8);
			}
		// Переворачивание H<->L регистров... китайцы! хуле!
		MB3_Prot_adr1reg=mb_data[81];mb_data[81]=mb_data[82];mb_data[82]=MB3_Prot_adr1reg;
		MB3_Prot_adr1reg=mb_data[83];mb_data[83]=mb_data[84];mb_data[84]=MB3_Prot_adr1reg;
		MB3_Prot_adr1reg=mb_data[85];mb_data[85]=mb_data[86];mb_data[86]=MB3_Prot_adr1reg;
		MB3_MyPac_OK=1;
	}

	zadan_v0.byte1 = mb_data[ATR_ZADAN_V_H];
	zadan_v0.byte2 = mb_data[ATR_ZADAN_V_H]>>8;
	zadan_v0.byte3 = mb_data[ATR_ZADAN_V_L];
	zadan_v0.byte4 = mb_data[ATR_ZADAN_V_L]>>8;
	if (zadan_vZash != zadan_v0.zadan_v){
		zadan_vZash = zadan_v0.zadan_v;
		zadan_v001p = zadan_v0.zadan_v + (zadan_v0.zadan_v * 0.005);
		zadan_v001m = zadan_v0.zadan_v - (zadan_v0.zadan_v * 0.005);
		zadan_v005p = zadan_v0.zadan_v + (zadan_v0.zadan_v * 0.05);
		zadan_v005m = zadan_v0.zadan_v - (zadan_v0.zadan_v * 0.05);
	}


//***********************************************************************************
//************************ ОБРАБОТКА и СОХРАНЕНиЕ ДАННЫХ ПАКЕТА *********************
//***********************************************************************************
	if (!MB_MyPac_OK) {
		if ( (MB_BUFF[0]==0) || (MB_BUFF[0]==mb_data[0]) ) {

			provSvyazCount2=0;

//********************** Команда 16(0x10) ЗАП�?СЬ нескольких регистров ******************
			if ( (MB_BUFF[1]==0x10) && (MB_buffCount==7) ){   //проверка команды и вычисление длины пакета
			   MB_Prot_adr1reg = (MB_BUFF[2]<<8)|MB_BUFF[3]; //адресс первого регистра
			   MB_Prot_kolreg =  (MB_BUFF[4]<<8)|MB_BUFF[5]; //кол регистров
			   MB_Prot_kolbyt = MB_BUFF[6];				     //кол байт далее
			}
			if ( (MB_buffCount==MB_Prot_kolbyt+7+2) && (!Crc16(MB_BUFF, MB_Prot_kolbyt+7+2)) ) {
			   while(MB_Prot_kolreg)   //сохраняем положение регистров (реле)
			      {
			       mb_data[--MB_Prot_kolreg + MB_Prot_adr1reg] = (MB_BUFF[6 + MB_Prot_kolbyt--]) |
			    		   (MB_BUFF[6 + MB_Prot_kolbyt--]<<8);
			      }
			   crc16rez = Crc16(MB_BUFF, 6);                  //расчёт crc16 для ответа
			   MB_BUFF[6] = crc16rez;                         //в ответ: Lo crc16
			   MB_BUFF[7] = crc16rez>>8;                      //в ответ: Hi crc16
			   txPac = 8; //должен быть кол-во отправляемых байт -1
			   MB_MyPac_OK=1;
			}//end Команда 0x10

//********************** Команда 06(0x06) ЗАП�?СЬ 1 регистра ******************
			if ( (MB_BUFF[1]==0x06) && (MB_buffCount==8) && (!Crc16(MB_BUFF, 8)) ) {
			   MB_Prot_adr1reg = (MB_BUFF[2]<<8)|MB_BUFF[3];          //адресс первого регистра
			   mb_data[MB_Prot_adr1reg] = (MB_BUFF[4]<<8)|MB_BUFF[5]; //сохраняем положение регистров (реле)
			   txPac = 8; //должен быть кол-во отправляемых байт -1
			   MB_MyPac_OK=1;
			}//end Команда 0x06
		//}
		//if ( (MB_BUFF[0]==0) || (MB_BUFF[0]==mb_data[0]) || (MB_BUFF[0]==30) ) {
//********************** Команда 03 (0x03) ЧТЕНиЕ нескольких регистров ******************
			if ( (MB_BUFF[1]==0x03) && (MB_buffCount==8) && (!Crc16(MB_BUFF, 8)) ){   //проверка команды и вычисление длины пакета
		   		MB_Prot_adr1reg = (MB_BUFF[2]<<8)|MB_BUFF[3]; //адресс первого регистра
		   		MB_Prot_kolreg =  ( (MB_BUFF[4]<<8)|MB_BUFF[5] ); //кол регистров
		   		//далее формирование отвера (запрошеных регистров)
		   		MB_Prot_kolbyt = MB_Prot_kolreg*2;  //количество байт далее
		   		MB_BUFF[2] = MB_Prot_kolbyt;
		   		while(MB_Prot_kolbyt)   //сохраняем положение регистров (реле)
		   			{
		   			MB_BUFF[--MB_Prot_kolbyt+3] = mb_data[MB_Prot_kolreg + MB_Prot_adr1reg-1];
		   			MB_BUFF[--MB_Prot_kolbyt+3] = (mb_data[MB_Prot_kolreg-- + MB_Prot_adr1reg-1])>>8;
		   			}
		   		crc16rez = Crc16(MB_BUFF, 3+MB_BUFF[2]);                  //расчёт crc16 для ответа
		   		MB_BUFF[3+MB_BUFF[2]] = crc16rez;                         //в ответ: Lo crc16
		   		MB_BUFF[4+MB_BUFF[2]] = crc16rez>>8;                      //в ответ: Hi crc16
		   		txPac = 5+MB_BUFF[2]; //должен быть кол-во отправляемых байт -1
		   		MB_MyPac_OK=1;
			}//end Команда 0x03

//********************** Команда 04 (0x04) ЧТЕНиЕ нескольких регистров ******************
			if ( (MB_BUFF[1]==0x04) && (MB_buffCount==8) && (!Crc16(MB_BUFF, 8)) ){   //проверка команды и вычисление длины пакета
		   		MB_Prot_adr1reg = (MB_BUFF[2]<<8)|MB_BUFF[3]; //адресс первого регистра
		   		MB_Prot_kolreg =  ( (MB_BUFF[4]<<8)|MB_BUFF[5] ); //кол регистров
		   		//далее формирование отвера (запрошеных регистров)
		   		MB_Prot_kolbyt = MB_Prot_kolreg*2;  //количество байт далее
		   		MB_BUFF[2] = MB_Prot_kolbyt;
		   		while(MB_Prot_kolbyt)   //сохраняем положение регистров (реле)
		   			{
					MB_BUFF[--MB_Prot_kolbyt+3] = mb_data[MB_Prot_kolreg + MB_Prot_adr1reg-1];
		   			MB_BUFF[--MB_Prot_kolbyt+3] = (mb_data[MB_Prot_kolreg-- + MB_Prot_adr1reg-1])>>8;
		   			}
		   		crc16rez = Crc16(MB_BUFF, 3+MB_BUFF[2]);                  //расчёт crc16 для ответа
		   		MB_BUFF[3+MB_BUFF[2]] = crc16rez;                         //в ответ: Lo crc16
		   		MB_BUFF[4+MB_BUFF[2]] = crc16rez>>8;                      //в ответ: Hi crc16
		   		txPac = 5+MB_BUFF[2]; //должен быть кол-во отправляемых байт -1
		   		MB_MyPac_OK=1;
			}//end Команда 0x04
		}//end MB_BUFF[0] (My ID)
//********************** Команда 04 (0x04) ЧТЕНиЕ нескольких регистров ID=30 ******************
		if (MB_BUFF[0]==30) {
			//mb_data[OUT_IST] |= (1<<5);
			if ( (MB_BUFF[1]==0x03) && (MB_buffCount==9) && (!Crc16(MB_BUFF, 9)) ) {
				//mb_data[OUT_IST] |= (1<<4);
				mb_data[GPM_V_L] = ( (MB_BUFF[3]<<8)|(MB_BUFF[4]) ); //GPM_V_H    123  // Напряжение с GPM-78213 (H)
				mb_data[GPM_V_H] = ( (MB_BUFF[5]<<8)|(MB_BUFF[6]) );
				tekysheeNapr.byte1 = mb_data[GPM_V_H];
				tekysheeNapr.byte2 = mb_data[GPM_V_H]>>8;
				tekysheeNapr.byte3 = mb_data[GPM_V_L];
				tekysheeNapr.byte4 = mb_data[GPM_V_L]>>8;
				// Проверка изменения напряжения
				if (tekysheeNapr.tekysheeU==provSvyaz) {provSvyazCount++;}//provSvyazCount
				else {provSvyazCount=0; errGPM=0;}
				if (provSvyazCount>=1000) {errGPM=1;}
				provSvyaz = tekysheeNapr.tekysheeU;
				//errGPM=0;
			}
			//else {errGPM=1;}
		}//end Команда 0x04 ID=30
		//else {errGPM=1;}
	} //end MB_MyPac_OK


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
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_USART3
                              |RCC_PERIPHCLK_I2C1|RCC_PERIPHCLK_TIM1
                              |RCC_PERIPHCLK_TIM2|RCC_PERIPHCLK_TIM34;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_HCLK;
  PeriphClkInit.Tim2ClockSelection = RCC_TIM2CLK_HCLK;
  PeriphClkInit.Tim34ClockSelection = RCC_TIM34CLK_HCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x2000090E;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 99;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 53;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */
  //htim2.Instance->ARR = 1699;
  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 719;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 169;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 79;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 719;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_RS485Ex_Init(&huart1, UART_DE_POLARITY_HIGH, 0, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_RS485Ex_Init(&huart3, UART_DE_POLARITY_HIGH, 0, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(StepD3_GPIO_Port, StepD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(StepD2_GPIO_Port, StepD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, OutRezerv1_Pin|PinLEDdebug_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : MBadr6_Pin MBadr4_Pin MBadr3_Pin MBadr2_Pin
                           MBadr1_Pin */
  GPIO_InitStruct.Pin = MBadr6_Pin|MBadr4_Pin|MBadr3_Pin|MBadr2_Pin
                          |MBadr1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : MBadr7_Pin */
  GPIO_InitStruct.Pin = MBadr7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(MBadr7_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : InKnB_Pin */
  GPIO_InitStruct.Pin = InKnB_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(InKnB_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : InKnM_Pin InLatrMax_Pin InLatrMin_Pin InRezerv2_Pin */
  GPIO_InitStruct.Pin = InKnM_Pin|InLatrMax_Pin|InLatrMin_Pin|InRezerv2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : InAuto2_Pin InAuto1_Pin */
  GPIO_InitStruct.Pin = InAuto2_Pin|InAuto1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : StepD3_Pin */
  GPIO_InitStruct.Pin = StepD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(StepD3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : StepD2_Pin */
  GPIO_InitStruct.Pin = StepD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(StepD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : OutRezerv1_Pin PinLEDdebug_Pin */
  GPIO_InitStruct.Pin = OutRezerv1_Pin|PinLEDdebug_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : MBadr5_Pin MBadr0_Pin */
  GPIO_InitStruct.Pin = MBadr5_Pin|MBadr0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

