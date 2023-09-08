/*
 * MB_veribl_my.h
 *
 *  Created on: 1 окт. 2021 г.
 *      Author: starv
 */

#ifndef INC_MB_VERIBL_MY_H_
#define INC_MB_VERIBL_MY_H_

////////..................... Дефайны


// Значения МБ -регистров для чтен/зап (Holding Registers)   mb_data[X]:
#define MB_D_MAS 150         // Размер массива модбас данных

  #define MY_ID       0     // Мой АйДи
  #define UART_SPD    1     // Скорость УАРТ (бит/сек)
  #define UART_UBRR   2     // Содержание регистров
  #define UART_DAT    3     // Нахуя это? (это вроде 7 или 8 бит настройка)
  #define UART_STP    4     // Стоп-битов
  #define UART_PAR    5     // Проверка паритеета
  #define UART_MB_T15 6     // Тайминг 1,5Т для модбуси
  #define UART_MB_T35 7     // Тайминг 3,5Т для модбуси
  #define MAST_LI     8     // Бит Мастер Ли?!  1-да
  #define PARAM_SL    9     // Разные параметры платы (собрать бы всё сюда 8,56,...
  #define MB_DATA0    10
  #define MB_DATA1    11
  #define MB_DATA2    12
  #define MB_DATA3    13
                          // 0-7                                       8-15                                Coil:
      #define MAS_P3  14  // резервный регистр для мастер платы (коммутации и измерений)                  //0-15
      #define MAS_P2  15  // _/_/_/_/_/_/_/_/                          12V/24V/110V/Охлажд/_/_/_/_        //16-31
      #define MAS_P1  16  // I02/I1/I2/I5/U50/U150/Uместа/Uист/        Uбп/Uвнеш/ШунтPA/ЗарC/_/_/U++/U--  //32-47
      #define REL_FUN 17  // capR/cap3/izmU/cap2/Pusk/cap1/Rout/izmI/  obk/_/_/_/_/_/_/_                  //48-63
      #define AVARII  18  // Флаги ошибок и аварий                                                        //64-79

      #define SPEEDF  19    // Обороты вентилятора
      #define TOK_A   20    // Ток ф А
      #define TOK_B   21    // Ток ф В
      #define TOK_C   22    // Ток ф С

      #define COUN_OI 23  // Счётчик превыш тока
      #define COUN_OF 24  // Счётчик обрыва фазы
      #define COUN_KZ 25  // Счётчик КЗ
      #define COUN_ER 26  // Счётчик любых ошибок
//    #define 		  27
//    #define 		  28
//    #define 		  29
//    #define 		  30

  #define IZM_FAZ 31   // Кол-во измер по фазам
  #define ZERO_A  32   // Знач средн точки фазы А
  #define ZERO_B  33   // Знач средн точки фазы B
  #define ZERO_C  34   // Знач средн точки фазы C

  #define IZM_OI  35   // Кол-во измер до Превыш тока
  #define IZM_OF  36   // Кол-во измер до Обрыва фазы
  #define IZM_KZ  37   // Кол-во измер до Коротк зам

  #define OVR_I   38   // Знач превыш тока
  #define OBR_F   39   // Знач обрыва фазы
  #define OVR_IKZ 40   // Знач тока КЗ

  #define SCALADC     41   // Код текущей подобраной шкалы
  #define LED_BR      42   // Яркость индикатора
  #define TIMREL_OFF  43   // отключение реле по таймеру (например при потере связи)
  #define TIM_CAPR    44   // отключение Шунтирующего конденсаторы резистора (когда кандёры зарядились)
  #define TIM_RAZC    45   // Задержка подачи напряжения на место для безопастной разрядки конденаторов
  #define OVR_I5      46   // Знач превыш тока ЭВ5
  #define OVR_I6      47   // Знач превыш тока ЭВ6
  #define OVR_I7      48   // Знач превыш тока ЭВ7
  #define OVR_I8      49   // Знач превыш тока ЭВ8

  #define OBR_FA  50    // Знач обрыва фазы АВТО
  #define OBR_F05 51    // Знач обрыва "половины" фазы
  #define OBR_F03 52    // Знач обрыва "трети" фазы
  #define OBR_F02 53    // Знач обрыва "четверти" фазы
  #define OBR_F1  54    // Знач обрыва фазы 1
  #define OBR_F2  55    // Знач обрыва фазы 2

  #define LED_CMB 56    // Комбинация светодиодов
  #define VRM_RL1 57    // Время задержки переключения реле 1 (с обкатки на замер)
  #define VRM_RL2 58    // Время задержки переключения реле 2 (с замера на обкатку)
  #define CRC_EEP 59    // Значение контрольной суммы EEPROM массива

  #define PROG_VER 60   // Версия прошивки
////////////////////////// Измерения источнтка на нижней плате ///////////////////////////////
  #define IA_IST_H  61  // Ток А источника (float_H)
  #define IA_IST_L  62  // Ток А источника (float_L)
  #define IB_IST_H  63  // Ток B источника (float_H)
  #define IB_IST_L  64  // Ток B источника (float_L)
  #define IC_IST_H  65  // Ток C источника (float_H)
  #define IC_IST_L  66  // Ток C источника (float_L)
  #define UA_IST_H  67  // Напряжение АB источника (float_H)
  #define UA_IST_L  68  // Напряжение АB источника (float_L)
  #define UB_IST_H  69  // Напряжение BC источника (float_H)
  #define UB_IST_L  70  // Напряжение BC источника (float_L)
  #define UC_IST_H  71  // Напряжение CA источника (float_H)
  #define UC_IST_L  72  // Напряжение CA источника (float_L)
  #define IA_IST    73  // Ток А источника
  #define IB_IST    74  // Ток B источника
  #define IC_IST    75  // Ток C источника
  #define UA_IST    76  // Напряжение АB источника
  #define UB_IST    77  // Напряжение BC источника
  #define UC_IST    78  // Напряжение CA источника
  #define COSA_IST  79  // Косинус фи фазы A
  #define COSB_IST  80  // Косинус фи фазы B
  #define COSC_IST  81  // Косинус фи фазы C
  #define WA_IST    82  // Мощность А источника
  #define WB_IST    83  // Мощность B источника
  #define WC_IST    84  // Мощность C источника

#define IN_IST    85  // Состояние входов и сигналов
               #define IN_AVTO1    0  // IN_IST - Режим авто 1
               #define IN_AVTO2    1  // IN_IST - Режим авто 2
               #define IN_AUTO_NOM 2  // IN_IST - Резерв
               #define IN_ATR_MIN  3  // IN_IST - АТР конечник минимум
               #define IN_ATR_MAX  4  // IN_IST - АТР конечник максимум
               #define IN_KN_MEN   5  // IN_IST - Кнопка "Меньше"
               #define IN_KN_BOL   6  // IN_IST - Кнопка "Больше"
               #define IN_KN_MPB   7  // IN_IST - Кнопки Меньше + Больше
               #define IN_STAT_START  8  // IN_IST - Едем(1) Нет(0)
               #define IN_STAT_NAPR   9  // IN_IST - ПоЧасовой(1) Против(0)
               #define IN_STAT_ENAB   10  // IN_IST - Отпустить(1) Держать(0)
               #define IN_AUTO_BOL    11  // IN_IST - используется для авто езды по уставкам
               #define IN_AUTO_MEN    12  // IN_IST - используется для авто езды по уставкам
               #define IN_AUTO_HOM    13  // IN_IST - Авто поиск НУЛЯ(МИН) и МАКС по конечникам
               #define IN_KN_MMMEN    14  // IN_IST - Кнопки долгое Меньше
               #define IN_KN_BBBOL    15  // IN_IST - Кнопки долгое Больше

#define OUT_IST   86  // Состояние выходов и сигналов
               #define OUT_REZERV    0  // OUT_IST - Резерв (вывод на мигание кнопок)
               #define OUT_ED_STEP   1  // OUT_IST - Шаги ЭД
               #define OUT_ED_NAPR   2  // OUT_IST - Направление ЭД
               #define OUT_ED_ENAB   3  // OUT_IST - Вкл работы ЭД
               #define OUT_SLEJEN    8  // OUT_IST - Бит постоянного слежения за U и отработкой ЭД
#define KOM_IST     87
               #define KOM_SLEJEN    0  // KOM_IST - Бит постоянного слежения за U и отработкой ЭД
//#define 			88
//#define 			89

  #define KITW_YSTW  90   // Уставка тревоги
  #define KITW_ID    91   // АйДи
  #define KITW_U     92   // Напряжение
  #define KITW_I_H   93   // Ток(H)
  #define KITW_I_L   94   // Ток(L)
  #define KITW_W_H   95   // Ватт(H)
  #define KITW_W_L   96   // Ватт(L)
  #define KITW_WH_H  97   // Ватт*час(H)
  #define KITW_WH_L  98   // Ватт*час(L)
  #define KITW_HZ    99   // Частота
  #define KITW_COS   100  // Косинус фи
  #define KITW_STAL  101  // Статус тревоги
//#define			 102
//#define			 103
//#define			 104
//#define			 105
//#define			 106
//#define			 107
//#define			 108
//#define			 109

  #define ATR_FSTP       110  // Количество шагов на весь диапазон U
  #define ATR_POL_ABS    111  // Положение щётки абсолютное (в шагах) (может быть отрицательным, где проснётся, там и НОЛЬ)
  #define ATR_POL_OTN    112  // Положение щётки относительное (в шагах) (НОЛЬ там где АТР МИН)
  #define ATR_VTOSH_H    113  // Вольт на шаг (H) Зависит от дробления шага на приводе
  #define ATR_VTOSH_L    114  // Вольт на шаг (L)
  #define ATR_ZADAN_V_H  115  // Заданное напряжение (H) Приходит с панели оператора
  #define ATR_ZADAN_V_L  116  // Заданное напряжение (L)
  #define ATR_ZADAN_SH   117  // Заданный положение (в шагах)
  #define ATR_FSTP_HD    118  // Количество шагов на весь диапазон U (измереное при наладке)
  #define ATR_MIN_V_H    119  // Напряжение на конецевике MIN (H)
  #define ATR_MIN_V_L    120  // Напряжение на конецевике MIN (L)
  #define ATR_MAX_V_H    121  // Напряжение на конецевике MAX (H)
  #define ATR_MAX_V_L    122  // Напряжение на конецевике MAX (L)
/////////////////////////////////////////////// Данные с ВАТТметра GPM-78213 /////////////////////////
  #define GPM_V_H    123  // Напряжение с GPM-78213 (H)
  #define GPM_V_L    124  // Напряжение с GPM-78213 (L)
  #define GPM_I_H    125  // Ток с GPM-78213 (H)
  #define GPM_I_L    126  // Ток с GPM-78213 (L)
  #define GPM_W_H    127  // Мощность с GPM-78213 (H)
  #define GPM_W_L    128  // Мощность с GPM-78213 (L)

/////OK//// ДОБАВИТЬ #define чисто на китайский ваттметр, т.к. запрашивается
/////OK//// с него всё одним пакетом и храниться пусть всё подряд в одном месте
/////OK//// и надо поменять номер стартового региста с которого будет храниться
/////OK//// данные с кит.ват.

  // рабочие переменные
float atr_VTOSH=0.0,	// Вольт на шаг
	  atr_ZADAN_V=0.0,	// Заданное напряжение
	  atr_MIN_V=0.0,	// Напряжение на конецевике MIN
	  atr_MAX_V=0.0;	// Напряжение на конецевике MAX

////////.................... МОДбус ....................////////
uint16_t mb_data[MB_D_MAS]={0,}; // МодБас регистры

volatile uint16_t //MB_dT15=10,       //множитель таймера для получения времени Модбаса Т15
                  MB_dT35=24,       //множитель таймера для получения времени Модбаса Т35
                  //MB_counT15=0,     //счётчик для получения времени Модбаса Т15
                  MB_counT35=0;     //счётчик для получения времени Модбаса Т35

uint8_t MB_BUFF[200]={0,};          //буфер УАРТ 1байт, но регистры 2байта!
uint8_t MB_preBUFF[1]={0},
		MB_buffCount=0;
uint8_t MB_Prot_kolbyt=0,
		MB_MyPac_OK=0;
uint16_t MB_Prot_adr1reg=0,
		 MB_Prot_kolreg=0,
		 MB_Prot_kolzapreg=0;
uint16_t crc16rez=0;
uint16_t rxPac=1;
uint16_t txPac=7;

////////.......... МОДбус для китайского Ваттметра...........////////
volatile uint16_t MB3_dT35=24,         //множитель таймера для получения времени Модбаса Т35
                  MB3_counT35=0;       //счётчик для получения времени Модбаса Т35
uint8_t MB3_BUFF[30]={0,};          //буфер УАРТ3
uint8_t MB3_preBUFF[30]={0,},
		MB3_buffCount=0;
uint8_t MB3_Prot_kolbyt=0,
		MB3_MyPac_OK=0;
uint16_t MB3_Prot_adr1reg=0,
		 MB3_Prot_kolreg=0,
		 MB3_Prot_kolzapreg=0,
		 MB3_Tim_Tx=10000,               //период опроса китайского ваттметра
		 MB3_count_Tx=0;                 //счётчик опроса кит.ват.
uint16_t rx3Pac=25;
uint16_t tx3Pac=8;

//////////////////////// Фильтры кнопок ////////////////////////////////
volatile uint16_t	In_Auto1_count=0,   //счётчик фильтра кнопки Авто1
					In_Auto2_count=0,   //счётчик фильтра кнопки Авто2
					In_Rezerv_count=0,   //счётчик фильтра кнопки Резерв
					In_ATRmin_count=0,   //счётчик фильтра кнопки АТР в минимуме
					In_ATRmax_count=0,   //счётчик фильтра кнопки АТР в максимуме
					In_KnMen_count=0,   //счётчик фильтра кнопки Меньше
					In_KnMMMen_count=0,   //счётчик фильтра кнопки Меньше (долгое нажатие)
					In_KnBol_count=0,   //счётчик фильтра кнопки Больше
					In_KnBBBol_count=0,   //счётчик фильтра кнопки Больше (долгое нажание)
					In_KnMpB_count=0,   //счётчик фильтра кнопки Больше
					In_count=5,        //уставка срабатывания (*10 мс)
					In_count2=100,        //уставка срабатывания (*10 мс)
					Delitel_count=0;    //Делитель для мигания

volatile int16_t	atr_pol_abs=0; //ATR_POL_OTN   112  // Положение щётки относительное (в шагах)

uint16_t count_stepHome=0;
uint8_t signal_0=0,					//сигнализировать "Отмена" или "Отключить"
		signal_1=0,					//сигнализировать "Принять" или "Включить"
		bitEdemNet=0;				//бит для реализации аппаратно-правильного включения ЕД (после выбора направления)
uint16_t out_Sign_count1=0;			//счётчик сигнала

///////////////.......... Уставки по умолчанию ..........///////////////


#endif /* INC_MB_VERIBL_MY_H_ */



////////////////////////////////////////////////////////////////////////
//////////////////////// Ч Е Р Н О В И К И /////////////////////////////
////////////////////////////////////////////////////////////////////////

//********************** Т15 надо ли? хз хз... **********************//
/*  if (MB_counT15>=MB_dT15) {
	   MB_counT15=0;
	   MB_buffCount=0;
	   HAL_UART_Receive_IT(&huart1, (uint8_t*)MB_preBUFF, rxPac);
	   //HAL_GPIO_TogglePin(GPIOB, PinLEDdebug_Pin);
	   //HAL_GPIO_WritePin(GPIOB, PinLEDdebug_Pin, GPIO_PIN_SET);
   }*/

//if (MB_BUFF[0]==mb_data[0]){MB_BUFF[0]=0; HAL_UART_Transmit_IT(&huart1,(uint8_t*)"MyID!\n\r", 7);}
//HAL_UART_Transmit_IT(&huart1,(uint8_t*)"He!\n\r", 5);
//HAL_Delay(1000);

	  //HAL_UART_Transmit(&huart3, TESTmassiv, 4,100);
	  //HAL_UART_Transmit(&huart2,str, strlen((char *)str), 100);

	  //HAL_Delay(100);
	  /*uint16_t MB_dT15=10,       // множитель таймера для получения времени Модбаса Т15
	             MB_dT35=24,       // множитель таймера для получения времени Модбаса Т35
	             MB_counT15,       // счётчик для получения времени Модбаса Т15
	             MB_counT35;       // счётчик для получения времени Модбаса Т35
*/
	  //if (MB_counT15>=MB_dT15){MB_counT15=0;}
	  //if (MB_counT35>=MB_dT35){MB_counT35=0;}


 /*if(flagADCc){
	  flagADCc=0;
	  //HAL_ADC_Stop_DMA(&hadc1);
	  sprintf(TESTmassiv, "Popizdovalllli!: \n\r");
	  HAL_UART_Transmit_IT(&huart3,TESTmassiv, strlen((char *)TESTmassiv));
	  HAL_Delay(100);
	  for (countFas=0;countFas<100;countFas++) {
	    sprintf(TESTmassiv, "adcABC%d= %d %d %d \n\r",countFas,adc_ABC[0+(3*countFas)],adc_ABC[1+(3*countFas)],adc_ABC[2+(3*countFas)]);
	    HAL_UART_Transmit_IT(&huart3,TESTmassiv, strlen((char *)TESTmassiv));
	    HAL_Delay(20);
	  }
	  //HAL_Delay(200);
	  //HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&adc_ABC, 3);
 }*/
	  //sprintf(TESTmassiv, "adcABC= %d %d %d \n\r",adc_A[0],adc_B[0],adc_C[0]);
	  //HAL_UART_Transmit_IT(&huart3,TESTmassiv, strlen((char *)TESTmassiv));

	  //huart3.Instance->TDR = 36;

	  //sprintf(TESTmassiv, "Popizdovalllli!: \n\r");
	  //HAL_UART_Transmit_IT(&huart1,TESTmassiv, strlen((char *)TESTmassiv));
	  //HAL_Delay(10000);
	  /*for (countFas=0;countFas<150;countFas++) {
	    //sprintf(TESTmassiv, "adcABC%d= %d %d %d \n\r",countFas,adc_ABC[0+(3*countFas)],adc_ABC[1+(3*countFas)],adc_ABC[2+(3*countFas)]);
	    sprintf(TESTmassiv, "adcABC%d= %d %d %d \n\r",countFas,adc_A[countFas],adc_B[countFas],adc_C[countFas]);
	    HAL_UART_Transmit_IT(&huart1,TESTmassiv, strlen((char *)TESTmassiv));
	    HAL_Delay(20);
	  }*/
