/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H__
#define __MAIN_H__

/* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

#include "cfm_bhfitness.h"
#include "max7219.h"
#include "keyboard.h"

#include "liftmotor.h"
#include "low_gpio.h"

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define LIFT_IN_Pin GPIO_PIN_0
#define LIFT_IN_GPIO_Port GPIOA
#define SPD_IN_Pin GPIO_PIN_1
#define SPD_IN_GPIO_Port GPIOA
#define SPD_DEC_Pin GPIO_PIN_7
#define SPD_DEC_GPIO_Port GPIOA
#define SPD_INC_Pin GPIO_PIN_0
#define SPD_INC_GPIO_Port GPIOB
#define STOP_Pin GPIO_PIN_1
#define STOP_GPIO_Port GPIOB
#define LIFT_UP_Pin GPIO_PIN_10
#define LIFT_UP_GPIO_Port GPIOB
#define LIFT_DOWN_Pin GPIO_PIN_11
#define LIFT_DOWN_GPIO_Port GPIOB
#define SPI2_CS_Pin GPIO_PIN_14
#define SPI2_CS_GPIO_Port GPIOB
#define C1_Pin GPIO_PIN_8
#define C1_GPIO_Port GPIOA
#define C2_Pin GPIO_PIN_9
#define C2_GPIO_Port GPIOA
#define C3_Pin GPIO_PIN_10
#define C3_GPIO_Port GPIOA
#define C4_Pin GPIO_PIN_11
#define C4_GPIO_Port GPIOA
#define R1_Pin GPIO_PIN_3
#define R1_GPIO_Port GPIOB
#define R2_Pin GPIO_PIN_4
#define R2_GPIO_Port GPIOB
#define R3_Pin GPIO_PIN_5
#define R3_GPIO_Port GPIOB
#define R4_Pin GPIO_PIN_6
#define R4_GPIO_Port GPIOB
#define R5_Pin GPIO_PIN_7
#define R5_GPIO_Port GPIOB
#define R6_Pin GPIO_PIN_8
#define R6_GPIO_Port GPIOB

/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the 
  *        HAL drivers code
  */
/* #define USE_FULL_ASSERT    1U */

/* USER CODE BEGIN Private defines */

#define MIN_SPEED						2.0
#define MAX_SPEED						18.0

/*					Флаги отображения индикатора								*/

#define VIEW_TIME					1	/*	Отображать время бега				*/
#define VIEW_DISTANCE				2	/*	Отображать дистанцию бега			*/
#define VIEW_REAL_VEL				3	/*	Отображать реальную скорость		*/
#define VIEW_SET_VEL				4	/*	Отображать заданную скорость		*/
#define VIEW_DIST_CNT				5	/*	Отображать число оборотов вала		*/
#define VIEW_ANGLE					6	/*	Отображать число оборотов вала		*/
#define VIEW_KEY_KOD				7	/*	Отображать число оборотов вала		*/

#define VIEW_ERR					15	/*	Отображать ошибку 					*/
#define VIEW_ERR_UPDOWN				16	/*	Отображать ошибку двигателя подъема */

#define DISP_1						1	/*	Адрес дисплея 1 				*/
#define DISP_2						5	/*	Адрес дисплея 2 				*/
#define DISP_3						9	/*	Адрес дисплея 3 				*/
#define DISP_4						13	/*	Адрес дисплея 4 				*/




/*						Флаги состояния дорожки								*/

#define STATUS_IS_RUN				1	/*	дорожка крутится				*/
#define STATUS_IS_DEC_STOPPED		2	/*	дорожка в процессе остановки	*/
#define STATUS_IS_STARTED			3	/*	дорожка в процессе разгона		*/
#define STATUS_IS_STOPPED			4	/*	Дорожка остановилась			*/

#define STATUS_ERR_ENCODER				20	/*	Нет обратной связи от энкодера	*/
#define STATUS_ERR_STOP_KEY				21	/*	Кнопка стоп						*/
#define STATUS_ERR_ENCODER_MOTUPDOWN	22	/*	Ошибка датчика двигателя подъема	*/

//******************************************************************************
//			            Коды клавиш
//******************************************************************************
#define KEY_KOD_START					2
#define KEY_KOD_STOP					31
#define KEY_KOD_SPEEDUP					13
#define KEY_KOD_SPEEDDOWN				19
#define KEY_KOD_KONSUP					1
//#define KEY_KOD_KONSDOWN				10
#define KEY_KOD_KONSDOWN				7

typedef struct
{
	unsigned int 	SetAngle;			//Заданный угол наклона ( от 0 до 16)
	unsigned int 	CurAngle;			//Текущий угол наклона (значение АЦП)
	unsigned int 	PrevAngle;			//Предыдущий угол наклона (Значение АЦП)
	unsigned int 	Statuse;			//Состояние
	unsigned int 	TimeUpDown;			//Время прошедшее с момента подъема опускания

}MOT_UP_DUWN_Define;


//	Регистр состояния
typedef enum
{
	DRG_STATE_STOP			= 0x01U,	/*Дорожка остановлена*/
	DRG_STATE_STOPED			= 0x02U,	/*Дорожка в процессе остановки*/
	DRG_STATE_STARTED			= 0x03U,	/*Дорожка в процессе запуска*/
	DRG_STATE_MOTION			= 0x04U	/*Дорожка движется*/
}DorogkaState;
	
typedef struct
{
	unsigned int 	DiametrVala;		//Диаметр приводного вала полотна
	unsigned int 	TimeToCalcSpeed;	//Время через которое происходит расчет времени
	unsigned int 	PrevValCount;		//Предыдущее значение счетчика
	long int 		CurrValCount;		//Текущее значение счетчика
	unsigned int 	CountDist;			//Счетчик дистанции
	unsigned int 	Angle;				//Угол наклона
	float 			Distance;			//Прошедшая дистанция (000,0)
	float 			CurrentVel;			//Текущая скорость
	float 			SettVel;			//Заданная скорость
	unsigned int	CurrwntFreqCh;		//Текущая частота частотника
	unsigned int	SetFreqCh;			//Заданная частота частотника
	DorogkaState	Status;				//Текущее состояние дорожки
	unsigned int	Time;				//Время прошедшее со старта программы
	unsigned int	CountTimeStart;		//Время перед стартом
	unsigned int	StatusStopKey;				//Текущее состояние дорожки
	//MOT_UP_DUWN_Define AngleStr;

//	CFM_BH_TypeDef* pCfmTypeDef;
//	LIFT_TypeDef	LiftTypeDef;		//Структура описывающая угол подъема
	//CFM_StateRegistrTypeDef rr;
	//CFM_BH_TypeDef	CfmTypeDef;			//Структура описывающая частотник
	
}DOROGKA_TypeDef;



void ViewDisp(unsigned int ndisp, unsigned int flag,uint32_t data);
int SecToTime(int sec);

void FillDataGerkone(int data_gerkone);
int GetGerkoneTime(void);


void StartProgram(int npr);
void KeyExCmd(char keykod);
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
