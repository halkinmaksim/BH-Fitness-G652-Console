#ifndef LIFTMOTOR_H
#define LIFTMOTOR_H

/*
	 *	Переделываем по Precor, стараемся внести минимум изменений
	06.04.2018
		Для исключения ошибок, необходимо задавать интервал с учетом дискретности
		Проверить преобразования из % и обратно
		Продумать ошибку максимума и минимума(скорее всего один раз во время включения)
*/

#include <math.h>
#include <stdint.h>


#define LIFT_ST_READY			0		/*	Ничего не происходит	*/
#define LIFT_ST_UP				1		/*	Движется вверх	*/
#define LIFT_ST_DOWN			2		/*	Движется вниз		*/
#define LIFT_ST_WAIT_ADC		4		/*	Ожидание АЦП		*/
#define LIFT_ST_KALIBR			8		/*	Идет калибровка	*/

#define LIFT_ST_ERR				32	/*	Ошибка двигателя подъема	*/


#define LIFT_ADC_BUFF_SIZE		32	/*	Размер буффера для данных АЦП*/

#define LIFT_ADC_TIME_CH		2	/*	Время за которое должны произойти изменения АЦП*/
#define LIFT_ADC_MIN_IZMINENIYA	20	/*	Минимальное изменение АЦП для угла подъема */

extern int SredADC;


//	Состояние двигателя угла подъема
typedef enum
{
	LIFT_STATE_MOTION_STOP		= 0x00U,
	LIFT_STATE_MOTION_UP		= 0x01U,
	LIFT_STATE_MOTION_DOWN		= 0x02U
}LIFT_StateMotionTypeDef;

//	Ошибки двигателя угла подъема
typedef enum
{
	LIFT_ERRORE_NONE			= 0x00U,		/*	!< No error             */
	LIFT_ERRORE_MIN				= 0x01U,		/*	*/
	LIFT_ERRORE_MAX				= 0x02U,		/*	Exceeded maximum ADC value*/
	LIFT_ERRORE_MOTION			= 0x03U			/*	No motion lift motor*/
}LIFT_ErroreMotionTypeDef;
/*
Структура для работы с углом подъема
*/
typedef struct
{
	uint32_t 					MaxValuePrc;		//	Для расчета угла
	uint32_t					MaxValueAdc;
	uint32_t 					MinValuePrc;
	uint32_t 					MinValueAdc;
	LIFT_StateMotionTypeDef		State;				//	Состояние двигателя подъема
	LIFT_ErroreMotionTypeDef	Errore;

	uint32_t 					Sadcprc;
	uint32_t 					CurrentAdc;
	uint32_t 					TimeChgAdc;			//	Время за которое должно пройти изменение АЦП
	uint32_t					DeltaAdc;			//	Значение на которое должно было измениться АЦП
	uint32_t 					PrevAngleAdc;		//	Предыдущее значение АЦП
	uint32_t 					CountTimeMotion;	//	Время движения двигателя
	uint32_t 					SetAngleAdc;		//	Заданное значение АЦП
	uint32_t 					SetAnglePrc;		//	Заданноеx значение в процентах
}LIFT_TypeDef;


typedef struct
{
	unsigned int 	SetAngle_prc;		//	Заданный угол наклона ( от 0 до 16)
	unsigned int 	SetAngle_adc;		//	Заданный угол наклона ( от 0 до 16)
	unsigned int 	CurAngle_prc;		//	Текущий угол наклона (значение АЦП)
	unsigned int 	CurAngle_adc;		//	Текущий угол наклона (значение АЦП)
	unsigned int 	PrevAngle_adc;		//	Предыдущий угол наклона (Значение АЦП)
	unsigned int 	Statuse;			//	Состояние
	unsigned int 	TimeUpDown;			//	Время прошедшее с момента подъема опускания
	unsigned int 	MaxValuePrc;		//	Для расчета угла
	unsigned int 	MaxValueAdc;
	unsigned int 	MinValuePrc;
	unsigned int 	MinValueAdc;
}LiftDef;

//extern LiftDef liftdefine;
extern int LiftADCbuff[LIFT_ADC_BUFF_SIZE];	//Буффер АЦП
//extern LIFT_TypeDef	LiftTypeDef;

extern void gpio_MotionUp(void);
extern void gpio_MotionDown(void);
extern void gpio_MotionStop(void);

unsigned int adc2prc(unsigned int adcval,LIFT_TypeDef* liftdef);
unsigned int prc2adc(unsigned int prcval,LIFT_TypeDef* liftdef);

//void InitLiftModule(void);					//Задание переменных
void SetAngle(LIFT_TypeDef* liftdef);		//Установить угол (в %)
//void SetAngle(unsigned int prcval);		//Установить угол (в %)
void KalibrLift(void);					//Калибровка угла наклона
//void ProcLift(void);					//Процесс контролля двигателя подъема
void ProcLift(LIFT_TypeDef* liftdef);	//Процесс контролля двигателя подъема
void FillDataAdc(int data_adc);			//Заполняем данные АЦП

int GetAdcLift(void);						//Выдать значение АЦП
void ChangIspADC(void);						//Проверка изменений АЦП
int module(int z1, int z2);

int AdcIsChange(LIFT_TypeDef* liftdef);


extern LIFT_TypeDef	LiftTypeDef;

#endif /* LIFTMOTOR_H */
