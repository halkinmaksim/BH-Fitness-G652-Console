#ifndef CFM_BHFITNESS_H
#define CFM_BHFITNESS_H

#include <stdint.h>
#include "stm32f1xx_hal.h"
//#include <stm32f1xx.h>

/*
Обратная связь от частотника является геркон, время между замыканиями геркона

16.04.2018	Проверить на сколько изменится скорость при 10 щелчках SpdInc
	Изменяется примерно на 20 -30 мсек


*/

/*			Команды для частоттника			*/

#define FREQ_CONTROLL_CMD_RUN			0x01	/*	Запуск движка*/
#define FREQ_CONTROLL_CMD_STOP			0x02	/*	Остановка движка*/
#define FREQ_CONTROLL_CMD_VEL_INC		0x03	/*	Увеличить скорость*/
#define FREQ_CONTROLL_CMD_VEL_DEC		0x04	/*	Уменьшить скорость*/
#define FREQ_CONTROLL_CMD_GET_FREQ		0x05	/*	Вернуть частоту*/


typedef enum{
	CFM_BH_STATE_STOP			= 0x00U,	/*	Двигатель остановлен	*/
	CFM_BH_STATE_RUN			= 0x01U,	/*	Двигатель запущен	*/
	CFM_BH_STATE_ERR			= 0x10U		/*	Ошибка работы двигателя	*/
}CFM_BH_StateTypeDef;

typedef enum{
	CFM_BH_STATE_INCDEC_LOW		= 0x00U,
	CFM_BH_STATE_INCDEC_HEIGHT	= 0x01U
}CFM_BH_IncDecStateTypeDef;


//	Состояние интерфейса
typedef enum
{
	CFM_STATE_IE_READY			= 0x00U,	/*	Готов для передачи данных	*/
	CFM_STATE_IE_WAIT			= 0x01U,	/*	Ожидание ответа	*/
	CFM_STATE_IE_DATA_READ		= 0x02U,	/*	Пришли данные, необходимо обработать */
	CFM_STATE_IE_ANS_TIMEOUT	= 0x03U		/*	Превышено время ожидания ответа	*/
}CFM_StateInterfaceTypeDef;

//	Регистр состояния
typedef enum
{
	CFM_STATE_REG_NOMOT			= 0xA2U,	/*	Нет вращения двигателя	*/
	CFM_STATE_REG_MOT			= 0x55U,	/*	Двигатель вращается	*/
	CFM_STATE_REG_SERVICE		= 0x25U,	/*	Преобразователь в сервисном режиме */
	CFM_STATE_REG_STOPDC		= 0x42U,	/*	Идет торможение постоянным током.	*/
	CFM_STATE_REG_RESTART		= 0x1BU		/*	Выдержка времени перед перезапуском двигателя, после перегрузки по току	*/
}CFM_StateRegistrTypeDef;

typedef struct{
	//uint32_t 				period_t;				//Время между прерываниями геркона
	//uint32_t 				max_period_t;			//Максимальное время между прерываниями геркона
	//uint32_t 				SetVel;					//Заданная скорость(задает пользователь) в км/ч *10
	//uint32_t				cntIncDecHeight;		//	Время высокого уровня на ножках увеличения уменьшения скорости
	//uint32_t				maxIncDecHeight;		//	Время высокого уровня на ножках увеличения уменьшения скорости
	CFM_BH_StateTypeDef 		cfmStateTypeDef;		//	Состояние двигателя
	CFM_BH_IncDecStateTypeDef 	cfmIncDecStateTypeDef;	//	Состояние пинов увеличения/уменьшения скорости
	// Делаем по образу Precor
	CFM_StateInterfaceTypeDef	cfmStateIfTypeDef;		//	Состояние интерфейса
	CFM_StateRegistrTypeDef 	cfmStateRegistrTypeDef;	//	Состояние частотника
	
	uint32_t	CFMfreq;								//	Частота на выходе частотника(здесь это отсчеты времени)
	uint32_t	CFMvel;									//	Скорость на выходе частотника
	uint32_t	SetCFMfreq;								//	Заданная частота
	uint32_t	SetCFMvel;								//	Заданная частота
	uint32_t	CountTimeReq;							//	Время прошедшее с момента запросса
	uint32_t	CountTimeWork;							//	Время прошедшее с момента запуска дорожки
	uint32_t	CountTimeStart;							//	Время перед запуском
	
}CFM_BH_TypeDef;

extern int TimeGerkon;


extern void gpio_StartCFM(void);
extern void gpio_StopCFM(void);
extern void gpio_SpdIncCFMHigth(void);
extern void gpio_SpdIncCFMLow(void);
extern void gpio_SpdDecCFMHigth(void);
extern void gpio_SpdDecCFMLow(void);


//	Процесс управления частотником
void ProcCFM(CFM_BH_TypeDef* cfmtypedef);
unsigned int FreqConvControll(unsigned int cmd,unsigned int param);
void FreqCorrect(CFM_BH_TypeDef* cfmtypedef);
unsigned int timeGerconeToVel(unsigned int time_gerkone);


void StartMotor(CFM_BH_TypeDef* cfmdef);
void StopMotor(CFM_BH_TypeDef* cfmdef);
void SetVel(CFM_BH_TypeDef* cfmdef,int vel);
void ProcCFMAns(uint8_t* buff, int sizebuff,CFM_BH_TypeDef* cfmdef);


#endif	/*	CFM_BHFITNESS_H	*/
