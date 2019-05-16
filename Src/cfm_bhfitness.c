
#include "cfm_bhfitness.h"




/*------------------------------------------------------------------------------
		ПРОЦЕСС КОНТРОЛЯ ЧАСТОТНИКА
------------------------------------------------------------------------------*/
void ProcCFM(CFM_BH_TypeDef* cfmdef)
{
	//	Если ошибки выходим
	if(cfmdef->cfmStateTypeDef == CFM_BH_STATE_ERR)
		return;
	
	//**************************************************************************
	//Делаем по образу и подобию Precor
	if(cfmdef->cfmStateIfTypeDef == CFM_STATE_IE_READY)
	{
		//cfmdef->CountTimeReq+= 1;
		//	Частотник готов к передачи данных
		//	Сначало смотрим что будем передавать,
		// Если скорость установки отлична от заданной, отправляем скорость
		if(cfmdef->SetCFMvel != cfmdef->CFMvel)
		{
			if(cfmdef->SetCFMvel == 0)
			{
				//Задана 0-вая скорость, проверяем остановлен двигатель или нет
				if(cfmdef->cfmStateRegistrTypeDef != CFM_STATE_REG_NOMOT)
				{
					//	Если двигатель вращаеться, останавливаем его
					StopMotor(cfmdef);
				}
				else
				{
					//Иначе просто обнуляем скорость, чтобы не попадать сюда
					cfmdef->SetCFMvel =0;
				}
			}
			//Иначе необходимо сделать коррекцию скорости
			else
			{
				if(cfmdef->cfmStateRegistrTypeDef == CFM_STATE_REG_MOT)
				{
					//	Двигатель уже запущен, поэтому просто изменяем скорость
					SetVel(cfmdef,cfmdef->SetCFMvel);
				}
				else
				{
					//	Двигатель не запущен, запустим его, потом изменим скорость
					StartMotor(cfmdef);
				}
			}
		}
		else
		{
			//	Скорее всего нужно проверить запущен или нет двигатель
			//	Дальше смотрим таймер, и отправляем если нужно запрос состояния

		}
		
	}
	

	//**************************************************************************
	
	
}


/**
* @brief Функция корректировки частоты
  * @note 
  * @note 
  * @retval None
  */
void CorrectFreq(int value, int setspeed)
{
	if(value < setspeed)
	{
		if((setspeed - value)>2)
			FreqConvControll(FREQ_CONTROLL_CMD_VEL_INC,10);
	}
	if(value > setspeed)
	{
		if((value - setspeed)>2)
			FreqConvControll(FREQ_CONTROLL_CMD_VEL_DEC,10);
	}
}
/**
* @brief Функция управления частотником
  * @note 
  * @note 
  * @retval None
  */
unsigned int FreqConvControll(unsigned int cmd,unsigned int param)
{
	switch(cmd){
		case FREQ_CONTROLL_CMD_RUN:
			gpio_StartCFM();
			break;
		case FREQ_CONTROLL_CMD_STOP:
			gpio_StopCFM();
			break;
		case FREQ_CONTROLL_CMD_VEL_INC:
			//Установить на какоето время 1 и отпустить
			gpio_SpdIncCFMHigth();
			HAL_Delay(10);
			gpio_SpdIncCFMLow();
			break;
		case FREQ_CONTROLL_CMD_VEL_DEC:
			//Установить на какоето время 1 и отпустить
			gpio_SpdDecCFMHigth();
			HAL_Delay(10);
			gpio_SpdDecCFMLow();
	//	//	flagDisp2 = VIEW_SET_VEL;
	//	//	RemoveFuncTime(&ViewDist);
	//	//	AddFuncTime(&ViewDist,105);

			break;
		case FREQ_CONTROLL_CMD_GET_FREQ:
			break;
	}
	return 0;
}

/**
* @brief Функция корректировки частоты на основании интервалов времени геркона
  * @note 
  * @note 
  * @retval None
  */
void FreqCorrect(CFM_BH_TypeDef* cfmtypedef)
{
	
}

/**
* @brief Временной интервал геркона в скорость
  * @note Скорость в км/час x 10
  * @note 
  * @retval None
  */
unsigned int timeGerconeToVel(unsigned int time_gerkone)
{
	if(time_gerkone==0)
		return 0;
	return 10000/time_gerkone;
}




/**
* @brief Запуск двигателя на минимальной скрости
  * @note 
  * @note 
  * @retval None
  */
void StartMotor(CFM_BH_TypeDef* cfmdef)
{
	gpio_StartCFM();
	FreqConvControll(FREQ_CONTROLL_CMD_VEL_INC,10);
	cfmdef->cfmStateRegistrTypeDef = CFM_STATE_REG_MOT;
}

/**
* @brief Остановка двигателя
  * @note 
  * @note 
  * @retval None
  */
void StopMotor(CFM_BH_TypeDef* cfmdef)
{
	gpio_StopCFM();
	cfmdef->SetCFMvel = 0;
	cfmdef->cfmStateRegistrTypeDef = CFM_STATE_REG_NOMOT;

}
/*----------------------------------------------------------------------------
 	 Задать скорость дорожки
 	 (Скорость задается на основании диаметров шкива и приводного валов)

 ----------------------------------------------------------------------------*/
/**
* @brief Задать скорость дорожки Скорость задается на основании диаметров шкива и приводного валов)
  * @note 
  * @note 
  * @retval None
  */
void SetVel(CFM_BH_TypeDef* cfmdef,int vel)
{
	//int curr_vel = timeGerconeToVel(TimeGerkon);
	//int curr_vel = timeGerconeToVel(cfmdef->CFMfreq);
	CorrectFreq(cfmdef->CFMvel,vel);
}










/**
* @brief Отпустить кнопки + -
  * @note 
  * @note 
  * @retval None
  */
void StopEncDecFreq(CFM_BH_TypeDef* cfmtypedef){

	gpio_SpdDecCFMLow();
	cfmtypedef->cfmIncDecStateTypeDef = CFM_BH_STATE_INCDEC_LOW;
}
