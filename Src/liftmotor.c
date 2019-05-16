/*
 * Галкин Максим
 * hakmax@bk.ru
 * 066-51-68-235
 *
 *	Модуль управления углом подъема
 *
 *		Во избежании ошибки, необходимо сравнивать значение АЦП с установленными
 *	только во время работы двигателя
 *		Для внешней программы необходимы только высокоуровневые функции
 *
 *		Угол наклона задается в процентах * 10 (т.е. 2,5% = 25)
 * */

#include "liftmotor.h"


LiftDef liftdefine;						//Структура угла подъема
int LiftADCbuff[LIFT_ADC_BUFF_SIZE];	//Буффер АЦП
int SredADC;
//LIFT_TypeDef	LiftTypeDef;



/*----------------------------------------------------------------------------
* 	преобразование АЦП в угол наклона
 ----------------------------------------------------------------------------*/
unsigned int adc2prc(unsigned int adcval,LIFT_TypeDef* liftdef)
{
	unsigned int dadc = liftdef->MaxValueAdc - liftdef->MinValueAdc;
	unsigned int dprc = (liftdef->MaxValuePrc - liftdef->MinValuePrc)*100;
	int buff;
    adcval = adcval+(liftdef->Sadcprc)/2;
    buff = (adcval - liftdef->MinValueAdc)/(liftdef->Sadcprc)+liftdef->MinValuePrc;
    if(adcval>liftdef->MaxValueAdc)
        return liftdef->MaxValuePrc;
	if(adcval<liftdef->MinValueAdc)
		return liftdef->MinValuePrc;
	return (unsigned int)buff;
    /*
	if(dadc!=0)
	{
		buff = ((dprc/dadc)*(adcval - liftdef->MinValueAdc)/100)+liftdef->MinValuePrc;

		if(adcval>liftdef->MaxValueAdc)
			return liftdef->MaxValuePrc;
		if(adcval<liftdef->MinValueAdc)
			return liftdef->MinValuePrc;
		return (unsigned int)buff;
		//return ((dprc/dadc)*(adcval - liftdefine.MinValueAdc)/100)+liftdefine.MinValuePrc;
	}
	else
	{
		return 0;
	}
    */
}

/*----------------------------------------------------------------------------
* 	преобразование угла наклона в АЦП
----------------------------------------------------------------------------*/
unsigned int prc2adc(unsigned int prcval,LIFT_TypeDef* liftdef)
{
	unsigned int dadc = liftdef->MaxValueAdc - liftdef->MinValueAdc;
	unsigned int dprc = liftdef->MaxValuePrc - liftdef->MinValuePrc;
	unsigned int buff;
    buff = (prcval - liftdef->MinValuePrc)*(liftdef->Sadcprc)+liftdef->MinValueAdc;
	return buff;
    
    /*
	if(dprc!=0)
	{
		buff = ((float)dadc/dprc);
		buff = buff*(prcval - liftdef->MinValuePrc)+liftdef->MinValueAdc;
	//	buff = (dadc/dprc)*(prcval - liftdefine.MinValuePrc)+liftdefine.MinValueAdc;
		return (int)buff;
	}
	else
	{
		
        return 0;
	}
    */
}

void SetAngle(LIFT_TypeDef* liftdef)
{
	int currentanglPrc;
	unsigned int prcval;
	//if(liftdef->)
	currentanglPrc = adc2prc((liftdef->CurrentAdc)-30 ,liftdef);
	prcval = adc2prc(liftdef->SetAngleAdc,liftdef);
	if(currentanglPrc > prcval)
	{
		// Говорим двигаться вниз
		liftdef->CountTimeMotion = 0;
		liftdef->PrevAngleAdc = liftdef->CurrentAdc;
		liftdef->State = LIFT_STATE_MOTION_DOWN;
		gpio_MotionDown();
		return;
	}
	if(currentanglPrc < prcval)
	{
		// Говорим двигаться вверх
		liftdef->CountTimeMotion = 0;
		liftdef->PrevAngleAdc = liftdef->CurrentAdc;
		liftdef->State = LIFT_STATE_MOTION_UP;
		gpio_MotionUp();
		return;
	}
}
/*----------------------------------------------------------------------------
* 	Установить угол (в %)
----------------------------------------------------------------------------*/
/*
void SetAngle(unsigned int prcval)
{
	int currentanglPrc;
	//	Исключаем ошибку минимума и максимума	
	if(prcval > liftdefine.MaxValuePrc)
		return;
	if(prcval < liftdefine.MinValuePrc)
		return;
	currentanglPrc = adc2prc(GetAdcLift());
	//	Если мотор находится в движении, правим уставку
	//	При этом проверяем направление движения

	if(liftdefine.Statuse != LIFT_ST_READY)
	{
		if(liftdefine.Statuse == LIFT_ST_UP)
		{
			if(currentanglPrc > prcval)
			{
				// Говорим двигаться вниз
				liftdefine.SetAngle_prc = prcval;
				liftdefine.CurAngle_prc = prcval;
				liftdefine.SetAngle_adc = prc2adc(prcval);
				liftdefine.Statuse = LIFT_ST_DOWN;
				liftdefine.TimeUpDown = 0;
				gpio_MotionDown();
				return;
			}
			else
			{
				liftdefine.SetAngle_prc = prcval;
				liftdefine.CurAngle_prc = prcval;
				liftdefine.SetAngle_adc = prc2adc(prcval);
			}
		}
		if(liftdefine.Statuse == LIFT_ST_DOWN)
		{
			if(currentanglPrc < prcval)
			{
				// Говорим двигаться вниз
				liftdefine.SetAngle_prc = prcval;
				liftdefine.CurAngle_prc = prcval;
				liftdefine.SetAngle_adc = prc2adc(prcval);
				liftdefine.Statuse = LIFT_ST_UP;
				liftdefine.TimeUpDown = 0;
				gpio_MotionUp();
				return;
			}
			else
			{
				liftdefine.SetAngle_prc = prcval;
				liftdefine.CurAngle_prc = prcval;
				liftdefine.SetAngle_adc = prc2adc(prcval);
			}
		}
		if(liftdefine.Statuse == LIFT_ST_ERR)
			return;
	}
	else {
		//Сначало определяем угол
		if(currentanglPrc<prcval)
		{
			liftdefine.SetAngle_prc = prcval;
			liftdefine.CurAngle_prc = prcval;
			liftdefine.SetAngle_adc = prc2adc(prcval);
			liftdefine.Statuse = LIFT_ST_UP;
			liftdefine.TimeUpDown = 0;
			gpio_MotionUp();
			return;
		}
		if(currentanglPrc>prcval)
		{
			liftdefine.SetAngle_prc = prcval;
			liftdefine.SetAngle_adc = prc2adc(prcval);
			liftdefine.CurAngle_prc = prcval;
			liftdefine.Statuse = LIFT_ST_DOWN;
			liftdefine.TimeUpDown = 0;
			gpio_MotionDown();
			return;
		}
	}

}
*/
/*----------------------------------------------------------------------------
* 	Запустить калибровку
* 	Опускаем дорожку, ждем прекращения изменений АЦП, запоминаем нижний уровень
* 	Поднимаем дорожку,ждем прекращения изменений АЦП, запоминаем верхний уровень
* 	уменьшаем диапазон на несколько процентов, и перезаписываем верхний и нижний уровень
----------------------------------------------------------------------------*/
void KalibrLift()
{
}
/*----------------------------------------------------------------------------
* 	Процесс контролля двигателя подъема
*		Данная ф-ция должна вызываться через определенные промежутки времени
*	и следит за изменением АЦП
*	Если за время не произошло изменений ацп, выдать ошибку
----------------------------------------------------------------------------*/
/*void ProcLift()
{
	//Если ни куда не движется, выходим
	if(liftdefine.Statuse == LIFT_ST_READY)
	{
		gpio_MotionStop();
		return;
	}
	if(liftdefine.Statuse == LIFT_ST_ERR)
	{
		gpio_MotionStop();
		return;
	}
	liftdefine.TimeUpDown++;
	liftdefine.CurAngle_adc = GetAdcLift();
	if(liftdefine.CurAngle_adc < (liftdefine.MinValueAdc - 50))
	{
		if(liftdefine.Statuse == LIFT_ST_DOWN)
		{
			liftdefine.Statuse = LIFT_ST_READY;
			liftdefine.CurAngle_prc = liftdefine.MinValuePrc;
			gpio_MotionStop();
			return;
		}

	}
	if(liftdefine.CurAngle_adc > (liftdefine.MaxValueAdc+50))
	{
		if(liftdefine.Statuse == LIFT_ST_UP)
		{
			liftdefine.Statuse = LIFT_ST_READY;
			liftdefine.CurAngle_prc = liftdefine.MaxValuePrc;
			gpio_MotionStop();
			return;
		}

	}
	liftdefine.SetAngle_adc = prc2adc(liftdefine.SetAngle_prc);
	int raznica;
	//Если движемся вверх
	if(liftdefine.Statuse == LIFT_ST_UP)
	{
		
		//__MODULE__
//		raznica = modf(liftdefine.CurAngle_adc,liftdefine.PrevAngle_adc);
		//Останавливаем, как только текущий угол больше заданного
		if(liftdefine.SetAngle_adc <= liftdefine.CurAngle_adc)
		{
			liftdefine.Statuse = LIFT_ST_READY;
			liftdefine.CurAngle_prc = liftdefine.SetAngle_prc;
			gpio_MotionStop();
			//liftdefine.Statuse = LIFT_ST_READY;
			//liftdefine.CurAngle_prc = liftdefine.SetAngle_prc;//adc2prc(liftdefine.SetAngle_adc);//adc2prc(liftdefine.CurAngle_adc);
		}
	}
	//Если движемся вниз
	if(liftdefine.Statuse == LIFT_ST_DOWN)
	{
//		raznica = modf(liftdefine.CurAngle_adc,liftdefine.PrevAngle_adc);
		//Останавливаем, как только текущий угол больше заданного
		if(liftdefine.SetAngle_adc >= liftdefine.CurAngle_adc)
		{
			liftdefine.Statuse = LIFT_ST_READY;
			liftdefine.CurAngle_prc = liftdefine.SetAngle_prc;//adc2prc(liftdefine.SetAngle_adc);//adc2prc(liftdefine.CurAngle_adc);
			gpio_MotionStop();
		}
	}
	//Если идет калибровка
	if(liftdefine.Statuse & LIFT_ST_KALIBR)
	{

	}
}
*/



void ProcLift(LIFT_TypeDef* liftdef)
{
	if(liftdef->State == LIFT_ERRORE_MOTION)
		return;
	
	if(liftdef->State != LIFT_STATE_MOTION_STOP)
	{
		//Есть движение
		if(!AdcIsChange(liftdef))
		{
			//Нет изменений
			//Останавливаем и выдаем ошибку
			gpio_MotionStop();
			//liftdef->Errore = LIFT_ERRORE_MOTION;
			return;
		}
		else
		{
			//изменения есть движемся дальше
			if(liftdef->State == LIFT_STATE_MOTION_UP)
			{
				//	Движемся вверх 
				//	Проверяем привысили значение или нет
				if(liftdef->CurrentAdc > liftdef->SetAngleAdc)
				{
					//	Превысили значение, останавливаем
					gpio_MotionStop();
					liftdef->State = LIFT_STATE_MOTION_STOP;
				}
			}
			else
			{
				//	Движемся вниз
				//	Проверяем ниже уровня,если да то останавливаем
				if(liftdef->CurrentAdc < liftdef->SetAngleAdc)
				{
					//	Значение меньше, останавливаем
					gpio_MotionStop();
					liftdef->State = LIFT_STATE_MOTION_STOP;
				}
			}
		}
	}
	else
	{
		//Нет движения
		if(liftdef->Errore == LIFT_ERRORE_NONE)
		{
			// 	Если ошибок нет
			//	проверяем нужно изменять положение или нет
			
			SetAngle(liftdef);
			
		}
		else
		{
			gpio_MotionStop();
			//	Если есть ошибки
		}
	}
}

/*
	Проверка изменений АЦП
	Попали сюда только если есть движение, увеличиваем счетчик
*/
int AdcIsChange(LIFT_TypeDef* liftdef)
{
	if(liftdef->CountTimeMotion > liftdef->TimeChgAdc)
	{
		//Прошло время, за которое должны были быть изменения
		if(module(liftdef->CurrentAdc,liftdef->PrevAngleAdc) > liftdef->DeltaAdc)
		{
			liftdef->CountTimeMotion = 0;
			liftdef->PrevAngleAdc = liftdef->CurrentAdc;
			return 1;
		}
		else
		{
			//	Иначе ошибка, что делать дальше, пксть думает другая программа
			return 0;
		}
	}
	else
	{
		liftdef->CountTimeMotion +=1;
		return 1;
	}
}
	



int count_data = 0;
/*----------------------------------------------------------------------------
* 	Заполняем данные АЦП
----------------------------------------------------------------------------*/
void FillDataAdc(int data_adc)
{

	LiftADCbuff[count_data] = data_adc;
	count_data++;
	if(count_data>=LIFT_ADC_BUFF_SIZE)
	{
		count_data = 0;
		if(liftdefine.Statuse == LIFT_ST_WAIT_ADC)
			liftdefine.Statuse = LIFT_ST_READY;
	}
	
}
/*----------------------------------------------------------------------------
* 	Выдать значение АЦП
----------------------------------------------------------------------------*/
int GetAdcLift()
{
	while(liftdefine.Statuse == LIFT_ST_WAIT_ADC);
	long int buff = 0;
	for(int i = 0; i< LIFT_ADC_BUFF_SIZE; i++)
	{
		buff += LiftADCbuff[i];
	}
	return buff/LIFT_ADC_BUFF_SIZE;
}
/*----------------------------------------------------------------------------
* 	Проверка изменений АЦП
* 	Необходимо подумать где лучше вызывать
----------------------------------------------------------------------------*/
void ChangIspADC()
{
	int curadc = GetAdcLift();
	int raznica = module(curadc,liftdefine.PrevAngle_adc);
	if(raznica<LIFT_ADC_MIN_IZMINENIYA)
	{
		liftdefine.Statuse = LIFT_ST_ERR;
		gpio_MotionStop();
	}
}
int module(int z1, int z2)
{
	if(z1<z2)
		return z2-z1;
	else
		return z1-z2;
}
