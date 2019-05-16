#include "low_gpio.h"

/*				Требуются из внешних файлов			*/
void gpio_StartCFM(void)
{
	HAL_GPIO_WritePin(GPIOB,STOP_Pin,GPIO_PIN_SET);
}
void gpio_StopCFM(void)
{
	HAL_GPIO_WritePin(GPIOB,STOP_Pin,GPIO_PIN_RESET);
}

void gpio_SpdIncCFMHigth(void)
{
	HAL_GPIO_WritePin(GPIOB,SPD_INC_Pin,GPIO_PIN_SET);
}
void gpio_SpdIncCFMLow(void)
{
	HAL_GPIO_WritePin(GPIOB,SPD_INC_Pin,GPIO_PIN_RESET);
}
void gpio_SpdDecCFMHigth(void)
{
	HAL_GPIO_WritePin(SPD_DEC_GPIO_Port,SPD_DEC_Pin,GPIO_PIN_SET);
}
void gpio_SpdDecCFMLow(void)
{
	HAL_GPIO_WritePin(SPD_DEC_GPIO_Port,SPD_DEC_Pin,GPIO_PIN_RESET);
}

void gpio_MotionUp(void)
{
	HAL_GPIO_WritePin(LIFT_DOWN_GPIO_Port,LIFT_DOWN_Pin,GPIO_PIN_SET);
}
void gpio_MotionDown(void)
{
	HAL_GPIO_WritePin(LIFT_UP_GPIO_Port,LIFT_UP_Pin,GPIO_PIN_SET);
	
}
void gpio_MotionStop(void)
{
	HAL_GPIO_WritePin(LIFT_UP_GPIO_Port,LIFT_UP_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LIFT_DOWN_GPIO_Port,LIFT_DOWN_Pin,GPIO_PIN_RESET);
}
