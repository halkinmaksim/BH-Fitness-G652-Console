#ifndef KEYBOARD_H
#define KEYBOARD_H


//#include "stm32f10x_rcc.h"
//#include "stm32f10x_gpio.h"
#include  "stm32f1xx_hal.h"

#define KEYBOARD_3x4 //Если закомментировано, тогда применяется клавиатура 4x4
#define MAX_VALUE   4 //Антидребезговая выдержка времени

#define CORTEX_M	3 //0 для Cortex-M, 3 для Cortex-M3 и 4 для Cortex-M4
#define GPIOSPEED	GPIO_Speed_50MHz

//Подключение столбцов (вход микроконтроллера)
#define KEYPAD_COLUMN       GPIOA  //Порт, подключённый к столбцам (ко входу)
#if CORTEX_M==0
#define RCC_COLUMN	RCC_AHBPeriph_GPIOA // RCC_AHBPeriph_GPIOB, RCC_AHBPeriph_GPIOC...
#elif CORTEX_M==3
#define RCC_COLUMN	RCC_APB2Periph_GPIOA //RCC_APB2Periph_GPIOB, RCC_APB2Periph_GPIOC... - включение тактирования
#elif CORTEX_M==4
#define RCC_COLUMN	RCC_AHB1Periph_GPIOA  // RCC_AHB1Periph_GPIOB, RCC_AHB1Periph_GPIOC...
#endif
#define NUM_COLUMN          4     //Количество столбцов клавиатуры
#define FIRST_BIT_COLUMN    8     //С какого бита подключены столбцы

//Подключение строк (выход микроконтроллера)
#define KEYPAD_ROW          GPIOB //Порт, подключённый к строкам (к выходу)
#if CORTEX_M==0
#define RCC_ROW	RCC_AHBPeriph_GPIOB // RCC_AHBPeriph_GPIOB, RCC_AHBPeriph_GPIOC...
#elif CORTEX_M==3
#define RCC_ROW	RCC_APB2Periph_GPIOB //RCC_APB2Periph_GPIOB, RCC_APB2Periph_GPIOC... - включение тактирования
#elif CORTEX_M==4
#define RCC_ROW	RCC_AHB1Periph_GPIOB  // RCC_AHB1Periph_GPIOB, RCC_AHB1Periph_GPIOC...
#endif
#define NUM_ROW             6     //Количество строк клавиатуры
#define FIRST_BIT_ROW       3     //С какого бита подключены строки





extern volatile char ScanKey; //Переменная для сохранения кода нажатой кнопки

//****************************************************************
//			            ПРОТОТИПЫ ФУНКЦИЙ
//****************************************************************




void KeypadInit(void);
char KeypadCheck(void);
char KeypadRead(void);
void KeypadScan(void);


#endif /* KEYBOARD_H */
