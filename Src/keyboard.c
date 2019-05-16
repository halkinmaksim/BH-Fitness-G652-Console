
#include "keyboard.h"

/*
 * a8-a11 входы COLUM
 * b3-b9 выходы	ROW
 * */

const char keykodes[6][6]={
		{1,2,3,4,5,6},
		{7,8,9,10,11,12},
		{13,14,15,16,17,18},
		{19,20,21,22,23,24},
		{25,26,27,28,29,30},
		{31,32,33,34,35,36}
};
volatile char ScanKey; //Переменная для сохранения кода нажатой кнопки

//==================================================================
//                   Инициализация клавиатуры
//==================================================================
void KeypadInit(void)
{
    char x;
    int temp=0;
	/*
    GPIO_InitTypeDef  GPIO_InitStruct;


	RCC_APB2PeriphClockCmd(RCC_ROW, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_COLUMN, ENABLE);

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE);
*/
//	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE);

    for(x=0; x<NUM_ROW; x++) //Формирование на выход
    {
    	temp |= (1<<(x+FIRST_BIT_ROW));
    }
	/*
    GPIO_InitStruct.GPIO_Pin = temp;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;

    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;//GPIO_Mode_Out_OD;

    GPIO_Init(KEYPAD_ROW, &GPIO_InitStruct);
    GPIO_ResetBits(GPIOB,GPIO_Pin_3|GPIO_Pin_4);
    GPIO_SetBits(GPIOB,GPIO_Pin_3|GPIO_Pin_4);


    for(temp=0, x=0; x<NUM_COLUMN; x++) //Формирование на вход
    {
        temp |= (1<<(x+FIRST_BIT_COLUMN));  //PORTn (подключение подтягивающих резисторов)
    }
    GPIO_InitStruct.GPIO_Pin = temp;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(KEYPAD_COLUMN, &GPIO_InitStruct);
*/

}

//==================================================================
//          	Проверка нажатия хотя бы одной кнопки
//==================================================================
char KeypadCheck(void)
{
    char x;
    int temp=0;
    for(x=0; x<NUM_ROW; x++)
    {
        temp |= (1<<(x+FIRST_BIT_ROW));
    }
  //  KEYPAD_ROW->ODR &=~ temp; //Установил на всех выходах клавиатуры (на выходах строк) '0'
    KEYPAD_ROW->BRR |= temp; //Установил на всех выходах клавиатуры (на выходах строк) '0'

    for(x=0; x<NUM_COLUMN; x++) //Проверяю нажатие, если да - возвращаю '1'
    {
        if(~(KEYPAD_COLUMN->IDR) & (1<<(x+FIRST_BIT_COLUMN)))   return 1;
    }
    return 0;
}
//==================================================================
//             	    Получение кода нажатой кнопки
//==================================================================
char KeypadRead(void)
{
    char key=ScanKey;
    ScanKey=0;
    return key;
}

//==================================================================
//Функция сканирования клавиатуры (запускается в прерывании таймера)
//==================================================================
void KeypadScan(void)
{
    char row, column, x;
    int temp,j;
    char key=0;
    static char lastkey=0;
    static char counter; //Переменная-счётчик для организации антидребезговой защиты

    if (KeypadCheck()==0)   { counter=0; return;} //Если ни одна кнопка не нажата - досвидос!

    for(row=0; row<NUM_ROW; row++) //Перебираю строки клавиатуры
    {
        for(x=0; x<NUM_ROW; x++)
        {
            //KEYPAD_ROW->ODR |= (1<<(x+FIRST_BIT_ROW)); //Установил на выходах строк '1'
        	KEYPAD_ROW->BSRR |= (1<<(x+FIRST_BIT_ROW)); //Установил на выходах строк '1'
            for(j=0; j<250; j++){j++;}
        }
      //  KEYPAD_ROW->ODR &=~ (1<<(row+FIRST_BIT_ROW)); //Устанавливаю на очередной строке клавиатуры '0'
        KEYPAD_ROW->BRR |= (1<<(row+FIRST_BIT_ROW)); //Устанавливаю на очередной строке клавиатуры '0'
        __NOP();
        for(j=0; j<250; j++){j++;}
        temp = KEYPAD_COLUMN->IDR; //Сохраняю значения входов (столбцов)

        for(column=0; column<NUM_COLUMN; column++) //Опрашиваю столбцы клавиатуры
        {
            if((temp & (1<<(column+FIRST_BIT_COLUMN)))==0)
            {
                key = keykodes[row][column]; //Получаю код кнопки
            }
        }
    }

    if(key!=lastkey)    { lastkey=key; counter=0; }
    else
    {
        if(counter==MAX_VALUE)
        {
           // counter=MAX_VALUE+10;
        	counter=0;
            ScanKey = key;
            return;
        }
        if(counter<MAX_VALUE)   counter++;
    }
}
