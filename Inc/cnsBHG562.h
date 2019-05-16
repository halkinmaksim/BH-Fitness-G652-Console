#ifndef CNSBHG562_H
#define CNSBHG562_H
/*
 * 		Описание команд для работы с консолью BH Fitness G562
 *		Пробуем сделать по образу Precore
 *		Структура содержит 12 команд
 *
 *		
 *
 *
 *
 *
 *
 *
 *		Пример посылки от консоли 20 байт в
 *		От консоли идут запросы в таком виде
 * [amps] [err]    [belt]  [lift]  [lfts] [lftg] [part:6] [ver] [type] [loop:5550] [inc:0] [kph:A]
 *		Ответ
 * [amps:0][err:2A][belt:2][lift:2][lfts:0][lftg:0][part:0006][ver:6B][type:20][loop:4C57] [inc:0][kph:2]
 *
 *	[kph:0] задает скорость С = 12 = 1,2 F = 15 = 1,5
 *	[inc:0]	задает угол подема
 *	[amps:0] 1 = 0,1А
 [lift:30] = ADC 48
 [belt:7] = скорее всего PWRB
 *	 Данные в формате ASCII
 *
 *
 *
 *	И так, имеем 12 запросов
 *
 *  Ошибки
 *  42 - Lift  

 *
 * */
 
 typedef struct {
	uint32_t	amps;	//	Ток частотника, здесь не используется
	uint32_t	err;	//	Ошибки дорожки, битовое поле
	uint32_t	belt;	//	Пока не знаю для чего
	uint32_t	lift;	//	Значение АЦП
	uint32_t	lfts;
	uint32_t	lftg;
	uint32_t	part;
	uint32_t	ver;
	uint32_t	type;
	uint32_t	loop;
	uint32_t	inc;	//	Значение угла подъема(25 = 2,5%)
	 uint32_t	kph;	//	Значение скорости х10 км/ч(100 = 10км/ч)
 }ConsoleTypeDef;	
 
 
 
 #endif /* CNSBHG562_H */
