
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"

/* USER CODE BEGIN Includes */

//#include "cfm_bhfitness.h"

//	17.05.2018	Проверить отсчеты времени геркона
	
//
//#include "keyboard.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
int temp_key;
int time_key_count = 0;
char prev_key = 0;
char KeyCodeRead = 0;

DOROGKA_TypeDef DrgDef;
//CFM_BH_TypeDef cfmtypedef;
//MOT_UP_DUWN_Define UpDownStruct;
CFM_BH_TypeDef	CfmTypeDef;
LIFT_TypeDef	LiftTypeDef;

unsigned int flagDisp1;
unsigned int flagDisp2;
unsigned int flagDisp3;
unsigned int flagDisp4;



#define GERKONE_BUFF_SIZE  4
int velbuff[GERKONE_BUFF_SIZE] = {0,0,0,0};	//Буффер АЦП
//int velbuff[4]={0,0,0,0};

int TimeGerkon;

__IO ITStatus ProcCFMReady 	= RESET;
__IO ITStatus LiftCFMReady 	= RESET;
__IO ITStatus ADCReady 		= RESET;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM3_Init(void);
static void MX_NVIC_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
int count_ms = 0;
int count_one_sec = 0;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	SystemInit();

	//	нициализация переменных дорожки 
	//cfmtypedef.CFMaddr = 81;
	LiftTypeDef.CurrentAdc = 0;
	LiftTypeDef.MaxValueAdc = 3800;
	//DrgDef.LiftTypeDef.MinValueAdc = 150;
	LiftTypeDef.MinValueAdc = 1400;
	LiftTypeDef.MaxValuePrc = 15;
	LiftTypeDef.MinValuePrc = 0;
	
	LiftTypeDef.Sadcprc = (LiftTypeDef.MaxValueAdc - LiftTypeDef.MinValueAdc)/(LiftTypeDef.MaxValuePrc - LiftTypeDef.MinValuePrc);
	LiftTypeDef.CountTimeMotion = 0;
	LiftTypeDef.DeltaAdc = 30;
	LiftTypeDef.Errore = LIFT_ERRORE_NONE;
	LiftTypeDef.PrevAngleAdc = 0;
	LiftTypeDef.SetAngleAdc = LiftTypeDef.MinValueAdc;
	LiftTypeDef.SetAnglePrc = LiftTypeDef.MinValuePrc;
	LiftTypeDef.State = LIFT_STATE_MOTION_STOP;
	LiftTypeDef.TimeChgAdc = 100;

	
	CfmTypeDef.CFMfreq = 0;
	CfmTypeDef.cfmStateIfTypeDef = CFM_STATE_IE_READY;
	CfmTypeDef.cfmStateRegistrTypeDef = CFM_STATE_REG_NOMOT;
	CfmTypeDef.CountTimeReq = 0;
	CfmTypeDef.cfmStateTypeDef = CFM_BH_STATE_STOP;
	
	DrgDef.Status = DRG_STATE_STOP;
	//CfmTypeDef.Angle = 0;
	
	/*flagDisp1 = VIEW_TIME;
	flagDisp2 = VIEW_ANGLE;
	//flagDisp2 = VIEW_SET_VEL;
	flagDisp3 = VIEW_SET_VEL;
	*/
	flagDisp1 = VIEW_TIME;
	flagDisp2 = VIEW_ANGLE;
	//flagDisp2 = VIEW_SET_VEL;
	flagDisp3 = VIEW_SET_VEL;
	
	
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_ADC1_Init();
  MX_SPI2_Init();
  MX_TIM3_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */


	
	
	
	Init_7219();
	Send_7219(1,0x01);//1
	Send_7219(2,0x02);//2
	Send_7219(3,0x03);//3
	Send_7219(4,0x04);//4
	Send_7219(5,0x05);//5
	Send_7219(6,0x06);//6
	Send_7219(7,0x07);//7
	Send_7219(8,0x08);//8
//	HAL_Delay(2000);
	//Clear_7219();
	//Number_7219(-4356);//попытаемся вывести отрицательное значение
//	HAL_Delay(2000);
	Clear_7219();
	MAX7219_WriteDig(1,1);
	MAX7219_WriteDig(2,2);
	MAX7219_WriteDig(3,3);
	MAX7219_WriteDig(4,4);
	MAX7219_WriteDig(5,5);
	MAX7219_WriteDig(6,6);
	MAX7219_WriteDig(7,7);
	MAX7219_WriteDig(8,8);
	
	MAX7219_WriteDig(9,9);
	MAX7219_WriteDig(10,10);
	MAX7219_WriteDig(11,11);
	MAX7219_WriteDig(12,12);
	MAX7219_WriteDig(13,13);
	MAX7219_WriteDig(14,14);
	MAX7219_WriteDig(15,15);
	
	
	//gpio_StartCFM();
	
	HAL_TIM_IC_Start_IT(&htim2,TIM_CHANNEL_2);
	HAL_TIM_Base_Start_IT(&htim2);
	HAL_TIM_Base_Start_IT(&htim3);
	HAL_ADC_Start_IT(&hadc1);
	
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  int i = 0;
 // LiftTypeDef.SetAngleAdc = prc2adc(5,&LiftTypeDef);
	//gpio_StopCFM();
  while (1)
  {
	  //	Проверяем нажатие клавиши
	  if(KeyCodeRead != 0)
	  {
		  //MAX7219_WriteDec(1,KeyCodeRead);
		  KeyExCmd(KeyCodeRead);
		  KeyCodeRead = 0;
	  }
	  if(count_ms>100)
	  {
		  //Здесь сделаем обновление дисплея
		  //Также здесь уменьшение скорости при остановке
		  if(DrgDef.Status == DRG_STATE_STOPED)
		  {
			  CfmTypeDef.SetCFMvel-=1;
			  if(CfmTypeDef.SetCFMvel <= 0)
			  {
				  StopMotor(&(CfmTypeDef));
				  DrgDef.Status = DRG_STATE_STOP;
				  DrgDef.Time=0;
			  }
		  }
		  //CfmTypeDef.SetCFMvel = 20;
			//MAX7219_WriteDec(9,cfmtypedef.SetCFMvel);
			//MAX7219_WriteDec(1,cfmtypedef.CFMvel);
		  
			ViewDisp(DISP_1, flagDisp1,1);
			ViewDisp(DISP_2, flagDisp2,1);
			//ViewDisp(DISP_3, flagDisp3,adc_sred);
			ViewDisp(DISP_3, flagDisp3,1);
			//MAX7219_WriteDec(DISP_3,DrgDef.LiftTypeDef.CurrentAdc);
		  
			//ViewDisp(DISP_3, flagDisp3,timer_capture);
			ViewDisp(DISP_4, flagDisp4,1);
		  
		  count_ms = 0;
		  i++;
		  if(i > 9999)
			  i = 0;
	  }
	  //Отсчеты времени по 1-й секунде
	  if(count_one_sec >1000)
	  {
		  if(DrgDef.Status == DRG_STATE_MOTION)
		  {
			  DrgDef.Time+=1;
		  }
		  if(DrgDef.Status == DRG_STATE_STARTED)
		  {
			  DrgDef.Time-=1;
			  if(DrgDef.Time == 0 )
			  {
				  DrgDef.Status = DRG_STATE_MOTION;
				  StartMotor(&(CfmTypeDef));
				  CfmTypeDef.SetCFMvel = 10;
			  }
			  
		  }
		  
		  //Счетчик секунд
		  
		  count_one_sec = 0;
	  }
		if(ProcCFMReady == SET)
		{
			ProcCFM(&(CfmTypeDef));
			ProcCFMReady = RESET;
		}
		if(LiftCFMReady == SET)
		{
			ProcLift(&LiftTypeDef);
			LiftCFMReady = RESET;
		}
		if(ADCReady == SET)
		{
			FillDataAdc(HAL_ADC_GetValue(&hadc1));
			//SredADC = GetAdcLift();
			//console_ans.lift = GetAdcLift();
			LiftTypeDef.CurrentAdc = GetAdcLift();
			//console_ans.lift = LiftTypeDef.CurrentAdc;
			//MAX7219_WriteDec(DISP_3,LiftTypeDef.CurrentAdc);
			ADCReady = RESET;  
		}
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV2;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* TIM2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM2_IRQn);
  /* TIM3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM3_IRQn);
}

/* ADC1 init function */
static void MX_ADC1_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Common config 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* SPI2 init function */
static void MX_SPI2_Init(void)
{

  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_IC_InitTypeDef sConfigIC;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 7999;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 3000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 7999;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 100;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPD_DEC_GPIO_Port, SPD_DEC_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, SPD_INC_Pin|STOP_Pin|LIFT_UP_Pin|LIFT_DOWN_Pin 
                          |SPI2_CS_Pin|R1_Pin|R2_Pin|R3_Pin 
                          |R4_Pin|R5_Pin|R6_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : SPD_DEC_Pin */
  GPIO_InitStruct.Pin = SPD_DEC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPD_DEC_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SPD_INC_Pin STOP_Pin LIFT_UP_Pin LIFT_DOWN_Pin 
                           SPI2_CS_Pin R1_Pin R2_Pin R3_Pin 
                           R4_Pin R5_Pin R6_Pin */
  GPIO_InitStruct.Pin = SPD_INC_Pin|STOP_Pin|LIFT_UP_Pin|LIFT_DOWN_Pin 
                          |SPI2_CS_Pin|R1_Pin|R2_Pin|R3_Pin 
                          |R4_Pin|R5_Pin|R6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : C1_Pin C2_Pin C3_Pin C4_Pin */
  GPIO_InitStruct.Pin = C1_Pin|C2_Pin|C3_Pin|C4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */



/**
  * @brief  Period elapsed callback in non blocking mode 
  * @param  htim TIM handle
  * @retval None
  */
int k = 0;

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	int IC_Rising_Val;
	
	if(htim->Instance == TIM2)
	{
		if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) // =RISING= EDGE DETECTED
		{
			// Get =RISING= EDGE Capture value 
			//	Попадая сюда, можем вычислить скорость дорожки
			IC_Rising_Val = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
			if(IC_Rising_Val>20)
			{
				CfmTypeDef.CFMfreq = IC_Rising_Val;
				FillDataGerkone(IC_Rising_Val);
			
				CfmTypeDef.CFMvel = timeGerconeToVel(IC_Rising_Val);
			}
			
			//TimeGerkon = IC_Rising_Val;
			
			// Reset Counter After Input Capture Interrupt Occurs	
			
			//HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET);
			//HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);
			//MAX7219_WriteDec(5,IC_Rising_Val);
			//k = __HAL_TIM_GET_COUNTER(htim);
			//MAX7219_WriteDec(1,0);
			__HAL_TIM_SET_COUNTER(&htim2,0x00);
			k = 0;
		}
		//ProcLift(&LiftTypeDef);
		//k +=1;
		
	}
	
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM2)
	{	
		//Попадаем сюда когда прошло больше 3 секунд
		//	Если дорожка была запущена - то ошибка
		//ProcLift(&LiftTypeDef);
		//k = __HAL_TIM_GET_COUNTER(htim);
		//k++;
		//MAX7219_WriteDec(1,k);
		if(CfmTypeDef.cfmStateRegistrTypeDef == CFM_STATE_REG_NOMOT)
		{
			CfmTypeDef.CFMvel = 0;
		}
		else
		{
			//иначе ошибка 
		}
	}
	//	Сюда попадаем каждую секунду
	if(htim->Instance == TIM3)
	{
		k++;
		//MAX7219_WriteDec(1,k);
		
		ProcCFMReady = SET;
		LiftCFMReady = SET;
		//Организуем контроль скорости и времени
		//Сюда будем поподать каждые 100 мс
	}
	
}
/*-------------------------------------------------------------------
	Пришли данные АЦП
--------------------------------------------------------------------*/
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc1)
{
	ADCReady = SET;
}

/*-------------------------------------------------------------------
	Запуск программы
--------------------------------------------------------------------*/
void StartProgram(int npr)
{
	velbuff[0]=0;
	velbuff[1]=0;
	velbuff[2]=0;
	velbuff[3]=0;
	if(DrgDef.StatusStopKey == 0)
	{
	//FreqConvControll(FREQ_CONTROLL_CMD_RUN,0);
		DrgDef.CurrValCount = 0;
		DrgDef.CurrentVel = 0;
		DrgDef.SettVel = MIN_SPEED;
//	DrgDef.Status = STATUS_IS_RUN;
		DrgDef.Status = DRG_STATE_STARTED;
		DrgDef.Time = 1;

		DrgDef.Distance = 0;
		DrgDef.CountDist = 0;
		
	//	flagDisp1 = VIEW_DISTANCE;
	//	flagDisp2 = VIEW_SET_VEL;
		//flagDisp3 = VIEW_ANGLE;
	//	flagDisp1 = VIEW_TIME; //flagDisp4 = VIEW_TIME;
	}
	//flagDisp1 = VIEW_REAL_VEL;
	//flagDisp2 = VIEW_SET_VEL;

}

/*-------------------------------------------------------------------
	ф-я выполнения команд от кнопок
--------------------------------------------------------------------*/

void KeyExCmd(char keykod)
{
	char key = keykod;

	if(key == KEY_KOD_START)
	{
		//Если дорожка не запущена - запускаем
		//gpio_StartCFM();
		if(DrgDef.Status == DRG_STATE_STOP)
		{
			StartProgram(1);
			//StartMotor(&(CfmTypeDef));
			//CfmTypeDef.SetCFMvel = 10;
		}
		
	}
	if(key == KEY_KOD_STOP)
	{
		if(DrgDef.Status == DRG_STATE_MOTION)
		{
		//gpio_StopCFM();
			DrgDef.Status = DRG_STATE_STOPED;
		}
		//StopMotor(&(CfmTypeDef));
	}
	if(key == KEY_KOD_SPEEDUP)
	{
		if(DrgDef.Status == DRG_STATE_MOTION)
		{
			CfmTypeDef.SetCFMvel+=1;
		}
		
		/*
		gpio_SpdIncCFMHigth();
		HAL_Delay(100);
		gpio_SpdIncCFMLow();
		HAL_Delay(100);
		*/
	}
	if(key == KEY_KOD_SPEEDDOWN)
	{
		if(DrgDef.Status == DRG_STATE_MOTION)
		{
			CfmTypeDef.SetCFMvel-=1;
		}
		
		/*
		gpio_SpdDecCFMHigth();
		HAL_Delay(100);
		gpio_SpdDecCFMLow();
		HAL_Delay(100);
		*/
	}
	if(key == KEY_KOD_KONSUP)
	{
		//gpio_MotionUp();
		//gpio_MotionDown();
		if(LiftTypeDef.SetAnglePrc < LiftTypeDef.MaxValuePrc)
		{
			LiftTypeDef.SetAnglePrc+=1;
			LiftTypeDef.SetAngleAdc = prc2adc(LiftTypeDef.SetAnglePrc,&LiftTypeDef);
		}
		
	}
	if(key == KEY_KOD_KONSDOWN)
	{
		if(LiftTypeDef.SetAnglePrc > LiftTypeDef.MinValuePrc)
		{
			LiftTypeDef.SetAnglePrc-=1;
			LiftTypeDef.SetAngleAdc = prc2adc(LiftTypeDef.SetAnglePrc,&LiftTypeDef);
		}
		//gpio_MotionStop();
	}
	
	
	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//char key = KeypadRead();
/*	if(key!=0)
	{
		temp_key = key;
		if(prev_key==key)
		{
			if((key == KEY_KOD_SPEEDUP)||(key == KEY_KOD_SPEEDDOWN))
			{
				if(time_key_count<200)
					return;
				time_key_count = 0;
			}
			else {
				if(time_key_count<400)
					return;
				time_key_count = 0;
			}

		}
		else {
			time_key_count = 0;
			prev_key = key;
		}
	}
	else {
		//prev_key = 0;
	}
	if(key == KEY_KOD_START)
	{
		if((DrgDef.Status == STATUS_IS_STOPPED)|(DrgDef.Status == STATUS_ERR_STOP_KEY))
			StartProgram(1);
	//	FreqConvControll(FREQ_CONTROLL_CMD_RUN,0);
	}
	if(key == KEY_KOD_STOP)
	{
	//	DrgDef.SettVel = 2.0;
	//	MotionUpDown(0);
	//	AddFuncTime(&StopProgram,DrgDef.CurrentVel*2001);
		DrgDef.Status = STATUS_IS_DEC_STOPPED;


	//	DrgDef.CurrValCount--;
	//	DrgDef.Status = 0;
	//	FreqConvControll(FREQ_CONTROLL_CMD_STOP,0);
		//FreqConvControll(FREQ_CONTROLL_CMD_STOP,0);
	}
	if(key == KEY_KOD_SPEEDUP)
	{
		if(DrgDef.SettVel < MAX_SPEED)
			DrgDef.SettVel +=0.1;
		FreqConvControll(FREQ_CONTROLL_CMD_VEL_INC,10);
	}
	if(key == KEY_KOD_SPEEDDOWN)
	{
		if(DrgDef.SettVel > MIN_SPEED)
			DrgDef.SettVel -=0.1;
		FreqConvControll(FREQ_CONTROLL_CMD_VEL_DEC,10);
	}
	if(key == KEY_KOD_KONSUP)
	{
		if(UpDownStruct.SetAngle < 16)
		{
			UpDownStruct.SetAngle++;
			MotionUpDown(UpDownStruct.SetAngle);
		}
	}
	if(key == KEY_KOD_KONSDOWN)
	{
		if(UpDownStruct.SetAngle > 0)
		{
			UpDownStruct.SetAngle--;
			MotionUpDown(UpDownStruct.SetAngle);
		}
	}
	*/
	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	/*
	char key = KeypadRead();
	if(key!=0)
	{
		temp_key = key;
		if(prev_key==key)
		{
			if((key == KEY_KOD_SPEEDUP)||(key == KEY_KOD_SPEEDDOWN))
			{
				if(time_key_count<200)
					return;
				time_key_count = 0;
			}
			else {
				if(time_key_count<400)
					return;
				time_key_count = 0;
			}

		}
		else {
			time_key_count = 0;
			prev_key = key;
		}
	}
	else {
		//prev_key = 0;
	}
	if(key == KEY_KOD_START)
	{
		if((DrgDef.Status == STATUS_IS_STOPPED)|(DrgDef.Status == STATUS_ERR_STOP_KEY))
			StartProgram(1);
	//	FreqConvControll(FREQ_CONTROLL_CMD_RUN,0);
	}
	if(key == KEY_KOD_STOP)
	{
	//	DrgDef.SettVel = 2.0;
	//	MotionUpDown(0);
	//	AddFuncTime(&StopProgram,DrgDef.CurrentVel*2001);
		DrgDef.Status = STATUS_IS_DEC_STOPPED;


	//	DrgDef.CurrValCount--;
	//	DrgDef.Status = 0;
	//	FreqConvControll(FREQ_CONTROLL_CMD_STOP,0);
		//FreqConvControll(FREQ_CONTROLL_CMD_STOP,0);
	}
	if(key == KEY_KOD_SPEEDUP)
	{
		if(DrgDef.SettVel < MAX_SPEED)
			DrgDef.SettVel +=0.1;
		FreqConvControll(FREQ_CONTROLL_CMD_VEL_INC,10);
	}
	if(key == KEY_KOD_SPEEDDOWN)
	{
		if(DrgDef.SettVel > MIN_SPEED)
			DrgDef.SettVel -=0.1;
		FreqConvControll(FREQ_CONTROLL_CMD_VEL_DEC,10);
	}
	if(key == KEY_KOD_KONSUP)
	{
		if(UpDownStruct.SetAngle < 16)
		{
			UpDownStruct.SetAngle++;
			MotionUpDown(UpDownStruct.SetAngle);
		}
		//	MotionUpDown(UpDownStruct.SetAngle=+1);
	}
	if(key == KEY_KOD_KONSDOWN)
	{
		if(UpDownStruct.SetAngle > 0)
		{
			UpDownStruct.SetAngle--;
			MotionUpDown(UpDownStruct.SetAngle);
		}
	}
	*/
}



/*-------------------------------------------------------------------
	Вывод информации на дисплей
--------------------------------------------------------------------*/
void ViewDisp(unsigned int ndisp, unsigned int flag,uint32_t data)
{
	float buff;
	switch(flag){
		case VIEW_TIME:
			MAX7219_WriteTime(ndisp,SecToTime(DrgDef.Time));
			break;
		case VIEW_DISTANCE:
			MAX7219_WriteFloat(ndisp,DrgDef.Distance);
			break;
		case VIEW_REAL_VEL:
			buff = (float)(CfmTypeDef.CFMvel)/10;
			MAX7219_WriteFloat(ndisp,buff);
			break;
		case VIEW_SET_VEL:
			buff = (float)(CfmTypeDef.SetCFMvel)/10;
			MAX7219_WriteFloat(ndisp,buff);
			break;
		case VIEW_DIST_CNT:
			MAX7219_WriteDec(ndisp,DrgDef.CountDist);
			break;
		case VIEW_ANGLE:
			
			MAX7219_WriteDec(ndisp,LiftTypeDef.SetAnglePrc);
			//MAX7219_WriteDec(ndisp,adc2prc(LiftTypeDef.CurrentAdc,&LiftTypeDef));
			//MAX7219_WriteDec(ndisp,UpDownStruct.SetAngle);
			break;
		case VIEW_KEY_KOD:
			MAX7219_WriteDec(ndisp,data);
			break;
		case VIEW_ERR:
			MAX7219_WriteErr(ndisp,DrgDef.Status);
			break;
		case VIEW_ERR_UPDOWN:
			//MAX7219_WriteErr(ndisp,UpDownStruct.Statuse);
			break;
		}
}

/*-------------------------------------------------------------------
 * Перевод секунд в минуты и секунды возвращает число mmss
--------------------------------------------------------------------*/
int SecToTime(int sec)
{
	int buff = 9999;
	if(sec < 6000)
	{
		buff = sec/60;
		buff = buff*100;
		buff +=sec%60;
	}
	return buff;
}

int count_data_gerkone = 0;
/*----------------------------------------------------------------------------
* 	Заполняем данные АЦП
----------------------------------------------------------------------------*/
void FillDataGerkone(int data_gerkone)
{

	velbuff[count_data_gerkone] = data_gerkone;
	count_data_gerkone++;
	if(count_data_gerkone>=GERKONE_BUFF_SIZE)
	{
		count_data_gerkone = 0;
	}
	
}

/*----------------------------------------------------------------------------
* 	Выдать значение АЦП
----------------------------------------------------------------------------*/
int GetGerkoneTime()
{

	long int buff = 0;
	for(int i = 0; i< GERKONE_BUFF_SIZE; i++)
	{
		buff += velbuff[i];
	}
	return buff/GERKONE_BUFF_SIZE;
}







//	Оправшиваем кнопки, необходимо сделать дребезг контактов
void HAL_SYSTICK_Callback()
{
	count_ms++;
	count_one_sec++;
	
	KeypadScan();
	int key = KeypadRead();
	
	if(key != 0)
	{
		//MAX7219_WriteDec(1,key);
		KeyCodeRead = key;
	}
	
}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
