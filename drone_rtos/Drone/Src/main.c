/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"

/* USER CODE BEGIN Includes */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "timers.h"

#include "LSM303AGR/LSM303AGR.h"
#include "LSM6DSL/LSM6DSL.h"
#include "HCSR04/HCSR04.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
DMA_HandleTypeDef hdma_i2c1_rx;
DMA_HandleTypeDef hdma_i2c1_tx;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim5;

UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_usart6_rx;
DMA_HandleTypeDef hdma_usart6_tx;

/* USER CODE BEGIN PV */
/* FreeRTOS */
#define NUM_TASKS 7
#define FREERTOS_IDLE_CALLBACK
TaskHandle_t taskHandles[NUM_TASKS];

/* Variáveis referentes ao LSM6DSL */
extern LSM6DSL_PackType		LSM6DSL_Pack;

HCSR04_PackType HCSR04_Pack;

/* Buffer de Envio da UART */
uint8_t uartStream[48];
uint8_t uartThrottle[8];

/* Contadores Input Capture */
unsigned long ulCount1, ulCount2, ulDataHCSR04;

/* Flag se o Chip Foi Configurado */
extern LSM6DSLStatus_t stateLSM6DSL;

/* TaskHandles */
#define	tMain			0			
#define	tInitLSM6DSL	1
#define	tReadLSM6DSL	2
#define tReadHCSR04		3
#define	tPack			4
#define	tSetPWM			5
#define tInitESC		6

/* TaskNotify */
#define MSG_LSM6DSL_X 	0x01
#define MSG_LSM6DSL_G 	0x02
#define MSG_PACK 		0x04
#define MSG_SETPWM		0x08
#define MSG_HCSR04_H	0x01
#define MSG_HCSR04_L	0x02

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_TIM5_Init(void);
                                    
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                                
/* USER CODE BEGIN PFP */
static void vMain(void *pvParameters);
static void vInitLSM6DSL(void *pvParameters);
static void vReadLSM6DSL(void *pvParameters);
static void vReadHCSR04(void *pvParameters);
static void vPack(void *pvParameters);
static void vSetPWM(void *pvParameters);
static void vInitESC(void *pvParameters);
/* USER CODE END PFP */

int main(void)
{
	HAL_Init();

	SystemClock_Config();

	MX_GPIO_Init();
	MX_DMA_Init();
	MX_I2C1_Init();
	MX_TIM1_Init();
	MX_USART6_UART_Init();
	MX_TIM5_Init();

	/* USER CODE BEGIN 2 */
	xTaskCreate(vMain, "MAIN_TASK", configMINIMAL_STACK_SIZE, NULL, 0, &taskHandles[tMain]);
	vTaskStartScheduler();
	/* USER CODE END 2 */
}

void SystemClock_Config(void)
{

	RCC_OscInitTypeDef RCC_OscInitStruct;
	RCC_ClkInitTypeDef RCC_ClkInitStruct;

	/**Configure the main internal regulator output voltage 
	*/
	__HAL_RCC_PWR_CLK_ENABLE();

	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	/**Initializes the CPU, AHB and APB busses clocks 
	*/
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = 16;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 16;
	RCC_OscInitStruct.PLL.PLLN = 400;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
	RCC_OscInitStruct.PLL.PLLQ = 9;
	RCC_OscInitStruct.PLL.PLLR = 4;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Initializes the CPU, AHB and APB busses clocks 
	*/
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
	                            | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Configure the Systick interrupt time 
	*/
	HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);

	/**Configure the Systick 
	*/
	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

	/* SysTick_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);
}

static void MX_I2C1_Init(void)
{

	hi2c1.Instance = I2C1;
	hi2c1.Init.ClockSpeed = 400000;
	hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c1) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

}

static void MX_TIM1_Init(void)
{

	TIM_ClockConfigTypeDef sClockSourceConfig;
	TIM_MasterConfigTypeDef sMasterConfig;
	TIM_OC_InitTypeDef sConfigOC;
	TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;

	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 1999;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 999;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 5;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	sConfigOC.Pulse = 6;
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	sConfigOC.Pulse = 7;
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	sConfigOC.Pulse = 8;
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
	sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
	sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
	sBreakDeadTimeConfig.DeadTime = 0;
	sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
	sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
	sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
	if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	HAL_TIM_MspPostInit(&htim1);

}

static void MX_TIM5_Init(void)
{

	TIM_MasterConfigTypeDef sMasterConfig;
	TIM_IC_InitTypeDef sConfigIC;

	htim5.Instance = TIM5;
	htim5.Init.Prescaler = 100;
	htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim5.Init.Period = 4294967295;
	htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	if (HAL_TIM_IC_Init(&htim5) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
	sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
	sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
	sConfigIC.ICFilter = 15;
	if (HAL_TIM_IC_ConfigChannel(&htim5, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

}

static void MX_USART6_UART_Init(void)
{

	huart6.Instance = USART6;
	huart6.Init.BaudRate = 115200;
	huart6.Init.WordLength = UART_WORDLENGTH_8B;
	huart6.Init.StopBits = UART_STOPBITS_1;
	huart6.Init.Parity = UART_PARITY_NONE;
	huart6.Init.Mode = UART_MODE_TX_RX;
	huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart6.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart6) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

}

static void MX_DMA_Init(void) 
{
	/* DMA controller clock enable */
	__HAL_RCC_DMA1_CLK_ENABLE();
	__HAL_RCC_DMA2_CLK_ENABLE();

	/* DMA interrupt init */
	/* DMA1_Stream0_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
	/* DMA1_Stream1_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
	/* DMA2_Stream1_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);
	/* DMA2_Stream6_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA2_Stream6_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(DMA2_Stream6_IRQn);

}

static void MX_GPIO_Init(void)
{

	GPIO_InitTypeDef GPIO_InitStruct;

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, Pin_Trig_Pin | LD2_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : B1_Pin */
	GPIO_InitStruct.Pin = B1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : Pin_Trig_Pin LD2_Pin */
	GPIO_InitStruct.Pin = Pin_Trig_Pin | LD2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : INT_Mag_Pin INT2_XL_Pin INT1_Gyro_Pin */
	GPIO_InitStruct.Pin = INT_Mag_Pin | INT2_XL_Pin | INT1_Gyro_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI0_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(EXTI0_IRQn);

	HAL_NVIC_SetPriority(EXTI4_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(EXTI4_IRQn);

	HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

	HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

/* --------------------------------- Task ------------------------------- */
static void vMain(void *pvParameters)
{
	unsigned long ulNotifiledMain;

	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
	
	xTaskCreate(vInitLSM6DSL, "InitLSM6DSL", configMINIMAL_STACK_SIZE, NULL, 0, &taskHandles[tInitLSM6DSL]);
	HAL_Delay(100);
	xTaskCreate(vInitESC, "InitESC", configMINIMAL_STACK_SIZE, NULL, 0, &taskHandles[tInitESC]);

	xTaskCreate(vReadLSM6DSL, "ReadLSM6DSL", configMINIMAL_STACK_SIZE, NULL, 1, &taskHandles[tReadLSM6DSL]);
	xTaskCreate(vReadHCSR04, "ReadHCSR04", configMINIMAL_STACK_SIZE, NULL, 1, &taskHandles[tReadHCSR04]);
	xTaskCreate(vSetPWM, "SetPWM", configMINIMAL_STACK_SIZE, NULL, 2, &taskHandles[tSetPWM]);
	xTaskCreate(vPack, "Pack", configMINIMAL_STACK_SIZE, NULL, 3, &taskHandles[tPack]);
	
	HAL_TIM_IC_Start_IT(&htim5, TIM_CHANNEL_1);

	for (;;)
	{
		if (xTaskNotifyWait(0, 0, &ulNotifiledMain, portMAX_DELAY) == pdTRUE)
		{
			/* READ LSM6DSL + HCSR04 */
			if ((ulNotifiledMain & (MSG_LSM6DSL_X  | MSG_LSM6DSL_G)) != 0x00)
			{
				ulNotifiledMain -= (MSG_LSM6DSL_X  | MSG_LSM6DSL_G);
				vTaskResume(taskHandles[tReadLSM6DSL]);	
				vTaskResume(taskHandles[tReadHCSR04]);
			}
	
			/* PACK UART */
			else if((ulNotifiledMain & MSG_PACK) != 0x00)
			{
				ulNotifiledMain -= MSG_PACK;
				vTaskResume(taskHandles[tPack]);
			}

			/* SET PWM */
			else if((ulNotifiledMain & MSG_SETPWM) != 0x00)
			{
				ulNotifiledMain -= MSG_SETPWM;
				vTaskResume(taskHandles[tSetPWM]);
			}
		}
	}
}

static void vInitLSM6DSL(void *pvParameters)
{
	for (;;)
	{
		initLSM6DSL();
		vTaskDelete(NULL);
	}
}

static void vInitESC(void *pvParameters)
{
	for (;;)
	{
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 100);
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 100);
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 100);
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 100);

		HAL_Delay(100);

		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 50);
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 50);
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 50);
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 50);

		vTaskDelete(NULL);
	}
}

static void vReadLSM6DSL(void *pvParameters)
{	
	vTaskSuspend(NULL);
	for (;;)
	{
		readLSM6DSL();
		vTaskSuspend(NULL);
	}
}

static void vReadHCSR04(void *pvParameters)
{
	unsigned long ulNotifiledRead;

	vTaskSuspend(NULL);
	for (;;)
	{
		BaseType_t xHigherPriorityTaskWoken;
		xHigherPriorityTaskWoken = pdFALSE;
		
		HAL_GPIO_WritePin(Pin_Trig_GPIO_Port, Pin_Trig_Pin, GPIO_PIN_RESET);
		HAL_Delay(2);
		HAL_GPIO_WritePin(Pin_Trig_GPIO_Port, Pin_Trig_Pin, GPIO_PIN_SET);
		HAL_Delay(10);
		HAL_GPIO_WritePin(Pin_Trig_GPIO_Port, Pin_Trig_Pin, GPIO_PIN_RESET);
		
		if (xTaskNotifyWait(0, 0xFF, &ulNotifiledRead, portMAX_DELAY) == pdTRUE)
		{
			if ((ulNotifiledRead & (MSG_HCSR04_H | MSG_HCSR04_L)) != 0x00)
			{
				ulDataHCSR04 = (ulCount2 - ulCount1) * 0.0001717 * 10000;
				HCSR04_Pack.HCSR04Stream[0] = ulDataHCSR04  & 0x000000FF;
				HCSR04_Pack.HCSR04Stream[1] = (ulDataHCSR04 & 0x0000FF00) >> 8;
				HCSR04_Pack.HCSR04Stream[2] = (ulDataHCSR04 & 0x00FF0000) >> 16;
				HCSR04_Pack.HCSR04Stream[3] = (ulDataHCSR04 & 0xFF000000) >> 24;
				
	
			}
			xTaskNotify(taskHandles[tMain], MSG_PACK, eSetValueWithOverwrite);	
		}
		vTaskSuspend(NULL);
	}
}

static void vPack(void *pvParameters)
{
	vTaskSuspend(NULL);
	for (;;)
	{
		/* Variáveis temporárias para o Tratamento de Dado*/
		uint8_t i, tempData, hData1, hData2;
				
		/* Início da mensagem com @*/
		uartStream[0] = '@';
						
		/* Conversão dos Dados */
		for (i = 0; i < 45; ++i)
		{
			/* Seleção de Dados */
			if (i < 12)			tempData = LSM6DSL_Pack.LSM6DSLData.LSM6DSLStream[i]; /* LSM6DSL (G_XL) */
			//else if(i < 18)		tempData = 0; /* LSM303AGR (M) */	
			else if(i < 16)		tempData = HCSR04_Pack.HCSR04Stream[i - 12]; /* HCSR04 (Altura) */
			else				tempData = 0; /* LSM6DSL (TimeStamp) */
										 
			/* Separação dos nibbles */
			hData1 = (uint8_t)(tempData >> 4);
			hData2 = (uint8_t)(tempData & 0x0F);
									
			/* Conversão do Dado */
			(hData1 <= 9) ? (uartStream[2*i + 1] = hData1 + 0x30) : (hData1 <= 0x0F) ? (uartStream[2*i + 1] = (hData1 - 0x0A) + 0x41) : (uartStream[2*i + 1] = '*');
			(hData2 <= 9) ? (uartStream[2*i + 2] = hData2 + 0x30) : (hData2 <= 0x0F) ? (uartStream[2*i + 2] = (hData2 - 0x0A) + 0x41) : (uartStream[2*i + 2] = '*');
		}

		/* Monta Fim da mensagem */
		uartStream[45] = '\r';
		uartStream[46] = '\n';
		uartStream[47] = '\0';
			
		/* Envia mensagem pela UART2 */
		if (HAL_UART_Transmit_DMA(&huart6, uartStream, 48) != HAL_OK)
		{
			_Error_Handler(__FILE__, __LINE__);
		}
		
		vTaskSuspend(NULL);
	}
}

static void vSetPWM(void *pvParameters)
{
	float throttle[4];
	for (;;)
	{
		/* Converte valores de aceleração */
		throttle[0] = 50 + (uartThrottle[1] - 48) * 0.5;
		throttle[1] = 50 + (uartThrottle[2] - 48) * 0.5;
		throttle[2] = 50 + (uartThrottle[3] - 48) * 0.5;
		throttle[3] = 50 + (uartThrottle[4] - 48) * 0.5;

		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, throttle[0]);
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, throttle[1]);
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, throttle[2]);
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, throttle[3]);

		if (HAL_UART_Receive_DMA(&huart6, uartThrottle, 8) != HAL_OK)
		{
			_Error_Handler(__FILE__, __LINE__);
		}

		vTaskSuspend(NULL);
	}
}

/* ------------------------------- Callbacks ------------------------------- */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	BaseType_t xHigherPriorityTaskWoken;
	xHigherPriorityTaskWoken = pdFALSE;

	if (stateLSM6DSL == LSM6DSL_On)
	{
		if (GPIO_Pin == INT2_XL_Pin)
		{
			xTaskNotifyFromISR(taskHandles[tMain], MSG_LSM6DSL_X, eSetBits, &xHigherPriorityTaskWoken);	
		}
		if (GPIO_Pin == INT1_Gyro_Pin)
		{
			xTaskNotifyFromISR(taskHandles[tMain], MSG_LSM6DSL_G, eSetBits, &xHigherPriorityTaskWoken);	
		}
	}
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	BaseType_t xHigherPriorityTaskWoken;
	xHigherPriorityTaskWoken = pdFALSE;
	
	if (htim->Instance == TIM5)
	{
		if (HAL_GPIO_ReadPin(GPIOA, Pin_Echo_Pin))
		{
			ulCount1 = __HAL_TIM_GetCompare(&htim5, TIM_CHANNEL_1);
			xTaskNotifyFromISR(taskHandles[tReadHCSR04], MSG_HCSR04_H, eSetBits, &xHigherPriorityTaskWoken);	
		}

		else if (!HAL_GPIO_ReadPin(GPIOA, Pin_Echo_Pin))
		{
			ulCount2 = __HAL_TIM_GetCompare(&htim5, TIM_CHANNEL_1);
			xTaskNotifyFromISR(taskHandles[tReadHCSR04], MSG_HCSR04_L, eSetBits, &xHigherPriorityTaskWoken);
		}
	}	
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	BaseType_t xHigherPriorityTaskWoken;
	xHigherPriorityTaskWoken = pdFALSE;

	HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
	xTaskNotifyFromISR(taskHandles[tMain], 0x08, eSetValueWithOverwrite, &xHigherPriorityTaskWoken);	
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	asm("nop");
}

/* ------------------------------- FreeRTOS ------------------------------- */
HAL_StatusTypeDef HAL_InitTick(uint32_t TickPriority)
{
	return HAL_OK;
}

uint32_t HAL_GetTick(void)
{
	return xTaskGetTickCount();
}

void HAL_Delay(__IO uint32_t Delay)
{
	vTaskDelay(Delay);
}

void vApplicationStackOverflowHook(TaskHandle_t pxTask, char *pcTaskName)
{
	(void)pcTaskName;
	(void)pxTask;

	__asm volatile
	(
		" bkpt #01                                                  \n"
		);

	while (1) ;
}

#if configSUPPORT_DYNAMIC_ALLOCATION == 1
void vApplicationMallocFailedHook()
{
	__asm volatile
	(
		" bkpt #01                                                  \n"
		);

	while (1) ;
}
#endif // configSUPPORT_DYNAMIC_ALLOCATION

#if ((configSUPPORT_DYNAMIC_ALLOCATION == 1) && (configUSE_TIMERS == 1))
uint16_t stacks[NUM_TASKS + 3];
#elif ((configSUPPORT_DYNAMIC_ALLOCATION == 1) || (configUSE_TIMERS == 1))
uint16_t stacks[NUM_TASKS + 2];
#else
uint16_t stacks[NUM_TASKS + 1];
#endif // configSUPPORT_DYNAMIC_ALLOCATION

#ifdef FREERTOS_IDLE_CALLBACK
__weak void freeRTOSIdleTaskCallback(void)
{
	// This function must be created by user...
}
#endif

void vApplicationIdleHook(void) // Utilizo a idle task para medir quando de memoria cada task esta consumindo em tempo de execucao. Para poder ajustar conforme a necessidade.
{
	static uint8_t cont = 0;
#if NUM_TASKS > 0
	uint8_t i;
#endif
	if (cont++ == 0)
	{
#if NUM_TASKS > 0
		for (i = 0; i < NUM_TASKS; i++)
		{
			if (taskHandles[i] != NULL)
				stacks[i] = uxTaskGetStackHighWaterMark(taskHandles[i]);
		}
#endif
		stacks[NUM_TASKS] = uxTaskGetStackHighWaterMark(NULL);

#if ((configSUPPORT_DYNAMIC_ALLOCATION == 1) && (configUSE_TIMERS == 1))
		stacks[NUM_TASKS + 1] = uxTaskGetStackHighWaterMark(xTimerGetTimerDaemonTaskHandle());
		stacks[NUM_TASKS + 2] = xPortGetFreeHeapSize();
#elif configSUPPORT_DYNAMIC_ALLOCATION == 1
		stacks[NUM_TASKS + 1] = xPortGetFreeHeapSize();
#elif configUSE_TIMERS == 1
		stacks[NUM_TASKS + 1] = uxTaskGetStackHighWaterMark(xTimerGetTimerDaemonTaskHandle());
#endif // configSUPPORT_DYNAMIC_ALLOCATION

		cont = 0;
	}

#ifdef FREERTOS_IDLE_CALLBACK
	freeRTOSIdleTaskCallback();
#endif
}
/* USER CODE END 4 */

void _Error_Handler(char *file, int line)
{
	/* USER CODE BEGIN Error_Handler_Debug */
				/* User can add his own implementation to report the HAL error return state */
	while (1)
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
