#include "LSM6DSL.h"
#include "stm32f4xx_hal.h"

extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef huart6;

extern uint8_t uartStream[48];

LSM6DSL_PackType LSM6DSL_Pack;

LSM6DSLStatus_t stateLSM6DSL = LSM6DSL_Off;

uint8_t i2cBuff;

/**
 * @brief Inicializa o Acelerometro e Girometro (I2C).
 *			Lê Who Am I Register
 *			Seta Accel para 12.5Hz na INT2
 *			Seta Gyro  para 12.5Hz na INT1
 * @param none.
 * @retval none.
 */
void initLSM6DSL(void)
{
	HAL_NVIC_DisableIRQ(EXTI0_IRQn);
	HAL_NVIC_DisableIRQ(EXTI4_IRQn);
	HAL_NVIC_DisableIRQ(EXTI9_5_IRQn);

	HAL_Delay(100);            // Setup CI

	/* Who am I */	
	while(HAL_I2C_Mem_Read(&hi2c1, LSM6DSL_ACC_GYRO_I2C_ADDRESS_HIGH, LSM6DSL_ACC_GYRO_WHO_AM_I_REG, 1, &i2cBuff, 1, 100) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}
	
	/* Check Who am I Response */
	if (LSM6DSL_ACC_GYRO_WHO_AM_I != i2cBuff)
	{
		for (;;)
		{
			asm("nop"); 
		}
	}

	/* Set XL ODR */
	i2cBuff = LSM6DSL_ACC_GYRO_ODR_XL_13Hz;          		// Acc = 12.5Hz (H-P mode)
	while(HAL_I2C_Mem_Write(&hi2c1, LSM6DSL_ACC_GYRO_I2C_ADDRESS_HIGH, LSM6DSL_ACC_GYRO_CTRL1_XL, 1, &i2cBuff, 1, 100) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}
	
	/* Set XL INT2 */
	i2cBuff = LSM6DSL_ACC_GYRO_INT2_DRDY_XL_ENABLED;          // INT2_DRDY_XL
	while(HAL_I2C_Mem_Write(&hi2c1, LSM6DSL_ACC_GYRO_I2C_ADDRESS_HIGH, LSM6DSL_ACC_GYRO_INT2_CTRL, 1, &i2cBuff, 1, 100) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	/* Set G ODR */
	i2cBuff = LSM6DSL_ACC_GYRO_ODR_G_13Hz;          			// Gyro = 12.5Hz (H-P mode)
	while(HAL_I2C_Mem_Write(&hi2c1, LSM6DSL_ACC_GYRO_I2C_ADDRESS_HIGH, LSM6DSL_ACC_GYRO_CTRL2_G, 1, &i2cBuff, 1, 100) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	/* Set G INT1 */
	i2cBuff = LSM6DSL_ACC_GYRO_INT1_DRDY_G_ENABLED;            // INT1_DRDY_G
	while(HAL_I2C_Mem_Write(&hi2c1, LSM6DSL_ACC_GYRO_I2C_ADDRESS_HIGH, LSM6DSL_ACC_GYRO_INT1_CTRL, 1, &i2cBuff, 1, 100) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	/* Set Pulsed INT1 INT2 */
	i2cBuff = LSM6DSL_ACC_GYRO_DRDY_PULSE;          			// DRDY_PULSED
	while(HAL_I2C_Mem_Write(&hi2c1, LSM6DSL_ACC_GYRO_I2C_ADDRESS_HIGH, LSM6DSL_ACC_GYRO_DRDY_PULSE_CFG, 1, &i2cBuff, 1, 100) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	/* Set HP Gyro */
	i2cBuff = LSM6DSL_ACC_GYRO_HP_EN_ENABLED;           		// HP_G_EN
	while(HAL_I2C_Mem_Write(&hi2c1, LSM6DSL_ACC_GYRO_I2C_ADDRESS_HIGH, LSM6DSL_ACC_GYRO_CTRL7_G, 1, &i2cBuff, 1, 100) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	/* Set TimerStamp */
	i2cBuff = LSM6DSL_ACC_GYRO_TIMER_ENABLED;            		// TIMER_EN
	while(HAL_I2C_Mem_Write(&hi2c1, LSM6DSL_ACC_GYRO_I2C_ADDRESS_HIGH, LSM6DSL_ACC_GYRO_CTRL10_C, 1, &i2cBuff, 1, 100) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	/* Set TimerStamp Resolution to 25us */
	i2cBuff = LSM6DSL_ACC_GYRO_TIMER_HR_25us;              	// TIMER_EN
	while(HAL_I2C_Mem_Write(&hi2c1, LSM6DSL_ACC_GYRO_I2C_ADDRESS_HIGH, LSM6DSL_ACC_GYRO_WAKE_UP_DUR, 1, &i2cBuff, 1, 100) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	HAL_NVIC_EnableIRQ(EXTI0_IRQn);
	HAL_NVIC_EnableIRQ(EXTI4_IRQn);
	HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
	stateLSM6DSL = LSM6DSL_On;
}

void readLSM6DSL(void)
{
	while (HAL_I2C_Mem_Read_DMA(&hi2c1, LSM6DSL_ACC_GYRO_I2C_ADDRESS_HIGH, LSM6DSL_ACC_GYRO_OUTX_L_G, 1, &LSM6DSL_Pack.LSM6DSLData.LSM6DSLStream[0], 12) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}
}

void packLSM6DSL()
{
	/**************************** PACK ***************************
	 *	@	Dado_G		Dado_XL		Dado_M		TimeStamp	\r\n\0
	 *	0	1----12		13---24		25--36		37-----44	45--47
	 *************************************************************/

	/* Variáveis temporárias para o Tratamento de Dado*/
	uint8_t i, tempData, hData1, hData2;
				
	/* Início da mensagem com @*/
	uartStream[0] = '@';
						
	/* Conversão dos Dados */
	for (i = 0; i < 45; ++i)
	{
		/* Seleção de Dados */
		if (i < 12)			tempData = LSM6DSL_Pack.LSM6DSLData.LSM6DSLStream[i]; /* LSM6DSL (G_XL) */
		else if(i < 18)		tempData = 0; /* LSM303AGR (M) */	
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
	while (HAL_UART_Transmit_DMA(&huart6, uartStream, 48) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}
}

