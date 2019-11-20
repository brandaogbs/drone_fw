#include "LSM303AGR.h"
#include "stm32f4xx_hal.h"

extern I2C_HandleTypeDef hi2c1;
extern uint8_t i2cBuff;
extern uint8_t I2C_Lock;

LSM303AGRStatus_t LSM303AGRState_init;
LSM303AGR_PackType LSM303AGR_Pack, LSM303AGR_Array[5] = { 0 }, LSM303AGR_DecimPack;
uint8_t LSM303AGR_DecimCount = 0;

/**
 * @brief Inicializa o Magne (I2C).
 * @param none.
 * @retval none.
 */
void initLSM303AGR(void)
{
	LSM303AGRState_init = LSM303AGR_read_who_am_i;
	HAL_NVIC_DisableIRQ(EXTI0_IRQn);
	HAL_NVIC_DisableIRQ(EXTI4_IRQn);
	HAL_NVIC_DisableIRQ(EXTI9_5_IRQn);

	HAL_Delay(100);  // Setup CI

	/* FSM LSM303AGR Start Up - Mode 1*/
	while(LSM303AGR_enabled != LSM303AGRState_init) 
	{
		/* Who am I */
		if (LSM303AGR_read_who_am_i == LSM303AGRState_init)
		{
			LSM303AGRState_init = LSM303AGR_wait_who_am_i;
			if (HAL_I2C_Mem_Read_IT(&hi2c1, LSM303AGR_MAG_I2C_ADDRESS_HIGH, LSM303AGR_MAG_WHO_AM_I_REG, 1, &i2cBuff, 1) != HAL_OK)
			{
				_Error_Handler(__FILE__, __LINE__);
			}
		}
		/* Set CFG A M */
		else if(LSM303AGR_set_cfg_a_m == LSM303AGRState_init)
		{
			i2cBuff = LSM303AGR_MAG_COMP_TEMP_EN | LSM303AGR_MAG_ODR_50Hz; 	// Comp temp e ODR = 50 Hz		
			LSM303AGRState_init = LSM303AGR_wait_cfg_a_m;
			if (HAL_I2C_Mem_Write_IT(&hi2c1, LSM303AGR_MAG_I2C_ADDRESS_HIGH, LSM303AGR_MAG_CFG_REG_A_M, 1, &i2cBuff, 1) != HAL_OK)
			{
				_Error_Handler(__FILE__, __LINE__);
			}
		}
		/* Set CFG B M */
		else if(LSM303AGR_set_cfg_b_m == LSM303AGRState_init)
		{
			i2cBuff = LSM303AGR_MAG_OFF_CANC;  			
			LSM303AGRState_init = LSM303AGR_wait_cfg_b_m;
			if (HAL_I2C_Mem_Write_IT(&hi2c1, LSM303AGR_MAG_I2C_ADDRESS_HIGH, LSM303AGR_MAG_CFG_REG_B_M, 1, &i2cBuff, 1) != HAL_OK)
			{
				_Error_Handler(__FILE__, __LINE__);
			}
		}	
		/* Set CFG C M */
		else if(LSM303AGR_set_cfg_c_m == LSM303AGRState_init)
		{
			i2cBuff = LSM303AGR_MAG_INT_DRDY;    			
			LSM303AGRState_init = LSM303AGR_wait_cfg_c_m;
			if (HAL_I2C_Mem_Write_IT(&hi2c1, LSM303AGR_MAG_I2C_ADDRESS_HIGH, LSM303AGR_MAG_CFG_REG_C_M, 1, &i2cBuff, 1) != HAL_OK)
			{
				_Error_Handler(__FILE__, __LINE__);
			}
		}
		/* Reset INT Mag */
		else if(LSM303AGR_read_drdy == LSM303AGRState_init)
		{   			
			LSM303AGRState_init = LSM303AGR_wait_drdy;
			if (HAL_I2C_Mem_Read_DMA(&hi2c1, LSM303AGR_MAG_I2C_ADDRESS_HIGH, LSM303AGR_MAG_OUTX_L_M, 1, &LSM303AGR_Pack.LSM303AGRData.LSM303AGRStream[0], 6) != HAL_OK)
			{
				_Error_Handler(__FILE__, __LINE__);
			}
			I2C_Lock = 1;
		}
	}
}

uint8_t decim4LSM303AGR(LSM303AGR_PackType* LSM303AGR_decimado)
{
	/* Subtrai a ultima amostra */
	LSM303AGR_Array[4].LSM303AGRData.LSM303AGRdata.M_X.Data_HW -= LSM303AGR_Array[3].LSM303AGRData.LSM303AGRdata.M_X.Data_HW;
	LSM303AGR_Array[4].LSM303AGRData.LSM303AGRdata.M_Y.Data_HW -= LSM303AGR_Array[3].LSM303AGRData.LSM303AGRdata.M_Y.Data_HW;
	LSM303AGR_Array[4].LSM303AGRData.LSM303AGRdata.M_Z.Data_HW -= LSM303AGR_Array[3].LSM303AGRData.LSM303AGRdata.M_Z.Data_HW;
	
	/* Atualzia amostras (shift) */	
	LSM303AGR_Array[3] = LSM303AGR_Array[2]; //coloar um for
	LSM303AGR_Array[2] = LSM303AGR_Array[1];
	LSM303AGR_Array[1] = LSM303AGR_Array[0];
	LSM303AGR_Array[0] = LSM303AGR_Pack;
	
	/* Soma ultima amostra no somatorio */
	LSM303AGR_Array[4].LSM303AGRData.LSM303AGRdata.M_X.Data_HW += LSM303AGR_Array[0].LSM303AGRData.LSM303AGRdata.M_X.Data_HW;
	LSM303AGR_Array[4].LSM303AGRData.LSM303AGRdata.M_Y.Data_HW += LSM303AGR_Array[0].LSM303AGRData.LSM303AGRdata.M_Y.Data_HW;
	LSM303AGR_Array[4].LSM303AGRData.LSM303AGRdata.M_Z.Data_HW += LSM303AGR_Array[0].LSM303AGRData.LSM303AGRdata.M_Z.Data_HW;
		
	LSM303AGR_DecimCount++;
	LSM303AGR_DecimCount = LSM303AGR_DecimCount % 4;
	
	if (!LSM303AGR_DecimCount)
	{
		// Decimando
		LSM303AGR_decimado->LSM303AGRData.LSM303AGRdata.M_Y.Data_HW = (LSM303AGR_Array[4].LSM303AGRData.LSM303AGRdata.M_Y.Data_HW << 2)/16;
		LSM303AGR_decimado->LSM303AGRData.LSM303AGRdata.M_X.Data_HW = (LSM303AGR_Array[4].LSM303AGRData.LSM303AGRdata.M_X.Data_HW << 2)/16;
		LSM303AGR_decimado->LSM303AGRData.LSM303AGRdata.M_Z.Data_HW = (LSM303AGR_Array[4].LSM303AGRData.LSM303AGRdata.M_Z.Data_HW << 2)/16;
	}	

	return (!LSM303AGR_DecimCount);
}

