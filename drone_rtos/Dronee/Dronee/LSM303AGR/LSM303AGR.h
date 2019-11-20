#ifndef __LSM303AGR__H
#define __LSM303AGR__H

#include "LSM303AGR_defines.h"
#include "stm32f4xx_hal.h"

typedef struct {
	uint8_t LSM303AGRData_L;
	uint8_t	LSM303AGRData_H;
}LSM303AGRTypeData_Byte_U;

typedef union {
	LSM303AGRTypeData_Byte_U LSM303AGRData_Byte;
	int16_t	Data_HW;
}LSM303AGRTypeData_U;

typedef struct
{
	LSM303AGRTypeData_U M_X;
	LSM303AGRTypeData_U M_Y;
	LSM303AGRTypeData_U M_Z;
	
} struct_DataType_LSM303AGR;

typedef union {
	uint8_t LSM303AGRStream[6];
	struct_DataType_LSM303AGR LSM303AGRdata;	
}LSM303AGR_DataType;

typedef union
{
	uint8_t LSM303AGRTimeStream[4];
	uint32_t TimeStamp;
}LSM303AGR_TimeType;

typedef struct
{
	LSM303AGR_TimeType LSM303AGRTime;
	LSM303AGR_DataType LSM303AGRData;
}LSM303AGR_PackType;

typedef enum /* Estados para a FSM de inicialização */
{
	LSM303AGR_read_who_am_i = 0,	// who am i
	LSM303AGR_wait_who_am_i,

	LSM303AGR_set_cfg_a_m, 		
	LSM303AGR_wait_cfg_a_m,
	
	LSM303AGR_set_cfg_b_m, 		
	LSM303AGR_wait_cfg_b_m,
	
	LSM303AGR_set_cfg_c_m, 		
	LSM303AGR_wait_cfg_c_m,
	
	LSM303AGR_read_drdy,
	LSM303AGR_wait_drdy,
	
	LSM303AGR_enabled				// final state of FSM
}LSM303AGRStatus_t;

/**
 * @brief Inicializa o Acelerometro e Girometro (I2C).
 *			Lê Who Am I Register
 *			Seta Accel para 12.5Hz na INT2
 *			Seta Gyro  para 12.5Hz na INT1
 * @param none.
 * @retval none.
 */
void initLSM303AGR(void);
uint8_t decim4LSM303AGR(LSM303AGR_PackType* LSM303AGR_decimado);

#endif  //__INIT__H



