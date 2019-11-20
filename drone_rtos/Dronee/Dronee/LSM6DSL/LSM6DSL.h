#ifndef __LSM6DSL__H
#define __LSM6DSL__H

#include "LSM6DSL_defines.h"
#include "stm32f4xx_hal.h"

typedef enum
{
	LSM6DSL_Off = 0,
	LSM6DSL_On,
}LSM6DSLStatus_t;

typedef struct 
{
	uint8_t LSM6DSLData_L;
	uint8_t	LSM6DSLData_H;
}LSM6DSLTypeData_Byte_U;

typedef union 
{
	LSM6DSLTypeData_Byte_U LSM6DSLData_Byte;
	int16_t	LSM6DSLData_HW;
}LSM6DSLTypeData_U;

typedef struct
{
	LSM6DSLTypeData_U G_X;
	LSM6DSLTypeData_U G_Y;
	LSM6DSLTypeData_U G_Z;

	LSM6DSLTypeData_U XL_X;
	LSM6DSLTypeData_U XL_Y;
	LSM6DSLTypeData_U XL_Z;
} struct_DataType_LSM6DSL;

typedef union 
{
	uint8_t LSM6DSLStream[12];
	struct_DataType_LSM6DSL LSM6DSLdata;	
}LSM6DSL_DataType;

typedef union
{
	uint8_t LSM6DLSTimeStream[4];
	uint32_t TimeStamp;
}LSM6DSL_TimeType;

typedef struct
{
	LSM6DSL_TimeType LSM6DSLTime;
	LSM6DSL_DataType LSM6DSLData;
}LSM6DSL_PackType;


/**
 * @brief Inicializa o Acelerometro e Girometro (I2C).
 *			Lê Who Am I Register
 *			Seta Accel para 12.5Hz na INT2
 *			Seta Gyro  para 12.5Hz na INT1
 * @param none.
 * @retval none.
 */
void initLSM6DSL(void);

void readLSM6DSL(void);

void packLSM6DSL(void);
#endif  //__INIT__H



