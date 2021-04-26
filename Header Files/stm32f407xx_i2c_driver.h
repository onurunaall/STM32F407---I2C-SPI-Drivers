/*
 * stm32f407xx_i2c_driver.h
 */

#ifndef INC_STM32F407XX_I2C_DRIVER_H_
#define INC_STM32F407XX_I2C_DRIVER_H_

#include "stm32f407xx.h"
#include <stdint.h>

typedef struct{
	uint32_t I2C_SCLSpeed;
	uint8_t I2C_DeviceAddress;
	uint8_t I2C_ACKControl;
	uint8_t I2C_FMDutyCycle;
}I2C_Config_t;

typedef struct{
	I2C_RegDef_t *pI2Cx; 	//This holds the base address of I2Cx(x:0,1,2...) peripheral
	I2C_Config_t I2C_Config;
}I2C_Handle_t;


//@I2C_SCLSpeed
#define I2C_SCL_SPEED_SM		100000
#define I2C_SCL_SPEED_FM4K		400000
#define I2C_SCL_SPEED_FM2K		200000

//@I2C_ACKControl
#define I2C_ACK_ENABLE			1
#define I2C_ACK_DISABLE			0

//@I2C_FMDutyCycle
#define I2C_FM_DUTY_2			0
#define I2C_FM_DUTY_16_9		1

/***************API supported by this driver********************/
//Peripheral Clock Setup
void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx,uint8_t EnorDi);

//Init and De-Init
void I2C_Init(I2C_Handle_t *pI2CHandle);
void I2C_DeInit(I2C_RegDef_t *pI2Cx); //deinitiliazes thee I2C port

//Data Send and Receive

//IRQ Configuration and ISR Handling
void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void I2C_IRQPriorityConfig(uint8_t IRQNumber,uint32_t IRQPriority);

//Other Peripheral Control APIs
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName);

//app callbacks
void I2C_ApplicationEventCallBack(I2C_Handle_t *pI2CHandle, uint8_t AppEv);


#endif /* INC_STM32F407XX_I2C_DRIVER_H_ */
