
#include "stm32f407xx.h"
#include <stdint.h>

uint16_t AHB_PreScaler[8]={2,4,8,16,64,128,256,512};
uint8_t APB1_PreScaler[4]={2,4,8,16};


void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx,uint8_t EnorDi){
	if(EnorDi == ENABLE){
		if(pI2Cx == I2C1){
			I2C1_PCLK_EN();
		}
		else if(pI2Cx == I2C2){
			I2C2_PCLK_EN();
		}
		else if(pI2Cx == I2C3){
			I2C3_PCLK_EN();
		}
	}
	else{
		if(pI2Cx == I2C1){
			I2C1_PCLK_DI();
		}
		else if(pI2Cx == I2C2){
			I2C2_PCLK_DI();
		}
		else if(pI2Cx == I2C3){
			I2C3_PCLK_DI();
		}
	}
}

uint32_t RCC_GetPLLOutputClock(){
	//PLL Calculation
}


uint32_t RCC_GetPCLK1Value(void){
	uint32_t pclk1, SystemClk;
	uint8_t clksrc, temp, ahp, apb1p;
	clksrc=((RCC->CFGR >> 2) & 0x3);

	if(clksrc == 0){
		SystemClk = 16000000; //HSI
	}
	else if(clksrc == 1){
		SystemClk = 8000000; //HSE
	}
	else if(clksrc == 2){
		SystemClk = RCC_GetPLLOutputClock(); //PLL
	}

	//AHB PreScalar
	temp=((RCC->CFGR >> 4) & 0xF); //masking
	if(temp<8){
		ahbp=1;
	}else{
		ahbp=AHB_PreScaler[temp-8];
	}

	//APB1 PreScalar
	temp = ((RCC->CFGR >> 10) & 0x7); //masking
	if(temp<4){
		apb1p=1;
	}else{
		apb1p=APB1_PreScaler[temp-4];
	}

	pclk1=((SystemClk/ahbp)/apb1p);

	return pclk1;
}

void I2C_Init(I2C_Handle_t *pI2CHandle){
	uint32_t tempreg =0;

	//ack control bit
	tempreg |= pI2CHandle->I2C_Config.I2C_ACKControl << 10;
	pI2CHandle->pI2Cx->CR1 = tempreg;

	//configure the FREQ field of CR2
	tempreg=0;
	tempreg |= RCC_GetPCLK1Value()/1000000U;
	pI2CHandle->pI2Cx->CR2 = (tempreg & 0x3F); //masking the bits expect first five bits

	//program device own address (if device acts as slave)
	tempreg |= (pI2CHandle->I2C_Config.I2C_DeviceAddress << 1);
	tempreg |= (1<<14);
	pI2CHandle->pI2Cx->OAR1 |= (tempreg & 0x3F);

	//CCR calculations
	uint16_t ccr_value=0;
	tempreg=0;
	if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM){
		//The mode is standart mode
		ccr_value=(RCC_GetPLLOutputClock()/(2*pI2CHandle->I2C_Config.I2C_SCLSpeed));
		tempreg |= (ccr_value & 0xFFF); //masking bits except first 12 ones
	}
	else{
		//The mode is fast mode
		tempreg |= (1<<15);
		tempreg |= (pI2CHandle->I2C_Config.I2C_FMDutyCycle << 14);

		if(pI2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_2){
			ccr_value = (RCC_GetPLLOutputClock() / (3*pI2CHandle->I2C_Config.I2C_SCLSpeed));
		}
		else{
			ccr_value = (RCC_GetPLLOutputClock() / (25*pI2CHandle->I2C_Config.I2C_SCLSpeed));
		}
		tempreg |= (ccr_value & 0xFFF);
	}

	pI2CHandle->pI2Cx->CCR |= tempreg;

}

void I2C_DeInit(I2C_RegDef_t *pI2Cx){
	if(pI2Cx == I2C1){
		I2C1_PCLK_DI();
	}
	else if(pI2Cx == I2C2){
		I2C2_PCLK_DI();
	}
	else if(pI2Cx == I2C3){
		I2C3_PCLK_DI();
	}
} //deinitiliazes the I2C Port

