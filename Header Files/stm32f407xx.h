/*
 * stm32f407xx.h
 *
 *  Created on: Sep 25, 2020
 *      Author: pc
 */

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_

#include <stdint.h>
#include <stddef.h>

#define _vo 		volatile
#define _weak		__attribute__((weak))

/**************** START:Processor Specific Details *****************/

/*
 *ARM CortexMx Processor NVIC ISERx register Base Addresses
 */
#define NVIC_ISER0			((volatile uint32_t*)0xE000E100)
#define NVIC_ISER1			((volatile uint32_t*)0xE000E104)
#define NVIC_ISER2 			((volatile uint32_t*)0xE000E108)
#define NVIC_ISER3 			((volatile uint32_t*)0xE000E10C)

/*
 *ARM CortexMx Processor NVIC ICERx register Base Addresses
 */
#define NVIC_ICER0 			((volatile uint32_t*)0xE000E180)
#define NVIC_ICER1 			((volatile uint32_t*)0xE000E184)
#define NVIC_ICER2 			((volatile uint32_t*)0xE000E188)
#define NVIC_ICER3 			((volatile uint32_t*)0xE000E18C)

/*
 *ARM CortexMx Processor Priority Register Base Address Calculation
 */
#define NVIC_PR_BASE_ADDR 			((volatile uint32_t*)0xE000E400)
#define NO_PR_BITS_IMPLEMENTED		4

/**************** END:Processor Specific Details *****************/


//base addresses of Flash and SRAM memories
#define FLASH_BASEADDR			0x80000000U //Flash Memory Base Address
#define SRAM1_BASEADDR			0x20000000U //SRAM1 Base Address
#define SRAM2_BASEADDR			0x20001C00U //SRAM2 Base Address
#define ROM_BASEADDR			0x1FFF0000	//ROM Base Address
#define SRAM_BASEADDR			SRAM1_BASEADDR //SRAM Base Address

//base addresses of AHBx and APBx Bus Peripheral
#define PERIPH_BASE				0x40000000
#define APB1PERIPH_BASEADDR		PERIPH_BASE
#define	APB2PERIPH_BASEADDR		0x40010000U
#define	AHB1PERIPH_BASEADDR		0x40020000U
#define	AHB2PERIPH_BASEADDR		0x50000000U

//base addresses of peripherals which are hanging on AHB1 bus
#define	GPIOA_BASEADDR			(AHB1PERIPH_BASEADDR+0x0000)
#define	GPIOB_BASEADDR			(AHB1PERIPH_BASEADDR+0x4000)
#define	GPIOC_BASEADDR			(AHB1PERIPH_BASEADDR+0x8000)
#define	GPIOD_BASEADDR			(AHB1PERIPH_BASEADDR+0x0C00)
#define	GPIOE_BASEADDR			(AHB1PERIPH_BASEADDR+0x1000)
#define	GPIOF_BASEADDR			(AHB1PERIPH_BASEADDR+0x1400)
#define	GPIOG_BASEADDR			(AHB1PERIPH_BASEADDR+0x1800)
#define	GPIOH_BASEADDR			(AHB1PERIPH_BASEADDR+0x1C00)
#define	GPIOI_BASEADDR			(AHB1PERIPH_BASEADDR+0x2000)
#define	GPIOJ_BASEADDR			(AHB1PERIPH_BASEADDR+0x2400)
#define	GPIOK_BASEADDR			(AHB1PERIPH_BASEADDR+0x2800)

#define RCC_BASEADDR			(AHB1PERIPH_BASEADDR+0x3800)

//base addresses of peripherals which are hanging on APB1 bus
#define	I2C1_BASEADDR			(APB1PERIPH_BASEADDR+0x5400)
#define	I2C2_BASEADDR			(APB1PERIPH_BASEADDR+0x5800)
#define	I2C3_BASEADDR			(APB1PERIPH_BASEADDR+0x5C00)
#define	SPI2_BASEADDR			(APB1PERIPH_BASEADDR+0x3800)
#define	SPI3_BASEADDR			(APB1PERIPH_BASEADDR+0x3C00)
#define	USART2_BASEADDR			(APB1PERIPH_BASEADDR+0x4400)
#define	USART3_BASEADDR			(APB1PERIPH_BASEADDR+0x4800)
#define	UART4_BASEADDR			(APB1PERIPH_BASEADDR+0x4C00)
#define	UART5_BASEADDR			(APB1PERIPH_BASEADDR+0x5000)

//base addresses of peripherals which are hanging on APB2 bus
#define	EXTI_BASEADDR			(APB2PERIPH_BASEADDR+0x3C00)
#define	SPI1_BASEADDR			(APB2PERIPH_BASEADDR+0x3000)
#define	USART1_BASEADDR			(APB2PERIPH_BASEADDR+0x1000)
#define	USART6_BASEADDR			(APB2PERIPH_BASEADDR+0x1400)
#define	SYSCFG_BASEADDR			(APB2PERIPH_BASEADDR+0x3800)

//Peripheral Register Definition Structure For RCC
typedef struct{
	volatile uint32_t MODER; 		//GPIO port mode register OFFSET=0x00
	volatile uint32_t OTYPER; 		//GPIO port output type register
	volatile uint32_t OSPEEDR; 		//GPIO port output speed register
	volatile uint32_t PUPDR;		//GPIO port pull-up/pull-down register
	volatile uint32_t IDR; 			//GPIO port input data register
	volatile uint32_t ODR;			//GPIO port output data register
	volatile uint32_t BSRR;			//GPIO port bit set/reset register
	volatile uint32_t LCKR;			//GPIO port configuration lock register
	volatile uint32_t AFR[2];		//AFR[0]=GPIO alternate function low afr[1]=high register
}GPIO_RegDef_t;

//Peripheral Register Definition Structure for EXTI
typedef struct{
	volatile uint32_t IMR;
	volatile uint32_t EMR;
	volatile uint32_t RTSR;
	volatile uint32_t FTSR;
	volatile uint32_t SWIER;
	volatile uint32_t PR;
}EXTI_RegDef_t;

//Peripheral Register Definition Structure for SYSCFG
typedef struct{
	volatile uint32_t MEMCRP;
	volatile uint32_t PMC;
	volatile uint32_t EXTICR[4];
	uint32_t RESERVED1[2];
	volatile uint32_t CMPCR;
	uint32_t RESERVED2[2];
	volatile uint32_t CFRG;
}SYSCFG_RegDef_t;

typedef struct{
	volatile uint32_t CR;			//RCC clock control register
	volatile uint32_t PLLCFGR;		//RCC PLL configuration register
	volatile uint32_t CFGR;			//RCC clock configuration register
	volatile uint32_t CIR;			//RCC clock interrupt register
	volatile uint32_t AHB1RSTR;		//RCC AHB1 peripheral reset register
	volatile uint32_t AHB2RSTR;		//RCC AHB2 peripheral reset register
	volatile uint32_t AHB3RSTR;		//RCC AHB3 peripheral reset register
	uint32_t RESERVED0;				// OFFSET=0x1C
	volatile uint32_t APB1RSTR;		//RCC APB1 peripheral reset register
	volatile uint32_t APB2RSTR;		//RCC APB2 peripheral reset register
	uint32_t RESERVED1[2];
	volatile uint32_t AHB1ENR;		//RCC AHB1 peripheral clock enable register
	volatile uint32_t AHB2ENR;		//RCC AHB2 peripheral clock enable register
	volatile uint32_t AHB3ENR;		//RCC AHB3 peripheral clock enable register
	uint32_t RESERVED2;
	volatile uint32_t APB1ENR;		//RCC APB1 peripheral clock enable register
	volatile uint32_t APB2ENR;		//RCC APB2 peripheral clock enable register
	uint32_t RESERVED3[2];
	volatile uint32_t AHB1LPENR;	//RCC AHB1 peripheral clock enable in low power mode register
	volatile uint32_t AHB2LPENR;	//RCC AHB2 peripheral clock enable in low power mode register
	volatile uint32_t AHB3LPENR;	//RCC AHB3 peripheral clock enable in low power mode register
	uint32_t RESERVED4;
	volatile uint32_t APB1LPENR;	//RCC APB1 peripheral clock enable in low power mode register
	volatile uint32_t APB2LPENR;	//RCC APB2 peripheral clock enabled in low power mode register
	uint32_t RESERVED5[2];
	volatile uint32_t BDCR;			//RCC Backup domain control register
	volatile uint32_t CSR;			//RCC clock control & status register
	uint32_t RESERVED6[2];
	volatile uint32_t SSCGR;		//RCC spread spectrum clock generation register
	volatile uint32_t PLLI2SCFGR;	//RCC PLLI2S configuration register
}RCC_RegDef_t;

typedef struct{
	volatile uint32_t CR1;
	volatile uint32_t CR2;
	volatile uint32_t SR;
	volatile uint32_t DR;
	volatile uint32_t CRC;
	volatile uint32_t RXCRCPR;
	volatile uint32_t TXCRCPR;
	volatile uint32_t I2SCFGR;
	volatile uint32_t I2SSPR;
}SPI_RegDef_t;

typedef struct{
	volatile uint32_t CR1;
	volatile uint32_t CR2;
	volatile uint32_t OAR1;
	volatile uint32_t OAR2;
	volatile uint32_t DR;
	volatile uint32_t SR1;
	volatile uint32_t SR2;
	volatile uint32_t CCR;
	volatile uint32_t TRISE;
	volatile uint32_t FLTR;
}I2C_RegDef_t;

//Peripheral Definitions (Peripheral base addresses typecasted to xxx_RegDef_t)
#define GPIOA 				((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB 				((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC 				((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD 				((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE 				((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF 				((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define GPIOG 				((GPIO_RegDef_t*)GPIOG_BASEADDR)
#define GPIOH 				((GPIO_RegDef_t*)GPIOH_BASEADDR)
#define GPIOI 				((GPIO_RegDef_t*)GPIOI_BASEADDR)
#define GPIOJ 				((GPIO_RegDef_t*)GPIOJ_BASEADDR)
#define GPIOK 				((GPIO_RegDef_t*)GPIOK_BASEADDR)

#define RCC					((RCC_RegDef_t*)RCC_BASEADDR)
#define EXTI 				((EXTI_RegDef_t*)EXTI_BASEADDR)
#define SYSCFG				((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)

#define SPI1				((SPI_RegDef_t*)SPI1_BASEADDR)
#define SPI2				((SPI_RegDef_t*)SPI2_BASEADDR)
#define SPI3				((SPI_RegDef_t*)SPI3_BASEADDR)

#define I2C1				((I2C_RegDef_t*)I2C1_BASEADDR)
#define I2C2				((I2C_RegDef_t*)I2C2_BASEADDR)
#define I2C3				((I2C_RegDef_t*)I2C3_BASEADDR)

//Clock Enable Macros for GPIOx Peripherals
#define GPIOA_PCLK_EN()		(RCC->AHB1ENR |=(1<<0))
#define GPIOB_PCLK_EN()		(RCC->AHB1ENR |=(1<<1))
#define GPIOC_PCLK_EN()		(RCC->AHB1ENR |=(1<<2))
#define GPIOD_PCLK_EN()		(RCC->AHB1ENR |=(1<<3))
#define GPIOE_PCLK_EN()		(RCC->AHB1ENR |=(1<<4))
#define GPIOF_PCLK_EN()		(RCC->AHB1ENR |=(1<<5))
#define GPIOG_PCLK_EN()		(RCC->AHB1ENR |=(1<<6))
#define GPIOH_PCLK_EN()		(RCC->AHB1ENR |=(1<<7))
#define GPIOI_PCLK_EN()		(RCC->AHB1ENR |=(1<<8))

//Clock Enable Macros for I2Cx Peripherals
#define I2C1_PCLK_EN() 		(RCC->APB1ENR |=(1<<21))
#define I2C2_PCLK_EN() 		(RCC->APB1ENR |=(1<<22))
#define I2C3_PCLK_EN() 		(RCC->APB1ENR |=(1<<23))

//Clock Enable Macros for SPIx Peripherals
#define SPI1_PCLK_EN() 		(RCC->APB2ENR |=(1<<12))
#define SPI2_PCLK_EN() 		(RCC->APB1ENR |=(1<<14))
#define SPI3_PCLK_EN() 		(RCC->APB1ENR |=(1<<15))
#define SPI4_PCLK_EN() 		(RCC->APB2ENR |=(1<<13))

//Clock Enable Macros for USARTx Peripherals
#define USART2_PCLK_EN() 	(RCC->APB1ENR |=(1<<17))
#define USART3_PCLK_EN() 	(RCC->APB1ENR |=(1<<18))
#define USART1_PCLK_EN() 	(RCC->APB2ENR |=(1<<4))
#define USART6_PCLK_EN() 	(RCC->APB2ENR |=(1<<5))
#define UART4_PCLK_EN() 	(RCC->APB1ENR |=(1<<19))
#define UART5_PCLK_EN() 	(RCC->APB1ENR |=(1<<20))

//Clock Enable Macros for SYSCFGx Peripheral
#define SYSCFG_PCLK_EN() 	(RCC->APB2ENR |=(1<<14))

//Clock Disable Macros for GPIOx peripherals
#define GPIOA_PCLK_DI() 	(RCC->AHB1ENR &= ~(1<<0))
#define GPIOB_PCLK_DI()		(RCC->AHB1ENR &= ~(1<<1))
#define GPIOC_PCLK_DI()		(RCC->AHB1ENR &= ~(1<<2))
#define GPIOD_PCLK_DI()		(RCC->AHB1ENR &= ~(1<<3))
#define GPIOE_PCLK_DI()		(RCC->AHB1ENR &= ~(1<<4))
#define GPIOF_PCLK_DI()		(RCC->AHB1ENR &= ~(1<<5))
#define GPIOG_PCLK_DI()		(RCC->AHB1ENR &= ~(1<<6))
#define GPIOH_PCLK_DI()		(RCC->AHB1ENR &= ~(1<<7))
#define GPIOI_PCLK_DI()		(RCC->AHB1ENR &= ~(1<<8))

//Clock Disable Macros for I2Cx Peripherals
#define I2C1_PCLK_DI() 		(RCC->APB1ENR &= ~(1<<21))
#define I2C2_PCLK_DI() 		(RCC->APB1ENR &= ~(1<<22))
#define I2C3_PCLK_DI() 		(RCC->APB1ENR &= ~(1<<23))

//Clock Disable Macros for SPIx Peripherals
#define SPI1_PCLK_DI() 		(RCC->APB2ENR &= ~(1<<12))
#define SPI2_PCLK_DI() 		(RCC->APB1ENR &= ~(1<<14))
#define SPI3_PCLK_DI() 		(RCC->APB1ENR &= ~(1<<15))
#define SPI4_PCLK_DI() 		(RCC->APB2ENR &= ~(1<<13))

//Clock Disable Macros for USARTx Peripherals
#define USART2_PCLK_DI() 	(RCC->APB1ENR &= ~(1<<17))
#define USART3_PCLK_DI() 	(RCC->APB1ENR &= ~(1<<18))
#define USART1_PCLK_DI() 	(RCC->APB2ENR &= ~(1<<4))
#define USART6_PCLK_DI() 	(RCC->APB2ENR &= ~(1<<5))
#define UART4_PCLK_DI() 	(RCC->APB1ENR &= ~(1<<19))
#define UART5_PCLK_DI() 	(RCC->APB1ENR &= ~(1<<20))

//Clock Disable Macros for SYSCFGx Peripheral
#define SYSCFG_PCLK_DI() 	(RCC->APB2ENR &= ~(1<<14))

//RESET GPIOx peripheral macros
#define GPIOA_REG_RESET()	do{(RCC->AHB1RSTR |= (1<<0)); (RCC->AHB1RSTR &= ~(1<<0));}while(0)
#define GPIOB_REG_RESET()	do{(RCC->AHB1RSTR |= (1<<1)); (RCC->AHB1RSTR &= ~(1<<1));}while(0)
#define GPIOC_REG_RESET()	do{(RCC->AHB1RSTR |= (1<<2)); (RCC->AHB1RSTR &= ~(1<<2));}while(0)
#define GPIOD_REG_RESET()	do{(RCC->AHB1RSTR |= (1<<3)); (RCC->AHB1RSTR &= ~(1<<3));}while(0)
#define GPIOE_REG_RESET()	do{(RCC->AHB1RSTR |= (1<<4)); (RCC->AHB1RSTR &= ~(1<<4));}while(0)
#define GPIOF_REG_RESET()	do{(RCC->AHB1RSTR |= (1<<5)); (RCC->AHB1RSTR &= ~(1<<5));}while(0)
#define GPIOG_REG_RESET()	do{(RCC->AHB1RSTR |= (1<<6)); (RCC->AHB1RSTR &= ~(1<<6));}while(0)
#define GPIOH_REG_RESET()	do{(RCC->AHB1RSTR |= (1<<7)); (RCC->AHB1RSTR &= ~(1<<7));}while(0)
#define GPIOI_REG_RESET()	do{(RCC->AHB1RSTR |= (1<<8)); (RCC->AHB1RSTR &= ~(1<<8));}while(0)

#define GPIO_BASEADDT_TO_CODE(x)	( (x == GPIOA) ? 0 :\
									  (x == GPIOB) ? 1 :\
									  (x == GPIOC) ? 2 :\
									  (x == GPIOB) ? 3 :\
									  (x == GPIOA) ? 4 :\
									  (x == GPIOB) ? 5 :\
									  (x == GPIOA) ? 6 :\
									  (x == GPIOB) ? 7 :0 )

//IRQ(Interrupt Request) Number of STM32F407XX mcu
#define IRQ_NO_EXTI0		6
#define IRQ_NO_EXTI1		7
#define IRQ_NO_EXTI2		8
#define IRQ_NO_EXTI3		9
#define IRQ_NO_EXTI4		10
#define IRQ_NO_EXTI9_5		23
#define IRQ_NO_EXTI15_10	40

#define NVIC_IRQ_PRI0		0
#define NVIC_IRQ_PRI1		1
#define NVIC_IRQ_PRI2		2
#define NVIC_IRQ_PRI3		3
#define NVIC_IRQ_PRI4		4
#define NVIC_IRQ_PRI5		5
#define NVIC_IRQ_PRI6		6
#define NVIC_IRQ_PRI7		7
#define NVIC_IRQ_PRI8		8
#define NVIC_IRQ_PRI9		9
#define NVIC_IRQ_PRI10		10
#define NVIC_IRQ_PRI11		11
#define NVIC_IRQ_PRI12		12
#define NVIC_IRQ_PRI13		13
#define NVIC_IRQ_PRI14		14
#define NVIC_IRQ_PRI15		15


//some generic macros
#define ENABLE 				1
#define DISABLE 			0
#define SET 				ENABLE
#define RESET 				DISABLE
#define GPIO_PIN_SET		SET
#define GPIO_PIN_RESET		RESET
#define FLAG_RESET			RESET
#define FLAG_SET			SET

/*************Bit Position Definition of SPI peripheral***************/
//Bit Position definition SPI_CR1
#define SPI_CR1_CPHA		0
#define SPI_CR1_CPOL		1
#define SPI_CR1_MSTR		2
#define SPI_CR1_BR			3
#define SPI_CR1_SPE			6
#define SPI_CR1_LSBFIRST	7
#define SPI_CR1_SSI			8
#define SPI_CR1_SSM			9
#define SPI_CR1_RXONLY		10
#define SPI_CR1_DFF			11
#define SPI_CR1_CRCNEXT		12
#define SPI_CR1_CRCEN		13
#define SPI_CR1_BIDIOE		14
#define SPI_CR1_BIDIMODE	15

//Bit Position Definitions SPI_CR2
#define SPI_CR2_RDXMAEN		0
#define SPI_CR2_TXDMAEN		1
#define SPI_CR2_SSOE		2
#define SPI_CR2_FRF			4
#define SPI_CR2_ERRIE		5
#define SPI_CR2_RXNEIE		6
#define SPI_CR2_TXEIE		7

//Bit Position Definition SPI_SR
#define SPI_SR_RXNE			0
#define SPI_SR_TXE			1
#define SPI_SR_CHSIDE		2
#define SPI_SR_UDR			3
#define SPI_SR_CRCERR		4
#define SPI_SR_MODF			5
#define SPI_SR_OVR			6
#define SPI_SR_BSY			7
#define SPI_SR_FRE			8

/*************Bit Position Definition of I2C peripheral***************/
//Bit Position definition I2C_CR1
#define I2C_CR1_PE			0
#define I2C_CR1_SMBU		1
#define I2C_CR1_SMPTYPE		3
#define I2C_CR1_ENARP		4
#define I2C_CR1_ENPEC		5
#define I2C_CR1_ENGC		6
#define I2C_CR1_NOSTRECTH	7
#define I2C_CR1_START		8
#define I2C_CR1_STOP		9
#define I2C_CR1_ACK			10
#define I2C_CR1_POS			11
#define I2C_CR1_PEC			12
#define I2C_CR1_ALERT		13
#define I2C_CR1_SWRST		15

//Bit Position Definitions I2C_CR2
#define I2C_CR2_FREQ		0
#define I2C_CR2_ITERREN		8
#define I2C_CR2_ITEVTEN		9
#define I2C_CR2_ITBUFEN		10
#define I2C_CR2_DMAEN		11
#define I2C_CR2_LAST		12

//Bit Position Definitions I2C_SR1
#define I2C_CR1_SB			0
#define I2C_CR1_ADDR		1
#define I2C_CR1_BTF			2
#define I2C_CR1_ADD10		3
#define I2C_CR1_STOPF		4
#define I2C_CR1_RxNE		6
#define I2C_CR1_TxE			7
#define I2C_CR1_BERR		8
#define I2C_CR1_ARLO		9
#define I2C_CR1_AF			10
#define I2C_CR1_OVR			11
#define I2C_CR1_PECERR		12
#define I2C_CR1_TIMEOUT		14
#define I2C_CR1_SMBALERT	15

//Bit Position Definitions I2C_SR2
#define I2C_CR1_MSL			0
#define I2C_CR1_BUSY		1
#define I2C_CR1_TRA			2
#define I2C_CR1_GENCALL		4
#define I2C_CR1_DUALF		7

//Bit Position Definitions I2C_CCR
#define I2C_CCR_CCR			0
#define I2C_CCR_DUTY		14
#define I2C_CCR_FS			15


#include "stm32f407xx_gpio_driver.h"
#include "stm32f407xx_spi_driver.h"
#include "stm32f407xx_i2c_driver.h"

#endif /* INC_STM32F407XX_H_ */
