/*
 * stm32f469xx.h
 *
 *  Created on: Feb 18, 2020
 *      Author: eriks
 */

#ifndef INC_STM32F469XX_H_
#define INC_STM32F469XX_H_

#include <stdint.h>

/************************************************
 * 				 GENERIC MACROS
 ************************************************/
#define ENABLE  	1
#define DISABLE 	0
#define SET 		ENABLE
#define RESET 		DISABLE

/************************************************
 * 					TYPEDEFS
 ************************************************/
//	GPIO
typedef struct {
	volatile uint32_t MODER;		//	Port mode register
	volatile uint32_t OTYPER;		//	Port output type register
	volatile uint32_t OSPEEDR;		//	Port output speed register
	volatile uint32_t PUPDR;		//	Port pull-up/pull-down register
	volatile uint32_t IDR;			//	Port input data register
	volatile uint32_t ODR;			//	Port output data register
	volatile uint32_t BSRR;			//	Port bit set/reset register
	volatile uint32_t LCKR;			//	Port configuration register
	volatile uint32_t AFR[2];		//	Alternate function registers [0] = low, [1] = high
} GPIO_RegDef_t;

//	RCC
typedef struct {
	volatile uint32_t CR;				//	Clock Control register
	volatile uint32_t PLLCFGR;			//	PLL configuration register
	volatile uint32_t CFGR;				//	Clock configuration register
	volatile uint32_t CIR;				//	Clock interrupt register
	volatile uint32_t AHB1RSTR;			//	AHB1 peripheral reset register
	volatile uint32_t AHB2RSTR;			//	AHB2 peripheral reset register
	volatile uint32_t AHB3RSTR;			//	AHB3 peripheral reset register
	uint32_t RESERVED0;
	volatile uint32_t APB1RSTR;			//	APB1 peripheral reset register
	volatile uint32_t APB2RSTR;			//	APB2 peripheral reset register
	uint32_t RESERVED1[2];
	volatile uint32_t AHB1ENR;			//	AHB1 peripheral clock enable register
	volatile uint32_t AHB2ENR;			//	AHB2 peripheral clock enable register
	volatile uint32_t AHB3ENR;			//	AHB3 peripheral clock enable register
	uint32_t RESERVED2;
	volatile uint32_t APB1ENR;			//	APB1 peripheral clock enable register
	volatile uint32_t APB2ENR;			//	APB2 peripheral clock enable register
	uint32_t RESERVED3[2];
	volatile uint32_t AHB1LPENR;		//	AHB1 peripheral clock enable in low power mode register
	volatile uint32_t AHB2LPENR;		//	AHB2 peripheral clock enable in low power mode register
	volatile uint32_t AHB3LPENR;		//	AHB3 peripheral clock enable in low power mode register
	uint32_t RESERVED4;
	volatile uint32_t APB1LPENR;		//	APB1 peripheral clock enable in low power mode register
	volatile uint32_t APB2LPENR;		//	APB2 peripheral clock enable in low power mode register
	uint32_t RESERVED5[2];
	volatile uint32_t BDCR;				//	Backup domain control register
	volatile uint32_t CSR;				//	Clock control and status register
	uint32_t RESERVED6[2];
	volatile uint32_t SSCGR;			//	Spread spectrum clock generation register
	volatile uint32_t PLLI2SCFGR;		//	PLLI2S configuration register
	volatile uint32_t PLLSAICFGR;		//	PLL configuration register
	volatile uint32_t DCKCFGR;			//	Dedicated clock configuration register
} RCC_RegDef_t;


/*************************************************
 * 					BASE ADDRESSES
 ************************************************/

//	MEMORY DOMAIN
#define FLASH_BASE				0x08000000U
#define SRAM1_BASE				0x20000000U
#define SRAM_BASE				SRAM1_BASE_ADDR

//	BUS DOMAIN
#define PERIPH_BASE				0x40000000U
#define APB1_BASE				PERIPH_BASE
#define APB2_BASE				0x40010000U
#define AHB1_BASE				0x40020000U
#define AHB2_BASE				0x50000000U

/*******************	AHB1	******************/
//	GPIO
#define GPIOA_BASE				AHB1_BASE
#define GPIOB_BASE				(AHB1_BASE + 0x0400U)
#define GPIOC_BASE				(AHB1_BASE + 0x0800U)
#define GPIOD_BASE				(AHB1_BASE + 0x0C00U)
#define GPIOE_BASE				(AHB1_BASE + 0x1000U)
#define GPIOF_BASE				(AHB1_BASE + 0x1400U)
#define GPIOG_BASE				(AHB1_BASE + 0x1800U)
#define GPIOH_BASE				(AHB1_BASE + 0x1C00U)
#define GPIOI_BASE				(AHB1_BASE + 0x2000U)
#define GPIOJ_BASE				(AHB1_BASE + 0x2400U)
#define GPIOK_BASE				(AHB1_BASE + 0x2800U)

//	RCC
#define RCC_BASE				(AHB1_BASE + 0x3800U)

/******************		APB1	******************/
//	I2C
#define I2C1_BASE				(APB1_BASE + 0x5400U)
#define I2C2_BASE				(APB1_BASE + 0x5800U)
#define I2C3_BASE				(APB1_BASE + 0x5C00U)
//	SPI
#define SPI2_BASE				(APB1_BASE + 0x3800U)
#define SPI3_BASE				(APB1_BASE + 0x3C00U)
//	USART
#define USART2_BASE				(APB1_BASE + 0x4400U)
#define USART3_BASE				(APB1_BASE + 0x4800U)
//	UART
#define UART4_BASE				(APB1_BASE + 0x4C00U)
#define UART5_BASE				(APB1_BASE + 0x5000U)

/******************		APB2	******************/
//	USART
#define USART1_BASE				(APB2_BASE + 0x1000U)
#define USART6_BASE				(APB2_BASE + 0x1400U)

#define SPI1_BASE				(APB2_BASE + 0x3000U)
#define SYSCFG_BASE				(APB2_BASE + 0x3800U)
#define EXTI_BASE				(APB2_BASE + 0x3C00U)


/*****************************************************
 * 				REGISTER OFFSETS
 *****************************************************/
//	RCC (Reset and clock control)
#define RCC_CR				(0x00)	//	Clock Control register
#define RCC_PLLCFGR			(0x04)	//	PLL configuration register
#define RCC_CFGR			(0x08)	//	Clock configuration register
#define RCC_CIR				(0x0C)	//	Clock interrupt register
#define RCC_AHB1RSTR		(0x10)	//	AHB1 peripheral reset register
#define RCC_AHB2RSTR		(0x14)	//	AHB2 peripheral reset register
#define RCC_AHB3RSTR		(0x18)	//	AHB3 peripheral reset register
#define RCC_APB1RSTR		(0x20)	//	APB1 peripheral reset register
#define RCC_APB2RSTR		(0x24)	//	APB2 peripheral reset register
#define RCC_AHB1ENR			(0x30)	//	AHB1 peripheral clock enable register
#define RCC_AHB2ENR			(0x34)	//	AHB2 peripheral clock enable register
#define RCC_AHB3ENR			(0x38)	//	AHB3 peripheral clock enable register
#define RCC_APB1ENR			(0x40)	//	APB1 peripheral clock enable register
#define RCC_APB2ENR			(0x44)	//	APB2 peripheral clock enable register
#define RCC_AHB1LPENR		(0x50)	//	AHB1 peripheral clock enable in low power mode register
#define RCC_AHB2LPENR		(0x54)	//	AHB2 peripheral clock enable in low power mode register
#define RCC_AHB3LPENR		(0x58)	//	AHB3 peripheral clock enable in low power mode register
#define RCC_APB1LPENR		(0x60)	//	APB1 peripheral clock enable in low power mode register
#define RCC_APB2LPENR		(0x64)	//	APB2 peripheral clock enable in low power mode register
#define RCC_BDCR			(0x70)	//	Backup domain control register
#define RCC_CSR				(0x74)	//	Clock control and status register
#define RCC_SSCGR			(0x80)	//	Spread spectrum clock generation register
#define RCC_PLLI2SCFGR		(0x84)	//	PLLI2S configuration register
#define RCC_PLLSAICFGR		(0x88)	//	PLL configuration register
#define RCC_DCKCFGR			(0x8C)	//	Dedicated clock configuration register

//	GPIO (General purpose Input/Output)
#define GPIO_MODER_OFFSET		(0x00)	//	Port mode register
#define GPIO_OTYPER_OFFSET		(0x04)	//	Port output type register
#define GPIO_OSPEEDR_OFFSET		(0x08)	//	Port output speed register
#define GPIO_PUPDR_OFFSET		(0x0C)	//	Port pull-up/pull-down register
#define GPIO_IDR_OFFSET			(0x10)	//	Port input data register
#define GPIO_ODR_OFFSET			(0x14)	//	Port output data register
#define GPIO_BSRR_OFFSET		(0x18)	//	Port bit set/reset register
#define GPIO_LCKR_OFFSET		(0x1C)	//	Port configuration lock register
#define GPIO_AFRL_OFFSET		(0x20)	//	Alternate function low register
#define GPIO_AFRH_OFFSET		(0x24)	//	Alternate function high register

//	SPI	(Serial Peripheral interface)
#define SPI_CR1_OFFSET 			(0x00)	//	Control register 1
#define SPI_CR2_OFFSET			(0x04)	//	Control register 2
#define SPI_SR_OFFSET			(0x08)	//	Status register
#define SPI_DR_OFFSET			(0x0C)	//	Data register
#define SPI_CRCPR_OFFSET		(0x10)	//	SPI CRC polynomial register
#define SPI_RXCRCR_OFFSET		(0x14)	//	RX CRC register
#define SPI_TXCRCR_OFFSET		(0x18)	//	TX CRC register
#define SPI_I2SCFGR_OFFSET		(0x1C)	//	SPI_I2S configuration register
#define SPI_I2SPR_OFFSET		(0x20)	//	SPI_I2S prescaler register

/*****************************************************
 * 				PERIPHERAL REGISTER MACROS
 *****************************************************/
//	GPIO
#define GPIOA		((GPIO_RegDef_t*) GPIOA_BASE)
#define GPIOB		((GPIO_RegDef_t*) GPIOB_BASE)
#define GPIOC		((GPIO_RegDef_t*) GPIOC_BASE)
#define GPIOD		((GPIO_RegDef_t*) GPIOD_BASE)
#define GPIOE		((GPIO_RegDef_t*) GPIOE_BASE)
#define GPIOF		((GPIO_RegDef_t*) GPIOF_BASE)
#define GPIOG		((GPIO_RegDef_t*) GPIOG_BASE)
#define GPIOH		((GPIO_RegDef_t*) GPIOH_BASE)
#define GPIOI		((GPIO_RegDef_t*) GPIOI_BASE)
#define GPIOJ		((GPIO_RegDef_t*) GPIOJ_BASE)
#define GPIOK		((GPIO_RegDef_t*) GPIOK_BASE)

//	RCC
#define RCC			((RCC_RegDef_t*) RCC_BASE)

//	I2C
//#define I2C			((I2C_))	//	I aint got time to write typedefs now

//	SPI

/******************************************************
 * 			PERIPHERAL CLOCK EN/DI MACROS
 *****************************************************/
//	GPIO
#define GPIOA_PCLK_EN() RCC->AHB1ENR |= (1 << 0)	//	Enable bit 0 in AHB1ENR (I/O port A enable)
#define GPIOB_PCLK_EN() RCC->AHB1ENR |= (1 << 1)

#define GPIOA_PCLK_DI() RCC->AHB1ENR &= ~(1 << 0)
#define GPIOB_PCLK_DI() RCC->AHB1ENR &= ~(1 << 0)

//	I2C
#define I2C1_PCLK_EN() RCC->APB1ENR |= (1 << 21)

//	SPI
#define SPI1_PCLK_EN() RCC->APB2ENR |= (1 << 21)





#endif /* INC_STM32F469XX_H_ */