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
#define ENABLE  		1
#define DISABLE 		0
#define SET 			ENABLE
#define RESET 			DISABLE
#define GPIO_PIN_SET	SET
#define GPIO_PIN_RESET	RESET

/*
 * @GPIOx_TO_EXTICFG
 */
#define GPIOx_TO_EXTICFG(x) 	( (x == GPIOA) ? 0x00U :\
								(x == GPIOB) ? 0x01U :\
								(x == GPIOC) ? 0x02U :\
								(x == GPIOD) ? 0x03U :\
								(x == GPIOE) ? 0x04U :\
								(x == GPIOF) ? 0x05U :\
								(x == GPIOG) ? 0x06U :\
								(x == GPIOH) ? 0x07U : 0)		//	Only in EXRICR1

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

//	EXTI
typedef struct
{
	volatile uint32_t IMR;    /*!< EXTI Interrupt mask register,            Address offset: 0x00 */
	volatile uint32_t EMR;    /*!< EXTI Event mask register,                Address offset: 0x04 */
	volatile uint32_t RTSR;   /*!< EXTI Rising trigger selection register,  Address offset: 0x08 */
	volatile uint32_t FTSR;   /*!< EXTI Falling trigger selection register, Address offset: 0x0C */
	volatile uint32_t SWIER;  /*!< EXTI Software interrupt event register,  Address offset: 0x10 */
	volatile uint32_t PR;     /*!< EXTI Pending register,                   Address offset: 0x14 */
} EXTI_RegDef_t;

//	SYSCFG
typedef struct
{
  volatile uint32_t MEMRMP;       /*!< SYSCFG memory remap register,                      Address offset: 0x00      */
  volatile uint32_t PMC;          /*!< SYSCFG peripheral mode configuration register,     Address offset: 0x04      */
  volatile uint32_t EXTICR[4];    /*!< SYSCFG external interrupt configuration registers, Address offset: 0x08-0x14 */
  uint32_t      RESERVED[2];  /*!< Reserved, 0x18-0x1C                                                          */
  volatile uint32_t CMPCR;        /*!< SYSCFG Compensation cell control register,         Address offset: 0x20      */
} SYSCFG_RegDef_t;

//	NVIC IRQ Numbers
typedef enum
{
/******  STM32 specific Interrupt Numbers **********************************************************************/
  WWDG_IRQn                   = 0,      /*!< Window WatchDog Interrupt                                         */
  PVD_IRQn                    = 1,      /*!< PVD through EXTI Line detection Interrupt                         */
  TAMP_STAMP_IRQn             = 2,      /*!< Tamper and TimeStamp interrupts through the EXTI line             */
  RTC_WKUP_IRQn               = 3,      /*!< RTC Wakeup interrupt through the EXTI line                        */
  FLASH_IRQn                  = 4,      /*!< FLASH global Interrupt                                            */
  RCC_IRQn                    = 5,      /*!< RCC global Interrupt                                              */
  EXTI0_IRQn                  = 6,      /*!< EXTI Line0 Interrupt                                              */
  EXTI1_IRQn                  = 7,      /*!< EXTI Line1 Interrupt                                              */
  EXTI2_IRQn                  = 8,      /*!< EXTI Line2 Interrupt                                              */
  EXTI3_IRQn                  = 9,      /*!< EXTI Line3 Interrupt                                              */
  EXTI4_IRQn                  = 10,     /*!< EXTI Line4 Interrupt                                              */
  DMA1_Stream0_IRQn           = 11,     /*!< DMA1 Stream 0 global Interrupt                                    */
  DMA1_Stream1_IRQn           = 12,     /*!< DMA1 Stream 1 global Interrupt                                    */
  DMA1_Stream2_IRQn           = 13,     /*!< DMA1 Stream 2 global Interrupt                                    */
  DMA1_Stream3_IRQn           = 14,     /*!< DMA1 Stream 3 global Interrupt                                    */
  DMA1_Stream4_IRQn           = 15,     /*!< DMA1 Stream 4 global Interrupt                                    */
  DMA1_Stream5_IRQn           = 16,     /*!< DMA1 Stream 5 global Interrupt                                    */
  DMA1_Stream6_IRQn           = 17,     /*!< DMA1 Stream 6 global Interrupt                                    */
  ADC_IRQn                    = 18,     /*!< ADC1, ADC2 and ADC3 global Interrupts                             */
  CAN1_TX_IRQn                = 19,     /*!< CAN1 TX Interrupt                                                 */
  CAN1_RX0_IRQn               = 20,     /*!< CAN1 RX0 Interrupt                                                */
  CAN1_RX1_IRQn               = 21,     /*!< CAN1 RX1 Interrupt                                                */
  CAN1_SCE_IRQn               = 22,     /*!< CAN1 SCE Interrupt                                                */
  EXTI9_5_IRQn                = 23,     /*!< External Line[9:5] Interrupts                                     */
  TIM1_BRK_TIM9_IRQn          = 24,     /*!< TIM1 Break interrupt and TIM9 global interrupt                    */
  TIM1_UP_TIM10_IRQn          = 25,     /*!< TIM1 Update Interrupt and TIM10 global interrupt                  */
  TIM1_TRG_COM_TIM11_IRQn     = 26,     /*!< TIM1 Trigger and Commutation Interrupt and TIM11 global interrupt */
  TIM1_CC_IRQn                = 27,     /*!< TIM1 Capture Compare Interrupt                                    */
  TIM2_IRQn                   = 28,     /*!< TIM2 global Interrupt                                             */
  TIM3_IRQn                   = 29,     /*!< TIM3 global Interrupt                                             */
  TIM4_IRQn                   = 30,     /*!< TIM4 global Interrupt                                             */
  I2C1_EV_IRQn                = 31,     /*!< I2C1 Event Interrupt                                              */
  I2C1_ER_IRQn                = 32,     /*!< I2C1 Error Interrupt                                              */
  I2C2_EV_IRQn                = 33,     /*!< I2C2 Event Interrupt                                              */
  I2C2_ER_IRQn                = 34,     /*!< I2C2 Error Interrupt                                              */
  SPI1_IRQn                   = 35,     /*!< SPI1 global Interrupt                                             */
  SPI2_IRQn                   = 36,     /*!< SPI2 global Interrupt                                             */
  USART1_IRQn                 = 37,     /*!< USART1 global Interrupt                                           */
  USART2_IRQn                 = 38,     /*!< USART2 global Interrupt                                           */
  USART3_IRQn                 = 39,     /*!< USART3 global Interrupt                                           */
  EXTI15_10_IRQn              = 40,     /*!< External Line[15:10] Interrupts                                   */
  RTC_Alarm_IRQn              = 41,     /*!< RTC Alarm (A and B) through EXTI Line Interrupt                   */
  OTG_FS_WKUP_IRQn            = 42,     /*!< USB OTG FS Wakeup through EXTI line interrupt                     */
  TIM8_BRK_TIM12_IRQn         = 43,     /*!< TIM8 Break Interrupt and TIM12 global interrupt                   */
  TIM8_UP_TIM13_IRQn          = 44,     /*!< TIM8 Update Interrupt and TIM13 global interrupt                  */
  TIM8_TRG_COM_TIM14_IRQn     = 45,     /*!< TIM8 Trigger and Commutation Interrupt and TIM14 global interrupt */
  TIM8_CC_IRQn                = 46,     /*!< TIM8 Capture Compare global interrupt                             */
  DMA1_Stream7_IRQn           = 47,     /*!< DMA1 Stream7 Interrupt                                            */
  FMC_IRQn                    = 48,     /*!< FMC global Interrupt                                              */
  SDIO_IRQn                   = 49,     /*!< SDIO global Interrupt                                             */
  TIM5_IRQn                   = 50,     /*!< TIM5 global Interrupt                                             */
  SPI3_IRQn                   = 51,     /*!< SPI3 global Interrupt                                             */
  UART4_IRQn                  = 52,     /*!< UART4 global Interrupt                                            */
  UART5_IRQn                  = 53,     /*!< UART5 global Interrupt                                            */
  TIM6_DAC_IRQn               = 54,     /*!< TIM6 global and DAC1&2 underrun error  interrupts                 */
  TIM7_IRQn                   = 55,     /*!< TIM7 global interrupt                                             */
  DMA2_Stream0_IRQn           = 56,     /*!< DMA2 Stream 0 global Interrupt                                    */
  DMA2_Stream1_IRQn           = 57,     /*!< DMA2 Stream 1 global Interrupt                                    */
  DMA2_Stream2_IRQn           = 58,     /*!< DMA2 Stream 2 global Interrupt                                    */
  DMA2_Stream3_IRQn           = 59,     /*!< DMA2 Stream 3 global Interrupt                                    */
  DMA2_Stream4_IRQn           = 60,     /*!< DMA2 Stream 4 global Interrupt                                    */
  ETH_IRQn                    = 61,     /*!< Ethernet global Interrupt                                         */
  ETH_WKUP_IRQn               = 62,     /*!< Ethernet Wakeup through EXTI line Interrupt                       */
  CAN2_TX_IRQn                = 63,     /*!< CAN2 TX Interrupt                                                 */
  CAN2_RX0_IRQn               = 64,     /*!< CAN2 RX0 Interrupt                                                */
  CAN2_RX1_IRQn               = 65,     /*!< CAN2 RX1 Interrupt                                                */
  CAN2_SCE_IRQn               = 66,     /*!< CAN2 SCE Interrupt                                                */
  OTG_FS_IRQn                 = 67,     /*!< USB OTG FS global Interrupt                                       */
  DMA2_Stream5_IRQn           = 68,     /*!< DMA2 Stream 5 global interrupt                                    */
  DMA2_Stream6_IRQn           = 69,     /*!< DMA2 Stream 6 global interrupt                                    */
  DMA2_Stream7_IRQn           = 70,     /*!< DMA2 Stream 7 global interrupt                                    */
  USART6_IRQn                 = 71,     /*!< USART6 global interrupt                                           */
  I2C3_EV_IRQn                = 72,     /*!< I2C3 event interrupt                                              */
  I2C3_ER_IRQn                = 73,     /*!< I2C3 error interrupt                                              */
  OTG_HS_EP1_OUT_IRQn         = 74,     /*!< USB OTG HS End Point 1 Out global interrupt                       */
  OTG_HS_EP1_IN_IRQn          = 75,     /*!< USB OTG HS End Point 1 In global interrupt                        */
  OTG_HS_WKUP_IRQn            = 76,     /*!< USB OTG HS Wakeup through EXTI interrupt                          */
  OTG_HS_IRQn                 = 77,     /*!< USB OTG HS global interrupt                                       */
  DCMI_IRQn                   = 78,     /*!< DCMI global interrupt                                             */
  HASH_RNG_IRQn               = 80,     /*!< Hash and Rng global interrupt                                     */
  FPU_IRQn                    = 81,     /*!< FPU global interrupt                                              */
  UART7_IRQn                  = 82,     /*!< UART7 global interrupt                                            */
  UART8_IRQn                  = 83,     /*!< UART8 global interrupt                                            */
  SPI4_IRQn                   = 84,     /*!< SPI4 global Interrupt                                             */
  SPI5_IRQn                   = 85,     /*!< SPI5 global Interrupt                                             */
  SPI6_IRQn                   = 86,     /*!< SPI6 global Interrupt                                             */
  SAI1_IRQn                   = 87,     /*!< SAI1 global Interrupt                                             */
  LTDC_IRQn                   = 88,     /*!< LTDC global Interrupt                                              */
  LTDC_ER_IRQn                = 89,     /*!< LTDC Error global Interrupt                                        */
  DMA2D_IRQn                  = 90,     /*!< DMA2D global Interrupt                                            */
  QUADSPI_IRQn                = 91,     /*!< QUADSPI global Interrupt                                          */
  DSI_IRQn                    = 92      /*!< DSI global Interrupt                                              */
} IRQn_Type;




/*************************************************
 * 					BASE ADDRESSES
 ************************************************/
//	ARM CORTEX
#define NVIC_ISER_BASE			0xE000E100U		//	Interrupt set-enable registers
#define NVIC_ICER_BASE			0xE000E180U		//	Interrupt clear-enable registers
#define NVIC_ISPR_BASE			0xE000E200U		//	Interrupt Set-Pending registers
#define NVIC_ICPR_BASE			0xE000E280U		//	Interrupt clear-Pending registers
#define NVIC_IABR_BASE			0xE000E300U		//	Interrupt Active bit registers
#define NVIC_IPR_BASE			0xE000E400U		//	Interrupt priority registers
#define NVIC_STIR_BASE			0xE000EF00U		//	Software trigger interrupt register

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

//	EXTI
#define EXTI		((EXTI_RegDef_t*) EXTI_BASE)

//	SYSCFG
#define SYSCFG		((SYSCFG_RegDef_t*) SYSCFG_BASE)

//	NVIC
#define NVIC_ISER_REG	((volatile uint32_t*) NVIC_ISER_BASE)
#define NVIC_ICER_REG	((volatile uint32_t*) NVIC_ICER_BASE)


/******************************************************
 * 			PERIPHERAL CLOCK EN/DI MACROS
 *****************************************************/
//	GPIO
#define GPIOA_PCLK_EN() RCC->AHB1ENR |= (1 << 0)	//	Enable bit 0 in AHB1ENR (I/O port A enable)
#define GPIOB_PCLK_EN() RCC->AHB1ENR |= (1 << 1)
#define GPIOC_PCLK_EN() RCC->AHB1ENR |= (1 << 2)
#define GPIOD_PCLK_EN() RCC->AHB1ENR |= (1 << 3)
#define GPIOE_PCLK_EN() RCC->AHB1ENR |= (1 << 4)
#define GPIOF_PCLK_EN() RCC->AHB1ENR |= (1 << 5)
#define GPIOG_PCLK_EN() RCC->AHB1ENR |= (1 << 6)
#define GPIOH_PCLK_EN() RCC->AHB1ENR |= (1 << 7)
#define GPIOI_PCLK_EN() RCC->AHB1ENR |= (1 << 8)
#define GPIOJ_PCLK_EN() RCC->AHB1ENR |= (1 << 9)
#define GPIOK_PCLK_EN() RCC->AHB1ENR |= (1 << 10)

#define GPIOA_PCLK_DI() RCC->AHB1ENR &= ~(1 << 0)
#define GPIOB_PCLK_DI() RCC->AHB1ENR &= ~(1 << 1)
#define GPIOC_PCLK_DI() RCC->AHB1ENR &= ~(1 << 2)
#define GPIOD_PCLK_DI() RCC->AHB1ENR &= ~(1 << 3)
#define GPIOE_PCLK_DI() RCC->AHB1ENR &= ~(1 << 4)
#define GPIOF_PCLK_DI() RCC->AHB1ENR &= ~(1 << 5)
#define GPIOG_PCLK_DI() RCC->AHB1ENR &= ~(1 << 6)
#define GPIOH_PCLK_DI() RCC->AHB1ENR &= ~(1 << 7)
#define GPIOI_PCLK_DI() RCC->AHB1ENR &= ~(1 << 8)
#define GPIOJ_PCLK_DI() RCC->AHB1ENR &= ~(1 << 9)
#define GPIOK_PCLK_DI() RCC->AHB1ENR &= ~(1 << 10)


//	I2C
#define I2C1_PCLK_EN() RCC->APB1ENR |= (1 << 21)
#define I2C1_PCLK_EN() RCC->APB1ENR &= ~(1 << 21)

//	SPI
#define SPI1_PCLK_EN() RCC->APB2ENR |= (1 << 21)
#define SPI1_PCLK_EN() RCC->APB2ENR &= ~(1 << 21)

//	SysCfg
#define SYSCFG_PCLK_EN() RCC->APB2ENR |= (1 << 14)
#define SYSCFG_PCLK_DI() RCC->APB2ENR &= ~(1 << 14)



#endif /* INC_STM32F469XX_H_ */
