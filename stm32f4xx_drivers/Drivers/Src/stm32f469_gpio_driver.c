/*
 * stm32f469_gpio_driver.c
 *
 *  Created on: 18 Feb 2020
 *      Author: eriks
 */

#include "stm32f469_gpio_driver.h"


//	Peripheral clock setup

/*
 * @fn 			- GPIO_PeriClockControl
 *
 * @brief 		- Enables or disables the clock for given GPIO port
 *
 * @param[in]	- base address of the GPIO peripheral
 * @param[in]	- Enable or disable the clock (ENABLE or DISABLE macros)
 *
 * @return 		- none
 *
 * @Note 		- none
 */
void GPIO_PeriClockControl(GPIO_RegDef_t* port, uint8_t enDi) {

	if (enDi) {
		if (port == GPIOA) {
			GPIOA_PCLK_EN();
		} else if (port == GPIOB) {
			GPIOB_PCLK_EN();
		} else if (port == GPIOC) {
			GPIOC_PCLK_EN();
		} else if (port == GPIOD) {
			GPIOD_PCLK_EN();
		} else if (port == GPIOE) {
			GPIOE_PCLK_EN();
		} else if (port == GPIOF) {
			GPIOF_PCLK_EN();
		} else if (port == GPIOG) {
			GPIOG_PCLK_EN();
		} else if (port == GPIOH) {
			GPIOH_PCLK_EN();
		} else if (port == GPIOI) {
			GPIOI_PCLK_EN();
		} else if (port == GPIOJ) {
			GPIOJ_PCLK_EN();
		} else if (port == GPIOK) {
			GPIOK_PCLK_EN();
		}
	} else {
		if (port == GPIOA) {
			GPIOA_PCLK_DI();
		} else if (port == GPIOB) {
			GPIOB_PCLK_DI();
		} else if (port == GPIOC) {
			GPIOC_PCLK_DI();
		} else if (port == GPIOD) {
			GPIOD_PCLK_DI();
		} else if (port == GPIOE) {
			GPIOE_PCLK_DI();
		} else if (port == GPIOF) {
			GPIOF_PCLK_DI();
		} else if (port == GPIOG) {
			GPIOG_PCLK_DI();
		} else if (port == GPIOH) {
			GPIOH_PCLK_DI();
		} else if (port == GPIOI) {
			GPIOI_PCLK_DI();
		} else if (port == GPIOJ) {
			GPIOJ_PCLK_DI();
		} else if (port == GPIOK) {
			GPIOK_PCLK_DI();
		}
	}

}

////////////////////	Init / deinit	//////////////////////////

/*
 * @fn 			- GPIO_Init
 *
 * @brief 		- Initialize the GPIO port
 *
 * @param[in]	- Init handle
 *
 * @return 		- none
 *
 * @Note 		- none
 */
void GPIO_Init(GPIO_Handle* pHandle) {

	uint32_t temp;

	//	GPIO Input/output mode
	if (pHandle->GPIO_PinConfig.GPIO_PinMode < 4) {
		temp = (pHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pHandle->GPIO_PinConfig.GPIO_pin));
		pHandle->pGPIOx->MODER |= temp;

	//	GPIO Interrupts
	//	Config EXTI ftsr and rtsr registers
	} else {

		//	Interrupt: falling edge trigger detection
		if (pHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IR_FT){
			EXTI->FTSR |= (1 << pHandle->GPIO_PinConfig.GPIO_pin);
			EXTI->RTSR &= ~(1 << pHandle->GPIO_PinConfig.GPIO_pin);

		//	Interrupt: Rising edge trigger detection
		} else if (pHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IR_RT){
			EXTI->RTSR |= (1 << pHandle->GPIO_PinConfig.GPIO_pin);
			EXTI->FTSR &= ~(1 << pHandle->GPIO_PinConfig.GPIO_pin);

		//	Interrupt: Rising/Falling edge trigger detection
		} else if (pHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IR_RFT){
			EXTI->FTSR |= (1 << pHandle->GPIO_PinConfig.GPIO_pin);
			EXTI->RTSR |= (1 << pHandle->GPIO_PinConfig.GPIO_pin);
		}

		//	Enable SYSCFG peripheral clock
		SYSCFG_PCLK_EN();

		//	Config selected GPIO port for exti in SYSCFG register
		uint8_t tmpRegNr = pHandle->GPIO_PinConfig.GPIO_pin / 4;
		uint8_t tmpPos = pHandle->GPIO_PinConfig.GPIO_pin % 4;
		SYSCFG->EXTICR[tmpRegNr] = 	GPIOx_TO_EXTICFG(pHandle->pGPIOx) << (tmpPos * 4);

		//	Enable exti interrupt delivery in IMR register
		EXTI->IMR |= (1 << pHandle->GPIO_PinConfig.GPIO_pin);


	}

	//	Config speed
	temp = (pHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pHandle->GPIO_PinConfig.GPIO_pin));
	pHandle->pGPIOx->OSPEEDR |= temp;

	//	Config pull-up, pull-down
	temp = (pHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pHandle->GPIO_PinConfig.GPIO_pin));
	pHandle->pGPIOx->PUPDR |= temp;

	//	config ouput type
	temp = (pHandle->GPIO_PinConfig.GPIO_PinOPType << (pHandle->GPIO_PinConfig.GPIO_pin));
	pHandle->pGPIOx->OTYPER |= temp;

	//	config AF
}

void GPIO_DeInit(GPIO_RegDef_t* pGPIO){
	pGPIO->BSRR;						//	  TODO
}

//	Read / write
uint8_t GPIO_ReadPin(GPIO_RegDef_t* pGPIO, uint16_t pin) {
	return ( (pGPIO->IDR & (1 << pin)) != 0 );
}

uint16_t GPIO_ReadPort(GPIO_RegDef_t* pGPIO) {
	return ( pGPIO->IDR );
}

void GPIO_WriteToPin(GPIO_RegDef_t* pGPIO, uint16_t pin, uint8_t value) {

	uint32_t temp = 1 << pin;

	if (value == GPIO_PIN_SET) {
		pGPIO->ODR |= temp;
	} else {
		pGPIO->ODR &= (~temp);
	}
}

void GPIO_WriteToPort(GPIO_RegDef_t* pGPIO, uint16_t value){
	pGPIO->ODR = value;
}

void GPIO_TogglePin(GPIO_RegDef_t* pGPIO, uint16_t pin){

	pGPIO->ODR ^= (1 << pin);

}

//	Interrupts
void GPIO_IRQConfig(IRQn_Type IRQ, uint8_t IRQPriority, uint8_t enDi){

	uint8_t tmpRegNr = IRQ / 32;
	uint8_t tmpRegPos = IRQ % 32;


	if (enDi == ENABLE) {
		NVIC_ISER_REG[tmpRegNr] |= (1 << tmpRegPos);
	}


}

void GPIO_IRQHandling(uint16_t pin){

}
