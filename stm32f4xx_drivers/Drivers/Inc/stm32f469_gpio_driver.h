/*
 * stm32f469_gpio_driver.h
 *
 *  Created on: 18 Feb 2020
 *      Author: eriks
 */

#ifndef INC_STM32F469_GPIO_DRIVER_H_
#define INC_STM32F469_GPIO_DRIVER_H_

#include "stm32f469xx.h"
#include <stdint.h>

//	Pin configuration
typedef struct {
	uint8_t GPIO_PinNumber;
	uint8_t GPIO_PinMode;
	uint8_t GPIO_PinSpeed;
	uint8_t GPIO_PinPuPdControl;
	uint8_t GPIO_PinAltFunMode;
} GPIO_PinConfig_t;

//	Holds both the configuration, and port
typedef struct {
	GPIO_RegDef_t *pGPIOx;
	GPIO_PinConfig_t GPIO_PinConfig;
} GPIO_Handle;


/****************************************************************
 * 								API
 ****************************************************************/

//	Peripheral clock setup
void GPIO_PeriClockControl(GPIO_RegDef_t*, uint8_t enDi);

//	Init / deinit
void GPIO_Init(GPIO_Handle*);
void GPIO_DeInit(GPIO_RegDef_t*);

//	Read / write
uint8_t GPIO_ReadPin(GPIO_RegDef_t*, uint8_t pinNumber);
uint16_t GPIO_ReadPort(void);
void GPIO_WriteToPin(void);
void GPIO_WriteTiPort(void);
void togglePin(void);

//	Interrupts
void GPIO_IRQConfig(void);





#endif /* INC_STM32F469_GPIO_DRIVER_H_ */
