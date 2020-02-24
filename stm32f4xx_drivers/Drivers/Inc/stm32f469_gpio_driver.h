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

/****************************************************************
 * 							MACROS
 ****************************************************************/

/*
 * @GPIO_PIN_MODES
 */
#define GPIO_MODE_IN			0
#define GPIO_MODE_OUT			1
#define GPIO_MODE_AF			2
#define GPIO_MODE_ANALOG		3

#define GPIO_MODE_IR_FT			4	//	Falling edge trigger
#define GPIO_MODE_IR_RT			5	//	Rising edge trigger
#define GPIO_MODE_IR_RFT		6	//	Rising, falling edge trigger

/*
 * @GPIO_OP_TYPES
 */
#define GPIO_OP_TYPE_PP			0	//	Push-pull
#define GPIO_OP_TYPE_OD			1	//	Open drain

/*
 * @GPIO_OP_SPEEDS
 */
#define GPIO_SPEED_LOW			0
#define GPIO_SPEED_MEDIUM		1
#define GPIO_SPEED_HIGH			2
#define GPIO_SPEED_VHIGH		3

/*
 * @GPIO_PUPD
 */
#define GPIO_PUPD_NONE				0
#define GPIO_PUPD_PU				1
#define GPIO_PUPD_PD				2

/*
 * @GPIO_PINS
 */
//#define GPIO_PIN_0				0x01
//#define GPIO_PIN_1				0x02
//#define GPIO_PIN_2				0x04
//#define GPIO_PIN_3				0x08
//#define GPIO_PIN_4				0x10
//#define GPIO_PIN_5				0x20
//#define GPIO_PIN_6				0x40
//#define GPIO_PIN_7				0x80
//#define GPIO_PIN_8				0x100
//#define GPIO_PIN_9				0x200
//#define GPIO_PIN_10				0x400
//#define GPIO_PIN_11				0x800
//#define GPIO_PIN_12				0x1000
//#define GPIO_PIN_13				0x2000
//#define GPIO_PIN_14				0x4000
//#define GPIO_PIN_15				0x8000

#define GPIO_PIN_0				0
#define GPIO_PIN_1				1
#define GPIO_PIN_2				2
#define GPIO_PIN_3				3
#define GPIO_PIN_4				4
#define GPIO_PIN_5				5
#define GPIO_PIN_6				6
#define GPIO_PIN_7				7
#define GPIO_PIN_8				8
#define GPIO_PIN_9				9
#define GPIO_PIN_10				10
#define GPIO_PIN_11				11
#define GPIO_PIN_12				12
#define GPIO_PIN_13				13
#define GPIO_PIN_14				14
#define GPIO_PIN_15				15


/****************************************************************
 * 							TYPEDEFS
 ****************************************************************/

//	Pin configuration
typedef struct {
	uint16_t GPIO_pin;				// @GPIO_PINS
	uint8_t GPIO_PinMode;			// @GPIO_PIN_MODES
	uint8_t GPIO_PinSpeed;			// @GPIO_OUTPUT_SPEEDS
	uint8_t GPIO_PinPuPdControl;	// @GPIO_PUPD
	uint8_t GPIO_PinOPType;			// @GPIO_OP_TYPES
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
uint8_t GPIO_ReadPin(GPIO_RegDef_t*, uint16_t pin);
uint16_t GPIO_ReadPort(GPIO_RegDef_t*);
void GPIO_WriteToPin(GPIO_RegDef_t*, uint16_t pin, uint8_t value);
void GPIO_WriteToPort(GPIO_RegDef_t*, uint16_t value);
void GPIO_TogglePin(GPIO_RegDef_t*, uint16_t pin);

//	Interrupts
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t enDi);
void GPIO_IRQHandling(uint16_t pin);





#endif /* INC_STM32F469_GPIO_DRIVER_H_ */
