/*
 * smt32f401xx_gpio_driver.h
 *
 *  Created on: Aug 29, 2025
 *      Author: PC RYZEN 5
 */

#ifndef INC_STM32F401XX_GPIO_DRIVER_H_
#define INC_STM32F401XX_GPIO_DRIVER_H_

#include "stm32f401xx.h"

#include <stdint.h>

/*
 * This is a configuration structure for a GPIO pin
 */
typedef struct
{
	uint8_t GPIO_PinNumber;			/* Possible values from @GPIO_PIN_NUMBERS */
	uint8_t GPIO_PinMode;			/* Possible values from @GPIO_PIN_MODES */
	uint8_t GPIO_PinSpeed;			/* Possible values from @GPIO_PIN_SPEED */
	uint8_t GPIO_PinPuPdControl;	/* Possible values from @GPIO_PIN_PU_PD_MODES */
	uint8_t GPIO_PinOutType;		/* Possible values from @GPIO_PIN_OUTPUT_TYPES */
	uint8_t GPIO_PinAltFunMode;		/* Possible values from @GPIO_PIN_ALT_FUNCTIONS */
}GPIO_PinConfig_t;

/*
 * This is a handle structure for GPIO pin
 */
typedef struct
{
	GPIO_RegDef_t 		*pGPIOx;		/* This holds the base address of the GPIO port which the pins belongs */
	GPIO_PinConfig_t 	GPIO_PinConfig;	/* This holds GPIO pin configuration settings */
}GPIO_Handle_t;

/*
 * @GPIO_PIN_NUMBERS
 * GPIO pin numbers
 */
#define GPIO_PIN_NUM_0					0
#define GPIO_PIN_NUM_1					1
#define GPIO_PIN_NUM_2					2
#define GPIO_PIN_NUM_3					3
#define GPIO_PIN_NUM_4					4
#define GPIO_PIN_NUM_5					5
#define GPIO_PIN_NUM_6					6
#define GPIO_PIN_NUM_7					7
#define GPIO_PIN_NUM_8					8
#define GPIO_PIN_NUM_9					9
#define GPIO_PIN_NUM_10					10
#define GPIO_PIN_NUM_11					11
#define GPIO_PIN_NUM_12					12
#define GPIO_PIN_NUM_13					13
#define GPIO_PIN_NUM_14					14
#define GPIO_PIN_NUM_15					15

/*
 * @GPIO_PIN_MODES
 * GPIO pin possible modes
 */
#define GPIO_MODE_INPUT					0
#define GPIO_MODE_OUTPUT				1
#define GPIO_MODE_ALTERNATE_FN			2
#define GPIO_MODE_ANALOG				3
#define GPIO_MODE_EXTI_FALLING_TRIGGER	4
#define GPIO_MODE_EXTI_RISING_TRIGGER	5
#define GPIO_MODE_EXTI_RISING_FALLING	6

/*
 * @GPIO_PIN_OUTPUT_TYPES
 * GPIO pin possible output types
 */
#define GPIO_OUTPUT_TYPE_PUSH_PULL		0
#define GPIO_OUTPUT_TYPE_OPEN_DRAIN		1

/*
 * @GPIO_PIN_SPEED
 * GPIO pin possible output speeds
 */
#define GPIO_OUTPUT_SPEED_LOW			0
#define GPIO_OUTPUT_SPEED_MEDIUM		1
#define GPIO_OUTPUT_SPEED_HIGH			2
#define GPIO_OUTPUT_SPEED_VERY_HIGH		3

/*
 * @GPIO_PIN_PU_PD_MODES
 * GPIO pin pull-up and pull-down modes
 */
#define GPIO_PIN_NO_PULLUP_PULLDOWN		0
#define GPIO_PIN_PULL_UP				1
#define GPIO_PIN_PULL_DOWN				2

/************************************************************************************************
 *
 * APIs supported by this driver
 * For more information about the APIs check the function definitions
 *
 *************************************************************************************************/

/*
 * Peripheral clock setup
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi);

/*
 * Init and De-init
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

/*
 * Read and write
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

/*
 * IRQ Configuration and ISR handling
 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);
void GPIO_IRQHandling(uint8_t PinNumber);



#endif /* INC_STM32F401XX_GPIO_DRIVER_H_ */
