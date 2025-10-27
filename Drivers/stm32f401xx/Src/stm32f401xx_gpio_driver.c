/*
 * stm32f401xx_gpio_driver.c
 *
 *  Created on: Aug 29, 2025
 *      Author: PC RYZEN 5
 */

#include "stm32f401xx_gpio_driver.h"

/**
 * @fn			- GPIO_PeriClockControl
 *
 * @brief		- Enables or disables peripheral clock for the given GPIO port
 *
 * @param		- pGPIOx: Pointer to the base address of the GPIO peripheral
 * @param		- EnorDi: Either enable or disable the GPIO peripheral, use ENABLE or DISABLE macros
 *
 * @return		- None
 *
 * @note		- None
 *
 **/
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi)
{
	if (EnorDi == ENABLE){
		if 	    (pGPIOx == GPIOA) GPIOA_PERI_CLK_EN();
		else if (pGPIOx == GPIOB) GPIOB_PERI_CLK_EN();
		else if (pGPIOx == GPIOC) GPIOC_PERI_CLK_EN();
		else if (pGPIOx == GPIOD) GPIOD_PERI_CLK_EN();
		else if (pGPIOx == GPIOE) GPIOE_PERI_CLK_EN();
		else if (pGPIOx == GPIOH) GPIOH_PERI_CLK_EN();
	} else {
		if 	    (pGPIOx == GPIOA) GPIOA_PERI_CLK_DI();
		else if (pGPIOx == GPIOB) GPIOB_PERI_CLK_DI();
		else if (pGPIOx == GPIOC) GPIOC_PERI_CLK_DI();
		else if (pGPIOx == GPIOD) GPIOD_PERI_CLK_DI();
		else if (pGPIOx == GPIOE) GPIOE_PERI_CLK_DI();
		else if (pGPIOx == GPIOH) GPIOH_PERI_CLK_DI();
	}
}

/**
 * @fn			- GPIO_Init
 *
 * @brief		- Initializes and configures the  given GPIO port
 *
 * @param		- pGPIOHandle: Pointer to the handle of GPIO port configuration
 *
 * @return		- None
 *
 * @note		- None
 *
 **/
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	uint32_t temp = 0;	// temporal register

	// Enable the peripheral clock
	GPIO_PeriClockControl(pGPIOHandle->pGPIOx, ENABLE);

	// 1. Configure the mode of gpio pin
	if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG) {
		temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		pGPIOHandle->pGPIOx->MODER &= ~(0x03 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); // Clearing the 2 bits
		pGPIOHandle->pGPIOx->MODER |= temp;	// Setting
	} else {
		// interrupt mode
		// 1. Configure interrupt trigger type: falling, rising or both of them
		if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_EXTI_FALLING_TRIGGER) {
			EXTI->RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);	// Clear the RTSR bit
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);	// Set the FTSR bit
		} else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_EXTI_RISING_TRIGGER){
			EXTI->FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);	// Clear the FTSR bit
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);	// Set the RTSR bit
		} else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_EXTI_RISING_FALLING){
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);	// Set the FTSR bit
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);	// Set the RTSR bit
		}

		// 2. Configure the interrupt GPIO port source in SYSCFG_EXTI
		uint8_t reg_index = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;
		uint8_t pin_index = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;
		uint8_t port_code = GPIO_BASEADDR_TO_INT_SRC_PORT_CODE(pGPIOHandle->pGPIOx);
		SYSCFG_PERI_CLK_EN();
		SYSCFG->EXTICR[reg_index] = port_code << (pin_index * 4);

		// 3. Enable the EXTI interrupt delivery using the Interrupt Mask Register IMR
		EXTI->IMR |= 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber;
	}
	//temp = 0;

	// 2. Configure the speed
	temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x03 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); // Clearing the 2 bits
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;
	//temp = 0;

	// 3. Configure the pull-up and pull-down settings
	temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x03 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); // Clearing the 2 bits
	pGPIOHandle->pGPIOx->PUPDR |= temp;
	//temp = 0;

	// 4. Configure the port output type
	if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_OUTPUT) {
		temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinOutType << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber;
		pGPIOHandle->pGPIOx->OTYPER &= ~(0x01 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); // Clearing the bit
		pGPIOHandle->pGPIOx->OTYPER |= temp;
		//temp = 0;
	}

	// 5. Configure the alternate functionality
	if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTERNATE_FN) {
		// Configure the alternate function registers
		//uint8_t reg_index = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
		uint8_t pin_index = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;
		if (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber <= 7) {
			pGPIOHandle->pGPIOx->AFRL &= ~(0x0f << (4 * pin_index)); // Clearing the 4 bits
			pGPIOHandle->pGPIOx->AFRL |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * pin_index));
		} else {
			pGPIOHandle->pGPIOx->AFRH &= ~(0x0f << (4 * pin_index)); // Clearing the 4 bits
			pGPIOHandle->pGPIOx->AFRH |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * pin_index));
		}
	}
}

/**
 * @fn			- GPIO_DeInit
 *
 * @brief		- Resets the given GPIO port to the default state
 *
 * @param		- pGPIOx: Pointer to the base address of the GPIO peripheral
 *
 * @return		- None
 *
 * @note		- None
 *
 **/
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
	if 	    (pGPIOx == GPIOA) GPIOA_REG_RESET();
	else if (pGPIOx == GPIOB) GPIOB_REG_RESET();
	else if (pGPIOx == GPIOC) GPIOC_REG_RESET();
	else if (pGPIOx == GPIOD) GPIOD_REG_RESET();
	else if (pGPIOx == GPIOE) GPIOE_REG_RESET();
	else if (pGPIOx == GPIOH) GPIOH_REG_RESET();
}

/**
 * @fn			- GPIO_ReadFromInputPin
 *
 * @brief		- This function reads the current value from the given GPIO pin
 *
 * @param		- pGPIOx: Pointer to the base address of the GPIO peripheral
 * @param		- PinNumber: Pin number of the given GPIO port
 *
 * @return		- 0 the GPIO input level is 0
 * 				- 1 the GPIO input level is 1
 *
 * @note		- None
 *
 **/
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	uint8_t value = (uint8_t)((pGPIOx->IDR >> PinNumber) & 0x00000001);
	return value;
}

/**
 * @fn			- GPIO_ReadFromInputPort
 *
 * @brief		- This function reads the current value from the given GPIO port
 *
 * @param		- pGPIOx: Pointer to the base address of the GPIO peripheral
 *
 * @return		- The current value of the given GPIO port, 16 bits
 *
 * @note		- None
 *
 **/
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	uint16_t value = (uint16_t)pGPIOx->IDR;
	return value;
}

/**
 * @fn			- GPIO_WriteToOutputPin
 *
 * @brief		- GPIO set output pin level
 *
 * @param		- pGPIOx: Pointer to the base address of the GPIO peripheral
 * @param		- PinNumber: Pin number of the given GPIO port
 * @param		- value: Output level. 0: low ; 1: high
 *
 * @return		- None
 *
 * @note		- None
 *
 **/
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t value)
{
	if (value == GPIO_PIN_SET) {
		pGPIOx->ODR |= (1 << PinNumber);
	} else {
		pGPIOx->ODR &= ~(1 << PinNumber);
	}
}

/**
 * @fn			- GPIO_WriteToOutputPort
 *
 * @brief		- GPIO set output port level
 *
 * @param		- pGPIOx: Pointer to the base address of the GPIO peripheral
 * @param		- value: Output level, 16 bits, 0: low ; 1: high
 *
 * @return		- None
 *
 * @note		- None
 *
 **/
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t value)
{
	pGPIOx->ODR = value;
}

/**
 * @fn			- GPIO_ToggleOutputPin
 *
 * @brief		- Toggle output level
 *
 * @param		- pGPIOx: Pointer to the base address of the GPIO peripheral
 * @param		- PinNumber: GPIO pin number
 *
 * @return		- None
 *
 * @note		- None
 *
 **/
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	pGPIOx->ODR ^= (1 << PinNumber);
}

/**
 * @fn			- GPIO_IRQInterruptConfig
 *
 * @brief		- Enables or disables an interrupt
 *
 * @param		- IRQNumber: Number or position of the interrupt in the NVIC Vector Table (0-84)
 * @param		- EnorDi: Either enable or disable an interrupt, use ENABLE or DISABLE macros
 *
 * @return		- None
 *
 * @note		- None
 *
 **/
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
	if (EnorDi == ENABLE){
		if (IRQNumber <= 31) {
			// 0 - 31, ISER0 register
			*NVIC_ISER0 |= (1 << IRQNumber);
		} else if (IRQNumber > 31 && IRQNumber < 64) {
			// 32 - 63, ISER1 register
			*NVIC_ISER1 |= (1 << (IRQNumber % 32));
		} else if (IRQNumber >= 64 && IRQNumber < 96) {
			// 64 - 95, ISER2 register
			*NVIC_ISER2 |= (1 << (IRQNumber % 32));
		}
	} else {
		if (IRQNumber <= 31) {
			// 0 - 31, ICER0 register
			*NVIC_ICER0 |= (1 << IRQNumber);
		} else if (IRQNumber > 31 && IRQNumber < 64) {
			// 32 - 63, ICER1 register
			*NVIC_ICER1 |= (1 << (IRQNumber % 32));
		} else if (IRQNumber >= 64 && IRQNumber < 96) {
			// 64 - 95, ICER2 register
			*NVIC_ICER2 |= (1 << (IRQNumber % 32));
		}
	}
}

/**
 * @fn			- GPIO_IRQPriorityConfig
 *
 * @brief		- Configures the interrupt request priority level
 *
 * @param		- IRQNumber: Number or position of the interrupt in the NVIC Vector Table (0-84)
 * @param		- IRQPriority: Priority level (0 - 15), the less the level the higher the priority
 *
 * @return		- None
 *
 * @note		- None
 *
 **/
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority) {
	// 1. Lets find out the IPR register
	uint8_t ipr_reg_index = IRQNumber / 4;	// Fields per register = 4, NVIC_IPR_index = IRQNumber / Sections per register
	uint8_t ipr_field_index = IRQNumber % 4;
	uint8_t ipr_field_bits_lenght = 8;

	// We need to shit 4 bits more because each ipr field implements the priority level in the the 4 msb, the 4 lsb are not implemented
	uint8_t shift_amount = (ipr_field_bits_lenght * ipr_field_index) + (8 - PRIORITY_BITS_IMPLEMENTED_NUM);
	*(NVIC_PRIORITY_BASE_ADDRESS + ipr_reg_index) |= IRQPriority << shift_amount;
	/*uint32_t ipr_value = IRQPriority << shift_amount;
	volatile uint32_t *ipr_reg_address = NVIC_PRIORITY_BASE_ADDRESS + ipr_reg_index;
	*ipr_reg_address |= ipr_value;*/
}

/**
 * @fn			- GPIO_IRQHandling
 *
 * @brief		- Clears the EXTI Pending Register corresponding to the pin number
 *
 * @param		- PinNumber: GPIO pin number attached to the EXTI line
 *
 * @return		- None
 *
 * @note		- None
 *
 **/
void GPIO_IRQHandling(uint8_t PinNumber)
{
	if (EXTI->PR & (1 << PinNumber)) {	// This bit is set when the selected edge event arrives on the external interrupt line.
		EXTI->PR |= (1 << PinNumber);	// This bit is cleared by programming it to ‘1’.
	}
}
