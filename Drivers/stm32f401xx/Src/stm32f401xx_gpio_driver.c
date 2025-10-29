/*
 * stm32f401xx_gpio_driver.c
 *
 *  Created on: Aug 29, 2025
 *      Author: David Yepes
 */

#include "stm32f401xx_gpio_driver.h"

/**
 * @fn			gpio_peri_clock_control
 *
 * @brief		Internal function to get the IRQ number from the GPIO pin number
 *
 * @param		p_gpio_handle Pointer to the handle of GPIO pin.
 *
 * @return		IRQ number
 *
 * @note		None
 *
 **/
static uint8_t gpio_get_irq_number(const gpio_handle_t *p_gpio_handle) {
	gpio_pin_number_t pin_number = p_gpio_handle->gpio_pin_config.gpio_pin_number;
	if (pin_number <= 4) {
		uint8_t irq_array[] = {IRQ_NO_EXTI0, IRQ_NO_EXTI1, IRQ_NO_EXTI2, IRQ_NO_EXTI3, IRQ_NO_EXTI4};
		return irq_array[pin_number];
	} else if (pin_number <= 9) {
		return IRQ_NO_EXTI9_5;
	} else {
		return IRQ_NO_EXTI15_10;
	}
}

/**
 * @fn			gpio_peri_clock_control
 *
 * @brief		Enables or disables peripheral clock for the given GPIO port
 *
 * @param		p_gpio_x Pointer to the base address of the GPIO peripheral
 * @param		en_or_di Either enable or disable the GPIO peripheral, use ENABLE or DISABLE macros
 *
 * @return		None
 *
 * @note		None
 *
 **/
void gpio_peri_clock_control(const gpio_reg_def_t *p_gpio_x, uint8_t en_or_di)
{
	if (en_or_di == ENABLE){
		if 	    (p_gpio_x == GPIOA) GPIOA_PERI_CLK_EN();
		else if (p_gpio_x == GPIOB) GPIOB_PERI_CLK_EN();
		else if (p_gpio_x == GPIOC) GPIOC_PERI_CLK_EN();
		else if (p_gpio_x == GPIOD) GPIOD_PERI_CLK_EN();
		else if (p_gpio_x == GPIOE) GPIOE_PERI_CLK_EN();
		else if (p_gpio_x == GPIOH) GPIOH_PERI_CLK_EN();
	} else {
		if 	    (p_gpio_x == GPIOA) GPIOA_PERI_CLK_DI();
		else if (p_gpio_x == GPIOB) GPIOB_PERI_CLK_DI();
		else if (p_gpio_x == GPIOC) GPIOC_PERI_CLK_DI();
		else if (p_gpio_x == GPIOD) GPIOD_PERI_CLK_DI();
		else if (p_gpio_x == GPIOE) GPIOE_PERI_CLK_DI();
		else if (p_gpio_x == GPIOH) GPIOH_PERI_CLK_DI();
	}
}

/**
 * @fn			gpio_init
 *
 * @brief		Initializes and configures the  given GPIO port.
 *
 * @param		p_gpio_handle Pointer to the handle of GPIO pin.
 * @param       p_gpio_x Pointer to the base address of the GPIO peripheral, possible values: 
 * 						 GPIOA, GPIOB, GPIOC, GPIOD, GPIOE o GPIOH.
 * @param		gpio_pin_config This structure holds GPIO pin configuration settings. 
 *
 * @return		None
 *
 * @note		Provided that all the gpio pins share the same configuration registers it makes sense
 * 				to use the ALL_BITS variable of each register instead of the bit fields in order to have
 * 				a simpler configuration process. To do that we need to use the next bitwise operations:
 * 
 * 				1. Clear the specific field bits using an inverted mask shifted to the position of 
 * 				   the field in the register: 
 * 
 * 				   REGISTER.ALLBITS &= ~( mask_value << (number_of_bits_of_the_field * pin_number) )
 * 
 * 				   mask_value = (2 exp number_of_bits_of_the_field) - 1. i.e: bits=1 mask_value=0x01 
 * 				   													          bits=2 mask_value=0x03
 * 																			  bits=3 mask_value=0x07
 * 
 * 			    2. Set the specific field bits using the desired value shifted to the position of
 * 				   the field in the register:
 *				   
 *	               REGISTER.ALLBITS |= ( desired_value << (number_of_bits_of_the_field * pin_number) )
 *				   
 **/
void gpio_init(gpio_handle_t *p_gpio_handle, gpio_reg_def_t *const p_gpio_x, const gpio_pin_config_t *gpio_pin_config)
{	
	// Enable the peripheral clock
	gpio_peri_clock_control(p_gpio_x, ENABLE);

	// 1. Configure the mode of gpio pin
	if (gpio_pin_config->gpio_pin_mode <= GPIO_MODE_ANALOG) {
		p_gpio_x->MODER.ALLBITS &= ~( 0x03 << (2 * gpio_pin_config->gpio_pin_number) );	// Clear the 2 bits of the MODER field
		p_gpio_x->MODER.ALLBITS |= ( gpio_pin_config->gpio_pin_mode << (2 * gpio_pin_config->gpio_pin_number) );	// Set the MODER field value
	} else {
		// Interrupt mode
		// 1. Configure interrupt trigger type: falling, rising or both of them
		if (gpio_pin_config->gpio_pin_mode == GPIO_MODE_EXTI_FALLING_TRIGGER) {
			EXTI->RTSR &= ~(1 << gpio_pin_config->gpio_pin_number);	// Clear the RTSR bit
			EXTI->FTSR |= (1 << gpio_pin_config->gpio_pin_number);	// Set the FTSR bit
		} else if (gpio_pin_config->gpio_pin_mode == GPIO_MODE_EXTI_RISING_TRIGGER){
			EXTI->FTSR &= ~(1 << gpio_pin_config->gpio_pin_number);	// Clear the FTSR bit
			EXTI->RTSR |= (1 << gpio_pin_config->gpio_pin_number);	// Set the RTSR bit
		} else if (gpio_pin_config->gpio_pin_mode == GPIO_MODE_EXTI_RISING_FALLING){
			EXTI->FTSR |= (1 << gpio_pin_config->gpio_pin_number);	// Set the FTSR bit
			EXTI->RTSR |= (1 << gpio_pin_config->gpio_pin_number);	// Set the RTSR bit
		}

		// 2. Configure the interrupt GPIO port source in SYSCFG_EXTI
		uint8_t reg_index = gpio_pin_config->gpio_pin_number / 4;
		uint8_t pin_index = gpio_pin_config->gpio_pin_number % 4;
		uint8_t port_code = GPIO_BASEADDR_TO_INT_SRC_PORT_CODE(p_gpio_x);
		SYSCFG_PERI_CLK_EN();
		SYSCFG->EXTICR[reg_index] = port_code << (pin_index * 4);	// Set the EXTICR register

		// 3. Enable the EXTI interrupt delivery using the Interrupt Mask Register IMR
		EXTI->IMR |= 1 << gpio_pin_config->gpio_pin_number;			// Set IMR bit
	}

	// 2. Configure the speed
	p_gpio_x->OSPEEDR.ALLBITS &= ~( 0x03 << (2 * gpio_pin_config->gpio_pin_number) ); 							// Clear the 2 bits of the OSPEEDR field
	p_gpio_x->OSPEEDR.ALLBITS |= ( gpio_pin_config->gpio_pin_speed << (2 * gpio_pin_config->gpio_pin_number) );	// Set the OSPEEDR field value 

	// 3. Configure the pull-up and pull-down settings
	p_gpio_x->PUPDR.ALLBITS &= ~( 0x03 << (2 * gpio_pin_config->gpio_pin_number) );										// Clear the 2 bits of the PUPDR field
	p_gpio_x->PUPDR.ALLBITS |= ( gpio_pin_config->gpio_pin_pu_pd_control << (2 * gpio_pin_config->gpio_pin_number) );	// Set the PUPDR field value 

	// 4. Configure the port output type
	if (gpio_pin_config->gpio_pin_mode == GPIO_MODE_OUTPUT) {
		p_gpio_x->OTYPER.ALLBITS &= ~( 0x01 << gpio_pin_config->gpio_pin_number );								// Clear the bit of the OTYPER field
		p_gpio_x->OTYPER.ALLBITS |= ( gpio_pin_config->gpio_pin_out_type << gpio_pin_config->gpio_pin_number );	// Set the OTYPER field value 
	}

	// 5. Configure the alternate functionality
	if (gpio_pin_config->gpio_pin_mode == GPIO_MODE_ALTERNATE_FN) {
		// Configure the alternate function registers
		//uint8_t reg_index = gpio_pin_config->gpio_pin_number / 8;
		uint8_t pin_index = gpio_pin_config->gpio_pin_number % 8;
		if (gpio_pin_config->gpio_pin_number <= 7) {
			p_gpio_x->AFRL.ALL_BITS &= ~( 0x0f << (4 * pin_index) );									// Clear the 4 bits of the AFRL field
			p_gpio_x->AFRL.ALL_BITS |= ( gpio_pin_config->gpio_pin_alt_fun_mode << (4 * pin_index) );	// Set the AFRL field value
		} else {
			p_gpio_x->AFRH.ALL_BITS &= ~( 0x0f << (4 * pin_index) ); 									// Clear the 4 bits of the AFRH field
			p_gpio_x->AFRH.ALL_BITS |= ( gpio_pin_config->gpio_pin_alt_fun_mode << (4 * pin_index) );	// Set the AFRH field value
		}
	}

	// 6. Store the configuration data into the handle
	p_gpio_handle->p_gpio_x = p_gpio_x;
	p_gpio_handle->gpio_pin_config = *gpio_pin_config;
}

/**
 * @fn			gpio_deinit
 *
 * @brief		Resets the given GPIO port to the default state
 *
 * @param		p_gpio_handle Pointer to the handle of GPIO pin
 *
 * @return		None
 *
 * @note		None
 *
 **/
void gpio_deinit(const gpio_handle_t *p_gpio_handle)
{
	if 	    (p_gpio_handle->p_gpio_x == GPIOA) GPIOA_REG_RESET();
	else if (p_gpio_handle->p_gpio_x == GPIOB) GPIOB_REG_RESET();
	else if (p_gpio_handle->p_gpio_x == GPIOC) GPIOC_REG_RESET();
	else if (p_gpio_handle->p_gpio_x == GPIOD) GPIOD_REG_RESET();
	else if (p_gpio_handle->p_gpio_x == GPIOE) GPIOE_REG_RESET();
	else if (p_gpio_handle->p_gpio_x == GPIOH) GPIOH_REG_RESET();
}

/**
 * @fn			gpio_read_from_input_pin
 *
 * @brief		This function reads the current value from the given GPIO pin
 *
 * @param		p_gpio_handle Pointer to the handle of GPIO pin
 *
 * @return		0 if the GPIO input level is 0 or
 * 				1 if the GPIO input level is 1
 *
 * @note		None
 *
 **/
uint8_t gpio_read_from_input_pin(const gpio_handle_t *p_gpio_handle)
{
	uint8_t value = (uint8_t)( (p_gpio_handle->p_gpio_x->IDR.ALLBITS >> p_gpio_handle->gpio_pin_config.gpio_pin_number) & 0x00000001 );
	return value;
}

/**
 * @fn			gpio_read_from_input_port
 *
 * @brief		This function reads the current value from the given GPIO port
 *
 * @param		p_gpio_x Pointer to the base address of the GPIO peripheral
 *
 * @return		The current value of the given GPIO port, 16 bits
 *
 * @note		None
 *
 **/
uint16_t gpio_read_from_input_port(const gpio_reg_def_t *p_gpio_x)
{
	uint16_t value = (uint16_t)p_gpio_x->IDR.ALLBITS;
	return value;
}

/**
 * @fn			gpio_write_to_output_pin
 *
 * @brief		GPIO set output pin level
 *
 * @param		p_gpio_handle Pointer to the handle of GPIO pin
 * @param		value Output level. 0: low ; 1: high
 *
 * @return		None
 *
 * @note		None
 *
 **/
void gpio_write_to_output_pin(gpio_handle_t *p_gpio_handle, uint8_t value)
{
	if (value == GPIO_PIN_SET) {
		p_gpio_handle->p_gpio_x->ODR.ALLBITS |= (1 << p_gpio_handle->gpio_pin_config.gpio_pin_number);
	} else {
		p_gpio_handle->p_gpio_x->ODR.ALLBITS &= ~(1 << p_gpio_handle->gpio_pin_config.gpio_pin_number);
	}
}

/**
 * @fn			gpio_write_to_output_port
 *
 * @brief		GPIO set output port level
 *
 * @param		p_gpio_x Pointer to the base address of the GPIO peripheral
 * @param		value Output level, 16 bits, 0: low ; 1: high
 *
 * @return		None
 *
 * @note		None
 *
 **/
void gpio_write_to_output_port(gpio_reg_def_t *p_gpio_x, uint16_t value)
{
	p_gpio_x->ODR.ALLBITS = value;
}

/**
 * @fn			gpio_toggle_output_pin
 *
 * @brief		Toggle output level
 *
 * @param		p_gpio_handle Pointer to the handle of GPIO pin
 *
 * @return		None
 *
 * @note		Provided that all the gpio pins share the same output data register it makes sense to use
 * 				the ALL_BITS variable of the ODR register instead of the bit fields. To perform the toggle 
 * 				action we can use the bitwise XOR operation between the current bit state and 1:
 * 				
 * 			    0 XOR 1 = 1, 
 * 				1 XOR 1 = 0
 *
 **/
void gpio_toggle_output_pin(gpio_handle_t *p_gpio_handle)
{
	p_gpio_handle->p_gpio_x->ODR.ALLBITS ^= (1 << p_gpio_handle->gpio_pin_config.gpio_pin_number);
}

/**
 * @fn			gpio_irq_interrupt_config
 *
 * @brief		Enables or disables an interrupt
 *
 * @param		irq_number Number or position of the interrupt in the NVIC Vector Table (0-84)
 * @param		en_or_di Either enable or disable an interrupt, use ENABLE or DISABLE macros
 *
 * @return		None
 *
 * @note		None
 *
 **/
void gpio_irq_interrupt_config(const gpio_handle_t *p_gpio_handle, uint8_t en_or_di)
{
	uint8_t irq_number = gpio_get_irq_number(p_gpio_handle);

	if (en_or_di == ENABLE){
		if (irq_number <= 31) {
			// 0 - 31, ISER0 register
			*NVIC_ISER0 |= (1 << irq_number);
		} else if (irq_number > 31 && irq_number < 64) {
			// 32 - 63, ISER1 register
			*NVIC_ISER1 |= (1 << (irq_number % 32));
		} else if (irq_number >= 64 && irq_number < 96) {
			// 64 - 95, ISER2 register
			*NVIC_ISER2 |= (1 << (irq_number % 32));
		}
	} else {
		if (irq_number <= 31) {
			// 0 - 31, ICER0 register
			*NVIC_ICER0 |= (1 << irq_number);
		} else if (irq_number > 31 && irq_number < 64) {
			// 32 - 63, ICER1 register
			*NVIC_ICER1 |= (1 << (irq_number % 32));
		} else if (irq_number >= 64 && irq_number < 96) {
			// 64 - 95, ICER2 register
			*NVIC_ICER2 |= (1 << (irq_number % 32));
		}
	}
}

/**
 * @fn			gpio_irq_priority_config
 *
 * @brief		Configures the interrupt request priority level
 *
 * @param		irq_number Number or position of the interrupt in the NVIC Vector Table (0-84)
 * @param		irq_priority Priority level (0 - 15), the less the level the higher the priority
 *
 * @return		None
 *
 * @note		None
 *
 **/
void gpio_irq_priority_config(const gpio_handle_t *p_gpio_handle, uint8_t irq_priority) {
	uint8_t irq_number = gpio_get_irq_number(p_gpio_handle);
	// Find out the IPR register
	uint8_t ipr_reg_index = irq_number / 4;	// Fields per register = 4, NVIC_IPR_index = irq_number / Fields per register
	uint8_t ipr_field_index = irq_number % 4;
	uint8_t ipr_field_bits_lenght = 8;

	// We need to shit 4 bits more because each ipr field implements the priority level in the the 4 msb, the 4 lsb are not implemented
	uint8_t shift_amount = (ipr_field_bits_lenght * ipr_field_index) + (8 - PRIORITY_BITS_IMPLEMENTED_NUM);
	*(NVIC_PRIORITY_BASE_ADDRESS + ipr_reg_index) |= irq_priority << shift_amount;
}

/**
 * @fn			gpio_irq_handling
 *
 * @brief		Clears the EXTI Pending Register corresponding to the pin number
 *
 * @param		pin_number GPIO pin number attached to the EXTI line
 *
 * @return		None
 *
 * @note		None
 *
 **/
void gpio_irq_handling(gpio_pin_number_t pin_number)
{
	if (EXTI->PR & (1 << pin_number)) {	// This bit is set when the selected edge event arrives on the external interrupt line.
		EXTI->PR |= (1 << pin_number);	// This bit is cleared by programming it to ‘1’.
	}
}
