/**
 ******************************************************************************
 * @file           : main.c
 * @author         : David Yepes
 * @brief          : Main program body
 ******************************************************************************
 * @attention   
 * 
 * Hardware Nucleo-F401RE
 * 
 * GPIO PA5:  Built-in LED (LD2)
 *            Active-high LED
 *  
 * GPIO PC13: Built-in user button (B1) 
 *            Has an external pull-up resistor (active-low button)
 **/

#include "stm32f401xx.h"
#include "stm32f401xx_gpio_driver.h"

const uint8_t BTN_PRESSED =	0;	

void delay(){
	for(uint32_t i=0; i<500000/2; i++);
}

int main (void) {
	gpio_handle_t gpio_led, gpio_button;
    gpio_pin_config_t gpio_pin_config;

	gpio_pin_config.gpio_pin_number = GPIO_PIN_NUM_5;   
	gpio_pin_config.gpio_pin_mode = GPIO_MODE_OUTPUT;
	gpio_pin_config.gpio_pin_speed = GPIO_OUTPUT_SPEED_MEDIUM;
	gpio_pin_config.gpio_pin_out_type = GPIO_OUTPUT_TYPE_PUSH_PULL;
	gpio_pin_config.gpio_pin_pu_pd_control = GPIO_PIN_NO_PULLUP_PULLDOWN;

	gpio_init(&gpio_led, GPIOA, &gpio_pin_config);

	gpio_pin_config.gpio_pin_number = GPIO_PIN_NUM_13;   
	gpio_pin_config.gpio_pin_mode = GPIO_MODE_INPUT; 
    
    gpio_init(&gpio_button, GPIOC, &gpio_pin_config);

	while(1) {
		if (gpio_read_from_input_pin(&gpio_button) == BTN_PRESSED) {
			delay();
			gpio_toggle_output_pin(&gpio_led);
		}
	}

	return 0;
}