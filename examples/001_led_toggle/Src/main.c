/**
 ******************************************************************************
 * @file           : main.c
 * @author         : David Yepes
 * @brief          : Main program body
 ******************************************************************************
 */

#include "stm32f401xx.h"
#include "stm32f401xx_gpio_driver.h"

void delay(){
	for(uint32_t i=0; i<500000; i++);
}

int main (void) {
	gpio_handle_t gpio_led;
	gpio_pin_config_t gpio_pin_config;

	gpio_pin_config.gpio_pin_number = GPIO_PIN_NUM_5;
	gpio_pin_config.gpio_pin_mode = GPIO_MODE_OUTPUT;
	gpio_pin_config.gpio_pin_speed = GPIO_OUTPUT_SPEED_MEDIUM;
	gpio_pin_config.gpio_pin_out_type = GPIO_OUTPUT_TYPE_PUSH_PULL;
	gpio_pin_config.gpio_pin_pu_pd_control = GPIO_PIN_NO_PULLUP_PULLDOWN;

	gpio_init(&gpio_led, GPIOA, &gpio_pin_config);

	while(1) {
		gpio_toggle_output_pin(&gpio_led);
		delay();
	}

	return 0;
}
