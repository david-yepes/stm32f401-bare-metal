/*
 * smt32f401xx_gpio_driver.h
 *
 *  Created on: Aug 29, 2025
 *      Author: David Yepes
 */

#ifndef INC_STM32F401XX_GPIO_DRIVER_H_
#define INC_STM32F401XX_GPIO_DRIVER_H_

#include "stm32f401xx.h"

#include <stddef.h>
#include <stdint.h>

/*
 * @GPIO_PORTS
 * Peripheral definitions (Peripheral base addresses typecasted to gpio_reg_def_t)
 */

#define GPIOA	( (gpio_reg_def_t*)GPIOA_BASEADDR )
#define GPIOB	( (gpio_reg_def_t*)GPIOB_BASEADDR )
#define GPIOC	( (gpio_reg_def_t*)GPIOC_BASEADDR )
#define GPIOD	( (gpio_reg_def_t*)GPIOD_BASEADDR )
#define GPIOE	( (gpio_reg_def_t*)GPIOE_BASEADDR )
#define GPIOH	( (gpio_reg_def_t*)GPIOH_BASEADDR )

/*
 * Bit-field structure for GPIO port mode register (GPIOx_MODER) (x = A..E and H)
 */
typedef union
{ 
	struct {
		uint32_t MODER0:2;   		/* GPIO port mode register 0                Bit: 0-1    */ 
		uint32_t MODER1:2;   		/* GPIO port mode register 1                Bit: 2-3    */
		uint32_t MODER2:2;   		/* GPIO port mode register 2                Bit: 4-5    */
		uint32_t MODER3:2;   		/* GPIO port mode register 3                Bit: 6-7    */
		uint32_t MODER4:2;   		/* GPIO port mode register 4                Bit: 8-9    */
		uint32_t MODER5:2;   		/* GPIO port mode register 5                Bit: 10-11  */
		uint32_t MODER6:2;   		/* GPIO port mode register 6                Bit: 12-13  */
		uint32_t MODER7:2;   		/* GPIO port mode register 7                Bit: 14-15  */
		uint32_t MODER8:2;   		/* GPIO port mode register 8                Bit: 16-17  */
		uint32_t MODER9:2;   		/* GPIO port mode register 9                Bit: 18-19  */
		uint32_t MODER10:2;  		/* GPIO port mode register 10               Bit: 20-21  */
		uint32_t MODER11:2;  		/* GPIO port mode register 11               Bit: 22-23  */
		uint32_t MODER12:2;  		/* GPIO port mode register 12               Bit: 24-25  */
		uint32_t MODER13:2;  		/* GPIO port mode register 13               Bit: 26-27  */
		uint32_t MODER14:2;  		/* GPIO port mode register 14               Bit: 28-29  */
		uint32_t MODER15:2;			/* GPIO port mode register 15               Bit: 30-31  */
	};
	uint32_t ALLBITS;
} GPIOx_MODER_t;

/*
 * GPIO port output type register (GPIOx_OTYPER) (x = A..E and H)
 */
typedef union
{ 
	struct 
	{
		uint32_t OT0:1;          	/* GPIO port output type register 0       	Bit: 0    	*/ 
		uint32_t OT1:1;          	/* GPIO port output type register 1         Bit: 1    	*/
		uint32_t OT2:1;          	/* GPIO port output type register 2         Bit: 2    	*/ 
		uint32_t OT3:1;          	/* GPIO port output type register 3         Bit: 3    	*/
		uint32_t OT4:1;          	/* GPIO port output type register 4         Bit: 4    	*/ 
		uint32_t OT5:1;          	/* GPIO port output type register 5         Bit: 5    	*/
		uint32_t OT6:1;          	/* GPIO port output type register 6         Bit: 6    	*/ 
		uint32_t OT7:1;          	/* GPIO port output type register 7         Bit: 7    	*/
		uint32_t OT8:1;          	/* GPIO port output type register 8         Bit: 8    	*/ 
		uint32_t OT9:1;          	/* GPIO port output type register 9         Bit: 9    	*/
		uint32_t OT10:1;         	/* GPIO port output type register 10        Bit: 10    	*/ 
		uint32_t OT11:1;         	/* GPIO port output type register 11        Bit: 11   	*/
		uint32_t OT12:1;         	/* GPIO port output type register 12        Bit: 12   	*/ 
		uint32_t OT13:1;         	/* GPIO port output type register 13        Bit: 13   	*/
		uint32_t OT14:1;         	/* GPIO port output type register 14        Bit: 14   	*/ 
		uint32_t OT15:1;         	/* GPIO port output type register 15        Bit: 15   	*/
		uint32_t RESERVED:16;		/* Reserved 								Bit: 16-31  */
	};
	uint32_t ALLBITS;    
} GPIOx_OTYPER_t;

/*
 * GPIO port output speed register (GPIOx_OSPEEDR) (x = A..E and H)
 */
typedef union
{ 
    struct 
	{
		uint32_t OSPEEDR0:2;      	/* GPIO port output speed register 0      	Bit: 0-1    */ 
		uint32_t OSPEEDR1:2;      	/* GPIO port output speed register 1      	Bit: 2-3    */
		uint32_t OSPEEDR2:2;      	/* GPIO port output speed register 2      	Bit: 4-5    */
		uint32_t OSPEEDR3:2;      	/* GPIO port output speed register 3      	Bit: 6-7    */
		uint32_t OSPEEDR4:2;      	/* GPIO port output speed register 4      	Bit: 8-9    */
		uint32_t OSPEEDR5:2;      	/* GPIO port output speed register 5      	Bit: 10-11  */
		uint32_t OSPEEDR6:2;      	/* GPIO port output speed register 6      	Bit: 12-13  */
		uint32_t OSPEEDR7:2;      	/* GPIO port output speed register 7      	Bit: 14-15  */
		uint32_t OSPEEDR8:2;      	/* GPIO port output speed register 8      	Bit: 16-17  */
		uint32_t OSPEEDR9:2;      	/* GPIO port output speed register 9      	Bit: 18-19  */
		uint32_t OSPEEDR10:2;     	/* GPIO port output speed register 10     	Bit: 20-21  */
		uint32_t OSPEEDR11:2;     	/* GPIO port output speed register 11     	Bit: 22-23  */
		uint32_t OSPEEDR12:2;     	/* GPIO port output speed register 12     	Bit: 24-25  */
		uint32_t OSPEEDR13:2;     	/* GPIO port output speed register 13     	Bit: 26-27  */
		uint32_t OSPEEDR14:2;     	/* GPIO port output speed register 14     	Bit: 28-29  */
		uint32_t OSPEEDR15:2;     	/* GPIO port output speed register 15     	Bit: 30-31  */
	};
	uint32_t ALLBITS;
} GPIOx_OSPEEDR_t;

/*
 * GPIO port pull-up/pull-down register (GPIOx_PUPDR) (x = A..E and H)
 */
typedef union
{ 
	struct {
		uint32_t PUPDR0:2;         	/* GPIO port pull-up/pull-down register 0  	Bit: 0-1    */ 
		uint32_t PUPDR1:2;         	/* GPIO port pull-up/pull-down register 1  	Bit: 2-3    */
		uint32_t PUPDR2:2;         	/* GPIO port pull-up/pull-down register 2  	Bit: 4-5    */
		uint32_t PUPDR3:2;         	/* GPIO port pull-up/pull-down register 3  	Bit: 6-7    */
		uint32_t PUPDR4:2;         	/* GPIO port pull-up/pull-down register 4  	Bit: 8-9    */
		uint32_t PUPDR5:2;         	/* GPIO port pull-up/pull-down register 5  	Bit: 10-11  */
		uint32_t PUPDR6:2;         	/* GPIO port pull-up/pull-down register 6  	Bit: 12-13  */
		uint32_t PUPDR7:2;         	/* GPIO port pull-up/pull-down register 7  	Bit: 14-15  */
		uint32_t PUPDR8:2;         	/* GPIO port pull-up/pull-down register 8  	Bit: 16-17  */
		uint32_t PUPDR9:2;         	/* GPIO port pull-up/pull-down register 9  	Bit: 18-19  */
		uint32_t PUPDR10:2;        	/* GPIO port pull-up/pull-down register 10 	Bit: 20-21  */
		uint32_t PUPDR11:2;        	/* GPIO port pull-up/pull-down register 11 	Bit: 22-23  */
		uint32_t PUPDR12:2;        	/* GPIO port pull-up/pull-down register 12 	Bit: 24-25  */
		uint32_t PUPDR13:2;        	/* GPIO port pull-up/pull-down register 13 	Bit: 26-27  */
		uint32_t PUPDR14:2;        	/* GPIO port pull-up/pull-down register 14 	Bit: 28-29  */
		uint32_t PUPDR15:2;        	/* GPIO port pull-up/pull-down register 15 	Bit: 30-31  */
	};
    uint32_t ALLBITS;
} GPIOx_PUPDR_t;

/*
 * GPIO port input data register (GPIOx_IDR) (x = A..E and H)
 */
typedef union
{ 
	struct {
		uint32_t IDR0:1;          	/* GPIO port input data register 0       	Bit: 0    	*/ 
		uint32_t IDR1:1;            /* GPIO port input data register 1         	Bit: 1    	*/
		uint32_t IDR2:1;            /* GPIO port input data register 2         	Bit: 2    	*/ 
		uint32_t IDR3:1;            /* GPIO port input data register 3         	Bit: 3    	*/
		uint32_t IDR4:1;            /* GPIO port input data register 4         	Bit: 4    	*/ 
		uint32_t IDR5:1;            /* GPIO port input data register 5         	Bit: 5    	*/
		uint32_t IDR6:1;            /* GPIO port input data register 6         	Bit: 6    	*/ 
		uint32_t IDR7:1;            /* GPIO port input data register 7         	Bit: 7    	*/
		uint32_t IDR8:1;            /* GPIO port input data register 8         	Bit: 8    	*/ 
		uint32_t IDR9:1;            /* GPIO port input data register 9         	Bit: 9    	*/
		uint32_t IDR10:1;           /* GPIO port input data register 10        	Bit: 10    	*/ 
		uint32_t IDR11:1;           /* GPIO port input data register 11        	Bit: 11   	*/
		uint32_t IDR12:1;           /* GPIO port input data register 12        	Bit: 12   	*/ 
		uint32_t IDR13:1;           /* GPIO port input data register 13        	Bit: 13   	*/
		uint32_t IDR14:1;           /* GPIO port input data register 14        	Bit: 14   	*/ 
		uint32_t IDR15:1;           /* GPIO port input data register 15        	Bit: 15   	*/
		uint32_t RESERVED:16;		/* Reserved 								Bit: 16-31  */
	};
	uint32_t ALLBITS;
} GPIOx_IDR_t;

/*
 * GPIO port output data register (GPIOx_ODR) (x = A..E and H)
 */
typedef union
{ 
	struct {
		uint32_t ODR0:1;           	/* GPIO port output data register 0       	Bit: 0    	*/ 
		uint32_t ODR1:1;           	/* GPIO port output data register 1         Bit: 1    	*/
		uint32_t ODR2:1;           	/* GPIO port output data register 2         Bit: 2    	*/ 
		uint32_t ODR3:1;           	/* GPIO port output data register 3         Bit: 3    	*/
		uint32_t ODR4:1;           	/* GPIO port output data register 4         Bit: 4    	*/ 
		uint32_t ODR5:1;           	/* GPIO port output data register 5         Bit: 5    	*/
		uint32_t ODR6:1;           	/* GPIO port output data register 6         Bit: 6    	*/ 
		uint32_t ODR7:1;           	/* GPIO port output data register 7         Bit: 7    	*/
		uint32_t ODR8:1;           	/* GPIO port output data register 8         Bit: 8    	*/ 
		uint32_t ODR9:1;           	/* GPIO port output data register 9         Bit: 9    	*/
		uint32_t ODR10:1;          	/* GPIO port output data register 10        Bit: 10    	*/ 
		uint32_t ODR11:1;          	/* GPIO port output data register 11        Bit: 11   	*/
		uint32_t ODR12:1;          	/* GPIO port output data register 12        Bit: 12   	*/ 
		uint32_t ODR13:1;          	/* GPIO port output data register 13        Bit: 13   	*/
		uint32_t ODR14:1;          	/* GPIO port output data register 14        Bit: 14   	*/ 
		uint32_t ODR15:1;          	/* GPIO port output data register 15        Bit: 15   	*/
		uint32_t RESERVED:16;		/* Reserved 								Bit: 16-31  */
	};
	uint32_t ALLBITS;
} GPIOx_ODR_t;

/*
 * GPIO port bit set/reset register (GPIOx_BSRR) (x = A..E and H)
 */
typedef union
{ 
    struct {
		uint32_t BS0:1;           	/* Port x reset bit 0       				Bit: 0    	*/ 
		uint32_t BS1:1;           	/* Port x reset bit 1         				Bit: 1    	*/
		uint32_t BS2:1;           	/* Port x reset bit 2         				Bit: 2    	*/ 
		uint32_t BS3:1;           	/* Port x reset bit 3         				Bit: 3    	*/
		uint32_t BS4:1;           	/* Port x reset bit 4         				Bit: 4    	*/ 
		uint32_t BS5:1;           	/* Port x reset bit 5         				Bit: 5    	*/
		uint32_t BS6:1;           	/* Port x reset bit 6         				Bit: 6    	*/ 
		uint32_t BS7:1;           	/* Port x reset bit 7         				Bit: 7    	*/
		uint32_t BS8:1;           	/* Port x reset bit 8         				Bit: 8    	*/ 
		uint32_t BS9:1;           	/* Port x reset bit 9         				Bit: 9    	*/
		uint32_t BS10:1;           	/* Port x reset bit 10        				Bit: 10    	*/ 
		uint32_t BS11:1;           	/* Port x reset bit 11        				Bit: 11   	*/
		uint32_t BS12:1;           	/* Port x reset bit 12        				Bit: 12   	*/ 
		uint32_t BS13:1;           	/* Port x reset bit 13        				Bit: 13   	*/
		uint32_t BS14:1;           	/* Port x reset bit 14        				Bit: 14   	*/ 
		uint32_t BS15:1;           	/* Port x reset bit 15        				Bit: 15   	*/
		uint32_t BR0:1;           	/* Port x set bit 0       					Bit: 16    	*/ 
		uint32_t BR1:1;           	/* Port x set bit 1         				Bit: 17    	*/
		uint32_t BR2:1;           	/* Port x set bit 2         				Bit: 18    	*/ 
		uint32_t BR3:1;           	/* Port x set bit 3         				Bit: 19   	*/
		uint32_t BR4:1;           	/* Port x set bit 4         				Bit: 20   	*/ 
		uint32_t BR5:1;           	/* Port x set bit 5         				Bit: 21   	*/
		uint32_t BR6:1;           	/* Port x set bit 6         				Bit: 22   	*/ 
		uint32_t BR7:1;           	/* Port x set bit 7         				Bit: 23   	*/
		uint32_t BR8:1;           	/* Port x set bit 8         				Bit: 24   	*/ 
		uint32_t BR9:1;           	/* Port x set bit 9         				Bit: 25   	*/
		uint32_t BR10:1;           	/* Port x set bit 10        				Bit: 26    	*/ 
		uint32_t BR11:1;           	/* Port x set bit 11        				Bit: 27   	*/
		uint32_t BR12:1;           	/* Port x set bit 12        				Bit: 28   	*/ 
		uint32_t BR13:1;           	/* Port x set bit 13        				Bit: 29   	*/
		uint32_t BR14:1;           	/* Port x set bit 14        				Bit: 30   	*/ 
		uint32_t BR15:1;           	/* Port x set bit 15        				Bit: 31   	*/
	};
	uint32_t ALLBITS;
} GPIOx_BSRR_t;

/*
 * GPIO port configuration lock register (GPIOx_LCKR) (x = A..E and H)
 */
typedef union
{ 
    struct{
		uint32_t LCK0:1;           	/* GPIO port configuration lock register 0 	Bit: 0    	*/ 
		uint32_t LCK1:1;           	/* GPIO port configuration lock register 1  Bit: 1    	*/
		uint32_t LCK2:1;           	/* GPIO port configuration lock register 2  Bit: 2    	*/ 
		uint32_t LCK3:1;           	/* GPIO port configuration lock register 3  Bit: 3    	*/
		uint32_t LCK4:1;           	/* GPIO port configuration lock register 4  Bit: 4    	*/ 
		uint32_t LCK5:1;           	/* GPIO port configuration lock register 5  Bit: 5    	*/
		uint32_t LCK6:1;           	/* GPIO port configuration lock register 6  Bit: 6    	*/ 
		uint32_t LCK7:1;           	/* GPIO port configuration lock register 7  Bit: 7    	*/
		uint32_t LCK8:1;           	/* GPIO port configuration lock register 8  Bit: 8    	*/ 
		uint32_t LCK9:1;           	/* GPIO port configuration lock register 9  Bit: 9    	*/
		uint32_t LCK10:1;           /* GPIO port configuration lock register 10 Bit: 10    	*/ 
		uint32_t LCK11:1;           /* GPIO port configuration lock register 11 Bit: 11   	*/
		uint32_t LCK12:1;           /* GPIO port configuration lock register 12 Bit: 12   	*/ 
		uint32_t LCK13:1;           /* GPIO port configuration lock register 13 Bit: 13   	*/
		uint32_t LCK14:1;           /* GPIO port configuration lock register 14 Bit: 14   	*/ 
		uint32_t LCK15:1;           /* GPIO port configuration lock register 15 Bit: 15   	*/
		uint32_t LCKK:1;           	/* GPIO port configuration lock register K  Bit: 16   	*/
		uint32_t RESERVED:15;		/* Reserved 								Bit: 17-31  */
	};
	uint32_t ALLBITS;
} GPIOx_LCKR_t;

/*
 * GPIO alternate function low register (GPIOx_AFRL) (x = A..E and H)
 */
typedef union
{ 
    struct  {
		uint32_t AFRL0:4;         	/* GPIO alternate function low register 0 	Bit: 0-3  	*/ 
		uint32_t AFRL1:4;         	/* GPIO alternate function low register 1  	Bit: 4-7   	*/
		uint32_t AFRL2:4;         	/* GPIO alternate function low register 2  	Bit: 8-11  	*/ 
		uint32_t AFRL3:4;         	/* GPIO alternate function low register 3  	Bit: 12-15 	*/
		uint32_t AFRL4:4;         	/* GPIO alternate function low register 4  	Bit: 16-19 	*/ 
		uint32_t AFRL5:4;         	/* GPIO alternate function low register 5  	Bit: 20-23 	*/
		uint32_t AFRL6:4;         	/* GPIO alternate function low register 6  	Bit: 24-27 	*/ 
		uint32_t AFRL7:4;         	/* GPIO alternate function low register 7  	Bit: 28-31 	*/	
	};
	uint32_t ALL_BITS;
} GPIOx_AFRL_t;

/*
 * GPIO alternate function high register (GPIOx_AFRH) (x = A..E and H)
 */
typedef union
{ 
	struct {
		uint32_t AFRH0:4;       	/* GPIO alternate function high register 0 	Bit: 0-3  	*/ 
		uint32_t AFRH1:4;       	/* GPIO alternate function high register 1  Bit: 4-7   	*/
		uint32_t AFRH2:4;       	/* GPIO alternate function high register 2  Bit: 8-11  	*/ 
		uint32_t AFRH3:4;       	/* GPIO alternate function high register 3  Bit: 12-15 	*/
		uint32_t AFRH4:4;       	/* GPIO alternate function high register 4  Bit: 16-19 	*/ 
		uint32_t AFRH5:4;       	/* GPIO alternate function high register 5  Bit: 20-23 	*/
		uint32_t AFRH6:4;       	/* GPIO alternate function high register 6  Bit: 24-27 	*/ 
		uint32_t AFRH7:4;       	/* GPIO alternate function high register 7  Bit: 28-31 	*/	
	};
	uint32_t ALL_BITS;
} GPIOx_AFRH_t;

/*
 * Peripheral register definition structure for GPIO
 */
typedef struct
{
	volatile GPIOx_MODER_t 		MODER;		/* GPIO port mode register 					Address offset: 0x00 */
	volatile GPIOx_OTYPER_t 	OTYPER;		/* GPIO port output type register			Address offset: 0x04 */
	volatile GPIOx_OSPEEDR_t	OSPEEDR;	/* GPIO port output speed register 			Address offset: 0x08 */
	volatile GPIOx_PUPDR_t 		PUPDR;		/* GPIO port pull-up/pull-down register		Address offset: 0x0C */
	volatile GPIOx_IDR_t 		IDR;		/* GPIO port input data register 			Address offset: 0x10 */
	volatile GPIOx_ODR_t 		ODR;		/* GPIO port output data register			Address offset: 0x14 */
	volatile GPIOx_BSRR_t 		BSRR;		/* GPIO port bit set/reset register			Address offset: 0x18 */
	volatile GPIOx_LCKR_t 		LCKR;		/* GPIO port configuration lock register	Address offset: 0x1C */
	volatile GPIOx_AFRL_t 		AFRL;		/* Alternate function low register			Address offset: 0x20 */
	volatile GPIOx_AFRH_t 		AFRH;		/* Alternate function high register			Address offset: 0x24 */
} gpio_reg_def_t;

/*
* @gpio_pin_number_t
* GPIO pin numbers
*/
typedef enum {
	GPIO_PIN_NUM_0 = 0,					
	GPIO_PIN_NUM_1,					
	GPIO_PIN_NUM_2,					
	GPIO_PIN_NUM_3,					
	GPIO_PIN_NUM_4,					
	GPIO_PIN_NUM_5,					
	GPIO_PIN_NUM_6,					
	GPIO_PIN_NUM_7,					
	GPIO_PIN_NUM_8,					
	GPIO_PIN_NUM_9,					
	GPIO_PIN_NUM_10,					
	GPIO_PIN_NUM_11,					
	GPIO_PIN_NUM_12,					
	GPIO_PIN_NUM_13,					
	GPIO_PIN_NUM_14,					
	GPIO_PIN_NUM_15					
} gpio_pin_number_t;

/*
 * @GPIO_PIN_MODES
 * GPIO pin possible modes
 */
typedef enum {
	GPIO_MODE_INPUT,					
	GPIO_MODE_OUTPUT,				
	GPIO_MODE_ALTERNATE_FN,			
	GPIO_MODE_ANALOG,				
	GPIO_MODE_EXTI_FALLING_TRIGGER,	
	GPIO_MODE_EXTI_RISING_TRIGGER,	
	GPIO_MODE_EXTI_RISING_FALLING	
} gpio_pin_mode_t;

/*
 * @GPIO_PIN_SPEED
 * GPIO pin possible output speeds
 */
typedef enum {
	GPIO_OUTPUT_SPEED_LOW,					
	GPIO_OUTPUT_SPEED_MEDIUM,				
	GPIO_OUTPUT_SPEED_HIGH,			
	GPIO_OUTPUT_SPEED_VERY_HIGH
} gpio_pin_speed_t;

/*
 * @GPIO_PIN_PU_PD_MODES
 * GPIO pin pull-up and pull-down modes
 */
typedef enum {
	GPIO_PIN_NO_PULLUP_PULLDOWN,					
	GPIO_PIN_PULL_UP,				
	GPIO_PIN_PULL_DOWN
} gpio_pin_pu_pd_control_t;

/*
 * @GPIO_PIN_OUTPUT_TYPES
 * GPIO pin possible output types
 */
typedef enum {
	GPIO_OUTPUT_TYPE_PUSH_PULL,					
	GPIO_OUTPUT_TYPE_OPEN_DRAIN
} gpio_pin_out_type_t;

/*
* @gpio_pin_alt_fun_mode_t
* GPIO pin possible alternate function options
*/
typedef enum {
	GPIO_PIN_AF_0 = 0,					
	GPIO_PIN_AF_1,					
	GPIO_PIN_AF_2,					
	GPIO_PIN_AF_3,					
	GPIO_PIN_AF_4,					
	GPIO_PIN_AF_5,					
	GPIO_PIN_AF_6,					
	GPIO_PIN_AF_7,					
	GPIO_PIN_AF_8,					
	GPIO_PIN_AF_9,					
	GPIO_PIN_AF_10,					
	GPIO_PIN_AF_11,					
	GPIO_PIN_AF_12,					
	GPIO_PIN_AF_13,					
	GPIO_PIN_AF_14,					
	GPIO_PIN_AF_15					
} gpio_pin_alt_fun_mode_t;

/*
 * @GPIO_PinConfig_t
 * This is a configuration structure for a GPIO pin
 */
typedef struct
{
	gpio_pin_number_t 			gpio_pin_number;		/* Possible values from @gpio_pin_number_t */
	gpio_pin_mode_t 			gpio_pin_mode;			/* Possible values from @GPIO_PIN_MODES */
	gpio_pin_speed_t 			gpio_pin_speed;			/* Possible values from @GPIO_PIN_SPEED */
	gpio_pin_pu_pd_control_t	gpio_pin_pu_pd_control;	/* Possible values from @GPIO_PIN_PU_PD_MODES */
	gpio_pin_out_type_t 		gpio_pin_out_type;		/* Possible values from @GPIO_PIN_OUTPUT_TYPES */
	gpio_pin_alt_fun_mode_t 	gpio_pin_alt_fun_mode;	/* Possible values from @GPIO_PIN_ALT_FUNCTIONS */
} gpio_pin_config_t;

/*
 * This is a handle structure for GPIO pin
 */
typedef struct
{
	gpio_reg_def_t 		*p_gpio_x;			/* This holds the base address of the GPIO port which the pins belongs */
	gpio_pin_config_t	gpio_pin_config;	/* This holds GPIO pin configuration settings */
} gpio_handle_t;
//typedef struct gpio_handle_t gpio_handle_t;	// Opaque type 

typedef void (*irq_handler_callback_t)( void );

/************************************************************************************************
 *
 * APIs supported by this driver
 * For more information about the APIs check the function definitions
 *
 *************************************************************************************************/

void gpio_peri_clock_control(const gpio_reg_def_t *p_gpio_x, uint8_t en_or_di);

void gpio_init(gpio_handle_t *p_gpio_handle, gpio_reg_def_t *const p_gpio_x, const gpio_pin_config_t *gpio_pin_config);
void gpio_deinit(const gpio_handle_t *p_gpio_handle);

uint8_t gpio_read_from_input_pin(const gpio_handle_t *p_gpio_handle);
uint16_t gpio_read_from_input_port(const gpio_reg_def_t *p_gpio_x);
void gpio_write_to_output_pin(gpio_handle_t *p_gpio_handle, uint8_t value);
void gpio_write_to_output_port(gpio_reg_def_t *p_gpio_x, uint16_t value);
void gpio_toggle_output_pin(gpio_handle_t *p_gpio_handle);

void gpio_irq_interrupt_config(const gpio_handle_t *p_gpio_handle, uint8_t en_or_di);
void gpio_irq_priority_config(const gpio_handle_t *p_gpio_handle, uint8_t irq_priority);
void gpio_irq_handling(gpio_pin_number_t pin_number);
void gpio_irq_callback_register(const gpio_handle_t *p_gpio_handle, irq_handler_callback_t callback);

#endif /* INC_STM32F401XX_GPIO_DRIVER_H_ */
