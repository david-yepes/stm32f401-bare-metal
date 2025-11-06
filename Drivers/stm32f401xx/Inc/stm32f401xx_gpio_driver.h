/**
 * @file stm32f401xx_gpio_driver.h
 * @author David Yepes
 * @brief 
 * @version 0.1
 * @date 2025-10-31
 * 
 * @copyright Copyright (c) 2025
 * 
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

#define GPIOA	( (gpio_reg_def_t*)GPIOA_BASEADDR )	//!< General-purpose I/O port A peripheral definition: GPIOA base addresses typecasted to gpio_reg_def_t
#define GPIOB	( (gpio_reg_def_t*)GPIOB_BASEADDR )	//!< General-purpose I/O port B peripheral definition: GPIOB base addresses typecasted to gpio_reg_def_t
#define GPIOC	( (gpio_reg_def_t*)GPIOC_BASEADDR )	//!< General-purpose I/O port C peripheral definition: GPIOC base addresses typecasted to gpio_reg_def_t
#define GPIOD	( (gpio_reg_def_t*)GPIOD_BASEADDR )	//!< General-purpose I/O port D peripheral definition: GPIOD base addresses typecasted to gpio_reg_def_t
#define GPIOE	( (gpio_reg_def_t*)GPIOE_BASEADDR )	//!< General-purpose I/O port E peripheral definition: GPIOE base addresses typecasted to gpio_reg_def_t
#define GPIOH	( (gpio_reg_def_t*)GPIOH_BASEADDR )	//!< General-purpose I/O port H peripheral definition: GPIOH base addresses typecasted to gpio_reg_def_t

/**
 * Bit-field structure for GPIO port mode register (GPIOx_MODER) (x = A..E and H)
 * 
 * MODERy[1:0]: Port x configuration bits (y = 0..15). These bits are written by software to configure the I/O direction mode.
 *  - 00: Input (reset state)\n
 *  - 01: General purpose output mode\n
 *  - 10: Alternate function mode\n
 *  - 11: Analog mode\n
 */
typedef union
{ 
	struct {
		uint32_t MODER0:2;   		//!< Bits 0-1: GPIO port mode register 0                	    
		uint32_t MODER1:2;   		//!< Bits 2-3: GPIO port mode register 1                	   
		uint32_t MODER2:2;   		//!< Bits 4-5: GPIO port mode register 2                	   
		uint32_t MODER3:2;   		//!< Bits 6-7: GPIO port mode register 3                	  
		uint32_t MODER4:2;   		//!< Bits 8-9: GPIO port mode register 4                	   
		uint32_t MODER5:2;   		//!< Bits 10-11: GPIO port mode register 5                	
		uint32_t MODER6:2;   		//!< Bits 12-13: GPIO port mode register 6                	 
		uint32_t MODER7:2;   		//!< Bits 14-15: GPIO port mode register 7                	 
		uint32_t MODER8:2;   		//!< Bits 16-17: GPIO port mode register 8                	
		uint32_t MODER9:2;   		//!< Bits 18-19: GPIO port mode register 9                	
		uint32_t MODER10:2;  		//!< Bits 20-21: GPIO port mode register 10               	 
		uint32_t MODER11:2;  		//!< Bits 22-23: GPIO port mode register 11               	 
		uint32_t MODER12:2;  		//!< Bits 24-25: GPIO port mode register 12               	 
		uint32_t MODER13:2;  		//!< Bits 26-27: GPIO port mode register 13               	 
		uint32_t MODER14:2;  		//!< Bits 28-29: GPIO port mode register 14               	 
		uint32_t MODER15:2;			//!< Bits 30-31: GPIO port mode register 15               	 
	};
	uint32_t ALLBITS;				//!< GPIO port mode 32 bits register
} GPIOx_MODER_t;

/**
 * GPIO port output type register (GPIOx_OTYPER) (x = A..E and H)
 * 
 * OTy: Port x configuration bits (y = 0..15). These bits are written by software to configure the output type of the I/O port.
 *  - 0: Output push-pull (reset state)
 *  - 1: Output open-drain
 */
typedef union
{ 
	struct 
	{
		uint32_t OT0:1;          	//!< Bit 0:   GPIO port output type register 0       	    
		uint32_t OT1:1;          	//!< Bit 1:   GPIO port output type register 1         	   
		uint32_t OT2:1;          	//!< Bit 2:   GPIO port output type register 2         	    
		uint32_t OT3:1;          	//!< Bit 3:   GPIO port output type register 3         	   
		uint32_t OT4:1;          	//!< Bit 4:   GPIO port output type register 4         	    
		uint32_t OT5:1;          	//!< Bit 5:   GPIO port output type register 5         	   
		uint32_t OT6:1;          	//!< Bit 6:   GPIO port output type register 6         	    
		uint32_t OT7:1;          	//!< Bit 7:   GPIO port output type register 7         	   
		uint32_t OT8:1;          	//!< Bit 8:   GPIO port output type register 8         	    
		uint32_t OT9:1;          	//!< Bit 9:   GPIO port output type register 9         	   
		uint32_t OT10:1;         	//!< Bit 10:  GPIO port output type register 10        	     
		uint32_t OT11:1;         	//!< Bit 11:  GPIO port output type register 11        	   
		uint32_t OT12:1;         	//!< Bit 12:  GPIO port output type register 12        	    
		uint32_t OT13:1;         	//!< Bit 13:  GPIO port output type register 13        	   
		uint32_t OT14:1;         	//!< Bit 14:  GPIO port output type register 14        	    
		uint32_t OT15:1;         	//!< Bit 15:  GPIO port output type register 15        	   
		uint32_t RESERVED:16;		//!< Bits 16-31: Reserved 								
	};
	uint32_t ALLBITS;    			//!< GPIO port output type 32 bits register
} GPIOx_OTYPER_t;

/**
 * GPIO port output speed register (GPIOx_OSPEEDR) (x = A..E and H)
 * 
 * OSPEEDRy[1:0]: Port x configuration bits (y = 0..15). These bits are written by software to configure the I/O output speed.
 *  - 00: Low speed
 *  - 01: Medium speed
 *  - 10: High speed
 *  - 11: Very high speed
 * 
 * Note: Refer to the product datasheets for the values of OSPEEDRy bits versus VDD
 * range and external load.
 */
typedef union
{ 
    struct 
	{
		uint32_t OSPEEDR0:2;      	//!< Bits 0-1:   GPIO port output speed register 0      	  
		uint32_t OSPEEDR1:2;      	//!< Bits 2-3:   GPIO port output speed register 1      	 
		uint32_t OSPEEDR2:2;      	//!< Bits 4-5:   GPIO port output speed register 2      	 
		uint32_t OSPEEDR3:2;      	//!< Bits 6-7:   GPIO port output speed register 3      	 
		uint32_t OSPEEDR4:2;      	//!< Bits 8-9:   GPIO port output speed register 4      	 
		uint32_t OSPEEDR5:2;      	//!< Bits 10-11: GPIO port output speed register 5      	 
		uint32_t OSPEEDR6:2;      	//!< Bits 12-13: GPIO port output speed register 6      	 
		uint32_t OSPEEDR7:2;      	//!< Bits 14-15: GPIO port output speed register 7      	 
		uint32_t OSPEEDR8:2;      	//!< Bits 16-17: GPIO port output speed register 8      	 
		uint32_t OSPEEDR9:2;      	//!< Bits 18-19: GPIO port output speed register 9      	 
		uint32_t OSPEEDR10:2;     	//!< Bits 20-21: GPIO port output speed register 10     	 
		uint32_t OSPEEDR11:2;     	//!< Bits 22-23: GPIO port output speed register 11     	 
		uint32_t OSPEEDR12:2;     	//!< Bits 24-25: GPIO port output speed register 12     	 
		uint32_t OSPEEDR13:2;     	//!< Bits 26-27: GPIO port output speed register 13     	 
		uint32_t OSPEEDR14:2;     	//!< Bits 28-29: GPIO port output speed register 14     	 
		uint32_t OSPEEDR15:2;     	//!< Bits 30-31: GPIO port output speed register 15     	 
	};
	uint32_t ALLBITS;				//!< GPIO port output speed 32 bits register
} GPIOx_OSPEEDR_t;

/*! 
 * GPIO port pull-up/pull-down register (GPIOx_PUPDR) (x = A..E and H)
 * 
 * PUPDRy[1:0]: Port x configuration bits (y = 0..15). These bits are written by software to configure the I/O pull-up or pull-down.
 *  - 00: No pull-up, pull-down
 *  - 01: Pull-up
 *  - 10: Pull-down
 *  - 11: Reserved
 */
typedef union
{ 	
	struct {		
		uint32_t PUPDR0:2;         	//!< Bit 0-1:   GPIO port pull-up/pull-down register 0  	  
		uint32_t PUPDR1:2;         	//!< Bit 2-3:   GPIO port pull-up/pull-down register 1  	 
		uint32_t PUPDR2:2;         	//!< Bit 4-5:   GPIO port pull-up/pull-down register 2  	 
		uint32_t PUPDR3:2;         	//!< Bit 6-7:   GPIO port pull-up/pull-down register 3  	 
		uint32_t PUPDR4:2;         	//!< Bit 8-9:   GPIO port pull-up/pull-down register 4  	 
		uint32_t PUPDR5:2;         	//!< Bit 10-11: GPIO port pull-up/pull-down register 5  	 
		uint32_t PUPDR6:2;         	//!< Bit 12-13: GPIO port pull-up/pull-down register 6  	 
		uint32_t PUPDR7:2;         	//!< Bit 14-15: GPIO port pull-up/pull-down register 7  	 
		uint32_t PUPDR8:2;         	//!< Bit 16-17: GPIO port pull-up/pull-down register 8  	 
		uint32_t PUPDR9:2;         	//!< Bit 18-19: GPIO port pull-up/pull-down register 9  	 
		uint32_t PUPDR10:2;        	//!< Bit 20-21: GPIO port pull-up/pull-down register 10 	 
		uint32_t PUPDR11:2;        	//!< Bit 22-23: GPIO port pull-up/pull-down register 11 	 
		uint32_t PUPDR12:2;        	//!< Bit 24-25: GPIO port pull-up/pull-down register 12 	 
		uint32_t PUPDR13:2;        	//!< Bit 26-27: GPIO port pull-up/pull-down register 13 	 
		uint32_t PUPDR14:2;        	//!< Bit 28-29: GPIO port pull-up/pull-down register 14 	 
		uint32_t PUPDR15:2;        	//!< Bit 30-31: GPIO port pull-up/pull-down register 15 	 
	};
    uint32_t ALLBITS;				//!< GPIO port pull-up/pull-down 32 bits register
} GPIOx_PUPDR_t;

/**
 * GPIO port input data register (GPIOx_IDR) (:x = A..E and H)
 * 
 * These bits are read-only and can be accessed in word mode only. They contain the input
 * value of the corresponding I/O port.
 *  - Bits 31:16 Reserved, must be kept at reset value.
 *  - Bits 15:0 IDRy: Port input data (y = 0..15) 
 */
typedef union
{ 
	struct {
		uint32_t IDR0:1;          	//!< Bit 0:  GPIO port input data register 0       		 
		uint32_t IDR1:1;            //!< Bit 1:  GPIO port input data register 1         	
		uint32_t IDR2:1;            //!< Bit 2:  GPIO port input data register 2         	 
		uint32_t IDR3:1;            //!< Bit 3:  GPIO port input data register 3         	
		uint32_t IDR4:1;            //!< Bit 4:  GPIO port input data register 4         	 
		uint32_t IDR5:1;            //!< Bit 5:  GPIO port input data register 5         	
		uint32_t IDR6:1;            //!< Bit 6:  GPIO port input data register 6         	 
		uint32_t IDR7:1;            //!< Bit 7:  GPIO port input data register 7         	
		uint32_t IDR8:1;            //!< Bit 8:  GPIO port input data register 8         	 
		uint32_t IDR9:1;            //!< Bit 9:  GPIO port input data register 9         	
		uint32_t IDR10:1;           //!< Bit 10: GPIO port input data register 10        		 
		uint32_t IDR11:1;           //!< Bit 11: GPIO port input data register 11        	
		uint32_t IDR12:1;           //!< Bit 12: GPIO port input data register 12        	 
		uint32_t IDR13:1;           //!< Bit 13: GPIO port input data register 13        	
		uint32_t IDR14:1;           //!< Bit 14: GPIO port input data register 14        	 
		uint32_t IDR15:1;           //!< Bit 15: GPIO port input data register 15        	
		uint32_t RESERVED:16;		//!< Bits 16-31: Reserved 									  
	};
	uint32_t ALLBITS;				//!< GPIO port input data 32 bits register
} GPIOx_IDR_t;

/**
 * GPIO port output data register (GPIOx_ODR) (x = A..E and H)
 * 
 * These bits can be read and written by software. Note: For atomic bit set/reset, the ODR bits can be individually set and reset by writing to the
 * GPIOx_BSRR register (x = A..E and H).
 *  - Bits 31:16 Reserved, must be kept at reset value.
 *  - Bits 15:0 ODRy: Port output data (y = 0..15).
 */
typedef union
{ 
	struct {
		uint32_t ODR0:1;           	//!< Bit 0:  GPIO port output data register 0       	    
		uint32_t ODR1:1;           	//!< Bit 1:  GPIO port output data register 1         	   
		uint32_t ODR2:1;           	//!< Bit 2:  GPIO port output data register 2         	    
		uint32_t ODR3:1;           	//!< Bit 3:  GPIO port output data register 3         	   
		uint32_t ODR4:1;           	//!< Bit 4:  GPIO port output data register 4         	    
		uint32_t ODR5:1;           	//!< Bit 5:  GPIO port output data register 5         	   
		uint32_t ODR6:1;           	//!< Bit 6:  GPIO port output data register 6         	    
		uint32_t ODR7:1;           	//!< Bit 7:  GPIO port output data register 7         	   
		uint32_t ODR8:1;           	//!< Bit 8:  GPIO port output data register 8         	    
		uint32_t ODR9:1;           	//!< Bit 9:  GPIO port output data register 9         	   
		uint32_t ODR10:1;          	//!< Bit 10: GPIO port output data register 10        	     
		uint32_t ODR11:1;          	//!< Bit 11: GPIO port output data register 11        	   
		uint32_t ODR12:1;          	//!< Bit 12: GPIO port output data register 12        	    
		uint32_t ODR13:1;          	//!< Bit 13: GPIO port output data register 13        	   
		uint32_t ODR14:1;          	//!< Bit 14: GPIO port output data register 14        	    
		uint32_t ODR15:1;          	//!< Bit 15: GPIO port output data register 15        	   
		uint32_t RESERVED:16;		//!< Bits 16-31: Reserved 									 
	};
	uint32_t ALLBITS;				//!< GPIO port output data 32 bits register
} GPIOx_ODR_t;

/**
 * GPIO port bit set/reset register (GPIOx_BSRR) (x = A..E and H)
 * 
 * Bits 31:16 BRy: Port x reset bit y (y = 0..15). These bits are write-only and can be accessed in word, half-word or byte mode. A read to these bits returns the value 0x0000. Note: If both BSx and BRx are set, BSx has priority.
 *  - 0: No action on the corresponding ODRx bit
 *  - 1: Resets the corresponding ODRx bit
 * 
 * Bits 15:0 BSy: Port x set bit y (y= 0..15). These bits are write-only and can be accessed in word, half-word or byte mode. A read to these bits returns the value 0x0000.
 *  - 0: No action on the corresponding ODRx bit
 *  - 1: Sets the corresponding ODRx bit
 */
typedef union
{ 
    struct {
		uint32_t BS0:1;           	//!< Bit 0:  Port x reset bit 0       					   	 
		uint32_t BS1:1;           	//!< Bit 1:  Port x reset bit 1         				   	
		uint32_t BS2:1;           	//!< Bit 2:  Port x reset bit 2         				   	 
		uint32_t BS3:1;           	//!< Bit 3:  Port x reset bit 3         				   	
		uint32_t BS4:1;           	//!< Bit 4:  Port x reset bit 4         				   	 
		uint32_t BS5:1;           	//!< Bit 5:  Port x reset bit 5         				   	
		uint32_t BS6:1;           	//!< Bit 6:  Port x reset bit 6         				   	 
		uint32_t BS7:1;           	//!< Bit 7:  Port x reset bit 7         				   	
		uint32_t BS8:1;           	//!< Bit 8:  Port x reset bit 8         				   	 
		uint32_t BS9:1;           	//!< Bit 9:  Port x reset bit 9         				   	
		uint32_t BS10:1;           	//!< Bit 10: Port x reset bit 10        				    	 
		uint32_t BS11:1;           	//!< Bit 11: Port x reset bit 11        				   	
		uint32_t BS12:1;           	//!< Bit 12: Port x reset bit 12        				   	 
		uint32_t BS13:1;           	//!< Bit 13: Port x reset bit 13        				   	
		uint32_t BS14:1;           	//!< Bit 14: Port x reset bit 14        				   	 
		uint32_t BS15:1;           	//!< Bit 15: Port x reset bit 15        				   	
		uint32_t BR0:1;           	//!< Bit 16: Port x set bit 0       					    	 
		uint32_t BR1:1;           	//!< Bit 17: Port x set bit 1         					    	
		uint32_t BR2:1;           	//!< Bit 18: Port x set bit 2         					    	 
		uint32_t BR3:1;           	//!< Bit 19: Port x set bit 3         					   	
		uint32_t BR4:1;           	//!< Bit 20: Port x set bit 4         					   	 
		uint32_t BR5:1;           	//!< Bit 21: Port x set bit 5         					   	
		uint32_t BR6:1;           	//!< Bit 22: Port x set bit 6         					   	 
		uint32_t BR7:1;           	//!< Bit 23: Port x set bit 7         					   	
		uint32_t BR8:1;           	//!< Bit 24: Port x set bit 8         					   	 
		uint32_t BR9:1;           	//!< Bit 25: Port x set bit 9         					   	
		uint32_t BR10:1;           	//!< Bit 26: Port x set bit 10        					    	 
		uint32_t BR11:1;           	//!< Bit 27: Port x set bit 11        					   	
		uint32_t BR12:1;           	//!< Bit 28: Port x set bit 12        					   	 
		uint32_t BR13:1;           	//!< Bit 29: Port x set bit 13        					   	
		uint32_t BR14:1;           	//!< Bit 30: Port x set bit 14        					   	 
		uint32_t BR15:1;           	//!< Bit 31: Port x set bit 15        					   	
	};
	uint32_t ALLBITS;				//!< GPIO port set/reset 32 bits register
} GPIOx_BSRR_t;

/**
 * GPIO port configuration lock register (GPIOx_LCKR) (x = A..E and H)
 * 
 * This register is used to lock the configuration of the port bits when a correct write sequence
   is applied to bit 16 (LCKK). The value of bits [15:0] is used to lock the configuration of the
   GPIO. During the write sequence, the value of LCKR[15:0] must not change. When the
   LOCK sequence has been applied on a port bit, the value of this port bit can no longer be
   modified until the next MCU or peripheral reset. Note: A specific write sequence is used to write 
   to the GPIOx_LCKR register. Only word access (32-bit long) is allowed during this write sequence.
   Each lock bit freezes a specific configuration register (control and alternate function registers).

   Bits 31:17 Reserved, must be kept at reset value.

   Bit 16 LCKK[16]: Lock key. This bit can be read any time. It can only be modified using the lock key write sequence.
    - 0: Port configuration lock key not active
	- 1: Port configuration lock key active. The GPIOx_LCKR register is locked until an MCU reset or a peripheral reset occurs.

   LOCK key write sequence:
    -# WR LCKR[16] = ‘1’ + LCKR[15:0]
    -# WR LCKR[16] = ‘0’ + LCKR[15:0]
    -# WR LCKR[16] = ‘1’ + LCKR[15:0]
    -# RD LCKR
    -# RD LCKR[16] = ‘1’ (this read operation is optional but it confirms that the lock is active).

   Note: During the LOCK key write sequence, the value of LCK[15:0] must not change.
   Any error in the lock sequence aborts the lock. After the first lock sequence on any bit of the port, 
   any read access on the LCKK bit returns ‘1’ until the next CPU reset.

   Bits 15:0 LCKy: Port x lock bit y (y= 0..15). These bits are read/write but can only be written when the LCKK bit is ‘0.
    - 0: Port configuration not locked
	- 1: Port configuration locked   
 */
typedef union
{ 
    struct{
		uint32_t LCK0:1;           	//!< Bit 0: GPIO port configuration lock register 0     
		uint32_t LCK1:1;           	//!< Bit 1: GPIO port configuration lock register 1     
		uint32_t LCK2:1;           	//!< Bit 2: GPIO port configuration lock register 2      
		uint32_t LCK3:1;           	//!< Bit 3: GPIO port configuration lock register 3     
		uint32_t LCK4:1;           	//!< Bit 4: GPIO port configuration lock register 4      
		uint32_t LCK5:1;           	//!< Bit 5: GPIO port configuration lock register 5     
		uint32_t LCK6:1;           	//!< Bit 6: GPIO port configuration lock register 6      
		uint32_t LCK7:1;           	//!< Bit 7: GPIO port configuration lock register 7     
		uint32_t LCK8:1;           	//!< Bit 8: GPIO port configuration lock register 8      
		uint32_t LCK9:1;           	//!< Bit 9: GPIO port configuration lock register 9     
		uint32_t LCK10:1;           //!< Bit 10: GPIO port configuration lock register 10      
		uint32_t LCK11:1;           //!< Bit 11: GPIO port configuration lock register 11    
		uint32_t LCK12:1;           //!< Bit 12: GPIO port configuration lock register 12     
		uint32_t LCK13:1;           //!< Bit 13: GPIO port configuration lock register 13    
		uint32_t LCK14:1;           //!< Bit 14: GPIO port configuration lock register 14     
		uint32_t LCK15:1;           //!< Bit 15: GPIO port configuration lock register 15    
		uint32_t LCKK:1;           	//!< Bit 16: GPIO port configuration lock register K     
		uint32_t RESERVED:15;		//!< Bits 17-37: Reserved 									 
	};
	uint32_t ALLBITS;				//!< GPIO port configuration lock 32 bits register
} GPIOx_LCKR_t;

/**
 * GPIO alternate function low register (GPIOx_AFRL) (x = A..E and H)
 * 
 * Bits 31:0 AFRLy: Alternate function selection for port x bit y (y = 0..7). These bits are written by software to configure alternate function I/Os
 * 
 * AFRLy selection: 0000= AF0,  0001= AF1, ...  1111= AF15
 */
typedef union
{ 
    struct  {
		uint32_t AFRL0:4;         	//!< Bits 0-3:   GPIO alternate function low register 0 	 
		uint32_t AFRL1:4;         	//!< Bits 4-7:   GPIO alternate function low register 1  	 
		uint32_t AFRL2:4;         	//!< Bits 8-11:  GPIO alternate function low register 2  	  
		uint32_t AFRL3:4;         	//!< Bits 12-15: GPIO alternate function low register 3  	 
		uint32_t AFRL4:4;         	//!< Bits 16-19: GPIO alternate function low register 4  	  
		uint32_t AFRL5:4;         	//!< Bits 20-23: GPIO alternate function low register 5  	 
		uint32_t AFRL6:4;         	//!< Bits 24-27: GPIO alternate function low register 6  	  
		uint32_t AFRL7:4;         	//!< Bits 28-31: GPIO alternate function low register 7  	 	
	};
	uint32_t ALL_BITS;				//!< GPIO alternate function low 32 bits register
} GPIOx_AFRL_t;

/**
 * GPIO alternate function high register (GPIOx_AFRH) (x = A..E and H)
 * 
 * Bits 31:0 AFRHy: Alternate function selection for port x bit y (y = 0..7). These bits are written by software to configure alternate function I/Os
 * 
 * AFRHy selection: 0000= AF0,  0001= AF1, ...  1111= AF15
 */
typedef union
{ 
	struct {
		uint32_t AFRH0:4;       	//!< Bits 0-3:   GPIO alternate function high register 0  	 
		uint32_t AFRH1:4;       	//!< Bits 4-7:   GPIO alternate function high register 1   	
		uint32_t AFRH2:4;       	//!< Bits 8-11:  GPIO alternate function high register 2   	 
		uint32_t AFRH3:4;       	//!< Bits 12-15: GPIO alternate function high register 3   	
		uint32_t AFRH4:4;       	//!< Bits 16-19: GPIO alternate function high register 4   	 
		uint32_t AFRH5:4;       	//!< Bits 20-23: GPIO alternate function high register 5   	
		uint32_t AFRH6:4;       	//!< Bits 24-27: GPIO alternate function high register 6   	 
		uint32_t AFRH7:4;       	//!< Bits 28-31: GPIO alternate function high register 7   		
	};
	uint32_t ALL_BITS;				//!< GPIO alternate function high 32 bits register
} GPIOx_AFRH_t;

/**
 * Peripheral register definition structure for GPIO
 */
typedef struct
{
	volatile GPIOx_MODER_t 		MODER;		//!< Address offset 0x00: GPIO port mode register 					
	volatile GPIOx_OTYPER_t 	OTYPER;		//!< Address offset 0x04: GPIO port output type register				
	volatile GPIOx_OSPEEDR_t	OSPEEDR;	//!< Address offset 0x08: GPIO port output speed register 			
	volatile GPIOx_PUPDR_t 		PUPDR;		//!< Address offset 0x0C: GPIO port pull-up/pull-down register		
	volatile GPIOx_IDR_t 		IDR;		//!< Address offset 0x10: GPIO port input data register 				
	volatile GPIOx_ODR_t 		ODR;		//!< Address offset 0x14: GPIO port output data register				
	volatile GPIOx_BSRR_t 		BSRR;		//!< Address offset 0x18: GPIO port bit set/reset register			
	volatile GPIOx_LCKR_t 		LCKR;		//!< Address offset 0x1C: GPIO port configuration lock register		
	volatile GPIOx_AFRL_t 		AFRL;		//!< Address offset 0x20: Alternate function low register			
	volatile GPIOx_AFRH_t 		AFRH;		//!< Address offset 0x24: Alternate function high register			
} gpio_reg_def_t;

/**
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

/**
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

/**
 * GPIO pin possible output speeds
 */
typedef enum {
	GPIO_OUTPUT_SPEED_LOW,					
	GPIO_OUTPUT_SPEED_MEDIUM,				
	GPIO_OUTPUT_SPEED_HIGH,			
	GPIO_OUTPUT_SPEED_VERY_HIGH
} gpio_pin_speed_t;

/**
 * GPIO pin pull-up and pull-down modes
 */
typedef enum {
	GPIO_PIN_NO_PULLUP_PULLDOWN,					
	GPIO_PIN_PULL_UP,				
	GPIO_PIN_PULL_DOWN
} gpio_pin_pu_pd_control_t;

/**
 * GPIO pin possible output types
 */
typedef enum {
	GPIO_OUTPUT_TYPE_PUSH_PULL,					
	GPIO_OUTPUT_TYPE_OPEN_DRAIN
} gpio_pin_out_type_t;

/**
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

/**
 * This is a configuration structure for a GPIO pin
 */
typedef struct
{
	gpio_pin_number_t 			gpio_pin_number;		//!< @see gpio_pin_number_t typedef enum
	gpio_pin_mode_t 			gpio_pin_mode;			//!< @see gpio_pin_mode_t typedef enum
	gpio_pin_speed_t 			gpio_pin_speed;			//!< @see gpio_pin_speed_t typedef enum
	gpio_pin_pu_pd_control_t	gpio_pin_pu_pd_control;	//!< @see gpio_pin_pu_pd_control_t typedef enum
	gpio_pin_out_type_t 		gpio_pin_out_type;		//!< @see gpio_pin_out_type_t typedef enum
	gpio_pin_alt_fun_mode_t 	gpio_pin_alt_fun_mode;	//!< @see gpio_pin_alt_fun_mode_t typedef enum
} gpio_pin_config_t;

/**
 * This is a handle structure for GPIO pin
 */
typedef struct
{
	gpio_reg_def_t 		*p_gpio_x;			//!< This holds the base address of the GPIO port which the pins belongs 
	gpio_pin_config_t	gpio_pin_config;	//!< This holds GPIO pin configuration settings 
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
