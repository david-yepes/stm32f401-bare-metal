/**
 * @file stm32f401xx.h
 * @author David Yepes
 * @brief 
 * @version 0.1
 * @date 2025-10-31
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#ifndef INC_STM32F401XX_H_
#define INC_STM32F401XX_H_

#include <stdint.h>

/*  ******************************** START Processor Specific Details ********************************************************
 * These register are described in the Cortex M4 devices Generic User Guide
 *
 * ARM Cortex Mx Processor Nested Vectored Interrupt Controller (NVIC) ISERx register addresses
 */
#define NVIC_ISER0				( (volatile uint32_t*)0xE000E100 )	//!< NVIC Interrupt Set-enable Register 0 defintion 
#define NVIC_ISER1				( (volatile uint32_t*)0xE000E104 )	//!< NVIC Interrupt Set-enable Register 1 defintion 
#define NVIC_ISER2				( (volatile uint32_t*)0xE000E108 )	//!< NVIC Interrupt Set-enable Register 2 defintion 
#define NVIC_ISER3				( (volatile uint32_t*)0xE000E10C )	//!< NVIC Interrupt Set-enable Register 3 defintion 

/*
 * ARM Cortex Mx Processor NVIC ICERx register addresses
 */
#define NVIC_ICER0				( (volatile uint32_t*)0XE000E180 )	//!< NVIC Interrupt Clear-enable Register 0 defintion 
#define NVIC_ICER1				( (volatile uint32_t*)0xE000E184 )	//!< NVIC Interrupt Clear-enable Register 1 defintion
#define NVIC_ICER2				( (volatile uint32_t*)0xE000E188 )	//!< NVIC Interrupt Clear-enable Register 2 defintion
#define NVIC_ICER3				( (volatile uint32_t*)0xE000E18C )	//!< NVIC Interrupt Clear-enable Register 3 defintion

/*
 * ARM Cortex Mx Processor Priority Register addresses
 */
#define NVIC_PRIORITY_BASE_ADDRESS	( (volatile uint32_t*)0xE000E400 )	//!< NVIC Interrupt Priority Registers base address defintion

/**
 * ARM Cortex Mx Processor number of priority bits implemented in NVIC Priority Register
 */
#define PRIORITY_BITS_IMPLEMENTED_NUM	4
/*********************************** END Processor Specific Details *********************************************************/

/*
 * Base addresses of Flash and SRAM memories
 */

#define FLASH_BASEADDR			0x08000000U		//!< Main memory block base address 
#define SRAM1_BASEADDR			0x20000000U		//!< Embedded SRAM1 base address 
#define ROM_BASEADDR			0x1FFF0000U		//!< System memory base address 
#define SRAM 					SRAM1_BASEADDR	//!< Embedded SRAM address

/*
 * AHBx and APBx Bus Peripheral base addresses
 */

#define PERIPH_BASEADDR			0x40000000U		//!< Peripheral base address
#define APB1PERIPH_BASEADDR		PERIPH_BASEADDR	//!< Advanced Peripheral Bus 1 base address
#define APB2PERIPH_BASEADDR		0x40010000U		//!< Advanced Peripheral Bus 2 base address
#define AHB1PERIPH_BASEADDR		0x40020000U		//!< Advanced High-performance Bus 1 base address
#define AHB2PERIPH_BASEADDR		0x50000000U		//!< Advanced High-performance Bus 2 base address

/*
 * Base addresses of peripherals which are hanging on AHB1 bus
 */

#define GPIOA_BASEADDR			(AHB1PERIPH_BASEADDR + 0x0000)	//!< General-purpose I/O port A (GPIOA) peripheral base address
#define GPIOB_BASEADDR			(AHB1PERIPH_BASEADDR + 0x0400)	//!< General-purpose I/O port B (GPIOB) peripheral base address
#define GPIOC_BASEADDR			(AHB1PERIPH_BASEADDR + 0x0800)	//!< General-purpose I/O port C (GPIOC) peripheral base address
#define GPIOD_BASEADDR			(AHB1PERIPH_BASEADDR + 0x0C00)	//!< General-purpose I/O port D (GPIOD) peripheral base address
#define GPIOE_BASEADDR			(AHB1PERIPH_BASEADDR + 0x1000)	//!< General-purpose I/O port E (GPIOE) peripheral base address
#define GPIOH_BASEADDR			(AHB1PERIPH_BASEADDR + 0x1C00)	//!< General-purpose I/O port H (GPIOH) peripheral base address
#define CRC_BASEADDR			(AHB1PERIPH_BASEADDR + 0x3000)	//!< Cyclic Redundancy Check calculation unit peripheral base address
#define RCC_BASEADDR			(AHB1PERIPH_BASEADDR + 0x3800)	//!< Reset and clock control peripheral base address
#define FLASREG_BASEADDR		(AHB1PERIPH_BASEADDR + 0x3C00)	//!< Flash interface register peripheral base address
#define DMA1_BASEADDR			(AHB1PERIPH_BASEADDR + 0x6000)	//!< Direct memory access 1 peripheral base address
#define DMA2_BASEADDR			(AHB1PERIPH_BASEADDR + 0x6400)  //!< Direct memory access 2 peripheral base address

/*
 * Base addresses of peripherals which are hanging on APB1 bus
 */

#define TIM2_BASEADDR			(APB1PERIPH_BASEADDR + 0x0000)	//!< General-purpose timer 2 (TIM2) peripheral base address
#define TIM3_BASEADDR			(APB1PERIPH_BASEADDR + 0x0400)	//!< General-purpose timer 3 (TIM3) peripheral base address
#define TIM4_BASEADDR			(APB1PERIPH_BASEADDR + 0x0800)	//!< General-purpose timer 4 (TIM4) peripheral base address
#define TIM5_BASEADDR			(APB1PERIPH_BASEADDR + 0x0C00)	//!< General-purpose timer 5 (TIM5) peripheral base address
#define RTCREG_BASEADDR			(APB1PERIPH_BASEADDR + 0x2800)	//!< Real-time clock peripheral base address
#define WWDG_BASEADDR			(APB1PERIPH_BASEADDR + 0x2C00)	//!< Window watchdog peripheral base address
#define IWDG_BASEADDR			(APB1PERIPH_BASEADDR + 0x3000)	//!< Independent watchdog peripheral base address
#define I2S2EXT_BASEADDR		(APB1PERIPH_BASEADDR + 0x3400)	//!< Inter-IC Sound Extension 2 (I2S2ext) peripheral base address
#define SPI2_I2S2_BASEADDR		(APB1PERIPH_BASEADDR + 0x3800)	//!< Serial Peripheral Interface 2 / Inter-IC Sound 2 (SPI2_I2S2) peripheral base address
#define SPI3_I2S3_BASEADDR		(APB1PERIPH_BASEADDR + 0x3C00)	//!< Serial Peripheral Interface 3 / Inter-IC Sound 3 (SPI3_I2S3) peripheral base address
#define I2S3EXT_BASEADDR		(APB1PERIPH_BASEADDR + 0x4000)	//!< Inter-IC Sound Extension 3 (I2S3ext) peripheral base address
#define USART2_BASEADDR			(APB1PERIPH_BASEADDR + 0x4400)	//!< Universal Synchronous/Asynchronous Receiver/Transmitter 2 (USART2) peripheral base address
#define I2C1_BASEADDR			(APB1PERIPH_BASEADDR + 0x5400)	//!< Inter-Integrated Circuit 1 (I2C1) peripheral base address
#define I2C2_BASEADDR			(APB1PERIPH_BASEADDR + 0x5800)	//!< Inter-Integrated Circuit 2 (I2C2) peripheral base address
#define I2C3_BASEADDR			(APB1PERIPH_BASEADDR + 0x5C00)	//!< Inter-Integrated Circuit 3 (I2C3) peripheral base address
#define PWR_BASEADDR			(APB1PERIPH_BASEADDR + 0x7000)	//!< Power control (PWR) peripheral base address

/*
 * Base addresses of peripherals which are hanging on APB2 bus
 */

#define TIM1_BASEADDR			(APB2PERIPH_BASEADDR + 0x0000)	//!< General-purpose timer 1 (TIM1) peripheral base address
#define USART1_BASEADDR			(APB2PERIPH_BASEADDR + 0x1000)	//!< Universal Synchronous/Asynchronous Receiver/Transmitter 1 (USART1) peripheral base address
#define USART6_BASEADDR			(APB2PERIPH_BASEADDR + 0x1400)	//!< Universal Synchronous/Asynchronous Receiver/Transmitter 6 (USART6) peripheral base address
#define ADC1_BASEADDR			(APB2PERIPH_BASEADDR + 0x2000)	//!< Analog-to-digital converter 1 (ADC1) peripheral base address
#define SDIO_BASEADDR			(APB2PERIPH_BASEADDR + 0x2C00)	//!< Secure digital input/output interface (SDIO) peripheral base address
#define SPI1_BASEADDR			(APB2PERIPH_BASEADDR + 0x3000)	//!< Serial Peripheral Interface 1 (SPI1) peripheral base address
#define SPI4_BASEADDR			(APB2PERIPH_BASEADDR + 0x3400)	//!< Serial Peripheral Interface 4 (SPI4) peripheral base address
#define SYSCFG_BASEADDR	 		(APB2PERIPH_BASEADDR + 0x3800)	//!< System configuration controller (SYSCFG) peripheral base address
#define EXTI_BASEADDR			(APB2PERIPH_BASEADDR + 0x3C00)	//!< External interrupt/event controller (EXTI) peripheral base address
#define TIM9_BASEADDR			(APB2PERIPH_BASEADDR + 0x4000)	//!< General-purpose timer 9 (TIM9) peripheral base address	
#define TIM10_BASEADDR			(APB2PERIPH_BASEADDR + 0x4400)	//!< General-purpose timer 10 (TIM10) peripheral base address
#define TIM11_BASEADDR			(APB2PERIPH_BASEADDR + 0x4800)	//!< General-purpose timer 11 (TIM11) peripheral base address

/**************************************************************************************
 *
 * Peripheral register definition structures
 *
 **************************************************************************************/

/**
 * Peripheral register definition structure for RCC
 */
typedef struct
{
	volatile uint32_t CR;			//! RCC clock control register 									Address offset: 0x00
	volatile uint32_t PLLCFGR;		//! RCC PLL configuration register								Address offset: 0x04
	volatile uint32_t CFGR;			//! RCC clock configuration register 							Address offset: 0x08
	volatile uint32_t CIR;			//! RCC clock interrupt register								Address offset: 0x0C
	volatile uint32_t AHB1RSTR;		//! RCC AHB1 peripheral reset register 							Address offset: 0x10
	volatile uint32_t AHB2RSTR;		//! RCC AHB2 peripheral reset register							Address offset: 0x14
	 	 	 uint32_t RESERVED0;	//! Reserved													Address offset: 0x18
	 	 	 uint32_t RESERVED1;	//! Reserved													Address offset: 0x1C
	volatile uint32_t APB1RSTR;		//! RCC APB1 peripheral reset register							Address offset: 0x20
	volatile uint32_t APB2RSTR;		//! RCC APB2 peripheral reset register							Address offset: 0x24
	 	 	 uint32_t RESERVED2;	//! Reserved													Address offset: 0x28
	 	 	 uint32_t RESERVED3;	//! Reserved													Address offset: 0x2C
	volatile uint32_t AHB1ENR;		//! RCC AHB1 peripheral clock enable register					Address offset: 0x30
	volatile uint32_t AHB2ENR;		//! RCC AHB2 peripheral clock enable register					Address offset: 0x34
	 	 	 uint32_t RESERVED4;	//! Reserved 													Address offset: 0x38
	 	 	 uint32_t RESERVED5;	//! Reserved													Address offset: 0x3C
	volatile uint32_t APB1ENR;		//! RCC APB1 peripheral clock enable register 					Address offset: 0x40
	volatile uint32_t APB2ENR;		//! RCC APB2 peripheral clock enable register					Address offset: 0x44
	 	 	 uint32_t RESERVED6;	//! Reserved													Address offset: 0x48
	 	 	 uint32_t RESERVED7;	//! Reserved													Address offset: 0x4C
	volatile uint32_t AHB1LPENR;	//! RCC AHB1 peripheral clock enable in low power mode register	Address offset: 0x50
	volatile uint32_t AHB2LPENR;	//! RCC AHB2 peripheral clock enable in low power mode register	Address offset: 0x54
	 	 	 uint32_t RESERVED8;	//! Reserved													Address offset: 0x58
	 	 	 uint32_t RESERVED9;	//! Reserved													Address offset: 0x5C
	volatile uint32_t APB1LPENR;	//! RCC APB1 peripheral clock enable in low power mode register Address offset: 0x60
	volatile uint32_t APB2LPENR;	//! RCC APB2 peripheral clock enable in low power mode register	Address offset: 0x64
	 	 	 uint32_t RESERVED10;	//! Reserved 													Address offset: 0x68
	 	 	 uint32_t RESERVED11;	//! Reserved													Address offset: 0x6C
	volatile uint32_t BDCR;			//! RCC Backup domain control register							Address offset: 0x70
	volatile uint32_t CSR;			//! RCC clock control & status register							Address offset: 0x74
	 	 	 uint32_t RESERVED12;	//! Reserved													Address offset: 0x78
	 	 	 uint32_t RESERVED13;	//! Reserved													Address offset: 0x7C
	volatile uint32_t SSCGR;		//! RCC spread spectrum clock generation register				Address offset: 0x80
	volatile uint32_t PLLI2SCFGR;	//! RCC PLLI2S configuration register							Address offset: 0x84
	 	 	 uint32_t RESERVED14;	//! Reserved													Address offset: 0x88
	volatile uint32_t DCKCFGR;		//! RCC Dedicated Clocks Configuration Register					Address offset: 0x8C
} RCC_RegDef_t;

/**
 * Peripheral register definition structure for EXTI
 */
typedef struct
{
	volatile uint32_t IMR;			//!< Interrupt mask register									Address offset: 0x00
	volatile uint32_t EMR;			//!< Event mask register										Address offset: 0x04 
	volatile uint32_t RTSR;			//!< Rising trigger selection register							Address offset: 0x08 
	volatile uint32_t FTSR;			//!< Falling trigger selection register							Address offset: 0x0C 
	volatile uint32_t SWIER;		//!< Software interrupt event register							Address offset: 0x10 
	volatile uint32_t PR;			//!< Pending register											Address offset: 0x14 
} EXTI_RegDef_t;

/**
 * Peripheral register definition structure for SYSCFG
 */
typedef struct
{
	volatile uint32_t MEMRMP;		//!< SYSCFG memory remap register								Address offset: 0x00
	volatile uint32_t PMC;			//!< SYSCFG peripheral mode configuration register				Address offset: 0x04
	volatile uint32_t EXTICR[4];	//!< SYSCFG external interrupt configuration registers 1 - 4	Address offset: 0x08, 0x0C, 0x10 and 0x14
	         uint32_t RESERVED[2];	//!< Reserved													Address offset: 0x18 and 0x1C
	volatile uint32_t CMPCR;		//!< Compensation cell control register							Address offset: 0x20
} SYSCFG_RegDef_t;

/*
 *  Peripheral definitions (Peripheral base addresses typecasted to xxx_RegDef_t)
 */

#define RCC		( (RCC_RegDef_t*)RCC_BASEADDR )			//!< RCC peripheral definition: Peripheral base addresses typecasted to RCC_RegDef_t
#define EXTI	( (EXTI_RegDef_t*)EXTI_BASEADDR )		//!< EXTI peripheral definition: Peripheral base addresses typecasted to EXTI_RegDef_t
#define SYSCFG	( (SYSCFG_RegDef_t*)SYSCFG_BASEADDR )	//!< SYSCFG peripheral definition: Peripheral base addresses typecasted to SYSCFG_RegDef_t

/*
 * Clock Enable Macros for GPIOx peripherals
 */

#define GPIOA_PERI_CLK_EN()	( RCC->AHB1ENR |= (1 << 0) )	//!< Clock Enable Macro for GPIOA peripheral
#define GPIOB_PERI_CLK_EN()	( RCC->AHB1ENR |= (1 << 1) )	//!< Clock Enable Macro for GPIOB peripheral
#define GPIOC_PERI_CLK_EN()	( RCC->AHB1ENR |= (1 << 2) )	//!< Clock Enable Macro for GPIOC peripheral
#define GPIOD_PERI_CLK_EN()	( RCC->AHB1ENR |= (1 << 3) )	//!< Clock Enable Macro for GPIOD peripheral
#define GPIOE_PERI_CLK_EN()	( RCC->AHB1ENR |= (1 << 4) )	//!< Clock Enable Macro for GPIOE peripheral
#define GPIOH_PERI_CLK_EN()	( RCC->AHB1ENR |= (1 << 7) )	//!< Clock Enable Macro for GPIOH peripheral

/*
 * Clock Enable Macros for I2Cx peripherals
 */

#define I2C1_PERI_CLK_EN()	( RCC->APB1ENR |= (1 << 21) )	//!< Clock Enable Macro for I2C1 peripheral
#define I2C2_PERI_CLK_EN()	( RCC->APB1ENR |= (1 << 22) )	//!< Clock Enable Macro for I2C2 peripheral
#define I2C3_PERI_CLK_EN()	( RCC->APB1ENR |= (1 << 23) )	//!< Clock Enable Macro for I2C3 peripheral

/*
 * Clock Enable Macros for SPIx peripherals
 *
 */

#define SPI1_PERI_CLK_EN()	( RCC->APB2ENR |= (1 << 12) )	//!< Clock Enable Macro for SPI1 peripheral
#define SPI2_PERI_CLK_EN()	( RCC->APB1ENR |= (1 << 14) )	//!< Clock Enable Macro for SPI2 peripheral
#define SPI3_PERI_CLK_EN()	( RCC->APB1ENR |= (1 << 15) )	//!< Clock Enable Macro for SPI3 peripheral
#define SPI4_PERI_CLK_EN()	( RCC->APB2ENR |= (1 << 13) )	//!< Clock Enable Macro for SPI4 peripheral

/*
 * Clock Enable Macros for USARTx peripherals
 */

#define USART1_PERI_CLK_EN()	( RCC->APB2ENR |= (1 << 4) )	//!< Clock Enable Macro for USART1 peripheral
#define USART2_PERI_CLK_EN()	( RCC->APB1ENR |= (1 << 17) )	//!< Clock Enable Macro for USART2 peripheral
#define USART6_PERI_CLK_EN()	( RCC->APB2ENR |= (1 << 5) )	//!< Clock Enable Macro for USART6 peripheral

/*
 * Clock Enable Macros for SYSCFG peripheral
 */

#define SYSCFG_PERI_CLK_EN()	( RCC->APB2ENR |= (1 << 14) )	//!< Clock Enable Macro for SYSCFG peripheral

/*
 * Clock Disable Macros for GPIOx peripherals
 */

#define GPIOA_PERI_CLK_DI()	( RCC->AHB1ENR &= ~(1 << 0) )	//!< Clock Disable Macro for GPIOA peripheral
#define GPIOB_PERI_CLK_DI()	( RCC->AHB1ENR &= ~(1 << 1) )	//!< Clock Disable Macro for GPIOB peripheral
#define GPIOC_PERI_CLK_DI()	( RCC->AHB1ENR &= ~(1 << 2) )	//!< Clock Disable Macro for GPIOC peripheral
#define GPIOD_PERI_CLK_DI()	( RCC->AHB1ENR &= ~(1 << 3) )	//!< Clock Disable Macro for GPIOD peripheral
#define GPIOE_PERI_CLK_DI()	( RCC->AHB1ENR &= ~(1 << 4) )	//!< Clock Disable Macro for GPIOE peripheral
#define GPIOH_PERI_CLK_DI()	( RCC->AHB1ENR &= ~(1 << 7) )	//!< Clock Disable Macro for GPIOH peripheral

/*
 * Clock Disable Macros for I2Cx peripherals
 */

#define I2C2_PERI_CLK_DI()	( RCC->APB1ENR &= ~(1 << 22) )	//!< Clock Disable Macro for I2C2 peripheral
#define I2C3_PERI_CLK_DI()	( RCC->APB1ENR &= ~(1 << 23) )	//!< Clock Disable Macro for I2C3 peripheral
#define I2C1_PERI_CLK_DI()	( RCC->APB1ENR &= ~(1 << 21) )	//!< Clock Disable Macro for I2C1 peripheral

/*
 * Clock Disable Macros for SPIx peripherals
 *
 */

#define SPI1_PERI_CLK_DI()	( RCC->APB2ENR &= ~(1 << 12) )	//!< Clock Disable Macro for SPI1 peripheral
#define SPI2_PERI_CLK_DI()	( RCC->APB1ENR &= ~(1 << 14) )	//!< Clock Disable Macro for SPI2 peripheral
#define SPI3_PERI_CLK_DI()	( RCC->APB1ENR &= ~(1 << 15) )	//!< Clock Disable Macro for SPI3 peripheral
#define SPI4_PERI_CLK_DI()	( RCC->APB2ENR &= ~(1 << 13) )	//!< Clock Disable Macro for SPI4 peripheral

/*
 * Clock Disable Macros for USARTx peripherals
 */

#define USART1_PERI_CLK_DI()	( RCC->APB2ENR &= ~(1 << 4) )	//!< Clock Disable Macro for USART1 peripheral
#define USART2_PERI_CLK_DI()	( RCC->APB1ENR &= ~(1 << 17) )	//!< Clock Disable Macro for USART2 peripheral
#define USART6_PERI_CLK_DI()	( RCC->APB2ENR &= ~(1 << 5) )	//!< Clock Disable Macro for USART6 peripheral

/*
 * Clock Disable Macros for SYSCFG peripheral
 */

#define SYSCFG_PERI_CLK_DI()	( RCC->APB2ENR &= ~(1 << 14) )	//!< Clock Disable Macro for SYSCFG peripheral

/*
 * Macros to reset GPIOx peripherals
 */
// do ... while ... condition zero loop: This is technique in C programming to execute
// multiple C statements using a single C macro
#define GPIOA_REG_RESET()		do{ (RCC->AHB1RSTR |= (1 << 0)); (RCC->AHB1RSTR &= ~(1 << 0)); } while(0)	//!< Macro to reset GPIOA peripheral
#define GPIOB_REG_RESET()		do{ (RCC->AHB1RSTR |= (1 << 1)); (RCC->AHB1RSTR &= ~(1 << 1)); } while(0)	//!< Macro to reset GPIOB peripheral
#define GPIOC_REG_RESET()		do{ (RCC->AHB1RSTR |= (1 << 2)); (RCC->AHB1RSTR &= ~(1 << 2)); } while(0)	//!< Macro to reset GPIOC peripheral
#define GPIOD_REG_RESET()		do{ (RCC->AHB1RSTR |= (1 << 3)); (RCC->AHB1RSTR &= ~(1 << 3)); } while(0)	//!< Macro to reset GPIOD peripheral
#define GPIOE_REG_RESET()		do{ (RCC->AHB1RSTR |= (1 << 4)); (RCC->AHB1RSTR &= ~(1 << 4)); } while(0)	//!< Macro to reset GPIOE peripheral
#define GPIOH_REG_RESET()		do{ (RCC->AHB1RSTR |= (1 << 7)); (RCC->AHB1RSTR &= ~(1 << 7)); } while(0)	//!< Macro to reset GPIOH peripheral

/**
 * This macro returns a code (between 0 to 4 and 7) for a given GPIO base address (x)
 */
#define GPIO_BASEADDR_TO_INT_SRC_PORT_CODE(x)	( (x == GPIOA) ? 0 :\
									  	  	  	  (x == GPIOB) ? 1 :\
									  	  	  	  (x == GPIOC) ? 2 :\
									  	  	  	  (x == GPIOD) ? 3 :\
									  	  	  	  (x == GPIOE) ? 4 :\
									              (x == GPIOH) ? 7 : 0 )

/*
 * IRQ (Interrupt Request) Number of STM32F401x MCU, see Vector table in the reference manual
 */
#define IRQ_NO_EXTI0			6	//!< Interrupt request number of EXTI0 line (GPIO0) 
#define IRQ_NO_EXTI1			7	//!< Interrupt request number of EXTI1 line (GPIO1) 
#define IRQ_NO_EXTI2			8	//!< Interrupt request number of EXTI2 line (GPIO2) 
#define IRQ_NO_EXTI3			9	//!< Interrupt request number of EXTI3 line (GPIO3) 
#define IRQ_NO_EXTI4			10	//!< Interrupt request number of EXTI4 line (GPIO4) 
#define IRQ_NO_EXTI9_5			23	//!< Interrupt request number of EXTI9_5 lines (GPIO5 - GPIO9) 
#define IRQ_NO_EXTI15_10		40	//!< Interrupt request number of EXTI15_10 lines (GPIO10 - GPIO15) 
#define IRQ_NO_SPI1				35	//!< Interrupt request number of SPI1 
#define IRQ_NO_SPI2				36	//!< Interrupt request number of SPI2
#define IRQ_NO_SPI3				51	//!< Interrupt request number of SPI3
#define IRQ_NO_SPI4				84	//!< Interrupt request number of SPI4

/*
 * IRQ (Interrupt Request) Priority levels
 */
#define NVIC_IRQ_PRIORITY_0		0
#define NVIC_IRQ_PRIORITY_1		1
#define NVIC_IRQ_PRIORITY_2		2
#define NVIC_IRQ_PRIORITY_3		3
#define NVIC_IRQ_PRIORITY_4		4
#define NVIC_IRQ_PRIORITY_5		5
#define NVIC_IRQ_PRIORITY_6		6
#define NVIC_IRQ_PRIORITY_7		7
#define NVIC_IRQ_PRIORITY_8		8
#define NVIC_IRQ_PRIORITY_9		9
#define NVIC_IRQ_PRIORITY_10	10
#define NVIC_IRQ_PRIORITY_11	11
#define NVIC_IRQ_PRIORITY_12	12
#define NVIC_IRQ_PRIORITY_13	13
#define NVIC_IRQ_PRIORITY_14	14
#define NVIC_IRQ_PRIORITY_15	15

// Some generic macros

#define ENABLE 			1
#define DISABLE 		0
#define SET 			ENABLE
#define RESET 			DISABLE
#define GPIO_PIN_SET	SET
#define GPIO_RESET		RESET


#endif /* INC_STM32F401XX_H_ */
