/*
 * stm32f401xx.h
 *
 *  Created on: Aug 28, 2025
 *      Author: David Yepes
 */

#ifndef INC_STM32F401XX_H_
#define INC_STM32F401XX_H_

#include <stdint.h>

/*********************************** START Processor Specific Details ********************************************************
 * These register are described in the Cortex M4 devices Generic User Guide
 *
 * ARM Cortex Mx Processor NVIC ISERx register addresses
 */
#define NVIC_ISER0				( (volatile uint32_t*)0xE000E100 )
#define NVIC_ISER1				( (volatile uint32_t*)0xE000E104 )
#define NVIC_ISER2				( (volatile uint32_t*)0xE000E108 )
#define NVIC_ISER3				( (volatile uint32_t*)0xE000E10C )

/*
 * ARM Cortex Mx Processor NVIC ICERx register addresses
 */
#define NVIC_ICER0				( (volatile uint32_t*)0XE000E180 )
#define NVIC_ICER1				( (volatile uint32_t*)0xE000E184 )
#define NVIC_ICER2				( (volatile uint32_t*)0xE000E188 )
#define NVIC_ICER3				( (volatile uint32_t*)0xE000E18C )

/*
 * ARM Cortex Mx Processor Priority Register addresses
 */
#define NVIC_PRIORITY_BASE_ADDRESS	( (volatile uint32_t*)0xE000E400 )

/*
 * ARM Cortex Mx Processor number of priority bits implemented in NVIC Priority Register
 */
#define PRIORITY_BITS_IMPLEMENTED_NUM	4
/*********************************** END Processor Specific Details *********************************************************/

/*
 * Base addresses of Flash and SRAM memories
 */

#define FLASH_BASEADDR			0x08000000U		/* Main memory block base address */
#define SRAM1_BASEADDR			0x20000000U		/* Embedded SRAM1 base address */
#define ROM_BASEADDR			0x1FFF0000U		/* System memory base address */
#define SRAM 					SRAM1_BASEADDR

/*
 * AHBx and APBx Bus Peripheral base addresses
 */

#define PERIPH_BASEADDR			0x40000000U		/* Peripheral base address */
#define APB1PERIPH_BASEADDR		PERIPH_BASEADDR	/* Advanced Peripheral Bus 1 base address */
#define APB2PERIPH_BASEADDR		0x40010000U		/* Advanced Peripheral Bus 2 base address */
#define AHB1PERIPH_BASEADDR		0x40020000U		/* Advanced High-performance Bus 1 base address */
#define AHB2PERIPH_BASEADDR		0x50000000U		/* Advanced High-performance Bus 2 base address */

/*
 * Base addresses of peripherals which are hanging on AHB1 bus
 */

#define GPIOA_BASEADDR			(AHB1PERIPH_BASEADDR + 0x0000)
#define GPIOB_BASEADDR			(AHB1PERIPH_BASEADDR + 0x0400)
#define GPIOC_BASEADDR			(AHB1PERIPH_BASEADDR + 0x0800)
#define GPIOD_BASEADDR			(AHB1PERIPH_BASEADDR + 0x0C00)
#define GPIOE_BASEADDR			(AHB1PERIPH_BASEADDR + 0x1000)
#define GPIOH_BASEADDR			(AHB1PERIPH_BASEADDR + 0x1C00)
#define CRC_BASEADDR			(AHB1PERIPH_BASEADDR + 0x3000)
#define RCC_BASEADDR			(AHB1PERIPH_BASEADDR + 0x3800)
#define FLASREG_BASEADDR		(AHB1PERIPH_BASEADDR + 0x3C00)
#define DMA1_BASEADDR			(AHB1PERIPH_BASEADDR + 0x6000)
#define DMA2_BASEADDR			(AHB1PERIPH_BASEADDR + 0x6400)

/*
 * Base addresses of peripherals which are hanging on APB1 bus
 */

#define TIM2_BASEADDR			(APB1PERIPH_BASEADDR + 0x0000)
#define TIM3_BASEADDR			(APB1PERIPH_BASEADDR + 0x0400)
#define TIM4_BASEADDR			(APB1PERIPH_BASEADDR + 0x0800)
#define TIM5_BASEADDR			(APB1PERIPH_BASEADDR + 0x0C00)
#define RTCREG_BASEADDR			(APB1PERIPH_BASEADDR + 0x2800)
#define WWDG_BASEADDR			(APB1PERIPH_BASEADDR + 0x2C00)
#define IWDG_BASEADDR			(APB1PERIPH_BASEADDR + 0x3000)
#define I2S2EXT_BASEADDR		(APB1PERIPH_BASEADDR + 0x3400)
#define SPI2_I2S2_BASEADDR		(APB1PERIPH_BASEADDR + 0x3800)
#define SPI3_I2S3_BASEADDR		(APB1PERIPH_BASEADDR + 0x3C00)
#define I2S3EXT_BASEADDR		(APB1PERIPH_BASEADDR + 0x4000)
#define USART2_BASEADDR			(APB1PERIPH_BASEADDR + 0x4400)
#define I2C1_BASEADDR			(APB1PERIPH_BASEADDR + 0x5400)
#define I2C2_BASEADDR			(APB1PERIPH_BASEADDR + 0x5800)
#define I2C3_BASEADDR			(APB1PERIPH_BASEADDR + 0x5C00)
#define PWR_BASEADDR			(APB1PERIPH_BASEADDR + 0x7000)

/*
 * Base addresses of peripherals which are hanging on APB2 bus
 */

#define TIM1_BASEADDR			(APB2PERIPH_BASEADDR + 0x0000)
#define USART1_BASEADDR			(APB2PERIPH_BASEADDR + 0x1000)
#define USART6_BASEADDR			(APB2PERIPH_BASEADDR + 0x1400)
#define ADC1_BASEADDR			(APB2PERIPH_BASEADDR + 0x2000)
#define SDIO_BASEADDR			(APB2PERIPH_BASEADDR + 0x2C00)
#define SPI1_BASEADDR			(APB2PERIPH_BASEADDR + 0x3000)
#define SPI4_BASEADDR			(APB2PERIPH_BASEADDR + 0x3400)
#define SYSCFG_BASEADDR	 		(APB2PERIPH_BASEADDR + 0x3800)
#define EXTI_BASEADDR			(APB2PERIPH_BASEADDR + 0x3C00)
#define TIM9_BASEADDR			(APB2PERIPH_BASEADDR + 0x4000)
#define TIM10_BASEADDR			(APB2PERIPH_BASEADDR + 0x4400)
#define TIM11_BASEADDR			(APB2PERIPH_BASEADDR + 0x4800)

/**************************************************************************************
 *
 * Peripheral register definition structures
 *
 **************************************************************************************/

/*
 * Peripheral register definition structure for RCC
 */
typedef struct
{
	volatile uint32_t CR;			/* RCC clock control register 									Address offset: 0x00 */
	volatile uint32_t PLLCFGR;		/* RCC PLL configuration register								Address offset: 0x04 */
	volatile uint32_t CFGR;			/* RCC clock configuration register 							Address offset: 0x08 */
	volatile uint32_t CIR;			/* RCC clock interrupt register									Address offset: 0x0C */
	volatile uint32_t AHB1RSTR;		/* RCC AHB1 peripheral reset register 							Address offset: 0x10 */
	volatile uint32_t AHB2RSTR;		/* RCC AHB2 peripheral reset register							Address offset: 0x14 */
	 	 	 uint32_t RESERVED0;	/* Reserved														Address offset: 0x18 */
	 	 	 uint32_t RESERVED1;	/* Reserved														Address offset: 0x1C */
	volatile uint32_t APB1RSTR;		/* RCC APB1 peripheral reset register							Address offset: 0x20 */
	volatile uint32_t APB2RSTR;		/* RCC APB2 peripheral reset register							Address offset: 0x24 */
	 	 	 uint32_t RESERVED2;	/* Reserved														Address offset: 0x28 */
	 	 	 uint32_t RESERVED3;	/* Reserved														Address offset: 0x2C */
	volatile uint32_t AHB1ENR;		/* RCC AHB1 peripheral clock enable register					Address offset: 0x30 */
	volatile uint32_t AHB2ENR;		/* RCC AHB2 peripheral clock enable register					Address offset: 0x34 */
	 	 	 uint32_t RESERVED4;	/* Reserved 													Address offset: 0x38 */
	 	 	 uint32_t RESERVED5;	/* Reserved														Address offset: 0x3C */
	volatile uint32_t APB1ENR;		/* RCC APB1 peripheral clock enable register 					Address offset: 0x40 */
	volatile uint32_t APB2ENR;		/* RCC APB2 peripheral clock enable register					Address offset: 0x44 */
	 	 	 uint32_t RESERVED6;	/* Reserved														Address offset: 0x48 */
	 	 	 uint32_t RESERVED7;	/* Reserved														Address offset: 0x4C */
	volatile uint32_t AHB1LPENR;	/* RCC AHB1 peripheral clock enable in low power mode register	Address offset: 0x50 */
	volatile uint32_t AHB2LPENR;	/* RCC AHB2 peripheral clock enable in low power mode register	Address offset: 0x54 */
	 	 	 uint32_t RESERVED8;	/* Reserved														Address offset: 0x58 */
	 	 	 uint32_t RESERVED9;	/* Reserved														Address offset: 0x5C */
	volatile uint32_t APB1LPENR;	/* RCC APB1 peripheral clock enable in low power mode register 	Address offset: 0x60 */
	volatile uint32_t APB2LPENR;	/* RCC APB2 peripheral clock enable in low power mode register	Address offset: 0x64 */
	 	 	 uint32_t RESERVED10;	/* Reserved 													Address offset: 0x68 */
	 	 	 uint32_t RESERVED11;	/* Reserved														Address offset: 0x6C */
	volatile uint32_t BDCR;			/* RCC Backup domain control register							Address offset: 0x70 */
	volatile uint32_t CSR;			/* RCC clock control & status register							Address offset: 0x74 */
	 	 	 uint32_t RESERVED12;	/* Reserved														Address offset: 0x78 */
	 	 	 uint32_t RESERVED13;	/* Reserved														Address offset: 0x7C */
	volatile uint32_t SSCGR;		/* RCC spread spectrum clock generation register				Address offset: 0x80 */
	volatile uint32_t PLLI2SCFGR;	/* RCC PLLI2S configuration register							Address offset: 0x84 */
	 	 	 uint32_t RESERVED14;	/* Reserved														Address offset: 0x88 */
	volatile uint32_t DCKCFGR;		/* RCC Dedicated Clocks Configuration Register					Address offset: 0x8C */
} RCC_RegDef_t;

/*
 * Peripheral register definition structure for EXTI
 */
typedef struct
{
	volatile uint32_t IMR;			/* Interrupt mask register										Address offset: 0x00 */
	volatile uint32_t EMR;			/* Event mask register											Address offset: 0x04 */
	volatile uint32_t RTSR;			/* Rising trigger selection register							Address offset: 0x08 */
	volatile uint32_t FTSR;			/* Falling trigger selection register							Address offset: 0x0C */
	volatile uint32_t SWIER;		/* Software interrupt event register							Address offset: 0x10 */
	volatile uint32_t PR;			/* Pending register												Address offset: 0x14 */
} EXTI_RegDef_t;

/*
 * Peripheral register definition structure for SYSCFG
 */
typedef struct
{
	volatile uint32_t MEMRMP;		/* SYSCFG memory remap register									Address offset: 0x00 */
	volatile uint32_t PMC;			/* SYSCFG peripheral mode configuration register				Address offset: 0x04 */
	volatile uint32_t EXTICR[4];	/* SYSCFG external interrupt configuration registers 1 - 4		Address offset: 0x08, 0x0C, 0x10 and 0x14 */
	         uint32_t RESERVED[2];	/* Reserved														Address offset: 0x18 and 0x1C */
	volatile uint32_t CMPCR;		/* Compensation cell control register							Address offset: 0x20 */
} SYSCFG_RegDef_t;

/*
 *  Peripheral definitions (Peripheral base addresses typecasted to xxx_RegDef_t)
 */

#define RCC		( (RCC_RegDef_t*)RCC_BASEADDR )
#define EXTI	( (EXTI_RegDef_t*)EXTI_BASEADDR )
#define SYSCFG	( (SYSCFG_RegDef_t*)SYSCFG_BASEADDR )

/*
 * Clock Enable Macros for GPIOx peripherals
 */

#define GPIOA_PERI_CLK_EN()	( RCC->AHB1ENR |= (1 << 0) )
#define GPIOB_PERI_CLK_EN()	( RCC->AHB1ENR |= (1 << 1) )
#define GPIOC_PERI_CLK_EN()	( RCC->AHB1ENR |= (1 << 2) )
#define GPIOD_PERI_CLK_EN()	( RCC->AHB1ENR |= (1 << 3) )
#define GPIOE_PERI_CLK_EN()	( RCC->AHB1ENR |= (1 << 4) )
#define GPIOH_PERI_CLK_EN()	( RCC->AHB1ENR |= (1 << 7) )

/*
 * Clock Enable Macros for I2Cx peripherals
 */

#define I2C1_PERI_CLK_EN()	( RCC->APB1ENR |= (1 << 21) )
#define I2C2_PERI_CLK_EN()	( RCC->APB1ENR |= (1 << 22) )
#define I2C3_PERI_CLK_EN()	( RCC->APB1ENR |= (1 << 23) )

/*
 * Clock Enable Macros for SPIx peripherals
 *
 */

#define SPI1_PERI_CLK_EN()	( RCC->APB2ENR |= (1 << 12) )
#define SPI2_PERI_CLK_EN()	( RCC->APB1ENR |= (1 << 14) )
#define SPI3_PERI_CLK_EN()	( RCC->APB1ENR |= (1 << 15) )
#define SPI4_PERI_CLK_EN()	( RCC->APB2ENR |= (1 << 13) )

/*
 * Clock Enable Macros for USARTx peripherals
 */

#define USART1_PERI_CLK_EN()	( RCC->APB2ENR |= (1 << 4) )
#define USART2_PERI_CLK_EN()	( RCC->APB1ENR |= (1 << 17) )
#define USART6_PERI_CLK_EN()	( RCC->APB2ENR |= (1 << 5) )

/*
 * Clock Enable Macros for SYSCFG peripheral
 */

#define SYSCFG_PERI_CLK_EN()	( RCC->APB2ENR |= (1 << 14) )

/*
 * Clock Disable Macros for GPIOx peripherals
 */

#define GPIOA_PERI_CLK_DI()	( RCC->AHB1ENR &= ~(1 << 0) )
#define GPIOB_PERI_CLK_DI()	( RCC->AHB1ENR &= ~(1 << 1) )
#define GPIOC_PERI_CLK_DI()	( RCC->AHB1ENR &= ~(1 << 2) )
#define GPIOD_PERI_CLK_DI()	( RCC->AHB1ENR &= ~(1 << 3) )
#define GPIOE_PERI_CLK_DI()	( RCC->AHB1ENR &= ~(1 << 4) )
#define GPIOH_PERI_CLK_DI()	( RCC->AHB1ENR &= ~(1 << 7) )

/*
 * Clock Disable Macros for I2Cx peripherals
 */

#define I2C2_PERI_CLK_DI()	( RCC->APB1ENR &= ~(1 << 22) )
#define I2C3_PERI_CLK_DI()	( RCC->APB1ENR &= ~(1 << 23) )
#define I2C1_PERI_CLK_DI()	( RCC->APB1ENR &= ~(1 << 21) )

/*
 * Clock Disable Macros for SPIx peripherals
 *
 */

#define SPI1_PERI_CLK_DI()	( RCC->APB2ENR &= ~(1 << 12) )
#define SPI2_PERI_CLK_DI()	( RCC->APB1ENR &= ~(1 << 14) )
#define SPI3_PERI_CLK_DI()	( RCC->APB1ENR &= ~(1 << 15) )
#define SPI4_PERI_CLK_DI()	( RCC->APB2ENR &= ~(1 << 13) )

/*
 * Clock Disable Macros for USARTx peripherals
 */

#define USART1_PERI_CLK_DI()	( RCC->APB2ENR &= ~(1 << 4) )
#define USART2_PERI_CLK_DI()	( RCC->APB1ENR &= ~(1 << 17) )
#define USART6_PERI_CLK_DI()	( RCC->APB2ENR &= ~(1 << 5) )

/*
 * Clock Disable Macros for SYSCFG peripheral
 */

#define SYSCFG_PERI_CLK_DI()	( RCC->APB2ENR &= ~(1 << 14) )

/*
 * Macros to reset GPIOx peripherals
 */
// do ... while ... condition zero loop: This is technique in C programming to execute
// multiple C statements using a single C macro
#define GPIOA_REG_RESET()		do{ (RCC->AHB1RSTR |= (1 << 0)); (RCC->AHB1RSTR &= ~(1 << 0)); } while(0)
#define GPIOB_REG_RESET()		do{ (RCC->AHB1RSTR |= (1 << 1)); (RCC->AHB1RSTR &= ~(1 << 1)); } while(0)
#define GPIOC_REG_RESET()		do{ (RCC->AHB1RSTR |= (1 << 2)); (RCC->AHB1RSTR &= ~(1 << 2)); } while(0)
#define GPIOD_REG_RESET()		do{ (RCC->AHB1RSTR |= (1 << 3)); (RCC->AHB1RSTR &= ~(1 << 3)); } while(0)
#define GPIOE_REG_RESET()		do{ (RCC->AHB1RSTR |= (1 << 4)); (RCC->AHB1RSTR &= ~(1 << 4)); } while(0)
#define GPIOH_REG_RESET()		do{ (RCC->AHB1RSTR |= (1 << 7)); (RCC->AHB1RSTR &= ~(1 << 7)); } while(0)

/*
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
#define IRQ_NO_EXTI0			6
#define IRQ_NO_EXTI1			7
#define IRQ_NO_EXTI2			8
#define IRQ_NO_EXTI3			9
#define IRQ_NO_EXTI4			10
#define IRQ_NO_EXTI9_5			23
#define IRQ_NO_EXTI15_10		40
#define IRQ_NO_SPI1				35
#define IRQ_NO_SPI2				36
#define IRQ_NO_SPI3				51
#define IRQ_NO_SPI4				84

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
