/*
 * stm32g4xx.h
 *
 *  Created on: Nov 15, 2024
 *      Author: pebl91
 */

#ifndef INC_STM32G4XX_H_
#define INC_STM32G4XX_H_

#include <stdint.h>

#define __vo volatile

/***************************************START:Processor Specific Details *************************************************
 *
 * ARM Cortex Mx Processor NVIC ISERx register Addresses
 */

#define NVIC_ISER0						((__vo uint32_t*)0xE000E100)
#define NVIC_ISER1						((__vo uint32_t*)0xE000E104)
#define NVIC_ISER2						((__vo uint32_t*)0xE000E108)
#define NVIC_ISER3						((__vo uint32_t*)0xE000E10c)

/*
 * ARM Cortex Mx Proccessor NVIC ICERx register Addresses
 */

#define NVIC_ICER0						((__vo uint32_t*)0xE000E180)
#define NVIC_ICER1						((__vo uint32_t*)0xE000E184)
#define NVIC_ICER2						((__vo uint32_t*)0xE000E188)
#define NVIC_ICER3						((__vo uint32_t*)0xE000E18c)

/*
 * ARM Cortex Mx Proccessor Priority Register Address Calculation
 */
#define NVIC_PR_BASE_ADDR				((__vo uint32_t*)0xE000E400)

#define NO_PR_BITS_IMPLEMENTED

/*************************************************************************************************************************/

/*
 * base addresses of Flash and SRAM memories
 */

#define FLASH_BASEADDR					0x00000000U							/* base address of flash */
#define SRAM1_BASEADDR					0x20000000U							/* base address of SRAM1 */
#define SRAM2_BASEADDR					0x20014000U							/* base address of SRAM2 */
#define ROM								0x1FFF0000U							/* base address of system memory */
#define SRAM 							SRAM1_BASEADDR						/* macro for SRAM address */

/*
 * AHBx and APBx Bus Peripheral base addresses
 */

#define PERIPH_BASEADDR					0x40000000U							/* base address of peripheral bus */
#define APB1PERIPH_BASEADDR				PERIPH_BASEADDR						/* base address of APB1 (same as peripheral) */
#define APB2PERIPH_BASEADDR				0x40010000U							/* base address of APB2 */
#define AHB1PERIPH_BASEADDR				0x40020000U							/* base address of AHB1 */
#define AHB2PERIPH_BASEADDR				0x48000000U							/* base address of AHB2 */

/*
 * Base addresses of peripherals which are hanging on AHB1 bus
 */

#define RCC_BASEADDR					0x40021000U							/* base address of RCC */

/*
 * Base addresses of peripherals which are hanging on AHB2 bus
 */

#define GPIOA_BASEADDR 					(AHB2PERIPH_BASEADDR + 0x0000)		/* base address of GPIOA */
#define GPIOB_BASEADDR					(AHB2PERIPH_BASEADDR + 0x0400)		/* base address of GPIOB */
#define GPIOC_BASEADDR					(AHB2PERIPH_BASEADDR + 0x0800)		/* base address of GPIOC */
#define GPIOD_BASEADDR					(AHB2PERIPH_BASEADDR + 0x0C00)		/* base address of GPIOD */
#define GPIOE_BASEADDR					(AHB2PERIPH_BASEADDR + 0x1000)		/* base address of GPIOE */
#define GPIOF_BASEADDR					(AHB2PERIPH_BASEADDR + 0x1400)		/* base address of GPIOF */
#define GPIOG_BASEADDR					(AHB2PERIPH_BASEADDR + 0x1800)		/* base address of GPIOG */

/*
 * Base addresses of peripherals which are hangign on APB1 bus
 */

#define I2C1_BASEADDR					(APB1PERIPH_BASEADDR + 0x5400)		/* base address of I2C1 */
#define I2C2_BASEADDR					(APB1PERIPH_BASEADDR + 0x5800)		/* base address of I2C2 */
#define I2C3_BASEADDR					(APB1PERIPH_BASEADDR + 0x7800)		/* base address of I2C3 */
#define I2C4_BASEADDR					(APB1PERIPH_BASEADDR + 0x8400)		/* base address of I2C4 */

#define SPI2_BASEADDR					(APB1PERIPH_BASEADDR + 0x3800)		/* base address of SPI2 */
#define SPI3_BASEADDR					(APB1PERIPH_BASEADDR + 0x3C00)		/* base address of SPI3 */

#define USART2_BASEADDR					(APB1PERIPH_BASEADDR + 0x4400)		/* base address of USART2 */
#define USART3_BASEADDR					(APB1PERIPH_BASEADDR + 0x4800)		/* base address of USART3 */

#define UART4_BASEADDR					(APB1PERIPH_BASEADDR + 0x4C00)		/* base address of UART4 */
#define UART5_BASEADDR					(APB1PERIPH_BASEADDR + 0x5000)		/* base address of UART5 */

#define LPUART1_BASEADDR				(APB1PERIPH_BASEADDR + 0x8000)		/* base address of LPUART1 */

/*
 * Base addresses of peripherals which are hangign on APB2 bus
 */

#define SPI1_BASEADDR					(APB2PERIPH_BASEADDR + 0x3000)		/* base address of SPI1 */
#define SPI4_BASEADDR					(APB2PERIPH_BASEADDR + 0x3C00)		/* base address of SPI4 */

#define USART1_BASEADDR					(APB2PERIPH_BASEADDR + 0x3800)		/* base address of USART1 */

#define EXTI_BASEADDR					(APB2PERIPH_BASEADDR + 0x0400)		/* base address of EXTI */

#define SYSCFG_BASEADDR					(APB2PERIPH_BASEADDR + 0x0000)		/* base address of SYSCFG */


/********************************* peripheral register definition structures *********************************/

/*
 * peripheral register definition structure for GPIO
 */
typedef struct
{
	__vo uint32_t MODER;					/* Set mode of GPIO			 						Address offset:	0x00 */
	__vo uint32_t OTYPER;					/* Output type of register							Address offset:	0x04 */
	__vo uint32_t OSPEEDR;					/* Output speed register 							Address offset:	0x08 */
	__vo uint32_t PUPDR;					/* Pull-up/Pull-dwon register						Address offset:	0x0c */
	__vo uint32_t IDR;						/* Input data register								Address offset:	0x10 */
	__vo uint32_t ODR;						/* Output data register 							Address offset:	0x14 */
	__vo uint32_t BSRR;						/* Bit set/reset register 							Address offset:	0x18 */
	__vo uint32_t LCKR;						/* Configuration lock register						Address offset:	0x1c */
	__vo uint32_t AFR[2];						/* Alternate function low register				Address offset:	0x20 */
	__vo uint32_t BRR;						/* Bit reset register								Address offset:	0x28 */
}GPIO_RegDef_t;

/*
 * peripheral register definition structure for RCC
 */
typedef struct
{
	__vo uint32_t CR;					/* Clock control register			 									Address offset:	0x00 */
	__vo uint32_t ICSCR;				/* Internal clock sources calibration register 							Address offset:	0x04 */
	__vo uint32_t CFGR;					/* Clock configuration register			 								Address offset:	0x08 */
	__vo uint32_t PLLCFGR;				/* PLL configuration register			 								Address offset:	0x0c */
	__vo uint32_t reserved1;			/* reserved1			 												Address offset:	0x10 */
	__vo uint32_t reserved2;			/* reserved2			 												Address offset:	0x14 */
	__vo uint32_t CIER;					/* Clock interrupt enable register			 							Address offset:	0x18 */
	__vo uint32_t CIFR;					/* Clock interrupt flag register			 							Address offset:	0x1c */
	__vo uint32_t CICR;					/* Clock interrupt clear register		 								Address offset:	0x20 */
	__vo uint32_t reserved3;			/* reserved3		 													Address offset:	0x24 */
	__vo uint32_t AHB1RSTR;				/* AHB1 peripheral reset register		 								Address offset:	0x28 */
	__vo uint32_t AHB2RSTR;				/* AHB2 peripheral reset register		 								Address offset:	0x2c */
	__vo uint32_t AHB3RSTR;				/* AHB3 peripheral reset register		 								Address offset:	0x30 */
	__vo uint32_t reserved4;			/* reserved4			 												Address offset:	0x34 */
	__vo uint32_t APB1RSTR1;			/* APB1 peripheral reset register 1			 							Address offset:	0x38 */
	__vo uint32_t APB1RSTR2;			/* APB1 peripheral reset register 2		 								Address offset:	0x3c */
	__vo uint32_t APB2RSTR;				/* APB2 peripheral reset register			 							Address offset:	0x40 */
	__vo uint32_t reserved5;			/* reserve			 													Address offset:	0x44 */
	__vo uint32_t AHB1ENR;				/* AHB1 peripheral clock enable register		 						Address offset:	0x48 */
	__vo uint32_t AHB2ENR;				/* AHB2 peripheral clock enable register		 						Address offset:	0x4c */
	__vo uint32_t AHB3ENR;				/* AHB3 peripheral clock enable register		 						Address offset:	0x50 */
	__vo uint32_t reserved6;			/* reserved6			 												Address offset:	0x54 */
	__vo uint32_t APB1ENR1;				/* APB1 peripheral clock enable register 1		 						Address offset:	0x58 */
	__vo uint32_t APB1ENR2;				/* APB1 peripheral clock enable register 2		 						Address offset:	0x5c */
	__vo uint32_t APB2ENR;				/* APB2 peripheral clock enable register		 						Address offset:	0x60 */
	__vo uint32_t reserved7;			/* reserve			 													Address offset:	0x64 */
	__vo uint32_t AHB1SMENR;			/* AHB1 peripheral clocks enable in Sleep and Stop modes register		Address offset:	0x68 */
	__vo uint32_t AHB2SMENR;			/* AHB2 peripheral clocks enable in Sleep and Stop modes register		Address offset:	0x6c */
	__vo uint32_t AHB3SMENR;			/* AHB3 peripheral clocks enable in Sleep and Stop modes register		Address offset:	0x70 */
	__vo uint32_t reserved8;			/* reserved8			 												Address offset:	0x74 */
	__vo uint32_t APB1SMENR1;			/* APB1 peripheral clocks enable in Sleep and Stop modes register 1		Address offset:	0x78 */
	__vo uint32_t APB1SMENR2;			/* APB1 peripheral clocks enable in Sleep and Stop modes register 2		Address offset:	0x7c */
	__vo uint32_t APB2SMENR;			/* APB2 peripheral clocks enable in Sleep and Stop modes register		Address offset:	0x80 */
	__vo uint32_t reserved9;			/* reserved9			 												Address offset:	0x84 */
	__vo uint32_t CCIPR;				/* Peripherals independent clock configuration register			 		Address offset:	0x88 */
	__vo uint32_t BDCR;					/* RTC domain control register		 									Address offset:	0x90 */
	__vo uint32_t CSR;					/* Control/status register			 									Address offset:	0x94 */
	__vo uint32_t CRRCR;				/* Clock recovery RC register		 									Address offset:	0x98 */
	__vo uint32_t CCIPR2;				/* Peripherals independent clock configuration register			 		Address offset:	0x9c */
}RCC_RegDef_t;

/*
 * peripheral register definition structure for EXTI
 */
typedef struct
{
	__vo uint32_t IMR1;					/* Interrupt mask register 1 		 				Address offset:	0x00 */
	__vo uint32_t EMR1;					/* Event mask register 1							Address offset:	0x04 */
	__vo uint32_t RTSR1;				/* Rising trigger selection register 1 				Address offset:	0x08 */
	__vo uint32_t FTSR1;				/* Falling trigger selection register 1				Address offset:	0x0c */
	__vo uint32_t SWIER1;				/* Software interrupt event register 1				Address offset:	0x10 */
	__vo uint32_t PR1;					/* Pending register 1								Address offset:	0x14 */
	__vo uint32_t reserved1;			/* reserved1										Address offset:	0x18 */
	__vo uint32_t reserved2;			/* reserved2 										Address offset:	0x1c */
	__vo uint32_t IMR2;					/* Interrupt mask register 2 						Address offset:	0x20 */
	__vo uint32_t EMR2;					/* Event mask register 2							Address offset:	0x24 */
	__vo uint32_t RTSR2;				/* Rising trigger selection register 2				Address offset:	0x28 */
	__vo uint32_t FTSR2;				/* Falling trigger selection register 2				Address offset:	0x2c */
	__vo uint32_t SWIER2;				/* Software interrupt event register 2				Address offset:	0x30 */
	__vo uint32_t PR2;					/* Pending register 2								Address offset:	0x34 */
}EXTI_RegDef_t;


/*
 * peripheral register definition structure for SYSCFG
 */
typedef struct
{
	__vo uint32_t MEMRMP;				/* SYSCFG memory remap register 		 					Address offset:	0x00 */
	__vo uint32_t CFGR1;				/* SYSCFG configuration register 1							Address offset:	0x04 */
	__vo uint32_t EXTICR[4];			/* SYSCFG external interrupt configuration register 1		Address offset:	0x08 */
										/* 0 for EXTICR1, 1 for EXTICR2, 2 for EXTICR2, 3 for EXTICR4					 */
	__vo uint32_t SCSR;					/* SYSCFG CCM SRAM control and status register				Address offset:	0x18 */
	__vo uint32_t CFGR2;				/* SYSCFG configuration register 2							Address offset:	0x1c */
	__vo uint32_t SWPR;					/* SYSCFG CCM SRAM write protection register				Address offset:	0x20 */
	__vo uint32_t SKR;					/* SYSCFG CCM SRAM key register								Address offset:	0x24 */
}SYSCFG_RegDef_t;

/*
 * peripheral definitions (Peripheral base addresses typcasted to xxx_RegDef_t)
 */

#define GPIOA				((GPIO_RegDef_t*)GPIOA_BASEADDR)		/* Macro for GPIOA registers */
#define GPIOB				((GPIO_RegDef_t*)GPIOB_BASEADDR)		/* Macro for GPIOB registers */
#define GPIOC				((GPIO_RegDef_t*)GPIOC_BASEADDR)		/* Macro for GPIOC registers */
#define GPIOD				((GPIO_RegDef_t*)GPIOD_BASEADDR)		/* Macro for GPIOD registers */
#define GPIOE				((GPIO_RegDef_t*)GPIOE_BASEADDR)		/* Macro for GPIOE registers */
#define GPIOF				((GPIO_RegDef_t*)GPIOF_BASEADDR)		/* Macro for GPIOF registers */
#define GPIOG				((GPIO_RegDef_t*)GPIOG_BASEADDR)		/* Macro for GPIOG registers */

#define RCC					((RCC_RegDef_t*)RCC_BASEADDR)

#define EXTI				((EXTI_RegDef_t*)EXTI_BASEADDR)

#define SYSCFG				((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)


/*
 * Clock enabel macros for GPIOx peripherials
 */
#define GPIOA_PCLK_EN()	( RCC->AHB2ENR |= (1 << 0) )				/* Enabel GPIOA clock */
#define GPIOB_PCLK_EN()	( RCC->AHB2ENR |= (1 << 1) )				/* Enabel GPIOB clock */
#define GPIOC_PCLK_EN()	( RCC->AHB2ENR |= (1 << 2) )				/* Enabel GPIOC clock */
#define GPIOD_PCLK_EN()	( RCC->AHB2ENR |= (1 << 3) )				/* Enabel GPIOD clock */
#define GPIOE_PCLK_EN()	( RCC->AHB2ENR |= (1 << 4) )				/* Enabel GPIOE clock */
#define GPIOF_PCLK_EN()	( RCC->AHB2ENR |= (1 << 5) )				/* Enabel GPIOF clock */
#define GPIOG_PCLK_EN()	( RCC->AHB2ENR |= (1 << 6) )				/* Enabel GPIOG clock */

/*
 * Clock enabel macros for I2Cx peripherials
 */
#define I2C1_PCLK_EN()	( RCC->APB1ENR1 |= (1 << 21) )				/* Enabel I2C1 clock */
#define I2C2_PCLK_EN()	( RCC->APB1ENR1 |= (1 << 22) )				/* Enabel I2C2 clock */
#define I2C3_PCLK_EN()	( RCC->APB1ENR1 |= (1 << 30) )				/* Enabel I2C3 clock */
#define I2C4_PCLK_EN()	( RCC->APB1ENR2 |= (1 << 1)  )				/* Enabel I2C4 clock */

/*
 * Clock enabel macros for SPIx peripherials
 */
#define SPI1_PCLK_EN()	( RCC->APB2ENR  |= (1 << 12) )				/* Enabel SPI1 clock */
#define SPI2_PCLK_EN()	( RCC->APB1ENR1 |= (1 << 14) )				/* Enabel SPI2 clock */
#define SPI3_PCLK_EN()	( RCC->APB1ENR1 |= (1 << 15) )				/* Enabel SPI3 clock */
#define SPI4_PCLK_EN()	( RCC->APB2ENR  |= (1 << 15) )				/* Enabel SPI4 clock */

/*
 * Clock enabel macros for USARTx peripherials
 */
#define USART1_PCLK_EN()	( RCC->APB2ENR  |= (1 << 14) )			/* Enabel USART1 clock   */
#define USART2_PCLK_EN()	( RCC->APB1ENR1 |= (1 << 17) )			/* Enabel USART2 clock   */
#define USART3_PCLK_EN()	( RCC->APB1ENR1 |= (1 << 18) )			/* Enabel USART3 clock 	 */
#define UART4_PCLK_EN()		( RCC->APB1ENR1 |= (1 << 19) )			/* Enabel UART4 clock  	 */
#define UART5_PCLK_EN()		( RCC->APB1ENR1 |= (1 << 20) )			/* Enabel UART5 clock    */
#define LPUART1_PCLK_EN()	( RCC->APB1ENR2 |= (1 << 0) )			/* Enabel LPUART1 clock  */

/*
 * Clock enabel macros for SYSCFG peripherials
 */
#define SYSCFG_PCLK_EN()	( RCC->APB2ENR |= (1 << 0) )			/* Enabel SYSCFG clock   */

/*
 * Clock disable macros for GPIOx peripherials
 */
#define GPIOA_PCLK_DI()	( RCC->AHB2ENR &= ~(1 << 0) )				/* Disable GPIOA clock */
#define GPIOB_PCLK_DI()	( RCC->AHB2ENR &= ~(1 << 1) )				/* Disable GPIOB clock */
#define GPIOC_PCLK_DI()	( RCC->AHB2ENR &= ~(1 << 2) )				/* Disable GPIOC clock */
#define GPIOD_PCLK_DI()	( RCC->AHB2ENR &= ~(1 << 3) )				/* Disable GPIOD clock */
#define GPIOE_PCLK_DI()	( RCC->AHB2ENR &= ~(1 << 4) )				/* Disable GPIOE clock */
#define GPIOF_PCLK_DI()	( RCC->AHB2ENR &= ~(1 << 5) )				/* Disable GPIOF clock */
#define GPIOG_PCLK_DI()	( RCC->AHB2ENR &= ~(1 << 6) )				/* Disable GPIOG clock */

/*
 * Clock disable macros for I2Cx peripherials
 */
#define I2C1_PCLK_DI()	( RCC->APB1ENR1 &= ~(1 << 21) )				/* Disable I2C1 clock */
#define I2C2_PCLK_DI()	( RCC->APB1ENR1 &= ~(1 << 22) )				/* Disable I2C2 clock */
#define I2C3_PCLK_DI()	( RCC->APB1ENR1 &= ~(1 << 30) )				/* Disable I2C3 clock */
#define I2C4_PCLK_DI()	( RCC->APB1ENR2 &= ~(1 << 1)  )				/* Disable I2C4 clock */

/*
 * Clock disable macros for SPIx peripherials
 */
#define SPI1_PCLK_DI()	( RCC->APB2ENR  &= ~(1 << 12) )				/* Disable SPI1 clock */
#define SPI2_PCLK_DI()	( RCC->APB1ENR1 &= ~1 << 14) )				/* Disable SPI2 clock */
#define SPI3_PCLK_DI()	( RCC->APB1ENR1 &= ~(1 << 15) )				/* Disable SPI3 clock */
#define SPI4_PCLK_DI()	( RCC->APB2ENR  &= ~(1 << 15) )				/* Disable SPI4 clock */

/*
 * Clock disable macros for USARTx peripherials
 */
#define USART1_PCLK_DI()	( RCC->APB2ENR  &= ~(1 << 14) )			/* Disable USART1 clock   */
#define USART2_PCLK_DI()	( RCC->APB1ENR1 &= ~(1 << 17) )			/* Disable USART2 clock   */
#define USART3_PCLK_DI()	( RCC->APB1ENR1 &= ~(1 << 18) )			/* Disable USART3 clock   */
#define UART4_PCLK_DI()		( RCC->APB1ENR1 &= ~(1 << 19) )			/* Disable UART4 clock    */
#define UART5_PCLK_DI()		( RCC->APB1ENR1 &= ~(1 << 20) )			/* Disable UART5 clock    */
#define LPUART1_PCLK_DI()	( RCC->APB1ENR2 &= ~(1 << 0) )			/* Disable LPUART1 clock  */

/*
 * Clock disable macros for SYSCFG peripherials
 */
#define SYSCFG_PCLK_DI()	( RCC->APB2ENR &= ~(1 << 0) )			/* Disable SYSCFG clock   */

/*
 * Macros to reset GPIOx peripherals
 */
#define GPIOA_REG_RESET()		do{ (RCC->AHB2RSTR |= (1 << 0));			(RCC->AHB2RSTR &= ~(1 << 0)); } while(0)
#define GPIOB_REG_RESET()		do{ (RCC->AHB2RSTR |= (1 << 1));			(RCC->AHB2RSTR &= ~(1 << 1)); } while(0)
#define GPIOC_REG_RESET()		do{ (RCC->AHB2RSTR |= (1 << 2));			(RCC->AHB2RSTR &= ~(1 << 2)); } while(0)
#define GPIOD_REG_RESET()		do{ (RCC->AHB2RSTR |= (1 << 3));			(RCC->AHB2RSTR &= ~(1 << 3)); } while(0)
#define GPIOE_REG_RESET()		do{ (RCC->AHB2RSTR |= (1 << 4));			(RCC->AHB2RSTR &= ~(1 << 4)); } while(0)
#define GPIOF_REG_RESET()		do{ (RCC->AHB2RSTR |= (1 << 5));			(RCC->AHB2RSTR &= ~(1 << 5)); } while(0)
#define GPIOG_REG_RESET()		do{ (RCC->AHB2RSTR |= (1 << 6));			(RCC->AHB2RSTR &= ~(1 << 6)); } while(0)

/*
 *  returns port code for given GPIOx base address
 */
#define GPIO_BASSEADDR_TO_CODE(x)	   ((x == GPIOA) ? 0 :\
										(x == GPIOB) ? 1 :\
										(x == GPIOC) ? 2 :\
										(x == GPIOD) ? 3 :\
										(x == GPIOE) ? 4 :\
										(x == GPIOF) ? 5 :\
										(x == GPIOG) ? 6:0)

/*
 * IRQ(Interrupt Request) Numbers of STM32G491x MCU
 */
#define IRQ_NO_EXTI0			6
#define IRQ_NO_EXTI1			7
#define IRQ_NO_EXTI2			8
#define IRQ_NO_EXTI3			9
#define IRQ_NO_EXTI4			10
#define IRQ_NO_EXTI5_9			23
#define IRQ_NO_EXTI10_15		40

//some generic macros
#define ENABLE 				1
#define DISABLE 			0
#define SET 				ENABLE
#define RESET 				DISABLE
#define GPIO_PIN_SET		SET
#define GPIO_PIN_RESET		RESET


#include "stm32g491xx_gpio_driver.h"

#endif /* INC_STM32G4XX_H_ */
