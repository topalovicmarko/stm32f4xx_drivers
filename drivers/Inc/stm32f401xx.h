/*
 * stm32f401xx.h
 *
 *  Created on: July 31, 2025
 *      Author: Marko Topalović
 */

#ifndef INC_STM32F401XX_H_
#define INC_STM32F401XX_H_

#include <stddef.h>			//NULL je ovde definisano
#include <stdint.h>

#define __vo volatile
#define __weak __attribute__((weak))

/**************************** START: Processor Specific Details ****************************
 *
 * ARM Conrtex Mx Processor NVIC ISERx register Addresses
 */

#define NVIC_ISER0   		 ((__vo uint32_t*)0xE000E100)  // IRQ  0-31
#define NVIC_ISER1  		 ((__vo uint32_t*)0xE000E104)  // IRQ 32-63
#define NVIC_ISER2  		 ((__vo uint32_t*)0xE000E108)  // IRQ 64-95
#define NVIC_ISER3  		 ((__vo uint32_t*)0xE000E10C)  // IRQ 96-127
#define NVIC_ISER4  		 ((__vo uint32_t*)0xE000E110)  // IRQ 128-159
#define NVIC_ISER5  		 ((__vo uint32_t*)0xE000E114)  // IRQ 160-191
#define NVIC_ISER6  		 ((__vo uint32_t*)0xE000E118)  // IRQ 192-223
#define NVIC_ISER7  		 ((__vo uint32_t*)0xE000E11C)  // IRQ 224-255 (Cortex-M4 koristi do 240)

/*
 * ARM Conrtex Mx Processor NVIC ICERx register Addresses
 */
#define NVIC_ICER0   		((__vo uint32_t*)0xE000E180)  // IRQ  0-31
#define NVIC_ICER1   		((__vo uint32_t*)0xE000E184)  // IRQ 32-63
#define NVIC_ICER2   		((__vo uint32_t*)0xE000E188)  // IRQ 64-95
#define NVIC_ICER3   		((__vo uint32_t*)0xE000E18C)  // IRQ 96-127
#define NVIC_ICER4   		((__vo uint32_t*)0xE000E190)  // IRQ 128-159
#define NVIC_ICER5   		((__vo uint32_t*)0xE000E194)  // IRQ 160-191
#define NVIC_ICER6   		((__vo uint32_t*)0xE000E198)  // IRQ 192-223
#define NVIC_ICER7   		((__vo uint32_t*)0xE000E19C)  // IRQ 224-255 (Cortex-M4 koristi do 240)

/*
 * ARM Conrtex Mx Processor Priority Register Address Calculation
 */

#define NVIC_PR_BASE_ADDR	((__vo uint32_t*)0xE000E400)


#define NO_PR_BITS_IMPLEMENTED		4

/*
 * Base addresses of Flash and SRAM memories
 */

#define FLASH_BASEADDR			0x08000000U				/*!< Početna adresa Flash memorije gde se nalazi programski kod */
#define SRAM1_BASEADDR			0x20000000U	//96KB		/*!< Početna adresa glavne SRAM memorije (96KB na STM32F401) */
//#define SRAM2_BASEADDR		0x20018000U				/*!< Ne koristi se - F401 nema SRAM2 memoriju  96KB*1024 = 98304 bajta = 0x18000 u hex (offset za sram1)
#define ROM_BASEADDR			0x1FFF0000U				/*!< Početna adresa sistemskog ROM-a (sadrži bootloader) */
#define SRAM 					SRAM1_BASEADDR			/*!< Alias za glavnu SRAM memoriju (podrazumeva SRAM1) */


/*
 * AHBx and APBx Bus Peripheral base addresses
 */

#define PERIPH_BASEADDR			0x40000000U
#define APB1PERIPH_BASEADDR		PERIPH_BASEADDR
#define APB2PERIPH_BASEADDR		0x40010000U
#define AHB1PERIPH_BASEADDR		0x40020000U
#define AHB2PERIPH_BASEADDR		0x50000000U

/*
 * Base addresses of peripherals which are hanging on AHB1 bus
 * TODO: Complete for all other peripherals
 */

//									  BASE			 + OFFSET
#define GPIOA_BASEADDR			(AHB1PERIPH_BASEADDR + 0x0000)
#define GPIOB_BASEADDR			(AHB1PERIPH_BASEADDR + 0x0400)
#define GPIOC_BASEADDR			(AHB1PERIPH_BASEADDR + 0x0800)
#define GPIOD_BASEADDR			(AHB1PERIPH_BASEADDR + 0x0C00)
#define GPIOE_BASEADDR			(AHB1PERIPH_BASEADDR + 0x1000)
#define GPIOF_BASEADDR			(AHB1PERIPH_BASEADDR + 0x1400)
#define GPIOG_BASEADDR			(AHB1PERIPH_BASEADDR + 0x1800)
#define GPIOH_BASEADDR			(AHB1PERIPH_BASEADDR + 0x1C00)
#define GPIOI_BASEADDR			(AHB1PERIPH_BASEADDR + 0x2000)
#define RCC_BASEADDR 			(AHB1PERIPH_BASEADDR + 0x3800)

/*
 * Base addresses of peripherals which are hanging on APB1 bus
 * TODO: Complete for all other peripherals
 */

//									  BASE			 + OFFSET
#define I2C1_BASEADDR			(APB1PERIPH_BASEADDR + 0x5400)
#define I2C2_BASEADDR			(APB1PERIPH_BASEADDR + 0x5800)
#define I2C3_BASEADDR			(APB1PERIPH_BASEADDR + 0x5C00)

#define SPI2_BASEADDR			(APB1PERIPH_BASEADDR + 0x3800)
#define SPI3_BASEADDR			(APB1PERIPH_BASEADDR + 0x3C00)

#define USART2_BASEADDR			(APB1PERIPH_BASEADDR + 0x4400)
#define USART3_BASEADDR			(APB1PERIPH_BASEADDR + 0x4800)
#define UART4_BASEADDR			(APB1PERIPH_BASEADDR + 0x4C00)
#define UART5_BASEADDR			(APB1PERIPH_BASEADDR + 0x5000)

/*
 * Base addresses of peripherals which are hanging on APB2 bus
 * TODO: Complete for all other peripherals
 */

#define EXTI_BASEADDR	 		(APB2PERIPH_BASEADDR + 0x3C00)
#define SPI1_BASEADDR			(APB2PERIPH_BASEADDR + 0x3000)
#define SPI4_BASEADDR			(APB2PERIPH_BASEADDR + 0x3400)
#define SYSCFG_BASEADDR			(APB2PERIPH_BASEADDR + 0x3800)
#define USART1_BASEADDR			(APB2PERIPH_BASEADDR + 0x1000)
#define USART6_BASEADDR			(APB2PERIPH_BASEADDR + 0x1400)

/************************************************ Peripherial register definiton structures ************************************************/

typedef struct
{
	__vo uint32_t MODER;			/*!< GPIO port mode register, 																	Address offset: 0x00 */
	__vo uint32_t OTYPER;			/*!< GPIO port output type register, 															Address offset: 0x04 */
	__vo uint32_t OSPEEDR;			/*!< GPIO port output speed register, 															Address offset: 0x08 */
	__vo uint32_t PUPDR;			/*!< GPIO port pull-up/pull-down register, 														Address offset: 0x0C */
	__vo uint32_t IDR;				/*!< GPIO port input data register, 															Address offset: 0x10 */
	__vo uint32_t ODR;				/*!< GPIO port output data register, 															Address offset: 0x14 */
	__vo uint32_t BSRR;				/*!< GPIO port bit set/reset register, 															Address offset: 0x18 */
	__vo uint32_t LCKR;				/*!< GPIO port configuration lock register, 													Address offset: 0x1C */
	__vo uint32_t AFR[2];			/*!< AFR[0] GPIO alternate function low register, AFR[1] GPIO alternate function high register,	Address offset: 0x20 , 0x24 */
}GPIO_RegDef_t;

/*
 *  GPIO_RegDef_t *pGIOA = (GPIO_RegDef_t*)0x40020000;
 *
 *  Helps programmers for easy access to various registers of the peripherals
 *  pGPIOA->MODER = 25; //storing value 25 in to MODER register
 *  *(0x40020000+0x00) = 25; // this is how compiler does
 *  *pGPIOA->ODR = 44; //storing value 44 in to ODER register
 *  *(0x40020000+0x14) = 44; // this is how compiler does
 */


/*
*Peripherial register definiton structures for RCC
*/

typedef struct
{
	__vo uint32_t CR; 				/*!<RCC clock control register,																	Address offset: 0x00 */
	__vo uint32_t PLLCFGR; 			/*!<RCC PLL configuration register,																Address offset: 0x04 */
	__vo uint32_t CFGR; 			/*!<RCC clock configuration register,															Address offset: 0x08 */
	__vo uint32_t CIR; 				/*!<RCC clock interrupt register,																Address offset: 0x0C */
	__vo uint32_t AHB1RSTR; 		/*!<RCC AHB1 peripheral reset register,															Address offset: 0x10 */
	__vo uint32_t AHB2RSTR; 		/*!<RCC AHB2 peripheral reset register,															Address offset: 0x14 */
	__vo uint32_t AHB3RSTR; 		/*!<RCC AHB3 peripheral reset register,															Address offset: 0x18 */
	uint32_t 	  RESERVED0;		/*!<Reserved, 0x1C																									 */
	__vo uint32_t APB1RSTR; 		/*!<RCC APB1 peripheral reset register,															Address offset: 0x20 */
	__vo uint32_t APB2RSTR; 		/*!<RCC APB2 peripheral reset register,															Address offset: 0x24 */
	uint32_t 	  RESERVED1[2];		/*!<Reserved, 0x28-0x2C																								 */
	__vo uint32_t AHB1ENR; 			/*!<RCC AHB1 peripheral clock register,															Address offset: 0x30 */
	__vo uint32_t AHB2ENR; 			/*!<RCC AHB2 peripheral clock enable register,													Address offset: 0x34 */
	__vo uint32_t AHB3ENR; 			/*!<RCC AHB3 peripheral clock enable register,													Address offset: 0x38 */
	uint32_t 	  RESERVED2;		/*!<Reserved, 0x3C																									 */
	__vo uint32_t APB1ENR; 			/*!<RCC APB1 peripheral clock enable register,													Address offset: 0x40 */
	__vo uint32_t APB2ENR; 			/*!<RCC APB2 peripheral clock enable register,													Address offset: 0x44 */
	uint32_t 	  RESERVED3[2];		/*!<Reserved 0x48-0x4C																								 */
	__vo uint32_t AHB1LPENR; 		/*!<RCC AHB1 peripheral clock enable in low power mode register,								Address offset: 0x50 */
	__vo uint32_t AHB2LPENR; 		/*!<RCC AHB2 peripheral clock enable in low power mode register,								Address offset: 0x54 */
	__vo uint32_t AHB3LPENR; 		/*!<RCC AHB3 peripheral clock enable in low power mode register,								Address offset: 0x58 */
	uint32_t 	  RESERVED4;		/*!<Reserved, 0x5C																									 */
	__vo uint32_t APB1LPENR; 		/*!<RCC APB1 peripheral clock enable in low power mode register,								Address offset: 0x60 */
	__vo uint32_t APB2LPENR; 		/*!<RCC APB2 peripheral clock enabled in low power mode register,								Address offset: 0x64 */
	uint32_t 	  RESERVED5[2];		/*!<Reserved 0x68-0x6C																								 */
	__vo uint32_t BDCR; 			/*!<RCC Backup domain control register,															Address offset: 0x70 */
	__vo uint32_t CSR; 				/*!<RCC clock control & status register,														Address offset: 0x74 */
	uint32_t 	  RESERVED6[2];		/*!<Reserved 0x78-0x7C																								 */
	__vo uint32_t SSCGR; 			/*!<RCC spread spectrum clock generation register,												Address offset: 0x80 */
	__vo uint32_t PLLI2SCFGR; 		/*!<RCC PLLI2S configuration register,															Address offset: 0x84 */
	__vo uint32_t PLLSAICFGR; 		/*!<RCC PLL configuration register,																Address offset: 0x88 */
	__vo uint32_t DCKCFGR; 			/*!<RCC Dedicated Clock Configuration Register,													Address offset: 0x8C */
}RCC_RegDef_t;

/*
*Peripherial register definiton structures for EXTI
*/

typedef struct
{
	__vo uint32_t IMR;			/*!< Interrupt mask register, 																	Address offset: 0x00 */
	__vo uint32_t EMR;			/*!< Event mask register, 																		Address offset: 0x04 */
	__vo uint32_t RTSR;			/*!< Rising trigger selection register, 														Address offset: 0x08 */
	__vo uint32_t FTSR;			/*!< Falling trigger selection register, 														Address offset: 0x0C */
	__vo uint32_t SWIER;		/*!< Software interrupt event register, 														Address offset: 0x10 */
	__vo uint32_t PR;			/*!< Pending register, 																			Address offset: 0x14 */
}EXTI_RegDef_t;

/*
*Peripherial register definiton structures for SPI
*/

typedef struct
{
	__vo uint32_t CR1;			/*!< SPI control register 1, 																	Address offset: 0x00 */
	__vo uint32_t CR2;			/*!< SPI control register 2, 																	Address offset: 0x04 */
	__vo uint32_t SR;			/*!< SPI status register, 																		Address offset: 0x08 */
	__vo uint32_t DR;			/*!< SPI data register, 																		Address offset: 0x0C */
	__vo uint32_t CRCPR;		/*!< SPI CRC polynomial register, 																Address offset: 0x10 */
	__vo uint32_t RXCRCR;		/*!< SPI RX CRC register, 																		Address offset: 0x14 */
	__vo uint32_t TXCRCR;		/*!< SPI TX CRC register, 																		Address offset: 0x18 */
	__vo uint32_t I2SCFGR;		/*!< SPI_I2S configuration register, 															Address offset: 0x1C */
	__vo uint32_t I2SPR;		/*!< SPI_I2S prescaler register, 																Address offset: 0x20 */
}SPI_RegDef_t;

/*
*Peripherial register definiton structures for I2C
*/

typedef struct
{
	__vo uint32_t CR1;			/*!< I2C Control register 1, 																	Address offset: 0x00 */
	__vo uint32_t CR2;			/*!< I2C control register 2, 																	Address offset: 0x04 */
	__vo uint32_t OAR1;			/*!< I2C Own address register 1, 																Address offset: 0x08 */
	__vo uint32_t OAR2;			/*!< I2C Own address register 2, 																Address offset: 0x0C */
	__vo uint32_t DR;			/*!< I2C Data register, 																		Address offset: 0x10 */
	__vo uint32_t SR1;			/*!< I2C Status register 1, 																	Address offset: 0x14 */
	__vo uint32_t SR2;			/*!< I2C Status register 2, 																	Address offset: 0x18 */
	__vo uint32_t CCR;			/*!< I2C Clock control register	, 																Address offset: 0x1C */
	__vo uint32_t TRISE;		/*!< I2C TRISE register, 																		Address offset: 0x20 */
	__vo uint32_t FLTR;			/*!< I2C FLTR register, 																		Address offset: 0x24 */
}I2C_RegDef_t;


/*
*Peripherial register definiton structures for SYSCFG
*/

typedef struct
{
	__vo uint32_t MEMRMP;			/*!< SYSCFG memory remap register, 															Address offset: 0x00 */
	__vo uint32_t PMC;				/*!< SYSCFG peripheral mode configuration register, 										Address offset: 0x04 */
	__vo uint32_t EXTICR[4];		/*!< SYSCFG external interrupt configuration register 1-4, 									Address offset: 0x08 - 0x14 */
	uint32_t	  RESERVED1[2];		/*!< 																						Reserved 0x18-0x1C 	*/
	__vo uint32_t CMPCR;			/*!< Compensation cell control register, 													Address offset: 0x20 */
	uint32_t	  RESERVED2[2];		/*!< 																						Reserved 0x24-0x28 	*/
}SYSCFG_RegDef_t;

/*
 * Peripheral definitions (Peripheral base addresses typecasted to xxx_RegDef_t)
 */

#define GPIOA		((GPIO_RegDef_t *)GPIOA_BASEADDR)
#define GPIOB		((GPIO_RegDef_t *)GPIOB_BASEADDR)
#define GPIOC		((GPIO_RegDef_t *)GPIOC_BASEADDR)
#define GPIOD		((GPIO_RegDef_t *)GPIOD_BASEADDR)
#define GPIOE		((GPIO_RegDef_t *)GPIOE_BASEADDR)
#define GPIOF		((GPIO_RegDef_t *)GPIOF_BASEADDR)
#define GPIOG		((GPIO_RegDef_t *)GPIOG_BASEADDR)
#define GPIOH		((GPIO_RegDef_t *)GPIOH_BASEADDR)
#define GPIOI		((GPIO_RegDef_t *)GPIOI_BASEADDR)

#define RCC			((RCC_RegDef_t *)RCC_BASEADDR)

#define EXTI 		((EXTI_RegDef_t *)EXTI_BASEADDR)

#define SYSCFG 		((SYSCFG_RegDef_t *)SYSCFG_BASEADDR)

#define SPI1		((SPI_RegDef_t*)SPI1_BASEADDR)
#define SPI2		((SPI_RegDef_t*)SPI2_BASEADDR)
#define SPI3		((SPI_RegDef_t*)SPI3_BASEADDR)
#define SPI4		((SPI_RegDef_t*)SPI4_BASEADDR)

#define I2C1		((I2C_RegDef_t*)I2C1_BASEADDR)
#define I2C2		((I2C_RegDef_t*)I2C1_BASEADDR)
#define I2C3		((I2C_RegDef_t*)I2C1_BASEADDR)

/*
 *  Clock Enable Macros for GPIOx peripharals
 */

#define GPIOA_PCLK_EN()		( RCC->AHB1ENR |= (1 << 0) )
#define GPIOB_PCLK_EN()		( RCC->AHB1ENR |= (1 << 1) )
#define GPIOC_PCLK_EN()		( RCC->AHB1ENR |= (1 << 2) )
#define GPIOD_PCLK_EN()		( RCC->AHB1ENR |= (1 << 3) )
#define GPIOE_PCLK_EN()		( RCC->AHB1ENR |= (1 << 4) )
#define GPIOF_PCLK_EN()		( RCC->AHB1ENR |= (1 << 5) )
#define GPIOG_PCLK_EN()		( RCC->AHB1ENR |= (1 << 6) )
#define GPIOH_PCLK_EN()		( RCC->AHB1ENR |= (1 << 7) )
#define GPIOI_PCLK_EN()		( RCC->AHB1ENR |= (1 << 8) )

/*
 *  Clock Enable Macros for I2Cx peripharals
 */


#define I2C1_PCLK_EN()		( RCC->APB1ENR |= (1 << 21) )
#define I2C2_PCLK_EN()		( RCC->APB1ENR |= (1 << 22) )
#define I2C3_PCLK_EN()		( RCC->APB1ENR |= (1 << 23) )

/*
 *  Clock Enable Macros for SPIx peripharals
 */

#define SPI1_PCLK_EN()		( RCC->APB2ENR |= (1 << 12) )
#define SPI2_PCLK_EN()		( RCC->APB1ENR |= (1 << 14) )
#define SPI3_PCLK_EN()		( RCC->APB1ENR |= (1 << 15) )
#define SPI4_PCLK_EN()		( RCC->APB2ENR |= (1 << 13) )

/*
 *  Clock Enable Macros for USARTx peripharals
 */

#define USART1_PCLK_EN()		( RCC->APB2ENR |= (1 << 4) )
#define USART2_PCLK_EN()		( RCC->APB1ENR |= (1 << 17) )
#define USART3_PCLK_EN()		( RCC->APB1ENR |= (1 << 18) )
#define UART4_PCLK_EN()			( RCC->APB1ENR |= (1 << 19) )
#define UART5_PCLK_EN()			( RCC->APB1ENR |= (1 << 20) )
#define USART6_PCLK_EN()		( RCC->APB2ENR |= (1 << 5) )

/*
 *  Clock Enable Macros for SYSCFG peripharals
 */
#define SYSCFG_PCLK_EN()		( RCC->APB2ENR |= (1 << 14) )

/*
 *  Clock Disable Macros for GPIOx peripharals
 */

#define GPIOA_PCLK_DI()		( RCC->AHB1ENR &= ~(1 << 0) )
#define GPIOB_PCLK_DI()		( RCC->AHB1ENR &= ~(1 << 1) )
#define GPIOC_PCLK_DI()		( RCC->AHB1ENR &= ~(1 << 2) )
#define GPIOD_PCLK_DI()		( RCC->AHB1ENR &= ~(1 << 3) )
#define GPIOE_PCLK_DI()		( RCC->AHB1ENR &= ~(1 << 4) )
#define GPIOF_PCLK_DI()		( RCC->AHB1ENR &= ~(1 << 5) )
#define GPIOG_PCLK_DI()		( RCC->AHB1ENR &= ~(1 << 6) )
#define GPIOH_PCLK_DI()		( RCC->AHB1ENR &= ~(1 << 7) )
#define GPIOI_PCLK_DI()		( RCC->AHB1ENR &= ~(1 << 8) )

/*
 *  Clock Disable Macros for I2Cx peripharals
 */

#define I2C1_PCLK_DI()		( RCC->APB1ENR &= ~(1 << 21) )
#define I2C2_PCLK_DI()		( RCC->APB1ENR &= ~(1 << 22) )
#define I2C3_PCLK_DI()		( RCC->APB1ENR &= ~(1 << 23) )

/*
 *  Clock Disable Macros for SPIx peripharals
 */

#define SPI1_PCLK_DI()		( RCC->APB2ENR &= ~(1 << 12) )
#define SPI2_PCLK_DI()		( RCC->APB1ENR &= ~(1 << 14) )
#define SPI3_PCLK_DI()		( RCC->APB1ENR &= ~(1 << 15) )
#define SPI4_PCLK_DI()		( RCC->APB2ENR &= ~(1 << 13) )

/*
 *  Clock Disable Macros for USARTx peripharals
 */

#define USART1_PCLK_DI()		( RCC->APB2ENR &= ~(1 << 4) )
#define USART2_PCLK_DI()		( RCC->APB1ENR &= ~(1 << 17) )
#define USART3_PCLK_DI()		( RCC->APB1ENR &= ~(1 << 18) )
#define UART4_PCLK_DI()			( RCC->APB1ENR &= ~(1 << 19) )
#define UART5_PCLK_DI()			( RCC->APB1ENR &= ~(1 << 20) )
#define USART6_PCLK_DI()		( RCC->APB2ENR &= ~(1 << 5) )

/*
 *  Clock Disable Macros for SYSCFG peripharals
 */

/*
 *  Macros to reset GPIOx peripherals
 */
#define GPIOA_REG_RESET() 		do { ( RCC->AHB1RSTR |= (1 << 0) );		( RCC->AHB1RSTR &= ~(1 << 0) );} while(0);
#define GPIOB_REG_RESET() 		do { ( RCC->AHB1RSTR |= (1 << 1) );		( RCC->AHB1RSTR &= ~(1 << 1) );} while(0);
#define GPIOC_REG_RESET() 		do { ( RCC->AHB1RSTR |= (1 << 2) );		( RCC->AHB1RSTR &= ~(1 << 2) );} while(0);
#define GPIOD_REG_RESET() 		do { ( RCC->AHB1RSTR |= (1 << 3) );		( RCC->AHB1RSTR &= ~(1 << 3) );} while(0);
#define GPIOE_REG_RESET() 		do { ( RCC->AHB1RSTR |= (1 << 4) );		( RCC->AHB1RSTR &= ~(1 << 4) );} while(0);
#define GPIOF_REG_RESET() 		do { ( RCC->AHB1RSTR |= (1 << 5) );		( RCC->AHB1RSTR &= ~(1 << 5) );} while(0);
#define GPIOG_REG_RESET() 		do { ( RCC->AHB1RSTR |= (1 << 6) );		( RCC->AHB1RSTR &= ~(1 << 6) );} while(0);
#define GPIOH_REG_RESET() 		do { ( RCC->AHB1RSTR |= (1 << 7) );		( RCC->AHB1RSTR &= ~(1 << 7) );} while(0);
#define GPIOI_REG_RESET() 		do { ( RCC->AHB1RSTR |= (1 << 8) );		( RCC->AHB1RSTR &= ~(1 << 8) );} while(0);

#define GPIO_BASEADDR_TO_CODE(x)  ( (x == GPIOA) ?0:\
									(x == GPIOB) ?1:\
									(x == GPIOC) ?2:\
									(x == GPIOD) ?3:\
									(x == GPIOE) ?4:\
									(x == GPIOF) ?5:\
									(x == GPIOG) ?6:\
									(x == GPIOH) ?7:\
									(x == GPIOI) ?8:0 )

/*
 * IRQ(Interrupt Request) Number of STM32F401xx MCU
 * NOTE: update these macros with valid values according to your MCU
 * TODO: You may complete this list for other pripherals
 */

#define IRQ_NO_EXTI0			6
#define IRQ_NO_EXTI1			7
#define IRQ_NO_EXTI2			8
#define IRQ_NO_EXTI3			9
#define IRQ_NO_EXTI4			10
#define IRQ_NO_EXTI9_5			23
#define IRQ_NO_EXTI15_10		40
#define IRQ_NO_SPI2				36

/*
 *  Macros for all the possible priority levels
 */
#define NVIC_IRQ_PRI0			0
#define NVIC_IRQ_PRI15			15

//Some generic macros

#define ENABLE 					1
#define DISABLE 				0
#define SET 					ENABLE
#define RESET 					DISABLE
#define GPIO_PIN_SET			SET
#define GPIO_PIN_RESET			RESET
#define FLAG_RESET				RESET
#define FLAG_SET				SET

/*
 *Bit position definitions of SPI_CR1 peipheral
*/
#define SPI_CR1_CPHA			0
#define SPI_CR1_CPOL			1
#define SPI_CR1_MSTR			2
#define SPI_CR1_BR				3
#define SPI_CR1_SPE				6
#define SPI_CR1_LSBFISRT		7
#define SPI_CR1_SSI				8
#define SPI_CR1_SSM				9
#define SPI_CR1_RXONLY			10
#define SPI_CR1_DFF				11
#define SPI_CR1_CRCNEXT			12
#define SPI_CR1_CRCEN			13
#define SPI_CR1_BIDIOE			14
#define SPI_CR1_BIDIMODE		15

/*
 *Bit position definitions of SPI_CR2 peipheral
*/
#define SPI_CR2_RXDMAEN			0
#define SPI_CR2_TXDMAEN			1
#define SPI_CR2_SSOE			2
#define SPI_CR2_FRF				4
#define SPI_CR2_ERRIE			5
#define SPI_CR2_RXNEIE			6
#define SPI_CR2_TXEIE			7

/*
 *Bit position definitions of SPI_SR peipheral
*/
#define SPI_SR_RXNE				0
#define SPI_SR_TXE				1
#define SPI_SR_CHSIDE			2
#define SPI_SR_UDR				3
#define SPI_SR_CRCERR			4
#define SPI_SR_MODF				5
#define SPI_SR_OVR				6
#define SPI_SR_BSY				7
#define SPI_SR_FRE				8

/**************************************************************
 * Bit position definitions I2C peripheral
 *************************************************************/
/*
 * Bit position definitions I2C_CR1
 */

#define I2C_CR1_PE				0
#define I2C_CR1_NOSTRECH		7
#define I2C_CR1_START			8
#define I2C_CR1_STOP			9
#define I2C_CR1_ACK				10
#define I2C_CR1_SWRST			15

/*
 * Bit position definitions I2C_CR2
 */

#define I2C_CR2_FREQ			0
#define I2C_CR2_ITERREN			8
#define I2C_CR2_ITEVTEN			9
#define I2C_CR2_ITBUFEN			10
#define I2C_CR2_DMAEN			11
#define I2C_CR2_LAST			12

/*
 * Bit position definitions I2C_OAR1
 */

#define I2C_OAR1_ADD0			0
#define I2C_OAR1_ADD71			1
#define I2C_OAR1_ADD98			8
#define I2C_OAR1_ADDMDOE		15

/*
 * Bit position definitions I2C_SR1
 */

#define I2C_SR1_SB				0
#define I2C_SR1_ADDR			1
#define I2C_SR1_BTF				2
#define I2C_SR1_ADD10			3
#define I2C_SR1_STOPF			4
#define I2C_SR1_RXNE			6
#define I2C_SR1_TXE				7
#define I2C_SR1_BERR			8
#define I2C_SR1_ARLO			9
#define I2C_SR1_AF				10
#define I2C_SR1_OVR				11
#define I2C_SR1_TIMEOUT			14

#include "stm32f401xx_gpio_driver.h"
#include "stm32f401xx_spi_driver.h"
#include "stm32f401xx_i2c_driver.h"

#endif /* INC_STM32F401XX_H_ */
