/*
 * stm32f401xx.h
 *
 *  Created on: July 31, 2025
 *      Author: Marko Topalović
 */

#ifndef INC_STM32F401XX_H_
#define INC_STM32F401XX_H_

#include<stdint.h>

#define __vo volatile

/*
 * Base addresses of Flash and SRAM memories
 */

#define FLASH_BASEADDR			0x08000000U				/*!<explain this macro briefly here */
#define SRAM1_BASEADDR			0x20000000U	//112KB		/*!<explain this macro briefly here */
#define SRAM2_BASEADDR			0x2001C000U				/*!<explain this macro briefly here */
#define ROM_BASEADDR			0x1FFF0000U
#define SRAM 					SRAM1_BASEADDR


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
#define GPIOJ_BASEADDR			(AHB1PERIPH_BASEADDR + 0x2400)
#define GPIOK_BASEADDR			(AHB1PERIPH_BASEADDR + 0x2800)

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
#define SYSCFG_BASEADDR			(APB2PERIPH_BASEADDR + 0x3800)
#define USART1_BASEADDR			(APB2PERIPH_BASEADDR + 0x1000)
#define USART2_BASEADDR			(APB2PERIPH_BASEADDR + 0x1400)

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

typedef struct
{


}RCC_RegDef_t;

/*
 * Peripheral definitions (Peripheral base addresses typecasted to xxx_RegDef_t)
 */

#define GPIOA		(GPIO_RegDef_t *)GPIOA_BASEADDR
#define GPIOB		(GPIO_RegDef_t *)GPIOB_BASEADDR
#define GPIOC		(GPIO_RegDef_t *)GPIOC_BASEADDR
#define GPIOD		(GPIO_RegDef_t *)GPIOD_BASEADDR
#define GPIOE		(GPIO_RegDef_t *)GPIOE_BASEADDR
#define GPIOF		(GPIO_RegDef_t *)GPIOF_BASEADDR
#define GPIOG		(GPIO_RegDef_t *)GPIOG_BASEADDR
#define GPIOH		(GPIO_RegDef_t *)GPIOH_BASEADDR
#define GPIOI		(GPIO_RegDef_t *)GPIOI_BASEADDR
#define GPIOJ		(GPIO_RegDef_t *)GPIOJ_BASEADDR
#define GPIOK		(GPIO_RegDef_t *)GPIOK_BASEADDR


#endif /* INC_STM32F401XX_H_ */
