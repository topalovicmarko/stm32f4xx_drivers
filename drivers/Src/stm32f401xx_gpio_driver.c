/*
 * stm32f401xx_gpio_driver.c
 *
 *  Created on: Aug 1, 2025
 *      Author: Marko Topalović
 */

#include "stm32f401xx_gpio_driver.h"

/*
 * Peripheral Clock setup
 */

/********************************************************************************
 * @fn				- GPIO_PeriClockControl
 *
 * @brief          	- This function enables or disables peripheral clock for the given GPIO port
 *
 * @param[in]	   	- base address of the gpio peripherals
 * @param[in]		- ENALBLE or DISABLE macros
 * @param[in]		-
 *
 * @return			-	none
 *
 * @Note			-	none
 *
 *******************************************************************************/
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_EN();
		}else if (pGPIOx == GPIOB)
		{
			GPIOB_PCLK_EN();
		}else if (pGPIOx == GPIOC)
		{
			GPIOC_PCLK_EN();
		}else if (pGPIOx == GPIOD)
		{
			GPIOD_PCLK_EN();
		}else if (pGPIOx == GPIOE)
		{
			GPIOE_PCLK_EN();
		}else if (pGPIOx == GPIOF)
		{
			GPIOF_PCLK_EN();
		}else if (pGPIOx == GPIOG)
		{
			GPIOG_PCLK_EN();
		}else if (pGPIOx == GPIOH)
		{
			GPIOH_PCLK_EN();
		}else if (pGPIOx == GPIOI)
		{
			GPIOI_PCLK_EN();
		}
	}
	else
	{
		if(pGPIOx == GPIOA)
				{
					GPIOA_PCLK_DI();
				}else if (pGPIOx == GPIOB)
				{
					GPIOB_PCLK_DI();
				}else if (pGPIOx == GPIOC)
				{
					GPIOC_PCLK_DI();
				}else if (pGPIOx == GPIOD)
				{
					GPIOD_PCLK_DI();
				}else if (pGPIOx == GPIOE)
				{
					GPIOE_PCLK_DI();
				}else if (pGPIOx == GPIOF)
				{
					GPIOF_PCLK_DI();
				}else if (pGPIOx == GPIOG)
				{
					GPIOG_PCLK_DI();
				}else if (pGPIOx == GPIOH)
				{
					GPIOH_PCLK_DI();
				}else if (pGPIOx == GPIOI)
				{
					GPIOI_PCLK_DI();
				}
	}
}

/*
 * Init and De - init
 */

/********************************************************************************
 * @fn				- GPIO_Init
 *
 * @brief          	-
 *
 * @param[in]	   	-
 * @param[in]		-
 * @param[in]		-
 *
 * @return			-
 *
 * @Note			-
 *
 *******************************************************************************/
void GPIO_Init(GPIO_Handle_t *pGPIOHandle )
{

}

/********************************************************************************
 * @fn				- GPIO_DeInit
 *
 * @brief          	-
 *
 * @param[in]	   	-
 * @param[in]		-
 * @param[in]		-
 *
 * @return			-
 *
 * @Note			-
 *
 *******************************************************************************/
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{

}

/*
 * Data read and write
 */

/********************************************************************************
 * @fn				- GPIO_ReadFromInputPin
 *
 * @brief          	-
 *
 * @param[in]	   	-
 * @param[in]		-
 * @param[in]		-
 *
 * @return			-
 *
 * @Note			-
 *
 *******************************************************************************/
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{

	return 0;
}

/********************************************************************************
 * @fn				- GPIO_ReadFromInputPort
 *
 * @brief          	-
 *
 * @param[in]	   	-
 * @param[in]		-
 * @param[in]		-
 *
 * @return			-
 *
 * @Note			-
 *
 *******************************************************************************/
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{

	return 0;
}

/********************************************************************************
 * @fn				- GPIO_WriteToOutputPin
 *
 * @brief          	-
 *
 * @param[in]	   	-
 * @param[in]		-
 * @param[in]		-
 *
 * @return			-
 *
 * @Note			-
 *
 *******************************************************************************/
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value)
{

}

/********************************************************************************
 * @fn				- GPIO_WriteToOutputPort
 *
 * @brief          	-
 *
 * @param[in]	   	-
 * @param[in]		-
 * @param[in]		-
 *
 * @return			-
 *
 * @Note			-
 *
 *******************************************************************************/
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value)
{

}

/********************************************************************************
 * @fn				- GPIO_ToggleOutputPin
 *
 * @brief          	-
 *
 * @param[in]	   	-
 * @param[in]		-
 * @param[in]		-
 *
 * @return			-
 *
 * @Note			-
 *
 *******************************************************************************/
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{

}

/*
 * IRQ Configuration and ISR handlig
 */

/********************************************************************************
 * @fn				- GPIO_IRQConfig
 *
 * @brief          	-
 *
 * @param[in]	   	-
 * @param[in]		-
 * @param[in]		-
 *
 * @return			-
 *
 * @Note			-
 *
 *******************************************************************************/
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnorDi)
{

}

/********************************************************************************
 * @fn				- GPIO_IRQHandling
 *
 * @brief          	-
 *
 * @param[in]	   	-
 * @param[in]		-
 * @param[in]		-
 *
 * @return			-
 *
 * @Note			-
 *
 *******************************************************************************/
void GPIO_IRQHandling(uint8_t PinNumber)
{

}
