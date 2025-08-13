/*
 * stm32f401xx_spi_driver.c
 *
 *  Created on: Aug 8, 2025
 *      Author: Marko Topalović
 */

#include "stm32f401xx_spi_driver.h"

/*
 * Peripheral Clock setup
 */
/********************************************************************************
 * @fn				- SPI_PeriClockControl
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
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
		{
			if(pSPIx == SPI1)
			{
				SPI1_PCLK_EN();
			}
			else if (pSPIx == SPI2)
			{
				SPI2_PCLK_EN();
			}
			else if (pSPIx == SPI3)
			{
				SPI3_PCLK_EN();
			}
			else if (pSPIx == SPI4)
			{
				SPI4_PCLK_EN();
			}
		}
		else
		{
			if(pSPIx == SPI1)
			{
				SPI1_PCLK_DI();
			}else if (pSPIx == SPI2)
			{
				SPI2_PCLK_DI();
			}else if (pSPIx == SPI3)
			{
				SPI3_PCLK_DI();
			}else if (pSPIx == SPI4)
			{
				SPI4_PCLK_DI();
			}
		}
}

/*
 * Init and De - init
 */
/********************************************************************************
 * @fn				- SPI_Init
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
void SPI_Init(SPI_Handle_t *pSPIHandle )
{
	//Peripheral clock enable
	SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);

	//Configure SPI_CR1 register
	uint32_t tempreg = 0;

	//1. Configure the device mode
	tempreg |= pSPIHandle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR;

	//2. Configure the bus config
	if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD)
	{
		//bidi mode should be cleared
		tempreg &= ~(1 << SPI_CR1_BIDIMODE);
	}
	else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD)
	{
		//bidi mode should be set
		tempreg |= (1 << SPI_CR1_BIDIMODE);
	}
	else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY)
	{
		//BIDI mode should be cleared
		tempreg &= ~(1 << SPI_CR1_BIDIMODE);
		//RXONLY bit must be set
		tempreg |= (1 << SPI_CR1_RXONLY);
	}

	// 3. Configure the SPI serial clock speed (Baud rate)
	tempreg |= pSPIHandle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR;

	//4. Configure the DFF
	tempreg |= pSPIHandle->SPIConfig.SPI_DFF << SPI_CR1_DFF;

	//5. Configure the CPOL
	tempreg |= pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL;

	//6. Configure the CPHA
	tempreg |= pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA;

	pSPIHandle->pSPIx->CR1 = tempreg;
}

/********************************************************************************
 * @fn				- SPI_DeInit
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
void SPI_DeInit(SPI_RegDef_t *pSPIx)
{
		pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);	//Turn off SPI before resetting - good practice

		pSPIx->CR1 = 0;
		pSPIx->CR2 = 0;
		pSPIx->SR = 0;
		pSPIx->DR = 0;
}

uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName)
{
	if(pSPIx->SR & FlagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}

/*
 * Data Send and Receive
 */

/********************************************************************************
 * @fn				- SPI_SendData
 *
 * @brief          	-
 *
 * @param[in]	   	-
 * @param[in]		-
 * @param[in]		-
 *
 * @return			-
 *
 * @Note			- This is blocking call
 *
 *******************************************************************************/
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len)
{
	while(Len > 0)
	{
		// 1. Wait until TXE is set
		while(SPI_GetFlagStatus(pSPIx,SPI_TXE_FLAG) == FLAG_RESET);

		//2. Check the DFF bit i CR1
		if(pSPIx->CR1 & ( 1 << SPI_CR1_DFF))
		{
			//16 bit dff
			//1. Load the data in to the DR
			pSPIx->DR = *((uint16_t*)pTxBuffer);
			Len--;
			Len--;
			(uint16_t*)pTxBuffer++;
		}
		else
		{
			//8 bit DFF
			pSPIx->DR = *pTxBuffer;
			Len--;
			pTxBuffer++;
		}
	}
}

/********************************************************************************
 * @fn				- SPI_ReceiveData
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
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len)
{
	while(Len > 0)
		{
			// 1. Wait until RXNE is set
			while(SPI_GetFlagStatus(pSPIx,SPI_RXNE_FLAG) == FLAG_RESET);

			//2. Check the DFF bit i CR1
			if(pSPIx->CR1 & ( 1 << SPI_CR1_DFF))
			{
				//16 bit dff
				//1. Load the data from DR to RX buffer address
				*((uint16_t*)pRxBuffer) = pSPIx->DR;
				Len--;
				Len--;
				(uint16_t*)pRxBuffer++;
			}
			else
			{
				//8 bit DFF
				*pRxBuffer = pSPIx->DR;
				Len--;
				pRxBuffer++;
			}
		}
}

/*
 * IRQ Configuration and ISR handling
 */

/********************************************************************************
 * @fn				- SPI_IRQInterruptConfig
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
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pSPIx->CR1 |= (1 << SPI_CR1_SPE);
	}
	else
	{
		pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);
	}
}


/********************************************************************************
 * @fn				- SPI_SSIConfig
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
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pSPIx->CR1 |= (1 << SPI_CR1_SSI);
	}
	else
	{
		pSPIx->CR1 &= ~(1 << SPI_CR1_SSI);
	}
}

/********************************************************************************
 * @fn				- SPI_SSOEConfig
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
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pSPIx->CR2 |= (1 << SPI_CR2_SSOE);
	}
	else
	{
		pSPIx->CR2 &= ~(1 << SPI_CR2_SSOE);
	}
}

/*
 * IRQ Configuration and ISR handling
 */

/********************************************************************************
 * @fn				- SPI_IRQInterruptConfig
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
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{

}

/********************************************************************************
 * @fn				- SPI_IRQPriorityConfig
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
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{

}

/********************************************************************************
 * @fn				- SPI_IRQHandling
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
void SPI_IRQHandling(SPI_Handle_t *pHandle)
{

}

/*
 * Other Peripheral Control APIs
 */
