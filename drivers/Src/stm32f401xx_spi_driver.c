/*
 * stm32f401xx_spi_driver.c
 *
 *  Created on: Aug 8, 2025
 *      Author: Marko Topalović
 */

#include "stm32f401xx_spi_driver.h"

static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle);

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
 * @fn				- SPI_SendDataIT
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
uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len)
{
	uint8_t state = pSPIHandle->TxState;

	if( state != SPI_BUSY_IN_TX)
	{
		//1. Save the Tx buffer address and Len information in some global variables
		pSPIHandle->pTxBuffer = pTxBuffer;
		pSPIHandle->TxLen = Len;

		//2. Mark th SPI state as busy in transmission so that
		//	no other code can take over same SPI peripheral until transmission is over
		pSPIHandle->TxState = SPI_BUSY_IN_TX;

		//3. Enable the TXEIE control bit to ger interrupt whenever TXE flag is set in SR
		pSPIHandle->pSPIx->CR2 |= ( 1 << SPI_CR2_TXEIE );
	}

	return state;
}

/********************************************************************************
 * @fn				- SPI_ReceiveDataIT
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
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len)
{
	uint8_t state = pSPIHandle->RxState;

		if( state != SPI_BUSY_IN_RX)
		{
			//1. Save the Rx buffer address and Len information
			pSPIHandle->pRxBuffer = pRxBuffer;
			pSPIHandle->RxLen = Len;

			//2. Mark SPI state as busy in reception
			pSPIHandle->RxState = SPI_BUSY_IN_RX;

			//3. // Enable RXNEIE to generate interrupt when RXNE flag is set
			pSPIHandle->pSPIx->CR2 |= ( 1 << SPI_CR2_RXNEIE );
		}

		return state;
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
	uint8_t temp1, temp2;
	//First lets check for TXE
	temp1 = pHandle->pSPIx->SR & ( 1 << SPI_SR_TXE );
	temp2 = pHandle->pSPIx->CR2 & ( 1 << SPI_CR2_TXEIE );

	if( temp1 && temp2 )
	{
		//Handle TXE
		spi_txe_interrupt_handle(pHandle);
	}

	//Check for RXNE
	temp1 = pHandle->pSPIx->SR & ( 1 << SPI_SR_RXNE );
	temp2 = pHandle->pSPIx->CR2 & ( 1 << SPI_CR2_RXNEIE );

	if( temp1 && temp2 )
		{
			//Handle RXNE
			spi_rxne_interrupt_handle(pHandle);
		}

	//Check for ovr flag
	temp1 = pHandle->pSPIx->SR & ( 1 << SPI_SR_OVR );
	temp2 = pHandle->pSPIx->CR2 & ( 1 << SPI_CR2_ERRIE );

	if( temp1 && temp2 )
		{
			//Handle ovr error
			spi_ovr_err_interrupt_handle(pHandle);
		}

}

//some helper funciton implementations

static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	//2. Check the DFF bit i CR1
	if(pSPIHandle->pSPIx->CR1 & ( 1 << SPI_CR1_DFF))
	{
		//16 bit dff
		//1. Load the data in to the DR
		pSPIHandle->pSPIx->DR = *((uint16_t*)pSPIHandle->pTxBuffer);
		pSPIHandle->TxLen--;
		pSPIHandle->TxLen--;
		(uint16_t*)pSPIHandle->pTxBuffer++;
	}
	else
	{
		//8 bit DFF
		pSPIHandle->pSPIx->DR = *pSPIHandle->pTxBuffer;
		pSPIHandle->TxLen--;
		pSPIHandle->pTxBuffer++;
	}

	if( ! pSPIHandle->TxLen )
	{
		//Txlen is zero, so close the SPI transmission and inform the application that TX is over

		//This prevents interrupts from settinf up of TXE Flag
		SPI_CloseTransmission(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_TX_CMPLT);
	}
}

static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	//Do rxing as per the diff
		if(pSPIHandle->pSPIx->CR1 & ( 1 << 11))
		{
			//16 bit
			*((uint16_t*)pSPIHandle->pRxBuffer) = (uint16_t)pSPIHandle->pSPIx->DR;
			pSPIHandle->RxLen -= 2;
			pSPIHandle->pRxBuffer--;
			pSPIHandle->pRxBuffer--;
		}
		else
		{
			//8 bit
			*(pSPIHandle->pRxBuffer) = (uint8_t) pSPIHandle->pSPIx->DR;
			pSPIHandle->RxLen --;
			pSPIHandle->pRxBuffer--;
		}

		if( ! pSPIHandle->RxLen )
		{
			//This prevents interrupts from settinf up of TXE Flag
			SPI_CloseReception(pSPIHandle);
			SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_RX_CMPLT);
		}
}

static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	uint8_t temp;

	//1. clear the ovr flag
	if(pSPIHandle->TxState != SPI_BUSY_IN_TX)
	{
		temp = pSPIHandle->pSPIx->DR;
		temp = pSPIHandle->pSPIx->SR;
	}
	(void)temp;
	//2. inform th application
	SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_OVR_ERR);
}

void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->CR2 &= ~( 1 << SPI_CR2_TXEIE);
	pSPIHandle->pTxBuffer = NULL;
	pSPIHandle->TxLen = 0;
	pSPIHandle->TxState = SPI_READY;
}

void SPI_CloseReception(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->CR2 &= ~( 1 << SPI_CR2_RXNEIE);
	pSPIHandle->pRxBuffer = NULL;
	pSPIHandle->RxLen = 0;
	pSPIHandle->RxState = SPI_READY;
}

void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx)
{
	uint8_t temp;
	temp = pSPIx -> DR;
	temp = pSPIx -> SR;
	(void)temp;
}

__weak void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEv)
{
	//This is a weak implementation . the application may override this function
}


/*
 * Other Peripheral Control APIs
 */
