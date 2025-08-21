/*
 * stm32f401xx_i2c_driver.c
 *
 *  Created on: Aug 19, 2025
 *      Author: Marko Topalović
 */

#include "stm32f401xx_i2c_driver.h"

uint16_t AHB_PreScaler[8] = {2,4,8,16,64,128,256,512};
uint16_t APB1_PreScaler[4] = {2,4,8,16};

static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx);
static void I2C_ExecuteAdressPhase(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr);
static void I2C_ClearADDRFlag(I2C_RegDef_t *pI2Cx);
static void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx);

static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->CR1 |= ( 1 << I2C_CR1_START);
}

static void I2C_ExecuteAdressPhase(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr)
{
	SlaveAddr = SlaveAddr << 1;
	SlaveAddr &= ~(1); 			//SlaveAddr is Slave address + r/w bit=0 //~(1) -> 0b11111110 sa & dobijamo da je lsb 0
	pI2Cx->DR = SlaveAddr;
}

static void I2C_ClearADDRFlag(I2C_RegDef_t *pI2Cx)
{
	uint32_t dummyRead = pI2Cx->SR1;
	dummyRead = pI2Cx->SR2;
	(void)dummyRead;		//(void) → da kompajler ne kuka što se ta promenljiva ne koristi
}

static void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->CR1 |= ( 1 << I2C_CR1_STOP);
}

/********************************************************************************
 *  @fn				- I2C_PeripheralControl
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
void I2C_I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
		{
			pI2Cx->CR1 |= ( 1 << I2C_CR1_PE );
			//pI2CBaseAddress->CR1 |= I2C_CR1_PE_bit_Mask;
		}
		else
		{
			pI2Cx->CR1 &= ~(1 << 0);
		}
}


/********************************************************************************
 * @fn				- I2C_PeriClockControl
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
void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
		{
			if(pI2Cx == I2C1)
			{
				I2C1_PCLK_EN();
			}
			else if (pI2Cx == I2C2)
			{
				I2C2_PCLK_EN();
			}
			else if (pI2Cx == I2C3)
			{
				I2C3_PCLK_EN();
			}
		}
		else
		{
			if(pI2Cx == I2C1)
			{
				I2C1_PCLK_DI();
			}else if (pI2Cx == I2C2)
			{
				I2C2_PCLK_DI();
			}else if (pI2Cx == I2C3)
			{
				I2C3_PCLK_DI();
			}
		}
}

uint32_t RCC_GetPLLOutputClock(void)
{
	return 0;
}

uint32_t RCC_GetPLCK1Value(void)
{
	uint32_t pclk1, SystemClk;
	uint8_t clksrc, temp, ahbp, apb1p;

	clksrc = (RCC->CFGR >> 2) & 0x3;				//0x3 -> 0b0011

	if(clksrc == 0)
	{
		SystemClk = 16000000;						//HSI
	}
	else if(clksrc == 1)
	{
		SystemClk = 8000000;						//HSE
	}
	else if(clksrc == 2)
	{
		SystemClk = RCC_GetPLLOutputClock();		//PLL output
	}

	//For AHB
	temp = (RCC->CFGR >> 4) & 0xF;					//0xF -> 0b1111

	if(temp < 8)									//0x8 -> 0b1000
	{
		ahbp = 1;									//AHB Prescaler
	}
	else
	{
		ahbp = AHB_PreScaler[temp-8];
	}

	//For APB1
	temp = (RCC->CFGR >> 10) & 0x7;					//0xF -> 0b0111

	if(temp < 4)									//0x4 -> 0b0100
	{
		apb1p = 1;									//APB Prescaler
	}
	else
	{
		apb1p = APB1_PreScaler[temp-4];
	}

	pclk1 = (SystemClk / ahbp) / apb1p;

	return pclk1;
}
/********************************************************************************
 * @fn				- I2C_Init
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
void I2C_Init(I2C_Handle_t *pI2CHandle )
{
	uint32_t tempreg = 0;

	//ack control bit
	tempreg |= pI2CHandle -> I2C_Config.I2C_ACKControl << 10 ;
	pI2CHandle->pI2Cx->CR1 = tempreg;

	//Configure the FREQ field of CR2
	tempreg = 0;
	tempreg |= RCC_GetPLCK1Value() / 1000000U ;
	pI2CHandle->pI2Cx->CR2 = (tempreg & 0x3F) ;		//0b0011 1111

	//program the device own address
	tempreg |= pI2CHandle->I2C_Config.I2C_DeviceAddress << 1;
	tempreg |= ( 1 << 14);
	pI2CHandle->pI2Cx->OAR1 = tempreg;

	//CCR calculations
	uint16_t ccr_value = 0;
	tempreg = 0;
	if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
	{
		//Mode is standard mode
		ccr_value = RCC_GetPLCK1Value() / ( 2 * pI2CHandle->I2C_Config.I2C_SCLSpeed);
		tempreg |= (ccr_value & 0xFFF);
	}
	else
	{
		//Mode is fast mode
		tempreg |= ( 1 << 15 );
		tempreg |= (pI2CHandle->I2C_Config.I2C_FMDutyCycle << 14);
		if(pI2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_2)
		{
			ccr_value = RCC_GetPLCK1Value() / ( 3 * pI2CHandle->I2C_Config.I2C_SCLSpeed);
		}
		else
		{
			ccr_value = RCC_GetPLCK1Value() / ( 25 * pI2CHandle->I2C_Config.I2C_SCLSpeed);
		}
		tempreg |= (ccr_value & 0xFFF);
	}
	pI2CHandle->pI2Cx->CCR = tempreg;
}

/********************************************************************************
 * @fn				- I2C_DeInit
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
void I2C_DeInit(I2C_RegDef_t *pI2Cx)
{
	//TODO
}

uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName)
{
	if(pI2Cx->SR1 & FlagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}

/********************************************************************************
 * @fn				- I2C_MasterSendData
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
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxbuffer, uint32_t Len, uint8_t SlaveAddr)
{
	//1. Generate the START condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	//2. Confirm that start generation is completed by checking the SB flag in the SR1
	// Note: Until SB is cleared SCL will be streched (pulled to LOW)
	while( ! (I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB)));

	//3. Send the adrress of the slave with r/w bit set to w(0) (total 8 bits)
	I2C_ExecuteAdressPhase(pI2CHandle->pI2Cx, SlaveAddr);

	//4. Confirm that address phase is completed by checking the ADDR flag in teh SR1
	while( ! (I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR)));

	//5. Clear the ADDR flag according to its software sequence
	//Note: Until ADDR is cleared SCL will be streched (pulled to LOW)
	I2C_ClearADDRFlag(pI2CHandle->pI2Cx);

	//6. Send the data until Len becomes0
	while(Len > 0)
	{
		while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_TXE)); //Wati till TXE is set
		pI2CHandle->pI2Cx->DR = *pTxbuffer;
		pTxbuffer++;
		Len--;
	}

	//7. When Len becomes zero whait for TXE=1 and BTF=1 before generating the STOP ocndition
	// Note: TXE=1, BTF=1, means that both SR and DR are empty and next transmission should begin
	// when BTF=1 SCL will be streched (pulled to LOW)

	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_TXE));

	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_BTF));

	//8. Generate STOP condition and master need not to wait for the completion of stop condition.
	// Note: generating STOP, automacally clears the BTF
	I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

}

