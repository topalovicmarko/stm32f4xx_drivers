/*
 * 007spi_cmd_handling.c
 *
 *  Created on: Aug 14, 2025
 *      Author: Marko Topalović
 */

#include <string.h>
#include "stm32f401xx.h"

//Command codes
#define COMMAND_LED_CTRL		0x50
#define COMMAND_SENSOR_READ		0x51
#define COMMAND_LED_READ		0x52
#define COMMAND_PRINT			0x53
#define COMMAND_ID_READ			0x54

#define LED_ON		1
#define LED_OFF		0

//Arduino analog pins
#define ANALOG_PIN0		0
#define ANALOG_PIN1		1
#define ANALOG_PIN2		2
#define ANALOG_PIN3		3
#define ANALOG_PIN4		4

//Arduino led
#define	LED_PIN			9

void delay(void)
{
	for(uint32_t i = 0 ; i < 500000/2 ; i++);
}

/*
* PB15 --> SPI2_MOSI
* PB14 --> SPI2_MISO
* PB13 --> SPI2_SCLK
* PB12 --> SPI2_NSS
* ALT function mode : 5
*/

void SPI2_GPIOInits()
{
	GPIO_Handle_t SPIPins;

	SPIPins.pGPIOx = GPIOB;
	SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTF;
	SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
	SPIPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	//SCLK
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIO_Init(&SPIPins);

	//MOSI
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
	GPIO_Init(&SPIPins);

	//MISO
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
	GPIO_Init(&SPIPins);

	//NSS
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GPIO_Init(&SPIPins);
}

void SPI2_Inits(void)
{
	SPI_Handle_t SPI2handle;

	SPI2handle.pSPIx = SPI2;
	SPI2handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV8;	//generates sclk 2Mhz
	SPI2handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
	SPI2handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI2handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPI2handle.SPIConfig.SPI_SSM = SPI_SSM_DI ;	//Hardware slave managment enabled for NSS pin

	SPI_Init(&SPI2handle);
}

void GPIO_ButtonInit(void)
{
	GPIO_Handle_t GpioBtn;

	//This is btn gpio configuration
	GpioBtn.pGPIOx = GPIOC;
	GpioBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GpioBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_Init(&GpioBtn);
}

uint8_t SPI_VerifyResponse(uint8_t ackbyte)
{
	if(ackbyte == 0xF5)
	{
		//ack
		return 1;
	}

	return 0;
}


int main(void)
{

	uint8_t dummy_write = 0xff;
	uint8_t dummy_read;

	GPIO_ButtonInit();

	//This function is used to initialize the GPIO pinns to behave as SPI2 pins
	SPI2_GPIOInits();

	//This function is used to initialize the SPI2 peripheral parameters
	SPI2_Inits();

	/*
	 * Making SSOE 1 does NSS output enable
	 * The NSS pin is automacally managed by the hardware
	 * i.e when SPE=1 , NSS will be pulled to low
	 * and NSS pinn will be high when SPE=0
	 */
	SPI_SSOEConfig(SPI2, ENABLE);

	while(1)
	{
		//wait till button is pressed
		while(GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13));

		//to avoid button de-bouncing related issues 200ms of delay
		delay();

		//Enable the SPI2 peripheral
		SPI_PeripheralControl(SPI2,ENABLE);

		//1. CMD_LED_CTRL	<pin no(1)>		<value(1)>

		uint8_t commandcode = COMMAND_LED_CTRL;
		uint8_t ackbyte;
		uint8_t args[2];

		//send command
		SPI_SendData(SPI2, &commandcode, 1);

		//do dummy read to clear off the RXNE
		SPI_ReceiveData(SPI2, &dummy_read, 1);

		//send some dummy bits (1byte) to fetch to response from the slave
		SPI_SendData(SPI2, &dummy_write, 1);

		//read the ack byte received
		SPI_ReceiveData(SPI2, &ackbyte, 1);

		if (SPI_VerifyResponse(ackbyte))
		{
			//send arguments
			args[0]= LED_PIN;
			args[1]= LED_ON;
			SPI_SendData(SPI2, args, 2);
		}
		//end of COMMAND_LED_CTRL

		//2. CMD_SENSOR_READ <analog pin number(0)>

		//wait till button is pressed
		while(GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13));

		//to avoid button de-bouncing related issues 200ms of delay
		delay();

		commandcode = COMMAND_SENSOR_READ;

		//send command
		SPI_SendData(SPI2, &commandcode, 1);

		//do dummy read to clear off the RXNE
		SPI_ReceiveData(SPI2, &dummy_read, 1);

		//send some dummy bits (1byte) to fetch to response from the slave
		SPI_SendData(SPI2, &dummy_write, 1);

		//read the ack byte received
		SPI_ReceiveData(SPI2, &ackbyte, 1);

		if (SPI_VerifyResponse(ackbyte))
		{
			args[0]= ANALOG_PIN0;

			//send arguments
			SPI_SendData(SPI2, args, 1);	//sending 1 byte of

			//do dummy read to clear off the RXNE
			SPI_ReceiveData(SPI2, &dummy_read, 1);

			// insert some delay so that slave can ready with the data
			delay();

			//send some dummy bits (1byte) to fetch to response from the slave
			SPI_SendData(SPI2, &dummy_write, 1);

			uint8_t analog_read;
			SPI_ReceiveData(SPI2, &analog_read, 1);
		}

		//3. COMMAND_LED_READ <pin no(1)>

		// Wait for button press
		while(GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13));
		// For the De-bouncing of the Button
		delay();

		//-----------------------------------------
		// Execute CMD_LED_READ <pin no(1)>
		commandcode = COMMAND_LED_READ;
		// Send Data
		SPI_SendData(SPI2, &commandcode, 1);
		SPI_ReceiveData(SPI2, &dummy_read, 1); // Dummy Read to clear the RXNE
		// Receive Data
		SPI_SendData(SPI2, &dummy_write, 1); // Send dummy byte to fetch the response from the slave
		SPI_ReceiveData(SPI2, &ackbyte, 1);
		if( SPI_VerifyResponse(ackbyte) )
		{
			// Send Arguments
			args[0] = LED_PIN;
			SPI_SendData(SPI2, args, 1);
			SPI_ReceiveData(SPI2, &dummy_read, 1); // Dummy Read to clear the RXNE
			delay();
			// Receive Data
			SPI_SendData(SPI2, &dummy_write, 1); // Send dummy byte to fetch the response from the slave
			uint8_t led_read;
			SPI_ReceiveData(SPI2, &led_read, 1);
		}

		//4. COMMAND_PRINT <len(2)> 	<message(len)>
		// Wait for button press
		while(GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13));
		// For the De-bouncing of the Button
		delay();


		// Execute CMD_PRINT <length(2)> <message(length)>
		commandcode = COMMAND_PRINT;
		// Send Data
		SPI_SendData(SPI2, &commandcode, 1);
		SPI_ReceiveData(SPI2, &dummy_read, 1); // Dummy Read to clear the RXNE
		// Receive Data
		SPI_SendData(SPI2, &dummy_write, 1); // Send dummy byte to fetch the response from the slave
		SPI_ReceiveData(SPI2, &ackbyte, 1);
		if( SPI_VerifyResponse(ackbyte) )
		{
			// Send Arguments
			char secret_message[] = "This is a Secrete Message for the Arduino";
			args[0] = strlen(secret_message);
			SPI_SendData(SPI2, args, 1);
			SPI_ReceiveData(SPI2, &dummy_read, 1); // Dummy Read to clear the RXNE
			SPI_SendData(SPI2, (uint8_t*)secret_message, strlen(secret_message));
		}


		//5.COMMAND_ID_READ
		// Wait for button press
		while(GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13));
		// For the De-bouncing of the Button
		delay();


		// Execute CMD_ID_READ
		commandcode = COMMAND_ID_READ;
		// Send Data
		SPI_SendData(SPI2, &commandcode, 1);
		SPI_ReceiveData(SPI2, &dummy_read, 1); // Dummy Read to clear the RXNE
		// Receive Data
		SPI_SendData(SPI2, &dummy_write, 1); // Send dummy byte to fetch the response from the slave
		SPI_ReceiveData(SPI2, &ackbyte, 1);
		if( SPI_VerifyResponse(ackbyte) )
		{
			uint8_t board_id[10];
			for(int i=0; i<10; i++)
			{
			// Receive A Byte
			SPI_SendData(SPI2, &dummy_write, 1); // Send dummy byte to fetch the response from the slave
			SPI_ReceiveData(SPI2, &board_id[i], 1);
			}

		}


		//confirm SPI is not busy
		while( SPI_GetFlagStatus(SPI2, SPI_BUSY_FLAG) );

		//Disable the SPI2 peripheral
		SPI_PeripheralControl(SPI2,DISABLE);

	}

	return 0;
}
