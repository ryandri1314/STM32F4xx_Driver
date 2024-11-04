/*
 * spi_Master_receive_data.c
 *
 *  Created on: Nov 4, 2024
 *      Author: uri-mary
 */

#include "stdio.h"
#include "string.h"

#include "stm32f4xx.h"
#include "stm32f4xx_spi.h"
#include "stm32f4xx_gpio.h"

/*
 * PB12	->	NSS
 * PB13	->	SCK
 * PB14	->	MISO
 * PB15	->	MOSI
 * ALT FUNC MODE: 5
 *
 * */
#define MAX_LENG	100

char package[MAX_LENG];
uint8_t readByte;

/*
 * Initialize Function
 * */

void Initialize() {
	/*
	 * Initialize GPIO Pins
	 * */
	GPIO_Handle_t GPIO_Pin;
	GPIO_Pin.pGPIOx = GPIOB;
	GPIO_Pin.pGPIO_PinConfig.GPIO_PinAltFuncMode = 5;
	GPIO_Pin.pGPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALT_FUNC;
	GPIO_Pin.pGPIO_PinConfig.GPIO_PinOPType = GPIO_OTY_PUSH_PULL;
	GPIO_Pin.pGPIO_PinConfig.GPIO_PinPUPD = GPIO_NO_PULL_UP_PULL_DOWN;
	GPIO_Pin.pGPIO_PinConfig.GPIO_PinSpeed = GPIO_SPE_VHIGH;

		//Initialize NSS Pin
	GPIO_Pin.pGPIO_PinConfig.GPIO_PinNumber = 12;
	GPIO_Init(&GPIO_Pin);

		//Initialize SCK Pin
	GPIO_Pin.pGPIO_PinConfig.GPIO_PinNumber = 13;
	GPIO_Init(&GPIO_Pin);

		//Initialize MISO Pin
	GPIO_Pin.pGPIO_PinConfig.GPIO_PinNumber = 14;
	GPIO_Init(&GPIO_Pin);

		//Initialize MOSI Pin
	GPIO_Pin.pGPIO_PinConfig.GPIO_PinNumber = 15;
	GPIO_Init(&GPIO_Pin);

	/*
	 * Initialize SPI2
	 * */
	SPI_Handle_t SPI_Handle;
	SPI_Handle.pSPIx = SPI2;
	SPI_Handle.SPI_Config.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI_Handle.SPI_Config.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI_Handle.SPI_Config.SPI_DFF = SPI_DFF_8BIT;
	SPI_Handle.SPI_Config.SPI_SclkSpeed = SPI_SCLK_SPEED_D32;
	SPI_Handle.SPI_Config.SPI_SSM = SPI_SSM_DI;
	SPI_Handle.SPI_Config.SPI_CPHA = SPI_CPHA_LOW;
	SPI_Handle.SPI_Config.SPI_CPOL = SPI_CPOL_LOW;

	SPI_Init(&SPI_Handle);
}

int main() {
	/*	Initialize	*/
	uint8_t dummy = 0xff;
	int i = 0;
	Initialize();

	/*	Enable Peripherals	*/
	SPI_PeripheralControl(SPI2, ENABLE);

	/*	Enable SSOE bit to allow hardware management NSS Pin	*/
	SPI_SSOEConfig(SPI2, ENABLE);

	while (1) {
		SPI_SendData(SPI2, &dummy, 1); /*	Clear TXE Flag	*/
		SPI_ReceiveData(SPI2, &readByte, 1);
//		for (i; i < 13; i++) {
//			SPI_SendData(SPI2, &dummy, 1);
//			SPI_ReceiveData(SPI2, &readByte, 1);
		package[i++] = readByte;
//		}
		if (i >= 13) {
			break;
		}
	}

	/*	Confirm SPI is not BSY	*/
	while (SPI_GetFlagStatus(SPI2, SPI_BSY_FLAG));

	/*	Disable Peripherals	*/
	SPI_PeripheralControl(SPI2, DISABLE);

	return 0;
}
