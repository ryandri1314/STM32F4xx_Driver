/*
 * spi_Master_Slave.c
 *
 *  Created on: Oct 29, 2024
 *      Author: mary_uri
 */

#include "stm32f4xx.h"
#include "stm32f4xx_spi.h"
#include "stm32f4xx_gpio.h"

#include "string.h"

/*
 * PB12	->	NSS
 * PB13	->	SCK
 * PB14	->	MISO
 * PB15	->	MOSI
 * ALT FUNC MODE: 5
 *
 * */

void SPI_GPIO_Init() {
	GPIO_Handle_t SPI_Pins;
	SPI_Pins.pGPIOx = GPIOB;
	SPI_Pins.pGPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALT_FUNC;
	SPI_Pins.pGPIO_PinConfig.GPIO_PinAltFuncMode = 5;
	SPI_Pins.pGPIO_PinConfig.GPIO_PinOPType = GPIO_OTY_PUSH_PULL;
	SPI_Pins.pGPIO_PinConfig.GPIO_PinPUPD = GPIO_NO_PULL_UP_PULL_DOWN;
	SPI_Pins.pGPIO_PinConfig.GPIO_PinSpeed = GPIO_SPE_VHIGH;

	// Set NSS Pin
	SPI_Pins.pGPIO_PinConfig.GPIO_PinNumber = 12;
	GPIO_Init(&SPI_Pins);

	// Set SCK Pin
	SPI_Pins.pGPIO_PinConfig.GPIO_PinNumber = 13;
	GPIO_Init(&SPI_Pins);

	// Set MISO Pin
	SPI_Pins.pGPIO_PinConfig.GPIO_PinNumber = 14;
	GPIO_Init(&SPI_Pins);

	// Set MOSI Pin
	SPI_Pins.pGPIO_PinConfig.GPIO_PinNumber = 15;
	GPIO_Init(&SPI_Pins);
}

void SPI_Inits() {
	SPI_Handle_t SPI2_Handle;
	SPI2_Handle.pSPIx = SPI2;
	SPI2_Handle.SPI_Config.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2_Handle.SPI_Config.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2_Handle.SPI_Config.SPI_DFF = SPI_DFF_8BIT;
	SPI2_Handle.SPI_Config.SPI_SclkSpeed = SPI_SCLK_SPEED_D8; /*	Set Clock is 2MHz	*/
	SPI2_Handle.SPI_Config.SPI_CPOL = SPI_CPOL_HIGH;
	SPI2_Handle.SPI_Config.SPI_CPHA = SPI_CPHA_LOW;
	SPI2_Handle.SPI_Config.SPI_SSM = SPI_SSM_DI; /*		Disable Software Slave Management for NSS Pin	*/

	SPI_Init(&SPI2_Handle);
}

int main() {
	char *user_chars = "I love you, Maris!";
	SPI_GPIO_Init();
	SPI_Inits();
	SPI_PeripheralControl(SPI2, ENABLE);
	SPI_SSOEConfig(SPI2, ENABLE);

	SPI_SendData(SPI2, (uint8_t *)user_chars, strlen(user_chars));

	while (SPI_GetFlagStatus(SPI2, SPI_BSY_FLAG));
	SPI_PeripheralControl(SPI2, DISABLE);

	while (1);

	return 0;
}
