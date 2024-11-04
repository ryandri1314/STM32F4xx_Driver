/*
 * MArduino_SSTM32F4xx.c
 *
 *  Created on: Oct 29, 2024
 *      Author: mary_uri
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

SPI_Handle_t SPI_Handle;

/*
 * Initialize Function
 * */
void Initialize(void) {
	/*
	 * Initialize GPIO Pins
	 * */
	GPIO_Handle_t GPIO_Handle;
	GPIO_Handle.pGPIOx = GPIOB;
	GPIO_Handle.pGPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALT_FUNC;
	GPIO_Handle.pGPIO_PinConfig.GPIO_PinAltFuncMode = 5;
	GPIO_Handle.pGPIO_PinConfig.GPIO_PinOPType = GPIO_OTY_PUSH_PULL;
	GPIO_Handle.pGPIO_PinConfig.GPIO_PinPUPD = GPIO_NO_PULL_UP_PULL_DOWN;
	GPIO_Handle.pGPIO_PinConfig.GPIO_PinSpeed = GPIO_SPE_VHIGH;

		//Initialize NSS Pin
	GPIO_Handle.pGPIO_PinConfig.GPIO_PinNumber = 12;
	GPIO_Init(&GPIO_Handle);

		//Initialize SCK Pin
	GPIO_Handle.pGPIO_PinConfig.GPIO_PinNumber = 13;
	GPIO_Init(&GPIO_Handle);

		//Initialize MISO Pin
	GPIO_Handle.pGPIO_PinConfig.GPIO_PinNumber = 14;
	GPIO_Init(&GPIO_Handle);

		//Initialize MOSI Pin
	GPIO_Handle.pGPIO_PinConfig.GPIO_PinNumber = 15;
	GPIO_Init(&GPIO_Handle);

	GPIO_Handle_t LED;
	LED.pGPIOx = GPIOB;
	LED.pGPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_10;
	LED.pGPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUTPUT;
	LED.pGPIO_PinConfig.GPIO_PinSpeed = GPIO_SPE_VHIGH;
	LED.pGPIO_PinConfig.GPIO_PinOPType = GPIO_OTY_PUSH_PULL;
	LED.pGPIO_PinConfig.GPIO_PinPUPD = GPIO_NO_PULL_UP_PULL_DOWN;

	GPIO_Init(&LED);
	GPIO_WriteToPin(GPIOB, GPIO_PIN_10, RESET);

	/*
	 * Initialize SPI2
	 * */
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
	Initialize();

	/*	Enable SSOE bit to allow hardware management NSS Pin	*/
	SPI_SSOEConfig(SPI2, ENABLE);

	/*	Enable IRQInterrupt for SPI2	*/
	SPI_IRQPriorityConfig(IRQ_SPI2, 3);
	SPI_IRQInterruptConfig(IRQ_SPI2, ENABLE);
//	GPIO_WriteToPin(GPIOB, GPIO_PIN_10, SET);

	while (1) {
		//TODO
		/*	Enable Peripherals	*/
		SPI_PeripheralControl(SPI2, ENABLE);

		/*	Fetch data from Arduino in interrupt mode	*/
		while (SPI_SendDataIT(&SPI_Handle, &dummy, 1) == SPI_BSY_RX);
		while (SPI_ReceiveDataIT(&SPI_Handle, &readByte, 1) == SPI_BSY_RX);

		/*	Confirm SPI is not BSY	*/
		while (SPI_GetFlagStatus(SPI2, SPI_BSY_FLAG));

		/*	Disable Peripherals	*/
		SPI_PeripheralControl(SPI2, DISABLE);
	}


	return 0;
}

/*
 * Running when receive data from Arduino
 * */
void SPI2_IRQHandler(void) {
	SPI_IRQHandling(&SPI_Handle);
}

void SPI_ApplicationEventCallback(SPI_Handle_t *pSPI_Handle, uint8_t AppEv) {
	static uint32_t i = 0;
//	GPIO_TogglePin(GPIOB, GPIO_PIN_10);

	/*	When RXNE Event completed, data from DS will copy into package	*/
	if (AppEv == SPI_EVENT_RX_CMPLT) {
		package[i++] = readByte;
		if ((readByte == '\0') || (i == MAX_LENG)) {
			package[i - 1] = '\0';
			i = 0;
		}
	}
}


