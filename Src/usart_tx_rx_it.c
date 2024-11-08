/*
 * usart_tx_rx_it.c
 *
 *  Created on: Nov 7, 2024
 *      Author: uri-mary
 */

#include "stdio.h"
#include "string.h"
#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_usart.h"

char package[6];
USART_Handle_t USART_Handle;
uint8_t received = 0, sent = 0;

void Initialize() {
	/*
	 * 	GPIO Init
	 * */
	GPIO_Handle_t GPIO_Handle;
	GPIO_Handle.pGPIOx = GPIOA;
	GPIO_Handle.pGPIO_PinConfig.GPIO_PinAltFuncMode = 7;
	GPIO_Handle.pGPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALT_FUNC;
	GPIO_Handle.pGPIO_PinConfig.GPIO_PinOPType = GPIO_OTY_PUSH_PULL;
	GPIO_Handle.pGPIO_PinConfig.GPIO_PinPUPD = GPIO_PIN_PULL_UP;
	GPIO_Handle.pGPIO_PinConfig.GPIO_PinSpeed = GPIO_SPE_VHIGH;

	// TX Pin
	GPIO_Handle.pGPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_2;
	GPIO_Init(&GPIO_Handle);

	// RX Pin
	GPIO_Handle.pGPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_3;
	GPIO_Init(&GPIO_Handle);

	/*
	 * 	USART2 Init
	 * */
	USART_Handle.pUSARTx = USART2;
	USART_Handle.USART_Config.USART_BaudRate = USART_BAUD_RATE_115200;
	USART_Handle.USART_Config.USART_HWFlowControl = USART_HW_CTRL_NONE;
	USART_Handle.USART_Config.USART_Mode = USART_MODE_TXRX;
	USART_Handle.USART_Config.USART_StopBits = USART_STOP_1_BIT;
	USART_Handle.USART_Config.USART_ParityControl = USART_PARITY_DIS;
	USART_Handle.USART_Config.USART_WordLength = USART_WORD_LEN_8BITS;

	USART_Init(&USART_Handle);
}

int main() {
	Initialize();
	USART_IRQInterruptConfig(IRQ_USART2, ENABLE);
	USART_PeripheralControl(USART2, ENABLE);

	while (1) {
		while (USART_ReceiveDataIT(&USART_Handle, (uint8_t *)package, 6));
		USART_SendDataIT(&USART_Handle, (uint8_t *)package, strlen(package));
	}

	return 0;
}

void USART2_IRQHandler(void) {
	USART_IRQHandling(&USART_Handle);
}

void USART_ApplicationEventCallback(USART_Handle_t *pUSART_Handle, uint8_t AppEv) {
	if (AppEv == USART_EVENT_RX_CMPL) {
		received = 1;
	} else if (AppEv == USART_EVENT_TX_CMPL) {
		sent = 1;
	}
}




