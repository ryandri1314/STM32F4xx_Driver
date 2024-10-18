/**
 ******************************************************************************
 * @file           : main.c
 * @author         : Auto-generated by STM32CubeIDE
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */

#include <stdint.h>
#include <stm32f4xx.h>
#include "stm32f4xx_gpio.h"

int c = 0;

void delay() {
	for (uint32_t i = 0; i <= 200000; i++) {

	}
}

int main(void)
{
	GPIO_Handle_t LED, BUTTON;
	LED.pGPIOx = GPIOB;
	LED.pGPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_3;
	LED.pGPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUTPUT;
	LED.pGPIO_PinConfig.GPIO_PinSpeed = GPIO_SPE_VHIGH;
	LED.pGPIO_PinConfig.GPIO_PinOPType = GPIO_OTY_PUSH_PULL;
	LED.pGPIO_PinConfig.GPIO_PinPUPD = GPIO_NO_PULL_UP_PULL_DOWN;

	GPIO_Init(&LED);
	GPIO_WriteToPin(GPIOB, GPIO_PIN_3, RESET);

	BUTTON.pGPIOx = GPIOE;
	BUTTON.pGPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_13;
	BUTTON.pGPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
	BUTTON.pGPIO_PinConfig.GPIO_PinSpeed = GPIO_SPE_VHIGH;
	BUTTON.pGPIO_PinConfig.GPIO_PinPUPD = GPIO_PIN_PULL_UP;

	GPIO_Init(&BUTTON);
	GPIO_IRQPriorityConfig(IRQ_EXTI15_10, 3);
	GPIO_IRQInterruptConfig(IRQ_EXTI15_10, ENABLE);

    /* Loop forever */
	while (1) {
//		GPIO_TogglePin(GPIOB, GPIO_PIN_3);
//		delay();
	}
}

void EXTI15_10_IRQHandler() {
	delay();
	GPIO_IRQHandling(GPIO_PIN_13);
	GPIO_TogglePin(GPIOB, GPIO_PIN_3);
	c++;
}
