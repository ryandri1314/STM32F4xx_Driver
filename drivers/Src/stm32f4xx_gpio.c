/*
 * stm32f4xx_gpio.c
 *
 *  Created on: Oct 15, 2024
 *      Author: mary_uri
 */

#include "stm32f4xx_gpio.h"

void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t State) {
	if (State == ENABLE) {		/*	Enable Clock for GPIOx	*/
		if (pGPIOx == GPIOA) {
			GPIOA_PCLK_EN();
		} else if (pGPIOx == GPIOB) {
			GPIOB_PCLK_EN();
		} else if (pGPIOx == GPIOC) {
			GPIOC_PCLK_EN();
		} else if (pGPIOx == GPIOD) {
			GPIOD_PCLK_EN();
		} else if (pGPIOx == GPIOE) {
			GPIOE_PCLK_EN();
		} else if (pGPIOx == GPIOF) {
			GPIOF_PCLK_EN();
		} else if (pGPIOx == GPIOG) {
			GPIOG_PCLK_EN();
		} else if (pGPIOx == GPIOH) {
			GPIOH_PCLK_EN();
		} else {
			GPIOI_PCLK_EN();
		}
	} else {		/*		Disable Clock for GPIOx		*/
		if (pGPIOx == GPIOA) {
			GPIOA_PCLK_DIS();
		} else if (pGPIOx == GPIOB) {
			GPIOB_PCLK_DIS();
		} else if (pGPIOx == GPIOC) {
			GPIOC_PCLK_DIS();
		} else if (pGPIOx == GPIOD) {
			GPIOD_PCLK_DIS();
		} else if (pGPIOx == GPIOE) {
			GPIOE_PCLK_DIS();
		} else if (pGPIOx == GPIOF) {
			GPIOF_PCLK_DIS();
		} else if (pGPIOx == GPIOG) {
			GPIOG_PCLK_DIS();
		} else if (pGPIOx == GPIOH) {
			GPIOH_PCLK_DIS();
		} else {
			GPIOI_PCLK_DIS();
		}
	}
}

void GPIO_Init(GPIO_Handle_t *pGPIO_Handle) {
	/*	1 - Enable peripheral clock			*/
	GPIO_PeriClockControl(pGPIO_Handle->pGPIOx, ENABLE);


	uint32_t temp = 0;
	/*	2 - Configure mode of GPIO Pin		*/
	if (pGPIO_Handle->pGPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG) { /* Normal MODE */
		temp = (pGPIO_Handle->pGPIO_PinConfig.GPIO_PinMode << (2 * pGPIO_Handle->pGPIO_PinConfig.GPIO_PinNumber));
		pGPIO_Handle->pGPIOx->MODER &= ~(0x3 << (2 * pGPIO_Handle->pGPIO_PinConfig.GPIO_PinNumber));
		pGPIO_Handle->pGPIOx->MODER |= temp;

	} else { /* Interrupt MODE */
		if (pGPIO_Handle->pGPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT) {
			/*	Enable falling trigger	*/
			EXTI->FTSR |= (1 << pGPIO_Handle->pGPIO_PinConfig.GPIO_PinNumber);
			/*	Disable rising trigger	*/
			EXTI->RTSR &= ~(1 << pGPIO_Handle->pGPIO_PinConfig.GPIO_PinNumber);
		} else if (pGPIO_Handle->pGPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT) {
			/*	Enable rising trigger	*/
			EXTI->RTSR |= (1 << pGPIO_Handle->pGPIO_PinConfig.GPIO_PinNumber);
			/*	Disable falling trigger	*/
			EXTI->FTSR &= ~(1 << pGPIO_Handle->pGPIO_PinConfig.GPIO_PinNumber);
		} else if (pGPIO_Handle->pGPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT) {
			/*	Enable rising & falling trigger		*/
			EXTI->RTSR |= (1 << pGPIO_Handle->pGPIO_PinConfig.GPIO_PinNumber);
			EXTI->FTSR |= (1 << pGPIO_Handle->pGPIO_PinConfig.GPIO_PinNumber);
		}

		/*	Configure EXTICR register	*/
		uint8_t index, pos;
		index = pGPIO_Handle->pGPIO_PinConfig.GPIO_PinNumber / 4;
		pos = pGPIO_Handle->pGPIO_PinConfig.GPIO_PinNumber % 4;
		uint16_t port = GPIO_BASEADDR_TO_EXTICR_CODE(pGPIO_Handle->pGPIOx);
		SYSCFG_PCLK_EN();
		SYSCFG->EXTICR[index] |= (port << (4 * pos));

		/*	Disable Interrupt Mark	*/
		EXTI->IMR |= (1 << pGPIO_Handle->pGPIO_PinConfig.GPIO_PinNumber);
	}

	/*	3 - Configure speed					*/
	temp = (pGPIO_Handle->pGPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIO_Handle->pGPIO_PinConfig.GPIO_PinNumber));
	pGPIO_Handle->pGPIOx->OSPEEDR &= ~(0x3 << (2 * pGPIO_Handle->pGPIO_PinConfig.GPIO_PinNumber));
	pGPIO_Handle->pGPIOx->OSPEEDR |= temp;

	/*	4 - Configure Pull-up Pull-down		*/
	temp = (pGPIO_Handle->pGPIO_PinConfig.GPIO_PinPUPD << (2 * pGPIO_Handle->pGPIO_PinConfig.GPIO_PinNumber));
	pGPIO_Handle->pGPIOx->PUPDR &= ~(0x3 << (2 * pGPIO_Handle->pGPIO_PinConfig.GPIO_PinNumber));
	pGPIO_Handle->pGPIOx->PUPDR |= temp;

	/*	5 - Configure OpenType				*/
	temp = (pGPIO_Handle->pGPIO_PinConfig.GPIO_PinOPType << pGPIO_Handle->pGPIO_PinConfig.GPIO_PinNumber);
	pGPIO_Handle->pGPIOx->OTYPER &= ~(0x3 << pGPIO_Handle->pGPIO_PinConfig.GPIO_PinNumber);
	pGPIO_Handle->pGPIOx->OTYPER |= temp;

	/*	6 - Configure ALT Function			*/
	if (pGPIO_Handle->pGPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALT_FUNC) {
		uint8_t index, pos;
		index = pGPIO_Handle->pGPIO_PinConfig.GPIO_PinNumber / 8;
		pos = pGPIO_Handle->pGPIO_PinConfig.GPIO_PinNumber % 8;

		temp = (pGPIO_Handle->pGPIO_PinConfig.GPIO_PinAltFuncMode << (4 * pos));
		pGPIO_Handle->pGPIOx->AFR[index] &= ~(0xF << (4 * pos));
		pGPIO_Handle->pGPIOx->AFR[index] |= temp;
	}
}

void GPIO_DeInit(GPIO_RegDef_t *pGPIOx) {
	if (pGPIOx == GPIOA) {
	    GPIOA_REG_RESET();
	} else if (pGPIOx == GPIOB) {
	    GPIOB_REG_RESET();
	} else if (pGPIOx == GPIOC) {
	    GPIOC_REG_RESET();
	} else if (pGPIOx == GPIOD) {
	    GPIOD_REG_RESET();
	} else if (pGPIOx == GPIOE) {
	    GPIOE_REG_RESET();
	} else if (pGPIOx == GPIOF) {
	    GPIOF_REG_RESET();
	} else if (pGPIOx == GPIOG) {
	    GPIOG_REG_RESET();
	} else if (pGPIOx == GPIOH) {
	    GPIOH_REG_RESET();
	} else {
	    GPIOI_REG_RESET();
	}
}

uint8_t GPIO_ReadFromPin(GPIO_RegDef_t *pGPIOx, uint8_t GPIO_PinNumber) {
	uint8_t result;
	result = (uint8_t)(pGPIOx->IDR >> GPIO_PinNumber) & 0x00000001;
	return result;
}

uint16_t GPIO_ReadFromPort(GPIO_RegDef_t *pGPIOx) {
	uint16_t result;
	result = (uint16_t)(pGPIOx->IDR);
	return result;
}

void GPIO_WriteToPin(GPIO_RegDef_t *pGPIOx, uint8_t GPIO_PinNumber, uint8_t Value) {
	if (Value == GPIO_PIN_SET) {	/*	Write SET	*/
		pGPIOx->ODR |= (1 << GPIO_PinNumber);
	} else {	/*	Write RESET	*/
		pGPIOx->ODR &= ~(1 << GPIO_PinNumber);
	}
}
void GPIO_WriteToPort(GPIO_RegDef_t *pGPIOx, uint16_t Value) {
	pGPIOx->ODR = Value;
}

void GPIO_TogglePin(GPIO_RegDef_t *pGPIOx, uint8_t GPIO_PinNumber) {
	pGPIOx->ODR ^= (1 << GPIO_PinNumber);
}

void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t State) {
	if (State == ENABLE) { /**/
		if (IRQNumber <= 31) {
			*NVIC_ISER0 |= (1 << IRQNumber);
		} else if (IRQNumber >= 32 && IRQNumber <= 63) {
			*NVIC_ISER1 |= (1 << (IRQNumber % 32));
		} else if (IRQNumber >= 64 && IRQNumber < 96) {
			*NVIC_ISER2 |= (1 << (IRQNumber % 64));
		} else {
			*NVIC_ISER3 |= (1 << (IRQNumber % 32));
		}
	} else  {
		if (IRQNumber <= 31) {
			*NVIC_ICER0 |= (1 << IRQNumber);
		} else if (IRQNumber < 64) {
			*NVIC_ICER1 |= (1 << (IRQNumber % 64));
		} else if (IRQNumber >= 64 && IRQNumber < 96) {
			*NVIC_ICER2 |= (1 << (IRQNumber % 96));
		} else {
			*NVIC_ICER3 |= (1 << (IRQNumber % 32));
		}
	}
}

void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority) {
	uint8_t ipr = IRQNumber / 4;
	uint8_t irq = IRQNumber % 4;

	*(NVIC_IPR0 + ipr) |= (IRQPriority << (8 * irq + 4));
}

void GPIO_IRQHandling(uint8_t GPIO_PinNumber) {
	if (EXTI->PR & (1 << GPIO_PinNumber)) {
		EXTI->PR |= 1 << GPIO_PinNumber;
	}
}
