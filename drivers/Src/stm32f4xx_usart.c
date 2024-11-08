/*
 * stm32f4xx_usart.c
 *
 *  Created on: Nov 5, 2024
 *      Author: uri-mary
 */

#include "stm32f4xx_usart.h"
#include "stm32f4xx_rcc.h"
#include "stddef.h"

static void usart_tcie_interrupt_handle(USART_Handle_t *pUSART_Handle);
static void usart_txeie_interrupt_handle(USART_Handle_t *pUSART_Handle);
static void usart_rxneie_interrupt_handle(USART_Handle_t *pUSART_Handle);

/*
 *	Peripheral Clock Setup
 */
void USART_PeriClockControl(USART_RegDef_t *pUSARTx, uint8_t State) {
	if (State == ENABLE) {	/*	Enable CLK for USARTx		*/
		if (pUSARTx == USART1) {
			USART1_PCLK_EN();
		} else if (pUSARTx == USART2) {
			USART2_PCLK_EN();
		} else if (pUSARTx == USART3) {
			USART3_PCLK_EN();
		} else if (pUSARTx == UART4) {
			UART4_PCLK_EN();
		} else if (pUSARTx == UART5) {
			UART5_PCLK_EN();
		} else if (pUSARTx == USART6) {
			USART6_PCLK_EN();
		}
	} else if (State == DISABLE) {	/*	Disable CLK for USARTx	*/
		if (pUSARTx == USART1) {
			USART1_PCLK_DIS();
		} else if (pUSARTx == USART2) {
			USART2_PCLK_DIS();
		} else if (pUSARTx == USART3) {
			USART3_PCLK_DIS();
		} else if (pUSARTx == UART4) {
			UART4_PCLK_DIS();
		} else if (pUSARTx == UART5) {
			UART5_PCLK_DIS();
		} else if (pUSARTx == USART6) {
			USART6_PCLK_DIS();
		}
	}
}

/*
 *	Init & Deinit Func
 */
void USART_Init(USART_Handle_t *pUSART_Handle) {
	/*	Enable peripheral clock		*/
	USART_PeriClockControl(pUSART_Handle->pUSARTx, ENABLE);

	uint32_t tempReg = 0;
	/*	Configure CR1	*/
		// MODE
	if (pUSART_Handle->USART_Config.USART_Mode == USART_MODE_RX) {
		tempReg |= (1 << USART_CR1_RE);
	} else if (pUSART_Handle->USART_Config.USART_Mode == USART_MODE_TX) {
		tempReg |= (1 << USART_CR1_TE);
	} else if (pUSART_Handle->USART_Config.USART_Mode == USART_MODE_TXRX) {
		tempReg |= ((1 << USART_CR1_TE) | (1 << USART_CR1_RE));
	}
		// Parity bit
	if (pUSART_Handle->USART_Config.USART_ParityControl == USART_PARITY_EVEN) {
		tempReg |= (1 << USART_CR1_PCE);
	} else if (pUSART_Handle->USART_Config.USART_ParityControl == USART_PARITY_ODD) {
		tempReg |= (1 << USART_CR1_PS);
		tempReg |= (1 << USART_CR1_PCE);
	}
		// Word length
	tempReg |= (pUSART_Handle->USART_Config.USART_WordLength << USART_CR1_M);

	pUSART_Handle->pUSARTx->CR1 = tempReg;
	tempReg = 0;

	/*	Configure CR2	*/
		// Stop bit
	tempReg |= (pUSART_Handle->USART_Config.USART_StopBits << USART_CR2_STOP);
	pUSART_Handle->pUSARTx->CR2 = tempReg;
	tempReg = 0;

	/*	Configure CR3	*/
		// Configure HW Control
	if (pUSART_Handle->USART_Config.USART_HWFlowControl == USART_HW_CTRL_CTS) {
		tempReg |= (1 << USART_CR3_CTSE);
	} else if (pUSART_Handle->USART_Config.USART_HWFlowControl == USART_HW_CTRL_RTS) {
		tempReg |= (1 << USART_CR3_RTSE);
	} else if (pUSART_Handle->USART_Config.USART_HWFlowControl == USART_HW_CTRL_BOTH) {
		tempReg |= (1 << USART_CR3_RTSE) | (1 << USART_CR3_CTSE);
	}
	pUSART_Handle->pUSARTx->CR3 = tempReg;

	/*	Configure Baud rate	*/
	USART_SetBaudRate(pUSART_Handle->pUSARTx, pUSART_Handle->USART_Config.USART_BaudRate);

}
void USART_DeInit(USART_Handle_t *pUSART_Handle);

/*
 *	Send & Receive Data
 */
void USART_SendData(USART_Handle_t *pUSART_Handle, uint8_t *pTxBuffer, uint8_t Len) {
	uint16_t *pData;

	for (uint32_t i = 0; i < Len; i++) {
		// Wait for TXE is set
		while (!USART_GetFlagStatus(pUSART_Handle->pUSARTx, USART_FLAG_TXE));

		// Check length of Data Frame
		if (pUSART_Handle->USART_Config.USART_WordLength == USART_WORD_LEN_9BITS) { /*		9bits	*/
			// Load 2 byte data to DR
			pData = (uint16_t *)pTxBuffer;
			pUSART_Handle->pUSARTx->DR = (*pData & (uint16_t)0x1FF);

			// Check parity bit
			if (pUSART_Handle->USART_Config.USART_ParityControl == USART_PARITY_DIS) {
				// Sent 9 bits
				pTxBuffer++;
				pTxBuffer++;
			} else {
				// Sent 8 bits
				pTxBuffer++;
			}
		} else { /*		8bits	*/
			pUSART_Handle->pUSARTx->DR = (*pTxBuffer & (uint8_t)0xFF);
			pTxBuffer++;
		}
	}

	// Wait Flag TC is set
	while (!USART_GetFlagStatus(pUSART_Handle->pUSARTx, USART_FLAG_TC));
}

void USART_ReceiveData(USART_Handle_t *pUSART_Handle, uint8_t *pRxBuffer, uint8_t Len) {
	for (uint32_t i = 0; i < Len; i++) {
		// Wait RXNE Flag is set
		while (!USART_GetFlagStatus(pUSART_Handle->pUSARTx, USART_FLAG_RXNE));

		// Check word length
		if (pUSART_Handle->USART_Config.USART_WordLength == USART_WORD_LEN_9BITS) {
			// Check Parity bit
			if (pUSART_Handle->USART_Config.USART_ParityControl == USART_PARITY_DIS) { // Receive 9bit data
				*((uint16_t *)pRxBuffer) = ((uint16_t)pUSART_Handle->pUSARTx->DR & (uint16_t)0x1FF);
				pRxBuffer++;
			} else { // Receive 8bit data
				*pRxBuffer = (pUSART_Handle->pUSARTx->DR & (uint8_t)0xFF);
			}
			pRxBuffer++;
		} else {
			// Check Parity bit
			if (pUSART_Handle->USART_Config.USART_ParityControl == USART_PARITY_DIS) { // Receive 9bit data
				*pRxBuffer = (pUSART_Handle->pUSARTx->DR & (uint8_t)0xFF);
			} else { // Receive 8bit data
				*pRxBuffer = (pUSART_Handle->pUSARTx->DR & (uint8_t)0x7F);
			}
			pRxBuffer++;
		}
	}
}

uint8_t USART_SendDataIT(USART_Handle_t *pUSART_Handle, uint8_t *pTxBuffer, uint8_t Len) {
	uint8_t state = pUSART_Handle->TxState;
	if (state != USART_BSY_TX) {
		/*	1 - Save address of pTxBuffer into pUSART_Handle	*/
		pUSART_Handle->pTxBuffer = pTxBuffer;
		pUSART_Handle->TxLen = Len;

		/*	2 - Mark SPI is busy	*/
		pUSART_Handle->TxState = USART_BSY_TX;

		/*	3 - Enable TXEIE bit	*/
		pUSART_Handle->pUSARTx->CR1 |= (1 << USART_CR1_TXEIE);

		/*	4 - Enable TCIE bit		*/
		pUSART_Handle->pUSARTx->CR1 |= (1 << USART_CR1_TCIE);

		/*	Transmit data will be handle in interrupt	*/
	}
	return state;
}

uint8_t USART_ReceiveDataIT(USART_Handle_t *pUSART_Handle, uint8_t *pRxBuffer, uint8_t Len) {
	uint8_t state = pUSART_Handle->RxState;
	if (state != USART_BSY_RX) {
		/*	1 - Save address of pTxBuffer into pUSART_Handle	*/
		pUSART_Handle->pRxBuffer = pRxBuffer;
		pUSART_Handle->RxLen = Len;

		/*	2 - Mark SPI is busy	*/
		pUSART_Handle->RxState = USART_BSY_RX;
		(void)pUSART_Handle->pUSARTx->DR;

		/*	3 - Enable TXEIE bit	*/
		pUSART_Handle->pUSARTx->CR1 |= (1 << USART_CR1_RXNEIE);

		/*	Transmit data will be handle in interrupt	*/
	}
	return state;
}

/*
 *	IRQ Configuration & IRQ Handling
 */
void USART_IRQInterruptConfig(uint8_t IRQNumber, uint8_t State) {
	if (State == ENABLE) { /**/
		if (IRQNumber <= 31) {
			*NVIC_ISER0 |= (1 << IRQNumber);
		} else if (IRQNumber >= 32 && IRQNumber <= 63) {
			*NVIC_ISER1 |= (1 << (IRQNumber % 32));
		} else if (IRQNumber >= 64 && IRQNumber < 96) {
			*NVIC_ISER2 |= (1 << (IRQNumber % 32));
		} else {
			*NVIC_ISER3 |= (1 << (IRQNumber % 32));
		}

	} else  {
		if (IRQNumber <= 31) {
			*NVIC_ICER0 |= (1 << IRQNumber);
		} else if (IRQNumber < 64) {
			*NVIC_ICER1 |= (1 << (IRQNumber % 32));
		} else if (IRQNumber >= 64 && IRQNumber < 96) {
			*NVIC_ICER2 |= (1 << (IRQNumber % 32));
		} else {
			*NVIC_ICER3 = (1 << (IRQNumber % 32));
		}
	}
}

void USART_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority) {
	uint8_t ipr = IRQNumber / 4;
	uint8_t irq = IRQNumber % 4;

	*(NVIC_IPR0 + ipr) |= (IRQPriority << (8 * irq + 4));
}

void USART_IRQHandling(USART_Handle_t *pUSART_Handle) {
	uint8_t temp1, temp2;

	/*	TCIE Interrupt	*/
	temp1 = (pUSART_Handle->pUSARTx->SR & (1 << USART_SR_TC));
	temp2 = (pUSART_Handle->pUSARTx->CR1 & (1 << USART_CR1_TCIE));
	if (temp1 && temp2) {
		/*	Handle TCIE interrupt	*/
		usart_tcie_interrupt_handle(pUSART_Handle);
	}

	/*	TXEIE Interrupt	*/
	temp1 = (pUSART_Handle->pUSARTx->SR & (1 << USART_SR_TXE));
	temp2 = (pUSART_Handle->pUSARTx->CR1 & (1 << USART_CR1_TXEIE));
	if (temp1 && temp2) {
		/*	Handle TXEIE interrupt	*/
		usart_txeie_interrupt_handle(pUSART_Handle);
	}

	/*	RXNEIE Interrupt	*/
	temp1 = (pUSART_Handle->pUSARTx->SR & (1 << USART_SR_RXNE));
	temp2 = (pUSART_Handle->pUSARTx->CR1 & (1 << USART_CR1_RXNEIE));
	if (temp1 && temp2) {
		/*	Handle RXNEIE interrupt	*/
		usart_rxneie_interrupt_handle(pUSART_Handle);
	}

	/*	Handle CTSIE interrupt	*/
	temp1 = (pUSART_Handle->pUSARTx->SR & (1 << USART_SR_CTS));
	temp2 = (pUSART_Handle->pUSARTx->CR3 & (1 << USART_CR3_CTSIE));
	if (temp1 && temp2) {
		pUSART_Handle->pUSARTx->CR3 &= ~(1 << USART_CR3_CTSIE);

		// Call Application
		USART_ApplicationEventCallback(pUSART_Handle, USART_EVENT_CTS);
	}

	/*	Handle IDLEIE interrupt	*/
	temp1 = (pUSART_Handle->pUSARTx->SR & (1 << USART_SR_IDLE));
	temp2 = (pUSART_Handle->pUSARTx->CR1 & (1 << USART_CR1_IDLEIE));
	if (temp1 && temp2) {
		pUSART_Handle->pUSARTx->CR1 &= ~(1 << USART_CR1_IDLEIE);

		// Call Application
		USART_ApplicationEventCallback(pUSART_Handle, USART_EVENT_IDLE);
	}

	/*	Handle ORE interrupt	*/
	temp1 = (pUSART_Handle->pUSARTx->SR & (1 << USART_SR_ORE));
	temp2 = (pUSART_Handle->pUSARTx->CR1 & (1 << USART_CR1_RXNEIE));
	if (temp1 && temp2) {
		pUSART_Handle->pUSARTx->CR1 &= ~(1 << USART_CR1_RXNEIE);

		// Call Application
		USART_ApplicationEventCallback(pUSART_Handle, USART_EVENT_ORE);
	}

	/*	Handle PE interrupt	*/
	temp1 = (pUSART_Handle->pUSARTx->SR & (1 << USART_SR_PE));
	temp2 = (pUSART_Handle->pUSARTx->CR1 & (1 << USART_CR1_PEIE));
	if (temp1 && temp2) {
		pUSART_Handle->pUSARTx->CR1 &= ~(1 << USART_CR1_PEIE);

		// Call Application
		USART_ApplicationEventCallback(pUSART_Handle, USART_EVENT_PE);
	}

	/*	Handle LBD interrupt	*/
	temp1 = (pUSART_Handle->pUSARTx->SR & (1 << USART_SR_LBD));
	temp2 = (pUSART_Handle->pUSARTx->CR2 & (1 << USART_CR2_LBDIE));
	if (temp1 && temp2) {
		pUSART_Handle->pUSARTx->CR2 &= ~(1 << USART_CR2_LBDIE);

		// Call Application
		USART_ApplicationEventCallback(pUSART_Handle, USART_EVENT_LBD);
	}

	/*	Handle ERR interrupt	*/
	temp1 = (pUSART_Handle->pUSARTx->CR3 & (1 << USART_CR3_EIE));
	if (temp1) {
		temp2 = (pUSART_Handle->pUSARTx->SR);
		if (temp2 & (1 << USART_SR_ORE)) {
			USART_ApplicationEventCallback(pUSART_Handle, USART_EVENT_ORE);
		} else if (temp2 & (1 << USART_SR_FE)) {
			USART_ApplicationEventCallback(pUSART_Handle, USART_EVENT_FE);
		} else if (temp2 & (1 << USART_SR_NF)) {
			USART_ApplicationEventCallback(pUSART_Handle, USART_EVENT_NF);
		}
	}
}

void usart_tcie_interrupt_handle(USART_Handle_t *pUSART_Handle) {
	/*	If TxLen is 0, close transmit and call application	*/
	if (pUSART_Handle->TxState == USART_BSY_TX) {
		if (!pUSART_Handle->TxLen) {
			// Clear TC & TCIE bit
			pUSART_Handle->pUSARTx->SR &= ~(USART_SR_TC);
			pUSART_Handle->pUSARTx->CR1 &= ~(USART_CR1_TCIE);

			pUSART_Handle->TxState = USART_READY;
			pUSART_Handle->TxLen = 0;
			pUSART_Handle->pTxBuffer = NULL;

			// Call application
			USART_ApplicationEventCallback(pUSART_Handle, USART_EVENT_TX_CMPL);
		}
	}
}

void usart_txeie_interrupt_handle(USART_Handle_t *pUSART_Handle) {
	uint16_t *pData;
	if (pUSART_Handle->TxState == USART_BSY_TX) {
		if (pUSART_Handle->TxLen > 0) {
			if (pUSART_Handle->USART_Config.USART_WordLength == USART_WORD_LEN_9BITS) {
				pData = (uint16_t *)pUSART_Handle->pTxBuffer;
				pUSART_Handle->pUSARTx->DR = (*pData & (uint16_t)0x1FF);
				pUSART_Handle->pTxBuffer++;
				pUSART_Handle->TxLen--;
				if (pUSART_Handle->USART_Config.USART_ParityControl == USART_PARITY_DIS) {
					pUSART_Handle->pTxBuffer++;
					pUSART_Handle->TxLen--;
				}
			} else {
				pUSART_Handle->pUSARTx->DR = (*pUSART_Handle->pTxBuffer & (uint8_t)0xFF);
				pUSART_Handle->pTxBuffer++;
				pUSART_Handle->TxLen--;
			}
		} else if (!pUSART_Handle->TxLen) {
			pUSART_Handle->pUSARTx->CR1 &= ~(1 << USART_CR1_TXEIE);
			pUSART_Handle->TxLen = 0;
			pUSART_Handle->TxState = USART_READY;
			USART_ApplicationEventCallback(pUSART_Handle, USART_EVENT_TX_CMPL);
		}
	}
}

void usart_rxneie_interrupt_handle(USART_Handle_t *pUSART_Handle) {
	if (pUSART_Handle->RxState == USART_BSY_RX) {
		if (pUSART_Handle->RxLen > 0) {
			if (pUSART_Handle->USART_Config.USART_WordLength == USART_WORD_LEN_9BITS) {
				// Check Parity bit
				if (pUSART_Handle->USART_Config.USART_ParityControl == USART_PARITY_DIS) { // Receive 9bit data
					*((uint16_t *)pUSART_Handle->pRxBuffer) = ((uint16_t)pUSART_Handle->pUSARTx->DR & (uint16_t)0x1FF);
					pUSART_Handle->pRxBuffer++;
					pUSART_Handle->RxLen--;
				} else { // Receive 8bit data
					*pUSART_Handle->pRxBuffer = (pUSART_Handle->pUSARTx->DR & (uint8_t)0xFF);
				}
				pUSART_Handle->pRxBuffer++;
				pUSART_Handle->RxLen--;
			} else {
				// Check Parity bit
				if (pUSART_Handle->USART_Config.USART_ParityControl == USART_PARITY_DIS) { // Receive 9bit data
					*pUSART_Handle->pRxBuffer = (pUSART_Handle->pUSARTx->DR & (uint8_t)0xFF);
				} else { // Receive 8bit data
					*pUSART_Handle->pRxBuffer = (pUSART_Handle->pUSARTx->DR & (uint8_t)0x7F);
				}
				pUSART_Handle->pRxBuffer++;
				pUSART_Handle->RxLen--;
			}
		} else if (!pUSART_Handle->RxLen) {
			pUSART_Handle->pUSARTx->CR1 &= ~(1 << USART_CR1_RXNEIE);
			pUSART_Handle->RxLen = 0;
			pUSART_Handle->RxState = USART_READY;
			USART_ApplicationEventCallback(pUSART_Handle, USART_EVENT_RX_CMPL);
		}
	}
}

/*
 *	Other Peripheral Funcs
 */
uint8_t USART_GetFlagStatus(USART_RegDef_t *pUSARTx, uint8_t FlagName) {
	uint8_t Status = FLAG_RESET;
	if (pUSARTx->SR & FlagName) {
		Status = FLAG_SET;
	}
	return Status;
}

void USART_ClearFlag(USART_RegDef_t *pUSARTx, uint8_t FlagName) {
	pUSARTx->SR &= ~(FlagName);
}

void USART_PeripheralControl(USART_RegDef_t *pUSARTx, uint8_t State) {
	if (State == ENABLE) {
		pUSARTx->CR1 |= (1 << USART_CR1_UE);
	} else {
		pUSARTx->CR1 &= ~(1 << USART_CR1_UE);
	}
}

void USART_SetBaudRate(USART_RegDef_t *pUSARTx, uint32_t BaudRate) {
	uint32_t PCLK, USART_DIV, M_Part, F_Part;
	uint32_t tempReg = 0;

	/*	Get PCLK	*/
	if (pUSARTx == USART1 || pUSARTx == USART6) {
		PCLK = RCC_GetPCLK2Value();
	} else {
		PCLK = RCC_GetPCLK1Value();
	}

	/*	Check Sample in OVER8 bit	*/
	if (pUSARTx->CR1 & (1 << USART_CR1_OVER8)) {
		USART_DIV = ((25 * PCLK) / (2 * BaudRate));
	} else {
		USART_DIV = ((25 * PCLK) / (4 * BaudRate));
	}

	/*	Mantissa part	*/
	M_Part = USART_DIV / 100;
	tempReg |= (M_Part << 4);

	/*	Fraction part	*/
	F_Part = (USART_DIV % 100);
	if (pUSARTx->CR1 & (1 << USART_CR1_OVER8)) {
		F_Part = (((F_Part * 8 + 50) / 100)) & ((uint8_t)0x07);
	} else {
		F_Part = (((F_Part * 16 + 50) / 100)) & ((uint8_t)0x0F);
	}

	tempReg |= F_Part;
	pUSARTx->BRR = tempReg;
}

/*
 * Application Callback
 * */
__attribute__((weak)) void USART_ApplicationEventCallback(USART_Handle_t *pUSART_Handle, uint8_t AppEv);

