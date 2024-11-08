/*
 * stm32f4xx_spi.c
 *
 *  Created on: Oct 27, 2024
 *      Author: mary_uri
 */

#include "stm32f4xx_spi.h"
#include "stddef.h"

static void spi_txe_interrupt_handle(SPI_Handle_t *pSPI_Handle);
static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPI_Handle);
static void spi_ovr_interrupt_handle(SPI_Handle_t *pSPI_Handle);

void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t State) {
	if (State == ENABLE) {	/*	Enable CLK for SPIx		*/
		if (pSPIx == SPI1) {
			SPI1_PCLK_EN();
		} else if (pSPIx == SPI2) {
			SPI2_PCLK_EN();
		} else if (pSPIx == SPI3) {
			SPI3_PCLK_EN();
		}
	} else if (State == DISABLE) {	/*	Disable CLK for SPIx	*/
		if (pSPIx == SPI1) {
			SPI1_PCLK_DIS();
		} else if (pSPIx == SPI2) {
			SPI2_PCLK_DIS();
		} else if (pSPIx == SPI3) {
			SPI3_PCLK_DIS();
		}
	}
}

void SPI_Init(SPI_Handle_t *pSPI_Handle) {
	/*	1 - Enable peripheral clock		*/
	SPI_PeriClockControl(pSPI_Handle->pSPIx, ENABLE);

	uint32_t temp = 0;

	/*	2 - Configure Device Mode		*/
	temp |= (pSPI_Handle->SPI_Config.SPI_DeviceMode << SPI_CR1_MSTR);

	/*	3 - Configure Bus		*/
	if (pSPI_Handle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_FD) {
		temp &= ~(1 << SPI_CR1_BIDIMODE);
	} else if (pSPI_Handle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_HD) {
		temp |= (1 << SPI_CR1_BIDIMODE);
	} else if (pSPI_Handle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY) {
		temp &= ~(1 << SPI_CR1_BIDIMODE);
		temp |= (1 << SPI_CR1_RXONLY);
	}

	/*	4 - Configure Baud rate	 	*/
	temp |= (pSPI_Handle->SPI_Config.SPI_SclkSpeed << SPI_CR1_BR);

	/*	5 - Configure the DFF		*/
	temp |= (pSPI_Handle->SPI_Config.SPI_DFF << SPI_CR1_DFF);

	/*	6 - Configure the CPOL		*/
	temp |= (pSPI_Handle->SPI_Config.SPI_CPOL << SPI_CR1_CPOL);

	/*	7 - Configure the CPHA		*/
	temp |= (pSPI_Handle->SPI_Config.SPI_CPHA << SPI_CR1_CPHA);

	/*	7 - Configure the SSM		*/
	temp |= (pSPI_Handle->SPI_Config.SPI_SSM << SPI_CR1_SSM);

	pSPI_Handle->pSPIx->CR1 = temp;
}

void SPI_DeInit(SPI_RegDef_t *pSPIx) {
	//TODO
}

uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint8_t FlagName) {
	uint8_t status = RESET;
	if (pSPIx->SR & FlagName) {
		status = FLAG_SET;
	}
	return status;
}

/*	Send and Receive Data	*/
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len) {
	while (Len > 0) {
		/*	Waiting TXE Flag set	*/
		while (SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG) == (uint8_t)(FLAG_RESET));

		/*	Check DFF bit in CR1 Reg	*/
		if (pSPIx->CR1 & (1 << SPI_CR1_DFF)) {	/*	16 bit	*/
			/*	Load data to DR		*/
			pSPIx->DR = *((uint16_t *)pTxBuffer);
			Len--;
			Len--;
			(uint16_t *)pTxBuffer++;
		} else {	/*	8 bit	*/
			pSPIx->DR = *pTxBuffer;
			Len--;
			pTxBuffer++;
		}
	}
}

void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len) {
	while (Len > 0) {
		/*	Waiting RXNE Flag set	*/
		while (SPI_GetFlagStatus(pSPIx, SPI_RXNE_FLAG) == (uint8_t)(FLAG_RESET));

		/*	Check DFF bit in CR1 Reg	*/
		if (pSPIx->CR1 & (1 << SPI_CR1_DFF)) {	/*	16 bit	*/
			/*	Load data from DR		*/
			*((uint16_t *)pRxBuffer) = pSPIx->DR;
			Len--;
			Len--;
			(uint16_t *)pRxBuffer++;
		} else {	/*	8 bit	*/
			*(pRxBuffer) = pSPIx->DR;
			Len--;
			pRxBuffer++;
		}
	}
}

uint8_t SPI_SendDataIT(SPI_Handle_t *pSPI_Handle, uint8_t *pTxBuffer, uint32_t Len) {
	uint8_t state = pSPI_Handle->TxState;
	if (state != SPI_BSY_TX) {
		/*	1 - Save address of pTxBuffer into SPI_Handle	*/
		pSPI_Handle->pTxBuffer = pTxBuffer;
		pSPI_Handle->TxLen = Len;

		/*	2 - Mark SPI is busy	*/
		pSPI_Handle->TxState = SPI_BSY_TX;

		/*	3 - Enable TXEIE bit	*/
		pSPI_Handle->pSPIx->CR2 |= (1 << SPI_CR2_TXEIE);

		/*	Transmit data will be handle in interrupt	*/
	}

	return state;
}

uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPI_Handle, uint8_t *pRxBuffer, uint32_t Len) {
	uint8_t state = pSPI_Handle->RxState;
	if (state != SPI_BSY_RX) {
		/*	1 - Save address of pRxBuffer into SPI_Handle	*/
		pSPI_Handle->pRxBuffer = pRxBuffer;
		pSPI_Handle->RxLen = Len;

		/*	2 - Mark SPI is busy	*/
		pSPI_Handle->RxState = SPI_BSY_RX;

		/*	3 - Enable RXNEIE bit	*/
		pSPI_Handle->pSPIx->CR2 |= (1 << SPI_CR2_RXNEIE);

		/*	Receive data will be handle in interrupt	*/
	}

	return state;
}

/*	Interrupt	*/
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t State) {
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

void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority){
	uint8_t ipr = IRQNumber / 4;
	uint8_t irq = IRQNumber % 4;

	*(NVIC_IPR0 + ipr) |= (IRQPriority << (8 * irq + 4));
}

void SPI_IRQHandling(SPI_Handle_t *pSPI_Handle) {
	uint8_t temp1, temp2;

	//TXE
	temp1 = pSPI_Handle->pSPIx->SR & (1 << SPI_SR_TXE);
	temp2 = pSPI_Handle->pSPIx->CR2 & (1 << SPI_CR2_TXEIE);
	if (temp1 && temp2) {
		/*	Handle TXE	*/
		spi_txe_interrupt_handle(pSPI_Handle);
	}

	//RXNE
	temp1 = pSPI_Handle->pSPIx->SR & (1 << SPI_SR_RXNE);
	temp2 = pSPI_Handle->pSPIx->CR2 & (1 << SPI_CR2_RXNEIE);
	if (temp1 && temp2) {
		/*	Handle RXNE	*/
		spi_rxne_interrupt_handle(pSPI_Handle);
	}

	//OVR
	temp1 = pSPI_Handle->pSPIx->SR & (1 << SPI_SR_OVR);
	temp2 = pSPI_Handle->pSPIx->CR2 & (1 << SPI_CR2_ERRIE);
	if (temp1 && temp2) {
		/*	Handle OVR	*/
		spi_ovr_interrupt_handle(pSPI_Handle);
	}
}

static void spi_txe_interrupt_handle(SPI_Handle_t *pSPI_Handle) {
	/*	Check DFF bit in CR1 Reg	*/
	if (pSPI_Handle->pSPIx->CR1 & (1 << SPI_CR1_DFF)) {	/*	16 bit	*/
		/*	Load data to DR		*/
		pSPI_Handle->pSPIx->DR = *((uint16_t *)pSPI_Handle->pTxBuffer);
		pSPI_Handle->TxLen--;
		pSPI_Handle->TxLen--;
		(uint16_t *)pSPI_Handle->pTxBuffer++;
	} else {	/*	8 bit	*/
		pSPI_Handle->pSPIx->DR = *pSPI_Handle->pTxBuffer;
		pSPI_Handle->TxLen--;
		pSPI_Handle->pTxBuffer++;
	}

	if (!pSPI_Handle->TxLen) {
		SPI_CloseTransmition(pSPI_Handle);
		SPI_ApplicationEventCallback(pSPI_Handle, SPI_EVENT_TX_CMPLT);
	}
}

void SPI_CloseTransmition(SPI_Handle_t *pSPI_Handle) {
	pSPI_Handle->pSPIx->CR2 &= ~(1 << SPI_CR2_TXEIE);
	pSPI_Handle->TxState = SPI_READY;
	pSPI_Handle->pTxBuffer = NULL;
	pSPI_Handle->TxLen = 0;
}

static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPI_Handle) {
	/*	Check DFF bit in CR1 Reg	*/
	if (pSPI_Handle->pSPIx->CR1 & (1 << SPI_CR1_DFF)) {	/*	16 bit	*/
		/*	Load data from DR		*/
		*((uint16_t *)pSPI_Handle->pRxBuffer) = (uint16_t)pSPI_Handle->pSPIx->DR;
		pSPI_Handle->RxLen--;
		pSPI_Handle->RxLen--;
		(uint16_t *)pSPI_Handle->pRxBuffer++;
	} else {	/*	8 bit	*/
		*(pSPI_Handle->pRxBuffer) = (uint8_t)pSPI_Handle->pSPIx->DR;
		pSPI_Handle->RxLen--;
		pSPI_Handle->pRxBuffer++;
	}

	if (!(pSPI_Handle->RxLen)) {
		SPI_CloseReceivetion(pSPI_Handle);
		SPI_ApplicationEventCallback(pSPI_Handle, SPI_EVENT_RX_CMPLT);
	}
}

void SPI_CloseReceivetion(SPI_Handle_t *pSPI_Handle) {
	pSPI_Handle->pSPIx->CR2 &= ~(1 << SPI_CR2_RXNEIE);
	pSPI_Handle->RxLen = 0;
	pSPI_Handle->pRxBuffer = NULL;
	pSPI_Handle->RxState = SPI_READY;
}

static void spi_ovr_interrupt_handle(SPI_Handle_t *pSPI_Handle) {
	// Clear OVR Flag
	if (pSPI_Handle->TxState != SPI_BSY_TX) {
		SPI_ClearOVRFlag(pSPI_Handle->pSPIx);
	}

	//Inform Application
	SPI_ApplicationEventCallback(pSPI_Handle, SPI_EVENT_OVR_CMPLT);
}

void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx) {
	uint8_t temp;
	temp = pSPIx->DR;
	temp = pSPIx->SR;
	(void)temp;
}

__attribute__((weak)) void SPI_ApplicationEventCallback(SPI_Handle_t *pSPI_Handle, uint8_t AppEv);

/*		*/
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t State) {
	if (State == ENABLE) {
		pSPIx->CR1 |= (1 << SPI_CR1_SPE);
	} else {
		pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);
	}
}

void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t State) {
	if (State == ENABLE) {
		pSPIx->CR1 |= (1 << SPI_CR1_SSI);
	} else {
		pSPIx->CR1 &= ~(1 << SPI_CR1_SSI);
	}
}

void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t State) {
	if (State == ENABLE) {
		pSPIx->CR2 |= (1 << SPI_CR2_SSOE);
	} else {
		pSPIx->CR2 &= ~(1 << SPI_CR2_SSOE);
	}
}

