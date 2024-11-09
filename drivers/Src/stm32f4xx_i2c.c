/*
 * stm32f4xx_i2c.c
 *
 *  Created on: Nov 8, 2024
 *      Author: uri-mary
 */

#include "stm32f4xx_i2c.h"
#include "stm32f4xx_rcc.h"

/*
 * 	Static funcs
 * */
static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx);
static void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx);
static void I2C_SendSlaveAddPhaseWrite(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddress);
static void I2C_SendSlaveAddPhaseRead(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddress);
static void I2C_ClearAddrFlag(I2C_Handle_t *pI2C_Handle);
static void I2C_MasterHandleTXEInterrupt(I2C_Handle_t *pI2C_Handle);
static void I2C_MasterHandleRXNEInterrupt(I2C_Handle_t *pI2C_Handle);

/*
 *	Peripheral Clock Setup
 */
void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t State) {
	if (State == ENABLE) {
		if (pI2Cx == I2C1) {
			I2C1_PCLK_EN();
		} else if (pI2Cx == I2C2) {
			I2C2_PCLK_EN();
		} else {
			I2C3_PCLK_EN();
		}
	} else {
		if (pI2Cx == I2C1) {
			I2C1_PCLK_DIS();
		} else if (pI2Cx == I2C2) {
			I2C2_PCLK_DIS();
		} else {
			I2C3_PCLK_DIS();
		}
	}

}

/*
 *	Init & Deinit Func
 */
void I2C_Init(I2C_Handle_t *pI2C_Handle) {
	uint32_t tempReg = 0;

	/*	Enable peripheral clock		*/
	I2C_PeriClockControl(pI2C_Handle->pI2Cx, ENABLE);

	/*	Configure ACK bit	*/
	tempReg |= (pI2C_Handle->I2C_Config.I2C_AckCtrl << I2C_CR1_ACK);
	pI2C_Handle->pI2Cx->CR1 = tempReg;

	/*	Configure FREQ in CR2	*/
	tempReg = 0;
	tempReg = RCC_GetPCLK1Value() / 1000000U;
	pI2C_Handle->pI2Cx->CR2 = (tempReg & (uint8_t)0x3F);

	/*	Configure OAR1 Reg	*/
	tempReg = 0;
	tempReg |= (pI2C_Handle->I2C_Config.I2C_DeviceAdd << 1);
	tempReg |= (1 << 14);
	pI2C_Handle->pI2Cx->OAR1 = tempReg;

	/*	Configure CCR Reg	*/
	tempReg = 0;
	uint16_t rcc_value = 0;
	if (pI2C_Handle->I2C_Config.I2C_SCLSpeed == I2C_SCL_SPEED_SM) { // In Standard mode
		rcc_value = (RCC_GetPCLK1Value() / (2 * pI2C_Handle->I2C_Config.I2C_SCLSpeed));
	} else { // In Fast mode
		tempReg |= (1 << I2C_CCR_FS);
		tempReg |= (pI2C_Handle->I2C_Config.I2C_FMDutyCycle << I2C_CCR_DUTY);
		if (pI2C_Handle->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_16_9) { // Fm 16-9
			rcc_value = (RCC_GetPCLK1Value() / (25 * pI2C_Handle->I2C_Config.I2C_SCLSpeed));
		} else { // Fm 2-1
			rcc_value = (RCC_GetPCLK1Value() / (3 * pI2C_Handle->I2C_Config.I2C_SCLSpeed));
		}
	}
	tempReg |= (rcc_value & (uint8_t)0xFFF);
	pI2C_Handle->pI2Cx->CCR = tempReg;

	/*	Configure TRISE Reg	*/
	tempReg = 0;
	if (pI2C_Handle->I2C_Config.I2C_SCLSpeed == I2C_SCL_SPEED_SM) { // Standard mode
		tempReg = (RCC_GetPCLK1Value() / 1000000U) + 1;
	} else {
		tempReg = ((RCC_GetPCLK1Value() * 300) / 1000000000U) + 1;
	}
	pI2C_Handle->pI2Cx->TRISE = tempReg & ((uint8_t)0x3F);
}

void I2C_DeInit(I2C_Handle_t *pI2C_Handle);

/*
 *	Master Send & Receive Data
 */
void I2C_MasterSendData(I2C_Handle_t *pI2C_Handle, uint8_t *pTxBuffer, uint8_t Len, uint8_t SlaveAddress, uint8_t SR) {
	// Start condition
	I2C_GenerateStartCondition(pI2C_Handle->pI2Cx);

	// Wait to SB Flag is set
	while (!I2C_GetFlagStatus(pI2C_Handle->pI2Cx, I2C_FLAG_SB));

	// Send Slave's Address & Read/Write bit
	I2C_SendSlaveAddPhaseWrite(pI2C_Handle->pI2Cx,SlaveAddress);

	// Wait ADDR FLag is set
	while (!I2C_GetFlagStatus(pI2C_Handle->pI2Cx, I2C_FLAG_ADDR));

	// Clear ADDR Flag, then SCK will pulled to LOW level
	I2C_ClearAddrFlag(pI2C_Handle);

	// Send data until Len equal 0
	while (Len > 0) {
		// Wait until TXE Flag is set
		while (!I2C_GetFlagStatus(pI2C_Handle->pI2Cx, I2C_FLAG_TXE));
		pI2C_Handle->pI2Cx->DR = *pTxBuffer;
		pTxBuffer++;
		Len--;
	}

	// When Len equal 0, wait until TXE & BTF Flag set, then Stop
	while (!I2C_GetFlagStatus(pI2C_Handle->pI2Cx, I2C_FLAG_TXE));
	while (!I2C_GetFlagStatus(pI2C_Handle->pI2Cx, I2C_FLAG_BTF));

	// When stop condition generated, BTF Flag will be clear
	if (SR == I2C_DISABLE_SR) {
		I2C_GenerateStopCondition(pI2C_Handle->pI2Cx);
	}
}

void I2C_MasterReceiveData(I2C_Handle_t *pI2C_Handle, uint8_t *pRxBuffer, uint8_t Len, uint8_t SlaveAddress, uint8_t SR) {
	// Start condition
	I2C_GenerateStartCondition(pI2C_Handle->pI2Cx);

	// Wait to SB Flag is set
	while (!I2C_GetFlagStatus(pI2C_Handle->pI2Cx, I2C_FLAG_SB));

	// Send Slave's Address & Read/Write bit
	I2C_SendSlaveAddPhaseRead(pI2C_Handle->pI2Cx,SlaveAddress);

	// Wait ADDR FLag is set
	while (!I2C_GetFlagStatus(pI2C_Handle->pI2Cx, I2C_FLAG_ADDR));

	// Clear ADDR Flag, then SCK will pulled to LOW level
	I2C_ClearAddrFlag(pI2C_Handle);

	// Receive 1 byte
	if (Len == 1) {
		// Disable ACK bit
		I2C_ManageAck(pI2C_Handle->pI2Cx, DISABLE);

		// Wait until RXNE Flag is set
		while (!I2C_GetFlagStatus(pI2C_Handle->pI2Cx, I2C_FLAG_RXNE));

		// Generate Stop condition
		if (SR == I2C_DISABLE_SR) {
			I2C_GenerateStopCondition(pI2C_Handle->pI2Cx);
		}

		// Load data into buffer
		*pRxBuffer = pI2C_Handle->pI2Cx->DR;
	} else if (Len > 1) {
		for (uint32_t i = Len; i > 0; --i) {
			// Wait until RXNE Flag is set
			while (!I2C_GetFlagStatus(pI2C_Handle->pI2Cx, I2C_FLAG_RXNE));
			if (i == 2) {
				// Disable ACK bit
				I2C_ManageAck(pI2C_Handle->pI2Cx, DISABLE);

				// Generate Stop condition
				if (SR == I2C_DISABLE_SR) {
					I2C_GenerateStopCondition(pI2C_Handle->pI2Cx);
				}
			}
			// Load data into buffer
			*pRxBuffer = pI2C_Handle->pI2Cx->DR;
			pRxBuffer++;
		}
	}

	if (pI2C_Handle->I2C_Config.I2C_AckCtrl == I2C_ACK_ENABLE) {
		I2C_ManageAck(pI2C_Handle->pI2Cx, ENABLE);
	}
}

uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2C_Handle, uint8_t *pTxBuffer, uint8_t Len, uint8_t SlaveAddress, uint8_t SR) {
	uint8_t state = pI2C_Handle->I2C_State;

	// Check state of I2C
	if ((state != I2C_BSY_TX) && (state != I2C_BSY_RX)) {
		// Set info into pI2C_Handle
		pI2C_Handle->pTxBuffer = pTxBuffer;
		pI2C_Handle->TxLen = Len;
		pI2C_Handle->SAddress = SlaveAddress;
		pI2C_Handle->I2C_State = I2C_BSY_TX;
		pI2C_Handle->SR = SR;

		// Generate Start Condition
		I2C_GenerateStartCondition(pI2C_Handle->pI2Cx);

		// Set ITEVFEN and ITBUFEN Flag
		pI2C_Handle->pI2Cx->CR2 |= (1 << I2C_CR2_ITEVTEN);
		pI2C_Handle->pI2Cx->CR2 |= (1 << I2C_CR2_ITBUFEN);
	}

	return state;
}
uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2C_Handle, uint8_t *pRxBuffer, uint8_t Len, uint8_t SlaveAddress, uint8_t SR) {
	uint8_t state = pI2C_Handle->I2C_State;

	// Check state of I2C
	if ((state != I2C_BSY_TX) && (state != I2C_BSY_RX)) {
		// Set info into pI2C_Handle
		pI2C_Handle->pRxBuffer = pRxBuffer;
		pI2C_Handle->RxLen = Len;
		pI2C_Handle->RxSize = Len;
		pI2C_Handle->SAddress = SlaveAddress;
		pI2C_Handle->I2C_State = I2C_BSY_RX;
		pI2C_Handle->SR = SR;

		// Generate Start Condition
		I2C_GenerateStartCondition(pI2C_Handle->pI2Cx);

		// Set ITEVFEN and ITBUFEN Flag
		pI2C_Handle->pI2Cx->CR2 |= (1 << I2C_CR2_ITEVTEN);
		pI2C_Handle->pI2Cx->CR2 |= (1 << I2C_CR2_ITBUFEN);
	}

	return state;
}

/*
 *	Slave Send & Receive Data
 */
void I2C_SlaveSendData(I2C_RegDef_t *pI2Cx, uint8_t Data);
uint8_t I2C_SlaveReceiveData(I2C_RegDef_t *pI2Cx);

/*
 *	Close send & receive Data
 */
void I2C_CloseSendData(I2C_Handle_t *pI2C_Handle) {
	// Clear ITEVFEN and ITBUFEN Flag
	pI2C_Handle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITEVTEN);
	pI2C_Handle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITBUFEN);

	// Clear info in pI2C_Handle
	pI2C_Handle->I2C_State = I2C_READY;
	pI2C_Handle->TxLen = 0;
	pI2C_Handle->pTxBuffer = NULL;
}

void I2C_CloseReceiveData(I2C_Handle_t *pI2C_Handle) {
	// Clear ITEVFEN and ITBUFEN Flag
	pI2C_Handle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITEVTEN);
	pI2C_Handle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITBUFEN);

	// Clear info in pI2C_Handle
	pI2C_Handle->I2C_State = I2C_READY;
	pI2C_Handle->RxLen = 0;
	pI2C_Handle->RxSize = 0;
	pI2C_Handle->pRxBuffer = NULL;
	if (pI2C_Handle->I2C_Config.I2C_AckCtrl == I2C_ACK_ENABLE) {
		I2C_ManageAck(pI2C_Handle->pI2Cx, ENABLE);
	}
}

/*
 *	IRQ Configuration & IRQ Handling
 */
void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t State) {
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
			*NVIC_ICER3 |= (1 << (IRQNumber % 32));
		}
	}
}

void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority) {
	uint8_t ipr = IRQNumber / 4;
	uint8_t irq = IRQNumber % 4;

	*(NVIC_IPR0 + ipr) |= (IRQPriority << (8 * irq + 4));
}

void I2C_Event_IRQHandling(I2C_Handle_t *pI2C_Handle) {
	uint32_t temp1, temp2, temp3;

	// Get ITEVFEN and ITBUFEN Flag
	temp1 = (pI2C_Handle->pI2Cx->CR2 & (1 << I2C_CR2_ITEVTEN));
	temp2 = (pI2C_Handle->pI2Cx->CR2 & (1 << I2C_CR2_ITBUFEN));

	// Start bit IT
	temp3 = (pI2C_Handle->pI2Cx->SR1 & (1 << I2C_SR1_SB));
	if (temp1 && temp3) {
		if (pI2C_Handle->I2C_State == I2C_BSY_TX) {
			I2C_SendSlaveAddPhaseWrite(pI2C_Handle->pI2Cx, pI2C_Handle->SAddress);
		} else if (pI2C_Handle->I2C_State == I2C_BSY_RX) {
			I2C_SendSlaveAddPhaseRead(pI2C_Handle->pI2Cx, pI2C_Handle->SAddress);
		}
	}

	// Address IT
	temp3 = (pI2C_Handle->pI2Cx->SR1 & (1 << I2C_SR1_ADDR));
	if (temp1 && temp3) {
		I2C_ClearAddrFlag(pI2C_Handle);
	}

	// 10 bit Header sent IT
	temp3 = (pI2C_Handle->pI2Cx->SR1 & (1 << I2C_SR1_ADD10));
	if (temp1 && temp3) {
		//TODO
	}

	// Stop received IT
	temp3 = (pI2C_Handle->pI2Cx->SR1 & (1 << I2C_SR1_STOPF));
	if (temp1 && temp3) {
		//TODO
		// Clear STOPF Flag
		pI2C_Handle->pI2Cx->CR1 |= 0x0000;

		// Call Application func
		I2C_ApplicationEventCallback(pI2C_Handle, I2C_EVENT_STOPF);
	}

	// Data byte transfer finished IT
	temp3 = (pI2C_Handle->pI2Cx->SR1 & (1 << I2C_SR1_BTF));
	if (temp1 && temp3) {
		//TODO
		if (pI2C_Handle->I2C_State == I2C_BSY_TX) {
			if (pI2C_Handle->pI2Cx->SR1 & (1 << I2C_SR1_TXE)) {	/*	Check TXE Flag	*/
				if (pI2C_Handle->TxLen == 0) {
					// Generate Stop Condition
					if (pI2C_Handle->SR == I2C_DISABLE_SR) {
						I2C_GenerateStopCondition(pI2C_Handle->pI2Cx);
					}

					// Close SendData func
					I2C_CloseSendData(pI2C_Handle);

					// Call Application func
					I2C_ApplicationEventCallback(pI2C_Handle, I2C_EVENT_TX_CMPLT);
				}
			}
		} else if (pI2C_Handle->I2C_State == I2C_BSY_RX) {
			//TODO
		}
	}

	// Transmit buffer empty IT
	temp3 = (pI2C_Handle->pI2Cx->SR1 & (1 << I2C_SR1_TXE));
	if (temp1 && temp2 && temp3) {
		// Check Device mode
		if (pI2C_Handle->pI2Cx->SR2 & (1 << I2C_SR2_MSL)) { /*	Master mode	*/
			if (pI2C_Handle->I2C_State == I2C_BSY_TX) {
				I2C_MasterHandleTXEInterrupt(pI2C_Handle);
			}
		} else { /*	Slave mode	*/
			if (pI2C_Handle->pI2Cx->SR2 & (1 << I2C_SR2_TRA)) {
				I2C_ApplicationEventCallback(pI2C_Handle, I2C_EVENT_DATA_REQ);
			}
		}
	}

	// Receive buffer not empty IT
	temp3 = (pI2C_Handle->pI2Cx->SR1 & (1 << I2C_SR1_RXNE));
	if (temp1 && temp2 && temp3) {
		// Check Device mode
		if (pI2C_Handle->pI2Cx->SR2 & (1 << I2C_SR2_MSL)) { /*	Master mode	*/
			if (pI2C_Handle->I2C_State == I2C_BSY_RX) {
				I2C_MasterHandleRXNEInterrupt(pI2C_Handle);
			}
		} else { /*	Slave mode	*/
			if (!(pI2C_Handle->pI2Cx->SR2 & (1 << I2C_SR2_TRA))) {
				I2C_ApplicationEventCallback(pI2C_Handle, I2C_EVENT_DATA_RCV);
			}
		}
	}
}

void I2C_Error_IRQHandling(I2C_Handle_t *pI2C_Handle) {
	uint32_t temp1, temp2;

	// Get Flag ITERREN
	temp2 = (pI2C_Handle->pI2Cx->CR2 & (1 << I2C_CR2_ITERREN));

	// Bus error IT
	temp1 = (pI2C_Handle->pI2Cx->SR1 & (1 << I2C_SR1_BERR));
	if (temp1 && temp2) {
		// Clear BERR Flag
		pI2C_Handle->pI2Cx->SR1 &= ~(1 << I2C_SR1_BERR);

		// Call Application func
		I2C_ApplicationEventCallback(pI2C_Handle, I2C_ERR_BERR);
	}

	// Arbitration loss (Master) IT
	temp1 = (pI2C_Handle->pI2Cx->SR1 & (1 << I2C_SR1_ARLO));
	if (temp1 && temp2) {
		// Clear ARLO Flag
		pI2C_Handle->pI2Cx->SR1 &= ~(1 << I2C_SR1_ARLO);

		// Call Application func
		I2C_ApplicationEventCallback(pI2C_Handle, I2C_ERR_ARLO);
	}

	// Acknowledge failure IT
	temp1 = (pI2C_Handle->pI2Cx->SR1 & (1 << I2C_SR1_AF));
	if (temp1 && temp2) {
		// Clear AF Flag
		pI2C_Handle->pI2Cx->SR1 &= ~(1 << I2C_SR1_AF);

		// Call Application func
		I2C_ApplicationEventCallback(pI2C_Handle, I2C_ERR_AF);
	}

	// Overrun/Underrun IT
	temp1 = (pI2C_Handle->pI2Cx->SR1 & (1 << I2C_SR1_OVR));
	if (temp1 && temp2) {
		// Clear OVR Flag
		pI2C_Handle->pI2Cx->SR1 &= ~(1 << I2C_SR1_OVR);

		// Call Application func
		I2C_ApplicationEventCallback(pI2C_Handle, I2C_ERR_OVR);
	}

	// PEC error IT
	temp1 = (pI2C_Handle->pI2Cx->SR1 & (1 << I2C_SR1_PECERR));
	if (temp1 && temp2) {
		// Clear PECERR Flag
		pI2C_Handle->pI2Cx->SR1 &= ~(1 << I2C_SR1_PECERR);

		// Call Application func
		I2C_ApplicationEventCallback(pI2C_Handle, I2C_ERR_PECERR);
	}

	// Timeout/Tlow error IT
	temp1 = (pI2C_Handle->pI2Cx->SR1 & (1 << I2C_SR1_TIMEOUT));
	if (temp1 && temp2) {
		// Clear TIMEOUT Flag
		pI2C_Handle->pI2Cx->SR1 &= ~(1 << I2C_SR1_TIMEOUT);

		// Call Application func
		I2C_ApplicationEventCallback(pI2C_Handle, I2C_ERR_TIMEOUT);
	}

	// SMBus Alert IT
	temp1 = (pI2C_Handle->pI2Cx->SR1 & (1 << I2C_SR1_SMBALERT));
	if (temp1 && temp2) {
		// Clear SMBALERT Flag
		pI2C_Handle->pI2Cx->SR1 &= ~(1 << I2C_SR1_SMBALERT);

		// Call Application func
		I2C_ApplicationEventCallback(pI2C_Handle, I2C_ALERT_SMBALERT);
	}
}

/*
 *	Other Peripheral Funcs
 */
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint8_t FlagName) {
	uint8_t status = FLAG_RESET;
	if (pI2Cx->SR1 & FlagName) {
		status = FLAG_SET;
	}
	return status;
}

void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t State) {
	if (State == ENABLE) {
		pI2Cx->CR1 |= (1 << I2C_CR1_PE);
	} else {
		pI2Cx->CR1 &= ~(1 << I2C_CR1_PE);
	}
}

void I2C_ManageAck(I2C_RegDef_t *pI2Cx, uint8_t State) {
	if (State == ENABLE) {
		pI2Cx->CR1 |= (1 << I2C_CR1_ACK);
	} else {
		pI2Cx->CR1 &= ~(1 << I2C_CR1_ACK);
	}
}

static void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx) {
	pI2Cx->CR1 |= (1 << I2C_CR1_STOP);
}

static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx) {
	pI2Cx->CR1 |= (1 << I2C_CR1_START);
}

static void I2C_SendSlaveAddPhaseWrite(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddress) {
	SlaveAddress = SlaveAddress << 1;
	SlaveAddress &= ~(1); // Write mode chosen
	pI2Cx->DR = SlaveAddress;
}

static void I2C_SendSlaveAddPhaseRead(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddress) {
	SlaveAddress = SlaveAddress << 1;
	SlaveAddress |= 1; // Read mode chosen
	pI2Cx->DR = SlaveAddress;
}

static void I2C_ClearAddrFlag(I2C_Handle_t *pI2C_Handle) {
	uint32_t dummy_read;

	// Check device mode
	if (pI2C_Handle->pI2Cx->SR2 & (1 << I2C_SR2_MSL)) { /*	Master mode	*/
		if (pI2C_Handle->I2C_State == I2C_BSY_RX) {
			if (pI2C_Handle->RxSize == 1) {
				// Disable ACK bit
				I2C_ManageAck(pI2C_Handle->pI2Cx, DISABLE);

				// Clear SR1 & SR2
				dummy_read = pI2C_Handle->pI2Cx->SR1;
				dummy_read = pI2C_Handle->pI2Cx->SR2;
				(void)dummy_read;
			}
		} else {
			// Clear SR1 & SR2
			dummy_read = pI2C_Handle->pI2Cx->SR1;
			dummy_read = pI2C_Handle->pI2Cx->SR2;
			(void)dummy_read;
		}
	} else { /*	Slave mode	*/
		// Clear SR1 & SR2
		dummy_read = pI2C_Handle->pI2Cx->SR1;
		dummy_read = pI2C_Handle->pI2Cx->SR2;
		(void)dummy_read;
	}
}

static void I2C_MasterHandleTXEInterrupt(I2C_Handle_t *pI2C_Handle) {
	if (pI2C_Handle->TxLen > 0) {
		// Load data into DR
		pI2C_Handle->pI2Cx->DR = *(pI2C_Handle->pTxBuffer);

		pI2C_Handle->TxLen--;
		pI2C_Handle->pTxBuffer++;
	}
}

static void I2C_MasterHandleRXNEInterrupt(I2C_Handle_t *pI2C_Handle) {
	if (pI2C_Handle->RxSize == 1) {
		*pI2C_Handle->pRxBuffer = pI2C_Handle->pI2Cx->DR;
		pI2C_Handle->RxLen--;
	}

	if (pI2C_Handle->RxSize > 1) {
		if (pI2C_Handle->RxLen == 2) {
			I2C_ManageAck(pI2C_Handle->pI2Cx, DISABLE);
		}
		*pI2C_Handle->pRxBuffer = pI2C_Handle->pI2Cx->DR;
		pI2C_Handle->RxLen--;
		pI2C_Handle->pRxBuffer++;
	}

	if (pI2C_Handle->RxLen == 0) {
		// Generate Stop condition
		if (pI2C_Handle->SR == I2C_DISABLE_SR) {
			I2C_GenerateStopCondition(pI2C_Handle->pI2Cx);
		}

		// Close receive data
		I2C_CloseReceiveData(pI2C_Handle);

		// Call Application func
		I2C_ApplicationEventCallback(pI2C_Handle, I2C_EVENT_RX_CMPLT);
	}
}

/*
 * Application Callback
 * */
__attribute__((weak)) void I2C_ApplicationEventCallback(I2C_Handle_t *pI2C_Handle, uint8_t AppEv);
void I2C_SlaveManageCallbackEvents(I2C_RegDef_t *pI2Cx, uint8_t State);


