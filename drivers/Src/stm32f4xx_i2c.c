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

uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2C_Handle, uint8_t *pTxBuffer, uint8_t Len, uint8_t SlaveAddress, uint8_t SR);
uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2C_Hanlde, uint8_t *pRxBuffer, uint8_t Len, uint8_t SlaveAddress, uint8_t SR);

/*
 *	Slave Send & Receive Data
 */
void I2C_SlaveSendData(I2C_RegDef_t *pI2Cx, uint8_t Data);
uint8_t I2C_SlaveReceiveData(I2C_RegDef_t *pI2Cx);

/*
 *	Close send & receive Data
 */
void I2C_CloseSendData(I2C_Handle_t *pI2C_Handle);
void I2C_CloseReceiveData(I2C_Handle_t *pI2C_Handle);

/*
 *	IRQ Configuration & IRQ Handling
 */
void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t State);
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void I2C_Event_IRQHandling(I2C_Handle_t *pI2C_Handle);
void I2C_Error_IRQHandling(I2C_Handle_t *pI2C_Handle);

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
	SlaveAddress = SlaveAddress < 1;
	SlaveAddress &= ~(1); // Write mode chosen
	pI2Cx->DR = SlaveAddress;
}

static void I2C_SendSlaveAddPhaseRead(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddress) {
	SlaveAddress = SlaveAddress < 1;
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

/*
 * Application Callback
 * */
void I2C_ApplicationEventCallback(I2C_Handle_t *pI2C_Handle, uint8_t AppEv);
void I2C_SlaveManageCallbackEvents(I2C_RegDef_t *pI2Cx, uint8_t State);


