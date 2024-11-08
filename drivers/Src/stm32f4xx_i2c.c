/*
 * stm32f4xx_i2c.c
 *
 *  Created on: Nov 8, 2024
 *      Author: uri-mary
 */

#include "stm32f4xx_i2c.h"
#include "stm32f4xx_rcc.h"


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
void I2C_MasterSendData(I2C_Handle_t *pI2C_Handle, uint8_t *pTxBuffer, uint8_t Len);
void I2C_MasterReceiveData(I2C_Handle_t *pI2C_Handle, uint8_t *pRxBuffer, uint8_t Len);
uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2C_Handle, uint8_t *pTxBuffer, uint8_t Len);
uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2C_Hanlde, uint8_t *pRxBuffer, uint8_t Len);

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
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint8_t FlagName);
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t State) {
	if (State == ENABLE) {
		pI2Cx->CR1 |= (1 << I2C_CR1_PE);
	} else {
		pI2Cx->CR1 &= ~(1 << I2C_CR1_PE);
	}
}

void I2C_ManageAck(I2C_RegDef_t *pI2Cx, uint8_t State);
void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx);

/*
 * Application Callback
 * */
void I2C_ApplicationEventCallback(I2C_Handle_t *pI2C_Handle, uint8_t AppEv);
void I2C_SlaveManageCallbackEvents(I2C_RegDef_t *pI2Cx, uint8_t State);


