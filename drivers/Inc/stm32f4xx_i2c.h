/*
 * stm32f4xx_i2c.h
 *
 *  Created on: Nov 8, 2024
 *      Author: uri-mary
 */

#ifndef INC_STM32F4XX_I2C_H_
#define INC_STM32F4XX_I2C_H_


#include "stm32f4xx.h"

typedef struct {
	uint32_t I2C_SCLSpeed;
	uint8_t	I2C_DeviceAdd;
	uint8_t I2C_AckCtrl;
	uint8_t I2C_FMDutyCycle;
} I2C_Config_t;

typedef struct {
	I2C_Config_t I2C_Config;
	I2C_RegDef_t *pI2Cx;
	uint8_t TxLen;
	uint8_t RxLen;
	uint8_t *pTxBuffer;
	uint8_t *pRxBuffer;
	uint8_t I2C_State;
	uint8_t SAddress;
	uint8_t RxSize;
	uint8_t SR;	// repeat start value
} I2C_Handle_t;

/*	Define I2C's state	*/
#define I2C_READY	0
#define I2C_BSY_TX	1
#define I2C_BSY_RX	2

/*	Define I2C's Speed	*/
#define I2C_SCL_SPEED_SM	100000
#define I2C_SCL_SPEED_FM	400000

/*	Define I2C's ACK Ctrl	*/
#define I2C_ACK_DISABLE	1
#define I2C_ACK_ENABLE	1

/*	Define I2C's FM Duty cycle	*/
#define I2C_FM_DUTY_2		0
#define I2C_FM_DUTY_16_9	1

/*	Define I2C's flags	*/
#define I2C_FLAG_SB			(1 << I2C_SR1_SB)
#define I2C_FLAG_ADDR		(1 << I2C_SR1_ADDR)
#define I2C_FLAG_BTF		(1 << I2C_SR1_BTF)
#define I2C_FLAG_ADD10		(1 << I2C_SR1_ADD10)
#define I2C_FLAG_STOPF		(1 << I2C_SR1_STOPF)
#define I2C_FLAG_RXNE		(1 << I2C_SR1_RXNE)
#define I2C_FLAG_TXE		(1 << I2C_SR1_TXE)
#define I2C_FLAG_BERR		(1 << I2C_SR1_BERR)
#define I2C_FLAG_ARLO		(1 << I2C_SR1_ARLO)
#define I2C_FLAG_AF			(1 << I2C_SR1_AF)
#define I2C_FLAG_PECERR		(1 << I2C_SR1_PECERR)
#define I2C_FLAG_TIMEOUT	(1 << I2C_SR1_TIMEOUT)
#define I2C_FLAG_SMBALERT	(1 << I2C_SR1_SMBALERT)

#define I2C_DISABLE_SR	RESET
#define I2C_ENABLE_SR	SET

/*	Define I2C's application events	*/
#define I2C_EVENT_TX_CMPLT	0
#define I2C_EVENT_RX_CMPLT	1
#define I2C_EVENT_STOPF		2
#define I2C_ERR_BERR		3
#define I2C_ERR_PECERR		4
#define I2C_ERR_ARLO		5
#define I2C_ERR_AF			6
#define I2C_ERR_TIMEOUT		7
#define I2C_ERR_OVR			8
#define I2C_EVENT_ADD10		9
#define I2C_EVENT_ADDR		10
#define I2C_EVENT_BTF		11
#define I2C_ALERT_SMBALERT	12
#define I2C_EVENT_DATA_REQ	13
#define I2C_EVENT_DATA_RCV	14


/*
 *	Peripheral Clock Setup
 */
void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t State);

/*
 *	Init & Deinit Func
 */
void I2C_Init(I2C_Handle_t *pI2C_Handle);
void I2C_DeInit(I2C_Handle_t *pI2C_Handle);

/*
 *	Master Send & Receive Data
 */
void I2C_MasterSendData(I2C_Handle_t *pI2C_Handle, uint8_t *pTxBuffer, uint8_t Len, uint8_t SlaveAddress, uint8_t SR);
void I2C_MasterReceiveData(I2C_Handle_t *pI2C_Handle, uint8_t *pRxBuffer, uint8_t Len, uint8_t SlaveAddress, uint8_t SR);
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
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint8_t FlagName);
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t State);
void I2C_ManageAck(I2C_RegDef_t *pI2Cx, uint8_t State);

/*
 * Application Callback
 * */
void I2C_ApplicationEventCallback(I2C_Handle_t *pI2C_Handle, uint8_t AppEv);
void I2C_SlaveManageCallbackEvents(I2C_RegDef_t *pI2Cx, uint8_t State);


#endif /* INC_STM32F4XX_I2C_H_ */
