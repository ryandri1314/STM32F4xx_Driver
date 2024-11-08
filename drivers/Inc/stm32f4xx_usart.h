/*
 * stm32f4xx_usart.h
 *
 *  Created on: Nov 5, 2024
 *      Author: uri-mary
 */

#ifndef INC_STM32F4XX_USART_H_
#define INC_STM32F4XX_USART_H_

#include "stm32f4xx.h"

/*	Define USART's MODE	*/
#define USART_MODE_RX	0
#define USART_MODE_TX	1
#define USART_MODE_TXRX	2

/*	Define USART's Baud rate	*/
#define USART_BAUD_RATE_1200			1200
#define USART_BAUD_RATE_2400			2400
#define USART_BAUD_RATE_9600			9600
#define USART_BAUD_RATE_19200			19200
#define USART_BAUD_RATE_38400			38400
#define USART_BAUD_RATE_57600			57600
#define USART_BAUD_RATE_115200			115200
#define USART_BAUD_RATE_230400			230400
#define USART_BAUD_RATE_460800			460800
#define USART_BAUD_RATE_921600			921600
#define USART_BAUD_RATE_2M				2000000
#define USART_BAUD_RATE_3M				3000000

/*	Define USART's Parity	*/
#define USART_PARITY_EVEN	0
#define USART_PARITY_ODD	1
#define USART_PARITY_DIS	2

/*	Define USART's Data length	*/
#define USART_WORD_LEN_8BITS	0
#define USART_WORD_LEN_9BITS	1

/*	Define USART's Stop bit	*/
#define USART_STOP_1_BIT		0
#define USART_STOP_0_5_BIT		1
#define USART_STOP_2_BIT		2
#define USART_STOP_1_5_BIT		3

/*	Define USART's HW control	*/
#define USART_HW_CTRL_NONE	0
#define USART_HW_CTRL_CTS	1
#define USART_HW_CTRL_RTS	2
#define USART_HW_CTRL_BOTH	3

/*	Define USART's Flag	*/
#define USART_FLAG_TXE	(1 << USART_SR_TXE)
#define USART_FLAG_RXNE	(1 << USART_SR_RXNE)
#define USART_FLAG_TC	(1 << USART_SR_TC)

/*	Define USART's State	*/
#define USART_READY		0
#define USART_BSY_TX	1
#define USART_BSY_RX	2

/*	Define USART's Event	*/
#define USART_EVENT_TX_CMPL		0
#define USART_EVENT_RX_CMPL		1
#define USART_EVENT_CTS			2
#define USART_EVENT_IDLE		3
#define USART_EVENT_PE			4
#define USART_EVENT_FE			5
#define USART_EVENT_NF			6
#define USART_EVENT_LBD			7
#define USART_EVENT_ORE			8

typedef struct {
	uint8_t	USART_Mode;
	uint32_t USART_BaudRate;
	uint8_t	USART_StopBits;
	uint8_t	USART_WordLength;
	uint8_t	USART_ParityControl;
	uint8_t USART_HWFlowControl;
} USART_Config_t;

typedef struct {
	USART_RegDef_t *pUSARTx;
	USART_Config_t USART_Config;
	uint8_t *pTxBuffer;
	uint8_t *pRxBuffer;
	uint8_t TxLen;
	uint8_t RxLen;
	uint8_t TxState;
	uint8_t RxState;
} USART_Handle_t;

/*
 *	Peripheral Clock Setup
 */
void USART_PeriClockControl(USART_RegDef_t *pUSARTx, uint8_t State);

/*
 *	Init & Deinit Func
 */
void USART_Init(USART_Handle_t *pUSART_Handle);
void USART_DeInit(USART_Handle_t *pUSART_Handle);

/*
 *	Send & Receive Data
 */
void USART_SendData(USART_Handle_t *pUSART_Handle, uint8_t *pTxBuffer, uint8_t Len);
void USART_ReceiveData(USART_Handle_t *pUSART_Handle, uint8_t *pRxBuffer, uint8_t Len);
uint8_t USART_SendDataIT(USART_Handle_t *pUSART_Handle, uint8_t *pTxBuffer, uint8_t Len);
uint8_t USART_ReceiveDataIT(USART_Handle_t *pUSART_Hanlde, uint8_t *pRxBuffer, uint8_t Len);

/*
 *	IRQ Configuration & IRQ Handling
 */
void USART_IRQInterruptConfig(uint8_t IRQNumber, uint8_t State);
void USART_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void USART_IRQHandling(USART_Handle_t *pUSART_Handle);

/*
 *	Other Peripheral Funcs
 */
uint8_t USART_GetFlagStatus(USART_RegDef_t *pUSARTx, uint8_t FlagName);
void USART_ClearFlag(USART_RegDef_t *pUSARTx, uint8_t FlagName);
void USART_PeripheralControl(USART_RegDef_t *pUSARTx, uint8_t State);
void USART_SetBaudRate(USART_RegDef_t *pUSARTx, uint32_t BaudRate);

/*
 * Application Callback
 * */
void USART_ApplicationEventCallback(USART_Handle_t *pUSART_Handle, uint8_t AppEv);

#endif /* INC_STM32F4XX_USART_H_ */
