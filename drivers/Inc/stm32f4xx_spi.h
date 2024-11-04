/*
 * stm32f4xx_spi.h
 *
 *  Created on: Oct 27, 2024
 *      Author: mary_uri
 */

#ifndef INC_STM32F4XX_SPI_H_
#define INC_STM32F4XX_SPI_H_

#include "stm32f4xx.h"

#define SPI_DEVICE_MODE_MASTER	1
#define SPI_DEVICE_MODE_SLAVE	0

#define SPI_BUS_CONFIG_FD				0
#define SPI_BUS_CONFIG_HD				1
#define SPI_BUS_CONFIG_SIMPLEX_RXONLY	2

#define SPI_SCLK_SPEED_D2	0
#define SPI_SCLK_SPEED_D4	1
#define SPI_SCLK_SPEED_D8	2
#define SPI_SCLK_SPEED_D16	3
#define SPI_SCLK_SPEED_D32	4
#define SPI_SCLK_SPEED_D64	5
#define SPI_SCLK_SPEED_D128	6
#define SPI_SCLK_SPEED_D256	7

#define SPI_DFF_8BIT	0
#define SPI_DFF_16BIT	1

#define SPI_CPOL_LOW	0
#define SPI_CPOL_HIGH	1

#define SPI_CPHA_LOW	0
#define SPI_CPHA_HIGH	1

#define SPI_SSM_DI	0
#define SPI_SSM_EN	1

#define	SPI_RXNE_FLAG	(1 << SPI_SR_RXNE)
#define SPI_TXE_FLAG	(1 << SPI_SR_TXE)
#define SPI_BSY_FLAG	(1 << SPI_SR_BSY)

#define SPI_READY	0
#define SPI_BSY_TX	1
#define SPI_BSY_RX	2

#define SPI_EVENT_TX_CMPLT	1
#define SPI_EVENT_RX_CMPLT	2
#define SPI_EVENT_OVR_CMPLT	3
#define SPI_EVENT_CRC_CMPLT	4

typedef struct {
	uint8_t SPI_DeviceMode;
	uint8_t SPI_BusConfig;
	uint8_t SPI_SclkSpeed;
	uint8_t SPI_DFF;
	uint8_t SPI_CPOL;
	uint8_t SPI_CPHA;
	uint8_t SPI_SSM;
} SPI_Config_t;

typedef struct {
	SPI_RegDef_t *pSPIx;
	SPI_Config_t SPI_Config;
	uint8_t *pTxBuffer;
	uint8_t *pRxBuffer;
	uint8_t TxLen;
	uint8_t RxLen;
	uint8_t TxState;
	uint8_t RxState;
} SPI_Handle_t;

void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t State);
void SPI_Init(SPI_Handle_t *pSPI_Handle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);

uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint8_t FlagName);

/*	Send and Receive Data	*/
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len);
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len);

uint8_t SPI_SendDataIT(SPI_Handle_t *pSPI_Handle, uint8_t *pTxBuffer, uint32_t Len);
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPI_Handle, uint8_t *pRxBuffer, uint32_t Len);


void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t State);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *pSPI_Handle);

void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t State);
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t State);
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t State);

void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx);
void SPI_CloseTransmition(SPI_Handle_t *pSPI_Handle);
void SPI_CloseReceivetion(SPI_Handle_t *pSPI_Handle);

/*	Application callback	*/
void SPI_ApplicationEventCallback(SPI_Handle_t *pSPI_Handle, uint8_t AppEv);


#endif /* INC_STM32F4XX_SPI_H_ */
