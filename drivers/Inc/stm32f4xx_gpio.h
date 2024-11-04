/*
 * stm32f4xx_gpio.h
 *
 *  Created on: Oct 15, 2024
 *      Author: mary_uri
 */

#ifndef INC_STM32F4XX_GPIO_H_
#define INC_STM32F4XX_GPIO_H_

#include "stm32f4xx.h"

//Define PIN number
#define GPIO_PIN_0	0
#define GPIO_PIN_1	1
#define GPIO_PIN_2	2
#define GPIO_PIN_3	3
#define GPIO_PIN_4	4
#define GPIO_PIN_5	5
#define GPIO_PIN_6	6
#define GPIO_PIN_7	7
#define GPIO_PIN_8	8
#define GPIO_PIN_9	9
#define GPIO_PIN_10	10
#define GPIO_PIN_11	11
#define GPIO_PIN_12	12
#define GPIO_PIN_13	13
#define GPIO_PIN_14	14
#define GPIO_PIN_15	15

//Define MODE
#define GPIO_MODE_INPUT		0
#define GPIO_MODE_OUTPUT	1
#define GPIO_MODE_ALT_FUNC	2
#define GPIO_MODE_ANALOG	3
#define GPIO_MODE_IT_FT		4	// Falling
#define GPIO_MODE_IT_RT		5	// Rising
#define GPIO_MODE_IT_RFT	6	// All

//Define OType
#define GPIO_OTY_PUSH_PULL	0
#define GPIO_OTY_OPEN_DRAIN	1

//Define Speed
#define GPIO_SPE_LOW	0		/* Low speed 		*/
#define GPIO_SPE_MED	1		/* Medium speed 	*/
#define GPIO_SPE_HIGH	2		/* High speed 		*/
#define GPIO_SPE_VHIGH	3		/* Very high speed 	*/

//Define Pull-up Pull-down
#define GPIO_NO_PULL_UP_PULL_DOWN	0
#define	GPIO_PIN_PULL_UP			1
#define GPIO_PIN_PULL_DOWN			2

//Define ALT Function
#define AF0		0
#define AF1		1
#define AF2		2
#define AF3		3
#define AF4		4
#define AF5		5
#define AF6		6
#define AF7		7
#define AF8		8
#define AF9		9
#define AF10	10
#define AF11    11
#define AF12    12
#define AF13	13
#define AF14	14
#define AF15	15

typedef struct {
	uint8_t GPIO_PinNumber;
	uint8_t GPIO_PinMode;
	uint8_t GPIO_PinSpeed;
	uint8_t GPIO_PinPUPD;
	uint8_t GPIO_PinOPType;
	uint8_t GPIO_PinAltFuncMode;
} GPIO_Pin_Config_t;

typedef struct {
	GPIO_RegDef_t *pGPIOx;
	GPIO_Pin_Config_t pGPIO_PinConfig;
} GPIO_Handle_t;


void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t State);
void GPIO_Init(GPIO_Handle_t *pGPIO_Handle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);
uint8_t GPIO_ReadFromPin(GPIO_RegDef_t *pGPIOx, uint8_t GPIO_PinNumber);
uint16_t GPIO_ReadFromPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToPin(GPIO_RegDef_t *pGPIOx, uint8_t GPIO_PinNumber, uint8_t Value);
void GPIO_WriteToPort(GPIO_RegDef_t *pGPIOx, uint16_t Value);
void GPIO_TogglePin(GPIO_RegDef_t *pGPIOx, uint8_t GPIO_PinNumber);

void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t State);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void GPIO_IRQHandling(uint8_t GPIO_PinNumber);


#endif /* INC_STM32F4XX_GPIO_H_ */
