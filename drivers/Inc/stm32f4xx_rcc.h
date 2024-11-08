/*
 * stm32f4xx_rcc.h
 *
 *  Created on: Nov 6, 2024
 *      Author: uri-mary
 */

#ifndef INC_STM32F4XX_RCC_H_
#define INC_STM32F4XX_RCC_H_

#include "stm32f4xx.h"

/*	Get Clock Value of APB1	*/
uint32_t RCC_GetPCLK1Value(void);

/*	Get Clock Value of APB2	*/
uint32_t RCC_GetPCLK2Value(void);


#endif /* INC_STM32F4XX_RCC_H_ */
