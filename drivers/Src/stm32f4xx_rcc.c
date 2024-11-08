/*
 * stm32f4xx_rcc.c
 *
 *  Created on: Nov 6, 2024
 *      Author: uri-mary
 */

#include "stm32f4xx_rcc.h"

uint16_t AHB_PreScaler[8 ] = { 2, 4, 8, 16, 64, 128, 256, 512 };
uint16_t APB_PreScaler[4] = { 2, 4, 8, 16 };

/*	Get Clock Value of APB1	*/
uint32_t RCC_GetPCLK1Value(void) {
	uint32_t SYS_CLK, PCLK1;
	uint8_t sys_clk_type, AHB_pre_scaler, APB1_pre_scaler, temp;

	sys_clk_type = ((RCC->CFGR >> 2) & 0x03);

	/*	Find SYS_Clock	*/
	if (sys_clk_type == 0) {
		SYS_CLK = 16000000;
	} else if (sys_clk_type == 1) {
		SYS_CLK = 8000000;
	}

	/*	Find pre-scaler of AHB	*/
	temp = ((RCC->CFGR >> 4) & 0xF);
	if (temp < 8) {
		AHB_pre_scaler = 1;
	} else {
		AHB_pre_scaler = AHB_PreScaler[temp - 8];
	}

	/*	Find pre-scaler of APB1	*/
	temp = ((RCC->CFGR >> 10) & 0x07);
	if (temp < 4) {
		APB1_pre_scaler = 1;
	} else {
		APB1_pre_scaler = APB_PreScaler[temp - 4];
	}

	/*	Find PCLK1	*/
	PCLK1 = ((SYS_CLK / AHB_pre_scaler) / APB1_pre_scaler);

	return PCLK1;
}

/*	Get Clock Value of APB2	*/
uint32_t RCC_GetPCLK2Value(void) {
	uint32_t SYS_CLK, PCLK2;
	uint8_t sys_clk_type, AHB_pre_scaler, APB2_pre_scaler, temp;

	sys_clk_type = ((RCC->CFGR >> 2) & 0x03);

	/*	Find SYS_Clock	*/
	if (sys_clk_type == 0) {
		SYS_CLK = 16000000;
	} else if (sys_clk_type == 1) {
		SYS_CLK = 8000000;
	}

	/*	Find pre-scaler of AHB	*/
	temp = ((RCC->CFGR >> 4) & 0xF);
	if (temp < 8) {
		AHB_pre_scaler = 1;
	} else {
		AHB_pre_scaler = AHB_PreScaler[temp - 8];
	}

	/*	Find pre-scaler of APB1	*/
	temp = ((RCC->CFGR >> 13) & 0x07);
	if (temp < 4) {
		APB2_pre_scaler = 1;
	} else {
		APB2_pre_scaler = APB_PreScaler[temp - 4];
	}

	/*	Find PCLK1	*/
	PCLK2 = ((SYS_CLK / AHB_pre_scaler) / APB2_pre_scaler);

	return PCLK2;
}
