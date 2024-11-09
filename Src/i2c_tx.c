/*
 * i2c_tx.c
 *
 *  Created on: Nov 9, 2024
 *      Author: uri-mary
 */

#include "stdio.h"
#include "string.h"
#include "stm32f4xx_i2c.h"
#include "stm32f4xx_gpio.h"

// Global variables
#define DEVICE_ADDR		0x60
#define SLAVE_ADDR		0x68
I2C_Handle_t I2C_Handle;
char msg[] = "hello";


/*
 *	ALT Func 4
 * 	I2C1 Pins
 * 	PB6 -> SCL
 * 	PB7 -> SDA
 *
 * */


/*
 * Init func
 * */
void Initializer() {
	/*
	 *	GPIO Init
	 * */
	GPIO_Handle_t GPIO_Handle;
	GPIO_Handle.pGPIOx = GPIOB;
	GPIO_Handle.pGPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALT_FUNC;
	GPIO_Handle.pGPIO_PinConfig.GPIO_PinAltFuncMode = AF4;
	GPIO_Handle.pGPIO_PinConfig.GPIO_PinOPType = GPIO_OTY_OPEN_DRAIN;
	GPIO_Handle.pGPIO_PinConfig.GPIO_PinPUPD = GPIO_PIN_PULL_UP;
	GPIO_Handle.pGPIO_PinConfig.GPIO_PinSpeed = GPIO_SPE_VHIGH;

	// Pin SCL
	GPIO_Handle.pGPIO_PinConfig.GPIO_PinNumber = 6;
	GPIO_Init(&GPIO_Handle);

	// Pin SDA
	GPIO_Handle.pGPIO_PinConfig.GPIO_PinNumber = 7;
	GPIO_Init(&GPIO_Handle);

	/*
	 * 	I2C Init
	 * */
	I2C_Handle.pI2Cx = I2C1;
	I2C_Handle.I2C_Config.I2C_AckCtrl = I2C_ACK_ENABLE;
	I2C_Handle.I2C_Config.I2C_DeviceAdd = DEVICE_ADDR;
	I2C_Handle.I2C_Config.I2C_FMDutyCycle = I2C_FM_DUTY_2;
	I2C_Handle.I2C_Config.I2C_SCLSpeed = I2C_SCL_SPEED_SM;

	I2C_Init(&I2C_Handle);
}


int main() {
	Initializer();
	I2C_PeripheralControl(I2C_Handle.pI2Cx, ENABLE);

	while (1) {
		I2C_MasterSendData(&I2C_Handle, (uint8_t *)msg, strlen(msg), SLAVE_ADDR, 0);
		while (1);
	}

	return 0;
}

