/*
 * smt32f4xx.h
 *
 *  Created on: Oct 15, 2024
 *      Author: mary_uri
 */

#ifndef DRIVERS_INC_SMT32F4XX_H_
#define DRIVERS_INC_SMT32F4XX_H_

#include <stdint.h>

#define __vol	volatile

/*	Define NVIC ISERx Core	*/
#define NVIC_ISER0	((__vol	uint32_t *)0xE000E100)
#define NVIC_ISER1	((__vol	uint32_t *)0xE000E104)
#define NVIC_ISER2	((__vol	uint32_t *)0xE000E108)
#define NVIC_ISER3	((__vol	uint32_t *)0xE000E10C)

/*	Define NVIC ICERx Core	*/
#define NVIC_ICER0	((__vol	uint32_t *)0XE000E180)
#define NVIC_ICER1	((__vol	uint32_t *)0XE000E184)
#define NVIC_ICER2	((__vol	uint32_t *)0XE000E188)
#define NVIC_ICER3	((__vol	uint32_t *)0XE000E18C)

/*	Define NVIC ICPRx Core	*/
#define NVIC_ICPR0	((__vol	uint32_t *)0XE000E280)
#define NVIC_ICPR1	((__vol	uint32_t *)0XE000E284)
#define NVIC_ICPR2	((__vol	uint32_t *)0XE000E288)
#define NVIC_ICPR3	((__vol	uint32_t *)0XE000E28C)

/*	Define NVIC IPRx Core	*/
#define NVIC_IPR0	((__vol uint32_t *)0xE000E400)

//Registers's base address
#define FLASH_BASEADDR 	0x08000000U
#define SRAM1_BASEADDR 	0x20000000U
#define SRAM2_BASEADDR 	0x2001C000U
#define ROM_BASEADDR 	0x1FFF0000U
#define SRAM 			SRAM1_BASEADDR

//Peripherals's base address
#define PERIPR_BASEADDR 		0x40000000U
#define APB1_PERIPR_BASEADDR 	PERIPR_BASEADDR
#define APB2_PERIPR_BASEADDR 	0x40010000U
#define AHB1_PERIPR_BASEADDR 	0x40020000U
#define AHB2_PERIPR_BASEADDR 	0x50000000U

//AHB1
#define GPIOA_BASEADDR	(AHB1_PERIPR_BASEADDR + 0x0000)
#define GPIOB_BASEADDR	(AHB1_PERIPR_BASEADDR + 0x0400)
#define GPIOC_BASEADDR	(AHB1_PERIPR_BASEADDR + 0x0800)
#define GPIOD_BASEADDR	(AHB1_PERIPR_BASEADDR + 0x0C00)
#define GPIOE_BASEADDR	(AHB1_PERIPR_BASEADDR + 0x1000)
#define GPIOF_BASEADDR	(AHB1_PERIPR_BASEADDR + 0x1400)
#define GPIOG_BASEADDR	(AHB1_PERIPR_BASEADDR + 0x1800)
#define GPIOH_BASEADDR	(AHB1_PERIPR_BASEADDR + 0x1C00)
#define GPIOI_BASEADDR	(AHB1_PERIPR_BASEADDR + 0x2000)
#define GPIOJ_BASEADDR	(AHB1_PERIPR_BASEADDR + 0x2400)
#define GPIOK_BASEADDR	(AHB1_PERIPR_BASEADDR + 0x2800)
#define RCC_BASEADDR	(AHB1_PERIPR_BASEADDR + 0x3800)

//APB1
#define TIM2_BASEADDR	(APB1_PERIPR_BASEADDR + 0x0000)
#define TIM3_BASEADDR	(APB1_PERIPR_BASEADDR + 0x0400)
#define TIM4_BASEADDR	(APB1_PERIPR_BASEADDR + 0x0800)
#define TIM5_BASEADDR	(APB1_PERIPR_BASEADDR + 0x0C00)
#define TIM6_BASEADDR	(APB1_PERIPR_BASEADDR + 0x1000)
#define TIM7_BASEADDR	(APB1_PERIPR_BASEADDR + 0x1400)
#define TIM12_BASEADDR	(APB1_PERIPR_BASEADDR + 0x1800)
#define TIM13_BASEADDR	(APB1_PERIPR_BASEADDR + 0x1C00)
#define TIM14_BASEADDR	(APB1_PERIPR_BASEADDR + 0x2000)

#define USART2_BASEADDR	(APB1_PERIPR_BASEADDR + 0x4400)
#define USART3_BASEADDR	(APB1_PERIPR_BASEADDR + 0x4800)
#define UART4_BASEADDR	(APB1_PERIPR_BASEADDR + 0x4C00)
#define UART5_BASEADDR	(APB1_PERIPR_BASEADDR + 0x5000)

#define I2C1_BASEADDR	(APB1_PERIPR_BASEADDR + 0x5400)
#define I2C2_BASEADDR	(APB1_PERIPR_BASEADDR + 0x5800)
#define I2C3_BASEADDR	(APB1_PERIPR_BASEADDR + 0x5C00)

#define SPI2_BASEADDR	(APB1_PERIPR_BASEADDR + 0x3800)
#define SPI3_BASEADDR	(APB1_PERIPR_BASEADDR + 0x3C00)

//APB2
#define SPI1_BASEADDR	(APB2_PERIPR_BASEADDR + 0x3000)
#define SPI4_BASEADDR	(APB2_PERIPR_BASEADDR + 0x3400)
#define SYSCFG_BASEADDR	(APB2_PERIPR_BASEADDR + 0x3800)
#define EXTI_BASEADDR	(APB2_PERIPR_BASEADDR + 0x3C00)
#define USART1_BASEADDR	(APB2_PERIPR_BASEADDR + 0x1000)
#define USART6_BASEADDR	(APB2_PERIPR_BASEADDR + 0x1400)

//GPIO
typedef struct {
	__vol uint32_t MODER;
	__vol uint32_t OTYPER;
	__vol uint32_t OSPEEDR;
	__vol uint32_t PUPDR;
	__vol uint32_t IDR;
	__vol uint32_t ODR;
	__vol uint32_t BSRR;
	__vol uint32_t LCKR;
	__vol uint32_t AFR[2];
} GPIO_RegDef_t;

#define GPIOA ((GPIO_RegDef_t *)GPIOA_BASEADDR)
#define GPIOB ((GPIO_RegDef_t *)GPIOB_BASEADDR)
#define GPIOC ((GPIO_RegDef_t *)GPIOC_BASEADDR)
#define GPIOD ((GPIO_RegDef_t *)GPIOD_BASEADDR)
#define GPIOE ((GPIO_RegDef_t *)GPIOE_BASEADDR)
#define GPIOF ((GPIO_RegDef_t *)GPIOF_BASEADDR)
#define GPIOG ((GPIO_RegDef_t *)GPIOG_BASEADDR)
#define GPIOH ((GPIO_RegDef_t *)GPIOH_BASEADDR)
#define GPIOI ((GPIO_RegDef_t *)GPIOI_BASEADDR)

//RCC
typedef struct {
	__vol uint32_t CR;					/*Address offset 0x00 */
	__vol uint32_t PLLCFGR;				/*Address offset 0x04 */
	__vol uint32_t CFGR;				/*Address offset 0x08 */
	__vol uint32_t CIR;					/*Address offset 0x0C */
	__vol uint32_t AHB1RSTR;			/*Address offset 0x10 */
	__vol uint32_t AHB2RSTR;			/*Address offset 0x14 */
	__vol uint32_t AHB3RSTR;			/*Address offset 0x18 */
	uint32_t RESERVED0;		//0x1C
	__vol uint32_t APB1RSTR;			/*Address offset 0x20 */
	__vol uint32_t APB2RSTR;			/*Address offset 0x24 */
	uint32_t RESERVED1[2];	//0x28 - 0x2C
	__vol uint32_t AHB1ENR;				/*Address offset 0x30 */
	__vol uint32_t AHB2ENR;				/*Address offset 0x34 */
	__vol uint32_t AHB3ENR;				/*Address offset 0x38 */
	uint32_t RESERVED2;		//0x3C
	__vol uint32_t APB1ENR;				/*Address offset 0x40 */
	__vol uint32_t APB2ENR;				//0x44
	uint32_t RESERVED3[2];	//0x48 - 0x4C
	__vol uint32_t AHB1LPENR;			/*Address offset 0x50 */
	__vol uint32_t AHB2LPENR;			/*Address offset 0x54 */
	__vol uint32_t AHB3LPENR;			/*Address offset 0x58 */
	uint32_t RESERVED4;		//0x5C
	__vol uint32_t APB1LPENR;			/*Address offset 0x60 */
	__vol uint32_t APB2LPENR;			/*Address offset 0x64 */
	uint32_t RESERVED5[2];	//0x68 - 0x6C
	__vol uint32_t BDCR;				/*Address offset 0x70 */
	__vol uint32_t CSR;					/*Address offset 0x74 */
	uint32_t RESERVED6[2];	//0x78 - 0x7C
	__vol uint32_t SSCGR;				/*Address offset 0x80 */
	__vol uint32_t PLLI2SCFGR;			/*Address offset 0x84 */
} RCC_RegDef_t;

#define RCC ((RCC_RegDef_t *)RCC_BASEADDR)

//EXTI
typedef struct {
	__vol uint32_t IMR;		/*Address offset 0x00 */
	__vol uint32_t EMR;		/*Address offset 0x04 */
	__vol uint32_t RTSR;	/*Address offset 0x08 */
	__vol uint32_t FTSR;	/*Address offset 0x0C */
	__vol uint32_t SWIER;	/*Address offset 0x10 */
	__vol uint32_t PR;		/*Address offset 0x14 */
} EXTI_RegDef_t;

#define EXTI ((EXTI_RegDef_t *)EXTI_BASEADDR)

//SYSCFG
typedef struct {
	__vol uint32_t MEMRMP;		/*Address offset 0x00 */
	__vol uint32_t PMC;			/*Address offset 0x04 */
	__vol uint32_t EXTICR[4];	/*Address offset 0x08 - 0x14 */
	uint32_t RESERVED[2];	//0x18 - 0x1C
	__vol uint32_t CMPCR;		/*Address offset 0x20 */
} SYSCFG_RegDef_t;

#define SYSCFG ((SYSCFG_RegDef_t *)SYSCFG_BASEADDR)

//SPI
typedef struct {
	__vol uint32_t CR1;			/*Address offset 0x00 */
	__vol uint32_t CR2;			/*Address offset 0x04 */
	__vol uint32_t SR;			/*Address offset 0x08 */
	__vol uint32_t DR;			/*Address offset 0x0C */
	__vol uint32_t CRCPR;		/*Address offset 0x10 */
	__vol uint32_t RXCRCR;		/*Address offset 0x14 */
	__vol uint32_t TXCRCR;		/*Address offset 0x18 */
	__vol uint32_t I2SCFGR;		/*Address offset 0x1C */
	__vol uint32_t I2SPR;		/*Address offset 0x20 */
} SPI_RegDef_t;

#define SPI1 ((SPI_RegDef_t *)SPI1_BASEADDR)
#define SPI2 ((SPI_RegDef_t *)SPI2_BASEADDR)
#define SPI3 ((SPI_RegDef_t *)SPI3_BASEADDR)
#define SPI4 ((SPI_RegDef_t *)SPI4_BASEADDR)

//I2C
typedef struct {
	__vol uint32_t CR1;			/*Address offset 0x00 */
	__vol uint32_t CR2;			/*Address offset 0x04 */
	__vol uint32_t OAR1;		/*Address offset 0x08 */
	__vol uint32_t OAR2;		/*Address offset 0x0C */
	__vol uint32_t DR;			/*Address offset 0x10 */
	__vol uint32_t SR1;			/*Address offset 0x14 */
	__vol uint32_t SR2;			/*Address offset 0x18 */
	__vol uint32_t CCR;			/*Address offset 0x1C */
	__vol uint32_t TRISE;		/*Address offset 0x20 */
	__vol uint32_t FLTR;		/*Address offset 0x24 */
} I2C_RegDef_t;

#define I2C1 ((I2C_RegDef_t *)I2C1_BASEADDR)
#define I2C2 ((I2C_RegDef_t *)I2C2_BASEADDR
#define I2C3 ((I2C_RegDef_t *)I2C3_BASEADDR

//USART
typedef struct {
	__vol uint32_t SR;			/*Address offset 0x00 */
	__vol uint32_t DR;			/*Address offset 0x04 */
	__vol uint32_t BRR;			/*Address offset 0x08 */
	__vol uint32_t CR1;			/*Address offset 0x0C */
	__vol uint32_t CR2;			/*Address offset 0x10 */
	__vol uint32_t CR3;			/*Address offset 0x14 */
	__vol uint32_t GTPR;		/*Address offset 0x18 */
} USART_RegDef_t;

#define USART1 	((USART_RegDef_t *)USART1_BASEADDR)
#define USART2 	((USART_RegDef_t *)USART2_BASEADDR)
#define USART3 	((USART_RegDef_t *)USART3_BASEADDR)
#define UART4	((USART_RegDef_t *)UART4_BASEADDR)
#define UART5	((USART_RegDef_t *)UART5_BASEADDR)
#define USART6 	((USART_RegDef_t *)USART6_BASEADDR)

//Enable CLK for GPIOx
#define GPIOA_PCLK_EN()	(RCC->AHB1ENR |= (1 << 0))
#define GPIOB_PCLK_EN()	(RCC->AHB1ENR |= (1 << 1))
#define GPIOC_PCLK_EN()	(RCC->AHB1ENR |= (1 << 2))
#define GPIOD_PCLK_EN()	(RCC->AHB1ENR |= (1 << 3))
#define GPIOE_PCLK_EN()	(RCC->AHB1ENR |= (1 << 4))
#define GPIOF_PCLK_EN()	(RCC->AHB1ENR |= (1 << 5))
#define GPIOG_PCLK_EN()	(RCC->AHB1ENR |= (1 << 6))
#define GPIOH_PCLK_EN()	(RCC->AHB1ENR |= (1 << 7))
#define GPIOI_PCLK_EN()	(RCC->AHB1ENR |= (1 << 8))

//Enable CLK for I2Cx
#define I2C1_PCLK_EN()	(RCC->APB1ENR |= (1 << 21))
#define I2C2_PCLK_EN()	(RCC->APB1ENR |= (1 << 22))
#define I2C3_PCLK_EN()	(RCC->APB1ENR |= (1 << 23))

//Enable CLK for SPIx
#define SPI1_PCLK_EN()	(RCC->APB2ENR |= (1 << 12))
#define SPI2_PCLK_EN()	(RCC->APB1ENR |= (1 << 14))
#define SPI3_PCLK_EN()	(RCC->APB1ENR |= (1 << 15))


//Enable CLK for UARTx
#define USART1_PCLK_EN()	(RCC->APB2ENR |= (1 << 4))
#define USART2_PCLK_EN()	(RCC->APB1ENR |= (1 << 17))
#define USART3_PCLK_EN()	(RCC->APB1ENR |= (1 << 18))
#define UART4_PCLK_EN()		(RCC->APB1ENR |= (1 << 19))
#define UART5_PCLK_EN()		(RCC->APB1ENR |= (1 << 20))
#define USART6_PCLK_EN()	(RCC->APB2ENR |= (1 << 5))

//Enable CLK for SYSCFG
#define SYSCFG_PCLK_EN()	(RCC->APB2ENR |= (1 << 14))

//Disable CLK for GPIOx
#define GPIOA_PCLK_DIS()	(RCC->AHB1ENR &=~ (1 << 0))
#define GPIOB_PCLK_DIS()	(RCC->AHB1ENR &=~ (1 << 1))
#define GPIOC_PCLK_DIS()	(RCC->AHB1ENR &=~ (1 << 2))
#define GPIOD_PCLK_DIS()	(RCC->AHB1ENR &=~ (1 << 3))
#define GPIOE_PCLK_DIS()	(RCC->AHB1ENR &=~ (1 << 4))
#define GPIOF_PCLK_DIS()	(RCC->AHB1ENR &=~ (1 << 5))
#define GPIOG_PCLK_DIS()	(RCC->AHB1ENR &=~ (1 << 6))
#define GPIOH_PCLK_DIS()	(RCC->AHB1ENR &=~ (1 << 7))
#define GPIOI_PCLK_DIS()	(RCC->AHB1ENR &=~ (1 << 8))

//Disable CLK for I2Cx
#define I2C1_PCLK_DIS()	(RCC->APB1ENR &=~ (1 << 21))
#define I2C2_PCLK_DIS()	(RCC->APB1ENR &=~ (1 << 22))
#define I2C3_PCLK_DIS()	(RCC->APB1ENR &=~ (1 << 23))

//Disable CLK for SPIx
#define SPI1_PCLK_DIS()	(RCC->APB2ENR &=~ (1 << 12))
#define SPI2_PCLK_DIS()	(RCC->APB1ENR &=~ (1 << 14))
#define SPI3_PCLK_DIS()	(RCC->APB1ENR &=~ (1 << 15))

//Disable CLK for UARTx
#define USART1_PCLK_DIS()	(RCC->APB2ENR &=~ (1 << 4))
#define USART2_PCLK_DIS()	(RCC->APB1ENR &=~ (1 << 17))
#define USART3_PCLK_DIS()	(RCC->APB1ENR &=~ (1 << 18))
#define UART4_PCLK_DIS()	(RCC->APB1ENR &=~ (1 << 19))
#define UART5_PCLK_DIS()	(RCC->APB1ENR &=~ (1 << 20))
#define USART6_PCLK_DIS()	(RCC->APB2ENR &=~ (1 << 5))

//Disable CLK for SYSCFG
#define SYSCFG_PCLK_DIS()	(RCC->PBB2ENR &=~ (1 << 14))

#define ENABLE	1
#define DISABLE	0
#define SET		ENABLE
#define RESET	DISABLE
#define GPIO_PIN_SET	SET
#define GPIO_PIN_RESET	RESET
#define FLAG_SET	SET
#define FLAG_RESET	RESET

//Reset Reg for GPIOx
#define GPIOA_REG_RESET()	do {	(RCC->AHB1RSTR |= (1 << 0));	(RCC->AHB1RSTR &= ~(1 << 0));	} while(0)
#define GPIOB_REG_RESET()	do {	(RCC->AHB1RSTR |= (1 << 1));	(RCC->AHB1RSTR &= ~(1 << 1));	} while(0)
#define GPIOC_REG_RESET()	do {	(RCC->AHB1RSTR |= (1 << 2));	(RCC->AHB1RSTR &= ~(1 << 2));	} while(0)
#define GPIOD_REG_RESET()	do {	(RCC->AHB1RSTR |= (1 << 3));	(RCC->AHB1RSTR &= ~(1 << 3));	} while(0)
#define GPIOE_REG_RESET()	do {	(RCC->AHB1RSTR |= (1 << 4));	(RCC->AHB1RSTR &= ~(1 << 4));	} while(0)
#define GPIOF_REG_RESET()	do {	(RCC->AHB1RSTR |= (1 << 5));	(RCC->AHB1RSTR &= ~(1 << 5));	} while(0)
#define GPIOG_REG_RESET()	do {	(RCC->AHB1RSTR |= (1 << 6));	(RCC->AHB1RSTR &= ~(1 << 6));	} while(0)
#define GPIOH_REG_RESET()	do {	(RCC->AHB1RSTR |= (1 << 7));	(RCC->AHB1RSTR &= ~(1 << 7));	} while(0)
#define GPIOI_REG_RESET()	do {	(RCC->AHB1RSTR |= (1 << 8));	(RCC->AHB1RSTR &= ~(1 << 8));	} while(0)

//Convert configure for EXTICR
#define GPIO_BASEADDR_TO_EXTICR_CODE(x)		\
	((x==GPIOA)?0:(x==GPIOB)?1:(x==GPIOC)?2:	\
			(x==GPIOD)?3:(GPIOE)?4:(x==GPIOF)?5:	\
					(x==GPIOG)?6:(x==GPIOH)?7:(x==GPIOI)?8:0)

//Define EXTI Interrupt line
#define IRQ_EXTI0		6
#define IRQ_EXTI1		7
#define IRQ_EXTI2		8
#define IRQ_EXTI3		9
#define IRQ_EXTI4		10
#define IRQ_EXTI9_5		23
#define IRQ_EXTI15_10	40

#define IRQ_SPI1	35
#define IRQ_SPI2	36
#define IRQ_SPI3	51

//Define SPI_CR1 Reg
#define SPI_CR1_CPHA		0
#define SPI_CR1_CPOL		1
#define SPI_CR1_MSTR		2
#define SPI_CR1_BR			3
#define SPI_CR1_SPE			6
#define SPI_CR1_LSBFIRST	7
#define SPI_CR1_SSI			8
#define SPI_CR1_SSM			9
#define SPI_CR1_RXONLY		10
#define SPI_CR1_DFF			11
#define SPI_CR1_CRCNEXT		12
#define SPI_CR1_CRCEN		13
#define SPI_CR1_BIDIOE		14
#define SPI_CR1_BIDIMODE	15

//Define SPI_CR2 Reg
#define SPI_CR2_RXDMAEND	0
#define SPI_CR2_TXDMAEND	1
#define SPI_CR2_SSOE		2
#define SPI_CR2_FRF			4
#define SPI_CR2_ERRIE		5
#define SPI_CR2_RXNEIE		6
#define SPI_CR2_TXEIE		7

//Define SPI_SR Reg
#define SPI_SR_RXNE		0
#define SPI_SR_TXE		1
#define SPI_SR_CHSIDE	2
#define SPI_SR_UDR		3
#define SPI_SR_CRCERR	4
#define SPI_SR_MODF		5
#define SPI_SR_OVR		6
#define SPI_SR_BSY		7
#define SPI_SR_FRE		8




#endif /* DRIVERS_INC_SMT32F4XX_H_ */
