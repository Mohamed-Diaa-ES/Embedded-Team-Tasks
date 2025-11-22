/*
 * STM32F40xx->h
 *
 *  Created on: Nov 1, 2025
 *      Author: twitter
 */

#ifndef INC_STM32F40XX_H_
#define INC_STM32F40XX_H_
#include <stdint.h>


#define RCC_BaseAddress			0x40023800U
#define Flash_BaseAddress		0x80000000U
#define SRAM1_BaseAddress		0x20000000U
#define SRAM2_BaseAddress		0x2001C000U//offset to start with SRAM
#define ROM_BaseAddress			0x1FFF0000U //system memory
#define SRAM 					SRAM1_BaseAddress


#define Periph_base				0x40000000U
#define APB1Periph_Base			Periph_base
#define APB2_Offset				0x00010000U
#define AHB1_Offset				0x00020000U
#define AHB2_Offset				0x10000000U
#define APB2Periph_Base			Periph_base+APB2_Offset
#define AHB1Periph_Base			Periph_base+AHB1_Offset
#define AHB2Periph_Base 		Periph_base+AHB2_Offset


/*******************-- AHB1_Bus --**********************/
#define GPIOA_BaseAddress	(AHB1Periph_Base+ 0x0000U)
#define GPIOB_BaseAddress	(AHB1Periph_Base+ 0x0400U)
#define GPIOC_BaseAddress	(AHB1Periph_Base+ 0x0800U)
#define GPIOD_BaseAddress	(AHB1Periph_Base+ 0x0C00U)
#define GPIOE_BaseAddress	(AHB1Periph_Base+ 0x1000U)
#define GPIOF_BaseAddress	(AHB1Periph_Base+ 0x1400U)
#define GPIOG_BaseAddress	(AHB1Periph_Base+ 0x1800U)
#define GPIOH_BaseAddress	(AHB1Periph_Base+ 0x1C00U)
#define GPIOI_BaseAddress	(AHB1Periph_Base+ 0x2000U)
/*******************-- AHB1_Bus --**********************/


/*******************-- APB1_Bus --**********************/
#define I2C1_BaseAddress	(APB1Periph_Base + 0x5400U)
#define I2C2_BaseAddress	(APB1Periph_Base + 0x5800U)
#define I2C3_BaseAddress	(APB1Periph_Base + 0x5C00U)

#define SPI2_BaseAddress	(APB1Periph_Base + 0x3C00U)
#define SPI3_BaseAddress	(APB1Periph_Base + 0x3800U)

#define USART2_BaseAddress	(APB1Periph_Base + 0x4400U)
#define USART3_BaseAddress	(APB1Periph_Base + 0x4800U)
#define UART4_BaseAddress	(APB1Periph_Base + 0x5C00U)
#define UART5_BaseAddress	(APB1Periph_Base + 0x4C00U)

/*******************-- APB1_Bus --**********************/
/*******************-- APB2_Bus --**********************/
#define SPI1_BaseAddress	(APB2Periph_Base + 	0x3000U)
#define USART1_BaseAddress	(APB2Periph_Base + 	0x1000U)
#define USART6_BaseAddress	(APB2Periph_Base + 	0x1400U)
#define EXTI_BaseAddress	(APB2Periph_Base + 	0x3C00U)
#define SYSCFG_BaseAddress	(APB2Periph_Base + 	0x3800U)
/*******************-- APB2_Bus --**********************/
/**********************NVIC********************************/

#define NVIC_ISER0  ((volatile uint32_t*)0xE000E100)
#define NVIC_ISER1	((volatile uint32_t*)0xE000E104)
#define NVIC_ISER2	((volatile uint32_t*)0xE000E108)
#define NVIC_ISER3	((volatile uint32_t*)0xE000E10C)

#define NVIC_ICER0  ((volatile uint32_t*)0XE000E180)
#define NVIC_ICER1	((volatile uint32_t*)0xE000E184)
#define NVIC_ICER2	((volatile uint32_t*)0xE000E188)
#define NVIC_ICER3	((volatile uint32_t*)0xE000E18C)

#define NVIC_IPR_BaseAddress  ((volatile uint32_t*)0xE000E100)


#define BitsNumImplemented 		4
/**********************NVIC********************************/
/********************************** Peripheral Register Definition ***********************************************************/
/******************************************************GPIO Structure*******************************************************/
typedef struct
{
	/*Configuration Register*/
	volatile uint32_t GPIOx_MODER; // Changing the mode of the register
	volatile uint32_t GPIOx_OTYPER;
	volatile uint32_t GPIOx_OSPEEDR;
	volatile uint32_t GPIOx_PUPDR ;
	volatile uint32_t GPIOx_IDR;
	volatile uint32_t GPIOx_ODR;
	volatile uint32_t GPIOx_BSRR;
	volatile uint32_t GPIOx_LCKR;
	volatile uint64_t GPIOx_AFRL_H; //Low then High from Pin 0 to 15 shift 4 bits at a time to go from a pin to another


}GPIO_Reg_Def_t;

/**peripheral Definition */
#define GPIOA 	((GPIO_Reg_Def_t*)GPIOA_BaseAddress)
#define GPIOB	((GPIO_Reg_Def_t*)GPIOB_BaseAddress)
#define GPIOC	((GPIO_Reg_Def_t*)GPIOC_BaseAddress)
#define GPIOD 	((GPIO_Reg_Def_t*)GPIOD_BaseAddress)
#define GPIOE	((GPIO_Reg_Def_t*)GPIOE_BaseAddress)
#define GPIOF	((GPIO_Reg_Def_t*)GPIOF_BaseAddress)
#define GPIOG 	((GPIO_Reg_Def_t*)GPIOG_BaseAddress)
#define GPIOH	((GPIO_Reg_Def_t*)GPIOH_BaseAddress)
#define GPIOI	((GPIO_Reg_Def_t*)GPIOI_BaseAddress)

/**peripheral Definition */
/******************************************************GPIO Structure*******************************************************/

/**peripheral Definition */
/******************************************************RCC Structure*******************************************************/
typedef struct
{
	/*Configuration Register*/
	volatile uint32_t RCC_CR; // Changing the mode of the register
	volatile uint32_t RCC_PLLCFGR;
	volatile uint32_t RCC_CFGR;
	volatile uint32_t RCC_CIR ;
	volatile uint32_t RCC_AHB1RSTR;
	volatile uint32_t RCC_AHB2RSTR;
	volatile uint32_t RCC_AHB3RSTR;
	uint32_t Reserved0;
	volatile uint32_t RCC_APB1RSTR;
	volatile uint32_t RCC_APB2RSTR;
	uint32_t Reserved1[2];
	volatile uint32_t RCC_AHB1ENR;
	volatile uint32_t RCC_AHB2ENR;
	volatile uint32_t RCC_AHB3ENR;
	uint32_t Reserved2;
	volatile uint32_t RCC_APB1ENR;
	volatile uint32_t RCC_APB2ENR;
	uint32_t Reserved3[2];
	volatile uint32_t RCC_AHB1LPENR;
	volatile uint32_t RCC_AHB2LPENR;
	volatile uint32_t RCC_AHB3LPENR;
	uint32_t Reserved4;
	volatile uint32_t RCC_APB1LPENR;
	volatile uint32_t RCC_APB2LPENR;
	uint32_t Reserved5[2];
	volatile uint32_t RCC_BDCR;
	volatile uint32_t RCC_CSR;
	uint32_t Reserved6[2];
	volatile uint32_t RCC_SSCGR;
	volatile uint32_t RCC_PLLI2SCFGR;
}RCC_Reg_Def_t;

#define RCC_Reg   ((RCC_Reg_Def_t *)RCC_BaseAddress)
/********************************** Peripheral Register Enable Macros ***********************************************************/
#define GPIOA_PClk_EN()		(RCC_Reg->RCC_AHB1ENR|=(1<<0))
#define GPIOB_PClk_EN()		(RCC_Reg->RCC_AHB1ENR|=(1<<1))
#define GPIOC_PClk_EN()		(RCC_Reg->RCC_AHB1ENR|=(1<<2))
#define GPIOD_PClk_EN()		(RCC_Reg->RCC_AHB1ENR|=(1<<3))
#define GPIOE_PClk_EN()		(RCC_Reg->RCC_AHB1ENR|=(1<<4))
#define GPIOF_PClk_EN()		(RCC_Reg->RCC_AHB1ENR|=(1<<5))
#define GPIOG_PClk_EN()		(RCC_Reg->RCC_AHB1ENR|=(1<<6))
#define GPIOH_PClk_EN()		(RCC_Reg->RCC_AHB1ENR|=(1<<7))
#define GPIOI_PClk_EN()		(RCC_Reg->RCC_AHB1ENR|=(1<<8))
//I2c
#define I2C1_PClk_EN()		(RCC_Reg->RCC_APB1ENR|=(1<<21))
#define I2C2_PClk_EN()		(RCC_Reg->RCC_APB1ENR|=(1<<22))
#define I2C3_PClk_EN()		(RCC_Reg->RCC_APB1ENR|=(1<<23))
//SPI
#define SPI1_PClk_EN()		(RCC_Reg->RCC_APB1ENR|=(1<<12))
#define SPI2_PClk_EN()		(RCC_Reg->RCC_APB1ENR|=(1<<14))
#define SPI3_PClk_EN()		(RCC_Reg->RCC_APB2ENR|=(1<<15))
//UART && USART
#define USART1_PClk_EN()		(RCC_Reg->RCC_APB2ENR|=(1<<4))
#define USART2_PClk_EN()		(RCC_Reg->RCC_APB1ENR|=(1<<17))
#define USART3_PClk_EN()		(RCC_Reg->RCC_APB1ENR|=(1<<18))
#define UART4_PClk_EN()		(RCC_Reg->RCC_APB1ENR|=(1<<19))
#define UART5_PClk_EN()		(RCC_Reg->RCC_APB1ENR|=(1<<20))
#define USART6_PClk_EN()		(RCC_Reg->RCC_APB2ENR|=(1<<5))
//EXTI
#define USART1_PClk_EN()		(RCC_Reg->RCC_APB2ENR|=(1<<4))

//SYSCFG
#define SYSCFG_PClk_EN()		(RCC_Reg->RCC_APB2ENR|=(1<<14))
/*************************************************************************************************************************/
/********************************** Peripheral Register Disable Macros ***********************************************************/
#define GPIOA_PClk_DI()		(RCC_Reg->RCC_AHB1ENR&=~(1<<0))
#define GPIOB_PClk_DI()		(RCC_Reg->RCC_AHB1ENR&=~(1<<1))
#define GPIOC_PClk_DI()		(RCC_Reg->RCC_AHB1ENR&=~(1<<2))
#define GPIOD_PClk_DI()		(RCC_Reg->RCC_AHB1ENR&=~(1<<3))
#define GPIOE_PClk_DI()		(RCC_Reg->RCC_AHB1ENR&=~(1<<4))
#define GPIOF_PClk_DI()		(RCC_Reg->RCC_AHB1ENR&=~(1<<5))
#define GPIOG_PClk_DI()		(RCC_Reg->RCC_AHB1ENR&=~(1<<6))
#define GPIOH_PClk_DI()		(RCC_Reg->RCC_AHB1ENR&=~(1<<7))
#define GPIOI_PClk_DI()		(RCC_Reg->RCC_AHB1ENR&=~(1<<8))

//I2c
#define I2C1_PClk_DI()		(RCC_Reg->RCC_APB1ENR&=~(1<<21))
#define I2C2_PClk_DI()		(RCC_Reg->RCC_APB1ENR&=~(1<<22))
#define I2C3_PClk_DI()		(RCC_Reg->RCC_APB1ENR&=~(1<<23))
//SPI
#define SPI1_PClk_DI()		(RCC_Reg->RCC_APB1ENR&=~(1<<12))
#define SPI2_PClk_DI()		(RCC_Reg->RCC_APB1ENR&=~(1<<14))
#define SPI3_PClk_DI()		(RCC_Reg->RCC_APB2ENR&=~(1<<15))
//UART && USART
#define USART1_PClk_DI()		(RCC_Reg->RCC_APB2ENR&=~(1<<4))
#define USART2_PClk_DI()		(RCC_Reg->RCC_APB1ENR&=~(1<<17))
#define USART3_PClk_DI()		(RCC_Reg->RCC_APB1ENR&=~(1<<18))
#define UART4_PClk_DI()		(RCC_Reg->RCC_APB1ENR&=~(1<<19))
#define UART5_PClk_DI()		(RCC_Reg->RCC_APB1ENR&=~(1<<20))
#define USART6_PClk_DI()		(RCC_Reg->RCC_APB2ENR&=~(1<<5))
//SYSCFG
#define SYSCFG_PClk_DI()		(RCC_Reg->RCC_APB2ENR&=~(1<<14))
/********************************** Peripheral Register Disable Macros ***********************************************************/
/********************************************************Macros**********************************************************************/
#define Enable 		1
#define Disable 	0
#define Set 		1
#define Reset 		0
#define GPIO_High 	Set
#define GPIO_Low 	Reset


#define GPIOA_REG_Reset()	do {(RCC_Reg->RCC_AHB1RSTR|=(1<<0));	(RCC_Reg->RCC_AHB1RSTR&=~(1<<0)); }while(0)
#define GPIOB_REG_Reset()	do {(RCC_Reg->RCC_AHB1RSTR|=(1<<1));	(RCC_Reg->RCC_AHB1RSTR&=~(1<<1)); }while(0)
#define GPIOC_REG_Reset()	do {(RCC_Reg->RCC_AHB1RSTR|=(1<<2));	(RCC_Reg->RCC_AHB1RSTR&=~(1<<2)); }while(0)
#define GPIOD_REG_Reset()	do {(RCC_Reg->RCC_AHB1RSTR|=(1<<3));	(RCC_Reg->RCC_AHB1RSTR&=~(1<<3)); }while(0)
#define GPIOE_REG_Reset()	do {(RCC_Reg->RCC_AHB1RSTR|=(1<<4));	(RCC_Reg->RCC_AHB1RSTR&=~(1<<4)); }while(0)
#define GPIOF_REG_Reset()	do {(RCC_Reg->RCC_AHB1RSTR|=(1<<5));	(RCC_Reg->RCC_AHB1RSTR&=~(1<<5)); }while(0)
#define GPIOG_REG_Reset()	do {(RCC_Reg->RCC_AHB1RSTR|=(1<<6));	(RCC_Reg->RCC_AHB1RSTR&=~(1<<6)); }while(0)
#define GPIOH_REG_Reset()	do {(RCC_Reg->RCC_AHB1RSTR|=(1<<7));	(RCC_Reg->RCC_AHB1RSTR&=~(1<<7)); }while(0)
#define GPIOI_REG_Reset()	do {(RCC_Reg->RCC_AHB1RSTR|=(1<<8));	(RCC_Reg->RCC_AHB1RSTR&=~(1<<8)); }while(0)
/********************************************************Macros**********************************************************************/
/*********************************************************EXTI************************************************************************/
/**Structure of EXTI */
typedef struct 
{
	volatile uint32_t EXTI_IMR;
	volatile uint32_t EXTI_EMR;
	volatile uint32_t EXTI_RTSR;
	volatile uint32_t EXTI_FTSR;
	volatile uint32_t EXTI_SWIER;
	volatile uint32_t EXTI_PR;
} EXTI_Reg_Def_t;

/**Structure of EXTI */


#define EXTI_Reg   (( EXTI_Reg_Def_t *)EXTI_BaseAddress)
/*********************************************************EXTI************************************************************************/
/*********************************************************SysCfg************************************************************************/
/**Structure of EXTI */
typedef struct 
{
	volatile uint32_t SYSCFG_MEMRMP;
	volatile uint32_t SYSCFG_PMC;
	volatile uint32_t SYSCFG_EXTICR[4]; // 0x14
	uint32_t Reseved[2]; //0x14+4 and 0x01C+4
	volatile uint32_t  SYSCFG_CMPCR;
} SYSCFG_Reg_Def_t;

/**Structure of SYSCFG */


#define SYSCFG_Reg   (( SYSCFG_Reg_Def_t *)SYSCFG_BaseAddress)
/*********************************************************SysCfg************************************************************************/
#define Get_GPIO_Address_Code(GPIO_REG) (	(GPIO_Reg==GPIOA)?0:\
											(GPIO_Reg==GPIOB)?1:\
											(GPIO_Reg==GPIOC)?2:\
											(GPIO_Reg==GPIOD)?3:\
											(GPIO_Reg==GPIOE)?4:\
											(GPIO_Reg==GPIOF)?5:\
											(GPIO_Reg==GPIOG)?6:\
											(GPIO_Reg==GPIOH)?7:8)
										
/**********************************************************IRQ NUMBERs for the EXTIs*******************************************************************/


#define IRQ_NO_EXTI_0		6
#define IRQ_NO_EXTI_1		7
#define IRQ_NO_EXTI_2		8
#define IRQ_NO_EXTI_3		9
#define IRQ_NO_EXTI_4		10
#define IRQ_NO_EXTI9_5		23
#define IRQ_NO_EXTI15_10	40


/**********************************************************IRQ NUMBERs*******************************************************************/
/**********************************************************ISER Registers*******************************************************************/


/**********************************************************ISER Registers*******************************************************************/

#endif /* INC_STM32F40XX_H_ */