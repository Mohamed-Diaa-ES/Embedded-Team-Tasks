/*
 * Stm324Fxx_GPIO_Interface.h
 *
 *  Created on: Nov 14, 2025
 *      Author: twitter
 */

#ifndef INC_STM324FXX_GPIO_INTERFACE_H_
#define INC_STM324FXX_GPIO_INTERFACE_H_
#include "STM32F40xx.h"
#include <stdint.h>
/***************************************************/
///GPIO_PIN_MODES
///GPIO Input MODE 
#define GPIO_MODE_INPUT     0
#define GPIO_MODE_Output    1
#define GPIO_MODE_ALT_FN    2    
#define GPIO_MODE_Analog    3    
//GPIO Interrupt 
#define GPIO_MODE_IT_FT     4
#define GPIO_MODE_IT_RT     5
#define GPIO_MODE_IT_RFT    6

///GPIO_Output_MODES
#define GPIO_OP_Type_PushPull       0
#define GPIO_OP_Type_OpenDrain      1
// GPIO_SPEED_MODES

#define  Low_speed              0
#define  Medium_speed           1
#define  High_speed             2
#define  Very_high_speed        3
// GPIO_PullUp_PullDown_MODES
#define  Neither_Pull_UpDown    0
#define  Pull_up                1
#define  Pull_down              2
/***GPIO PIN NUMBERS */
#define GPIO_PinNum_0        0
#define GPIO_PinNum_1        1
#define GPIO_PinNum_2        2
#define GPIO_PinNum_3        3
#define GPIO_PinNum_4        4
#define GPIO_PinNum_5        5
#define GPIO_PinNum_6        6
#define GPIO_PinNum_7        7
#define GPIO_PinNum_8        8
#define GPIO_PinNum_9        9
#define GPIO_PinNum_10      10
#define GPIO_PinNum_11      11
#define GPIO_PinNum_12      12
#define GPIO_PinNum_13      13
#define GPIO_PinNum_14      14
#define GPIO_PinNum_15      15
/***GPIO PIN NUMBERS */


/***************************************************/
typedef struct 
{
uint8_t PortNum;
uint8_t PinNum;
uint8_t Mode;
uint8_t Speed;
uint8_t OutputType;
uint8_t PullUp_PullDown;
uint8_t Alternate_Function;

}GPIO_PinConfig_t;
typedef struct 
{
    //ptr to base address
    GPIO_Reg_Def_t *ptr_GPIOxr ; // this holds the base address
    GPIO_PinConfig_t GPIO_PinConfig; //-> this  holds the GPIO pin configuration settings.
}GPIO_Handle_t;
/*-------------------------------GPIO APIS---------------------------------------- */

void GPIO_Init(GPIO_Handle_t* GPIO_Ptr);   //initialize
void GPIO_DeInit(GPIO_Reg_Def_t* ptr_GPIOxr); //deinitialize -> reset state ->use the RCC reset register

// enable and disable  certain GPIO
/**
 * -> needs base address 
 */

void GPIO_PeriClockControl(GPIO_Reg_Def_t* ptr_GPIOxr,uint8_t En_Dis);
//
uint8_t GPIO_ReadFromInputPin(GPIO_Reg_Def_t* ptr_GPIOxr,uint8_t PinNum); //read a pin
uint16_t GPIO_ReadFromInputPort(GPIO_Reg_Def_t* ptr_GPIOxr); // read from Group 
void GPIO_WriteToOutputPin(GPIO_Reg_Def_t* ptr_GPIOxr,uint8_t PinNum,uint8_t Value); //write a pin
void GPIO_WriteToOutputPort(GPIO_Reg_Def_t* ptr_GPIOxr,uint16_t Value); // write a group
void GPIO_ToggleOutputPin(GPIO_Reg_Def_t* ptr_GPIOxr,uint8_t PinNum);
void GPIO_ToggleOutputPort(GPIO_Reg_Def_t* ptr_GPIOxr);

void GPIO_IRQConfig_EN_DIS(uint8_t IRQ_Num,uint8_t En_Dis);
void GPIO_IRQConfig_Priority(uint8_t IRQ_Number,uint8_t IRQ_priority);
void GPIO_IRQHandling(uint8_t PinNum);//Handling the interrupts




#endif /* INC_STM324FXX_GPIO_INTERFACE_H_ */
