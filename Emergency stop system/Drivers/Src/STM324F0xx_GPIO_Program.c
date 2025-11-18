/*
 * STM324F0xx_GPIO_Program.c
 *
 *  Created on: Nov 14, 2025
 *      Author: twitter
 */

#include "Stm324Fxx_GPIO_Interface.h"


void GPIO_Init(GPIO_Handle_t* GPIO_Ptr)
{
    if (GPIO_Ptr->GPIO_PinConfig.PinNum>GPIO_PinNum_15)
    {
        return;
    }
    
    uint32_t Temp=0;
    if (GPIO_Ptr->GPIO_PinConfig.Mode<=GPIO_MODE_Analog)
    {
            Temp =GPIO_Ptr->ptr_GPIOxr->GPIOx_MODER;
            Temp &=~(3<<(2*GPIO_Ptr->GPIO_PinConfig.PinNum));
             
            Temp |=GPIO_Ptr->GPIO_PinConfig.Mode<<(2*GPIO_Ptr->GPIO_PinConfig.PinNum);
            GPIO_Ptr->ptr_GPIOxr->GPIOx_MODER =Temp;
    }else   
    {
        
// Force pin to input mode
                Temp = GPIO_Ptr->ptr_GPIOxr->GPIOx_MODER;
                Temp &= ~(3 << (2 * GPIO_Ptr->GPIO_PinConfig.PinNum)); // Clear mode bits
                GPIO_Ptr->ptr_GPIOxr->GPIOx_MODER = Temp; // Input mode (00)

            if (GPIO_Ptr->GPIO_PinConfig.Mode==GPIO_MODE_IT_FT)
            {
                
                EXTI_Reg->EXTI_FTSR|=(1<<GPIO_Ptr->GPIO_PinConfig.PinNum);
                EXTI_Reg->EXTI_RTSR&=~(1<<GPIO_Ptr->GPIO_PinConfig.PinNum);
            }else if (GPIO_Ptr->GPIO_PinConfig.Mode==GPIO_MODE_IT_RT)
            {
                EXTI_Reg->EXTI_RTSR|=(1<<GPIO_Ptr->GPIO_PinConfig.PinNum);
                EXTI_Reg->EXTI_FTSR&=~(1<<GPIO_Ptr->GPIO_PinConfig.PinNum);
                
                
            }else if (GPIO_Ptr->GPIO_PinConfig.Mode==GPIO_MODE_IT_RFT)
            {
                EXTI_Reg->EXTI_FTSR|=(1<<GPIO_Ptr->GPIO_PinConfig.PinNum);
                EXTI_Reg->EXTI_RTSR|=(1<<GPIO_Ptr->GPIO_PinConfig.PinNum);
                
            }
            SYSCFG_PClk_EN();
            uint8_t whichline=GPIO_Ptr->GPIO_PinConfig.PinNum/4;
            uint8_t whichField=(GPIO_Ptr->GPIO_PinConfig.PinNum%4)*4;
            uint8_t whichport=(GPIO_Ptr->ptr_GPIOxr-GPIOA)/4; //or use Get_GPIO_Address_Code(GPIO_PTR)
            // config the GPIO_Port selection  in sysconfig-EXTICr
            SYSCFG_Reg->SYSCFG_EXTICR[whichline] &= ~(0xF << (whichField)); // clear
            SYSCFG_Reg->SYSCFG_EXTICR[whichline] |= (whichport << (whichField)); // set
            //enable the exti delviery using the interrupt IMR 
                EXTI_Reg->EXTI_IMR|=(1<<GPIO_Ptr->GPIO_PinConfig.PinNum);
    }


    Temp =0;
    if (GPIO_Ptr->GPIO_PinConfig.Speed<=Very_high_speed)
    {
            Temp =GPIO_Ptr->ptr_GPIOxr->GPIOx_OSPEEDR;
            Temp &=~(3<<(2*GPIO_Ptr->GPIO_PinConfig.PinNum));
             
            Temp |=GPIO_Ptr->GPIO_PinConfig.Speed<<(2*GPIO_Ptr->GPIO_PinConfig.PinNum);
            GPIO_Ptr->ptr_GPIOxr->GPIOx_OSPEEDR =Temp;
    }else   
    {

    }
    Temp =0;
    if (GPIO_Ptr->GPIO_PinConfig.PullUp_PullDown<=Pull_down)
    {
            Temp =GPIO_Ptr->ptr_GPIOxr->GPIOx_PUPDR;
            Temp &=~(3<<(2*GPIO_Ptr->GPIO_PinConfig.PinNum));
             
            Temp |=GPIO_Ptr->GPIO_PinConfig.PullUp_PullDown<<(2*GPIO_Ptr->GPIO_PinConfig.PinNum);
            GPIO_Ptr->ptr_GPIOxr->GPIOx_PUPDR =Temp;
    }else   
    {

    }
    Temp =0;
    if (GPIO_Ptr->GPIO_PinConfig.OutputType<=GPIO_High)
    {
            Temp =GPIO_Ptr->ptr_GPIOxr->GPIOx_OTYPER;
            Temp &=~(1<<(GPIO_Ptr->GPIO_PinConfig.PinNum));
             
            Temp |=GPIO_Ptr->GPIO_PinConfig.OutputType<<(GPIO_Ptr->GPIO_PinConfig.PinNum);
            GPIO_Ptr->ptr_GPIOxr->GPIOx_OTYPER =Temp;
    }else   
    {

    }
    
    if (GPIO_Ptr->GPIO_PinConfig.Mode==GPIO_MODE_ALT_FN)
    {
            Temp =GPIO_Ptr->ptr_GPIOxr->GPIOx_AFRL_H;
            Temp &=~(0xF<<(4*GPIO_Ptr->GPIO_PinConfig.PinNum));
             
            Temp |=GPIO_Ptr->GPIO_PinConfig.Alternate_Function<<(4*GPIO_Ptr->GPIO_PinConfig.PinNum);
            GPIO_Ptr->ptr_GPIOxr->GPIOx_AFRL_H =Temp;
    }


}   //initialize
void GPIO_DeInit(GPIO_Reg_Def_t* ptr_GPIOxr)
{
        if (GPIOA==ptr_GPIOxr)
        {
            GPIOA_REG_Reset();   
        } 
        else if (GPIOB==ptr_GPIOxr)
        {
            GPIOB_REG_Reset();   
        }
        else if (GPIOC==ptr_GPIOxr)
        {
            GPIOC_REG_Reset();   
        }
        else if (GPIOD==ptr_GPIOxr)
        {
            GPIOD_REG_Reset();   
        }
        else if (GPIOE==ptr_GPIOxr)
        {
            GPIOE_REG_Reset();   
        }
        else if (GPIOF==ptr_GPIOxr)
        {
            GPIOF_REG_Reset();   
        }
        else if (GPIOG==ptr_GPIOxr)
        {
            GPIOG_REG_Reset();   
        }
        else if (GPIOH==ptr_GPIOxr)
        {
            GPIOH_REG_Reset();   
        }
        else if (GPIOI==ptr_GPIOxr)
        {
            GPIOI_REG_Reset();   
        }
        
    
    
    
} //deinitialize -> reset state ->use the RCC reset register

// enable and disable  certain GPIO
/**
 * -> needs base address 
 */

void GPIO_PeriClockControl(GPIO_Reg_Def_t* ptr_GPIOxr,uint8_t En_Dis)
{
    if (En_Dis ==Enable)
    {
        if (GPIOA==ptr_GPIOxr)
        {
            GPIOA_PClk_EN();   
        } 
        else if (GPIOB==ptr_GPIOxr)
        {
            GPIOB_PClk_EN();   
        }
        else if (GPIOC==ptr_GPIOxr)
        {
            GPIOC_PClk_EN();   
        }
        else if (GPIOD==ptr_GPIOxr)
        {
            GPIOD_PClk_EN();   
        }
        else if (GPIOE==ptr_GPIOxr)
        {
            GPIOE_PClk_EN();   
        }
        else if (GPIOF==ptr_GPIOxr)
        {
            GPIOF_PClk_EN();   
        }
        else if (GPIOG==ptr_GPIOxr)
        {
            GPIOG_PClk_EN();   
        }
        else if (GPIOH==ptr_GPIOxr)
        {
            GPIOH_PClk_EN();   
        }
        else if (GPIOI==ptr_GPIOxr)
        {
            GPIOI_PClk_EN();   
        }
        
    }else 
    {
        if (GPIOA==ptr_GPIOxr)
        {
            GPIOA_PClk_DI();   
        } 
        else if (GPIOB==ptr_GPIOxr)
        {
            GPIOB_PClk_DI();   
        }
        else if (GPIOC==ptr_GPIOxr)
        {
            GPIOC_PClk_DI();   
        }
        else if (GPIOD==ptr_GPIOxr)
        {
            GPIOD_PClk_DI();   
        }
        else if (GPIOE==ptr_GPIOxr)
        {
            GPIOE_PClk_DI();   
        }
        else if (GPIOF==ptr_GPIOxr)
        {
            GPIOF_PClk_DI();   
        }
        else if (GPIOH==ptr_GPIOxr)
        {
            GPIOH_PClk_DI();   
        }
        else if (GPIOI==ptr_GPIOxr)
        {
            GPIOI_PClk_DI();   
        }
        
    }
    
}
//
uint8_t GPIO_ReadFromInputPin(GPIO_Reg_Def_t* ptr_GPIOxr,uint8_t PinNum)
{
    
} //read a pin
uint16_t GPIO_ReadFromInputPort(GPIO_Reg_Def_t* ptr_GPIOxr)
{
    
} // read from Group 
void GPIO_WriteToOutputPin(GPIO_Reg_Def_t* ptr_GPIOxr,uint8_t PinNum,uint8_t Value)
{
    
} //write a pin
void GPIO_WriteToOutputPort(GPIO_Reg_Def_t* ptr_GPIOxr,uint16_t Value)
{
    
} // write a group
void GPIO_ToggleOutputPin(GPIO_Reg_Def_t* ptr_GPIOxr,uint8_t PinNum)
{
    
}
void GPIO_ToggleOutputPort(GPIO_Reg_Def_t* ptr_GPIOxr)
{
    
}

void GPIO_IRQConfig_EN_DIS(uint8_t IRQ_Num,uint8_t En_Dis)
{
    if (En_Dis==Enable)
    {
        if (IRQ_Num<=31)
        {
            /* code */
            *NVIC_ISER0|=1<<(IRQ_Num%32);
        }else if(IRQ_Num>31&&IRQ_Num<=63)
        {
            *NVIC_ISER1|=1<<(IRQ_Num%32);
            
        }else if(IRQ_Num>63&&IRQ_Num<96)
        {
            *NVIC_ISER2|=1<<(IRQ_Num%32);
            
        }
        
    }else 
    {      if (IRQ_Num<=31)
        {
            /* code */
            *NVIC_ICER0|=1<<(IRQ_Num%32);
        }else if(IRQ_Num>31&&IRQ_Num<=63)
        {
            *NVIC_ICER1|=1<<(IRQ_Num%32);
            
        }else if(IRQ_Num>63&&IRQ_Num<96)
        {
            *NVIC_ICER2|=1<<(IRQ_Num%32);
            
        }
    }
    
}
void GPIO_IRQConfig_Priority(uint8_t IRQ_Number,uint8_t IRQ_priority)
{
    uint8_t IPRX =IRQ_Number/4;
    uint8_t IPRX_Section =IRQ_Number%4;
    uint8_t ShiftAmount =(IPRX_Section*8)+(8-BitsNumImplemented);
    *(NVIC_IPR_BaseAddress+IPRX)|=(IRQ_priority<<ShiftAmount);
}
void GPIO_IRQHandling(uint8_t PinNum)
{
 if (PinNum==0)
 {
    GPIOA->GPIOx_ODR&=~(0x6); //pin 1 and 2 
}if (PinNum==3)
{
    
    GPIOA->GPIOx_ODR|=(0x6); //pin 1 and 2 
 }
 
 
    
}

