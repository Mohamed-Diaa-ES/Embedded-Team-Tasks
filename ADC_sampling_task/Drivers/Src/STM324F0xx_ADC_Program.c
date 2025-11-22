
/*
 * STM324Fxx_ADC_Program.c
 * Created on: Nov 22, 2025
 * Author: mohamed deyaa
 */

#include "STM324F0xx_ADC_Interface.h"


void ADC_PeriClockControl(ADC_Handle_t* ADC_Ptr, uint8_t En_Dis)
{
    (void)ADC_Ptr; 
    if (En_Dis) { ADC1_PClk_EN(); } else { ADC1_PClk_DI(); }
}


void ADC_Init(ADC_Handle_t* ADC_Ptr)
{
    if (!ADC_Ptr) return;

  
    ADC_PeriClockControl(ADC_Ptr, 1U);

  
    ADC_Ptr->ptr_ADCcommon->CCR &= ~(3U << 16);
    ADC_Ptr->ptr_ADCcommon->CCR |= ADC_CCR_PSC_DIV4;

   
    uint32_t cr1 = ADC_Ptr->ptr_ADCxr->CR1;
    cr1 &= ~(3U << 24);
    cr1 |= ADC_Ptr->ADC_Config.Resolution;
    ADC_Ptr->ptr_ADCxr->CR1 = cr1;

  
    uint32_t cr2 = ADC_Ptr->ptr_ADCxr->CR2;
    cr2 &= ~(ADC_ALIGN_LEFT | ADC_CONT);
    cr2 |= (ADC_Ptr->ADC_Config.Alignment |
            (ADC_Ptr->ADC_Config.Continuous ? ADC_CONT : ADC_SINGLE) |
            ADC_EOCS); 
    ADC_Ptr->ptr_ADCxr->CR2 = cr2;

   
    ADC_Ptr->ptr_ADCxr->SQR1 &= ~(0xFU << 20);

  
    ADC_SetChannel(ADC_Ptr, ADC_Ptr->ADC_Config.Channel);

    
    ADC_SetSamplingTime(ADC_Ptr, ADC_Ptr->ADC_Config.Channel,
                        ADC_Ptr->ADC_Config.SamplingTime);


    ADC_Ptr->ptr_ADCxr->CR2 |= ADC_ADON;
}


void ADC_DeInit(ADC_Handle_t* ADC_Ptr)
{
    if (!ADC_Ptr) return;
 
    ADC_Ptr->ptr_ADCxr->CR2 &= ~ADC_ADON;

    ADC_PeriClockControl(ADC_Ptr, 0U);
}


void ADC_SetChannel(ADC_Handle_t* ADC_Ptr, uint8_t channel)
{
    
    uint32_t sqr3 = ADC_Ptr->ptr_ADCxr->SQR3;
    sqr3 &= ~0x1FU;
    sqr3 |= (channel & 0x1FU);
    ADC_Ptr->ptr_ADCxr->SQR3 = sqr3;
    ADC_Ptr->ADC_Config.Channel = channel;
}


void ADC_SetSamplingTime(ADC_Handle_t* ADC_Ptr, uint8_t channel, uint8_t smp)
{
    if (channel <= 9U) {
        uint32_t pos = ADC_SMPR2_POS(channel);
        uint32_t smpr2 = ADC_Ptr->ptr_ADCxr->SMPR2;
        smpr2 &= ~(0x7U << pos);
        smpr2 |= ((uint32_t)(smp & 0x7U) << pos);
        ADC_Ptr->ptr_ADCxr->SMPR2 = smpr2;
    } else {
        uint32_t pos = ADC_SMPR1_POS(channel);
        uint32_t smpr1 = ADC_Ptr->ptr_ADCxr->SMPR1;
        smpr1 &= ~(0x7U << pos);
        smpr1 |= ((uint32_t)(smp & 0x7U) << pos);
        ADC_Ptr->ptr_ADCxr->SMPR1 = smpr1;
    }
    ADC_Ptr->ADC_Config.SamplingTime = smp;
}


void ADC_StartConversion(ADC_Handle_t* ADC_Ptr)
{
  
    ADC_Ptr->ptr_ADCxr->CR2 |= ADC_SWSTART;
}

uint8_t ADC_IsConversionDone(ADC_Handle_t* ADC_Ptr)
{
    return ( (ADC_Ptr->ptr_ADCxr->SR & ADC_SR_EOC) ? 1U : 0U );
}

uint16_t ADC_ReadData12(ADC_Handle_t* ADC_Ptr)
{
   
    return (uint16_t)(ADC_Ptr->ptr_ADCxr->DR & 0xFFFFU);
}
