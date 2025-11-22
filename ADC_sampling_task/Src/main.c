
#include "STM32F40xx.h"
#include "Stm324Fxx_GPIO_Interface.h"
#include "Stm324F0xx_ADC_Interface.h"

/* Assume LM35 on ADC channel 0 (PA0), LED on PA5 */
#define VREF_VOLTAGE     3.300f
#define Max_TempWanted   40.0f

int main(void)
{
    GPIO_Handle_t led;
    led.ptr_GPIOxr = GPIOA;
    led.GPIO_PinConfig.PinNum = GPIO_PinNum_5;
    led.GPIO_PinConfig.Mode   = GPIO_MODE_Output;
    led.GPIO_PinConfig.Speed  = Medium_speed;
    led.GPIO_PinConfig.OutputType = GPIO_Low;      
    led.GPIO_PinConfig.PullUp_PullDown = Neither_Pull_UpDown;
    GPIO_PeriClockControl(GPIOA, Enable);
    GPIO_Init(&led);
    led.ptr_GPIOxr->GPIOx_ODR &= ~(1U << led.GPIO_PinConfig.PinNum); 

    GPIO_Handle_t ain;
    ain.ptr_GPIOxr = GPIOA;
    ain.GPIO_PinConfig.PinNum = GPIO_PinNum_0;
    ain.GPIO_PinConfig.Mode   = GPIO_MODE_Analog;
    ain.GPIO_PinConfig.Speed  = Low_speed;
    ain.GPIO_PinConfig.PullUp_PullDown = Neither_Pull_UpDown;
    GPIO_Init(&ain);

    /* === ADC1 init === */
    ADC_Handle_t adc;
    adc.ptr_ADCxr     = ADC1;
    adc.ptr_ADCcommon = ADC_Common;
    adc.ADC_Config.Channel      = 0;                  
    adc.ADC_Config.Resolution   = ADC_RES_12B;        
    adc.ADC_Config.Alignment    = ADC_ALIGN_RIGHT;
    adc.ADC_Config.Continuous   = 0;                  
    adc.ADC_Config.SamplingTime = ADC_SMP_84CYCLES;   
    ADC_Init(&adc);

    while (1)
    {
        ADC_StartConversion(&adc);
        while (!ADC_IsConversionDone(&adc));
        uint16_t raw   = ADC_ReadData12(&adc);
        float volts    = ADC_RawToVoltage(raw, VREF_VOLTAGE);
        float temp_c   = volts * 100.0f;  

        if (temp_c >= Max_TempWanted)
            led.ptr_GPIOxr->GPIOx_ODR |=  (1U << led.GPIO_PinConfig.PinNum); 
        else
            led.ptr_GPIOxr->GPIOx_ODR &= ~(1U << led.GPIO_PinConfig.PinNum); 
    }
}
