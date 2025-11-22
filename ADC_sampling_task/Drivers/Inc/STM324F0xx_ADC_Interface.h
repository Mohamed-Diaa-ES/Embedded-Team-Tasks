
#ifndef INC_STM324FXX_ADC_INTERFACE_H_
#define INC_STM324FXX_ADC_INTERFACE_H_

#include "STM32F40xx.h"
#include <stdint.h>

/* ===================== ADC register maps (STM32F4 style) ===================== */
typedef struct
{
    volatile uint32_t SR;     /* Status register */
    volatile uint32_t CR1;    /* Control register 1 */
    volatile uint32_t CR2;    /* Control register 2 */
    volatile uint32_t SMPR1;  /* Sample time register 1 (ch 10..18) */
    volatile uint32_t SMPR2;  /* Sample time register 2 (ch 0..9)  */
    volatile uint32_t JOFR1;  /* Injected offset 1 */
    volatile uint32_t JOFR2;  /* Injected offset 2 */
    volatile uint32_t JOFR3;  /* Injected offset 3 */
    volatile uint32_t JOFR4;  /* Injected offset 4 */
    volatile uint32_t HTR;    /* Analog watchdog high threshold */
    volatile uint32_t LTR;    /* Analog watchdog low threshold */
    volatile uint32_t SQR1;   /* Regular sequence register 1 */
    volatile uint32_t SQR2;   /* Regular sequence register 2 */
    volatile uint32_t SQR3;   /* Regular sequence register 3 */
    volatile uint32_t JSQR;   /* Injected sequence register */
    volatile uint32_t JDR1;   /* Injected data 1 */
    volatile uint32_t JDR2;   /* Injected data 2 */
    volatile uint32_t JDR3;   /* Injected data 3 */
    volatile uint32_t JDR4;   /* Injected data 4 */
    volatile uint32_t DR;     /* Regular data register */
} ADC_Reg_Def_t;

typedef struct
{
    volatile uint32_t CSR;    /* Common status */
    volatile uint32_t CCR;    /* Common control: prescaler, etc. */
    volatile uint32_t CDR;    /* Data register (dual/triple mode) */
} ADC_Common_Reg_Def_t;

/* Base addresses (F4 family) */
#define ADC1_BaseAddress        (APB2Periph_Base + 0x2000U) /* 0x40012000 */
#define ADC_Common_BaseAddress  (APB2Periph_Base + 0x2300U) /* 0x40012300 */

#define ADC1        ((ADC_Reg_Def_t*)ADC1_BaseAddress)
#define ADC_Common  ((ADC_Common_Reg_Def_t*)ADC_Common_BaseAddress)

/* ===================== Clock enable/disable (APB2ENR bit 8: ADC1EN) ===================== */
#define ADC1_PClk_EN()   (RCC_Reg->RCC_APB2ENR |= (1U << 8))
#define ADC1_PClk_DI()   (RCC_Reg->RCC_APB2ENR &= ~(1U << 8))

/* ===================== Useful bitfields for CR1/CR2 ===================== */
/* CR1: resolution (RES[25:24]) */
#define ADC_RES_12B   (0U << 24)
#define ADC_RES_10B   (1U << 24)
#define ADC_RES_8B    (2U << 24)
#define ADC_RES_6B    (3U << 24)

/* CR2: alignment, continuous mode, ADON, SWSTART */
#define ADC_ALIGN_LEFT    (1U << 11)
#define ADC_ALIGN_RIGHT   (0U)
#define ADC_CONT          (1U << 1)
#define ADC_SINGLE        (0U)
#define ADC_ADON          (1U << 0)
#define ADC_SWSTART       (1U << 30)
#define ADC_EOCS          (1U << 10)  /* EOC at end of each conversion */

/* SR: End of conversion flag */
#define ADC_SR_EOC        (1U << 1)

/* Common prescaler in ADC_Common->CCR (bits 17:16) */
#define ADC_CCR_PSC_DIV2  (0U << 16)
#define ADC_CCR_PSC_DIV4  (1U << 16)
#define ADC_CCR_PSC_DIV6  (2U << 16)
#define ADC_CCR_PSC_DIV8  (3U << 16)

/* ===================== Sampling time choices (3-bit fields) ===================== */
/* STM32F4 sampling cycles: 3,15,28,56,84,112,144,480 */
#define ADC_SMP_3CYCLES     (0U)
#define ADC_SMP_15CYCLES    (1U)
#define ADC_SMP_28CYCLES    (2U)
#define ADC_SMP_56CYCLES    (3U)
#define ADC_SMP_84CYCLES    (4U)
#define ADC_SMP_112CYCLES   (5U)
#define ADC_SMP_144CYCLES   (6U)
#define ADC_SMP_480CYCLES   (7U)

/* Helpers to position sampling bits in SMPRx */
#define ADC_SMPR2_POS(ch)   ((ch) * 3U)        /* ch 0..9 */
#define ADC_SMPR1_POS(ch)   (((ch) - 10U) * 3U)/* ch 10..18 */

/* ===================== Handle & Config (matches your style) ===================== */
typedef struct {
    uint8_t  Channel;        /* 0..18 */
    uint8_t  Resolution;     /* ADC_RES_* */
    uint8_t  Alignment;      /* ADC_ALIGN_RIGHT / ADC_ALIGN_LEFT */
    uint8_t  Continuous;     /* ADC_SINGLE / ADC_CONT */
    uint8_t  SamplingTime;   /* ADC_SMP_* */
} ADC_Config_t;

typedef struct {
    ADC_Reg_Def_t*       ptr_ADCxr;    /* ADC1 */
    ADC_Common_Reg_Def_t*ptr_ADCcommon;/* ADC_Common */
    ADC_Config_t         ADC_Config;
} ADC_Handle_t;

/* ===================== API ===================== */
void   ADC_PeriClockControl(ADC_Handle_t* ADC_Ptr, uint8_t En_Dis);
void   ADC_Init(ADC_Handle_t* ADC_Ptr);
void   ADC_DeInit(ADC_Handle_t* ADC_Ptr);

void   ADC_SetChannel(ADC_Handle_t* ADC_Ptr, uint8_t channel);
void   ADC_SetSamplingTime(ADC_Handle_t* ADC_Ptr, uint8_t channel, uint8_t smp);

void   ADC_StartConversion(ADC_Handle_t* ADC_Ptr);
uint8_t ADC_IsConversionDone(ADC_Handle_t* ADC_Ptr);
uint16_t ADC_ReadData12(ADC_Handle_t* ADC_Ptr);

/* Helper: raw->voltage for 12-bit */
static inline float ADC_RawToVoltage(uint16_t raw, float vref)
{ return (raw * vref) / 4095.0f; }

#endif /* INC_STM324FXX_ADC_INTERFACE_H_ */
