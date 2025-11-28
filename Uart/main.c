
#include "STM32F40xx.h"
#include "Stm324Fxx_GPIO_Interface.h"
#include "Stm324F0xx_ADC_Interface.h"
#include "stm32f4xx.h"
#include "stm32f4xx_ll_usart.h"     
#include "stm32f4xx_ll_gpio.h"     
#include "stm32f4xx_ll_bus.h"     
#include <stdint.h>




static inline void uart_send_byte(USART_TypeDef *USARTx, uint8_t ch)
{
    while (!LL_USART_IsActiveFlag_TXE(USARTx));// flag to be down
    LL_USART_TransmitData8(USARTx, ch);
}

static inline void uart_wait_tc(USART_TypeDef *USARTx)
{
    while (!LL_USART_IsActiveFlag_TC(USARTx)); // flag to go down 
}

//
static void uart_send_cstr(USART_TypeDef *USARTx, const char *s)
{
    if (s == NULL) return;
    while (*s) {
        uart_send_byte(USARTx, (uint8_t)*s++);
    }
    uart_wait_tc(USARTx);
}


static void uart_send_int32(USART_TypeDef *USARTx, int32_t value)
{
    if (value == 0) {
        uart_send_byte(USARTx, '0');
        uart_wait_tc(USARTx);
        return;
    }



    
if (value< 0) {
        uart_send_byte(USARTx, '-');
        value*=-1;
    }

    char buf[10];  
    int i = 0;
    while (value > 0) {
        buf[i++] = (char)('0' + (value % 10U));
        value /= 10U;
    }
    while (i-- > 0) {
        uart_send_byte(USARTx, (uint8_t)buf[i]);
    }

    uart_wait_tc(USARTx);
}

/* Optional: map '\n' to "\r\n" for terminals expecting CRLF */
static void uart_send_line(USART_TypeDef *USARTx, const char *s)
{
    if (s == NULL) return;
    while (*s) {
        char c = *s++;
        if (c == '\n') uart_send_byte(USARTx, '\r');
        uart_send_byte(USARTx, (uint8_t)c);
    }
    uart_send_byte(USARTx, '\r');
    uart_send_byte(USARTx, '\n');
    uart_wait_tc(USARTx);
}

/* ============================ Main ===================================== */

int main(void)
{
    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

    /* System interrupt init*/
    NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

    /* Configure the system clock (CubeMX generates this) */
    SystemClock_Config();

    /* Initialize all configured peripherals (CubeMX generates these) */
    MX_GPIO_Init();
    MX_USART2_UART_Init();          // Uses LL to configure USART2

    /* ---- Demo: use the helpers ---- */
    uart_send_line(USART2, "LL UART demo");
    uart_send_cstr(USART2, "Value = ");
    uart_send_int32(USART2, -123456789);
    uart_send_cstr(USART2, "\r\n");
    uart_send_int32(USART2, (int32_t)-2147483648);   // test INT_MIN
    uart_send_cstr(USART2, "\r\n");

    while (1) {
        /* your application loop */
    }
}

/* CubeMX generates SystemClock_Config(), Error_Handler(), assert handlers here */
