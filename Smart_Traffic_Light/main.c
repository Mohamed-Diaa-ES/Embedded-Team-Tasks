#include <stdint.h>
#define RCC_AHB1ENR   (*((volatile uint32_t*)0x40023830)) //Enabler register for AHB
#define GPIOA_MODER   (*((volatile uint32_t*)0x40020000)) //for changing modes
#define GPIOA_ODR     (*((volatile uint32_t*)0x40020014)) //GPIO output data register
#define GPIOB_MODER   (*((volatile uint32_t*)0x40020400)) //GPIOB moder
#define GPIOB_IDR     (*((volatile uint32_t*)0x40020410)) //IDR input Data register

/**
pin places as said in the task
 */
#define RED_LED_PIN     5
#define YELLOW_LED_PIN  6
#define GREEN_LED_PIN   7
#define BUTTON_PIN      0

void delay_ms(uint32_t ms);

int main(void) {
   
    RCC_AHB1ENR |= (1 << 0); 
   
    RCC_AHB1ENR |= (1 << 1); 


    GPIOA_MODER &= ~(0x3 << (2 * RED_LED_PIN));     
    GPIOA_MODER |=  (0x1 << (2 * RED_LED_PIN));     

    GPIOA_MODER &= ~(0x3 << (2 * YELLOW_LED_PIN));
    GPIOA_MODER |=  (0x1 << (2 * YELLOW_LED_PIN));

    GPIOA_MODER &= ~(0x3 << (2 * GREEN_LED_PIN));
    GPIOA_MODER |=  (0x1 << (2 * GREEN_LED_PIN));

   
    GPIOB_MODER &= ~(0x3 << (2 * BUTTON_PIN)); 

    while (1) {
        
        GPIOA_ODR |=  (1 << GREEN_LED_PIN);
        GPIOA_ODR &= ~(1 << YELLOW_LED_PIN);
        GPIOA_ODR &= ~(1 << RED_LED_PIN);

       
        if (GPIOB_IDR & (1 << BUTTON_PIN)) {
           
            GPIOA_ODR |=  (1 << YELLOW_LED_PIN);
            GPIOA_ODR &= ~(1 << GREEN_LED_PIN);
            delay_ms(2000);

           
            GPIOA_ODR |=  (1 << RED_LED_PIN);
            GPIOA_ODR &= ~(1 << YELLOW_LED_PIN);
            delay_ms(5000);
        }
    }
}
//Searched on the internet how to do the delay function
void delay_ms(uint32_t ms) {
    for (uint32_t i = 0; i < ms * 4000; i++) {
        __asm__("nop");//assembly writing in C
    }
}
