
#define RCC_AHB1ENR   (*((volatile unsigned int*)0x40023830))
#define GPIOA_MODER   (*((volatile unsigned int*)0x40020000))
#define GPIOA_ODR     (*((volatile unsigned int*)0x40020014))
#define GPIOB_MODER   (*((volatile unsigned int*)0x40020400))
#define GPIOB_IDR     (*((volatile unsigned int*)0x40020410))

#define RED_LED_PIN     5
#define YELLOW_LED_PIN  6
#define GREEN_LED_PIN   7
#define BUTTON_PIN      0

void delay_ms(unsigned int ms);

int main(void) {
    // 1. Enable GPIOA and GPIOB clocks
    RCC_AHB1ENR |= (1 << 0); // GPIOA
    RCC_AHB1ENR |= (1 << 1); // GPIOB

    // 2. Set PA5, PA6, PA7 as output (MODER = 01)
    GPIOA_MODER &= ~(0x3 << (2 * RED_LED_PIN));     // Clear bits
    GPIOA_MODER |=  (0x1 << (2 * RED_LED_PIN));     // Set as output

    GPIOA_MODER &= ~(0x3 << (2 * YELLOW_LED_PIN));
    GPIOA_MODER |=  (0x1 << (2 * YELLOW_LED_PIN));

    GPIOA_MODER &= ~(0x3 << (2 * GREEN_LED_PIN));
    GPIOA_MODER |=  (0x1 << (2 * GREEN_LED_PIN));

    // 3. Set PB0 as input (MODER = 00)
    GPIOB_MODER &= ~(0x3 << (2 * BUTTON_PIN)); // Input mode

    while (1) {
        // Default: Green ON
        GPIOA_ODR |=  (1 << GREEN_LED_PIN);
        GPIOA_ODR &= ~(1 << YELLOW_LED_PIN);
        GPIOA_ODR &= ~(1 << RED_LED_PIN);

        // Check if button is pressed
        if (GPIOB_IDR & (1 << BUTTON_PIN)) {
            // Yellow ON for 2 seconds
            GPIOA_ODR |=  (1 << YELLOW_LED_PIN);
            GPIOA_ODR &= ~(1 << GREEN_LED_PIN);
            delay_ms(2000);

            // Red ON for 5 seconds
            GPIOA_ODR |=  (1 << RED_LED_PIN);
            GPIOA_ODR &= ~(1 << YELLOW_LED_PIN);
            delay_ms(5000);
        }
    }
}

void delay_ms(unsigned int ms) {
    for (unsigned int i = 0; i < ms * 4000; i++) {
        __asm__("nop");
    }
}
