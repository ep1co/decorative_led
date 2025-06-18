#include "stm32f4xx.h"
#include <string.h>

#define LED_RED_PIN     5
#define LED_YELLOW_PIN  6
#define LED_GREEN_PIN   7

volatile uint8_t led_mode = 0; // 0: blink, 1: steady
volatile uint8_t led_state[3] = {0, 0, 0}; // 0: off, 1: on (RED, YELLOW, GREEN)

void delay(volatile uint32_t t) { while(t--); }

void GPIO_Config(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOCEN;
    GPIOA->MODER |= (1 << (LED_RED_PIN*2)) | (1 << (LED_YELLOW_PIN*2)) | (1 << (LED_GREEN_PIN*2));
}

void UART2_Config(void) {
    GPIOA->MODER &= ~((3 << (2*2)) | (3 << (3*2)));
    GPIOA->MODER |= (2 << (2*2)) | (2 << (3*2));
    GPIOA->AFR[0] |= (7 << (2*4)) | (7 << (3*4));
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
    USART2->BRR = 0x8B; // 16MHz/115200
    USART2->CR1 |= USART_CR1_RE | USART_CR1_TE | USART_CR1_UE;
    USART2->CR1 |= USART_CR1_RXNEIE;
    NVIC_EnableIRQ(USART2_IRQn);
}

void Timer_Config(void) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    TIM2->PSC = 16000-1;
    TIM2->ARR = 200-1;
    TIM2->DIER |= TIM_DIER_UIE;
    TIM2->CR1 |= TIM_CR1_CEN;
    NVIC_EnableIRQ(TIM2_IRQn);
}

void EXTI_Config(void) {
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
    SYSCFG->EXTICR[3] |= SYSCFG_EXTICR4_EXTI13_PC;
    EXTI->IMR |= EXTI_IMR_IM13;
    EXTI->FTSR |= EXTI_FTSR_TR13;
    NVIC_EnableIRQ(EXTI15_10_IRQn);
}

void update_leds(void) {
    // RED
    if (led_mode == 0) {
        // blink mode handled in timer
    } else {
        if (led_state[0]) GPIOA->ODR |= (1 << LED_RED_PIN);
        else GPIOA->ODR &= ~(1 << LED_RED_PIN);
    }
    // YELLOW
    if (led_mode == 0) {
        // blink mode handled in timer
    } else {
        if (led_state[1]) GPIOA->ODR |= (1 << LED_YELLOW_PIN);
        else GPIOA->ODR &= ~(1 << LED_YELLOW_PIN);
    }
    // GREEN
    if (led_mode == 0) {
        // blink mode handled in timer
    } else {
        if (led_state[2]) GPIOA->ODR |= (1 << LED_GREEN_PIN);
        else GPIOA->ODR &= ~(1 << LED_GREEN_PIN);
    }
}

void USART2_IRQHandler(void) {
    if(USART2->SR & USART_SR_RXNE) {
        static char buf[8];
        static uint8_t idx = 0;
        char c = USART2->DR;
        if(c == '\r' || c == '\n') {
            buf[idx] = 0;
            if(strncmp(buf, "123R", 4) == 0) {
                led_state[0] = !led_state[0]; // RED
            } else if(strncmp(buf, "123Y", 4) == 0) {
                led_state[1] = !led_state[1]; // YELLOW
            } else if(strncmp(buf, "123B", 4) == 0) {
                led_state[2] = !led_state[2]; // GREEN
            }
            update_leds();
            idx = 0;
        } else if(idx < 7) {
            buf[idx++] = c;
        }
    }
}

void TIM2_IRQHandler(void) {
    if(TIM2->SR & TIM_SR_UIF) {
        TIM2->SR &= ~TIM_SR_UIF;
        if(led_mode == 0) { // blink mode
            static uint8_t blink = 0;
            blink = !blink;
            // RED
            if (led_state[0]) {
                if (blink) GPIOA->ODR |= (1 << LED_RED_PIN);
                else GPIOA->ODR &= ~(1 << LED_RED_PIN);
            } else {
                GPIOA->ODR &= ~(1 << LED_RED_PIN);
            }
            // YELLOW
            if (led_state[1]) {
                if (blink) GPIOA->ODR |= (1 << LED_YELLOW_PIN);
                else GPIOA->ODR &= ~(1 << LED_YELLOW_PIN);
            } else {
                GPIOA->ODR &= ~(1 << LED_YELLOW_PIN);
            }
            // GREEN
            if (led_state[2]) {
                if (blink) GPIOA->ODR |= (1 << LED_GREEN_PIN);
                else GPIOA->ODR &= ~(1 << LED_GREEN_PIN);
            } else {
                GPIOA->ODR &= ~(1 << LED_GREEN_PIN);
            }
        }
    }
}

void EXTI15_10_IRQHandler(void) {
    if(EXTI->PR & EXTI_PR_PR13) {
        EXTI->PR = EXTI_PR_PR13;
        led_mode = !led_mode;
        update_leds();
    }
}

int main(void) {
    GPIO_Config();
    UART2_Config();
    Timer_Config();
    EXTI_Config();
    while(1) {}
}
