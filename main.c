#include "stm32f3xx.h"
#include "main.h"
#include "string.h"

void clock_init(void);
static void gpio_init(void);
static void timer1_init(void);
static void timer2_init(void);
static void timer3_init(void);

uint8_t uartBuffer[4];
volatile uint8_t uartIndex = 0;
volatile uint8_t uartDataReady = 0;

volatile uint8_t flag_exti1     = 0;
volatile uint8_t flag_Timer[3]  = {0, 0, 0};
volatile uint8_t toggleLed[3]   = {0, 0, 0};
volatile uint8_t ledState       = 0;

#define button_pressed 	0
#define button_0 		5
#define LED_1 			12
#define LED_2 			11
#define LED_3			12

void uart1_init(void)
{
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;

    // 2. PA9 (TX) as Alternate function output push-pull 50Mhz
    GPIOA->MODER &= ~GPIO_MODER_MODER9_Msk;              // Xoá MODER9
    GPIOA->MODER |=  GPIO_MODER_MODER9_1;               // MODE9 = '10' (AF mode)
    GPIOA->OTYPER &= ~GPIO_OTYPER_OT_9;                 // Push Pull
    GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR9_Msk;       // Tốc độ cao
    GPIOA->PUPDR &= ~GPIO_PUPDR_PUPDR9_Msk;             // No pull
    // Chọn AF7: viết vào AFR[1], bit (9-8)*4 = 4
    GPIOA->AFR[1] &= ~(0xF << ((9 - 8) * 4));
    GPIOA->AFR[1] |=  (7 << ((9 - 8) * 4));


	 // 3. Cấu hình PA10 (RX) là input floating
    GPIOA->MODER &= ~GPIO_MODER_MODER10_Msk;
    GPIOA->MODER |=  GPIO_MODER_MODER10_1;              // MODE10 = '10' (AF mode)
    GPIOA->OTYPER &= ~GPIO_OTYPER_OT_10;                // Push‑Pull (mặc định)
    GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR10_Msk;      // Tốc độ cao
    GPIOA->PUPDR &= ~GPIO_PUPDR_PUPDR10_Msk;            // No pull
    // AF7: viết vào AFR[1], vị trí (10-8)*4 = 8
    GPIOA->AFR[1] &= ~(0xF << ((10 - 8) * 4));
    GPIOA->AFR[1] |=  (7 << ((10 - 8) * 4));

	// 4. Cấu hình USART1
	USART1->CR1 = 0;       // Reset CR1
	USART1->CR2 = 0;       // Reset CR2
	USART1->CR3 = 0;       // Reset CR3

	 // 5. Baud rate = 115200, assuming PCLK2 = 72 MHz
	    USART1->BRR = (39 << 4) | (1 & 0xF);

	    // 6. Enable transmitter and receiver, enable USART
	    USART1->CR1 |= USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;

	    // 7. Cho phép ngắt khi có dữ liệu nhận
	    USART1->CR1 |= USART_CR1_RXNEIE;

	    // 8. Bật ngắt USART1 trong NVIC, ưu tiên ngắt
	    NVIC_EnableIRQ(USART1_IRQn);
	    NVIC_SetPriority(USART1_IRQn, 1);
}

//fix
void uart_send_char(char c) {
	while (!(USART1->ISR & USART_ISR_TXE));  // Chờ thanh ghi transmit rỗng
	USART1->RDR = c;
}

void uart_send_string(char *str) {
	while (*str) {
		uart_send_char(*str++);
	}
}

	void USART1_IRQHandler(void) {
	    if (USART1->ISR & USART_ISR_RXNE) {
	        uint8_t data = USART1->RDR;  // Đọc dữ liệu (đồng thời xóa cờ RXNE)

	        uartBuffer[uartIndex++] = data;

	        if (uartIndex >= 4) {
	            uartIndex = 0;
	            uartDataReady = 1;  // Đánh dấu có dữ liệu đầy đủ 4 byte
	        }
	    }
	}

	void EXTI1_IRQHandler(void)
	{
	    if (EXTI->PR & (1 << 1)) {
	        EXTI->PR = (1 << 1);  // Xóa cờ ngắt
	        ledState ^= 1;
	        flag_exti1 = 1;
	    }
	}

	void TIM1_UP_TIM16_IRQHandler() {
	    if (TIM1->SR & TIM_SR_UIF) {
	    	flag_Timer[0] = 1;
	        TIM1->SR &= ~TIM_SR_UIF;
	    }
	}

	void TIM2_IRQHandler(void) {
	    if (TIM2->SR & TIM_SR_UIF) {
	    	flag_Timer[1] = 1;
	        TIM2->SR &= ~TIM_SR_UIF;
	    }
	}

	void TIM3_IRQHandler(void) {
	    if (TIM3->SR & TIM_SR_UIF) {
	    	flag_Timer[2] = 1;
	        TIM3->SR &= ~TIM_SR_UIF;
	    }
	}

	void toggleLED(char led)
	{
		switch(led)
		{
		case 'G':
			toggleLed[0] ^= 1;
			if(toggleLed[0])
			{
				if(!ledState) { TIM1->CR1 |= (1 << 0); }
				else { GPIOA->ODR |= (1 << 5); }
			}
			else
			{
				TIM1->CR1 &= ~(1 << 0);
				TIM1->CNT = 0;
				GPIOA->ODR &= ~(1 << 5);
			}
			break;
		case 'Y':
			toggleLed[1] ^= 1;
			if(toggleLed[1]) {
				if(!ledState) { TIM2->CR1 |= (1 << 0); }
				else { GPIOA->ODR |= (1 << 4); }
			}
			else
			{
				TIM2->CR1 &= ~(1 << 0);
				TIM2->CNT = 0;
				GPIOA->ODR &= ~(1 << 4);
			}
			break;
		case 'R':
			toggleLed[2] ^= 1;
			if(toggleLed[2])
			{
				if(!ledState) { TIM3->CR1 |= (1 << 0); }
				else { GPIOA->ODR |= (1 << 3); }
			}
			else
			{
				TIM3->CR1 &= ~(1 << 0);
				TIM3->CNT = 0;
				GPIOA->ODR &= ~(1 << 3);
			}
			break;
		default: break;
		}
	}

	void clock_init(void) {
	    // Bật HSE (High-Speed External oscillator)
	    RCC->CR |= RCC_CR_HSEON;
	    while (!(RCC->CR & RCC_CR_HSERDY)); // Chờ HSE sẵn sàng

	    // Bật FLASH Prefetch buffer và thiết lập latency
	    FLASH->ACR |= FLASH_ACR_PRFTBE;
	    FLASH->ACR &= ~FLASH_ACR_LATENCY;
	    FLASH->ACR |= FLASH_ACR_LATENCY_2;

	    // Thiết lập PLL (HSE * 9 = 72 MHz)
	    RCC->CFGR |= RCC_CFGR_PLLSRC | RCC_CFGR_PLLMUL9;

	    RCC->CR |= RCC_CR_PLLON;
	    while (!(RCC->CR & RCC_CR_PLLRDY));

	    // Chuyển SYSCLK sang PLL
	    RCC->CFGR |= RCC_CFGR_SW_PLL;
	    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);

	    // Thiết lập APB1, APB2, AHB
	    RCC->CFGR |= RCC_CFGR_HPRE_DIV1;     // AHB = SYSCLK
	    RCC->CFGR |= RCC_CFGR_PPRE1_DIV2;    // APB1 = SYSCLK/2
	    RCC->CFGR |= RCC_CFGR_PPRE2_DIV1;    // APB2 = SYSCLK
	}

	void timer1_init(void) {
	    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;

	    TIM1->PSC = 7199;     // Prescaler
	    TIM1->ARR = 1999;     // Auto-reload
	    TIM1->DIER |= TIM_DIER_UIE; // Cho phép ngắt update

	    NVIC_EnableIRQ(TIM1_UP_TIM16_IRQn);
	    NVIC_SetPriority(TIM1_UP_TIM16_IRQn, 1);
	}

	void timer2_init(void) {
	    // Bật clock TIM2 (nằm trong APB1)
	    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

	    TIM2->PSC = 7199;            // Prescaler
	    TIM2->ARR = 1999;            // Auto-reload
	    TIM2->DIER |= TIM_DIER_UIE;  // Cho phép ngắt update

	    NVIC_EnableIRQ(TIM2_IRQn);
	    NVIC_SetPriority(TIM2_IRQn, 1);
	}

	void timer3_init(void) {
	    // Bật clock TIM3 (nằm trong APB1)
	    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

	    TIM3->PSC = 7199;            // Prescaler
	    TIM3->ARR = 1999;            // Auto-reload
	    TIM3->DIER |= TIM_DIER_UIE;  // Cho phép ngắt update

	    NVIC_EnableIRQ(TIM3_IRQn);
	    NVIC_SetPriority(TIM3_IRQn, 1);
	}

void gpio_init(void) {
	// Bật clock GPIOA và GPIOB
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOBEN;

	// PA11 và PA12 làm output mode, push-pull
	GPIOA->MODER &= ~(GPIO_MODER_MODER11_Msk | GPIO_MODER_MODER12_Msk); // clear mode
	GPIOA->MODER |=  (1 << GPIO_MODER_MODER11_Pos) | (1 << GPIO_MODER_MODER12_Pos); // output mode
	GPIOA->OTYPER &= ~(GPIO_OTYPER_OT_11 | GPIO_OTYPER_OT_12); // push-pull
	GPIOA->OSPEEDR |= (3 << GPIO_OSPEEDER_OSPEEDR11_Pos) | (3 << GPIO_OSPEEDER_OSPEEDR12_Pos); // high speed

	// PB12 làm output mode, push-pull
	GPIOB->MODER &= ~(GPIO_MODER_MODER12_Msk);
	GPIOB->MODER |=  (1 << GPIO_MODER_MODER12_Pos);
	GPIOB->OTYPER &= ~(GPIO_OTYPER_OT_12);
	GPIOB->OSPEEDR |= (3 << GPIO_OSPEEDER_OSPEEDR12_Pos);

	// PA1 làm input với pull-up
	GPIOA->MODER &= ~(GPIO_MODER_MODER1_Msk); // input mode
	GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR1_Msk);
	GPIOA->PUPDR |=  (1 << GPIO_PUPDR_PUPDR1_Pos); // pull-up

	// Cấu hình ngắt EXTI1 từ PA1
	SYSCFG->EXTICR[0] &= ~SYSCFG_EXTICR1_EXTI1_Msk;
	SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI1_PA;

	EXTI->IMR  |= EXTI_IMR_MR1;      // Cho phép ngắt từ line 1
	EXTI->FTSR |= EXTI_FTSR_TR1;     // Ngắt cạnh xuống
	EXTI->RTSR &= ~EXTI_RTSR_TR1;    // Không ngắt cạnh lên

	NVIC_EnableIRQ(EXTI1_IRQn);
	NVIC_SetPriority(EXTI1_IRQn, 0);
}

//

//change pin
int main(void)
{
    ledState = 0;
    clock_init();
    gpio_init();
    uart1_init();
    timer1_init(); timer2_init(); timer3_init();

    while(1)
    {
        if(flag_exti1)
        {
            flag_exti1 = 0;
            if(ledState)
            {
                // Dừng timers, bật LEDs theo toggleLed
                TIM1->CR1 &= ~TIM_CR1_CEN;
                TIM2->CR1 &= ~TIM_CR1_CEN;
                TIM3->CR1 &= ~TIM_CR1_CEN;
                TIM1->CNT = TIM2->CNT = TIM3->CNT = 0;
                if(toggleLed[0]) GPIOA->ODR |= (1 << LED_1);
                if(toggleLed[1]) GPIOA->ODR |= (1 << LED_2);
                if(toggleLed[2]) GPIOB->ODR |= (1 << LED_3);
            } else
            {
                // Bật lại timers nếu toggleLed đang bật
                if(toggleLed[0]) TIM1->CR1 |= TIM_CR1_CEN;
                if(toggleLed[1]) TIM2->CR1 |= TIM_CR1_CEN;
                if(toggleLed[2]) TIM3->CR1 |= TIM_CR1_CEN;
            }
        }
        if(flag_Timer[0]){ flag_Timer[0] = 0; GPIOA->ODR ^= (1 << LED_1); }
        if(flag_Timer[1]){ flag_Timer[1] = 0; GPIOA->ODR ^= (1 << LED_2); }
        if(flag_Timer[2]){ flag_Timer[2] = 0; GPIOB->ODR ^= (1 << LED_3); }

        if(uartDataReady)
        {
            uartDataReady = 0;
            if(uartBuffer[0] == '1' && uartBuffer[1] == '2' && uartBuffer[2] == '3')
            { toggleLED(uartBuffer[3]); }
        }
    }
}
