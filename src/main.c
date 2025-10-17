// ****************************************************************
// * TEAM22: H. MOBLEY and J. TINGEY
// * CPEG222 Proj3A, 10/17/25
//
// ****************************************************************
#include "stm32f4xx.h"
#include "SSD_Array.h"
#include <stdio.h>

#define TRIG_PIN  4      // PA4  – Trigger
#define ECHO_PIN  0      // PB0  – Echo (TIM3_CH3)
#define BTN_PIN   13     // PC13 – User Button
#define TRIG_PORT GPIOA
#define ECHO_PORT GPIOB
#define BTN_PORT  GPIOC

volatile uint32_t pulse_start = 0;
volatile uint32_t pulse_end = 0;
volatile uint32_t pulse_width_us = 0;
volatile uint8_t  edge_state = 0;
volatile uint8_t update_flag = 0;
volatile uint8_t display_inch = 0;
volatile uint8_t current_digit = 0;
volatile float    last_distance_cm = 0.0f;
volatile uint32_t systick_count = 0;

void GPIO_Init(void);
void TIM2_Init(void);
void TIM3_InputCapture_Init(void);
void USART2_Init(void);
void trigger_pulse(void);
void delay_us(uint32_t us);
void send_string(const char *s);

int main(void) {
    SystemCoreClockUpdate();
    GPIO_Init();
    USART2_Init();
    SSD_init();
    TIM3_InputCapture_Init();
    TIM2_Init();
    SysTick_Config(SystemCoreClock / 2U);   // 0.5 s

    while (1) {
        if (update_flag) {
            update_flag = 0;
            edge_state = 0;
            pulse_width_us = 0;
            trigger_pulse();

            for (volatile uint32_t i=0;i<50000;i++){
                if (pulse_width_us>0) break;
                delay_us(10);
            }

            if (pulse_width_us>100 && pulse_width_us<25000) {
                float d = (float)pulse_width_us / 58.0f;
                if (d>=2.0f && d<=400.0f) last_distance_cm = d;
            }

            float val = display_inch ? last_distance_cm/2.54f : last_distance_cm;
            if (val>99.99f) val = 99.99f;

            char msg[64];
            sprintf(msg,"Distance: %.2f %s\r\n", val, display_inch?"in":"cm", pulse_width_us);
            send_string(msg);
        }
    }
}

void GPIO_Init(void){
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN|RCC_AHB1ENR_GPIOBEN|RCC_AHB1ENR_GPIOCEN;
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

    // Trigger
    TRIG_PORT->MODER &= ~(3u<<(TRIG_PIN*2));
    TRIG_PORT->MODER |=  (1u<<(TRIG_PIN*2));
    TRIG_PORT->ODR   &= ~(1u<<TRIG_PIN);

    // Button PC13
    BTN_PORT->MODER &= ~(3u<<(BTN_PIN*2));
    BTN_PORT->PUPDR &= ~(3u<<(BTN_PIN*2));
    BTN_PORT->PUPDR |=  (1u<<(BTN_PIN*2));
    SYSCFG->EXTICR[3] = SYSCFG_EXTICR4_EXTI13_PC;
    EXTI->IMR  |= EXTI_IMR_IM13;
    EXTI->FTSR |= EXTI_FTSR_TR13;
    NVIC_EnableIRQ(EXTI15_10_IRQn);
}

void TIM2_Init(void){
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    TIM2->PSC = 16-1; TIM2->ARR = 500-1;
    TIM2->DIER |= TIM_DIER_UIE;
    NVIC_EnableIRQ(TIM2_IRQn);
    TIM2->CR1 |= TIM_CR1_CEN;
}

void TIM3_InputCapture_Init(void){
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

    GPIOB->MODER &= ~(3u<<(ECHO_PIN*2));
    GPIOB->MODER |=  (2u<<(ECHO_PIN*2));
    GPIOB->AFR[0] &= ~(0xF<<(ECHO_PIN*4));
    GPIOB->AFR[0] |=  (0x2<<(ECHO_PIN*4));  // AF2 TIM3_CH3

    TIM3->PSC = 16-1; TIM3->ARR = 0xFFFF;
    TIM3->CCMR2 &= ~TIM_CCMR2_CC3S;
    TIM3->CCMR2 |=  TIM_CCMR2_CC3S_0;
    TIM3->CCER  &= ~TIM_CCER_CC3P;
    TIM3->CCER  |=  TIM_CCER_CC3E;
    TIM3->DIER  |=  TIM_DIER_CC3IE;
    NVIC_EnableIRQ(TIM3_IRQn);
    TIM3->CR1 |= TIM_CR1_CEN;
}
void TIM2_IRQHandler(void)
{
    if (TIM2->SR & TIM_SR_UIF)
    {
        TIM2->SR &= ~TIM_SR_UIF;
        float val = display_inch ? (last_distance_cm / 2.54f) : last_distance_cm;
        if (val > 99.99f) val = 99.99f;

        int display_value = (int)(val * 100.0f);

        SSD_update(current_digit, display_value, 2);
        current_digit = (current_digit + 1) % 4;
    }
}

void TIM3_IRQHandler(void){
    if (TIM3->SR & TIM_SR_CC3IF){
        if (edge_state==0){
            pulse_start = TIM3->CCR3; edge_state=1;
            TIM3->CCER |= TIM_CCER_CC3P;
        } else {
            pulse_end = TIM3->CCR3; edge_state=0;
            pulse_width_us = (pulse_end>=pulse_start)?
                             (pulse_end-pulse_start):
                             ((0xFFFF-pulse_start)+pulse_end);
            TIM3->CCER &= ~TIM_CCER_CC3P;
        }
        TIM3->SR &= ~TIM_SR_CC3IF;
    }
}

void SysTick_Handler(void){
    systick_count++;
    if (systick_count>=1){ systick_count=0; update_flag=1; }
}

void EXTI15_10_IRQHandler(void){
    if (EXTI->PR & EXTI_PR_PR13){
        display_inch ^= 1;
        EXTI->PR = EXTI_PR_PR13;
    }
}

void USART2_Init(void){
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    GPIOA->MODER &= ~(0xF<<(2*2));
    GPIOA->MODER |=  (0xA<<(2*2));
    GPIOA->AFR[0] |=  (0x7700u);
    USART2->BRR = SystemCoreClock/115200;
    USART2->CR1 = USART_CR1_TE|USART_CR1_UE;
}

void send_string(const char *s){
    while(*s){ while(!(USART2->SR & USART_SR_TXE)); USART2->DR=*s++; }
}

void trigger_pulse(void){
    TRIG_PORT->ODR |= (1<<TRIG_PIN);
    delay_us(15);
    TRIG_PORT->ODR &= ~(1<<TRIG_PIN);
}

void delay_us(uint32_t us){
    us *= (SystemCoreClock/1000000)/5;
    while(us--) __NOP();
}