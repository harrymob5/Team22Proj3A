// ****************************************************************
// * TEAM22: H. MOBLEY and J. TINGEY
// * CPEG222 Proj3B, 10/24/25
//
// ****************************************************************
#include "stm32f4xx.h"
#include "SSD_Array.h"
#include <stdio.h>
#include <stdint.h>
#include <math.h>

#define TRIG_PIN    4       // PA4
#define ECHO_PIN    0       // PB0 (TIM3_CH3)
#define SERVO_PIN   6       // PC6 (TIM3_CH1)
#define BTN_PIN     13      // PC13 - User Button
#define TRIG_PORT   GPIOA
#define ECHO_PORT   GPIOB
#define SERVO_PORT  GPIOC
#define BTN_PORT    GPIOC


volatile uint32_t pulse_start = 0; 
volatile uint32_t pulse_end = 0;
volatile uint32_t pulse_width_us = 0;
volatile uint8_t edge_state = 0;
volatile uint8_t display_inch = 0;
volatile uint8_t current_digit = 0;
volatile float last_distance_cm = 0.0f;

void GPIO_Init(void);
void TIM2_Init(void);
void TIM3_PWM_and_IC_Init(void);
void USART2_Init(void);
void trigger_pulse(void);
void delay_us(uint32_t us);
void delay_ms(uint32_t ms);
void send_string(const char *s);
static inline uint32_t angle_to_pulse_us(int angle_deg);

int main(void){
    SystemCoreClockUpdate();
    GPIO_Init();
    USART2_Init();
    SSD_init();

    TIM3_PWM_and_IC_Init();
    TIM2_Init();
    SysTick_Config(SystemCoreClock / 2U); // 0.5 ms

    const int step_deg = 5;
    const int angle_min = -45;
    const int angle_max = 45;
    int angle = angle_min, dir = 1;

    while (1)
    {
        // TODO: set servo position using angle_to_pulse_us(angle)
        delay_ms(250); // give servo time to move

        edge_state = 0;
        pulse_width_us = 0;
        trigger_pulse();

        while (pulse_width_us == 0) delay_us(1);

    float raw_cm = (pulse_width_us > 0) ? ((float)pulse_width_us / 58.0f) : 0.0f;

    float display_val = display_inch ? (raw_cm / 2.54f) : raw_cm;
    if (display_val > 99.99f) display_val = 99.99f;

    if (display_inch) {
        last_distance_cm = display_val * 2.54f; 
    } else {
        last_distance_cm = display_val;         
    }

    char out[96];
    if (display_inch) {
        sprintf(out, "Angle: %d deg, Pulse: %lu us, Range: %.2f in\r\n",
                angle, (unsigned long)pulse_width_us, display_val);
    } else {
        sprintf(out, "Angle: %d deg, Pulse: %lu us, Range: %.2f cm\r\n",
                angle, (unsigned long)pulse_width_us, display_val);
    }
    send_string(out);

        // TODO: implement servo sweep logic
    }
}

void GPIO_Init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN;
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

    // Trigger (PA4)
    TRIG_PORT->MODER &= ~(3u << (TRIG_PIN*2));
    TRIG_PORT->MODER |=  (1u << (TRIG_PIN*2));
    TRIG_PORT->ODR   &= ~(1u << TRIG_PIN);

    // Button (PC13)
    BTN_PORT->MODER &= ~(3u << (BTN_PIN * 2));
    BTN_PORT->PUPDR &= ~(3u << (BTN_PIN * 2));
    BTN_PORT->PUPDR |=  (1u << (BTN_PIN * 2));
    SYSCFG->EXTICR[3] |=  SYSCFG_EXTICR4_EXTI13_PC;
    EXTI->IMR |= EXTI_IMR_IM13;
    EXTI->FTSR |= EXTI_FTSR_TR13;
    NVIC_EnableIRQ(EXTI15_10_IRQn);
}

void TIM2_Init(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    TIM2->PSC = 16 - 1;
    TIM2->ARR = 500 - 1;
    TIM2->DIER |= TIM_DIER_UIE;
    NVIC_EnableIRQ(TIM2_IRQn);
    TIM2->CR1 |= TIM_CR1_CEN;
}

void TIM3_PWM_and_IC_Init(void){
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN;
    // PC6 -> TIM3_CH1 (AF2) for servo PWM
    SERVO_PORT->MODER &= ~(3u << (SERVO_PIN * 2));
    SERVO_PORT->MODER |=  (2u << (SERVO_PIN * 2));
    SERVO_PORT->AFR[0] &= ~(0xFu << (SERVO_PIN * 4));
    SERVO_PORT->AFR[0] |=  (0x2u << (SERVO_PIN * 4));
    SERVO_PORT->OTYPER &= ~(1 << SERVO_PIN);
    SERVO_PORT->OSPEEDR |= (0x3 << (SERVO_PIN * 2));
    SERVO_PORT->PUPDR &= ~(0x3 << (SERVO_PIN * 2));
    // PB0 -> TIM3_CH3 (AF2) for echo capture
    ECHO_PORT->MODER &= ~(3u << (ECHO_PIN * 2));
    ECHO_PORT->MODER |=  (2u << (ECHO_PIN * 2));
    ECHO_PORT->AFR[0] &= ~(0xFu << (ECHO_PIN * 4));
    ECHO_PORT->AFR[0] |=  (0x2u << (ECHO_PIN * 4));
    TIM3->PSC = (SystemCoreClock / 1000000U) - 1; // 1 MHz
    TIM3->ARR = 19999; // 20ms period (50Hz)
    // CH1: PWM output
    TIM3->CCMR1 &= ~TIM_CCMR1_OC1M;
    TIM3->CCMR1 |= (6 << TIM_CCMR1_OC1M_Pos);
    TIM3->CCMR1 |= TIM_CCMR1_OC1PE;
    TIM3->CCER |= TIM_CCER_CC1E;
    // CH3: input capture
    TIM3->CCMR2 &= ~TIM_CCMR2_CC3S;
    TIM3->CCMR2 |= TIM_CCMR2_CC3S_0;
    TIM3->CCER &= ~TIM_CCER_CC3P;
    TIM3->CCER |= TIM_CCER_CC3E;
    TIM3->DIER |= TIM_DIER_CC3IE;
    NVIC_EnableIRQ(TIM3_IRQn);
    TIM3->CR1 |= TIM_CR1_ARPE;
    TIM3->EGR = TIM_EGR_UG;
    TIM3->CR1 |= TIM_CR1_CEN;
}

void SysTick_Handler(void) {}

void TIM2_IRQHandler(void){
    if (TIM2->SR & TIM_SR_UIF) {
        TIM2->SR &= ~TIM_SR_UIF;
        float val = display_inch ? (last_distance_cm / 2.54f) : last_distance_cm;
        if (val > 99.99f) val = 99.99f;

        int display_value = (int)(val * 100.0f + 0.5f);
        SSD_update(current_digit, display_value, 2);
        current_digit = (current_digit + 1) % 4;
    }
}

void TIM3_IRQHandler(void){
    if (TIM3->SR & TIM_SR_CC3IF){
        if (edge_state == 0) {
            pulse_start = TIM3->CCR3;
            edge_state = 1;
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

void EXTI15_10_IRQHandler(void){
    if (EXTI->PR & EXTI_PR_PR13) {
        display_inch ^= 1;
        EXTI->PR = EXTI_PR_PR13;
    }
}

void USART2_Init(void){
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    GPIOA->MODER &= ~(0xF << (2*2));
    GPIOA->MODER |=  (0xA << (2*2));
    GPIOA->AFR[0] |= (0x7700u);
    USART2->BRR = SystemCoreClock / 115200;
    USART2->CR1 = USART_CR1_TE | USART_CR1_UE;
}

void send_string(const char *s){
    while (*s) {
        while (!(USART2->SR & USART_SR_TXE));
        USART2->DR = *s++;
    }
}

void trigger_pulse(void){
    TRIG_PORT->BSRR = (1u << TRIG_PIN);
    delay_us(15);
    TRIG_PORT->BSRR = (1u << (TRIG_PIN + 16)); // drive low
}

void delay_us(uint32_t us){
    us *= (SystemCoreClock/1000000)/5;
    while (us--) __NOP();
}

void delay_ms(uint32_t ms){
    while (ms--) delay_us(1000);
}
