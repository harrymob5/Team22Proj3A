// ****************************************************************
// * TEAM22: H. MOBLEY and J. TINGEY
// * CPEG222 Proj3A, 10/17/25
//
// ****************************************************************
#include "stm32f4xx.h"
#include "SSD_Array.h"
#include <stdio.h>

#define trig_pin   4      // PA4  - Trigger
#define echo_pin   0     // PB0 - Echo (TIM3_CH3)
#define trig_port  GPIOA
#define echo_port  GPIOB

#define btn_pin    13     // PC13 - User Button
#define btn_port   GPIOC

volatile uint32_t pulse_start = 0;
volatile uint32_t pulse_end = 0;
volatile uint32_t pulse_width_us = 0;
volatile uint8_t  edge_state = 0;

volatile uint8_t  update_flag = 0;       // SysTick 0.5s flag
volatile uint8_t  display_inch = 0;      // 0 = cm, 1 = inch
volatile float    last_distance_cm = 0;  // Latest cm reading
volatile uint8_t  current_digit = 0;     // SSD multiplex index

void GPIO_Init(void);
void TIM2_Init(void);
void TIM3_Init(void);
void USART2_Init(void);
void SysTick_Handler(void);
void TIM2_IRQHandler(void);
void TIM3_IRQHandler(void);
void EXTI15_10_IRQHandler(void);

void trigger_pulse(void);
void delay_us(uint32_t us);
void send_string(const char *s);