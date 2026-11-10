/* Minimal STM32F411 register map (only what this firmware uses), written from RM0383.
 * Kept local so the build needs no vendor SDK. */
#ifndef AR_REG_H
#define AR_REG_H
#include <stdint.h>
#define REG(addr) (*(volatile uint32_t *)(addr))

typedef struct { volatile uint32_t MODER, OTYPER, OSPEEDR, PUPDR, IDR, ODR, BSRR, LCKR, AFR[2]; } gpio_t;
#define GPIOA ((gpio_t *)0x40020000u)
#define GPIOB ((gpio_t *)0x40020400u)
#define GPIOC ((gpio_t *)0x40020800u)

#define RCC_BASE 0x40023800u
#define RCC_CR REG(RCC_BASE + 0x00)
#define RCC_AHB1ENR REG(RCC_BASE + 0x30)
#define RCC_APB1ENR REG(RCC_BASE + 0x40)
#define RCC_APB2ENR REG(RCC_BASE + 0x44)

typedef struct {
    volatile uint32_t CR1, CR2, SMCR, DIER, SR, EGR, CCMR1, CCMR2, CCER, CNT, PSC, ARR, RCR, CCR1, CCR2, CCR3, CCR4, BDTR;
} tim_t;
#define TIM2 ((tim_t *)0x40000000u)
#define TIM3 ((tim_t *)0x40000400u)
#define TIM4 ((tim_t *)0x40000800u)

typedef struct { volatile uint32_t SR, DR, BRR, CR1, CR2, CR3, GTPR; } usart_t;
#define USART2 ((usart_t *)0x40004400u)
#define USART_SR_RXNE (1u << 5)
#define USART_SR_TXE (1u << 7)
#define USART_SR_ORE (1u << 3)

typedef struct { volatile uint32_t CR1, CR2, OAR1, OAR2, DR, SR1, SR2, CCR, TRISE; } i2c_t;
#define I2C3 ((i2c_t *)0x40005C00u)

typedef struct {
    volatile uint32_t SR, CR1, CR2, SMPR1, SMPR2, JOFR[4], HTR, LTR, SQR1, SQR2, SQR3, JSQR, JDR[4], DR;
} adc_t;
#define ADC1 ((adc_t *)0x40012000u)

#define IWDG_KR REG(0x40003000u)
#define IWDG_PR REG(0x40003004u)
#define IWDG_RLR REG(0x40003008u)

#define SYST_CSR REG(0xE000E010u)
#define SYST_RVR REG(0xE000E014u)
#define SYST_CVR REG(0xE000E018u)
#define NVIC_ISER(n) REG(0xE000E100u + 4u * (n))
#define SCB_CPACR REG(0xE000ED88u)

#define F_CPU 16000000u /* HSI, no PLL: ample for a 100 Hz loop and 20 kHz PWM */
#endif
