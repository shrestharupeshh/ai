/* STM32F411CEU6 ("Black Pill") HAL for the AgriRover base controller. Bare metal, HSI 16 MHz.
 *
 * STATUS: cross-compiles and links; NOT yet run on hardware. Pin map = electrical/pcb/gen/design.py (MCU_NETS).
 *   PA0  BUMP (in)        PA1  MOTOR_EN (out)   PA2/PA3 USART2 TX/RX      PA4/PA5 M1/M2 current monitor (ADC1 IN4/IN5)
 *   PA6/PA7 TIM3 enc R    PA8  I2C3 SCL         PA15/PB3 TIM2 enc L       PB0  VMOT sense (ADC1 IN8)
 *   PB1  M_EN (out)       PB4  I2C3 SDA         PB5  ESTOP_SENSE (in)     PB6..PB9 TIM4 CH1..4 PWM (M1_PWM1, M1_PWM2, M2_PWM1, M2_PWM2)
 *   PB10 M_DIAG (in, active low)                PC13 on-board LED (active low)
 */
#include <math.h>
#include "../hal.h"
#include "reg.h"

#define PWM_TOP 799u /* 16 MHz / 800 = 20 kHz */
#define LEFT_SIGN (+1)
#define RIGHT_SIGN (-1) /* right motor is mirrored on the chassis */
#define OCM_MV_PER_A 500 /* TB9051FTG carrier current-monitor gain (Pololu 2997 documentation) */
#define INA219_ADDR 0x40
#define IMU_ADDR 0x6A /* LSM6DSOX */
#define SHUNT_MILLIOHM 5

static volatile uint32_t ms;
static volatile uint8_t tick_flag;
static uint8_t rxbuf[256], txbuf[256];
static volatile uint8_t rx_head, rx_tail, tx_head, tx_tail;
static int32_t enc_acc[2];
static uint16_t enc_prev[2];
static int8_t duty_sign[2];
static uint8_t imu_ok, ina_ok;
static uint16_t vbat_mv;
static int16_t ibat_ma;

void SysTick_Handler(void) {
    ms++;
    if (ms % (1000 / AR_CTRL_HZ) == 0) tick_flag = 1;
}

void USART2_IRQHandler(void) {
    uint32_t sr = USART2->SR;
    if (sr & (USART_SR_RXNE | USART_SR_ORE)) {
        uint8_t b = (uint8_t)USART2->DR;
        uint8_t nh = (uint8_t)(rx_head + 1);
        if (nh != rx_tail) {
            rxbuf[rx_head] = b;
            rx_head = nh;
        }
    }
    if ((sr & USART_SR_TXE) && (USART2->CR1 & (1u << 7))) {
        if (tx_tail != tx_head) USART2->DR = txbuf[tx_tail++];
        else USART2->CR1 &= ~(1u << 7); /* TXEIE off */
    }
}

static void pin_mode(gpio_t *g, int pin, int mode, int af, int od, int pull) {
    g->MODER = (g->MODER & ~(3u << (pin * 2))) | ((uint32_t)mode << (pin * 2));
    g->OTYPER = (g->OTYPER & ~(1u << pin)) | ((uint32_t)od << pin);
    g->PUPDR = (g->PUPDR & ~(3u << (pin * 2))) | ((uint32_t)pull << (pin * 2));
    g->AFR[pin / 8] = (g->AFR[pin / 8] & ~(0xFu << ((pin % 8) * 4))) | ((uint32_t)af << ((pin % 8) * 4));
}
enum { IN = 0, OUT = 1, AF = 2, AN = 3 };

/* ---------------------------------------------------------------- I2C3 master, blocking with timeouts */
static int i2c_wait(volatile uint32_t *reg, uint32_t mask) {
    for (uint32_t n = 0; n < 20000; n++)
        if (*reg & mask) return 1;
    I2C3->CR1 |= (1u << 9);  /* STOP */
    I2C3->SR1 = 0;           /* clear AF/BERR/ARLO */
    return 0;
}

static int i2c_write(uint8_t addr, const uint8_t *data, int n) {
    I2C3->CR1 |= (1u << 8); /* START */
    if (!i2c_wait(&I2C3->SR1, 1u << 0)) return 0;
    I2C3->DR = (uint32_t)(addr << 1);
    if (!i2c_wait(&I2C3->SR1, 1u << 1)) return 0;
    (void)I2C3->SR2;
    for (int i = 0; i < n; i++) {
        if (!i2c_wait(&I2C3->SR1, 1u << 7)) return 0;
        I2C3->DR = data[i];
    }
    if (!i2c_wait(&I2C3->SR1, 1u << 2)) return 0; /* BTF */
    I2C3->CR1 |= (1u << 9);
    return 1;
}

static int i2c_read2(uint8_t addr, uint8_t reg, uint8_t out[2]) { /* two-byte register read (RM0383 N=2 sequence) */
    I2C3->CR1 |= (1u << 8);
    if (!i2c_wait(&I2C3->SR1, 1u << 0)) return 0;
    I2C3->DR = (uint32_t)(addr << 1);
    if (!i2c_wait(&I2C3->SR1, 1u << 1)) return 0;
    (void)I2C3->SR2;
    if (!i2c_wait(&I2C3->SR1, 1u << 7)) return 0;
    I2C3->DR = reg;
    if (!i2c_wait(&I2C3->SR1, 1u << 2)) return 0;
    I2C3->CR1 |= (1u << 8) | (1u << 10) | (1u << 11); /* repeated START, ACK, POS */
    if (!i2c_wait(&I2C3->SR1, 1u << 0)) return 0;
    I2C3->DR = (uint32_t)(addr << 1) | 1u;
    if (!i2c_wait(&I2C3->SR1, 1u << 1)) return 0;
    (void)I2C3->SR2;
    I2C3->CR1 &= ~(1u << 10); /* NACK the 2nd byte */
    if (!i2c_wait(&I2C3->SR1, 1u << 2)) return 0;
    I2C3->CR1 |= (1u << 9);
    out[0] = (uint8_t)I2C3->DR;
    out[1] = (uint8_t)I2C3->DR;
    I2C3->CR1 &= ~(1u << 11);
    return 1;
}

static uint16_t adc_read(int ch) {
    ADC1->SQR3 = (uint32_t)ch;
    ADC1->CR2 |= (1u << 30); /* SWSTART */
    for (uint32_t n = 0; n < 10000 && !(ADC1->SR & 2u); n++) {}
    return (uint16_t)ADC1->DR;
}

void hal_init(void) {
    SCB_CPACR |= (0xFu << 20);                                    /* FPU on */
    RCC_AHB1ENR |= 7u;                                            /* GPIOA, B, C */
    RCC_APB1ENR |= (1u << 0) | (1u << 1) | (1u << 2) | (1u << 17) | (1u << 23); /* TIM2, TIM3, TIM4, USART2, I2C3 */
    RCC_APB2ENR |= (1u << 8);                                     /* ADC1 */
    /* outputs first, in their safe state */
    GPIOA->BSRR = (1u << (1 + 16));
    GPIOB->BSRR = (1u << (1 + 16));
    pin_mode(GPIOA, 1, OUT, 0, 0, 0);
    pin_mode(GPIOB, 1, OUT, 0, 0, 0);
    pin_mode(GPIOC, 13, OUT, 0, 0, 0);
    pin_mode(GPIOA, 0, IN, 0, 0, 1);
    pin_mode(GPIOB, 5, IN, 0, 0, 1);  /* pull-ups duplicate the on-board ones: an unplugged loop reads "open" */
    pin_mode(GPIOB, 10, IN, 0, 0, 1);
    pin_mode(GPIOA, 2, AF, 7, 0, 0);
    pin_mode(GPIOA, 3, AF, 7, 0, 1);
    pin_mode(GPIOA, 4, AN, 0, 0, 0);
    pin_mode(GPIOA, 5, AN, 0, 0, 0);
    pin_mode(GPIOB, 0, AN, 0, 0, 0);
    pin_mode(GPIOA, 6, AF, 2, 0, 1);
    pin_mode(GPIOA, 7, AF, 2, 0, 1);
    pin_mode(GPIOA, 15, AF, 1, 0, 1);
    pin_mode(GPIOB, 3, AF, 1, 0, 1);
    pin_mode(GPIOA, 8, AF, 4, 1, 0);
    pin_mode(GPIOB, 4, AF, 9, 1, 0);
    for (int p = 6; p <= 9; p++) pin_mode(GPIOB, p, AF, 2, 0, 2); /* PWM pins, pull-down while Hi-Z */
    /* encoders: quadrature x4, input filter 8 samples */
    TIM2->SMCR = 3u;
    TIM2->CCMR1 = 0x0101u | (3u << 4) | (3u << 12);
    TIM2->ARR = 0xFFFFFFFFu;
    TIM2->CR1 = 1u;
    TIM3->SMCR = 3u;
    TIM3->CCMR1 = 0x0101u | (3u << 4) | (3u << 12);
    TIM3->ARR = 0xFFFFu;
    TIM3->CR1 = 1u;
    /* PWM 20 kHz, all four channels PWM mode 1 with preload, start at 0 % */
    TIM4->PSC = 0;
    TIM4->ARR = PWM_TOP;
    TIM4->CCMR1 = (6u << 4) | (1u << 3) | (6u << 12) | (1u << 11);
    TIM4->CCMR2 = (6u << 4) | (1u << 3) | (6u << 12) | (1u << 11);
    TIM4->CCR1 = TIM4->CCR2 = TIM4->CCR3 = TIM4->CCR4 = 0;
    TIM4->CCER = 0x1111u;
    TIM4->EGR = 1u;
    TIM4->CR1 = (1u << 7) | 1u;
    /* USART2 115200 8N1 */
    USART2->BRR = (F_CPU + 115200u / 2) / 115200u;
    USART2->CR1 = (1u << 13) | (1u << 3) | (1u << 2) | (1u << 5);
    NVIC_ISER(1) = (1u << (38 - 32));
    /* ADC1: 84-cycle sampling on IN4, IN5, IN8 */
    ADC1->SMPR2 = (4u << 12) | (4u << 15) | (4u << 24);
    ADC1->CR2 = 1u;
    /* I2C3 100 kHz from 16 MHz */
    I2C3->CR1 = 0;
    I2C3->CR2 = 16u;
    I2C3->CCR = 80u;
    I2C3->TRISE = 17u;
    I2C3->CR1 = 1u;
    SYST_RVR = F_CPU / 1000u - 1u;
    SYST_CVR = 0;
    SYST_CSR = 7u;
    /* sensors (optional: absence is reported as NaN / 0, never blocks the loop for long) */
    uint8_t who[2] = {0, 0};
    imu_ok = (uint8_t)(i2c_read2(IMU_ADDR, 0x0F, who) && who[0] == 0x6C);
    if (imu_ok) {
        const uint8_t g[2] = {0x11, 0x40}; /* CTRL2_G: 104 Hz, +/-250 dps */
        imu_ok = (uint8_t)i2c_write(IMU_ADDR, g, 2);
    }
    const uint8_t cfg[3] = {0x00, 0x2F, 0xFF}; /* INA219: 32 V range, PGA /2 (80 mV), 12-bit x128 averaging both, continuous */
    ina_ok = (uint8_t)i2c_write(INA219_ADDR, cfg, 3);
    /* independent watchdog ~250 ms */
    IWDG_KR = 0x5555u;
    IWDG_PR = 3u;
    IWDG_RLR = 250u;
    IWDG_KR = 0xCCCCu;
}

uint32_t hal_millis(void) { return ms; }

void hal_wait_tick(void) {
    while (!tick_flag) __asm__ volatile("wfi");
    tick_flag = 0;
}

void hal_read_inputs(ar_inputs_t *in) {
    static uint8_t slow;
    uint8_t b[2];
    uint16_t c0 = (uint16_t)TIM2->CNT, c1 = (uint16_t)TIM3->CNT;
    enc_acc[0] += LEFT_SIGN * (int16_t)(c0 - enc_prev[0]);
    enc_acc[1] += RIGHT_SIGN * (int16_t)(c1 - enc_prev[1]);
    enc_prev[0] = c0;
    enc_prev[1] = c1;
    in->t_ms = ms;
    in->enc_left = enc_acc[0];
    in->enc_right = enc_acc[1];
    in->i_left_ma = (int16_t)(duty_sign[0] * (int32_t)adc_read(4) * 3300 / 4095 * 1000 / OCM_MV_PER_A);
    in->i_right_ma = (int16_t)(duty_sign[1] * (int32_t)adc_read(5) * 3300 / 4095 * 1000 / OCM_MV_PER_A);
    in->vmot_mv = (uint16_t)((uint32_t)adc_read(8) * 3300 / 4095 * 122 / 22);
    in->estop_open = (uint8_t)((GPIOB->IDR >> 5) & 1u);
    in->bumper_open = (uint8_t)(GPIOA->IDR & 1u);
    in->driver_fault = (uint8_t)(!((GPIOB->IDR >> 10) & 1u));
    in->gyro_z = NAN;
    if (imu_ok && i2c_read2(IMU_ADDR, 0x26, b))
        in->gyro_z = (float)(int16_t)(b[0] | (b[1] << 8)) * (8.75e-3f * 3.14159265f / 180.0f);
    if (ina_ok && (++slow % 10) == 0) { /* 10 Hz battery monitor */
        if (i2c_read2(INA219_ADDR, 0x02, b)) vbat_mv = (uint16_t)((((uint16_t)b[0] << 8 | b[1]) >> 3) * 4);
        if (i2c_read2(INA219_ADDR, 0x01, b)) ibat_ma = (int16_t)((int16_t)((uint16_t)b[0] << 8 | b[1]) * 10 / SHUNT_MILLIOHM / 1); /* 10 uV LSB */
    }
    in->vbat_mv = vbat_mv;
    in->ibat_ma = ibat_ma;
}

static void set_bridge(volatile uint32_t *fwd, volatile uint32_t *rev, float duty, int sign, int8_t *sgn) {
    duty *= (float)sign;
    uint32_t v = (uint32_t)(fabsf(duty) * (float)(PWM_TOP + 1));
    if (v > PWM_TOP + 1) v = PWM_TOP + 1;
    *sgn = (int8_t)((duty >= 0 ? 1 : -1) * sign);
    if (duty >= 0) { *rev = 0; *fwd = v; } else { *fwd = 0; *rev = v; }
}

void hal_write_outputs(const ar_outputs_t *out) {
    set_bridge(&TIM4->CCR1, &TIM4->CCR2, out->driver_enable ? out->duty_left : 0.0f, LEFT_SIGN, &duty_sign[0]);
    set_bridge(&TIM4->CCR3, &TIM4->CCR4, out->driver_enable ? out->duty_right : 0.0f, RIGHT_SIGN, &duty_sign[1]);
    GPIOB->BSRR = out->driver_enable ? (1u << 1) : (1u << (1 + 16));
    GPIOA->BSRR = out->rail_enable ? (1u << 1) : (1u << (1 + 16));
}

int hal_uart_read(uint8_t *byte) {
    if (rx_tail == rx_head) return 0;
    *byte = rxbuf[rx_tail++];
    return 1;
}

void hal_uart_write(const uint8_t *data, size_t len) {
    for (size_t i = 0; i < len; i++) {
        uint8_t nh = (uint8_t)(tx_head + 1);
        if (nh == tx_tail) break; /* buffer full: drop the rest of this telemetry frame */
        txbuf[tx_head] = data[i];
        tx_head = nh;
    }
    USART2->CR1 |= (1u << 7);
}

void hal_watchdog_kick(void) { IWDG_KR = 0xAAAAu; }
void hal_led(int on) { GPIOC->BSRR = on ? (1u << (13 + 16)) : (1u << 13); }
