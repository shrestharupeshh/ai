/* Cortex-M4 startup for STM32F411: vector table, .data/.bss init, default handlers. */
#include <stdint.h>
extern uint32_t _estack, _sidata, _sdata, _edata, _sbss, _ebss;
int main(void);
void SysTick_Handler(void);
void USART2_IRQHandler(void);

void Default_Handler(void) {
    for (;;) {} /* the independent watchdog resets the MCU; outputs default to the safe state */
}

void Reset_Handler(void) {
    *(volatile uint32_t *)0xE000ED88u |= (0xFu << 20); /* enable the FPU before any hard-float code runs */
    uint32_t *src = &_sidata, *dst = &_sdata;
    while (dst < &_edata) *dst++ = *src++;
    for (dst = &_sbss; dst < &_ebss;) *dst++ = 0;
    main();
    for (;;) {}
}

#define NIRQ 86
__attribute__((section(".isr_vector"), used)) void (*const vector_table[16 + NIRQ])(void) = {
    [0] = (void (*)(void))(&_estack),
    [1] = Reset_Handler,
    [2 ... 14] = Default_Handler,
    [15] = SysTick_Handler,
    [16 ... 16 + 37] = Default_Handler,
    [16 + 38] = USART2_IRQHandler,
    [16 + 39 ... 16 + NIRQ - 1] = Default_Handler,
};
