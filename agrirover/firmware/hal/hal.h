/* Hardware abstraction used by app/main.c. Implemented for STM32F411 (hal/stm32f411) and for the host (tests). */
#ifndef AR_HAL_H
#define AR_HAL_H
#include <stddef.h>
#include <stdint.h>
#include "../core/control.h"

void hal_init(void);
uint32_t hal_millis(void);
void hal_wait_tick(void);                        /* blocks until the next 100 Hz control tick */
void hal_read_inputs(ar_inputs_t *in);           /* encoders, currents, voltages, switches, IMU */
void hal_write_outputs(const ar_outputs_t *out); /* PWM, driver EN, motor-rail enable */
int hal_uart_read(uint8_t *byte);                /* non-blocking, 1 if a byte was read */
void hal_uart_write(const uint8_t *data, size_t len);
void hal_watchdog_kick(void);
void hal_led(int on);
#endif
