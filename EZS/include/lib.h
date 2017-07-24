#ifndef _LIB_H_
#define _LIB_H_
// TODO remove unused imports
#include <libopencm3/stm32/f4/memorymap.h>
#include <libopencm3/cm3/common.h>
#include <libopencm3/stm32/f4/rcc.h>
#include <libopencm3/stm32/f4/gpio.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/adc.h>

#define TIMER_TICKS_PER_MICROSECOND 42

void initRCC(void);
int initRotaryEncoderTimer(uint32_t tim, uint32_t portA, uint16_t pinA, uint8_t afA, uint32_t portB, uint16_t pinB, uint8_t afB);
int initTimer(uint32_t tim);

void initPinOutput(uint32_t port, uint16_t pin);
void setPinHigh(uint32_t port, uint16_t pin);
void setPinLow(uint32_t port, uint16_t pin);

/*
 * Initializes ADC to read from port/pin.
 */
void initADC(uint32_t port, uint16_t pin);
uint16_t readADCBlocking(void);

#endif

