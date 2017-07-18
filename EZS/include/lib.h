#ifndef _LIB_H_
#define _LIB_H_
// TODO remove unused imports
#include <libopencm3/stm32/f4/memorymap.h>
#include <libopencm3/cm3/common.h>
#include <libopencm3/stm32/f4/rcc.h>
#include <libopencm3/stm32/f4/gpio.h>
#include <libopencm3/stm32/timer.h>


int initRotaryEncoderTimer(uint32_t tim, enum rcc_periph_clken rccTim, uint32_t portA, uint16_t pinA, uint8_t afA, enum rcc_periph_clken rcc_A, uint32_t portB, uint16_t pinB, uint8_t afB, enum rcc_periph_clken rccB);

#endif
