#pragma once

#include <stdint.h>

/*
//#include <stdint.h>

//#define DC_LEFT 0
//#define DC_RIGHT 1

#ifdef __cplusplus
extern "C" {
#endif

void adc_init();

//void adc_get(uint8_t direction, uint8_t speed);

#ifdef __cplusplus
}
#endif
*/

#ifdef __cplusplus
extern "C" {
#endif

void clock_setup(void);
void adc_setup(void);
uint16_t read_adc_naiive(uint8_t channel);

#ifdef __cplusplus
}
#endif
