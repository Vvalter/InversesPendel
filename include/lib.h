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

/// The timer ticks per microsecond.  Can be adjusting by measuring
/// pwm on o-scope
#define PWM_TIMER_TICKS_PER_MICROSECOND 84.0

/// PWM Frequency, default is 62 kHz
// 162 corresponds to 80kHz (80.78kHz)
#define PWM_FREQUENCY_kHz (162.0*0.2)

/// PWM Period, set automatically by the options above
#define PWM_PERIOD ((1.0/PWM_FREQUENCY_kHz) * 1000.0 \
                    * PWM_TIMER_TICKS_PER_MICROSECOND / 2)

#ifdef __cplusplus
extern "C" {
#endif

void initRCC(void);
int initRotaryEncoderTimer(uint32_t tim, uint32_t portA, uint16_t pinA, uint8_t afA, uint32_t portB, uint16_t pinB, uint8_t afB);
int initTimer(uint32_t tim);

void initPinOutput(uint32_t port, uint16_t pin);
void setPinHigh(uint32_t port, uint16_t pin);
void setPinLow(uint32_t port, uint16_t pin);

void initPinInput(uint32_t port, uint16_t pin, uint8_t pull);
bool readPin(uint32_t port, uint16_t pin);

/*
 * Initializes ADC to read from port/pin.
 */
void initADC(uint32_t port, uint16_t pin);
uint16_t readADCBlocking(void);
float readPoti(void);

void pwm_init(uint32_t timer, uint8_t channel, uint32_t period, uint32_t port, uint16_t pin, uint16_t af);

void stopMotor(void);
void startMotor(void);
void breakMotor(void);

void driveLeft(float speed);
void driveLeftFast(void);

void driveRight(float speed);
void driveRightFast(void);

void initHardware(void);

/*
 * Helper Functions
 */
int convertPendulumAngleToUpper(int16_t positionPendulum);
int convertPendulumAngleToLower(int16_t positionPendulum);

double fmin(double a, double b);
double fabs(double a);

#ifdef __cplusplus
}
#endif
#endif

