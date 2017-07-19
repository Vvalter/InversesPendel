#include <cyg/hal/hal_arch.h>
#include <cyg/hal/hal_io.h>
#include <cyg/kernel/kapi.h>
#include <cyg/infra/diag.h>

#include <libopencm3/stm32/f4/memorymap.h>
#include <libopencm3/cm3/common.h>
#include <libopencm3/stm32/f4/rcc.h>
#include <libopencm3/stm32/f4/gpio.h>
#include <libopencm3/stm32/timer.h>

#include "ezs_io.h"
#include "lib.h"

//#define PI 3.14159265359
#define PRIO 0
/* Thread-Stack */
#define STACKSIZE CYGNUM_HAL_STACK_SIZE_MINIMUM+1024
static cyg_uint8    polling_stack[STACKSIZE];
static cyg_thread   polling_data;
static cyg_handle_t polling_handle;

/*
 * TIM5 gives time
 * TIM2 gives rotary encoder on ping A5 and A1
 * TIM3 rotary input C6 C7
 * enable pin PC0 
 * PWM: PD13, PD14, PD15 with TIM4
 *      PD13 100%
 *      PD14 25%
 */

/// The timer ticks per microsecond.  Can be adjusting by measuring
/// pwm on o-scope
#define PWM_TIMER_TICKS_PER_MICROSECOND 84.0

/// PWM Frequency, default is 62 kHz
// 162 corresponds to 80kHz (80.78kHz)
#define PWM_FREQUENCY_kHz (162.0*0.2)

/// PWM Period, set automatically by the options above
#define PWM_PERIOD ((1.0/PWM_FREQUENCY_kHz) * 1000.0 \
                    * PWM_TIMER_TICKS_PER_MICROSECOND / 2)


void pwm_init(uint32_t timer, uint8_t channel, uint32_t period) {
// function stolen from somewhere on stackoverflow

  // Convert channel number to internal rep
  enum tim_oc_id chan;
  switch (channel) {
    case 1:   chan = TIM_OC1; break;
    case 2:   chan = TIM_OC2; break;
    case 3:   chan = TIM_OC3; break;
    case 4:   chan = TIM_OC4; break;
    default:  chan = TIM_OC4; break;
  }

  // Timer Base Configuration
  // timer_reset(timer);
  timer_set_mode(timer, TIM_CR1_CKD_CK_INT, // clock division
                        TIM_CR1_CMS_EDGE,   // Center-aligned mode selection
                        TIM_CR1_DIR_UP);    // TIMx_CR1 DIR: Direction
  timer_continuous_mode(timer);             // Disables TIM_CR1_OPM (One pulse mode)
  timer_set_period(timer, period);                    // Sets TIMx_ARR
  timer_set_prescaler(timer, 1);               // Adjusts speed of timer
  timer_set_clock_division(timer, 0);            // Adjusts speed of timer
  timer_set_master_mode(timer, TIM_CR2_MMS_UPDATE);   // Master Mode Selection
  timer_enable_preload(timer);                        // Set ARPE bit in TIMx_CR1

  // Channel-specific settings
  timer_set_oc_value(timer, chan, 0);             // sets TIMx_CCRx
  /*
   * In PWM mode 1 channel is active as long as TIMx_CNT<TIMx_CCR1, else inactive
   */
  timer_set_oc_mode(timer, chan, TIM_OCM_PWM1);   // Sets PWM Mode 1
  timer_enable_oc_preload(timer, chan);           // Sets OCxPE in TIMx_CCMRx
  timer_set_oc_polarity_high(timer, chan);        // set desired polarity in TIMx_CCER
  timer_enable_oc_output(timer, chan);             // set CCxE bit in TIMx_CCER  (enable output)

  // Initialize all counters in the register
  switch (timer) {
    case TIM1:  TIM1_EGR |= TIM_EGR_UG; break;
    case TIM2:  TIM2_EGR |= TIM_EGR_UG; break;
    case TIM3:  TIM3_EGR |= TIM_EGR_UG; break;
    case TIM4:  TIM4_EGR |= TIM_EGR_UG; break;
    case TIM5:  TIM5_EGR |= TIM_EGR_UG; break;
    case TIM6:  TIM6_EGR |= TIM_EGR_UG; break;
    case TIM7:  TIM7_EGR |= TIM_EGR_UG; break;
    case TIM8:  TIM8_EGR |= TIM_EGR_UG; break;
    }
}

void pwm_setup(void) {
  rcc_periph_clock_enable(RCC_TIM4);
  //pwm_init(TIM4, 1, PWM_PERIOD);
  pwm_init(TIM4, 2, PWM_PERIOD);
  pwm_init(TIM4, 3, PWM_PERIOD);
  pwm_init(TIM4, 4, PWM_PERIOD);

  // LED channels = PD12..15
  rcc_peripheral_enable_clock(&RCC_AHB1ENR, RCC_AHB1ENR_IOPDEN);

  gpio_mode_setup(GPIOD, GPIO_MODE_AF, GPIO_PUPD_NONE,
                  GPIO13 | GPIO14 | GPIO15);
  // AF2 = TIM4_CH1..4
  gpio_set_af(GPIOD, GPIO_AF2, GPIO13 | GPIO14 | GPIO15);
  timer_enable_counter(TIM4);
}

void polling_thread(cyg_addrword_t arg)
{

        int success;
        success = initRotaryEncoderTimer(TIM2, GPIOA, GPIO5, GPIO_AF1, GPIOA, GPIO1, GPIO_AF1);
        success = initRotaryEncoderTimer(TIM3, GPIOC, GPIO6, GPIO_AF2, GPIOC, GPIO7, GPIO_AF2);
        //success = initRotaryEncoderTimer(TIM4, GPIOD, GPIO13, GPIO_AF2, GPIOD, GPIO12, GPIO_AF2);
        success = initTimer(TIM5);

        initPinOutput(GPIOC, GPIO0);
        setPinHigh(GPIOC, GPIO0);

        pwm_setup();

	timer_set_oc_value(TIM4, TIM_OC2, PWM_PERIOD);
	timer_set_oc_value(TIM4, TIM_OC3, PWM_PERIOD * 0.75);

        unsigned int last_time = 0;
        int last_angle = 0;
        double duty = 0.0;

        int cnt = 0;
        for (;;) {
                unsigned int current_time = timer_get_counter(TIM5);
                int current_angle = timer_get_counter(TIM2);

                if (current_time < last_time) {
                        last_time = current_time;
                        last_angle = current_angle;
                        //continue;
                }

                if (current_time == last_time) {
                        continue;
                }

                if (cnt >= 10) {
                        int drehgeber = timer_get_counter(TIM3);
                        drehgeber = ((drehgeber + 2400)) % 2400;
                        duty = drehgeber / 2400.0;
                        timer_set_oc_value(TIM4, TIM_OC3, PWM_PERIOD * duty);
                        cnt = 0;
                }
                cnt ++;


                //double current_drehzahl = 


                last_time = current_time;
                last_angle = current_angle;
                ezs_printf("Time: %10u | Angle: %10d | duty %3.3f | TIM3: %10d \n", (unsigned int)(timer_get_counter(TIM5)/(42*1e6)), (int) timer_get_counter(TIM2), duty, (int) timer_get_counter(TIM3));
                //ezs_printf("TIM2: %10d TIM3: %10d TIM4: %10d TIM5: %10u\n", (int)timer_get_counter(TIM2), (int) timer_get_counter(TIM3), (int) timer_get_counter(TIM5), (unsigned int) (timer_get_counter(TIM5) / (42*1e6)));
        }
}

void cyg_user_start(void)
{
        cyg_thread_create(PRIO, &polling_thread, 0, "Pin polling thread", polling_stack, STACKSIZE, &polling_handle, &polling_data);
        cyg_thread_resume(polling_handle);
}
