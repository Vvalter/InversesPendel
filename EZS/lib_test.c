#include <cyg/hal/hal_arch.h>
#include <cyg/kernel/kapi.h>

#include <libopencm3/stm32/timer.h>

#include "ezs_io.h"
#define EZS_DUMB_SERIAL
#include "ezs_serial.h"
#include "lib.h"

#define PRIO 0
/* Thread-Stack */
#define STACKSIZE CYGNUM_HAL_STACK_SIZE_MINIMUM+1024
static cyg_uint8    polling_stack[STACKSIZE];
static cyg_thread   polling_data;
static cyg_handle_t polling_handle;


/*
 * TIM3 rotary input C6 C7
 */

void polling_thread(cyg_addrword_t arg)
{
	initRCC();

	initADC(GPIOB, GPIO0);

        initRotaryEncoderTimer(TIM2, GPIOA, GPIO5, GPIO_AF1, GPIOA, GPIO1, GPIO_AF1);
        initRotaryEncoderTimer(TIM3, GPIOC, GPIO6, GPIO_AF2, GPIOC, GPIO7, GPIO_AF2);

	initTimer(TIM5);

	char lastChar = ' ';
	uint32_t start = timer_get_counter(TIM5);
	for(;;) {
		if (timer_get_counter(TIM5) - start > 1000000) {
			start = timer_get_counter(TIM5);
			ezs_printf("%d %c\n", (int)timer_get_counter(TIM2),
					lastChar
					);
			lastChar = ezs_serial_getc();
		}
		//ezs_printf("%10d | %10d | %10d | %10u\n", (int) timer_get_counter(TIM3), (int) timer_get_counter(TIM2), adc_val, (unsigned int) (timer_get_counter(TIM5)/((double) 42 * 1e6)));
	}
}

void cyg_user_start(void)
{
        cyg_thread_create(PRIO, &polling_thread, 0, "Pin polling thread", polling_stack, STACKSIZE, &polling_handle, &polling_data);
        cyg_thread_resume(polling_handle);
}
