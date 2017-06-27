#include <cyg/hal/hal_arch.h>
#include <cyg/kernel/kapi.h>
#include <cyg/infra/diag.h>

#include <stdint.h>
#include <stdio.h>
#include <math.h>
#include <iso646.h>

#include "ezs_counter.h"
#include "ezs_dac.h"
#include "ezs_delay.h"
#include "ezs_dac.h"
#include "ezs_gpio.h"
#include "ezs_io.h"


//#define PI 3.14159265359
#define PRIO 0
/* Thread-Stack */
#define STACKSIZE CYGNUM_HAL_STACK_SIZE_MINIMUM+1024
static cyg_uint8 my_stack[STACKSIZE];
static cyg_uint8 sin_stack[STACKSIZE];
static cyg_handle_t handle;
static cyg_thread threaddata;
static cyg_handle_t sin_handle;
static cyg_thread sin_threaddata;

void test_thread(cyg_addrword_t arg)
{
	/* Einmalige Aufgaben ... */

	while(1)
	{
		/* Periodische Aufgaben ... */
		ezs_printf("Hallo Welt!\n");
		ezs_delay_us(1000000);
	}

}

void sin_thread(cyg_addrword_t arg){
	cyg_uint32 freq = 10; /* Frequenz hz */
	cyg_uint32 amp = 126; /* Amplitude */
	long dt = 1000000/(10*freq); /* Abtastrate us */
	float value;
	while(1){
		long t;
		for(t = 0; t <= 1000000/freq; t+=dt){
			value = 127+amp*sinf(2*M_PI*freq*t/1000000.);
			ezs_dac_write(value);
			ezs_delay_us(dt);
		}
	}
}


void cyg_user_start(void)
{
	ezs_gpio_init();
	ezs_dac_init();
	ezs_counter_init();

	/* Thread erzeugen ... */
	cyg_thread_create(PRIO, &test_thread, 0, "hallo welt thread",
		my_stack, STACKSIZE, &handle, &threaddata);
	cyg_thread_create(PRIO, &sin_thread, 0, "sin thread",
		sin_stack, STACKSIZE, &sin_handle, &sin_threaddata);


	/* Thread starten ... */
	cyg_thread_resume(handle);
	cyg_thread_resume(sin_handle);
}

