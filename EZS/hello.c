#include <cyg/hal/hal_arch.h>
#include <cyg/hal/hal_io.h>
#include <cyg/kernel/kapi.h>
#include <cyg/infra/diag.h>

#include <libopencm3/cm3/nvic.h>

#include <libopencm3/stm32/exti.h>
#include <libopencm3/stm32/gpio.h>

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
static cyg_uint8    polling_stack[STACKSIZE];
static cyg_thread   polling_data;
static cyg_handle_t polling_handle;

static cyg_uint8    interrupt_stack[STACKSIZE];
static cyg_handle_t interrupt_handle;
static cyg_thread   interrupt_data;

static volatile bool flag_a0, flag_a1;
static volatile int pos = 0;
static volatile int vala0, vala1;

void polling_thread(cyg_addrword_t arg)
{

	//cyg_uint32 pina0 = CYGHWR_HAL_STM32_PIN_IN(A, 0, PULLUP);
	//cyg_uint32 pina1 = CYGHWR_HAL_STM32_PIN_IN(A, 1, PULLUP);

//gpio_mode_setup(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, GPIO0);

	for (;;) {
		//gpio_set(LED_DISCO_BLUE_PORT, LED_DISCO_BLUE_PIN);
		//int val;
		//CYGHWR_HAL_STM32_GPIO_IN(pin, &val);
		ezs_printf("%d %d %d\n", pos, vala0, vala1);
	}
}

void interrupt_thread(cyg_addrword_t arg) {

}

inline void step(void) {
	int reg16 = gpio_port_read(GPIOA);

	int new_vala0 = reg16 & 1;
	int new_vala1 = (reg16 >> 1) & 1;

	if (!vala0 && !new_vala0) {
		if (new_vala1) {
			// clockwise
			pos ++;
		} else {
			pos --;
		}
	} else if (!vala0 && new_vala0) {
		if (!new_vala1) {
			// clockwise
			pos ++;
		} else {
			pos --;
		}
	} else if (vala0 && new_vala0) {
		if (new_vala1) {
			// clockwise
			pos ++;
		} else {
			pos --;
		}
	} else if (vala0 && !new_vala0) {
		if (vala1) {
			// clockwise
			pos ++;
		} else {
			pos --;
		}
	}

	vala0 = new_vala0;
	vala1 = new_vala1;
}

void interrupt_dsr_handler_a0(cyg_vector_t vec, cyg_ucount32 count, cyg_addrword_t data) {
	step();
}
void interrupt_dsr_handler_a1(cyg_vector_t vec, cyg_ucount32 count, cyg_addrword_t data) {
	step();
}

cyg_uint32 interrupt_isr_handler_a0(cyg_vector_t vector, cyg_addrword_t data) {
	cyg_interrupt_acknowledge(vector);
	return CYG_ISR_CALL_DSR;
}

cyg_uint32 interrupt_isr_handler_a1(cyg_vector_t vector, cyg_addrword_t data) {
	cyg_interrupt_acknowledge(vector);
	return CYG_ISR_CALL_DSR;
}

static cyg_handle_t  interrupt_isr_handle_a0, interrupt_isr_handle_a1;
static cyg_interrupt interrupt_intr_a0, interrupt_intr_a1;

void registerPinInterrupts(void) {
	// Register EXTI Interrupt on A0
	nvic_enable_irq(NVIC_EXTI0_IRQ);
	gpio_mode_setup(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, GPIO0);
	exti_select_source(EXTI0, GPIOA);
	exti_set_trigger(EXTI0, EXTI_TRIGGER_BOTH);

	cyg_interrupt_create(CYGNUM_HAL_INTERRUPT_EXTI0,// vector,
			1, 				// priority
			0,				// argument
			interrupt_isr_handler_a0, 	// isr handler
			interrupt_dsr_handler_a0,	// dsr handler
			&interrupt_isr_handle_a0, 	// isr handle
			&interrupt_intr_a0		// intr ???
			);
	cyg_interrupt_attach(interrupt_isr_handle_a0);
	cyg_interrupt_unmask(CYGNUM_HAL_INTERRUPT_EXTI0);

	// Register EXTI Interrupt on A1
	nvic_enable_irq(NVIC_EXTI0_IRQ);
	gpio_mode_setup(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, GPIO1);
	exti_select_source(EXTI1, GPIOA);
	exti_set_trigger(EXTI1, EXTI_TRIGGER_BOTH);

	cyg_interrupt_create(CYGNUM_HAL_INTERRUPT_EXTI1, // vector,
			1, 				 // priority
			0,				 // argument
			interrupt_isr_handler_a1, 	 // isr handler
			interrupt_dsr_handler_a1,	 // dsr handler
			&interrupt_isr_handle_a1,	 // isr handle
			&interrupt_intr_a1		 // intr ???
			);
	cyg_interrupt_attach(interrupt_isr_handle_a1);
	cyg_interrupt_unmask(CYGNUM_HAL_INTERRUPT_EXTI1);

}

void cyg_user_start(void)
{
	//CYGHWR_HAL_STM32_GPIO_IRQ_INIT();
	//cyg_uint32 pin = CYGHWR_HAL_STM32_PIN_IN(A, 0, PULLUP);
	//cyg_bool enabled = CYGHWR_HAL_VAR_GPIO_IRQ_ENABLE(pin, CYGHWR_HAL_VAR_IRQ_EDGE_BOTH, cyg_hal_var_gpio_callback_t handler, NULL);

	flag_a0 = 0;
	flag_a1 = 0;

	registerPinInterrupts();


	/* Thread erzeugen ... */
	cyg_thread_create(PRIO, &polling_thread, 0, "Pin polling thread", polling_stack, STACKSIZE, &polling_handle, &polling_data);
	//cyg_thread_create(PRIO, &interrupt_thread, 0, "Pin interrupt thread", interrupt_stack, STACKSIZE, &interrupt_handle, &interrupt_data);
	//cyg_thread_create(PRIO, &sin_thread, 0, "sin thread", sin_stack, STACKSIZE, &sin_handle, &sin_threaddata);

	int reg16 = gpio_port_read(GPIOA);

	vala0 = reg16 & 1;
	vala1 = (reg16 >> 1) & 1;

	/* Thread starten ... */
	//cyg_thread_resume(interrupt_handle);
	cyg_thread_resume(polling_handle);
}

