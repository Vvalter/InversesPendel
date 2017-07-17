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

//#include "stm32f4xx_hal.h"

//#define PI 3.14159265359
#define PRIO 0
/* Thread-Stack */
#define STACKSIZE CYGNUM_HAL_STACK_SIZE_MINIMUM+1024
static cyg_uint8    polling_stack[STACKSIZE];
static cyg_thread   polling_data;
static cyg_handle_t polling_handle;


void polling_thread(cyg_addrword_t arg)
{

	//cyg_uint32 pina0 = CYGHWR_HAL_STM32_PIN_IN(A, 0, PULLUP);
	//cyg_uint32 pina1 = CYGHWR_HAL_STM32_PIN_IN(A, 1, PULLUP);

	//gpio_mode_setup(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, GPIO0);
	//HAL_Init();
	//SystemClock_Config();
	//
	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO5);
	gpio_set_af(GPIOA, GPIO_AF1, GPIO5);

	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO1);
	gpio_set_af(GPIOA, GPIO_AF1, GPIO1);

	rcc_periph_clock_enable(RCC_GPIOA);
	//rcc_periph_clock_enable(RCC_GPIOB);
	rcc_periph_clock_enable(RCC_TIM2);

	timer_reset(TIM2);

	timer_set_mode(TIM2, TIM_CR1_CKD_CK_INT, //For dead time and filter sampling, not important for now.
		       TIM_CR1_CMS_EDGE,	//TIM_CR1_CMS_EDGE
						//TIM_CR1_CMS_CENTER_1
						//TIM_CR1_CMS_CENTER_2
						//TIM_CR1_CMS_CENTER_3 la frequencia del pwm se divide a la mitad.
			 TIM_CR1_DIR_UP);

	timer_set_prescaler(TIM2, 0);
	timer_set_repetition_counter(TIM2, 0);
	timer_enable_preload(TIM2);
	timer_continuous_mode(TIM2);

	timer_slave_set_mode(TIM2, TIM_SMCR_SMS_EM3); // encoder

	timer_set_oc_polarity_high(TIM2, TIM_OC1);
	timer_set_oc_polarity_high(TIM2, TIM_OC2);

	timer_ic_disable(TIM2, TIM_IC1);
	timer_ic_disable(TIM2, TIM_IC2);

	timer_ic_set_input(TIM2, TIM_IC1, TIM_IC_IN_TI1);
	timer_ic_set_input(TIM2, TIM_IC2, TIM_IC_IN_TI1);

	timer_disable_oc_output(TIM2, TIM_OC1);
	timer_disable_oc_output(TIM2, TIM_OC2);

	timer_disable_preload_complementry_enable_bits(TIM2);

	timer_set_period(TIM2, 0xFFFFFFFF);

	timer_enable_counter(TIM2);

	//timer_set_mode(TIM2, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);

	//MX_GPIO_Init();
	
	//MX_TIM2_Init();
	//timer_enable_counter(TIM2);

	for (;;) {
		//gpio_set(LED_DISCO_BLUE_PORT, LED_DISCO_BLUE_PIN);
		//int val;
		//CYGHWR_HAL_STM32_GPIO_IN(pin, &val);
		ezs_printf("hello %d \n", (int)timer_get_counter(TIM2));
	}
}

void cyg_user_start(void)
{
	//CYGHWR_HAL_STM32_GPIO_IRQ_INIT();
	//cyg_uint32 pin = CYGHWR_HAL_STM32_PIN_IN(A, 0, PULLUP);
	//cyg_bool enabled = CYGHWR_HAL_VAR_GPIO_IRQ_ENABLE(pin, CYGHWR_HAL_VAR_IRQ_EDGE_BOTH, cyg_hal_var_gpio_callback_t handler, NULL);


	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */

	/* Thread erzeugen ... */
	cyg_thread_create(PRIO, &polling_thread, 0, "Pin polling thread", polling_stack, STACKSIZE, &polling_handle, &polling_data);
	//cyg_thread_create(PRIO, &interrupt_thread, 0, "Pin interrupt thread", interrupt_stack, STACKSIZE, &interrupt_handle, &interrupt_data);
	//cyg_thread_create(PRIO, &sin_thread, 0, "sin thread", sin_stack, STACKSIZE, &sin_handle, &sin_threaddata);

	/* Thread starten ... */
	//cyg_thread_resume(interrupt_handle);
	cyg_thread_resume(polling_handle);
}
