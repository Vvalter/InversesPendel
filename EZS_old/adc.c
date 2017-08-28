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

        int success;
        success = initRotaryEncoderTimer(TIM2, GPIOA, GPIO5, GPIO_AF1, GPIOA, GPIO1, GPIO_AF1);
        success = initRotaryEncoderTimer(TIM3, GPIOC, GPIO6, GPIO_AF2, GPIOC, GPIO7, GPIO_AF2);
        success = initRotaryEncoderTimer(TIM4, GPIOD, GPIO13, GPIO_AF2, GPIOD, GPIO12, GPIO_AF2);
        success = initTimer(TIM5);

        unsigned int last_time = 0;
        int last_angle = 0;
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


                //double current_drehzahl = 


                last_time = current_time;
                last_angle = current_angle;
                ezs_printf("TIM2: %10d TIM3: %10d TIM4: %10d TIM5: %10u\n", (int)timer_get_counter(TIM2), (int) timer_get_counter(TIM3), (int) timer_get_counter(TIM5), (unsigned int) (timer_get_counter(TIM5) / (42*1e6)));
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
