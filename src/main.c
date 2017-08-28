#include <cyg/hal/hal_arch.h>
#include <cyg/kernel/kapi.h>

#include "state_machine.h"

#define PRIO 0
/* Thread-Stack */
#define STACKSIZE CYGNUM_HAL_STACK_SIZE_MINIMUM+1024
static cyg_uint8    polling_stack[STACKSIZE];
static cyg_thread   polling_data;
static cyg_handle_t polling_handle;
void polling_thread(cyg_addrword_t arg)
{
        state_machine_init();
        for (;;) {
                bool terminate = state_machine_step();
                if (terminate) {
                        break;
                }
        }
}

void cyg_user_start(void)
{
        cyg_thread_create(PRIO, &polling_thread, 0, "Pin polling thread", polling_stack, STACKSIZE, &polling_handle, &polling_data);
        cyg_thread_resume(polling_handle);
}
