#include <cyg/hal/hal_arch.h>
#include <cyg/kernel/kapi.h>
#include <libopencm3/stm32/timer.h>

#include <stdint.h>
#include <stdbool.h>

#include "ezs_io.h"
#include "state_machine.h"
#include "lib.h"

int printf(const char*, ...);

static enum state currentState;

static const char *stateToString(enum state st) {
        switch (st) {
                case GOING_LEFT: return "GOING_LEFT"; break;
                case GOING_RIGHT: return "GOING_RIGHT"; break;
                case GOING_GOAL: return "GOING_GOAL"; break;
                case WACKEL_LINKS: return "WACKEL_LINKS"; break;
                case WACKEL_RECHTS: return "WACKEL_RECHTS"; break;
                case AUFSCHWINGEN_IMPULS: return "AUFSCHWINGEN_IMPULS"; break;
                case AUFSCHWINGEN_WARTEN: return "AUFSCHWINGEN_WARTEN"; break;
                case MOTOR_MESSUNG_START: return "MOTOR_MESSUNG_START"; break;
                case MOTOR_MESSUNG_ACTION: return "MOTOR_MESSUNG_ACTION"; break;
                case MOTOR_MESSUNG_OUTPUT: return "MOTOR_MESSUNG_OUTPUT"; break;
                case IDLE: return "IDLE"; break;
        }
        return "NONE";
}

static struct PeripheralState peripheralState;

static void readPeripheralState(struct PeripheralState *st) {
        st->positionWaggon = (int32_t) timer_get_counter(TIM2);
        st->positionPendulum = (int16_t) timer_get_counter(TIM3);
        // TODO
        st->positionPendulum = ((st->positionPendulum % 2400) + 2400) % 2400;
        if (st->positionPendulum > 1200) st->positionPendulum -= 2400;
        st->time = (uint32_t) timer_get_counter(TIM5);
        st->tasterLeft = !readPin(GPIOD, GPIO2);
        st->tasterRight = !readPin(GPIOD, GPIO1);
}

static void printPeripheralState(struct PeripheralState *st) {
        //ezs_printf("PeripheralState: Waggon %d, Pendulum %d, time %u, Taster %d/%d currentState: %s\n", (int)st->positionWaggon, (int)st->positionPendulum, (unsigned int) st->time, (int) st->tasterLeft, (int)st->tasterRight, stateToString(currentState));
}

/**
 * Is called once before any call of state_machine_step.
 */
void state_machine_init(void) {
        initHardware();
        readPeripheralState(&peripheralState);
        peripheralState.leftBound = INT32_MIN;
        peripheralState.rightBound = INT32_MAX;
        currentState = MOTOR_MESSUNG_START;
}
/**
 * Continously called. If true is returned the application terminates.
 */
bool state_machine_step(void) {
        readPeripheralState(&peripheralState);

        /**
         * If the waggon is out of the defined bounding area, return to the middle
         */
        if (peripheralState.positionWaggon >= peripheralState.rightBound || peripheralState.positionWaggon <= peripheralState.leftBound) {
                currentState = GOING_GOAL;
        }

        printPeripheralState(&peripheralState);
        switch (currentState) {
                case GOING_LEFT:
                        currentState = handle_going_left(&peripheralState);
                        break;
                case GOING_RIGHT:
                        currentState = handle_going_right(&peripheralState);
                        break;
                case GOING_GOAL:
                        currentState = handle_going_goal(&peripheralState);
                        break;
                case WACKEL_LINKS:
                        currentState = handle_wackel_links(&peripheralState);
                        break;
                case WACKEL_RECHTS:
                        currentState = handle_wackel_rechts(&peripheralState);
                        break;
                case AUFSCHWINGEN_WARTEN:
                        currentState = handle_aufschwingen_warten(&peripheralState);
                        break;
                case AUFSCHWINGEN_IMPULS:
                        currentState = handle_aufschwingen_impuls(&peripheralState);
                        break;
                case MOTOR_MESSUNG_START:
                        currentState = handle_motor_messung_start(&peripheralState);
                        break;
                case MOTOR_MESSUNG_ACTION:
                        currentState = handle_motor_messung_action(&peripheralState);
                        break;
                case MOTOR_MESSUNG_OUTPUT:
                        currentState = handle_motor_messung_output(&peripheralState);
                        break;
                case IDLE: breakMotor();
                        break;
        }

        return false;
}
