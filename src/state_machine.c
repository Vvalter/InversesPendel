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
static int step = 0;

static const char *stateToString(enum state st) {
        switch (st) {
                case GOING_LEFT: return "GOING_LEFT"; break;
                case GOING_RIGHT: return "GOING_RIGHT"; break;
                case GOING_GOAL: return "GOING_GOAL"; break;
                case WACKEL_LINKS: return "WACKEL_LINKS"; break;
                case WACKEL_RECHTS: return "WACKEL_RECHTS"; break;
                case AUFSCHWINGEN_IMPULS: return "AUFSCHWINGEN_IMPULS"; break;
                case AUFSCHWINGEN_WARTEN: return "AUFSCHWINGEN_WARTEN"; break;
                case MOTOR_MESSUNG_INIT: return "MOTOR_MESSUNG_INIT"; break;
                case MOTOR_MESSUNG_ACTION: return "MOTOR_MESSUNG_ACTION"; break;
                case MOTOR_MESSUNG_OUTPUT: return "MOTOR_MESSUNG_OUTPUT"; break;
                case PID_INIT: return "PID_INIT"; break;
                case PID_STEP: return "PID_STEP"; break;
                case PWM_MESSUNG_INIT: return "PWM_MESSUNG_INIT"; break;
                case PWM_MESSUNG_ACTION: return "PWM_MESSUNG_ACTION"; break;
                case IDLE: return "IDLE"; break;
        }
        return "NONE";
}

static struct PeripheralState peripheralState;

static void readPeripheralState(struct PeripheralState *st) {
        st->positionWaggon = (int32_t) timer_get_counter(TIM2);
        st->positionPendulum = (int16_t) timer_get_counter(TIM3);
        st->time = (uint32_t) timer_get_counter(TIM5);
        st->tasterLeft = !readPin(GPIOD, GPIO2);
        st->tasterRight = !readPin(GPIOD, GPIO1);
}

static void printPeripheralState(struct PeripheralState *st) {
        step ++;
        if (step % 1000 == 0) {
                //ezs_printf("PeripheralState: Waggon %d, Pendulum %d, time %u, Taster %d/%d currentState: %s\n", (int)st->positionWaggon, (int)st->positionPendulum, (unsigned int) st->time, (int) st->tasterLeft, (int)st->tasterRight, stateToString(currentState));
        }
}

static void reset(void) {
        readPeripheralState(&peripheralState);
        peripheralState.leftBound = INT32_MIN;
        peripheralState.rightBound = INT32_MAX;
        currentState = PWM_MESSUNG_INIT;
}

/**
 * Is called once before any call of state_machine_step.
 */
void state_machine_init(void) {
        initHardware();
        reset();
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

        if (peripheralState.tasterLeft && peripheralState.tasterRight) {
                reset();
                return false;
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
                case MOTOR_MESSUNG_INIT:
                        currentState = handle_motor_messung_init(&peripheralState);
                        break;
                case MOTOR_MESSUNG_ACTION:
                        currentState = handle_motor_messung_action(&peripheralState);
                        break;
                case MOTOR_MESSUNG_OUTPUT:
                        currentState = handle_motor_messung_output(&peripheralState);
                        break;
                case PID_INIT:
                        currentState = handle_pid_init(&peripheralState);
                        break;
                case PID_STEP:
                        currentState = handle_pid_step(&peripheralState);
                        break;
                case PWM_MESSUNG_INIT:
                        currentState = handle_pwm_messung_init(&peripheralState);
                        break;
                case PWM_MESSUNG_ACTION:
                        currentState = handle_pwm_messung_action(&peripheralState);
                        break;
                case IDLE:
                        breakMotor();
                        if (convertPendulumAngleToUpper(peripheralState.positionPendulum) == 0) {
                                currentState = PID_INIT;
                        }
                        break;
        }

        return false;
}
