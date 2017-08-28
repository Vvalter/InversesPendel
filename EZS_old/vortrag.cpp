#include <cyg/hal/hal_arch.h>
#include <cyg/kernel/kapi.h>
#include <map>

#include <libopencm3/stm32/timer.h>

#include "stdlib.h"
#include "limits.h"

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

extern "C" int printf(const char *format, ...);

static const float SPEED = 0.5;

enum state { GOING_LEFT, GOING_RIGHT, GOING_GOAL, WACKEL_LINKS, WACKEL_RECHTS, AUFSCHWINGEN_WARTEN, AUFSCHWINGEN_IMPULS, DREHZAHL_MESSUNG, IDLE};

static const char *stateToString(enum state st) {
        switch (st) {
                case GOING_LEFT: return "GOING_LEFT"; break;
                case GOING_RIGHT: return "GOING_LEFT"; break;
                case GOING_GOAL: return "GOING_GOAL"; break;
                case WACKEL_LINKS: return "WACKEL_LINKS"; break;
                case WACKEL_RECHTS: return "WACKEL_RECHTS"; break;
                case AUFSCHWINGEN_IMPULS: return "AUFSCHWINGEN_IMPULS"; break;
                case AUFSCHWINGEN_WARTEN: return "AUFSCHWINGEN_WARTEN"; break;
                case DREHZAHL_MESSUNG: return "DREHZAHL_MESSUNG"; break;
                case IDLE: return "IDLE"; break;
        }
        return "NONE";
}


/*
 * TIM2 is position of waggon (32bit)
 *      pins
 *              A1 GREEN
 *              A5 WHITE
 *      39393
 *      39253
 *      39260
 * TIM3 is position of pendulum (16bit)
 *      pins
 *              C6 WHITE
 *              C7 GREEN
 *
 * TIM4 is pwm
 *      Channel 1 is IN 1
 *              pins B6
 *      Channel 2 is IN 2
 *              pins B7
 *
 * EN A15
 *
 * TIM5 is time (32bit)
 *
 * Taster links D1
 * Taster rechts D2
 *
 * ADC B0
 */

static bool motorOn = false;

static inline void stopMotor(void) {
        motorOn = false;
        setPinLow(GPIOA, GPIO15);
}

static inline void startMotor(void) {
        motorOn = true;
        setPinHigh(GPIOA, GPIO15);
}

static inline void driveLeft(void) {
        timer_set_oc_value(TIM4, TIM_OC1, PWM_PERIOD*SPEED);
        timer_set_oc_value(TIM4, TIM_OC2, PWM_PERIOD);
}
static inline void driveLeftFast(void) {
        timer_set_oc_value(TIM4, TIM_OC1, 0);
        timer_set_oc_value(TIM4, TIM_OC2, PWM_PERIOD);
}

static inline void breakMotor(void) {
        timer_set_oc_value(TIM4, TIM_OC1, PWM_PERIOD);
        timer_set_oc_value(TIM4, TIM_OC2, PWM_PERIOD);
}

static inline void driveRight(void) {
        timer_set_oc_value(TIM4, TIM_OC1, PWM_PERIOD);
        timer_set_oc_value(TIM4, TIM_OC2, PWM_PERIOD*SPEED);
}
static inline void driveRightFast(void) {
        timer_set_oc_value(TIM4, TIM_OC1, PWM_PERIOD);
        timer_set_oc_value(TIM4, TIM_OC2, 0);
}


void polling_thread(cyg_addrword_t arg)
{
	initRCC();

        // Position Waggon
        initRotaryEncoderTimer(TIM2, GPIOA, GPIO5, GPIO_AF1, GPIOA, GPIO1, GPIO_AF1);
        // Position Pendulum
        initRotaryEncoderTimer(TIM3, GPIOC, GPIO6, GPIO_AF2, GPIOC, GPIO7, GPIO_AF2);

        // IN1
        pwm_init(TIM4, 1, PWM_PERIOD, GPIOB, GPIO6, GPIO_AF2);
        // IN2
        pwm_init(TIM4, 2, PWM_PERIOD, GPIOB, GPIO7, GPIO_AF2);

	initTimer(TIM5);


        // EN
        initPinOutput(GPIOA, GPIO15);
        startMotor();

        // Taster links
        initPinInput(GPIOD, GPIO1, GPIO_PUPD_PULLUP);
        // Taster rechts
        initPinInput(GPIOD, GPIO2, GPIO_PUPD_PULLUP);

        // ADC
        // initADC(GPIOB, GPIO0);

        enum state currentState = DREHZAHL_MESSUNG;

        const int32_t UNDEFINED_VALUE = INT_MAX;
        int32_t leftWall = UNDEFINED_VALUE, rightWall = UNDEFINED_VALUE;
        int32_t leftBounding = UNDEFINED_VALUE, rightBounding = UNDEFINED_VALUE;
        // How near we need to be to the goal
        const int32_t GOAL_DIFF = 100;

        const int PRINT_INTERVAL = 1000;
        int printCounter = 0;

        // TODO assert leftWall < rightWall
        // TODO test if repeatedly setting driveRight is a problem
        // TODO kann man vllt. den blauen Knopf abfragen?
        

        int32_t lastPositionWaggon = UNDEFINED_VALUE;
        uint32_t lastPositionWaggonTime = UNDEFINED_VALUE;

        int32_t wackelStart = UNDEFINED_VALUE;
        int32_t wackelMaxTicks = TIMER_TICKS_PER_MICROSECOND * 1000 * 100;
        int32_t wackelMaxIncrement = TIMER_TICKS_PER_MICROSECOND * 1000 * 50;
        int32_t wackelMaxTicksMax = TIMER_TICKS_PER_MICROSECOND * 1000 * 400;
        int wackelAnzahl = 0;
        const int WACKEL_ANZAHL_MAX = 7;

        uint32_t impulsStart = UNDEFINED_VALUE;
        int impulsDirection = UNDEFINED_VALUE;
        const int IMPULS_LENGTH = TIMER_TICKS_PER_MICROSECOND * 1000 * 300;

        bool dontWackeln = false;

        ezs_printf("Startg\n");
        for (;;) {
                int32_t positionWaggon = (int32_t) timer_get_counter(TIM2);
                int16_t positionPendulum = (int16_t) timer_get_counter(TIM3);
                positionPendulum = ((positionPendulum % 2400) + 2400) % 2400;
                if (positionPendulum > 1200) positionPendulum -= 2400;
                uint32_t time = (uint32_t) timer_get_counter(TIM5);
                bool tasterLeft = !readPin(GPIOD, GPIO2);
                bool tasterRight = !readPin(GPIOD, GPIO1);

                if (positionWaggon != lastPositionWaggon) {
                        if (currentState == DREHZAHL_MESSUNG) {
                                ezs_printf("%d\n", time);
                        }
                        lastPositionWaggon = positionWaggon;
                        lastPositionWaggonTime = time;
                }

                bool moving;
                if (time - lastPositionWaggonTime > TIMER_TICKS_PER_MICROSECOND * 1000 * 250) {
                        moving = false;
                } else {
                        moving = true;
                }

                if (motorOn && !moving && currentState != DREHZAHL_MESSUNG && currentState != AUFSCHWINGEN_WARTEN && currentState != IDLE) {
                        stopMotor();
                        break;
                }

                switch (currentState) {
                        case IDLE:
                                breakMotor();
                                break;
                        case GOING_LEFT:
                                if (tasterLeft) {
                                        leftWall = positionWaggon;
                                        leftBounding = (rightWall - leftWall) * 0.1 + leftWall;
                                        rightBounding = rightWall - (rightWall - leftWall) * 0.05;
                                        currentState = GOING_RIGHT;
                                } else {
                                        driveLeft();
                                }
                                break;
                        case GOING_RIGHT:
                                if (tasterRight) {
                                        rightWall = positionWaggon;
                                        leftBounding = (rightWall - leftWall) * 0.1 + leftWall;
                                        rightBounding = rightWall - (rightWall - leftWall) * 0.05;
                                        currentState = GOING_GOAL;
                                } else {
                                        driveRight();
                                }
                                break;
                        case GOING_GOAL: {
                                int32_t goal = (leftWall + rightWall) / 2;
                                if (abs(positionWaggon-goal) <= GOAL_DIFF) {
                                        //breakMotor();
                                        if (dontWackeln) {
                                                currentState = AUFSCHWINGEN_WARTEN;
                                        } else {
                                                dontWackeln = true;
                                                wackelStart = time;
                                                wackelAnzahl = 0;
                                                wackelMaxTicks = TIMER_TICKS_PER_MICROSECOND * 1000 * 100;
                                                currentState = WACKEL_LINKS;
                                        }
                                } else if (positionWaggon < goal) {
                                        driveRight();
                                } else {
                                        driveLeft();
                                }
                                break;
                        }

                        case DREHZAHL_MESSUNG:
                                         driveRightFast();
                                         break;

                        case WACKEL_LINKS:
                        case WACKEL_RECHTS:
                        case AUFSCHWINGEN_IMPULS:
                        case AUFSCHWINGEN_WARTEN:
                                         if (positionWaggon >= rightBounding || positionWaggon <= leftBounding) {
                                                 currentState = GOING_GOAL;
                                                 break;
                                         }
                                         switch (currentState) {

                                                 case WACKEL_LINKS:
                                                         if (wackelAnzahl >= WACKEL_ANZAHL_MAX) {
                                                                 currentState = AUFSCHWINGEN_WARTEN;
                                                         } else if (time - wackelStart < wackelMaxTicks) {
                                                                 // Beschleunigen
                                                                 driveLeftFast();
                                                         } else {
                                                                 // Bremsen
                                                                 breakMotor();
                                                                 if (time - lastPositionWaggonTime > 5 * TIMER_TICKS_PER_MICROSECOND * 1000) {
                                                                         wackelStart = time;
                                                                         currentState = WACKEL_RECHTS;
                                                                         wackelAnzahl ++;
                                                                 }
                                                         }
                                                         break;
                                                 case WACKEL_RECHTS:
                                                         if (time - wackelStart < wackelMaxTicks) {
                                                                 // Beschleunigen
                                                                 driveRightFast();
                                                         } else {
                                                                 // Bremsen
                                                                 breakMotor();
                                                                 if (time - lastPositionWaggonTime > 5 * TIMER_TICKS_PER_MICROSECOND * 1000) {
                                                                         wackelMaxTicks += wackelMaxIncrement;
                                                                         if (wackelMaxTicks > wackelMaxTicksMax) {
                                                                                 wackelMaxTicks = wackelMaxTicksMax;
                                                                         }
                                                                         wackelStart = time;
                                                                         currentState = WACKEL_LINKS;
                                                                 }
                                                         }

                                                         break;
                                                 case AUFSCHWINGEN_WARTEN: 
                                                         breakMotor();
                                                         if (abs(positionPendulum) < 300) {
                                                                 impulsStart = time;
                                                                 impulsDirection = positionPendulum / abs(positionPendulum);
                                                                 currentState = AUFSCHWINGEN_IMPULS;
                                                         }
                                                         break;
                                                 case AUFSCHWINGEN_IMPULS:
                                                         if (time - impulsStart > IMPULS_LENGTH) {
                                                                 currentState = AUFSCHWINGEN_WARTEN;
                                                         } else {
                                                                 if (impulsDirection >= 0) {
                                                                         driveLeftFast();
                                                                 } else {
                                                                         driveRightFast();
                                                                 }
                                                         }
                                                         break;
                                                 default: break;
                                         }
                                         break;
                }

                if (currentState != DREHZAHL_MESSUNG && printCounter++ >= PRINT_INTERVAL) {
                        printCounter = 0;
                        int32_t goal = (leftWall + rightWall) / 2;
                        ezs_printf("%d %d %d %d %s %d %d %d %s %d %d\n", (int)positionWaggon,
                                        (int)positionPendulum,
                                        (int)leftWall, (int) rightWall,
                                        stateToString(currentState),
                                        tasterLeft,
                                        tasterRight,
                                        (int)goal,
                                        moving ? "MOVING" : "STOPPED",
                                        leftBounding,
                                        rightBounding
                                  );
                }
        }

        ezs_printf("EMERGENCY STOP\n");
}

void cyg_user_start(void)
{
        cyg_thread_create(PRIO, &polling_thread, 0, "Pin polling thread", polling_stack, STACKSIZE, &polling_handle, &polling_data);
        cyg_thread_resume(polling_handle);
}
