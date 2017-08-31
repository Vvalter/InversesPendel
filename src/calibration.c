#include <stdlib.h>
#include <math.h>

#include "ezs_io.h"
#include "state_machine.h"
#include "lib.h"

#define INTERVALL TIMER_TICKS_PER_MICROSECOND * 1000 * 1000


int printf(const char*, ...);

static double currentSpeed;
static const double speedIncrement = 0.0001;
static uint32_t lastTime;
static int32_t lastPosition;

enum state handle_pwm_messung_init(struct PeripheralState *st) {
        currentSpeed = 0.067;
        lastTime = st->time;
        lastPosition = st->positionWaggon;
        return PWM_MESSUNG_ACTION;
}

enum state handle_pwm_messung_action(struct PeripheralState *st) {
        if (st->tasterLeft || st->tasterRight) {
                breakMotor();
                return IDLE;
        }
        if (abs(st->time - lastTime) >= INTERVALL) {
                if (abs(lastPosition - st->positionWaggon) > 5) {
                        ezs_printf("Done: %f\n", currentSpeed);
                        return IDLE;
                } 
                driveLeft(currentSpeed);
                lastTime = st->time;
                lastPosition = st->positionWaggon;
                currentSpeed += speedIncrement;
                ezs_printf("%f\n", currentSpeed);
        }
        return PWM_MESSUNG_ACTION;
}
