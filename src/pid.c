#include <stdlib.h>

#include "ezs_io.h"
#include "state_machine.h"
#include "lib.h"

#define REGEL_INTERVALL TIMER_TICKS_PER_MICROSECOND * 1000 * 10

int printf(const char*, ...);

static int32_t lastPositionWaggon;
static int16_t lastPositionPendulum;
static uint32_t lastTime;

enum state handle_pid_init(struct PeripheralState *st) {
        lastPositionWaggon = st->positionWaggon;
        lastPositionPendulum = st->positionPendulum;
        lastTime = st->time;
        return PID_STEP;
}

enum state handle_pid_step(struct PeripheralState *st) {
        if (st->tasterLeft || st->tasterRight) {
                return IDLE;
        }
        if (abs(st->time-lastTime) >= REGEL_INTERVALL) {
                uint32_t dt = abs(lastTime - st->time);
                int16_t da = lastPositionPendulum - st->positionPendulum;
                int16_t dp = lastPositionWaggon - st->positionWaggon;


                lastPositionWaggon = st->positionWaggon;
                lastPositionPendulum = st->positionPendulum;
                lastTime = st->time;
        }
        return PID_STEP;
}
