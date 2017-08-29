#include <stdlib.h>
#include <string.h>

#include "ezs_io.h"
#include "state_machine.h"
#include "lib.h"

#define MAX_MESSUNGEN 8000

int printf(const char*, ...);

static int32_t lastPositionWaggon;
static uint32_t messungenTime[MAX_MESSUNGEN];
static uint32_t messungenPosition[MAX_MESSUNGEN];
static int currentMessung;

enum state handle_motor_messung_start(struct PeripheralState *st) {
        memset(messungenTime, 0, sizeof(messungenTime));
        memset(messungenPosition, 0, sizeof(messungenPosition));
        currentMessung = 0;
        lastPositionWaggon = st->positionWaggon;
        ezs_printf("Start Messung\n");
        return MOTOR_MESSUNG_ACTION;
}

enum state handle_motor_messung_action(struct PeripheralState *st) {
        if (st->tasterLeft || st->tasterRight) {
                return IDLE;
        }
        driveLeftFast();
        if (currentMessung >= MAX_MESSUNGEN) {
                return MOTOR_MESSUNG_OUTPUT;
        } else if (lastPositionWaggon != st->positionWaggon) {
                int32_t difference = abs(st->positionWaggon - lastPositionWaggon);
                lastPositionWaggon = st->positionWaggon;
                messungenPosition[currentMessung] = difference;
                messungenTime[currentMessung++] = st->time;
                return MOTOR_MESSUNG_ACTION;
        }
        return MOTOR_MESSUNG_ACTION;

}

enum state handle_motor_messung_output(struct PeripheralState *st) {
        breakMotor();
        int i;
        ezs_printf("START OF OUTPUT\n");
        for (i = 0; i < currentMessung; i++) {
                ezs_printf("%u %d\n", (unsigned int)messungenTime[i], (int)messungenPosition[i]);
        }
        ezs_printf("END OF OUTPUT\n");
        return IDLE;
}
