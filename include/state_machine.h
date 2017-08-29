#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

#include <stdint.h>
#include <stdbool.h>

struct PeripheralState {
        int32_t positionWaggon;
        int16_t positionPendulum;
        uint32_t time;

        bool tasterLeft, tasterRight;

        // These values are defined by aufschwingen.c
        int32_t leftBound, rightBound;
};
enum state {
        // Defined in aufschwingen.c
        GOING_LEFT, GOING_RIGHT, GOING_GOAL,
        WACKEL_LINKS, WACKEL_RECHTS,
        AUFSCHWINGEN_WARTEN, AUFSCHWINGEN_IMPULS,
        // Defined in messung.c
        MOTOR_MESSUNG_START, MOTOR_MESSUNG_ACTION, MOTOR_MESSUNG_OUTPUT,
        // Defined in pid.c
        PID_INIT, PID_STEP,
        // Defined in state_machine.c
        IDLE};

void state_machine_init(void);
bool state_machine_step(void);

/*
 * State transition functions
 */
enum state handle_going_left(struct PeripheralState *peripheralState);
enum state handle_going_right(struct PeripheralState *peripheralState);
enum state handle_going_goal(struct PeripheralState *peripheralState);
enum state handle_wackel_links(struct PeripheralState *peripheralState);
enum state handle_wackel_rechts(struct PeripheralState *peripheralState);
enum state handle_aufschwingen_warten(struct PeripheralState *peripheralState);
enum state handle_aufschwingen_impuls(struct PeripheralState *peripheralState);

enum state handle_motor_messung_start(struct PeripheralState *peripheralState);
enum state handle_motor_messung_action(struct PeripheralState *peripheralState);
enum state handle_motor_messung_output(struct PeripheralState *peripheralState);

enum state handle_pid_init(struct PeripheralState *peripheralState);
enum state handle_pid_step(struct PeripheralState *peripheralState);
#endif
