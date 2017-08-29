#include <stdlib.h>

#include "state_machine.h"
#include "lib.h"

static const float SPEED = 0.5f;
static int32_t leftWall, rightWall;

// How near we need to be to the goal
static const int32_t GOAL_DIFF = 100;
static int32_t goal = 0;

// How many ticks a Wackler should last
static int32_t wackelMaxTicks = TIMER_TICKS_PER_MICROSECOND * 1000 * 100;
// After each Wackler wackelMaxTicks is increased by wackelMaxIncrement
static const int32_t wackelMaxIncrement = TIMER_TICKS_PER_MICROSECOND * 1000 * 100;
// To a maximum value of wackelMaxTicksMax
static const int32_t wackelMaxTicksMax = TIMER_TICKS_PER_MICROSECOND * 1000 * 1000;
// Before changing direction we wait for this number of ticks
static const int32_t BREAK_TIME_TICKS = TIMER_TICKS_PER_MICROSECOND * 1000 * 5;
// The time the last Wackler started
static int32_t wackelStart = 0;
// How many Wackler are done and how many are left
static int wackelAnzahl = 0;
static const int WACKEL_ANZAHL_MAX = 6;

// When did the current impuls start
static uint32_t impulsStart = 0;
// In which direction is the impuls going
static int impulsDirection = 0;
// How long is an impuls
static const int IMPULS_LENGTH = TIMER_TICKS_PER_MICROSECOND * 1000 * 300;

enum state handle_going_left(struct PeripheralState *st){
        if (st->tasterLeft) {
                leftWall = st->positionWaggon;
                return GOING_RIGHT;
        } else {
                driveLeft(SPEED);
                return GOING_LEFT;
        }
}
enum state handle_going_right(struct PeripheralState *st){
        if (st->tasterRight) {
                rightWall = st->positionWaggon;
                goal = (leftWall + rightWall) / 2;
                return GOING_GOAL;
        } else {
                driveRight(SPEED);
                return GOING_RIGHT;
        }
}
enum state handle_going_goal(struct PeripheralState *st){
        if (abs(st->positionWaggon-goal) <= GOAL_DIFF) {
                int32_t width = rightWall - leftWall;
                st->leftBound= width * 0.1 + leftWall;
                st->rightBound= rightWall - width * 0.1;

                wackelStart = st->time;
                //wackelAnzahl = 0;
                return WACKEL_LINKS;
        } else if (st->positionWaggon < goal) {
                driveRight(SPEED);
                return GOING_GOAL;
        } else {
                driveLeft(SPEED);
                return GOING_GOAL;
        }
}
enum state handle_wackel_links(struct PeripheralState *st){
        if (wackelAnzahl >= WACKEL_ANZAHL_MAX) {
                return AUFSCHWINGEN_WARTEN;
        }
        if (st->time - wackelStart < wackelMaxTicks) {
                driveLeftFast();
                return WACKEL_LINKS;
        } else if (st->time - wackelStart < wackelMaxTicks + BREAK_TIME_TICKS) {
                breakMotor();
                return WACKEL_LINKS;
        } else {
                wackelStart = st->time;
                return WACKEL_RECHTS;
        }
}
enum state handle_wackel_rechts(struct PeripheralState *st){
        if (wackelAnzahl >= WACKEL_ANZAHL_MAX) {
                return AUFSCHWINGEN_WARTEN;
        }
        if (st->time - wackelStart < wackelMaxTicks) {
                driveRightFast();
                return WACKEL_RECHTS;
        } else if (st->time - wackelStart < wackelMaxTicks + BREAK_TIME_TICKS) {
                breakMotor();
                return WACKEL_RECHTS;
        } else {
                wackelStart = st->time;
                wackelAnzahl ++;
                wackelMaxTicks += wackelMaxIncrement;
                if (wackelMaxTicks > wackelMaxTicksMax) {
                        wackelMaxTicks = wackelMaxTicksMax;
                }
                return WACKEL_LINKS;
        }
}
enum state handle_aufschwingen_warten(struct PeripheralState *st){
        breakMotor();

        if (abs(st->positionPendulum) < 300) {
                impulsStart = st->time;
                impulsDirection = st->positionPendulum / abs(st->positionPendulum);
                return AUFSCHWINGEN_IMPULS;
        } else {
                return AUFSCHWINGEN_WARTEN;
        }
}
enum state handle_aufschwingen_impuls(struct PeripheralState *st){
        if (st->time - impulsStart > IMPULS_LENGTH) {
                return AUFSCHWINGEN_WARTEN;
        } else {
                if (impulsDirection >= 0) {
                        driveLeftFast();
                } else {
                        driveRightFast();
                }
                return AUFSCHWINGEN_IMPULS;
        }
}
